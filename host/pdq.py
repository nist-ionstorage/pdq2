#!/usr/bin/python
# -*- coding: utf8 -*-
#
# Robert Jordens <jordens@gmail.com>, 2012

import logging, struct
import numpy as np
from scipy import interpolate
import warnings

logger = logging.getLogger("Pdq")


class PyFtdi(object):
    def __init__(self, serial=None):
        self.dev = pylibftdi.Device(device_id=serial)

    def write(self, data):
        written = self.dev.write(data)
        if written < 0:
            raise pylibftdi.FtdiError(written,
                    self.dev.get_error_string())
        return written

    def close(self):
        self.dev.close()
        del self.dev


class D2xxFtdi(object):
    def __init__(self, serial=None):
        if serial is not None:
            self.dev = ftd2xx.openEx(serial)
        else:
            self.dev = ftd2xx.open()
        self.dev.setTimeouts(read=5000, write=5000)

    def write(self, data):
        written = self.dev.write(str(data))
        return written

    def close(self):
        self.dev.close()
        del self.dev


class FileFtdi(object):
    def __init__(self, serial="unknown"):
        self.fil = open("pdq_%s_ftdi.bin" % serial, "wb")

    def write(self, data):
        written = self.fil.write(data)
        return len(data)

    def close(self):
        self.fil.close()
        del self.fil


Ftdi = None

try:
    import pylibftdi
    Ftdi = PyFtdi
except ImportError:
    pass

try:
    import ftd2xx
    Ftdi = D2xxFtdi
except ImportError:
    pass

if Ftdi is None:
    Ftdi = FileFtdi
    warnings.warn("no ftdi library found. writing to files")


class Pdq(object):
    """
    PDQ DAC (a.k.a. QC_Waveform)
    """
    max_val = 1<<15 # signed 16 bit DAC
    max_out = 10.
    scale = 1/max_out # LSB/V
    freq = 100e6 # samples/s
    min_time = 12 # processing time for a order=4 point
    max_time = 1<<16 # unsigned 16 bit timer
    num_dacs = 3
    num_frames = 8
    max_data = 4*(1<<10) # 8kx16 8kx16 4kx16
    escape_char = b"\xa5"
    cordic_gain = 1.
    for i in range(16):
        cordic_gain *= np.sqrt(1 + 2**(-2*i))

    commands = {
            "RESET_EN":    b"\x00",
            "RESET_DIS":   b"\x01",
            "TRIGGER_EN":  b"\x02",
            "TRIGGER_DIS": b"\x03",
            "ARM_EN":      b"\x04",
            "ARM_DIS":     b"\x05",
            }

    def __init__(self, serial=None):
        self.dev = Ftdi(serial)

    def close(self):
        self.dev.close()
        del self.dev

    def cmd(self, cmd):
        return self.escape_char + self.commands[cmd]

    def write_cmd(self, cmd):
        return self.write(self.cmd(cmd))

    def escape(self, data):
        return data.replace(self.escape_char, self.escape_char +
                self.escape_char)

    def write(self, *segments):
        """
        writes data segments to device
        """
        for segment in segments:
            written = self.dev.write(segment)
            if written < len(segment):
                logger.error("wrote only %i of %i", written, len(segment))

    def write_data(self, *segments):
        return self.write(*(self.escape(seg) for seg in segments))

    def line_times(self, t, shift=0):
        scale = self.freq/2**shift
        t = t*scale
        tr = np.rint(t)
        dt = np.diff(tr)
        # FIXME: clipped intervals cumulatively skew subsequent ones
        #np.clip(times, self.min_time, self.max_time, out=times)
        assert np.all(dt >= 12), dt
        return t, tr, dt

    def interpolate(self, t, v, order, shift=0, tr=None):
        """
        calculate spline interpolation derivatives for data
        according to interpolation order
        also differentiates times (implicitly shifts to 0) and removes
        the last value (irrelevant since the frame ends here)
        """
        if order == 0:
            return [v[:-1]]
        spline = interpolate.splrep(t, v, k=order)
        if tr is None:
            tr = t
        dv = [interpolate.splev(tr[:-1], spline, der=i)
                for i in range(order + 1)]
        # correct for adder chain latency
        correction_map = [
                (1, -1/2., 2),
                (1,  1/3., 3),
                (2,   -1., 3),
                ]
        for i, c, j in correction_map:
            if j >= len(dv):
                break
            dv[i] -= c*dv[j]
        return dv

    def pack_frame(self, *parts_dtypes):
        frame = []
        for part, dtype in parts_dtypes:
            if dtype == "i6":
                part = part.astype("<i8")
                frame.append(part.astype("<i4"))
                frame.append((part >> 32).astype("<i2"))
            else:
                frame.append(part.astype("<" + dtype))
        frame = np.rec.fromarrays(frame) # interleave
        logger.debug("frame %s dtype %s shape %s length %s",
                frame, frame.dtype, frame.shape, len(bytes(frame.data)))
        return bytes(frame.data)

    def frame(self, t, v, order=3, aux=None, shift=0, wait=True, end=True,
            silence=False, stop=True):
        """
        serialize frame data
        voltages in volts, times in seconds
        """
        words = [1, 2, 3, 3]
        parts = []

        head = np.zeros(len(t) - 1, "<u2")
        head[:] |= sum(words[:order + 1]) + 1 # 4b
        #head[:] |= 0<<4 # typ # 2
        head[0] |= wait<<6 # 1
        head[-1] |= silence<<7 # 1
        if aux is not None:
            head[:] |= aux[:len(head)]<<8 # 1
        head[:] |= shift<<9 # 4
        head[-1] |= (not stop and end)<<13 # 1
        #head[0] |= 0<<14 # clear # 1
        #head[:] |= 0<<15 # reserved 2
        parts.append((head, "u2"))

        t, tr, dt = self.line_times(t, shift)
        parts.append((dt, "u2"))

        v = np.clip(v/self.max_out, -1, 1)
        for dv, w in zip(self.interpolate(t, v, order, shift, tr), words):
            parts.append((np.rint(dv*(2**(16*w - 1))), "i%i" % (2*w)))
        
        frame = self.pack_frame(*parts)

        if stop:
            frame += struct.pack("<HHH", (2<<0) | (silence<<7) | (end<<13),
                    1, int(v[-1]*2**15))
        return frame

    def cordic_frame(self, t, v, p, f, aux=None, shift=0, wait=True,
            end=True, silence=True, stop=True):
        words = [1, 2, 3, 3, 1, 2, 2]
        parts = []

        head = np.zeros(len(t) - 1, "<u2")
        head[:] |= sum(words) + 1 # 4b
        head[:] |= 1<<4 # typ # 2
        head[0] |= wait<<6 # 1
        head[-1] |= silence<<7 # 1
        if aux is not None:
            head[:] |= aux[:len(head)]<<8 # 1
        head[:] |= shift<<9 # 4
        head[-1] |= (not stop and end)<<13 # 1
        head[0] |= 1<<14 # clear # 1
        #head[:] |= 0<<15 # reserved # 1
        parts.append((head, "u2"))

        t, tr, dt = self.line_times(t, shift)
        parts.append((dt, "u2"))

        v = np.clip(v/self.max_out, -1, 1)/self.cordic_gain
        for dv, w in zip(self.interpolate(t, v, 3, shift, tr), words):
            parts.append((np.rint(dv*(2**(16*w - 1))), "i%i" % (2*w)))

        p = p/(2*np.pi)
        for dv, w in zip(self.interpolate(t, p, 0, 0, tr), [1]):
            parts.append((np.rint(dv*(2**(16*w))), "i%i" % (2*w)))

        f = f/self.freq
        for dv, w in zip(self.interpolate(t, f, 1, shift, tr), [2, 2]):
            parts.append((np.rint(dv*(2**(16*w - 1))), "i%i" % (2*w)))

        frame = self.pack_frame(*parts)

        if stop:
            frame += struct.pack("<HH HIIHIH HII", (2<<0) | (1<<4) |
                    (silence<<7) | (end<<13),
                    1, int(v[-1]*2**15), 0, 0, 0, 0, 0,
                    int(p[-1]*2**16), int(f[-1]*2**31), 0)
        return frame

    def map_frames(self, frames, map=None):
        table = []
        adr = self.num_frames
        for frame in frames:
            table.append(adr)
            adr += len(frame)//2
        assert adr <= self.max_data, adr
        t = []
        for i in range(self.num_frames):
            if map is not None and len(map) > i:
                i = map[i]
            if i is not None and len(table) > i:
                i = table[i]
            else:
                i = 0
            t.append(i)
        t = struct.pack("<" + "H"*self.num_frames, *t)
        return t + b"".join(frames)

    def add_mem_header(self, board, dac, data, adr=0):
        assert dac in range(self.num_dacs)
        head = struct.pack("<HHH", (board << 4) | dac,
                adr, adr + len(data)//2 - 1)
        return head + data

    def multi_frame(self, times_voltages, channel, map=None, **kwargs):
        frames = [self.frame(t, v, **kwargs) for t, v in times_voltages]
        data = self.map_frames(frames, map)
        board, dac = divmod(channel, self.num_dacs)
        data = self.add_mem_header(board, dac, data)
        return data


def main():
    import argparse
    import time

    parser = argparse.ArgumentParser(description="""PDQ DAC frontend.
            Evaluates times and voltages, interpolates and uploads
            them.""")
    parser.add_argument("-s", "--serial", default=None,
            help="device (FT245R) serial string [first]")
    parser.add_argument("-c", "--channel", default=None, type=int,
            help="channel: 3*board_num+dac_num [%(default)s]")
    parser.add_argument("-f", "--free", default=False,
            action="store_true",
            help="software trigger [%(default)s]")
    parser.add_argument("-n", "--disarm", default=False,
            action="store_true",
            help="disarm group [%(default)s]")
    parser.add_argument("-t", "--times",
            default="np.linspace(0, 1e-3, 11)",
            help="sample times (s) [%(default)s]")
    parser.add_argument("-v", "--voltages",
            default="(1-np.cos(t/t[-1]*np.pi))/2",
            help="sample voltages (V) [%(default)s]")
    parser.add_argument("-o", "--order", default=3, type=int,
            help="interpolation (0: const, 1: lin, 2: quad, 3: cubic)"
                 " [%(default)s]")
    parser.add_argument("-p", "--plot", help="plot to file [%(default)s]")
    parser.add_argument("-d", "--debug", default=False,
            action="store_true", help="debug communications")
    parser.add_argument("-r", "--reset", default=False,
            action="store_true", help="do reset before")

    args = parser.parse_args()

    if args.debug:
        logging.basicConfig(level=logging.DEBUG)
    else:
        logging.basicConfig(level=logging.WARNING)

    times = eval(args.times, globals(), {})
    voltages = eval(args.voltages, globals(), dict(t=times))

    if args.plot:
        from matplotlib import pyplot as plt
        fig, ax0 = plt.subplots()
        ax0.plot(times, voltages, "xk", label="points")
        if args.order:
            spline = interpolate.splrep(times, voltages, k=args.order)
            ttimes = np.arange(0, times[-1], 1/Pdq.freq)
            vvoltages = interpolate.splev(ttimes, spline)
            ax0.plot(ttimes, vvoltages, ",b", label="interpolation")
        fig.savefig(args.plot)

    dev = Pdq(serial=args.serial)
    if args.reset:
        dev.write(b"\x00") # flush any escape
        dev.write_cmd("RESET_EN")
        time.sleep(.1)
    else:
        dev.write_cmd("ARM_DIS")
    if args.channel is None:
        for channel in range(9):
            tv = [(times, .1*frame + channel + voltages)
                    for frame in range(dev.num_frames)]
            dev.write_data(dev.multi_frame(tv, channel=channel,
                                           order=args.order))
    else:
        tv = [(times, voltages)]
        map = [0] * dev.num_frames
        dev.write_data(dev.multi_frame(tv, channel=args.channel,
                                       order=args.order, map=map))
    if not args.disarm:
        dev.write_cmd("ARM_EN")
    if args.free:
        dev.write_cmd("TRIGGER_EN")


if __name__ == "__main__":
    main()
