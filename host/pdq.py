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
    scale = max_val/max_out # LSB/V
    freq = 100e6 # samples/s
    min_time = 12 # processing time for a order=4 point
    max_time = 1<<16 # unsigned 16 bit timer
    num_dacs = 3
    num_frames = 8
    max_data = 4*(1<<10) # 8kx16 8kx16 4kx16
    escape_char = b"\xa5"

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

    def interpolate(self, times, voltages, order, shift=0):
        """
        calculate spline interpolation derivatives for data
        according to interpolation order
        also differentiates times (implicitly shifts to 0) and removes
        the last voltage (irrelevant since the frame ends here)
        """
        times = times*(self.freq/(1<<shift))
        spline = interpolate.splrep(times, voltages, k=order)
        times = np.rint(times)
        derivatives = [interpolate.splev(times[:-1], spline, der=i)
                for i in range(order + 1)]
        times = np.diff(times)
        # FIXME: clipped intervals cumulatively skew subsequent ones
        #np.clip(times, self.min_time, self.max_time, out=times)
        assert np.all(times > 0), times
        return times, derivatives

    def frame(self, times, voltages, order=3, aux=None,
            time_shift=0, wait=True, end=True, silence=False):
        """
        serialize frame data
        voltages in volts, times in seconds,
        """
        voltages = np.clip(voltages, -self.max_val, self.max_val)*self.scale
        times, derivatives = self.interpolate(times, voltages,
                order, time_shift)
        words = [1, 2, 3, 3]
        line_len = sum(words[:len(derivatives)]) + 1 # dt
        length = len(times)

        frame = []

        head = np.zeros(length, "<u2")
        head[:] |= line_len<<0 # 4
        head[:] |= 0<<4 # typ # 2
        head[0] |= wait<<6 # 1
        head[-1] |= silence<<7 # 1
        if aux is not None:
            head[:] |= aux[:length]<<8 # 1
        head[:] |= time_shift<<9 # 4
        head[-1] |= end<<13 # 1
        head[:] |= 0<<14 # reserved 2
        frame.append(head)

        frame.append(times.astype("<u2"))

        for i, (v, word) in enumerate(zip(derivatives, words)):
            scale = 1 << (16*(word - 1))
            v = np.rint(scale*v).astype("<i8")
            if word == 3: # no i6 dtype
                frame.append(v.astype("<i4"))
                frame.append((v >> 32).astype("<i%i" % (word*2 - 4)))
            else:
                frame.append(v.astype("<i%i" % (word*2)))

        frame = np.rec.fromarrays(frame) # interleave
        data_length = len(bytes(frame.data))
        logger.debug("frame %s dtype %s shape %s length %s",
                frame, frame.dtype, frame.shape, data_length)
        assert data_length == 2*(1 + line_len)*length, (data_length, length,
                line_len, frame.shape)
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
        frames = [bytes(self.frame(t, v, **kwargs).data)
            for t, v in times_voltages]
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
