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
    min_time = 11 # processing time for a mode=4 point
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
        calculate b-spline interpolation derivatives for voltage data
        according to interpolation mode
        returns times, voltages and derivatives suitable for passing to
        frame()
        also removes the first times point (implicitly shifts to 0) and
        the last voltages/derivatives (irrelevant since the section ends
        here)
        """
        times = times*(self.freq/(1<<shift))
        voltages = np.clip(voltages, -self.max_val, self.max_val)*self.scale
        spline = interpolate.splrep(times, voltages, k=order-1)
        times = np.rint(times)
        derivatives = [interpolate.splev(times[:-1], spline, der=i)
                for i in range(order)]
        # FIXME: clipped intervals cumulatively skew
        # subsequent ones
        times = np.clip(np.diff(times), self.min_time, self.max_time)
        assert np.all(times > 0)
        return times, derivatives

    def frame(self, times, derivatives, order=4, aux=None,
            time_shift=0, wait=True, end=True):
        """
        serialize frame data
        voltages in volts, times in seconds,
        derivatives can be a number in which case the derivatives will
        be the interpolation, else it should be a list of derivatives
        """
        if not isinstance(derivatives, list):
            derivatives = np.clip(derivatives, -self.max_out, self.max_out)
            times, derivatives = self.interpolate(times, derivatives,
                    order, time_shift)
        words = [1, 2, 3, 3]
        line_len = sum(words[:len(derivatives)]) + 1 # dt
        length = len(times)

        frame = []

        head = np.zeros(length, "<u2")
        head[:] |= line_len<<0 # 4
        head[:] |= 0<<4 # typ # 2
        head[0] |= wait<<6 # 1
        head[:] |= 0<<7 # silence # 1
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

    def map_frames(self, frames, frame_map=None):
        table = []
        adr = self.num_frames
        for frame in frames:
            table.append(adr)
            adr += len(frame)//2
        assert adr <= self.max_data, adr
        t = []
        for i in range(self.num_frames):
            if i >= len(table):
                t.append(0)
            elif frame_map is None:
                t.append(table[i])
            elif i >= len(frame_map):
                t.append(0)
            else:
                t.append(table[frame_map[i]])
        t = struct.pack("<" + "H"*self.num_frames, *t)
        return t + b"".join(frames)

    def add_mem_header(self, board, dac, chunk, adr=0):
        assert dac in range(self.num_dacs)
        head = struct.pack("<BBHH", (board << 4) | dac, 0,
                adr, adr + len(chunk)//2 - 1)
        return head + chunk

    def multi_frame(self, times_voltages, channel, frame_map=None, **kwargs):
        frames = []
        for i, (t, v) in enumerate(times_voltages):
            frames.append(bytes(self.frame(t, v, **kwargs).data))
        data = self.map_frames(frames, frame_map)
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
            help="software trigger group, free running [%(default)s]")
    parser.add_argument("-n", "--disarm", default=False,
            action="store_true",
            help="disarm group [%(default)s]")
    parser.add_argument("-t", "--times",
            default="np.linspace(0, 1e-3, 11)",
            help="sample times (s) [%(default)s]")
    parser.add_argument("-v", "--voltages",
            default="(1-np.cos(t/t[-1]*np.pi))/2",
            help="sample voltages (V) [%(default)s]")
    parser.add_argument("-m", "--mode", default=4, type=int,
            help="interpolation (0: only aux, 1: const, 2: lin, 3: quad, 4: cubic)"
                 " [%(default)s]")
    parser.add_argument("-p", "--plot",
            help="plot to file [%(default)s]")
    parser.add_argument("-d", "--debug", default=False,
            action="store_true",
            help="debug communications")
    parser.add_argument("-r", "--reset", default=False,
            action="store_true",
            help="do reset")

    args = parser.parse_args()

    if args.debug:
        logging.basicConfig(level=logging.DEBUG)
    else:
        logging.basicConfig(level=logging.WARNING)

    times = eval(args.times, globals(), {})
    voltages = eval(args.voltages, globals(), dict(t=times))

    if args.plot:
        from matplotlib import pyplot as plt
        times -= times[0]
        fig, ax0 = plt.subplots()
        ax0.plot(times, voltages, "xk", label="points")
        if args.mode > 1:
            spline = interpolate.splrep(times, voltages,
                    s=0, k=args.mode-1)
            ttimes = np.arange(0, times[-1], 1/Pdq.freq)
            vvoltages = interpolate.splev(ttimes, spline, der=0)
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
                                           order=args.mode))
    else:
        tv = [(times, voltages)]
        frames = [0] * dev.num_frames
        dev.write_data(dev.multi_frame(tv, channel=args.channel,
                                       order=args.mode, frame_map=frames))
    if not args.disarm:
        dev.write_cmd("ARM_EN")
    if args.free:
        dev.write_cmd("TRIGGER_EN")


if __name__ == "__main__":
    main()
