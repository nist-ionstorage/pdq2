#!/usr/bin/python
# -*- coding: utf8 -*-
#
# Robert Jordens <jordens@gmail.com>, 2012
#

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
    num_boards = 3
    num_dacs = 3
    max_data = 8*(1<<10) # 6144 data buffer size per channel
    escape_char = b"\xaa"

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

    @staticmethod
    def interpolate(times, voltages, order):
        """
        calculate b-spline interpolation derivatives for voltage data
        according to interpolation mode
        returns times, voltages and derivatives suitable for passing to
        frame()
        also removes the first times point (implicitly shifts to 0) and
        the last voltages/derivatives (irrelevant since the section ends
        here)
        """
        derivatives = [voltages[:-1]]
        if order > 1:
            spline = interpolate.splrep(times, voltages, k=order-1)
            derivatives += [interpolate.splev(times[:-1], spline, der=i)
                    for i in range(1, order)]
        return np.diff(times), derivatives

    def frame(self, times, derivatives, order=4, aux=None,
            time_shift=0, trigger_first=False, wait_last=True):
        """
        serialize frame data
        voltages in volts, times in seconds,
        derivatives can be a number in which case the derivatives will
        be the interpolation, else it should be a list of derivatives
        """
        # FIXME implement DDS mode
        if not isinstance(derivatives, list):
            derivatives = np.clip(derivatives, -self.max_out, self.max_out)
            times, derivatives = self.interpolate(times, derivatives, order)
        line_len = {0: 0, 1: 1, 2: 3, 3: 6, 4: 9}[len(derivatives)]
        length = len(times)

        frame = []

        head = np.zeros((length,), "<u2")
        head[:] |= line_len<<0 # 4
        #head[:] |= 0<<4 # typ # 2
        head[-1] |= wait_last<<6 # 1
        head[0] |= trigger_first<<7 # 1
        head[:] |= time_shift<<8 # 4
        if aux is not None:
            head[:] |= aux[:length]<<12 # 1
        #head[:] |= reserved<<13 # 3
        frame.append(head)

        dt = np.rint(times*(self.freq/(1<<time_shift))) - 1
        # FIXME: clipped intervals cumulatively skew
        # subsequent ones
        np.clip(dt, self.min_time, self.max_time, out=dt)
        frame.append(dt.astype("<u2"))

        byts = [2, 4, 6, 6]
        for i, (v, byt) in enumerate(zip(derivatives, byts)):
            scale = self.scale*(1 << (8*(byt - 2)))/self.freq**i
            v = np.rint(scale*v).astype("<i8")
            # np.clip(v, -self.max_val, self.max_val, out=v)
            if byt == 6: # no i6 type
                frame.append(v.astype("<i4"))
                frame.append((v >> 32).astype("<i%i" % (byt - 4)))
            else:
                frame.append(v.astype("<i%i" % byt))

        frame = np.rec.fromarrays(frame) # interleave
        data_length = len(bytes(frame.data))
        logger.debug("frame %s dtype %s shape %s length %s",
                frame, frame.dtype, frame.shape, data_length)
        bpl = 2*(2 + line_len)
        assert data_length == bpl*length, (data_length, bpl, length,
                frame.shape)
        return frame

    @staticmethod
    def add_frame_header(frame, repeat, next):
        head = struct.pack("<BBH", next, repeat, frame.shape[0])
        logger.debug("frame header %r", head)
        return head + bytes(frame.data)

    def combine_frames(self, frames):
        lens = [len(frame)//2 for frame in frames[:-1]] # 16 bits
        mems = np.cumsum([len(frames)] + lens)
        chunk = bytes(mems.astype("<u2").data) # jump table
        logger.debug("mem len %i, %r", len(chunk), chunk)
        for frame in frames:
            chunk += frame
        assert len(chunk) <= self.max_data
        return chunk

    def add_mem_header(self, board, dac, adr, chunk):
        length = len(chunk)//2 # 16 bit memory
        assert dac in range(self.num_dacs)
        assert board in range(self.num_boards)
        head = struct.pack("<BBHH", board, dac, adr, length)
        logger.debug("mem header %r", head)
        return head + chunk

    def multi_frame(self, times_voltages, channel, repeat=0,
            next=None, **kwargs):
        frames = []
        for i, (t, v) in enumerate(times_voltages):
            n = i if next is None else next
            if t is None:
                frame = b""
            else:
                frame = self.frame(t, v, **kwargs)
            frames.append(self.add_frame_header(frame, repeat, n))
        data = self.combine_frames(frames)
        board, dac = divmod(channel, self.num_dacs)
        data = self.add_mem_header(board, dac, 0, data)
        return data


def main():
    import argparse
    parser = argparse.ArgumentParser(description="""PDQ DAC frontend.
            Evaluates times and voltages, interpolates them and uploads
            them.""")
    parser.add_argument("-s", "--serial", default=None,
            help="device (FT245R) serial string [first]")
    parser.add_argument("-c", "--channel", default=None, type=int,
            help="channel: 3*board_num+dac_num [%(default)s]")
    parser.add_argument("-i", "--frame", default=None, type=int,
            help="frame number [%(default)s]")
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
    data = []
    channels = (args.channel is None) and range(9) or [args.channel]
    for channel in channels:
        if args.frame is None:
            v = [.1*frame + channel + voltages for frame in range(8)]
            t = [times] * len(v)
            data.append(dev.multi_frame(zip(t, v), channel=channel,
                    order=args.mode))
        else:
            tv = [(None, None)] * 8
            tv[args.frame] = times, voltages
            data.append(dev.multi_frame(tv, channel=channel,
                    order=args.mode))
    dev.write_cmd("RESET_EN")
    dev.write_cmd("ARM_DIS")
    dev.write_cmd("TRIGGER_DIS")
    dev.write_data(*data)
    if not args.disarm:
        dev.write_cmd("ARM_EN")
    if args.free:
        dev.write_cmd("TRIGGER_EN")


if __name__ == "__main__":
    main()
