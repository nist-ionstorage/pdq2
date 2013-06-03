#!/usr/bin/python
# -*- coding: utf8 -*-
#
# Robert Jordens <jordens@gmail.com>, 2012
#

import logging, struct
import numpy as np
from scipy import interpolate


class PyFtdi(object):
    def __init__(self, serial=None):
        self.dev = pylibftdi.Device(device_id=serial)

    def write(self, data):
        written = self.dev.write(data)
        if written < 0:
            raise pylibftdi.FtdiError(written,
                    self.dev.get_error_string())
        return written


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


class FileFtdi(object):
    def __init__(self, serial="unknown"):
        self.fil = open("pdq_%s_ftdi.bin" % serial, "wb")
    def write(self, data):
        written = self.fil.write(data)
        return len(data)


try:
    import pylibftdi
    Ftdi = PyFtdi
    raise ImportError
except ImportError:
    try:
        import ftd2xx
        Ftdi = D2xxFtdi
    except ImportError:
        Ftdi = FileFtdi


class Pdq(object):
    """
    PDQ DAC (a.k.a. QC_Waveform)
    """
    range = (1<<15)-1 # signed 16 bit DAC
    scale = range/10. # LSB/V
    freq = 50e6 # samples/s
    min_time = 12 # processing time for a mode=3 point
    max_time = range # signed 16 bit timer
    num_branches = 8
    num_fpgas = 3
    num_dacs = 3
    max_data = 6144 # data buffer size per channel

    def __init__(self, serial=None):
        self.dev = Ftdi(serial)

    def serialize_header(self, channel, mode, trigger, branch_lens):
        """
        serialize the channel data header into a string
        """
        assert channel in range(self.num_fpgas*self.num_dacs)
        assert mode in range(4)
        assert len(branch_lens) == self.num_branches
        fpga, dac = divmod(channel, self.num_dacs)
        branch_ends = list(np.cumsum(branch_lens))
        # branch 7 always takes all data (0:end not
        # branch_end[-2]:branch_end[-1])
        data_len = int(branch_ends.pop())
        assert data_len <= self.max_data

        head = struct.pack("<B B B", 0x00, fpga, 0x10|dac)
        head += struct.pack("<B B B", 0x08, mode, bool(trigger))
        head += struct.pack("<B H", 0x02, 0x0000) # start addr
        head += struct.pack("<B H", 0x06, data_len) # reg_len
        for i, branch_end in enumerate(branch_ends):
            head += struct.pack("<B H", 0x20|i, int(branch_end))
        head += struct.pack("<B H", 0x01, data_len) # burst_len
        head += struct.pack("<B", 0x04) # burst data follows
        # head += struct.pack("<B", 0x03) # single data follows

        logging.debug("header len %i", len(head))
        logging.debug("header %r", head)

        return head

    def serialize_branch(self, voltages, times=None, *derivatives):
        """
        serialize channel branch data into buffer
        voltages in volts, times in seconds,
        derivatives according to chosen interpolation mode
        """
        branch = []

        scale = self.scale
        voltages = voltages*scale
        # np.clip(voltages, -self.range, self.range, out=voltages)
        branch.append(voltages.astype("i2"))

        if times is not None:
            dtimes = np.diff(np.r_[0, np.fabs(times)])*self.freq
            # FIXME: min_time=100 for mode 1,2?
            # FIXME: clipped intervals cumulatively skew
            # subsequent ones
            np.clip(dtimes, self.min_time, self.max_time, out=dtimes)
            # add the sign back in
            branch.append((np.sign(times)*dtimes).astype("i2"))

        if len(derivatives) >= 1:
            scale *= (1<<16)/self.freq
            deriv1 = (scale*derivatives[0]).astype("i4")
            branch.append(deriv1)

        if len(derivatives) >= 2:
            scale *= (1<<16)/self.freq
            # no i6 dtype
            deriv2 = (scale*derivatives[1]).astype("i8")
            branch.append(deriv2.astype("i4"))
            branch.append((deriv2>>32).astype("i2"))

        if len(derivatives) >= 3:
            scale *= 1/self.freq
            deriv3 = (scale*derivatives[2]).astype("i8")
            branch.append(deriv3.astype("i4"))
            branch.append((deriv3>>32).astype("i2"))

        branch = np.rec.fromarrays(branch, byteorder="<") # interleave

        if len(derivatives) >= 3: # mode == 3:
            assert len(branch.data)/2 == (2+2+4+6+6)/2*len(voltages), (
                  len(voltages), len(branch.data))
        logging.debug("branch shape %s len %i", branch.shape,
                len(branch.data))
        logging.debug("branch dtype %s", branch.dtype)
        logging.debug("branch %s", branch)
        # logging.debug("branch raw %s", str(branch.data))

        return branch.data # memory buffer
    
    def write(self, *segments):
        """
        writes segments to device
        segments is a flat sequence of channel header and branch data,
        pseudo-regex: (header branch*)*
        """
        for segment in segments:
            written = self.dev.write(segment)
            if written < len(segment):
                logging.error("wrote only %i of %i", written, len(segment))

    @staticmethod
    def interpolate(times, voltages, mode):
        """
        calculate b-spline interpolation derivatives for voltage data
        according to interpolation mode
        returns times, voltages and derivatives suitable for passing to
        serialize_branch()
        also removes the first times point (implicitly shifts to 0) and
        the last voltages/derivatives (irrelevant since the section ends
        here) and sets the pause/trigger marker on the last point
        as desired by the fpga
        """
        derivatives = ()
        if mode in (2, 3):
            spline = interpolate.splrep(times, voltages, s=0, k=mode)
            derivatives = [interpolate.splev(times[:-1], spline, der=i+1)
                    for i in range(mode)]
        voltages = voltages[:-1]
        if mode == 0:
            times = None
        else:
            times = times[1:]-times[0]
            times[-1] *= -1 # end/trigger marker
        return times, voltages, derivatives

    def prepare_simple(self, times, voltages, channel, mode=3,
            trigger=True):
        """
        interpolate, serialize and upload multi-branch, single-section
        data for one channel
        """
        assert len(voltages) == 8
        data = []
        branch_lens = []
        for v in voltages:
            if v is not None:
                t, v, der = self.interpolate(times, v, mode)
                logging.debug("data %s %s %s", t, v, der)
                d = self.serialize_branch(v, t, *der)
                data.append(d)
                branch_lens.append(len(d)/2) # 16 bits
            else:
                branch_lens.append(0)
        header = self.serialize_header(channel, mode, trigger, branch_lens)
        self.write(header, *data)


def main():
    import argparse
    parser = argparse.ArgumentParser(description="""PDQ DAC frontend.
            Evaluates times and voltages, interpolates them and uploads
            them to one section in one branch for one channel.""")
    parser.add_argument("-s", "--serial", default=None,
            help="device (FT245R) serial string [first]")
    parser.add_argument("-c", "--channel", default=0, type=int,
            help="channel: 3*fpga_num+dac_num [%(default)s]")
    parser.add_argument("-b", "--branch", default=7, type=int,
            help="branch number [%(default)s]")
    parser.add_argument("-f", "--free", default=False,
            action="store_true",
            help="untriggered, free running [%(default)s]")
    parser.add_argument("-t", "--times",
            default="np.linspace(0, 1e-3, 11)",
            help="sample times (s) [%(default)s]")
    parser.add_argument("-v", "--voltages",
            default="(1-np.cos(t/t[-1]*np.pi))/2",
            help="sample voltages (V) [%(default)s]")
    parser.add_argument("-m", "--mode", default=3, type=int,
            help="interpolation (0: v_only, 1: const, 2: quad, 3: cubic)"
                 " [%(default)s]")
    parser.add_argument("-x", "--test", default=False, action="store_true",
            help="test mode, assign all channels and all branches the"
                 "same waveform shifted by channel+.1*branch [%(default)s]")
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

    times = eval(args.times)
    voltages = eval(args.voltages, globals(), dict(t=times))

    if args.plot:
        from matplotlib import pyplot as plt
        times -= times[0]
        spline = interpolate.splrep(times, voltages,
                s=0, k=args.mode)
        ttimes = np.arange(0, times[-1], 1/Pdq.freq)
        vvoltages = interpolate.splev(ttimes, spline, der=0)
        fig, ax0 = plt.subplots()
        ax0.plot(times, voltages, "xk", label="points")
        ax0.plot(ttimes, vvoltages, ",b", label="interpolation")
        fig.savefig(args.plot)

    dev = Pdq(serial=args.serial)
    for channel in (args.channel == -1) and range(9) or [args.channel]:
        if args.branch == -1:
            # 8th branch (no 7) is always all data
            v = [.1*branch+channel+voltages for branch in range(8)]
        else:
            v = [None]*8
            v[args.branch] = voltages
        dev.prepare_simple(times, v, channel, args.mode, not args.free)


if __name__ == "__main__":
    main()
