#!/usr/bin/python
# Robert Jordens <jordens@gmail.com>, 2012

import logging
import numpy as np
from scipy import interpolate

from .pdq2 import Pdq2


def main(dev=None):
    import argparse
    import time

    parser = argparse.ArgumentParser(description="""PDQ2 frontend.
            Evaluates times and voltages, interpolates and uploads
            them.""")
    parser.add_argument("-s", "--serial", default="hwgrep://",
                        help="device url [%(default)s]")
    parser.add_argument("-c", "--channel", default=0, type=int,
                        help="channel: 3*board_num+dac_num [%(default)s]")
    parser.add_argument("-f", "--frame", default=0, type=int,
                        help="frame [%(default)s]")
    parser.add_argument("-e", "--free", default=False, action="store_true",
                        help="software trigger [%(default)s]")
    parser.add_argument("-n", "--disarm", default=False, action="store_true",
                        help="disarm group [%(default)s]")
    parser.add_argument("-t", "--times", default="np.arange(5)*1e-6",
                        help="sample times (s) [%(default)s]")
    parser.add_argument("-v", "--voltages",
                        default="(1-np.cos(t/t[-1]*2*np.pi))/2",
                        help="sample voltages (V) [%(default)s]")
    parser.add_argument("-o", "--order", default=3, type=int,
                        help="interpolation (0: const, 1: lin, 2: quad,"
                        " 3: cubic) [%(default)s]")
    parser.add_argument("-m", "--dcm", default=None, type=int,
                        help="choose fast 100MHz clock [%(default)s]")
    parser.add_argument("-x", "--demo", default=False, action="store_true",
                        help="demo mode: pulse and chirp, 1V*ch+0.1V*frame"
                        " [%(default)s]")
    parser.add_argument("-p", "--plot", help="plot to file [%(default)s]")
    parser.add_argument("-d", "--debug", default=False,
                        action="store_true", help="debug communications")
    parser.add_argument("-r", "--reset", default=False,
                        action="store_true", help="do reset before")
    parser.add_argument("-b", "--bit", default=False,
                        action="store_true", help="do bit test")

    args = parser.parse_args()

    if args.debug:
        logging.basicConfig(level=logging.DEBUG)
    else:
        logging.basicConfig(level=logging.WARNING)

    times = eval(args.times, globals(), {})
    voltages = eval(args.voltages, globals(), dict(t=times))

    dev = Pdq2(args.serial, dev)

    if args.reset:
        dev.write(b"\x00")  # flush any escape
        dev.cmd("RESET", True)
        time.sleep(.1)
    if args.dcm:
        dev.cmd("DCM", True)
        dev.freq = 100e6
    elif args.dcm == 0:
        dev.cmd("DCM", False)
        dev.freq = 50e6
    dev.cmd("START", False)

    if args.demo:
        for ch, channel in enumerate(dev.channels):
            for fr in range(dev.channels[0].num_frames):
                vi = .1*fr + ch + voltages
                channel.segment(times, vi, order=args.order)
                #, end=False)
                pi = 2*np.pi*(.01*fr + .1*ch + 0*voltages)
                fi = 10e6*times/times[-1]
                #channel.segment(2*times, voltages, pi, fi, trigger=False, silence=True)
            dev.write_channel(channel)
    elif args.bit:
        v = [-1, 0, -1]
        #for i in range(15):
        #    v.extend([(1 << i) - 1, 1 << i])
        v = np.array(v)*dev.channels[0].max_out/dev.channels[0].max_val
        t = np.arange(len(v))
        for channel in dev.channels:
            channel.segment(t, v, order=0, shift=15, stop=False, trigger=False)
            dev.write_channel(channel)
    else:
        c = dev.channels[args.channel]
        s = c.segment(times, voltages, order=args.order)
        map = [None] * c.num_frames
        map[args.frame] = s
        dev.write_channel(c)

    dev.cmd("START", True)
    if not args.disarm:
        dev.cmd("ARM", True)
    if args.free:
        dev.cmd("TRIGGER", True)
    dev.close()

    if args.plot:
        from matplotlib import pyplot as plt
        fig, ax0 = plt.subplots()
        ax0.plot(times, voltages, "xk", label="points")
        if args.order:
            spline = interpolate.splrep(times, voltages, k=args.order)
            ttimes = np.arange(0, times[-1], 1/dev.freq)
            vvoltages = interpolate.splev(ttimes, spline)
            ax0.plot(ttimes, vvoltages, ",b", label="interpolation")
        fig.savefig(args.plot)


if __name__ == "__main__":
    main()
