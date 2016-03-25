#!/usr/bin/python
# Copyright 2013-2015 Robert Jordens <jordens@gmail.com>
#
# This file is part of pdq2.
#
# pdq2 is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# pdq2 is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with pdq2.  If not, see <http://www.gnu.org/licenses/>.

import logging
import numpy as np
from scipy import interpolate

from .pdq2 import Pdq2

import argparse
import time


def get_argparser():
    parser = argparse.ArgumentParser(description="""PDQ2 frontend.
            Evaluates times and voltages, interpolates and uploads
            them.""")
    parser.add_argument("-s", "--serial", default="hwgrep://",
                        help="device url [%(default)s]")
    parser.add_argument("-c", "--channel", default=0, type=int,
                        help="channel: 3*board_num+dac_num [%(default)s]")
    parser.add_argument("-f", "--frame", default=0, type=int,
                        help="frame [%(default)s]")
    parser.add_argument("-t", "--times", default="np.arange(5)*1e-6",
                        help="sample times (s) [%(default)s]")
    parser.add_argument("-v", "--voltages",
                        default="(1-np.cos(t/t[-1]*2*np.pi))/2",
                        help="sample voltages (V) [%(default)s]")
    parser.add_argument("-o", "--order", default=3, type=int,
                        help="interpolation (0: const, 1: lin, 2: quad,"
                        " 3: cubic) [%(default)s]")
    parser.add_argument("-u", "--dump", help="dump to file [%(default)s]")
    parser.add_argument("-r", "--reset", default=False,
                        action="store_true", help="do reset before")
    parser.add_argument("-m", "--multiplier", default=False,
                        action="store_true", help="100MHz clock [%(default)s]")
    parser.add_argument("-n", "--disarm", default=False, action="store_true",
                        help="disarm group [%(default)s]")
    parser.add_argument("-e", "--free", default=False, action="store_true",
                        help="software trigger [%(default)s]")
    parser.add_argument("-d", "--debug", default=False,
                        action="store_true", help="debug communications")
    return parser


def main(dev=None):
    parser = get_argparser()
    args = parser.parse_args()

    if args.debug:
        logging.basicConfig(level=logging.DEBUG)
    else:
        logging.basicConfig(level=logging.WARNING)

    if args.dump:
        dev = open(args.dump, "wb")
    dev = Pdq2(args.serial, dev)

    if args.reset:
        dev.write(b"\x00\x00")  # flush any escape
        dev.cmd("RESET", True)
        time.sleep(.1)

    dev.cmd("DCM", args.multiplier)
    freq = 50e6
    if args.multiplier:
        freq *= 2

    times = np.around(eval(args.times, globals(), {})*freq)
    voltages = eval(args.voltages, globals(), dict(t=times/freq))

    dev.cmd("START", False)
    dev.cmd("ARM", True)
    dev.cmd("TRIGGER", True)

    dt = np.diff(times.astype(np.int))
    if args.order:
        tck = interpolate.splrep(times, voltages, k=args.order, s=0)
        u = interpolate.spalde(times, tck)
    else:
        u = voltages[:, None]
    segment = []
    for dti, ui in zip(dt, u):
        segment.append({
            "duration": int(dti),
            "channel_data": [{
                "bias": {
                    "amplitude": [float(uij) for uij in ui]
                }
            }]
        })
    program = [[] for i in range(dev.channels[args.channel].num_frames)]
    program[args.frame] = segment
    dev.program(program, [args.channel])

    dev.cmd("TRIGGER", args.free)
    dev.cmd("ARM", not args.disarm)
    dev.cmd("START", True)


if __name__ == "__main__":
    main()
