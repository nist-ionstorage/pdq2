#!/usr/bin/python3
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

import sys

from migen.sim.generic import run_simulation
from matplotlib import pyplot as plt
import numpy as np

from gateware.pdq2 import Pdq2Sim


def main():
    tb = Pdq2Sim(open(sys.argv[1], "rb").read())
    run_simulation(tb, vcd_name="pdq2.vcd", ncycles=1000)
    out = np.array(tb.outputs, np.uint16).view(np.int16)
    plt.plot(out)
    plt.show()

if __name__ == "__main__":
    main()
