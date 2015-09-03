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

from io import BytesIO

from migen.sim.generic import run_simulation
from matplotlib import pyplot as plt
import numpy as np

from gateware.pdq2 import Pdq2Sim
from host import cli


def main():
    buf = BytesIO()
    cli.main(buf)

    tb = Pdq2Sim(buf.getvalue())
    run_simulation(tb, vcd_name="pdq2.vcd", ncycles=700)
    out = np.array(tb.outputs, np.uint16).view(np.int16)
    plt.plot(out)
    plt.show()

if __name__ == "__main__":
    main()
