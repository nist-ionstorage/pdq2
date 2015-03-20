#!/usr/bin/python3
# Robert Jordens <jordens@gmail.com>, 2015

import sys

from migen.sim.generic import run_simulation
from matplotlib import pyplot as plt
import numpy as np

from gateware.pdq2 import Pdq2Sim


def main():
    tb = Pdq2Sim(open(sys.argv[1], "rb").read())
    run_simulation(tb, vcd_name="pdq2.vcd", ncycles=6000)
    out = np.array(tb.outputs, np.uint16).view(np.int16)
    plt.plot(out)
    plt.show()

if __name__ == "__main__":
    main()
