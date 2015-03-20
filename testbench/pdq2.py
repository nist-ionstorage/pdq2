#!/usr/bin/python
# Robert Jordens <jordens@gmail.com>, 2012

from io import BytesIO

from migen.sim.generic import run_simulation

from matplotlib import pyplot as plt
import numpy as np

from gateware.pdq2 import Pdq2Sim
from host import cli


def main():
    buf = BytesIO()
    cli.main(buf)

    tb = Pdq2Sim(list(buf.getvalue()), skip_ft245r=False)
    run_simulation(tb, vcd_name="pdq2.vcd", ncycles=6000)
    out = np.array(tb.outputs, np.uint16).view(np.int16)
    plt.plot(out)
    plt.show()

if __name__ == "__main__":
    main()
