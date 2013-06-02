from fractions import Fraction
from math import ceil
from operator import itemgetter

from migen.fhdl.std import *
#from migen.bus import wishbone, wishbone2asmi, csr, wishbone2csr, dfi

from dac import Dac
from comm import Comm
from ctrl import Ctrl


class Soc(Module):
    def __init__(self, platform):
        self.submodules.dac0 = Dac(platform.request("dac", 0))
        self.submodules.dac1 = Dac(platform.request("dac", 1))
        self.submodules.dac2 = Dac(platform.request("dac", 2))
        self.submodules.comm = Comm(platform.request("comm"),
                self.dac0, self.dac1, self.dac2)
        self.submodules.ctrl = Ctrl(platform.request("ctrl"),
                self.dac0, self.dac1, self.dac2)


if __name__ == "__main__":
    main()
