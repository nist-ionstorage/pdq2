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
        clk = ClockSignal()
        dacs = []
        for i in range(3):
            dac = Dac()
            setattr(self.submodules, "dac{}".format(i), dac)
            dacs.append(dac)
            pads = platform.request("dac", i)
            # FIXME: do we really need the inversion
            # would registering the output not be enough?
            self.comb += pads.clk_p.eq(~clk), pads.clk_n.eq(clk)
            self.specials += Instance("OBUFDS",
                    Instance.Input("I", ~clk),
                    Instance.Output("O", pads.data_clk_p),
                    Instance.Output("OB", pads.data_clk_n),
                    )
            for i in range(16):
                self.specials += Instance("OBUFDS",
                        Instance.Input("I", ~dac.out.out[i]),
                        Instance.Output("O", pads.data_p[i]),
                        Instance.Output("OB", pads.data_n[i]),
                        )

        self.submodules.comm = Comm(platform.request("comm"), *dacs)
        self.submodules.ctrl = Ctrl(platform.request("ctrl"), *dacs)


class TB(Module):
    def __init__(self):
        dacs = []
        for i in range(3):
            pads = platform.request("dac", i)
            dac = Dac()
            setattr(self.submodules, "dac{}".format(i), dac)
            dacs.append(dac)
        self.submodules.comm = Comm(platform.request("comm"), *dacs)
        self.submodules.ctrl = Ctrl(platform.request("ctrl"), *dacs)

def main():
    tb = TB()

if __name__ == "__main__":
    main()
