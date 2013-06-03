from migen.fhdl.std import *
from migen.genlib.record import Record
from migen.flow.actor import *

from dac import Dac
from comm import Comm, SimComm
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


tb_pads = [
        ("adr", 4),
        ("aux", 1),
        ("branch", 3),
        ("trigger", 1),
        ("reset", 1),
        ]

class TB(Module):
    def __init__(self, mem=None):
        dacs = []
        for i in range(2):
            dac = Dac()
            setattr(self.submodules, "dac{}".format(i), dac)
            dacs.append(dac)
        self.pads = Record(tb_pads)
        self.submodules.comm = SimComm(mem, *dacs)
        self.submodules.ctrl = Ctrl(self.pads, *dacs)
        self.outputs = []

    def do_simulation(self, s):
        self.outputs.append(s.rd(self.dac0.out.out))
        if s.cycle_counter == 0:
            s.wr(self.pads.branch, 0)


def main():
    from migen.sim.generic import Simulator, TopLevel
    from matplotlib import pyplot as plt
    import numpy as np

    fil = "pdq_None_ftdi.bin"
    mem = np.fromstring(
            open(fil, "rb").read(),
            dtype=np.uint8)

    n = 1000
    tb = TB(mem)
    sim = Simulator(tb, TopLevel("top.vcd"))
    sim.run(n)
    t = np.arange(n)/50e6
    v = np.array(tb.outputs, np.uint16).view(np.int16)*10./(1<<16)
    plt.plot(t, v)
    plt.show()


if __name__ == "__main__":
    main()
