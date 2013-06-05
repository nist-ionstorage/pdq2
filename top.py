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
        for i, mem in enumerate((1<<13, 1<<13, 1<<12)):
            dac = Dac(mem_depth=mem)
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
                        Instance.Input("I", ~dac.out.data[i]),
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
        for i in range(3):
            dac = Dac()
            setattr(self.submodules, "dac{}".format(i), dac)
            dacs.append(dac)
            dac.reader.mem.init = [0] * dac.reader.mem.depth
        self.pads = Record(tb_pads)
        self.submodules.comm = SimComm(mem, *dacs)
        self.submodules.ctrl = Ctrl(self.pads, *dacs)
        self.comb += self.comm.parser.adr.eq(self.pads.adr)
        self.outputs = []

    def do_simulation(self, s):
        if s.cycle_counter == 0:
            s.wr(self.pads.branch, 1)
            s.wr(self.comm.parser.adr, 1)
            # s.wr(self.pads.trigger, 1)
        self.outputs.append(s.rd(self.dac1.out.data))


def main():
    from migen.sim.generic import Simulator, TopLevel
    from matplotlib import pyplot as plt
    import numpy as np

    import pdq
    pdq.Ftdi = pdq.FileFtdi

    t = np.linspace(0, 3e-6, 11)
    v = [None] * 8
    v[1] = (1-np.cos(t/t[-1]*np.pi))/2
    p = pdq.Pdq("top_test")
    p.prepare_simple(t, v, channel=4, mode=3, trigger=False)
    p.dev.fil.close()
    fil = "pdq_top_test_ftdi.bin"
    mem = np.fromstring(
            open(fil, "rb").read(),
            dtype=np.uint8)

    n = 1000
    tb = TB(mem)
    sim = Simulator(tb, TopLevel("top.vcd"))
    sim.run(n)
    out = np.array(tb.outputs, np.uint16).view(np.int16)*20./(1<<16)
    tim = np.arange(out.shape[0])/50e6
    plt.plot(t, v[1])
    plt.plot(tim, out)
    plt.show()


if __name__ == "__main__":
    main()
