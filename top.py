from migen.fhdl.std import *
from migen.genlib.record import Record
from migen.flow.actor import *

from dac import Dac
from comm import Comm
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

        self.submodules.comm = Comm(platform.request("comm"), dacs)
        self.submodules.ctrl = Ctrl(platform.request("ctrl"), self.comm, dacs)


class TB(Module):
    comm_pads = [
            ("rxfl", 1),
            ("rdl", 1),
            ("rd_in", 1),
            ("rd_out", 1),
            ("data", 8),
            ("adr", 4),
            ]
    ctrl_pads = [
            ("aux", 1),
            ("interrupt", 3),
            ("trigger", 1),
            ("reset", 1),
            ("go2_in", 1),
            ("go2_out", 1),
            ]

    def __init__(self, mem=None):
        dacs = []
        for i in range(3):
            dac = Dac()
            setattr(self.submodules, "dac{}".format(i), dac)
            dacs.append(dac)
            dac.parser.mem.init = [0] * dac.parser.mem.depth
        self.comm_pads = Record(self.comm_pads)
        self.submodules.comm = Comm(self.comm_pads, dacs, mem)
        self.ctrl_pads = Record(self.ctrl_pads)
        self.submodules.ctrl = Ctrl(self.ctrl_pads, self.comm, dacs)
        self.outputs = []

    def do_simulation(self, s):
        if s.cycle_counter == 0:
            s.wr(self.ctrl_pads.interrupt, 0)
            s.wr(self.comm_pads.adr, 1)
            #s.wr(self.ctrl_pads.trigger, 1)
        self.outputs.append(s.rd(self.dac1.out.data))


def main():
    from migen.sim.generic import Simulator, TopLevel
    from matplotlib import pyplot as plt
    import numpy as np

    import pdq
    pdq.Ftdi = pdq.FileFtdi

    t = np.arange(11)*.26e-6
    v = (1-np.cos(t/t[-1]*np.pi))/2
    p = pdq.Pdq()
    mem = p.single_frame(t, v, channel=4, mode=3, aux=t>.5e-6,
            repeat=2, wait_last=True, time_shift=0)
    mem = (p.cmd("RESET_EN") + mem + p.cmd("RESET_DIS")
            + p.cmd("ARM_EN"))

    n = 3000
    tb = TB(list(mem))
    sim = Simulator(tb, TopLevel("top.vcd"))
    sim.run(n)
    out = np.array(tb.outputs, np.uint16).view(np.int16)*20./(1<<16)
    tim = np.arange(out.shape[0])/50e6
    plt.plot(t, v)
    plt.plot(tim, out)
    plt.show()


if __name__ == "__main__":
    main()
