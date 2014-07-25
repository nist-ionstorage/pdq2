# Robert Jordens <jordens@gmail.com> 2013

import random

from migen.fhdl.std import *
from migen.genlib.record import Record
from migen.flow.actor import *

from .dac import Dac
from .comm import Comm
from .ft245r import SimFt245r_rx


class Pdq(Module):
    def __init__(self, platform, fast=True, silence=True):
        self.clock_domains.cd_sys = ClockDomain()
        clk_p = self.cd_sys.clk
        clk_n = Signal()
        rst = Signal()

        clkin = platform.request("clk50")
        clkin_period = 20

        if not fast:
            self.comb += [
                    clk_p.eq(clkin),
                    clk_n.eq(~clkin),
                    self.cd_sys.rst.eq(rst),
            ]
        else:
            clkin_sdr = Signal()
            self.specials += Instance("IBUFG", i_I=clkin, o_O=clkin_sdr)

            dcm_clk2x = Signal()
            dcm_clk2x180 = Signal()
            dcm_locked = Signal()
            dcm_sel = Signal()
            self.specials += Instance("DCM_SP",
                    p_CLKDV_DIVIDE=2,
                    p_CLKFX_DIVIDE=1,
                    p_CLKFX_MULTIPLY=2,
                    p_CLKIN_DIVIDE_BY_2="FALSE",
                    p_CLKIN_PERIOD=clkin_period,
                    #p_CLK_FEEDBACK="2X",
                    p_CLK_FEEDBACK="NONE",
                    p_DLL_FREQUENCY_MODE="LOW",
                    p_DFS_FREQUENCY_MODE="LOW",
                    p_STARTUP_WAIT="TRUE",
                    p_CLKOUT_PHASE_SHIFT="NONE",
                    p_PHASE_SHIFT=0,
                    p_DUTY_CYCLE_CORRECTION="TRUE",
                    i_RST=0,
                    i_PSEN=0,
                    i_PSINCDEC=0,
                    i_PSCLK=0,
                    i_CLKIN=clkin_sdr,
                    o_LOCKED=dcm_locked,
                    #o_CLK2X=dcm_clk2x,
                    #o_CLK2X180=dcm_clk2x180,
                    o_CLKFX=dcm_clk2x,
                    o_CLKFX180=dcm_clk2x180,
                    #i_CLKFB=clk_p,
                    i_CLKFB=0,
                    )
            self.specials += Instance("BUFGMUX",
                    i_I0=clkin_sdr, i_I1=dcm_clk2x, i_S=dcm_sel,
                    o_O=clk_p)
            self.specials += Instance("BUFGMUX",
                    i_I0=~clkin_sdr, i_I1=dcm_clk2x180, i_S=dcm_sel,
                    o_O=clk_n)
            self.specials += Instance("FD", p_INIT=1,
                    i_D=~dcm_locked | rst,
                    i_C=self.cd_sys.clk, o_Q=self.cd_sys.rst)

        dacs = []
        for i, mem in enumerate((1<<13, 1<<13, 1<<12)):
        #for i, mem in enumerate((1<<13, (1<<12)+(1<<11), (1<<12)+(1<<11))):
            dac = Dac(mem_depth=mem)
            setattr(self.submodules, "dac{}".format(i), dac)
            dacs.append(dac)

            pads = platform.request("dac", i)
            # inverted clocks ensure setup and hold times of data
            ce = ~dac.out.silence if silence else 1
            self.specials += Instance("ODDR2",
                    i_C0=clk_p, i_C1=clk_n, i_CE=ce,
                    i_D0=0, i_D1=1, i_R=0, i_S=0, o_Q=pads.clk_p)
            self.specials += Instance("ODDR2",
                    i_C0=clk_p, i_C1=clk_n, i_CE=ce,
                    i_D0=1, i_D1=0, i_R=0, i_S=0, o_Q=pads.clk_n)
            dclk = Signal()
            self.specials += Instance("ODDR2",
                    i_C0=clk_p, i_C1=clk_n, i_CE=ce,
                    i_D0=0, i_D1=1, i_R=0, i_S=0, o_Q=dclk)
            self.specials += Instance("OBUFDS",
                    i_I=dclk,
                    o_O=pads.data_clk_p, o_OB=pads.data_clk_n)
            for i in range(16):
                self.specials += Instance("OBUFDS",
                        i_I=~dac.out.data[i],
                        o_O=pads.data_p[i], o_OB=pads.data_n[i])

        pads = platform.request("comm")
        self.submodules.comm = Comm(pads, dacs)
        self.comb += [
                rst.eq(self.comm.ctrl.reset),
                pads.go2_out.eq(dcm_locked),
                dcm_sel.eq(self.comm.ctrl.dcm_sel)
        ]


class TB(Module):
    comm_pads = [
            ("rxfl", 1),
            ("rdl", 1),
            ("rd_in", 1),
            ("rd_out", 1),
            ("data", 8),
            ("adr", 4),
            ("aux", 1),
            ("frame", 3),
            ("trigger", 1),
            ("reset", 1),
            ("go2_in", 1),
            ("go2_out", 1),
    ]

    def __init__(self, mem=None):
        self.pads = Record(self.comm_pads)
        self.pads.adr.reset = 15
        if mem is not None:
            #reader = SimReader(mem)
            simin = SimFt245r_rx(self.pads, list(mem))
            self.submodules += simin
        dacs = [InsertReset(Dac()) for i in range(3)]
        self.submodules.comm = InsertReset(Comm(self.pads, dacs), ["sys"])
        self.comb += self.comm.reset_sys.eq(self.comm.ctrl.reset)
        for i, dac in enumerate(dacs):
            setattr(self.submodules, "dac{}".format(i), dac)
            dac.parser.mem.init = [random.randrange(1<<16)
                    for i in range(dac.parser.mem.depth)]
            self.comb += dac.reset.eq(self.comm.ctrl.reset)
        self.outputs = []

    def do_simulation(self, selfp):
        self.outputs.append([
            getattr(selfp, "dac{}".format(i)).out.data for i in range(3)])
    do_simulation.passive = True


def _main():
    from migen.fhdl import verilog
    from migen.sim.generic import run_simulation
    from matplotlib import pyplot as plt
    import numpy as np

    #print(verilog.convert(TB([])))

    from host import pdq
    pdq.Ftdi = pdq.FileFtdi

    t = np.arange(4)*.12e-6
    v = 9*(1-np.cos(t/t[-1]*np.pi))/2

    p = pdq.Pdq()
    mem = b"\x00" # flush any escape
    mem += p.cmd("RESET_EN")
    mem += p.escape(p.multi_frame([(t, v)], channel=2))
    mem += p.cmd("ARM_EN")
    mem += p.cmd("TRIGGER_EN") + p.cmd("TRIGGER_DIS")
    mem += p.cmd("TRIGGER_EN") + p.cmd("TRIGGER_DIS")

    tb = TB(list(mem))

    run_simulation(tb, vcd_name="pdq.vcd", ncycles=2000)
    out = np.array(tb.outputs, np.uint16).view(np.int16)*20./(1<<16)
    tim = np.arange(out.shape[0])/p.freq
    plt.plot(t, v)
    plt.plot(tim, out[:, 2])
    plt.show()


if __name__ == "__main__":
    _main()
