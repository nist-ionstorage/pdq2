from migen.fhdl.std import *
from migen.genlib.record import Record
from migen.flow.actor import *

from dac import Dac
from comm import Comm


class Soc(Module):
    def __init__(self, platform):
        self.clock_domains.cd_sys = ClockDomain()
        clk_p = self.cd_sys.clk
        clk_n = Signal()
        dacs = []
        for i, mem in enumerate((1<<13, 1<<13, 1<<12)):
            dac = Dac(mem_depth=mem)
            setattr(self.submodules, "dac{}".format(i), dac)
            dacs.append(dac)

            pads = platform.request("dac", i)
            # FIXME: do we really need the inversion
            # would registering the output not be enough?
            self.comb += pads.clk_p.eq(clk_n), pads.clk_n.eq(clk_p)
            self.specials += Instance("OBUFDS",
                    Instance.Parameter("IOSTANDARD", "LVDS_25"),
                    Instance.Input("I", clk_n),
                    Instance.Output("O", pads.data_clk_p),
                    Instance.Output("OB", pads.data_clk_n),
                    )
            for i in range(16):
                self.specials += Instance("OBUFDS",
                        Instance.Parameter("IOSTANDARD", "LVDS_25"),
                        Instance.Input("I", ~dac.out.data[i]),
                        Instance.Output("O", pads.data_p[i]),
                        Instance.Output("OB", pads.data_n[i]),
                        )

        self.submodules.comm = Comm(platform.request("comm"), dacs)

        clkin = platform.request("clk50")
        clkin_period = 20

        platform.add_platform_command("""
NET "{clk50}" TNM_NET = "grp_clk50";
TIMESPEC "ts_grp_clk50" = PERIOD "grp_clk50" 20 ns HIGH 50%;
""", clk50=clkin)
        #TIMEGRP "dac_Out" OFFSET = OUT 10 ns AFTER "clk_dac";
 
        clkin_sdr = Signal()
        self.specials += Instance("IBUFG",
                Instance.Input("I", clkin),
                Instance.Output("O", clkin_sdr),
                )

        dcm_clk2x = Signal()
        dcm_clk2x180 = Signal()
        dcm_locked = Signal()
        self.specials += Instance("DCM_SP",
                Instance.Parameter("CLKDV_DIVIDE", 2),
                Instance.Parameter("CLKFX_DIVIDE", 1),
                Instance.Parameter("CLKFX_MULTIPLY", 4),
                Instance.Parameter("CLKIN_DIVIDE_BY_2", "FALSE"),
                Instance.Parameter("CLKIN_PERIOD", clkin_period),
                Instance.Parameter("CLK_FEEDBACK", "2X"),
                Instance.Parameter("DLL_FREQUENCY_MODE", "LOW"),
                Instance.Parameter("DFS_FREQUENCY_MODE", "LOW"),
                Instance.Parameter("STARTUP_WAIT", "FALSE"),
                Instance.Parameter("PHASE_SHIFT", 0),
                Instance.Parameter("DUTY_CYCLE_CORRECTION", "TRUE"),
                Instance.Input("RST", 0),
                Instance.Input("PSEN", 0),
                Instance.Input("PSINCDEC", 0),
                Instance.Input("PSCLK", 0),
                Instance.Input("CLKIN", clkin_sdr),
                Instance.Output("LOCKED", dcm_locked),
                Instance.Output("CLK2X", dcm_clk2x),
                Instance.Output("CLK2X180", dcm_clk2x180),
                Instance.Input("CLKFB", clk_p),
                )
        self.specials += Instance("BUFG",
                Instance.Input("I", dcm_clk2x),
                Instance.Output("O", clk_p),
                )
        self.specials += Instance("BUFG",
                Instance.Input("I", dcm_clk2x180),
                Instance.Output("O", clk_n),
                )
        #clk50 = platform.request("clk50")
        #self.comb += clk_p.eq(clk50), clk_n.eq(~clk50)
        
        sys_rst = ResetSignal()
        self.comb += sys_rst.eq(self.comm.ctrl.reset)



class TB(Module):
    comm_pads = [
            ("rxfl", 1),
            ("rdl", 1),
            ("rd_in", 1),
            ("rd_out", 1),
            ("data", 8),
            ("adr", 4),
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
        self.pads = Record(self.comm_pads)
        self.submodules.comm = Comm(self.pads, dacs, mem)
        self.outputs = []

    def do_simulation(self, s):
        if s.cycle_counter == 0:
            s.wr(self.pads.interrupt, 7) # pullup
            s.wr(self.pads.adr, 15) # active low, pullup
            s.wr(self.pads.trigger, 1) # pullup
        self.outputs.append(s.rd(self.dac1.out.data))


def main():
    from migen.sim.generic import Simulator, TopLevel
    from migen.sim.icarus import Runner
    from matplotlib import pyplot as plt
    import numpy as np

    import pdq
    pdq.Ftdi = pdq.FileFtdi

    t = np.arange(11)*.26e-6
    v = 9*(1-np.cos(t/t[-1]*np.pi))/2
    p = pdq.Pdq()
    mem = p.single_frame(t, v, channel=1, derivatives=4,
            aux=t>.5e-6, repeat=2, wait_last=True, time_shift=0)
    mem = (p.cmd("RESET_EN") + p.cmd("RESET_DIS") + mem
            + p.cmd("ARM_EN"))
    print(repr(mem))
    n = 10000
    tb = TB(list(mem))
    sim = Simulator(tb, TopLevel("top.vcd", clk_period=10)) #, Runner(keep_files=True))
    sim.run(n)
    out = np.array(tb.outputs, np.uint16).view(np.int16)*20./(1<<16)
    tim = np.arange(out.shape[0])/p.freq
    plt.plot(t, v)
    plt.plot(tim, out)
    plt.show()


if __name__ == "__main__":
    main()
