# Robert Jordens <jordens@gmail.com> 2013-2015

from migen.fhdl.std import *
from migen.genlib.record import Record
from migen.genlib.resetsync import AsyncResetSynchronizer

from .dac import Dac
from .comm import Comm
from .ft245r import Ft245r_rx, SimFt245r_rx, SimReader


class Pdq2Base(Module):
    def __init__(self, ctrl_pads, mem_depths=(1 << 13, 1 << 13, 1 << 12)):
        # (1 << 13, (1 << 12) + (1 << 11), (1 << 12) + (1 << 11))
        self.dacs = []
        for i, depth in enumerate(mem_depths):
            dac = Dac(mem_depth=depth)
            setattr(self.submodules, "dac{}".format(i), dac)
            self.dacs.append(dac)
        self.submodules.comm = Comm(ctrl_pads, self.dacs)


class Pdq2Sim(Module):
    ctrl_layout = [
        ("adr", 4),
        ("aux", 1),
        ("frame", 3),
        ("trigger", 1),
        ("reset", 1),
        ("go2_in", 1),
        ("go2_out", 1),
    ]

    def __init__(self, mem, skip_ft245r=True):
        self.ctrl_pads = Record(self.ctrl_layout)
        self.ctrl_pads.adr.reset = 0b1111
        self.ctrl_pads.trigger.reset = 1
        self.ctrl_pads.frame.reset = 0b000
        self.submodules.dut = ResetInserter(["sys"])(Pdq2Base(self.ctrl_pads))
        self.comb += self.dut.reset_sys.eq(self.dut.comm.ctrl.reset)
        if skip_ft245r:
            reader = SimReader(mem)
        else:
            reader = SimFt245r_rx(mem)
        self.submodules.reader = ResetInserter(["sys"])(reader)
        self.comb += self.reader.reset_sys.eq(self.dut.comm.ctrl.reset)
        self.comb += self.dut.comm.sink.connect(self.reader.source)
        # override high-ack during reset draining the reader
        self.comb += self.reader.source.ack.eq(self.dut.comm.sink.ack &
                                               ~self.dut.comm.ctrl.reset)
        self.outputs = []

    def do_simulation(self, selfp):
        self.outputs.append([dac.out.data for dac in selfp.dut.dacs])
    do_simulation.passive = True


class CRG(Module):
    def __init__(self, platform):
        self.clock_domains.cd_sys = ClockDomain()
        self.clk_p = self.cd_sys.clk
        self.clk_n = Signal()
        self.rst = Signal()
        self.dcm_locked = Signal()
        self.dcm_sel = Signal()

        clkin = platform.request("clk50")
        clkin_period = 20

        clkin_sdr = Signal()
        self.specials += Instance("IBUFG", i_I=clkin, o_O=clkin_sdr)

        dcm_clk2x = Signal()
        dcm_clk2x180 = Signal()
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
                o_LOCKED=self.dcm_locked,
                #o_CLK2X=dcm_clk2x,
                #o_CLK2X180=dcm_clk2x180,
                o_CLKFX=dcm_clk2x,
                o_CLKFX180=dcm_clk2x180,
                #i_CLKFB=clk_p,
                i_CLKFB=0,
                )
        self.specials += Instance("BUFGMUX",
                i_I0=clkin_sdr, i_I1=dcm_clk2x, i_S=self.dcm_sel,
                o_O=self.clk_p)
        self.specials += Instance("BUFGMUX",
                i_I0=~clkin_sdr, i_I1=dcm_clk2x180, i_S=self.dcm_sel,
                o_O=self.clk_n)
        self.specials += AsyncResetSynchronizer(
            self.cd_sys, ~self.dcm_locked | self.rst)


class Pdq2(Pdq2Base):
    def __init__(self, platform):
        ctrl_pads = platform.request("ctrl")
        Pdq2Base.__init__(self, ctrl_pads)
        self.submodules.crg = CRG(platform)
        comm_pads = platform.request("comm")
        self.submodules.reader = Ft245r_rx(comm_pads)
        self.comb += [
                self.comm.sink.connect(self.reader.source),
                self.crg.rst.eq(self.comm.ctrl.reset),
                ctrl_pads.go2_out.eq(self.crg.dcm_locked),
                self.crg.dcm_sel.eq(self.comm.ctrl.dcm_sel)
        ]

        for i, dac in enumerate(self.dacs):
            pads = platform.request("dac", i)
            # inverted clocks ensure setup and hold times of data
            ce = Signal()
            d = Signal.like(dac.out.data)
            self.comb += [
                    ce.eq(~dac.out.silence),
                    d.eq(~dac.out.data),  # pcb inversion
            ]

            self.specials += Instance("ODDR2",
                    i_C0=self.crg.clk_p, i_C1=self.crg.clk_n, i_CE=ce,
                    i_D0=0, i_D1=1, i_R=0, i_S=0, o_Q=pads.clk_p)
            self.specials += Instance("ODDR2",
                    i_C0=self.crg.clk_p, i_C1=self.crg.clk_n, i_CE=ce,
                    i_D0=1, i_D1=0, i_R=0, i_S=0, o_Q=pads.clk_n)
            dclk = Signal()
            self.specials += Instance("ODDR2",
                    i_C0=self.crg.clk_p, i_C1=self.crg.clk_n, i_CE=ce,
                    i_D0=0, i_D1=1, i_R=0, i_S=0, o_Q=dclk)
            self.specials += Instance("OBUFDS",
                    i_I=dclk, o_O=pads.data_clk_p, o_OB=pads.data_clk_n)
            for i in range(16):
                self.specials += Instance("OBUFDS",
                        i_I=d[i], o_O=pads.data_p[i], o_OB=pads.data_n[i])
