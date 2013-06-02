from migen.fhdl.std import *
from migen.genlib.fsm import FSM
from migen.genlib.record import Record
from migen.flow.actor import *
from migen.flow.network import *
from migen.actorlib import sim


frame_layout = [
        ("wait", 1),
        ("dt", 15),
        ("v0", 48),
        ("v1", 48),
        ("v2", 48),
        ("v3", 48),
        ]


class DacReader(Module):
    def __init__(self, pads):
        self.specials.mem = Memory(width=16, depth=6144)
        read = self.mem.get_port()
        self.specials += read
        self.order = Signal(2)
        self.branch = Signal(3)
        self.branch_addr = Array(Signal(13) for _ in range(8))

        self.frame_out = Source(frame_layout)
        self.busy = ~self.frame_out.stb

        states = "INIT DT V0 V1A V1B V2A V2B V2C V3A V3B V3C IDLE".split()
        fsm = FSM(*states)
        self.submodules += fsm

        fp = self.frame_out.payload
        fsm.act(fsm.INIT,
                read.adr.eq(self.branch_addr[self.branch]),
                self.frame_out.stb.eq(0))
        fsm.act(fsm.DT, fp.wait.eq(read.dat_r[15]))
        fsm.act(fsm.DT, fp.dt.eq(read.dat_r[:15]))
        fsm.act(fsm.V0, fp.v0[32:].eq(read.dat_r))
        fsm.act(fsm.V1A, fp.v1[32:].eq(read.dat_r))
        fsm.act(fsm.V1B, fp.v1[16:32].eq(read.dat_r))
        fsm.act(fsm.V2A, fp.v2[32:].eq(read.dat_r))
        fsm.act(fsm.V2B, fp.v2[16:32].eq(read.dat_r))
        fsm.act(fsm.V2C, fp.v2[:16].eq(read.dat_r))
        fsm.act(fsm.V3A, fp.v3[32:].eq(read.dat_r))
        fsm.act(fsm.V3B, fp.v3[16:32].eq(read.dat_r))
        fsm.act(fsm.V3C, fp.v3[:16].eq(read.dat_r))
        for state in "DT V0 V1A V1B V2A V2B V2C V3A V3B V3C".split():
            fsm.act(getattr(fsm, state),
                read.adr.eq(read.adr + 1))
        fsm.act(fsm.V0, If(self.order <= 0,
                fp.v1.eq(0),
                fp.v2.eq(0),
                fp.v3.eq(0),
                fsm.next_state(fsm.IDLE)))
        fsm.act(fsm.V1B, If(self.order <= 1,
                fp.v2.eq(0),
                fp.v3.eq(0),
                fsm.next_state(fsm.IDLE)))
        fsm.act(fsm.V2C, If(self.order <= 2,
                fp.v3.eq(0),
                fsm.next_state(fsm.IDLE)))
        fsm.act(fsm.IDLE,
                self.frame_out.stb.eq(1),
                If(self.frame_out.ack,
                    fsm.next_state(fsm.INIT),
                ).Else(
                    fsm.next_state(fsm.IDLE),
                ))


class DacOut(Module):
    def __init__(self, pads):
        self.frame_in = Sink(frame_layout)
        self.busy = ~self.frame_in.ack

        self.trigger = Signal()

        t = Signal(15)
        frame = Record(frame_layout)

        self.sync += [
                If(t == 0,
                    self.frame_in.ack.eq(0),
                    If(self.trigger,
                        t.eq(t + 1),
                    ),
                ).Elif(t < frame.dt,
                    t.eq(t + 1),
                ).Elif((t >= frame.dt) & self.frame_in.stb,
                    frame.raw_bits().eq(self.frame_in.raw_bits()),
                    self.frame_in.ack.eq(1),
                    t.eq(0),
                ),
                ]

        self.sync += [
                frame.v0.eq(frame.v0 + frame.v1),
                frame.v1.eq(frame.v1 + frame.v2),
                frame.v2.eq(frame.v2 + frame.v3),
                ]

        out = Signal(16)
        self.comb += out.eq(frame.v0[-16:])
       
        clk = ClockSignal()
        self.comb += [
                pads.clk_p.eq(~clk),
                pads.clk_n.eq(clk),
                ]
        self.specials += Instance("OBUFDS",
                Instance.Input("I", ~clk),
                Instance.Output("O", pads.data_clk_p),
                Instance.Output("OB", pads.data_clk_n),
                )
        for i in range(16):
            self.specials += Instance("OBUFDS",
                    Instance.Input("I", ~out[i]),
                    Instance.Output("O", pads.data_p[i]),
                    Instance.Output("OB", pads.data_n[i]),
                    )


class Dac(Module):
    def __init__(self, pads):
        g = DataFlowGraph()
        self.reader = DacReader(pads)
        self.out = DacOut(pads)
        g.add_connection(self.reader, self.out)
        self.submodules += CompositeActor(g)
