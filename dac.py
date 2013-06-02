from migen.fhdl.std import *
from migen.genlib.fsm import FSM
from migen.genlib.record import Record
from migen.flow.actor import *
from migen.flow.network import *
from migen.actorlib import spi, sim, structuring


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
        #mem_write = self.mem.get_port(write_capable=True,
        #        we_granularity=8)
        #self.specials += mem_write
        mem_read = self.mem.get_port()
        self.specials += mem_read
        self.order = Signal(2)
        self.branch = Signal(3)
        self.branch_addr = Array(Signal(13) for _ in range(8))

        self.frame = Source(frame_layout)
        self.busy = Signal()

        fsm_states = "INIT DT V0 V1A V1B V2A V2B V2C V3A V3B V3C IDLE".split()
        fsm_read = FSM(*fsm_states)
        self.submodules += fsm_read

        frame = self.frame.payload
        fsm_read.act(fsm_read.INIT,
                mem_read.adr.eq(self.branch_addr[self.branch]),
                self.frame.stb.eq(0),
                self.busy.eq(1))
        fsm_read.act(fsm_read.DT, frame.wait.eq(mem_read.dat_r[15]))
        fsm_read.act(fsm_read.DT, frame.dt.eq(mem_read.dat_r[:15]))
        fsm_read.act(fsm_read.V0, frame.v0[32:].eq(mem_read.dat_r))
        fsm_read.act(fsm_read.V1A, frame.v1[32:].eq(mem_read.dat_r))
        fsm_read.act(fsm_read.V1B, frame.v1[16:32].eq(mem_read.dat_r))
        fsm_read.act(fsm_read.V2A, frame.v2[32:].eq(mem_read.dat_r))
        fsm_read.act(fsm_read.V2B, frame.v2[16:32].eq(mem_read.dat_r))
        fsm_read.act(fsm_read.V2C, frame.v2[:16].eq(mem_read.dat_r))
        fsm_read.act(fsm_read.V3A, frame.v3[32:].eq(mem_read.dat_r))
        fsm_read.act(fsm_read.V3B, frame.v3[16:32].eq(mem_read.dat_r))
        fsm_read.act(fsm_read.V3C, frame.v3[:16].eq(mem_read.dat_r))
        for state in "DT V0 V1A V1B V2A V2B V2C V3A V3B V3C".split():
            fsm_read.act(getattr(fsm_read, state),
                mem_read.adr.eq(mem_read.adr + 1))
        fsm_read.act(fsm_read.V0, If(self.order <= 0,
                frame.v1.eq(0),
                frame.v2.eq(0),
                frame.v3.eq(0),
                fsm_read.next_state(fsm_read.IDLE)))
        fsm_read.act(fsm_read.V1B, If(self.order <= 1,
                frame.v2.eq(0),
                frame.v3.eq(0),
                fsm_read.next_state(fsm_read.IDLE)))
        fsm_read.act(fsm_read.V2C, If(self.order <= 2,
                frame.v3.eq(0),
                fsm_read.next_state(fsm_read.IDLE)))
        fsm_read.act(fsm_read.IDLE,
                self.frame.stb.eq(1),
                self.busy.eq(0),
                If(self.frame.ack,
                    fsm_read.next_state(fsm_read.INIT),
                ).Else(
                    fsm_read.next_state(fsm_read.IDLE),
                ))


class DacOut(Module):
    def __init__(self, pads):
        self.frame_in = Sink(frame_layout)
        self.busy = Signal()

        t = Signal(15)
        frame = Record(frame_layout)

        self.sync += [
                If(t < frame.dt,
                    self.frame_in.ack.eq(0),
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
        reader = DacReader(pads)
        out = DacOut(pads)
        g = DataFlowGraph()
        g.add_connection(reader, out)
        self.submodules += CompositeActor(g)
