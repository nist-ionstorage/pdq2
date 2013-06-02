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
    def __init__(self, pads, mem_data_width=16, mem_adr_width=13,
            mem_depth=6144):
        self.specials.mem = Memory(width=mem_data_width, depth=mem_depth)
        read = self.mem.get_port()
        self.specials += read
        self.order = Signal(2)
        self.branch = Signal(3)
        self.branch_adr = Array(Signal(mem_adr_width) for _ in range(8))
        self.branch_start = Signal(mem_adr_width)
        self.branch_end = Signal(mem_adr_width)
        self.sync += [
                If(self.branch == 0,
                    self.branch_start.eq(0),
                ).Else(
                    self.branch_start.eq(self.branch_adr[self.branch-1]),
                ),
                self.branch_end.eq(self.branch_adr[self.branch]),
                ]

        self.frame_out = Source(frame_layout)
        self.busy = ~self.frame_out.stb

        states = "INIT V0 DT V1A V1B V2A V2B V2C V3A V3B V3C IDLE".split()
        fsm = FSM(*states)
        self.submodules += fsm

        fp = self.frame_out.payload

        fsm.act(fsm.INIT,
                If((read.adr < self.branch_start) | 
                   (read.adr >= self.branch_end),
                    read.adr.eq(self.branch_start),
                ),
                )
        fsm.act(fsm.V0, fp.v0[32:].eq(read.dat_r[:16]))
        fsm.act(fsm.DT,
                fp.wait.eq(read.dat_r[15]),
                If(read.dat_r[15],
                    fp.dt.eq(1 + ~read.dat_r[:15]),
                ).Else(
                    fp.dt.eq(read.dat_r[:15]),
                ),
                )
        fsm.act(fsm.V1A, fp.v1[32:].eq(read.dat_r[:16]))
        fsm.act(fsm.V1B, fp.v1[16:32].eq(read.dat_r[:16]))
        fsm.act(fsm.V2A, fp.v2[32:].eq(read.dat_r[:16]))
        fsm.act(fsm.V2B, fp.v2[16:32].eq(read.dat_r[:16]))
        fsm.act(fsm.V2C, fp.v2[:16].eq(read.dat_r[:16]))
        fsm.act(fsm.V3A, fp.v3[32:].eq(read.dat_r[:16]))
        fsm.act(fsm.V3B, fp.v3[16:32].eq(read.dat_r[:16]))
        fsm.act(fsm.V3C, fp.v3[:16].eq(read.dat_r[:16]))

        for state in "V0 DT V1A V1B V2A V2B V2C V3A V3B V3C".split():
            fsm.act(getattr(fsm, state),
                read.adr.eq(read.adr + 1))

        fsm.act(fsm.INIT, fsm.next_state(fsm.V0))
        fsm.act(fsm.V0,
                If(self.order == 0,
                    fp.wait.eq(0),
                    fp.dt.eq(1),
                    fp.v1.eq(0),
                    fp.v2.eq(0),
                    fp.v3.eq(0),
                    self.frame_out.stb.eq(1),
                    fsm.next_state(fsm.IDLE),
                ).Else(
                    fsm.next_state(fsm.DT),
                ))
        fsm.act(fsm.DT, fsm.next_state(fsm.V1A))
        fsm.act(fsm.V1A, fsm.next_state(fsm.V1B))
        fsm.act(fsm.V1B,
                If(self.order == 1,
                    fp.v2.eq(0),
                    fp.v3.eq(0),
                    self.frame_out.stb.eq(1),
                    fsm.next_state(fsm.IDLE),
                ).Else(
                    fsm.next_state(fsm.V2A),
                ))
        fsm.act(fsm.V2A, fsm.next_state(fsm.V2B))
        fsm.act(fsm.V2B, fsm.next_state(fsm.V2C))
        fsm.act(fsm.V2C,
                If(self.order == 2,
                    fp.v3.eq(0),
                    self.frame_out.stb.eq(1),
                    fsm.next_state(fsm.IDLE),
                ).Else(
                    fsm.next_state(fsm.V3A),
                ))
        fsm.act(fsm.V3A, fsm.next_state(fsm.V3B))
        fsm.act(fsm.V3B, fsm.next_state(fsm.V3C))
        fsm.act(fsm.V3C,
                self.frame_out.stb.eq(1),
                fsm.next_state(fsm.IDLE))
        fsm.act(fsm.IDLE,
                If(self.frame_out.ack,
                    self.frame_out.stb.eq(0),
                    fsm.next_state(fsm.INIT),
                ).Else(
                    fsm.next_state(fsm.IDLE),
                ))


class DacOut(Module):
    def __init__(self, pads):
        self.frame_in = Sink(frame_layout)
        frame = Record(frame_layout)

        self.trigger = Signal()
        t = Signal(15)
        
        self.busy = t < frame.dt

        self.sync += [
                If(t < frame.dt,
                    frame.v0.eq(frame.v0 + frame.v1),
                    frame.v1.eq(frame.v1 + frame.v2),
                    frame.v2.eq(frame.v2 + frame.v3),
                    t.eq(t + 1),
                    self.frame_in.ack.eq(0),
                ).Elif(self.frame_in.stb &
                        (self.trigger | ~self.frame_in.payload.wait),
                    self.frame_in.ack.eq(1),
                    frame.raw_bits().eq(self.frame_in.raw_bits()),
                    t.eq(0),
                ),
                ]

        out = Signal(16)
        out.eq(frame.v0[-16:])
       
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
