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
            mem_depth=6*(1<<10)): # XC3S500E: 20BRAMS 18bits 1024loc
        self.specials.mem = Memory(width=mem_data_width, depth=mem_depth)
        read = self.mem.get_port()
        self.specials += read
        adr = Signal()
        self.sync += read.adr.eq(adr)
        self.order = Signal(2)
        self.branch = Signal(3)
        self.branch_adrs = Array(Signal(mem_adr_width) for _ in range(8))
        self.branch_start = Signal(mem_adr_width)
        self.branch_end = Signal(mem_adr_width)
        self.sync += [
                If(self.branch == 0,
                    self.branch_start.eq(0),
                ).Else(
                    self.branch_start.eq(self.branch_adrs[self.branch-1]),
                ),
                self.branch_end.eq(self.branch_adrs[self.branch]),
                ]

        self.frame_out = Source(frame_layout)
        self.busy = ~self.frame_out.stb

        states = "INIT V0 DT V1A V1B V2A V2B V2C V3A V3B V3C IDLE".split()
        fsm = FSM(*states)
        self.submodules += fsm

        read_states = states[1:]
        for i, state in enumerate(read_states[:-1]):
            fsm.act(getattr(fsm, state),
                adr.eq(read.adr + 1),
                fsm.next_state(getattr(fsm, read_states[i+1])),
                )
        
        fsm.act(fsm.INIT,
                If((read.adr < self.branch_start) | 
                   (read.adr >= self.branch_end),
                    adr.eq(self.branch_start),
                ),
                fsm.next_state(fsm.V0),
                )

        fp = self.frame_out.payload
        fsm.act(fsm.V0,
                fp.v0[32:].eq(read.dat_r[:16]),
                If(self.order == 0,
                    fp.wait.eq(0),
                    fp.dt.eq(2), # INIT, V0
                    fp.v1.eq(0),
                    fp.v2.eq(0),
                    fp.v3.eq(0),
                    self.frame_out.stb.eq(1),
                    fsm.next_state(fsm.IDLE),
                ))
        fsm.act(fsm.DT,
                fp.wait.eq(read.dat_r[15]),
                If(read.dat_r[15],
                    fp.dt.eq(1 + ~read.dat_r[:15]),
                ).Else(
                    fp.dt.eq(read.dat_r[:15]),
                ))
        fsm.act(fsm.V1A,
                fp.v1[32:].eq(read.dat_r[:16]),
                )
        fsm.act(fsm.V1B,
                fp.v1[16:32].eq(read.dat_r[:16]),
                If(self.order == 1,
                    fp.v2.eq(0),
                    fp.v3.eq(0),
                    self.frame_out.stb.eq(1),
                    fsm.next_state(fsm.IDLE),
                ))
        fsm.act(fsm.V2A,
                fp.v2[32:].eq(read.dat_r[:16]),
                )
        fsm.act(fsm.V2B,
                fp.v2[16:32].eq(read.dat_r[:16]),
                )
        fsm.act(fsm.V2C,
                fp.v2[:16].eq(read.dat_r[:16]),
                If(self.order == 2,
                    fp.v3.eq(0),
                    self.frame_out.stb.eq(1),
                    fsm.next_state(fsm.IDLE),
                ))
        fsm.act(fsm.V3A,
                fp.v3[32:].eq(read.dat_r[:16]),
                )
        fsm.act(fsm.V3B,
                fp.v3[16:32].eq(read.dat_r[:16]),
                )
        fsm.act(fsm.V3C,
                fp.v3[:16].eq(read.dat_r[:16]),
                If(self.order == 3,
                    self.frame_out.stb.eq(1),
                    fsm.next_state(fsm.IDLE),
                ),
                )
        fsm.act(fsm.IDLE,
                If(self.frame_out.ack,
                    self.frame_out.stb.eq(0),
                    fsm.next_state(fsm.INIT),
                ),
                )


class DacOut(Module):
    def __init__(self, pads):
        self.frame_in = Sink(frame_layout)
        frame = Record(frame_layout)

        self.trigger = Signal()
        t = Signal(15)
        t1 = Signal(15)
        self.sync += t.eq(t1)

        self.freerun = Signal()
        
        self.busy = t < frame.dt

        self.sync += [
                If(t < frame.dt,
                    self.frame_in.ack.eq(0),
                    frame.v0.eq(frame.v0 + frame.v1),
                    frame.v1.eq(frame.v1 + frame.v2),
                    frame.v2.eq(frame.v2 + frame.v3),
                    t1.eq(t + 1),
                ).Elif(self.frame_in.stb &
                        (self.freerun | self.trigger | ~frame.wait),
                    frame.raw_bits().eq(self.frame_in.payload.raw_bits()),
                    self.frame_in.ack.eq(1),
                    t1.eq(0),
                ),
                ]

        out = Signal(16)
        self.sync += out.eq(frame.v0[-16:])
       
        clk = ClockSignal()
        self.comb += [
                # FIXME: do we really need the inversion
                # would registering the output not be enough?
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
