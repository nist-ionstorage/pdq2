from migen.fhdl.std import *
from migen.genlib.record import Record
from migen.flow.actor import Source, Sink
from migen.flow.transactions import Token
from migen.actorlib.sim import SimActor
from migen.actorlib.structuring import Cast, Pack, pack_layout
from migen.flow.network import DataFlowGraph, CompositeActor


class SimFt245r_rx(Module):
    def __init__(self, pads, data):
        self.pads = pads
        self.data = data
        self.comb += pads.rd_in.eq(pads.rd_out)
        self.state = "init"
        self.wait = 0

    def do_simulation(self, s):
        if self.state == "init":
            self.wait += 1
            s.wr(self.pads.rxfl, 1)
            if self.wait >= 1:
                self.wait = 0
                self.state = "fill"
        elif self.state == "fill":
            if self.data:
                s.wr(self.pads.rxfl, 0)
                if s.rd(self.pads.rdl) == 0:
                    self.state = "setup"
        elif self.state == "setup":
            self.wait += 1
            if self.wait >= 2:
                s.wr(self.pads.data, self.data.pop(0))
                self.wait = 0
                self.state = "hold"
        elif self.state == "hold":
            if s.rd(self.pads.rdl) == 1:
                s.wr(self.pads.rxfl, 1)
                self.state = "init"


data_layout = [("data", 8)]


class Ft245r_rx(Module):
    def __init__(self, pads):
        self.data_out = Source(data_layout)

        pads.rdl.reset = 1

        self.busy = Signal()
        self.comb += self.busy.eq(~self.data_out.stb)

        states = "FILL SETUP HOLD".split()
        states = dict((v, i) for i, v in enumerate(states))
        state = Signal(max=len(states))

        t = Signal(max=3)
        self.comb += pads.rd_out.eq(~pads.rdl)

        actions = {
                states["FILL"]: [
                    If(t < 3,
                        t.eq(t + 1),
                    ).Else(
                        If(pads.rd_in, # from master to both
                            t.eq(0),
                            state.eq(states["SETUP"]),
                        ).Elif(~pads.rxfl, # master only
                            pads.rdl.eq(0), # master ronly, need to wait one cycle
                        ),
                    )],
                states["SETUP"]: [
                    If(t < 3,
                        t.eq(t + 1),
                    ).Else(
                        self.data_out.payload.data.eq(pads.data),
                        self.data_out.stb.eq(1),
                        pads.rdl.eq(1), # master only
                        t.eq(0),
                        state.eq(states["HOLD"]),
                    )],
                states["HOLD"]: [
                    If(self.data_out.ack,
                        self.data_out.stb.eq(0),
                        state.eq(states["FILL"]),
                    )],
                }
        self.sync += Case(state, actions)
