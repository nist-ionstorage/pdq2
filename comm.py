from migen.fhdl.std import *
from migen.genlib.fsm import FSM
from migen.genlib.record import Record
from migen.flow.actor import *
from migen.flow.network import *
from migen.actorlib import spi, sim, structuring


data = [("data", 8)]


class Ft245r_rx(Module):
    def __init__(self, pads):
        self.data_out = Source(data)

        rxf = Signal()
        rd = Signal()
        self.comb += rxf.eq(~pads.rxfl), pads.rdl.eq(~rd)
        self.comb += self.data_out.payload.data.eq(pads.data)

        self.busy = ~self.data_out.stb

        states = "RESET FILL SETUP HOLD BLOCK".split()
        fsm = FSM(*states)
        self.submodules += fsm
        t = Signal(max=1<<20)
        self.reset_out = Signal()

        fsm.act(fsm.RESET,
                If(t < (1<<20)-1,
                    t.eq(t + 1),
                    self.reset_out.eq(1),
                ).Else(
                    t.eq(0),
                    self.reset_out.eq(0),
                    fsm.next_state(fsm.FILL),
                ))
        fsm.act(fsm.FILL,
                If(pads.rd_in,
                    rd.eq(1),
                    fsm.next_state(fsm.SETUP),
                ).Elif(rxf,
                    pads.rd_out.eq(1),
                ))
        fsm.act(fsm.SETUP,
                If(t < 3,
                    t.eq(t + 1),
                ).Else(
                    t.eq(0),
                    self.data_out.stb.eq(1),
                    fsm.next_state(fsm.HOLD),
                ))
        fsm.act(fsm.HOLD,
                If(t < 1,
                    t.eq(t + 1),
                ).Elif(self.data_out.ack,
                    t.eq(0),
                    rd.eq(0),
                    pads.rd_out.eq(0),
                    self.data_out.stb.eq(0),
                    fsm.next_state(fsm.BLOCK),
                ))
        fsm.act(fsm.BLOCK,
                If(t < 3,
                    t.eq(t + 1),
                ).Else(
                    t.eq(0),
                    fsm.next_state(fsm.FILL),
                ))


class Parser(Module):
    def __init__(self, pads, *dacs):
        self.data_in = Sink(data)
        self.busy = ~self.data_in.ack
        #mem_write = self.mem.get_port(write_capable=True,
        #        we_granularity=8)
        #self.specials += mem_write


class Comm(Module):
    def __init__(self, pads, *dacs):
        self.reader = Ft245r_rx(pads)
        self.parser = Parser(pads, *dacs)
        g = DataFlowGraph()
        g.add_connection(self.reader, self.parser)
        self.submodules += CompositeActor(g)
        
        self.comb += pads.reset.eq(self.reader.reset_out)
