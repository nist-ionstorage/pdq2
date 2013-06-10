from migen.fhdl.std import *
from migen.genlib.record import Record
from migen.genlib.fsm import FSM
from migen.flow.actor import Source, Sink
from migen.flow.transactions import Token
from migen.actorlib.sim import SimActor
from migen.sim.generic import Simulator
from migen.flow.network import DataFlowGraph, CompositeActor


class Unescaper(Module):
    # something rare, 0xff and 0x00 are too common
    def __init__(self, layout, escape=0x5a):
        i = Sink(layout)
        oa, ob = Source(layout), Source(layout)
        self.i, self.oa, self.ob = i, oa, ob

        self.busy = Signal()
        self.comb += self.busy.eq(i.stb)

        self.comb += [oa.payload.eq(i.payload), ob.payload.eq(i.payload)]

        is_escape = Signal()
        self.comb += is_escape.eq(i.stb & (i.payload.raw_bits() == escape))
        was_escape = Signal()
        self.sync += If(i.ack & i.stb,
                was_escape.eq(is_escape & ~was_escape))
        ctrl = Cat(i.ack, oa.stb, ob.stb)
        self.comb += [
                If(is_escape == was_escape,
                    ctrl.eq(Cat(oa.ack, i.stb, 0)),
                ).Elif(is_escape,
                    ctrl.eq(Cat(1, 0, 0)),
                ).Else(
                    ctrl.eq(Cat(ob.ack, 0, i.stb)),
                )]


data_layout = [("data", 8)]


class SimSource(SimActor):
    def __init__(self, data):
        self.source = Source(data_layout)
        SimActor.__init__(self, self.gen(data))

    def gen(self, data):
        for i in data:
            # print("{} >".format(i))
            yield Token("source", {"data": i})

class SimSink(SimActor):
    def __init__(self, name):
        self.sink = Sink(data_layout)
        SimActor.__init__(self, self.gen(name))

    def gen(self, name):
        while True:
            t = Token("sink")
            yield t
            print("{}{}".format(name, t.value["data"]), end=" ")

class EscapeTB(Module):
    def __init__(self, data):
        self.source = SimSource(data)
        self.unescaper = Unescaper(data_layout, 0x5a)
        self.asink = SimSink("a")
        self.bsink = SimSink("b")
        g = DataFlowGraph()
        g.add_connection(self.source, self.unescaper)
        g.add_connection(self.unescaper, self.asink, "oa")
        g.add_connection(self.unescaper, self.bsink, "ob")
        self.submodules.comp = CompositeActor(g)

    def do_simulation(self, s):
        s.interrupt = self.source.token_exchanger.done


if __name__ == "__main__":
    from migen.fhdl import verilog

    print(verilog.convert(EscapeTB([])))

    data = [1, 2, 0x5a, 3, 4, 0x5a, 0x5a, 5, 6, 0x5a, 0x5a, 0x5a, 7, 8,
            0x5a, 0x5a, 0x5a, 0x5a, 9, 10]
    print("a1 a2 b3 a4 a90 a5 a6 a90 b7 a8 a90 a90 a9 a10")
    Simulator(EscapeTB(data)).run(200)
    print()
