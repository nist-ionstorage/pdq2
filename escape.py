# Robert Jordens <jordens@gmail.com> 2013

from migen.fhdl.std import *
from migen.flow.actor import Source, Sink
from migen.flow.transactions import Token
from migen.actorlib.sim import SimActor
from migen.sim.generic import run_simulation, StopSimulation
from migen.flow.network import DataFlowGraph, CompositeActor


class Unescaper(Module):
    # something rare, 0xff and 0x00 are too common, 0xaa is 0b10101010
    def __init__(self, layout, escape=0xaa):
        self.sink = i = Sink(layout)
        self.source_a, self.source_b = oa, ob = Source(layout), Source(layout)
        self.comb += oa.payload.eq(i.payload), ob.payload.eq(i.payload)

        self.busy = Signal()
        self.comb += self.busy.eq(0)

        is_escape = Signal()

        was_escape = Signal()
        self.sync += If(i.ack & i.stb, was_escape.eq(is_escape & ~was_escape))

        ctrl = Cat(i.ack, oa.stb, ob.stb)
        self.comb += [
                is_escape.eq(i.stb & (i.payload.raw_bits() == escape)),
                If(is_escape == was_escape, # 00 or 11: data, oa
                    ctrl.eq(Cat(oa.ack, i.stb, 0)),
                ).Elif(is_escape, # 01, swallow
                    ctrl.eq(Cat(1, 0, 0)),
                ).Else( # 10, command, ob
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
        self.recv = []
        SimActor.__init__(self, self.gen(name))

    def gen(self, name):
        while True:
            t = Token("sink")
            yield t
            #print("{}{}".format(name, t.value["data"]), end=" ")
            self.recv.append(t.value["data"])


class EscapeTB(Module):
    def __init__(self, data):
        self.source = SimSource(data)
        unescaper = Unescaper(data_layout)
        self.asink = SimSink("a")
        self.bsink = SimSink("b")
        g = DataFlowGraph()
        g.add_connection(self.source, unescaper)
        g.add_connection(unescaper, self.asink, "source_a")
        g.add_connection(unescaper, self.bsink, "source_b")
        self.submodules.comp = CompositeActor(g)

    def do_simulation(self, selfp):
        if self.source.token_exchanger.done:
            raise StopSimulation


if __name__ == "__main__":
    #from migen.fhdl import verilog
    #print(verilog.convert(EscapeTB([])))
    data = [1, 2, 0xaa, 3, 4, 0xaa, 0xaa, 5, 6, 0xaa, 0xaa, 0xaa, 7, 8,
            0xaa, 0xaa, 0xaa, 0xaa, 9, 10]
    aexpect = [1, 2, 4, 0xaa, 5, 6, 0xaa, 8, 0xaa, 0xaa, 9, 10]
    bexpect = [3, 7]
    tb = EscapeTB(data)
    run_simulation(tb, vcd_name="escape.vcd")
    assert tb.asink.recv == aexpect, (tb.asink.recv, aexpect)
    assert tb.bsink.recv == bexpect, (tb.bsink.recv, bexpect)
