from migen.fhdl.std import *
from migen.flow.actor import Source, Sink
from migen.flow.transactions import Token
from migen.actorlib.sim import SimActor
from migen.sim.generic import run_simulation, StopSimulation
from migen.flow.network import DataFlowGraph, CompositeActor

from gateware.escape import Unescaper


data_layout = [("data", 8)]


class SimSource(SimActor):
    def __init__(self, data):
        self.source = Source(data_layout)
        SimActor.__init__(self, self.gen(data))

    def gen(self, data):
        for i in data:
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
    data = [1, 2, 0xa5, 3, 4, 0xa5, 0xa5, 5, 6, 0xa5, 0xa5, 0xa5, 7, 8,
            0xa5, 0xa5, 0xa5, 0xa5, 9, 10]
    aexpect = [1, 2, 4, 0xa5, 5, 6, 0xa5, 8, 0xa5, 0xa5, 9, 10]
    bexpect = [3, 7]
    tb = EscapeTB(data)
    run_simulation(tb, vcd_name="escape.vcd")
    assert tb.asink.recv == aexpect, (tb.asink.recv, aexpect)
    assert tb.bsink.recv == bexpect, (tb.bsink.recv, bexpect)
