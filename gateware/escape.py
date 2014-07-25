# Robert Jordens <jordens@gmail.com> 2013

from migen.fhdl.std import *
from migen.flow.actor import Source, Sink


class Unescaper(Module):
    def __init__(self, layout, escape=0xa5):
        self.sink = i = Sink(layout)
        self.source_a = oa = Source(layout)
        self.source_b = ob = Source(layout)
        self.busy = Signal()

        ###

        is_escape = Signal()
        was_escape = Signal()
        ctrl = Cat(i.ack, oa.stb, ob.stb)

        self.sync += [
                If(i.ack & i.stb,
                    was_escape.eq(is_escape & ~was_escape)
                )
        ]

        self.comb += [
                oa.payload.eq(i.payload),
                ob.payload.eq(i.payload),
                is_escape.eq(i.stb & (i.payload.raw_bits() == escape)),
                If(is_escape == was_escape, # 00 or 11: data, oa
                    ctrl.eq(Cat(oa.ack, i.stb, 0)),
                ).Elif(is_escape, # 01, swallow
                    ctrl.eq(Cat(1, 0, 0)),
                ).Else( # 10, command, ob
                    ctrl.eq(Cat(ob.ack, 0, i.stb)),
                )
        ]
