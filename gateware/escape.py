# Copyright 2013-2015 Robert Jordens <jordens@gmail.com>
#
# This file is part of pdq2.
#
# pdq2 is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# pdq2 is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with pdq2.  If not, see <http://www.gnu.org/licenses/>.

from migen.fhdl.std import *
from migen.flow.actor import Source, Sink


class Unescaper(Module):
    """Split a data stream into an escaped low bandwidth command stream and an
    unescaped high bandwidth data stream.

    Items in the input stream that are escaped by being prefixed with the
    escape character, will be directed to the :attr:`source_b` output Source.

    Items that are not escaped, and the escaped escape character itself are
    directed at the :attr:`source_a` output Source.

    Args:
        layout (layout): Stream layout to split.
        escape (int): Escape character.

    Attributes:
        sink (Sink[layout]): Input stream.
        source_a (Source[layout]): High bandwidth unescaped data Source.
        source_b (Source[layout]): Low bandwidth command Source.
    """
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
