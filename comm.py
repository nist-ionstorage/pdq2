from migen.fhdl.std import *
from migen.genlib.fsm import FSM
from migen.genlib.record import Record
from migen.flow.actor import *
from migen.flow.network import *
from migen.actorlib import spi, sim, structuring


class Comm(Module):
    def __init__(self, pads, *dacs):
        pass
        #mem_write = self.mem.get_port(write_capable=True,
        #        we_granularity=8)
        #self.specials += mem_write
