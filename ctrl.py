from migen.fhdl.std import *
from migen.genlib.fsm import FSM
from migen.genlib.record import Record
from migen.flow.actor import *
from migen.flow.network import *
from migen.actorlib import spi, sim, structuring


class Ctrl(Module):
    def __init__(self, pads, *dacs):
        pass
