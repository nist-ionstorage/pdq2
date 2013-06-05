from migen.fhdl.std import *

class Ctrl(Module):
    def __init__(self, pads, *dacs):
        self.sync += [dac.reader.branch.eq(pads.branch) for dac in dacs]
        self.sync += [dac.out.trigger.eq(pads.trigger) for dac in dacs]
        self.comb += pads.aux.eq(Cat(*(dac.out.busy for dac in dacs)) == 0)
