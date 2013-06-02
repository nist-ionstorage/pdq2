from migen.fhdl.std import *

class Ctrl(Module):
    def __init__(self, pads, *dacs):
        for dac in dacs:
            self.comb += [
                    dac.reader.branch.eq(pads.branch),
                    dac.out.trigger.eq(pads.trigger),
                    ]
        self.comb += pads.aux.eq(0)
