from migen.fhdl.std import *

class Ctrl(Module):
    def __init__(self, pads, comm, dacs):
        busy = Signal()
        self.comb += busy.eq(Cat(*(dac.out.busy for dac in dacs)) != 0)
        self.comb += pads.aux.eq(Cat(*(dac.out.aux for dac in dacs)) != 0)
        
        self.comb += pads.go2_out.eq(pads.go2_in) # dummy loop
        self.comb += pads.reset.eq(comm.memwriter.reset)

        for dac in dacs:
            self.comb += dac.parser.interrupt.eq(pads.interrupt)
            self.comb += dac.out.trigger.eq(pads.trigger |
                    comm.memwriter.trigger)
            self.comb += dac.out.arm.eq(comm.memwriter.arm)
