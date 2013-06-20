from migen.fhdl.std import *
from migen.flow.actor import Source


class SimFt245r_rx(Module):
    def __init__(self, pads, data):
        self.pads = pads
        self.data = data
        self.comb += pads.rd_in.eq(pads.rd_out)
        self.state = "init"
        self.wait = 0

    def do_simulation(self, s):
        if self.state == "init":
            self.wait += 1
            s.wr(self.pads.rxfl, 1)
            if self.wait >= 1:
                self.wait = 0
                self.state = "fill"
        elif self.state == "fill":
            if self.data:
                s.wr(self.pads.rxfl, 0)
                if s.rd(self.pads.rdl) == 0:
                    self.state = "setup"
        elif self.state == "setup":
            self.wait += 1
            if self.wait >= 2:
                s.wr(self.pads.data, self.data.pop(0))
                self.wait = 0
                self.state = "hold"
        elif self.state == "hold":
            if s.rd(self.pads.rdl) == 1:
                s.wr(self.pads.rxfl, 1)
                self.state = "init"


data_layout = [("data", 8)]


class Ft245r_rx(Module):
    def __init__(self, pads):
        self.data_out = Source(data_layout)
        do = self.data_out
        self.comb += do.payload.data.eq(pads.data)

        complete = Signal()
        self.comb += complete.eq(do.stb & do.ack)
        self.busy = Signal()
        self.comb += self.busy.eq(~do.stb | do.ack)
        
        self.comb += pads.rdl.eq(~pads.rd_out) # master to all, enslave this

        wait = 5
        t = Signal(max=wait+1)

        self.sync += [
                If(t < wait,
                    t.eq(t + 1),
                ).Elif(pads.rd_in, # master to all
                    do.stb.eq(1),
                ).Elif(~pads.rxfl, # master only
                    pads.rd_out.eq(1), # master only
                    t.eq(0),
                ),
                If(complete,
                    do.stb.eq(0),
                    pads.rd_out.eq(0), # master only
                    t.eq(0),
                )]

