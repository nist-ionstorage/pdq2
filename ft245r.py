# Robert Jordens <jordens@gmail.com> 2013

from math import ceil
import random

from migen.fhdl.std import *
from migen.flow.actor import Source
from migen.genlib.fsm import FSM, NextState
from migen.genlib.misc import timeline


class SimFt245r_rx(Module):
    def __init__(self, pads, data):
        self.pads = pads
        self.data = data
        self.sync += pads.rd_in.eq(pads.rd_out)
        pads.rxfl.reset = 1
        self.dat = Signal(flen(pads.data))
        self.comb += [
                If(~pads.rdl,
                    pads.data.eq(self.dat),
                ).Else(
                    pads.data.eq(0x55),
                )]
        self.state = "fill"
        self.wait = 10

    def do_simulation(self, selfp):
        self.wait = max(0, self.wait - 1)
        if self.state == "fill":
            selfp.pads.rxfl = 1
            if self.data and self.wait == 0:
                selfp.pads.rxfl = 0
                self.state = "read"
        elif self.state == "read":
            if selfp.pads.rdl == 0:
                self.wait = random.choice([0, 4]) # 5 - wr
                self.state = "setup"
        elif self.state == "setup":
            if self.wait == 0:
                selfp.dat = self.data.pop(0)
                self.state = "wait"
            if selfp.pads.rdl == 1:
                self.wait = random.choice([0, 1])
                self.state = "delay"
        elif self.state == "wait":
            if selfp.pads.rdl == 1:
                self.wait = random.choice([0, 1])
                self.state = "delay"
        elif self.state == "delay":
            if self.wait == 0:
                self.wait = random.choice([0, 3])
                self.state = "fill"


data_layout = [("data", 8)]


class Ft245r_rx(Module):
    def __init__(self, pads, clk=10.):
        self.data_out = Source(data_layout)
        do = self.data_out

        self.busy = Signal()
        self.comb += self.busy.eq(~do.stb | do.ack)

        self.sync += do.stb.eq(do.stb & ~do.ack)

        # t_RDLl_Dv = 50 ns (setup)
        # t_RDLh_Di = 0ns (hold)
        # t_RDLl_RDLh = 50 ns (redundant, <= r_RDLl_Dv)
        # t_RDLh_RXFLh = 25ns (from FT245R) (redundant, <= t_RDLh_RDLl)
        # t_RXFLh_RXFLl = 80ns (from FT245R)
        # t_RDLh_RDLl = 50ns + t_RXh_RXl (we understand this as a
        #    conditional addition)

        # we read every t_hold + t_precharge + t_setup + 2 cycles
        # can only sustain 1MByte/s anyway at full speed USB
        # stb implicitly needs to be acked within a read cycle
        #clk /= 4 # slow it down
        t_latch = int(ceil(50/clk)) # t_RDLl_Dv
        t_drop = t_latch + int(ceil(20/clk)) # slave skew
        t_refill = t_drop + int(ceil(50/clk)) # t_RDLh_RDLl

        reading = Signal()
        # proxy rxfl to slaves, drive rdl
        self.sync += If(~reading & ~pads.rd_in, pads.rd_out.eq(~pads.rxfl))
        self.comb += pads.rdl.eq(~pads.rd_out)

        self.sync += timeline(pads.rd_in, [
                (0, [reading.eq(1)]),
                (t_latch, [do.stb.eq(1), do.payload.data.eq(pads.data)]),
                (t_drop, [pads.rd_out.eq(0)]),
                (t_refill, [reading.eq(0)]),
            ])
