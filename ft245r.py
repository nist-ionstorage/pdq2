from math import ceil
import random

from migen.fhdl.std import *
from migen.flow.actor import Source
from migen.genlib.fsm import FSM, NextState


class SimFt245r_rx(Module):
    def __init__(self, pads, data):
        self.pads = pads
        self.data = data
        self.sync += pads.rd_in.eq(pads.rd_out)
        pads.rxfl.reset = 1
        self.state = "init"
        self.wait = 0

    def do_simulation(self, s):
        self.wait = max(0, self.wait - 1)
        if self.state == "init":
            if self.data and self.wait == 0:
                s.wr(self.pads.rxfl, 0)
                self.state = "fill"
            else:
                s.wr(self.pads.rxfl, 1)
        elif self.state == "fill":
            if s.rd(self.pads.rdl) == 0:
                self.state = "setup"
                self.wait = 3 # rdl, data
        elif self.state == "setup":
            if self.wait == 0:
                s.wr(self.pads.data, self.data.pop(0))
                self.wait = 0
                self.state = "hold"
        elif self.state == "hold":
            if s.rd(self.pads.rdl) == 1:
                self.wait = random.choice([0, 1, 2, 5])
                self.state = "init"


data_layout = [("data", 8)]


class Ft245r_rx(Module):
    def __init__(self, pads, clk=10.):
        self.data_out = Source(data_layout)
        do = self.data_out

        self.busy = Signal()
        self.comb += self.busy.eq(~do.stb | do.ack)

        stb_set = Signal()
        self.sync += [
                do.stb.eq(do.stb & ~do.ack),
                If(stb_set,
                    do.stb.eq(1),
                    do.payload.raw_bits().eq(pads.data),
                )]
       
        self.comb += pads.rdl.eq(~pads.rd_out) # master, enslave this

        rd_in = Signal()
        rxf = Signal()
        self.sync += [
                rd_in.eq(pads.rd_in),
                rxf.eq(~pads.rxfl),
                ]

        # t_RDl_Dv <= 50 ns (setup)
        # t_RDh_Di >= 0ns (hold)
        # t_RDl_RDh >= 50 ns (redundant, <= r_RDl_Dv)
        # t_RDh_RXh <= 25ns (from FT245R)
        # t_RXh_RXl >= 80ns (from FT245R)
        # t_RDh_RDk >= 50ns + t_RXh_RXl (we understand this as a
        #    conditional addition)
        
        # we read every t_fill + t_read + t_slave = 100ns
        # can only sustain 1MByte/s anyway, full speed USB
        t_fill = int(ceil(25/clk)) - 1 # t_RDl_RXh - state
        t_read = int(ceil(50/clk)) - 3 # t_RDl_Dv - state, stb, latch
        t_slave = int(ceil(10/clk)+1) + 1 # slave skew - t_RDh_Di + latch
        # stb implicitly needs to be acked within
        # t_slave + t_fill + t_read cycles
        max_wait = max(t_fill, t_read, t_slave)
        t = Signal(max=max_wait + 1)
        t_reset = Signal()
        self.sync += [
                If(t_reset,
                    t.eq(0),
                ).Elif(t != max_wait,
                    t.eq(t + 1),
                )]

        self.submodules.fsm = fsm = FSM()
        fsm.act("READ",
                pads.rd_out.eq((t >= t_fill) & rxf),
                If(rd_in,
                    t_reset.eq(1),
                    NextState("SETUP"),
                ))
        fsm.act("SETUP",
                pads.rd_out.eq(1),
                If(t == t_read,
                    stb_set.eq(1),
                    t_reset.eq(1),
                    NextState("HOLD"),
                ),
                If(~rd_in,
                    t_reset.eq(1),
                    NextState("READ"),
                ))
        fsm.act("HOLD",
                pads.rd_out.eq(t < t_slave),
                If(~rd_in,
                    t_reset.eq(1),
                    NextState("READ"),
                ))
