from migen.fhdl.std import *
from migen.genlib.record import Record
from migen.flow.actor import Source, Sink
from migen.flow.transactions import Token
from migen.actorlib.sim import SimActor
from migen.actorlib.structuring import Cast, Pack, pack_layout
from migen.flow.network import DataFlowGraph, CompositeActor

from escape import Unescaper


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

        pads.rdl.reset = 1

        self.busy = Signal()
        self.comb += self.busy.eq(~self.data_out.stb)

        states = "FILL SETUP HOLD".split()
        states = dict((v, i) for i, v in enumerate(states))
        state = Signal(max=len(states))

        t = Signal(max=3)
        self.comb += pads.rd_out.eq(~pads.rdl)

        actions = {
                states["FILL"]: [
                    If(t < 3,
                        t.eq(t + 1),
                    ).Else(
                        If(pads.rd_in, # from master to both
                            t.eq(0),
                            state.eq(states["SETUP"]),
                        ).Elif(~pads.rxfl, # master only
                            pads.rdl.eq(0), # master ronly, need to wait one cycle
                        ),
                    )],
                states["SETUP"]: [
                    If(t < 3,
                        t.eq(t + 1),
                    ).Else(
                        self.data_out.payload.data.eq(pads.data),
                        self.data_out.stb.eq(1),
                        pads.rdl.eq(1), # master only
                        t.eq(0),
                        state.eq(states["HOLD"]),
                    )],
                states["HOLD"]: [
                    If(self.data_out.ack,
                        self.data_out.stb.eq(0),
                        state.eq(states["FILL"]),
                    )],
                }
        self.sync += Case(state, actions)


class SimReader(SimActor):
    def __init__(self, data):
        self.data_out = Source(data_layout)
        SimActor.__init__(self, self.data_gen(data))

    def data_gen(self, data):
        for msg in data:
            yield Token("data_out", {"data": msg})


mem_layout = [("data", 16)]


class MemWriter(Module):
    def __init__(self, dacs):
        self.data_in = Sink(mem_layout)
        self.busy = Signal()
        self.comb += self.busy.eq(~self.data_in.ack)
        mems = [dac.parser.mem.get_port(write_capable=True) for dac in dacs]
        self.specials += mems
       
        self.adr = Signal(4)

        dev_adr = Signal(16)
        board_adr = Signal(4)
        self.comb += board_adr.eq(dev_adr[:flen(self.adr)])
        dac_adr = Signal(2)
        self.comb += dac_adr.eq(dev_adr[8:8+flen(dac_adr)])
        listen = Signal()
        self.comb += listen.eq(board_adr == self.adr)
        data_len = Signal(16)
        mem_adr = Signal(16)
        self.comb += [mem.adr.eq(mem_adr[:flen(mem.adr)]) for mem in mems]
        mem_dat = Signal(16)
        self.comb += [mem.dat_w.eq(mem_dat) for mem in mems]
        we = Array(mem.we for mem in mems)[dac_adr]

        states = "DEV_ADR MEM_ADR DATA_LEN DATA".split()
        states = dict((v, i) for i, v in enumerate(states))
        state = Signal(max=len(states))
 
        self.comb += self.data_in.ack.eq(1) # can always accept
        data = self.data_in.payload.data
        self.comb += mem_dat.eq(data)
        self.comb += we.eq(listen & (state == states["DATA"])
                & self.data_in.stb)

        seq = [("DEV_ADR", dev_adr), ("MEM_ADR", mem_adr),
                ("DATA_LEN", data_len), ("DATA", None)]
        actions = {}
        for i, (st, reg) in enumerate(seq[:-1]):
            actions[states[st]] = [reg.eq(data),
                    state.eq(states[seq[i+1][0]])]
        actions[states["DATA"]] = [
                    mem_adr.eq(mem_adr + 1),
                    data_len.eq(data_len - 1),
                    If(data_len <= 1,
                        state.eq(states["DEV_ADR"]),
                    )]
        self.sync += If(self.data_in.stb & self.data_in.ack,
                Case(state, actions))

        self.command_in = Sink(data_layout)
        cmd = self.command_in.payload.data
        self.comb += self.command_in.ack.eq(1) # can alway accept
        
        self.reset = Signal()
        self.trigger = Signal()
        self.arm = Signal()
        
        commands = {
                "RESET_EN":    (0x00, [
                    self.reset.eq(1), self.trigger.eq(0), self.arm.eq(0)]),
                "RESET_DIS":   (0x01, [self.reset.eq(0)]),
                "TRIGGER_EN":  (0x02, [self.trigger.eq(1)]),
                "TRIGGER_DIS": (0x03, [self.trigger.eq(0)]),
                "ARM_EN":      (0x04, [self.arm.eq(1)]),
                "ARM_DIS":     (0x05, [self.arm.eq(0)]),
                }
        self.sync += If(self.command_in.stb, Case(cmd,
            dict((code, act) for name, (code, act) in commands.items())))


class Comm(Module):
    def __init__(self, pads, dacs, mem=None):
        g = DataFlowGraph()
        if mem is not None:
            #reader = SimReader(mem)
            self.submodules.simin = SimFt245r_rx(pads, mem)
        reader = Ft245r_rx(pads)
        unescaper = Unescaper(data_layout, 0x5a)
        g.add_connection(reader, unescaper)
        pack = Pack(data_layout, 2)
        g.add_connection(unescaper, pack, "oa", None)
        #g.add_connection(reader, pack)
        cast = Cast(pack_layout(data_layout, 2), mem_layout)
        g.add_connection(pack, cast)
        memwriter = MemWriter(dacs)
        self.memwriter = memwriter
        g.add_connection(cast, memwriter, None, "data_in")
        g.add_connection(unescaper, memwriter, "ob", "command_in")
        self.submodules += CompositeActor(g) 

        self.comb += memwriter.adr.eq(pads.adr)
