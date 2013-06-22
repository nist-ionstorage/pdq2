from migen.fhdl.std import *
from migen.flow.actor import Source, Sink
from migen.flow.transactions import Token
from migen.actorlib.sim import SimActor
from migen.actorlib.structuring import Cast, Pack, pack_layout
from migen.flow.network import DataFlowGraph, CompositeActor

from escape import Unescaper
from ft245r import data_layout, SimFt245r_rx, Ft245r_rx


class SimReader(SimActor):
    def __init__(self, data):
        self.data_out = Source(data_layout)
        SimActor.__init__(self, self.data_gen(data))

    def data_gen(self, data):
        for msg in data:
            yield Token("data_out", {"data": msg})


mem_layout = [("data", 16)]


class MemWriter(Module):
    def __init__(self, pads, dacs):
        self.data_in = Sink(mem_layout)
        self.busy = Signal()
        self.comb += self.busy.eq(~self.data_in.ack | self.data_in.stb)
        mems = [dac.parser.mem.get_port(write_capable=True) for dac in dacs]
        self.specials += mems

        dev_adr = Signal(16)
        board_adr = Signal(flen(pads.adr))
        self.comb += board_adr.eq(dev_adr[:flen(board_adr)])
        dac_adr = Signal(max=len(dacs))
        self.comb += dac_adr.eq(dev_adr[8:8+flen(dac_adr)])
        listen = Signal()
        self.comb += listen.eq(board_adr == ~pads.adr)
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
        self.comb += mem_dat.eq(data) # gated by we
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


class Ctrl(Module):
    def __init__(self, pads, dacs):
        self.command_in = Sink(data_layout)
        cmd = self.command_in.payload.data
        self.comb += self.command_in.ack.eq(1) # can alway accept
        self.busy = Signal()
        self.comb += self.busy.eq(~self.command_in.ack | self.command_in.stb)
        
        self.reset = Signal()
        self.comb += pads.reset.eq(self.reset)
        self.trigger = Signal()
        self.arm = Signal()
 
        for dac in dacs:
            self.sync += [
                    dac.parser.interrupt.eq(pads.interrupt),
                    dac.out.trigger.eq(pads.trigger | self.trigger),
                    dac.out.arm.eq(self.arm),
                    ] # register these to ease timing

        self.comb += pads.aux.eq(Cat(*(dac.out.aux for dac in dacs)) != 0)
        #self.comb += pads.go2_out.eq(pads.go2_in) # dummy loop
        self.comb += pads.go2_out.eq(Cat(*(~dac.out.busy for dac in
            dacs)) != 0)
       
        commands = {
                "RESET_EN":    (0x00, [self.reset.eq(1)]),
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
        unescaper = Unescaper(data_layout, 0xaa)
        g.add_connection(reader, unescaper)
        pack = Pack(data_layout, 2)
        g.add_connection(unescaper, pack, "oa")
        #g.add_connection(reader, pack) # no escaping
        cast = Cast(pack_layout(data_layout, 2), mem_layout)
        g.add_connection(pack, cast)
        self.memwriter = memwriter = MemWriter(pads, dacs)
        g.add_connection(cast, memwriter)
        self.ctrl = ctrl = Ctrl(pads, dacs)
        g.add_connection(unescaper, ctrl, "ob")
        self.submodules.graph = CompositeActor(g)
