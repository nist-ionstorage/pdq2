from migen.fhdl.std import *
from migen.genlib.record import Record
from migen.flow.actor import Source, Sink
from migen.flow.transactions import Token
from migen.actorlib.sim import SimActor
from migen.actorlib.structuring import Cast, Pack, pack_layout
from migen.flow.network import DataFlowGraph, CompositeActor


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

        states = "RESET FILL SETUP HOLD".split()
        states = dict((v, i) for i, v in enumerate(states))
        state = Signal(max=len(states))

        t = Signal(max=1<<20)
        self.reset = Signal()
        self.comb += self.reset.eq(state == states["RESET"])
        self.comb += pads.rd_out.eq(~pads.rdl)

        actions = {
                states["RESET"]: [
                    If(t < (1<<6)-1, # FIXME 20
                        t.eq(t + 1),
                    ).Else(
                        t.eq(0),
                        state.eq(states["FILL"]),
                    )],
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

        states = "DEV_ADR DATA_LEN MEM_ADR DATA".split()
        states = dict((v, i) for i, v in enumerate(states))
        state = Signal(max=len(states))
 
        self.comb += self.data_in.ack.eq(1) # can always accept
        data = self.data_in.payload.data
        actions = {
                states["DEV_ADR"]: [
                    we.eq(0),
                    dev_adr.eq(data),
                    state.eq(states["DATA_LEN"])],
                states["DATA_LEN"]: [
                    data_len.eq(data),
                    state.eq(states["MEM_ADR"])],
                states["MEM_ADR"]: [
                    mem_adr.eq(data),
                    state.eq(states["DATA"])],
                states["DATA"]: [
                    mem_dat.eq(data),
                    mem_adr.eq(mem_adr + 1),
                    data_len.eq(data_len - 1),
                    If(listen,
                        we.eq(1),
                    ),
                    If(data_len == 0,
                        state.eq(states["DEV_ADR"]),
                    )],
                }
        self.sync += If(self.data_in.stb & self.data_in.ack,
                Case(state, actions),
                )


class Comm(Module):
    def __init__(self, pads, dacs, mem=None):
        g = DataFlowGraph()
        if mem is not None:
            mem = list(mem)
            self.submodules.simin = SimFt245r_rx(pads, mem)
            #self.reader = SimReader(mem)
        self.reader = Ft245r_rx(pads)
        pack = Pack(data_layout, 2)
        g.add_connection(self.reader, pack)
        cast = Cast(pack_layout(data_layout, 2), mem_layout)
        g.add_connection(pack, cast)
        self.memwriter = MemWriter(dacs)
        g.add_connection(cast, self.memwriter)
        self.submodules += CompositeActor(g) 

        self.comb += self.memwriter.adr.eq(pads.adr)
        self.comb += pads.reset.eq(self.reader.reset)
        self.comb += pads.go2_out.eq(pads.go2_in) # dummy loop
