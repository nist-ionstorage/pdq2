from migen.fhdl.std import *
from migen.genlib.record import Record
from migen.flow.actor import Source, Sink
from migen.flow.transactions import Token
from migen.actorlib.sim import SimActor
from migen.flow.network import DataFlowGraph, CompositeActor


data_layout = [("data", 8)]


class Ft245r_rx(Module):
    def __init__(self, pads):
        self.data_out = Source(data_layout)

        rxf = Signal()
        rd = Signal()
        self.comb += rxf.eq(~pads.rxfl), pads.rdl.eq(~rd)

        self.busy = Signal()
        self.comb += self.busy.eq(~self.data_out.stb)

        states = "RESET FILL SETUP HOLD".split()
        states = dict((v, i) for i, v in enumerate(states))
        state = Signal(max=len(states))

        t = Signal(max=1<<20)
        self.reset = Signal()

        actions = {
                "RESET": [
                    self.reset.eq(1),
                    If(t < (1<<20)-1,
                        t.eq(t + 1),
                    ).Else(
                        self.reset.eq(0),
                        t.eq(0),
                        state.eq(states["FILL"]),
                    )],
                "FILL": [
                    If(t < 3,
                        t.eq(t + 1),
                    ).Else(
                        If(pads.rd_in, # from master to both
                            t.eq(0),
                            state.eq(states["SETUP"]),
                        ).Elif(rxf, # master only
                            rd.eq(1), # maste ronly
                            pads.rd_out.eq(1), # to both
                        ),
                    )],
                "SETUP": [
                    If(t < 3,
                        t.eq(t + 1),
                    ).Else(
                        self.data_out.payload.data.eq(pads.data),
                        self.data_out.stb.eq(1),
                        rd.eq(0), # master only
                        pads.rd_out.eq(0), # to both
                        t.eq(0),
                        state.eq(states["HOLD"]),
                    )],
                "HOLD": [
                    If(self.data_out.ack,
                        self.data_out.stb.eq(0),
                        state.eq(states["FILL"]),
                    )],
                }
        actions = dict((states[k], _) for k, _ in actions.items())
        self.sync += Case(state, actions)


class Parser(Module):
    def __init__(self, *dacs):
        self.data_in = Sink(data_layout)
        self.busy = Signal()
        self.comb += self.busy.eq(~self.data_in.ack)
        mems = [dac.reader.mem.get_port(write_capable=True) for dac in dacs]
        self.specials += mems

        states = "CMD ARG1 ARG2 PARSE".split()
        states = dict((v, i) for i, v in enumerate(states))
        state = Signal(max=len(states))

        cmds = {
            "DEV_ADR": 0x00,
            "DATA_LENGTH": 0x01,
            "MEM_ADR": 0x02,
            "SINGLE": 0x03,
            "BURST": 0x04,
            "MEM_LENGTH": 0x06,
            "MODE": 0x08,
            "DAC0": 0x10,
            "DAC1": 0x11,
            "DAC2": 0x12,
            "BRANCH0": 0x20,
            "BRANCH1": 0x21,
            "BRANCH2": 0x22,
            "BRANCH3": 0x23,
            "BRANCH4": 0x24,
            "BRANCH5": 0x25,
            "BRANCH6": 0x26,
        }

        cmd = Signal(8)
        arg = Signal(16)
        self.adr = Signal(4)
        dev_adr = Signal(4)
        listen = Signal()
        self.sync += listen.eq((dev_adr == self.adr)
                | (cmd == cmds["DEV_ADR"]))
        length = Signal(16)
        dac_adr = Signal(2)
        mem_adr = Signal(16)
        mem_dat = Signal(16)
        self.comb += [mem.adr.eq(mem_adr[:flen(mem.adr)]) for mem in mems]
        self.comb += [mem.dat_w.eq(mem_dat) for mem in mems]

        we = Array(mem.we for mem in mems)[dac_adr]
        order, freerun, branch_adrs = (Array(_)[dac_adr] for _ in zip(*[
            (dac.reader.order, dac.out.freerun, dac.reader.branch_adrs)
            for dac in dacs]))

        cmd_actions = {
                # officially there are three separate DACX commands.
                # here we trust that DEV_ADDR and the DACX command
                # will come in pairs.
                "DEV_ADR": [dev_adr.eq(arg[8:12]), dac_adr.eq(arg[:2])],
                "DATA_LENGTH": [length.eq(arg)],
                "MEM_ADR": [mem_adr.eq(arg)],
                "MEM_LENGTH": [branch_adrs[7].eq(arg)],
                "MODE": [order.eq(arg[:2]), freerun.eq(~arg[8])],
                "SINGLE": [mem_dat[:16].eq(arg), we.eq(1)],
                "BURST": [mem_dat[:16].eq(arg), we.eq(1),
                    If(length,
                        length.eq(length - 1),
                        state.eq(states["ARG1"]),
                    )],
                }
        for i in range(7):
            cmd_actions["BRANCH{}".format(i)] = [branch_adrs[i].eq(arg)]
        cmd_actions = dict((cmds[k], _) for k, _ in cmd_actions.items())

        pd = self.data_in.payload.data 

        ack_read_next = lambda reg, next: [
                If(we,
                    we.eq(0),
                    mem_adr.eq(mem_adr + 1),
                ),
                # this being only clk/2 max speed does not matter since
                # the source is even slower (clk/50)
                self.data_in.ack.eq(1),
                If(self.data_in.stb & self.data_in.ack,
                    self.data_in.ack.eq(0),
                    reg.eq(pd),
                    state.eq(states[next]),
                )]

        read_actions = {
                "CMD": ack_read_next(cmd, "ARG1"),
                "ARG1": ack_read_next(arg[:8], "ARG2"),
                "ARG2": ack_read_next(arg[8:], "PARSE"),
                "PARSE": [state.eq(states["CMD"]),
                    If(listen,
                        Case(cmd, cmd_actions),
                    )],
                }
        read_actions = dict((states[k], _) for k, _ in read_actions.items())
        self.sync += Case(state, read_actions)


class Comm(Module):
    def __init__(self, pads, *dacs):
        self.reader = Ft245r_rx(pads)
        self.parser = Parser(*dacs)
        g = DataFlowGraph()
        g.add_connection(self.reader, self.parser)
        self.submodules += CompositeActor(g)
        
        self.comb += self.parser.adr.eq(pads.adr)
        self.comb += pads.reset.eq(self.reader.reset)
        self.comb += pads.go2_out.eq(pads.go2_in) # dummy loop


class SimReader(SimActor):
    def __init__(self, data):
        self.data_out = Source(data_layout)
        SimActor.__init__(self, self.data_gen(data))

    def data_gen(self, data):
        for msg in data:
            yield Token("data_out", {"data": msg})


class SimComm(Module):
    def __init__(self, mem, *dacs):
        self.reader = SimReader(mem)
        self.parser = Parser(*dacs)
        g = DataFlowGraph()
        g.add_connection(self.reader, self.parser)
        self.submodules += CompositeActor(g) 
