from migen.fhdl.std import *
from migen.genlib.fsm import FSM
from migen.genlib.record import Record
from migen.flow.actor import *
from migen.flow.transactions import *
from migen.flow.network import *
from migen.actorlib import sim


data_layout = [("data", 8)]


class Ft245r_rx(Module):
    def __init__(self, pads):
        self.data_out = Source(data_layout)

        rxf = Signal()
        rd = Signal()
        self.comb += rxf.eq(~pads.rxfl), pads.rdl.eq(~rd)
        #self.comb += self.data_out.payload.data.eq(pads.data)

        self.busy = ~self.data_out.stb

        states = "RESET FILL SETUP HOLD".split()
        fsm = FSM(*states)
        self.submodules += fsm
        t = Signal(max=1<<20)
        t1 = Signal(max=1<<20)
        self.sync += t.eq(t1)
        self.reset = Signal()

        fsm.act(fsm.RESET,
                self.reset.eq(1),
                If(t < (1<<20)-1,
                    t1.eq(t + 1),
                ).Else(
                    self.reset.eq(0),
                    t1.eq(0),
                    fsm.next_state(fsm.FILL),
                ))
        fsm.act(fsm.FILL,
                If(t < 3,
                    t1.eq(t + 1),
                ).Else(
                    If(pads.rd_in,
                        t1.eq(0),
                        fsm.next_state(fsm.SETUP),
                    ).Elif(rxf,
                        rd.eq(1),
                        pads.rd_out.eq(1),
                    ),
                ))
        fsm.act(fsm.SETUP,
                If(t < 3,
                    t1.eq(t + 1),
                ).Else(
                    self.data_out.payload.data.eq(pads.data),
                    self.data_out.stb.eq(1),
                    rd.eq(0),
                    pads.rd_out.eq(0),
                    t1.eq(0),
                    fsm.next_state(fsm.HOLD),
                ))
        fsm.act(fsm.HOLD,
                If(self.data_out.ack,
                    self.data_out.stb.eq(0),
                    fsm.next_state(fsm.FILL),
                ))


class Parser(Module):
    def __init__(self, *dacs):
        self.data_in = Sink(data_layout)
        self.busy = ~self.data_in.ack
        mems = [dac.reader.mem.get_port(write_capable=True) for dac in dacs]
        self.specials += mems

        states = "CMD ARG1 ARG2 PARSE".split()
        #fsm = FSM(*states)
        #self.submodules += fsm
        state = Signal(2)

        cmd_codes = {
            "DEV_ADR": 0x00,
            "DATA_LENGTH": 0x01,
            "MEM_ADR": 0x02,
            "SINGLE": 0x03,
            "BURST": 0x04,
            "MEM_LENGTH": 0x06,
            "MODE": 0x08,
            "DAC0": 0x10,
            "DAC0": 0x11,
            "DAC0": 0x12,
            "BRANCH0": 0x20,
            "BRANCH1": 0x21,
            "BRANCH2": 0x22,
            "BRANCH3": 0x23,
            "BRANCH4": 0x24,
            "BRANCH5": 0x25,
            "BRANCH6": 0x26,
        }
        class Dummy(object):
            pass
        cmds = Dummy()
        for k, v in cmd_codes.items():
            setattr(cmds, k, v)

        cmd = Signal(8)
        arg = Signal(16)
        listen = Signal()
        self.adr = Signal(4)
        
        dev_adr = Signal(4)
        self.sync += listen.eq((dev_adr == self.adr)
                | (cmd == cmds.DEV_ADR))
        length = Signal(13)
        dac_adr = Signal(2)
        mem_adr = Signal(13)
        mem_dat = Signal(16)
        self.comb += [mem.adr.eq(mem_adr) for mem in mems]
        self.comb += [mem.dat_w.eq(mem_dat) for mem in mems]
        we = Array(mem.we for mem in mems)[dac_adr]
        #mem_adr, mem_dat, we = (Array(_)[dac_adr] for _ in zip(*[
        #    (mem.adr, mem.dat_w, mem.we) for mem in mems]))
        order, freerun, branch_adrs = (Array(_)[dac_adr] for _ in zip(*[
            (dac.reader.order, dac.out.freerun, dac.reader.branch_adrs)
            for dac in dacs]))

        actions = {
                cmds.DEV_ADR: [
                    dev_adr.eq(arg[8:12]),
                    # officially there are three separate DACX commands.
                    # here we trust that DEV_ADDR and the DACX command
                    # will come in pairs.
                    dac_adr.eq(arg[:2]),
                    ],
                cmds.DATA_LENGTH: [length.eq(arg[:13])],
                cmds.MEM_ADR: [mem_adr.eq(arg[:13])],
                cmds.MEM_LENGTH: [branch_adrs[7].eq(arg[:13])],
                cmds.MODE: [order.eq(arg[8:10]), freerun.eq(~arg[0])],
                cmds.SINGLE: [
                    mem_dat[:16].eq(arg),
                    we.eq(1),
                    mem_adr.eq(mem_adr + 1),
                    ],
                cmds.BURST: [
                    mem_dat[:16].eq(arg),
                    we.eq(1),
                    mem_adr.eq(mem_adr + 1),
                    If(length,
                        length.eq(length - 1),
                        state.eq(states.index("ARG1")),
                    ),
                    ],
                }
        for i in range(7):
            actions[cmd_codes["BRANCH{}".format(i)]] = [
                    branch_adrs[i].eq(arg[:13]),
                    ]

        pd = self.data_in.payload.data 

        read_actions = {
            states.index("CMD"): [
                we.eq(0),
                self.data_in.ack.eq(1),
                If(self.data_in.stb & self.data_in.ack,
                    self.data_in.ack.eq(0),
                    cmd.eq(pd),
                    state.eq(states.index("ARG1")),
                ),
                ],
            states.index("ARG1"): [
                we.eq(0),
                self.data_in.ack.eq(1),
                If(self.data_in.stb & self.data_in.ack,
                    self.data_in.ack.eq(0),
                    arg[:8].eq(pd),
                    state.eq(states.index("ARG2")),
                ),
                ],
            states.index("ARG2"): [
                self.data_in.ack.eq(1),
                If(self.data_in.stb & self.data_in.ack,
                    self.data_in.ack.eq(0),
                    arg[8:].eq(pd),
                    state.eq(states.index("PARSE")),
                ),
                ],
            states.index("PARSE"): [
                state.eq(states.index("CMD")),
                If(listen,
                    Case(cmd, actions),
                ),
                ],
        }
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
        self.comb += pads.go2_out.eq(0)


class SimReader(sim.SimActor):
    def __init__(self, data):
        self.data_out = Source(data_layout)
        sim.SimActor.__init__(self, self.data_gen(data))

    def data_gen(self, data):
        for msg in data:
            print("gen {}".format(repr(msg)))
            yield Token("data_out", {"data": msg})


class SimComm(Module):
    def __init__(self, mem, *dacs):
        self.reader = SimReader(mem)
        self.parser = Parser(*dacs)
        g = DataFlowGraph()
        g.add_connection(self.reader, self.parser)
        self.submodules += CompositeActor(g) 
