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

        cmds = ("DEV_ADR DATA_LENGTH MEM_ADR SINGLE BURST _5 MEM_LENGTH "
            "_7 MODE _9 DAC0 DAC1 DAC2 _13 _14 _15 _16 _17 _18 _19 "
            "BRANCH0 BRANCH1 BRANCH2 BRANCH3 BRANCH4 BRANCH5 BRANCH6").split()
        assert len(cmds) == 27, len(cmds)
        code_to_cmd = dict((code, cmd) for code, cmd in enumerate(cmds))

        cmd = Signal(8)
        arg = Signal(16)
        listen = Signal()
        self.adr = Signal(4)
        
        dev_adr = Signal(4)
        self.sync += listen.eq((dev_adr == self.adr)
                | (cmd == cmds.index("DEV_ADR")))
        length = Signal(13)
        length1 = Signal(13)
        self.sync += length.eq(length1)
        dac_adr = Signal(2)
        dac_adr1 = Signal(2)
        self.sync += dac_adr.eq(dac_adr1)
        adr = Signal(13)
        adr_inc = Signal()
        mem_adr = Signal(13)
        self.sync += mem_adr.eq(adr)
        mem_dat = Signal(18)
        self.comb += [mem.adr.eq(mem_adr) for mem in mems]
        self.comb += [mem.dat_w.eq(mem_dat) for mem in mems]
        we = Array(mem.we for mem in mems)[dac_adr]
        #mem_adr, mem_dat, we = (Array(_)[dac_adr] for _ in zip(*[
        #    (mem.adr, mem.dat_w, mem.we) for mem in mems]))
        order, freerun, branch_adrs = (Array(_)[dac_adr] for _ in zip(*[
            (dac.reader.order, dac.out.freerun, dac.reader.branch_adrs)
            for dac in dacs]))

        pd = self.data_in.payload.data 

        actions = {
                cmds.index("DEV_ADR"): [
                    dev_adr.eq(~arg[8:12]),
                    # officially there are three separate DACX commands.
                    # here we trust that DEV_ADDR and the DACX command
                    # will come in pairs.
                    dac_adr1.eq(arg[:2]),
                    ],
                cmds.index("DATA_LENGTH"): [length1.eq(arg[:13])],
                cmds.index("MEM_ADR"): [adr.eq(arg[:13])],
                cmds.index("MEM_LENGTH"): [branch_adrs[7].eq(arg[:13])],
                cmds.index("MODE"): [order.eq(arg[8:10]), freerun.eq(~arg[0])],
                cmds.index("SINGLE"): [
                    mem_dat[:16].eq(arg),
                    we.eq(1),
                    adr_inc.eq(1),
                    ],
                cmds.index("BURST"): [
                    mem_dat[:16].eq(arg),
                    we.eq(1),
                    adr_inc.eq(1),
                    If(length,
                        length1.eq(length - 1),
                        state.eq(states.index("ARG1")),
                    ),
                    ],
                }
        for i in range(7):
            actions[cmds.index("BRANCH{}".format(i))] = [
                    branch_adrs[i].eq(arg[:13]),
                    ]

        read_actions = {}
        for i, cstate in enumerate(states[:-1]):
            read_actions[i] = [
                self.data_in.ack.eq(0),
                If(self.data_in.stb,
                    self.data_in.ack.eq(1),
                    state.eq(i+1),
                  ),
                ]

        read_actions[states.index("CMD")].extend((
                adr_inc.eq(0),
                If(self.data_in.stb,
                    cmd.eq(pd),
                ),
                ))
        read_actions[states.index("ARG1")].extend((
                we.eq(0),
                adr.eq(mem_adr + adr_inc),
                If(self.data_in.stb,
                    arg[8:].eq(pd),
                ),
                ))
        read_actions[states.index("ARG2")].extend((
                If(self.data_in.stb,
                    arg[:8].eq(pd),
                ),
                ))
        read_actions[states.index("PARSE")] = (
                state.eq(states.index("CMD")),
                If(listen,
                    Case(cmd, actions),
                ),
                )
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
