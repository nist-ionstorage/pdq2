from migen.fhdl.std import *
from migen.genlib.fsm import FSM
from migen.genlib.record import Record
from migen.flow.actor import *
from migen.flow.network import *
from migen.actorlib import spi, sim, structuring


data = [("data", 8)]


class Ft245r_rx(Module):
    def __init__(self, pads):
        self.data_out = Source(data)

        rxf = Signal()
        rd = Signal()
        self.comb += rxf.eq(~pads.rxfl), pads.rdl.eq(~rd)
        self.comb += self.data_out.payload.data.eq(pads.data)

        self.busy = ~self.data_out.stb

        states = "RESET FILL SETUP HOLD BLOCK".split()
        fsm = FSM(*states)
        self.submodules += fsm
        t = Signal(max=1<<20)
        self.reset_out = Signal()

        fsm.act(fsm.RESET,
                If(t < (1<<20)-1,
                    t.eq(t + 1),
                    self.reset_out.eq(1),
                ).Else(
                    t.eq(0),
                    self.reset_out.eq(0),
                    fsm.next_state(fsm.FILL),
                ))
        fsm.act(fsm.FILL,
                If(pads.rd_in,
                    rd.eq(1),
                    fsm.next_state(fsm.SETUP),
                ).Elif(rxf,
                    pads.rd_out.eq(1),
                ))
        fsm.act(fsm.SETUP,
                If(t < 3,
                    t.eq(t + 1),
                ).Else(
                    t.eq(0),
                    #self.data_out.payload.data.eq(pads.data),
                    self.data_out.stb.eq(1),
                    fsm.next_state(fsm.HOLD),
                ))
        fsm.act(fsm.HOLD,
                If(t < 1,
                    t.eq(t + 1),
                ).Elif(self.data_out.ack,
                    t.eq(0),
                    rd.eq(0),
                    pads.rd_out.eq(0),
                    self.data_out.stb.eq(0),
                    fsm.next_state(fsm.BLOCK),
                ))
        fsm.act(fsm.BLOCK,
                If(t < 3,
                    t.eq(t + 1),
                ).Else(
                    t.eq(0),
                    fsm.next_state(fsm.FILL),
                ))


class Parser(Module):
    def __init__(self, pads, *dacs):
        self.data_in = Sink(data)
        self.busy = ~self.data_in.ack
        orders = Array(dac.reader.order for dac in dacs)
        freeruns = Array(dac.out.freerun for dac in dacs)
        branch_adrs = Array(dac.reader.branch_adr for dac in dacs)
        mems = [dac.reader.mem.get_port(write_capable=True) for dac in dacs]
        self.specials += mems

        states = "CMD ARG1 ARG2".split()
        fsm = FSM(*states)
        self.submodules += fsm

        cmds = ("DEV_ADR DATA_LENGTH MEM_ADR SINGLE BURST _5 MEM_LENGTH "
            "_7 MODE _9 DAC0 DAC1 DAC2 _13 _14 _15 _16 _17 _18 _19 "
            "BRANCH0 BRANCH1 BRANCH2 BRANCH3 BRANCH4 BRANCH5 BRANCH6").split()
        assert len(cmds) == 27, len(cmds)
        code_to_cmd = dict((code, cmd) for code, cmd in enumerate(cmds))

        cmd = Signal(8)
        arg = Signal(16)
        listen = Signal()
        
        length = Signal(13)
        dac_adr = Signal(2)
        mem_adr = Signal(13)
        mem_dat = Signal(16)
        self.comb += [mem.adr.eq(mem_adr) for mem in mems]
        self.comb += [mem.dat_w.eq(mem_dat) for mem in mems]
        wes = Array(mem.we for mem in mems)

        pd = self.data_in.payload.data 

        arg_actions = {
                cmds.index("DEV_ADR"): [
                    listen.eq(~arg[8:12] == pads.adr),
                    # officially there are three separate DACX commands.
                    # here we trust that DEV_ADDR and the DACX command
                    # will come in pairs.
                    dac_adr.eq(arg[:2]),
                    ],
                cmds.index("DATA_LENGTH"): [length.eq(arg[:13])],
                cmds.index("MEM_ADR"): [mem_adr.eq(arg[:13])],
                cmds.index("MEM_LENGTH"): [branch_adrs[dac_adr][7].eq(arg[:13])],
                cmds.index("MODE"): [
                    orders[dac_adr].eq(arg[8:10]),
                    freeruns[dac_adr].eq(~arg[0]),
                    ],
                cmds.index("SINGLE"): [
                    mem_dat.eq(arg),
                    wes[dac_adr].eq(1),
                    ],
                cmds.index("BURST"): [
                    mem_dat.eq(arg),
                    wes[dac_adr].eq(1),
                    mem_adr.eq(mem_adr + 1),
                    If(length,
                        length.eq(length - 1),
                        fsm.next_state(fsm.ARG1),
                    ),
                    ],
                }
        for i in range(7):
            arg_actions[cmds.index("BRANCH{}".format(i))] = [
                    branch_adrs[dac_adr][i].eq(arg[:13]),
                    ]

        fsm.act(fsm.CMD,
                self.data_in.ack.eq(1),
                If(self.data_in.stb,
                    cmd.eq(pd),
                    self.data_in.ack.eq(0),
                    fsm.next_state(fsm.ARG1),
                    wes[dac_adr].eq(0),
                ))
        fsm.act(fsm.ARG1,
                self.data_in.ack.eq(1),
                If(self.data_in.stb,
                    self.data_in.ack.eq(0),
                    arg[8:].eq(pd),
                    fsm.next_state(fsm.ARG2),
                    wes[dac_adr].eq(0),
                ))
        fsm.act(fsm.ARG2,
                self.data_in.ack.eq(1),
                If(self.data_in.stb,
                    arg[:8].eq(pd),
                    self.data_in.ack.eq(0),
                    fsm.next_state(fsm.CMD),
                    If(listen | cmd == cmds.index("DEV_ADR"),
                        Case(cmd, arg_actions),
                    ),
                ))



class Comm(Module):
    def __init__(self, pads, *dacs):
        self.reader = Ft245r_rx(pads)
        self.parser = Parser(pads, *dacs)
        g = DataFlowGraph()
        g.add_connection(self.reader, self.parser)
        self.submodules += CompositeActor(g)
        
        self.comb += pads.reset.eq(self.reader.reset_out)
        self.comb += pads.go2_out.eq(0)
