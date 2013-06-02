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
        mems = [dac.reader.mem.get_port(write_capable=True,
                we_granularity=8) for dac in dacs]
        self.specials += mems

        states = "CMD ARG1 ARG2 DATA PARSE".split()
        fsm = FSM(*states)
        self.submodules += fsm

        cmds = ("DEV_ADDR DATA_LENGTH MEM_ADDR SINGLE BURST _5 MEM_LENGTH "
            "_7 MODE _9 DAC0 DAC1 DAC2 _13 _14 _15 _16 _17 _18 _19 "
            "BRANCH0 BRANCH1 BRANCH2 BRANCH3 BRANCH4 BRANCH5 BRANCH6").split()
        assert len(cmds) == 27, len(cmds)
        code_to_cmd = dict((code, cmd) for code, cmd in enumerate(cmds))

        cmd = Signal(8)
        arg1 = Signal(8)
        arg2 = Signal(8)
        length = Signal(16)
        read = Signal(16)
        
        pd = self.data_in.payload.data 

        actions = {}

        fsm.act(fsm.CMD,
                self.data_in.ack.eq(1),
                If(self.data_in.stb,
                    cmd.eq(pd),
                    self.data_in.ack.eq(0),
                    If((pd == cmds.index("BURST")) |
                       (pd == cmds.index("SINGLE")),
                        fsm.next_state(fsm.DATA),
                    ).Else(
                        fsm.next_state(fsm.ARG1),
                    ),
                ))
        fsm.act(fsm.ARG1,
                self.data_in.ack.eq(1),
                If(self.data_in.stb,
                    self.data_in.ack.eq(0),
                    arg1.eq(pd),
                    fsm.next_state(fsm.ARG2),
                ))
        fsm.act(fsm.ARG2,
                self.data_in.ack.eq(1),
                If(self.data_in.stb,
                    arg2.eq(pd),
                    self.data_in.ack.eq(0),
                    fsm.next_state(fsm.CMD),
                    Case(cmd, actions),
                ))
        fsm.act(fsm.DATA,
                self.data_in.ack.eq(1),
                If(self.data_in.stb,
                    If(read < length,
                        read.eq(read + 1),
                        fsm.next_state(fsm.DATA),
                    ).Else(
                        read.eq(0),
                        fsm.next_state(fsm.CMD),
                    ),
                    Case(cmd, actions),
                    self.data_in.ack.eq(0),
                ))



class Comm(Module):
    def __init__(self, pads, *dacs):
        self.reader = Ft245r_rx(pads)
        self.parser = Parser(pads, *dacs)
        g = DataFlowGraph()
        g.add_connection(self.reader, self.parser)
        self.submodules += CompositeActor(g)
        
        self.comb += pads.reset.eq(self.reader.reset_out)
