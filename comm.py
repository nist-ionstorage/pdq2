# Robert Jordens <jordens@gmail.com> 2013

from migen.fhdl.std import *
from migen.flow.actor import Source, Sink
from migen.flow.transactions import Token
from migen.actorlib.sim import SimActor
from migen.actorlib.structuring import Cast, Pack, pack_layout
from migen.genlib.fsm import FSM, NextState

from escape import Unescaper
from ft245r import data_layout, SimFt245r_rx, Ft245r_rx


class SimReader(SimActor):
    def __init__(self, data):
        self.source = Source(data_layout)
        SimActor.__init__(self, self.data_gen(data))

    def data_gen(self, data):
        for msg in data:
            yield Token("data_out", {"data": msg})


mem_layout = [("data", 16)]


class MemWriter(Module):
    def __init__(self, adr, dacs):
        self.sink = Sink(mem_layout)

        mems = [dac.parser.mem.get_port(write_capable=True) for dac in dacs]
        self.specials += mems

        dac = Signal(max=len(dacs))
        mem_adr = Signal(16)
        mem_end = Signal(16)
        listen = Signal()
        we = Signal()
        pd = self.sink.payload.data

        self.sink.ack.reset = 1

        self.comb += [
                Array(mem.we for mem in mems)[dac].eq(we)
        ]
        for mem in mems:
            self.comb += [
                    mem.adr.eq(mem_adr),
                    mem.dat_w.eq(pd)
            ]

        self.submodules.fsm = fsm = FSM(reset_state="DEV_ADR")
        fsm.act("DEV_ADR",
                If(self.sink.stb,
                    NextState("MEM_ADR")
                )
        )
        fsm.act("MEM_ADR",
                If(self.sink.stb,
                    NextState("MEM_END")
                )
        )
        fsm.act("MEM_END",
                If(self.sink.stb,
                    NextState("DATA")
                )
        )
        fsm.act("DATA",
                If(self.sink.stb,
                    we.eq(listen),
                    If(mem_adr == mem_end,
                        NextState("DEV_ADR")
                    )
                )
        )

        self.sync += [
                If(fsm.ongoing("DEV_ADR"),
                    listen.eq(pd[:flen(adr)] == adr),
                    dac.eq(pd[8:])
                ),
                If(fsm.ongoing("MEM_ADR"),
                    mem_adr.eq(pd)
                ),
                If(fsm.ongoing("MEM_END"),
                    mem_end.eq(pd)
                ),
                If(fsm.ongoing("DATA"),
                    If(self.sink.stb,
                        mem_adr.eq(mem_adr + 1),
                    )
                )
        ]


class Ctrl(Module):
    def __init__(self, pads, dacs):
        self.reset = Signal()
        self.trigger = Signal()
        self.arm = Signal()

        self.sink = Sink(data_layout)

        self.sink.ack.reset = 1
        self.comb += pads.reset.eq(self.reset)

        # two stage synchronizer for inputs
        i0 = Signal(flen(pads.interrupt))
        t0 = Signal()

        self.sync += i0.eq(pads.interrupt), t0.eq(pads.trigger)

        for dac in dacs:
            self.sync += [
                    dac.parser.interrupt.eq(i0),
                    dac.out.trigger.eq(t0 | self.trigger),
                    dac.parser.arm.eq(self.arm),
                    ]

        self.comb += pads.aux.eq(Cat(*(dac.out.aux for dac in dacs)) != 0)
        self.comb += pads.go2_out.eq(pads.go2_in) # dummy loop
        #self.comb += pads.go2_out.eq(Cat(*(~dac.out.busy for dac in
        #    dacs)) != 0)

        commands = {
                "RESET_EN":    (0x00, [self.reset.eq(1)]),
                "RESET_DIS":   (0x01, [self.reset.eq(0)]),
                "TRIGGER_EN":  (0x02, [self.trigger.eq(1)]),
                "TRIGGER_DIS": (0x03, [self.trigger.eq(0)]),
                "ARM_EN":      (0x04, [self.arm.eq(1)]),
                "ARM_DIS":     (0x05, [self.arm.eq(0)]),
                }
        self.sync += If(self.sink.stb, Case(self.sink.payload.data,
            dict((code, act) for name, (code, act) in commands.items())))


class Comm(Module):
    def __init__(self, pads, dacs, mem=None):
        if mem is not None:
            #reader = SimReader(mem)
            simin = SimFt245r_rx(pads, mem)
            self.submodules += simin
        reader = Ft245r_rx(pads)
        unescaper = Unescaper(data_layout, 0xa5)
        pack = Pack(data_layout, 2)
        cast = Cast(pack_layout(data_layout, 2), mem_layout)
        memwriter = MemWriter(~pads.adr, dacs) # adr is active low
        self.ctrl = ctrl = Ctrl(pads, dacs)
        self.submodules += reader, unescaper, pack, cast, memwriter, ctrl

        self.comb += [
                unescaper.sink.connect(reader.source),
                pack.sink.connect(unescaper.source_a),
                cast.sink.connect(pack.source),
                memwriter.sink.connect(cast.source),
                ctrl.sink.connect(unescaper.source_b),
        ]
