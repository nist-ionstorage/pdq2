# Robert Jordens <jordens@gmail.com> 2013

from migen.fhdl.std import *
from migen.genlib.record import Record
from migen.genlib.misc import optree
from migen.flow.actor import Source, Sink
from migen.flow.plumbing import Multiplexer
from migen.genlib.cordic import Cordic
from migen.actorlib.fifo import SyncFIFO
from migen.genlib.fsm import FSM, NextState


line_layout = [
        ("header", [
            ("length", 4), # length in shorts
            ("typ", 2), # volt, dds
            ("wait", 1), # wait for trigger before
            ("silence", 1), # shut down clock during
            ("aux", 1), # aux channel value
            ("shift", 4), # time shift
            ("end", 1), # return to jump table afterwards
            ("reserved", 2),
        ]),
        ("dt", 16),
        ("data", 14*16),
]


class Parser(Module):
    def __init__(self, mem_depth=4*(1<<10)): # XC3S500E: 20x18bx1024
        self.specials.mem = Memory(width=16, depth=mem_depth)
        self.specials.read = read = self.mem.get_port()

        self.source = Source(line_layout)
        self.arm = Signal()
        self.frame = Signal(3)

        ###

        frame_length = Signal(flen(read.dat_r))
        frame_read = Signal(flen(read.dat_r))
        adr = Signal(flen(read.adr))
        inc = Signal()

        lp = self.source.payload
        raw = Signal(flen(lp.raw_bits()))
        self.comb += lp.raw_bits().eq(raw)
        lpa = Array([raw[i:i + flen(read.dat_r)] for i in
            range(0, flen(raw), flen(read.dat_r))])
        data_read = Signal(flen(lp.header.length))

        self.submodules.fsm = fsm = FSM(reset_state="JUMP")
        fsm.act("JUMP",
                read.adr.eq(self.frame),
                If(self.arm,
                    NextState("FRAME")
                )
        )
        fsm.act("FRAME",
                read.adr.eq(read.dat_r),
                inc.eq(1),
                If(read.dat_r == 0,
                    NextState("JUMP")
                ).Else(
                    NextState("HEADER")
                )
        )
        fsm.act("HEADER",
                read.adr.eq(adr),
                inc.eq(1),
                NextState("LINE")
        )
        fsm.act("LINE",
                read.adr.eq(adr),
                If(data_read == lp.header.length,
                    NextState("STB")
                ).Else(
                    inc.eq(1),
                )
        )
        fsm.act("STB",
                read.adr.eq(adr),
                self.source.stb.eq(1),
                If(self.source.ack,
                    inc.eq(1),
                    If(lp.header.end,
                        NextState("JUMP")
                    ).Else(
                        NextState("HEADER")
                    )
                ),
                If(~self.arm,
                    NextState("JUMP")
                )
        )

        self.sync += [
                If(inc,
                    adr.eq(read.adr + 1),
                ),
                If(fsm.ongoing("HEADER"),
                    raw.eq(read.dat_r),
                    data_read.eq(1),
                ),
                If(fsm.ongoing("LINE"),
                    lpa[data_read].eq(read.dat_r),
                    data_read.eq(data_read + 1),
                )
        ]


class DacOut(Module):
    def __init__(self):
        self.sink = Sink(line_layout)

        self.trigger = Signal()
        self.aux = Signal()
        self.silence = Signal()
        self.data = Signal(16)

        ###

        line = Record(line_layout)
        dt_dec = Signal(16)
        dt_end = Signal(16)
        dt = Signal(16)
        adv = Signal()
        tic = Signal()
        toc = Signal()
        stb = Signal()
        toc0 = Signal()
        inc = Signal()

        lp = self.sink.payload

        self.comb += [
                self.aux.eq(line.header.aux),
                self.silence.eq(line.header.silence),
                adv.eq(self.trigger | (self.sink.stb & ~lp.header.wait)),
                tic.eq(dt_dec == dt_end),
                toc.eq(dt == line.dt),
                self.sink.ack.eq(tic & toc & adv),
                If(tic,
                    If(~toc,
                        inc.eq(1)
                    ).Elif(adv,
                        inc.eq(1)
                    ).Elif(~toc0,
                        inc.eq(1)
                    )
                ),
                #inc.eq(tic & ~(toc & (adv | toc0))),
                stb.eq(self.sink.stb & self.sink.ack),
        ]

        subs = [
                Volt(lp.data, stb & (lp.header.typ == 0), inc),
                Dds(lp.data, stb & (lp.header.typ == 1), inc),
        ]

        for i, sub in enumerate(subs):
            self.submodules += sub

        self.sync += [
                toc0.eq(toc),
                self.data.eq(optree("+", [sub.data for sub in subs])),

                If(~tic,
                    dt_dec.eq(dt_dec + 1),
                ).Elif(~toc,
                    dt_dec.eq(0),
                    dt.eq(dt + 1),
                ).Elif(stb,
                    line.header.eq(lp.header),
                    line.dt.eq(lp.dt - 1),
                    dt_end.eq((1<<line.header.shift) - 1),
                    dt_dec.eq(0),
                    dt.eq(0),
                )
        ]


class Volt(Module):
    def __init__(self, data, stb, inc):
        self.data = Signal(16)

        ###

        v = [Signal(48) for i in range(4)] # amp, damp, ddamp, dddamp
        self.comb += self.data.eq(v[0][32:])

        self.sync += [
                If(inc,
                    v[0].eq(v[0] + v[1]),
                    v[1].eq(v[1] + v[2]),
                    v[2].eq(v[2] + v[3]),
                ),
                If(stb,
                    v[0].eq(0),
                    v[1].eq(0),
                    Cat(v[0][32:], v[1][16:], v[2], v[3]).eq(data),
                )
        ]


class Dds(Module):
    def __init__(self, data, stb, inc):
        self.data = Signal(16)

        ###

        self.submodules.cordic = Cordic(width=16, eval_mode="pipelined")

        z = [Signal(32) for i in range(3)] # phase, dphase, ddphase
        x = [Signal(48) for i in range(4)] # amp, damp, ddamp, dddamp
        self.comb += [
                self.cordic.xi.eq(x[0][32:]),
                self.cordic.yi.eq(0),
                self.cordic.zi.eq(z[0][16:]),
                self.data.eq(self.cordic.xo),
                ]
        self.sync += [
                z[0].eq(z[0] + z[1]),
                If(inc,
                    x[0].eq(x[0] + x[1]),
                    x[1].eq(x[1] + x[2]),
                    x[2].eq(x[2] + x[3]),
                    z[1].eq(z[1] + z[2]),
                ),
                If(stb,
                    z[0].eq(0),
                    x[0].eq(0),
                    x[1].eq(0),
                    Cat(x[0][32:], x[1][16:], x[2], x[3], z[0][16:], z[1], z[2]
                        ).eq(data),
                )
        ]


class Dac(Module):
    def __init__(self, fifo=8, **kwargs):
        self.submodules.parser = Parser(**kwargs)
        self.submodules.out = DacOut()
        if fifo:
            self.submodules.fifo = SyncFIFO(line_layout, fifo)
            self.comb += [
                    self.fifo.sink.connect(self.parser.source),
                    self.out.sink.connect(self.fifo.source),
            ]
        else:
            self.comb += self.out.sink.connect(self.parser.source)


class TB(Module):
    def __init__(self, mem=None):
        self.submodules.dac = Dac()
        if mem is not None:
            self.dac.parser.mem.init = mem
        self.outputs = []

    def do_simulation(self, selfp):
        self.outputs.append(selfp.dac.out.data)
        if selfp.simulator.cycle_counter == 5:
            selfp.dac.parser.arm = 1
        #    selfp.dac.parser.frame = 0
        elif selfp.simulator.cycle_counter == 40:
            selfp.dac.out.trigger = 1
        elif selfp.simulator.cycle_counter == 41:
            selfp.dac.out.trigger = 0
        elif selfp.simulator.cycle_counter == 200:
            selfp.dac.out.trigger = 1
        elif selfp.simulator.cycle_counter == 201:
            selfp.dac.out.trigger = 0

        #if selfp.simulator.cycle_counter == 200:
        #    self.dac.parser.frame = 0
        if (selfp.dac.out.sink.ack and
                selfp.dac.out.sink.stb):
            print("cycle {} data {}".format(
                selfp.simulator.cycle_counter,
                selfp.dac.out.data))


def _main():
    from migen.fhdl import verilog
    from migen.sim.generic import run_simulation
    from matplotlib import pyplot as plt
    import numpy as np
    from scipy import interpolate

    from host import pdq
    pdq.Ftdi = pdq.FileFtdi

    #print(verilog.convert(Dac()))

    t = np.arange(0, 6) * .22e-6
    v = 9*(1-np.cos(t/t[-1]*np.pi))/2
    p = pdq.Pdq()
    k = 4
    mem = p.map_frames([bytes(p.frame(t, v, order=k).data)])
    tb = TB(list(np.fromstring(mem, "<u2")))
    run_simulation(tb, ncycles=300, vcd_name="dac.vcd")

    plt.plot(t, v, "xk")

    sp = interpolate.splrep(t, v, k=k-1)
    tt = np.arange(t[0], t[-1], 1/p.freq)
    vv = interpolate.splev(tt, sp, der=0)
    plt.plot(tt, vv, ",g")

    out = np.array(tb.outputs, np.uint16).view(np.int16)*20./(1<<16)
    tim = np.arange(out.shape[0])/p.freq
    plt.plot(tim - 43/p.freq, out, "-r")
    plt.show()


if __name__ == "__main__":
    _main()
