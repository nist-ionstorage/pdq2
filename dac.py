# Robert Jordens <jordens@gmail.com> 2013

from migen.fhdl.std import *
from migen.genlib.record import Record
from migen.genlib.misc import optree
from migen.flow.actor import Source, Sink
from migen.genlib.cordic import Cordic
from migen.actorlib.fifo import SyncFIFO
from migen.genlib.fsm import FSM, NextState


line_layout = [
        ("header", [
            ("length", 4), # length in shorts
            ("typ", 2), # volt, dds
            ("wait", 1), # wait for trigger before
            ("silence", 1), # shut down clock during
            ("aux", 1), # aux channel
            ("shift", 4), # time shift
            ("end", 1), # return to jump table afterwards
            ("reserved", 2),
        ]),
        ("dt", 16),
        ("data", 14*16),
        ]


class Parser(Module):
    def __init__(self, mem_depth=4*(1<<10)): # XC3S500E: 20BRAMS 18bits 1024loc
        self.specials.mem = Memory(width=16, depth=mem_depth)
        self.specials.read = read = self.mem.get_port()

        self.line_out = Source(line_layout)
        self.arm = Signal()
        self.interrupt = Signal(4)

        frame_length = Signal(flen(read.dat_r))
        frame_read = Signal(flen(read.dat_r))
        adr = Signal(flen(read.adr))
        inc = Signal()

        lp = self.line_out.payload
        raw = Signal(flen(lp.raw_bits()))
        self.comb += lp.raw_bits().eq(raw)
        lpa = Array([raw[i:i + flen(read.dat_r)] for i in
            range(0, flen(raw), flen(read.dat_r))])
        data_read = Signal(max=len(lpa))

        self.submodules.fsm = fsm = FSM(reset_state="IRQ")
        fsm.act("IRQ",
                read.adr.eq(self.interrupt),
                If(self.arm,
                    NextState("FRAME")
                )
        )
        fsm.act("FRAME",
                read.adr.eq(read.dat_r),
                inc.eq(1),
                If(read.dat_r == 0,
                    NextState("IRQ")
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
                self.line_out.stb.eq(1),
                If(self.line_out.ack,
                    inc.eq(1),
                    If(lp.header.end,
                        NextState("IRQ")
                    ).Else(
                        NextState("HEADER")
                    )
                ),
                If(~self.arm,
                    NextState("IRQ")
                )
        )

        self.sync += [
                adr.eq(read.adr + inc),
                If(fsm.ongoing("HEADER"),
                    raw.eq(read.dat_r),
                    data_read.eq(1),
                ),
                If(fsm.ongoing("LINE"),
                    lpa[data_read].eq(read.dat_r),
                    data_read.eq(data_read + 1),
                ),
        ]


class DacOut(Module):
    def __init__(self):
        self.line_in = Sink(line_layout)

        self.trigger = Signal()
        self.aux = Signal()
        self.silence = Signal()
        self.data = Signal(16)

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

        lp = self.line_in.payload

        self.comb += [
                self.aux.eq(line.header.aux),
                self.silence.eq(line.header.silence),
                adv.eq(self.trigger | (self.line_in.stb & ~lp.header.wait)),
                tic.eq(dt_dec == dt_end),
                toc.eq(dt == line.dt),
                self.line_in.ack.eq(tic & toc & adv),
                inc.eq(tic & ~(toc & (adv | toc0))),
                stb.eq(self.line_in.stb & self.line_in.ack),
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
    def __init__(self, data, stb, tic):
        self.data = Signal(16)

        v = [Signal(48) for i in range(4)] # amp, damp, ddamp, dddamp
        self.comb += self.data.eq(v[0][32:])

        self.sync += [
                If(tic,
                    v[0].eq(v[0] + v[1]),
                    v[1].eq(v[1] + v[2]),
                    v[2].eq(v[2] + v[3]),
                ),
                If(stb,
                    v[0].eq(0),
                    v[1].eq(0),
                    Cat(v[0][32:], v[1][16:], v[2], v[3]).eq(data),
                )]


class Dds(Module):
    def __init__(self, data, stb, tic):
        self.submodules.cordic = Cordic(width=16, guard=None,
                eval_mode="pipelined", cordic_mode="rotate",
                func_mode="circular")
        self.data = Signal(16)

        z = [Signal(48) for i in range(3)] # phase, dphase, ddphase
        x = [Signal(48) for i in range(3)] # amp, damp, ddamp
        self.comb += [
                self.cordic.xi.eq(x[0][32:]),
                self.cordic.yi.eq(0),
                self.cordic.zi.eq(z[0][32:]),
                self.data.eq(self.cordic.xo),
                ]
        self.sync += [
                If(tic,
                    z[0].eq(z[0] + z[1]),
                    z[1].eq(z[1] + z[2]),
                    x[0].eq(x[0] + x[1]),
                    x[1].eq(x[1] + x[2]),
                ),
                If(stb,
                    z[0].eq(0),
                    z[1].eq(0),
                    x[0].eq(0),
                    x[1].eq(0),
                    Cat(z[0][32:], x[0][32:], z[1][16:], x[1][16:], z[2], x[2]).eq(data),
                )]


class Dac(Module):
    def __init__(self, fifo=0, **kwargs):
        self.submodules.parser = Parser(**kwargs)
        self.submodules.out = DacOut()
        if fifo:
            self.submodules.fifo = SyncFIFO(line_layout, fifo)
            self.comb += [
                    self.fifo.sink.connect(self.parser.line_out),
                    self.out.line_in.connect(self.fifo.source),
            ]
        else:
            self.comb += self.out.line_in.connect(self.parser.line_out)


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
        #    selfp.dac.parser.interrupt = 0
        elif selfp.simulator.cycle_counter == 40:
            selfp.dac.out.trigger = 1
        elif selfp.simulator.cycle_counter == 41:
            selfp.dac.out.trigger = 0
        elif selfp.simulator.cycle_counter == 200:
            selfp.dac.out.trigger = 1
        elif selfp.simulator.cycle_counter == 201:
            selfp.dac.out.trigger = 0

        #if selfp.simulator.cycle_counter == 200:
        #    self.dac.parser.interrupt = 0
        if (selfp.dac.out.line_in.ack and
                selfp.dac.out.line_in.stb):
            print("cycle {} data {}".format(
                selfp.simulator.cycle_counter,
                selfp.dac.out.data))


def main():
    from migen.fhdl import verilog
    from migen.sim.generic import run_simulation
    from matplotlib import pyplot as plt
    import numpy as np
    from scipy import interpolate
    import pdq
    pdq.Ftdi = pdq.FileFtdi

    #print(verilog.convert(Dac()))

    t = np.arange(0, 6) * .22e-6
    v = 9*(1-np.cos(t/t[-1]*np.pi))/2
    p = pdq.Pdq()
    k = 4
    mem = p.combine_frames([p.frame(t, v, order=k).data])
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
    main()
