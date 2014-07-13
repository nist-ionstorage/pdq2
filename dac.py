# Robert Jordens <jordens@gmail.com> 2013

from migen.fhdl.std import *
from migen.genlib.record import Record
from migen.genlib.misc import optree
from migen.flow.actor import Source, Sink
from migen.flow.network import CompositeActor, DataFlowGraph
from migen.genlib.cordic import Cordic
from migen.actorlib.fifo import SyncFIFO
from migen.genlib.fsm import FSM, NextState


line_layout = [
        ("header", [
            ("length", 4), # length in shorts
            ("typ", 2), # volt, dds
            ("wait", 1), # wait for trigger
            ("trigger", 1), # override previous wait
            ("shift", 4), # time shift
            ("aux", 1), # aux channel
            ("silence", 1), # shut down clock
            ("reserved", 2),
        ]),
        ("dt", 16),
        ("data", 15*16),
        ]


class Parser(Module):
    def __init__(self, mem_data_width=16, mem_adr_width=16,
            mem_depth=4*(1<<10)): # XC3S500E: 20BRAMS 18bits 1024loc
        self.specials.mem = Memory(width=mem_data_width, depth=mem_depth)
        self.specials.read = read = self.mem.get_port()

        self.line_out = Source(line_layout)
        self.arm = Signal()
        self.interrupt = Signal(4)
        self.busy = Signal()

        complete = Signal()
        repeat = Signal(8)
        next_frame = Signal(8)
        length = Signal(16)
        save_data = Signal(16)
        delayed = Signal()
        cur_frame = Signal(4)
        repetitions = Signal(8)
        save_interrupt = Signal(4)
        line_ready = Signal() # all fields read

        self.comb += [
                self.busy.eq(~self.line_out.stb | self.line_out.ack),
                complete.eq(self.line_out.stb & self.line_out.ack),
        ]

        self.sync += If(complete, self.line_out.stb.eq(0))

        lp = self.line_out.payload

        self.submodules.fsm = fsm = FSM(reset_state="IDLE")
        fsm.delayed_enter("IRQD", "IRQ", 0)
        fsm.act("IRQ", read.adr.eq(self.interrupt), NextState("FRAME"))
        fsm.delayed_enter("FRAMED", "FRAME", 0)
        fsm.act("FRAME", read.adr.eq(read.dat_r), NextState("MODE"))
        fsm.act("MODE", NextState("LENGTH"))
        fsm.act("LENGTH", length.eq(read.dat_r),
                read_adr.eq(read_adr + 1), NextState("DT"))
        fsm.act("MODE",)
        fsm.act("LENGTH",)
        fsm.act("DT",)
        fsm.act("DATA",)

        read_seq = [ # WAIT0/1 are wait states for data->read.adr->read.dat_r
                ("WAIT0", None, 0), ("IRQ", read.adr, 0), ("WAIT1", None, 1),
                ("MODE", Cat(next_frame, repeat), 1), ("LENGTH", length, 1),
                ("HEADER", lp.header.raw_bits(), 1), # will be overridden
                ("DT", lp.dt, 1),
                ] + [
                ("DATA%i" % i, lp.data[i*16:(i + 1)*16], 1) for i in range(15)
                ] + [
                (None,),
                ]

        states = dict((st, i) for i, (st, reg, inc) in enumerate(read_seq[:-1]))
        state = Signal(max=len(states))

        def read_next(reg, next_state, inc):
            ret = []
            if inc:
                ret.append(read.adr.eq(read.adr + inc))
            if reg:
                ret.append(reg.eq(read.dat_r))
            if next_state:
                ret.append(state.eq(states[next_state]))
            return ret

        seq_actions = dict((states[st], read_next(reg, read_seq[i+1][0], inc))
                for i, (st, reg, inc) in enumerate(read_seq[:-1]))

        seq_actions[states["HEADER"]] = [ # override
                If(self.line_out.stb & ~self.line_out.ack & ~delayed,
                    # first delayed sending cycle
                    save_data.eq(read.dat_r), # keep this, adr has advanceed
                    delayed.eq(1),
                ),
                If(complete | ~self.line_out.stb, # sent
                    lp.raw_bits().eq(0),
                    If(delayed,
                        lp.header.raw_bits().eq(save_data), # use kept
                    ).Else(
                        lp.header.raw_bits().eq(read.dat_r), # use current
                    ),
                    delayed.eq(0),
                    read.adr.eq(read.adr + 1),
                    state.eq(states["DT"]),
                )]
        self.sync += If(self.arm, Case(state, seq_actions))

        self.comb += line_ready.eq((state >= states["DT"]) &
                (state - states["DT"] == lp.header.length))
        self.sync += [
            If(line_ready,
                self.line_out.stb.eq(1),
                length.eq(length - 1),
                If(self.interrupt != save_interrupt,
                    read.adr.eq(self.interrupt),
                    save_interrupt.eq(self.interrupt),
                    # lp.trigger.eq(1), # no:
                    # use pads.trigger or line.trigger
                    state.eq(states["WAIT0"]),
                ).Elif(length <= 1,
                    If(repetitions >= repeat,
                        repetitions.eq(1),
                        read.adr.eq(next_frame),
                        cur_frame.eq(next_frame),
                    ).Else(
                        repetitions.eq(repetitions + 1),
                        read.adr.eq(cur_frame),
                    ),
                    state.eq(states["WAIT0"]),
                ).Else(
                    state.eq(states["HEADER"]),
                )
            )]


class DacOut(Module):
    def __init__(self):
        self.line_in = Sink(line_layout)
        self.busy = Signal()
        self.comb += self.busy.eq(~self.line_in.ack | self.line_in.stb)

        self.trigger = Signal()
        self.aux = Signal()
        self.silence = Signal()
        self.data = Signal(16)

        line = Record(line_layout)
        dt_dec = Signal(16)
        save_dt_dec = Signal(16)
        lp_dt_dec = Signal(16)
        hold = Signal()
        tic = Signal()
        toc = Signal()
        stb = Signal()

        lp = self.line_in.payload

        self.comb += [
                hold.eq(line.header.wait & ~(self.trigger |
                    (self.line_in.stb & lp.header.trigger))),
                tic.eq(dt_dec == 0), # counted 1<<shift cycles
                toc.eq(line.dt == 0), # counted dt*(1<<shift) cycles
                self.line_in.ack.eq(tic & toc & ~hold),
                stb.eq(self.line_in.stb & self.line_in.ack),
                lp_dt_dec.eq((1<<lp.header.shift) - 1),
        ]

        subs = [
            Volt(lp.data, stb & (lp.header.typ == 0), tic & ~toc),
            Dds(lp.data, stb & (lp.header.typ == 1), tic & ~toc),
        ]

        for i, sub in enumerate(subs):
            self.submodules += sub

        self.sync += [
                self.data.eq(optree("+", [sub.data for sub in subs])),

                If(~tic,
                    dt_dec.eq(dt_dec - 1),
                ),
                If(tic & ~toc,
                    dt_dec.eq(save_dt_dec),
                    line.dt.eq(line.dt - 1),
                ),
                If(stb,
                    line.dt.eq(lp.dt),
                    dt_dec.eq(lp_dt_dec),
                    save_dt_dec.eq(lp_dt_dec),

                    line.header.eq(lp.header),
                    self.aux.eq(lp.header.aux),
                    self.silence.eq(lp.header.silence),
                ),
        ]


class Volt(Module):
    def __init__(self, data, stb, tic):
        self.data = Signal(16)

        v = [Signal(48) for i in range(4)]
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
        x = [Signal(32) for i in range(2)] # amplitude, damplitude
        self.comb += [
                self.cordic.xi.eq(x[0][16:]),
                self.cordic.yi.eq(0),
                self.cordic.zi.eq(z[0][32:]),
                self.data.eq(self.cordic.xo),
                ]
        self.sync += [
                If(tic,
                    z[0].eq(z[0] + z[1]),
                    z[1].eq(z[1] + z[2]),
                    x[0].eq(x[0] + x[1]),
                ),
                If(stb,
                    z[0].eq(0),
                    z[1].eq(0),
                    x[0].eq(0),
                    Cat(z[0][32:], x[0][16:], z[1][16:], x[1], z[2]).eq(data),
                )]


class Dac(Module):
    def __init__(self, fifo=True, **kwargs):
        self.parser = Parser(**kwargs)
        self.out = DacOut()
        g = DataFlowGraph()
        if fifo:
            self.fifo = SyncFIFO(line_layout, 16)
            g.add_connection(self.parser, self.fifo)
            g.add_connection(self.fifo, self.out)
        else:
            g.add_connection(self.parser, self.out)
        self.submodules.graph = CompositeActor(g)


class TB(Module):
    def __init__(self, mem=None):
        self.submodules.dac = Dac()
        if mem is not None:
            self.dac.parser.mem.init = mem
        self.outputs = []

    def do_simulation(self, selfp):
        self.outputs.append(selfp.dac.out.data)
        if selfp.simulator.cycle_counter == 10:
            selfp.dac.parser.arm = 1
        #    selfp.dac.parser.interrupt = 0
        elif selfp.simulator.cycle_counter == 20:
            selfp.dac.out.trigger = 1
        elif selfp.simulator.cycle_counter == 21:
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
    frame = p.frame(t, v, order=4, aux=t>.6e-6, wait_last=True)
    frame = p.add_frame_header(frame, repeat=0, next=0)
    mem = p.combine_frames([frame])
    mem = list(np.fromstring(mem, "<u2"))
    tb = TB(mem)
    run_simulation(tb, ncycles=300, vcd_name="dac.vcd")

    plt.plot(t, v, "xk")
    
    sp = interpolate.splrep(t, v, k=3)
    tt = np.arange(t[0], t[-1], 1/p.freq)
    vv = interpolate.splev(tt, sp, der=0)
    plt.plot(tt, vv, "-g")

    out = np.array(tb.outputs, np.uint16).view(np.int16)*20./(1<<16)
    tim = np.arange(out.shape[0])/p.freq
    plt.plot(tim-30/p.freq, out, "-r")
    plt.show()


if __name__ == "__main__":
    main()
