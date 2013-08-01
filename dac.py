from migen.fhdl.std import *
from migen.genlib.record import Record
from migen.genlib.misc import optree
from migen.flow.actor import Source, Sink
from migen.flow.network import CompositeActor, DataFlowGraph
from migen.genlib.cordic import Cordic


line_layout = [
        ("header", [
            ("length", 4),
            ("typ", 2),
            ("wait", 1),
            ("trigger", 1),
            ("shift", 4),
            ("aux", 1),
            ("reserved", 3),
        ]),
        ("dt", 16),
        ("data", 15*16),
        ]

class Parser(Module):
    def __init__(self, mem_data_width=16, mem_adr_width=16,
            mem_depth=4*(1<<10)): # XC3S500E: 20BRAMS 18bits 1024loc
        self.specials.mem = Memory(width=mem_data_width, depth=mem_depth)
        read = self.mem.get_port()
        self.specials.read = read

        self.arm = Signal()

        self.line_out = Source(line_layout)
        fp = self.line_out.payload
        self.busy = Signal()
        self.comb += self.busy.eq(~self.line_out.stb | self.line_out.ack)
        complete = Signal()
        self.comb += complete.eq(self.line_out.stb & self.line_out.ack)
        self.sync += If(complete, self.line_out.stb.eq(0))

        repeat = Signal(8)
        next_frame = Signal(8)
        mode = Cat(next_frame, repeat)
        length = Signal(16)

        read_seq = [ # WAIT0/1 are wait states for data->read.adr->read.dat_r
                ("WAIT0", None, 0), ("IRQ", read.adr, 0), ("WAIT1", None, 1),
                ("MODE", mode, 1), ("LENGTH", length, 1),
                ("HEADER", fp.header.raw_bits(), 1), # will be overridden
                ("DT", fp.dt, 1),
                ] + [
                ("DATA%i" % i, fp.data[i*16:(i+1)*16], 1) for i in range(15)
                ] + [
                (None,),
                ]

        states = dict((st, i) for i, (st, reg, inc) in
                enumerate(read_seq[:-1]))
        state = Signal(max=len(states))

        def read_next(reg, next_state, inc):
            ret = []
            if inc: ret.append(read.adr.eq(read.adr + inc))
            if reg: ret.append(reg.eq(read.dat_r))
            if next_state: ret.append(state.eq(states[next_state]))
            return ret

        seq_actions = dict((states[st], read_next(reg, read_seq[i+1][0], inc))
                for i, (st, reg, inc) in enumerate(read_seq[:-1]))

        save_data = Signal(16)
        delayed = Signal()
        seq_actions[states["HEADER"]] = [ # override
                If(self.line_out.stb & ~self.line_out.ack & ~delayed,
                    # first delayed sending cycle
                    save_data.eq(read.dat_r), # keep this, adr has advanceed
                    delayed.eq(1),
                ),
                If(complete | ~self.line_out.stb, # sent
                    fp.raw_bits().eq(0),
                    If(delayed,
                        fp.header.raw_bits().eq(save_data), # use kept
                    ).Else(
                        fp.header.raw_bits().eq(read.dat_r), # use current
                    ),
                    delayed.eq(0),
                    read.adr.eq(read.adr + 1),
                    state.eq(states["DT"]),
                )]
        self.sync += If(self.arm, Case(state, seq_actions))
        
        cur_frame = Signal(4)
        repetitions = Signal(8)
        interrupt = Signal(4)
        self.interrupt = interrupt
        save_interrupt = Signal(4)

        line_ready = Signal() # all fields read
        self.comb += line_ready.eq((state >= states["DT"]) & 
                (state - states["DT"] == fp.header.length))
        self.sync += [
            If(line_ready,
                self.line_out.stb.eq(1),
                length.eq(length - 1),
                If(interrupt != save_interrupt,
                    read.adr.eq(interrupt),
                    save_interrupt.eq(interrupt),
                    # fp.trigger.eq(1), # no:
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

        line = Record(line_layout)

        self.data = Signal(16)
        dt_dec = Signal(16)
        next_triggers = Signal()
        self.comb += next_triggers.eq(self.line_in.stb &
                self.line_in.payload.header.trigger)
        hold = Signal()
        self.comb += hold.eq(line.header.wait &
            ~(self.trigger | next_triggers))
        delay = Signal()
        self.comb += delay.eq(dt_dec > 1)
        tic = Signal()
        self.comb += tic.eq(~delay & (line.dt > 1))
        self.comb += self.line_in.ack.eq(~delay & ~tic & ~hold)
        complete = Signal()
        self.comb += complete.eq(self.line_in.stb & self.line_in.ack)
    
        lp = self.line_in.payload

        self.sync += self.data.eq(0)
        for Sub in Dds, Volt:
            sub = Sub(lp, line, complete, tic)
            self.submodules += sub
            self.sync += If(sub.valid, self.data.eq(sub.data))

        self.sync += [
                If(delay,
                    dt_dec.eq(dt_dec - 1),
                ),
                If(tic,
                    line.dt.eq(line.dt - 1),
                    dt_dec.eq(1<<line.header.shift),
                ),
                If(complete,
                    line.header.eq(lp.header),
                    line.dt.eq(lp.dt),
                    self.aux.eq(lp.header.aux),
                    dt_dec.eq(1<<lp.header.shift),
                )]


class Volt(Module):
    def __init__(self, lp, line, complete, tic):
        self.valid = Signal()
        self.data = Signal(16)

        v = [Signal(48) for i in range(4)]
        self.comb += [
                self.data.eq(v[0][32:]),
                self.valid.eq(line.header.typ == 0),
                ]

        self.sync += [
                If(tic,
                    v[0].eq(v[0] + v[1]),
                    v[1].eq(v[1] + v[2]),
                    v[2].eq(v[2] + v[3]),
                ),
                If(complete,
                    v[0].eq(0),
                    v[1].eq(0),
                    Cat(v[0][32:], v[1][16:], v[2], v[3]).eq(lp.data),
                )]


class Dds(Module):
    def __init__(self, lp, line, complete, tic):
        self.submodules.cordic = Cordic(width=16, guard=None,
                eval_mode="pipelined", cordic_mode="rotate",
                func_mode="circular")
        self.valid = Signal()
        self.data = Signal(16)

        valid_sr = Signal(self.cordic.latency)
        self.sync += valid_sr.eq(Cat(line.header.typ == 1, valid_sr))

        z = [Signal(48) for i in range(3)]
        x = [Signal(32) for i in range(2)]
        self.comb += [
                self.cordic.xi.eq(x[0][16:]),
                self.cordic.yi.eq(0),
                self.cordic.zi.eq(z[0][32:]),
                self.data.eq(self.cordic.xo),
                self.valid.eq(valid_sr[-1]),
                ]
        self.sync += [
                If(tic,
                    z[0].eq(z[0] + z[1]),
                    z[1].eq(z[1] + z[2]),
                    x[0].eq(x[0] + x[1]),
                ),
                If(complete,
                    z[0].eq(0),
                    z[1].eq(0),
                    x[0].eq(0),
                    Cat(z[0][32:], x[0][16:], z[1][16:], x[1], z[2]
                        ).eq(lp.data),
                )]


class Dac(Module):
    def __init__(self, **kwargs):
        self.parser = Parser(**kwargs)
        self.out = DacOut()
        g = DataFlowGraph()
        g.add_connection(self.parser, self.out)
        self.submodules.graph = CompositeActor(g)


class TB(Module):
    def __init__(self, mem=None):
        self.submodules.dac = Dac()
        if mem is not None:
            self.dac.parser.mem.init = mem
        self.outputs = []

    def do_simulation(self, s):
        self.outputs.append(s.rd(self.dac.out.data))
        if s.cycle_counter == 30:
            s.wr(self.dac.parser.arm, 1)
        #    s.wr(self.dac.parser.interrupt, 2)
        #if s.cycle_counter == 100:
        #    s.wr(self.dac.out.trigger, 1)
        #if s.cycle_counter == 200:
        #    s.wr(self.dac.parser.interrupt, 0)
        if (s.rd(self.dac.out.line_in.ack) and
                s.rd(self.dac.out.line_in.stb)):
            f = self.dac.out.line
            print("line {} {}".format(s.rd(f.dt), s.rd(f.v0)//(1<<32)))


def main():
    from migen.fhdl import verilog
    from migen.sim.generic import Simulator, TopLevel
    from matplotlib import pyplot as plt
    import numpy as np
    from scipy import interpolate
    import pdq
    pdq.Ftdi = pdq.FileFtdi

    #print(verilog.convert(Dac()))

    t = np.arange(0, 6) * .22e-6
    v = 9*(1-np.cos(t/t[-1]*np.pi))/2
    p = pdq.Pdq()
    frame = p.frame(t, v, derivatives=4, aux=t>.6e-6, wait_last=True)
    frame = p.add_frame_header(frame, repeat=1, next=0)
    mem = p.combine_frames([frame])
    mem = list(np.fromstring(mem, "<u2"))

    tb = TB(mem)
    sim = Simulator(tb, TopLevel("dac.vcd"))
    n = 200
    sim.run(n)
    out = np.array(tb.outputs, np.uint16).view(np.int16)*20./(1<<16)
    tim = np.arange(out.shape[0])/p.freq
    spframe = interpolate.splrep(t, v, s=0, k=3)
    tt = np.arange(t[0], t[-1], 1/p.freq)
    vv = interpolate.splev(tt, spframe, der=0)
    plt.plot(t, v, "xk")
    plt.plot(tt, vv, ",g")
    plt.plot(tim-30/p.freq, out, "-r")
    plt.show()


if __name__ == "__main__":
    main()
