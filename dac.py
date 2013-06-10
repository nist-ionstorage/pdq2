from migen.fhdl.std import *
from migen.genlib.record import Record
from migen.genlib.misc import optree
from migen.flow.actor import Source, Sink
from migen.flow.network import CompositeActor, DataFlowGraph


# MEM:
#   IRQ0_ADR 16
#   ...
#   IRQ7_ADR 16
#   ADDITIONAL_ADR 16
#   ...
#   FRAME
#   ...
# FRAME:
#   MODE 16:
#       NEXT_FRAME 8
#       REPEAT 8
#   LENGTH 16
#   LINE
#   ...
# LINE:
#   HEADER 16:
#     TYP 4
#     WAIT 1
#     TRIGGER 1
#     SHIFT 4
#     AUX 4
#     RESERVED 2
#   DT 16
#   (V0 16)
#   (V1 32)
#   (V2 48)
#   (V3 48)


line_layout = [
        ("typ", 4),
        ("wait", 1),
        ("trigger", 1),
        ("shift", 4),
        ("aux", 4),
        ("reserved", 2),
        ("dt", 16),
        ("v0", 48),
        ("v1", 48),
        ("v2", 48),
        ("v3", 48),
        ]


class Parser(Module):
    def __init__(self, mem_data_width=16, mem_adr_width=16,
            mem_depth=4*(1<<10)): # XC3S500E: 20BRAMS 18bits 1024loc
        self.specials.mem = Memory(width=mem_data_width, depth=mem_depth)
        read = self.mem.get_port()
        self.specials.read = read

        self.line_out = Source(line_layout)
        fp = self.line_out.payload
        self.busy = Signal()
        self.comb += self.busy.eq(~self.line_out.stb | self.line_out.ack)
        complete = Signal()
        self.comb += complete.eq(self.line_out.stb & self.line_out.ack)
        self.sync += If(complete, self.line_out.stb.eq(0))
                # fp.raw_bits().eq(0), # no, we ignore non-typ bits

        repeat = Signal(8)
        next_frame = Signal(8)
        mode = Cat(next_frame, repeat)
        length = Signal(16)

        header = Cat(fp.typ, fp.wait, fp.trigger, fp.shift, fp.aux,
                fp.reserved)

        read_seq = [ # WAIT0/1 are wait states for data->read.adr->read.dat_r
                ("WAIT0", None, 0), ("IRQ", read.adr, 0), ("WAIT1", None, 1),
                ("MODE", mode, 1), ("LENGTH", length, 1),
                ("HEADER", header, 1), # will be overridden
                ("DT", fp.dt, 1),
                ("V0", fp.v0[32:], 1),
                ("V1", fp.v1[16:32], 1), ("V1A", fp.v1[32:], 1),
                ("V2", fp.v2[:16], 1), ("V2A", fp.v2[16:32], 1),
                    ("V2B", fp.v2[32:], 1),
                ("V3", fp.v3[:16], 1), ("V3A", fp.v3[16:32], 1),
                    ("V3B", fp.v3[32:], 1),
                (None,),
                ]

        states = dict((st, i) for i, (st, reg, inc) in
                enumerate(read_seq[:-1]))
        state = Signal(max=len(states))

        def read_next(reg, next_state, inc):
            ret = []
            if inc: ret.append(read.adr.eq(read.adr + inc))
            if reg: ret.append(reg.eq(read.dat_r[:flen(reg)]))
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
                    If(delayed,
                        header.eq(save_data), # use kept
                    ).Else(
                        header.eq(read.dat_r), # use current
                    ),
                    delayed.eq(0),
                    read.adr.eq(read.adr + 1),
                    state.eq(states["DT"]),
                )]
        self.sync += Case(state, seq_actions)
        
        cur_frame = Signal(4)
        repetitions = Signal(8)
        interrupt = Signal(4)
        self.interrupt = interrupt
        save_interrupt = Signal(4)

        short_send = "DT V0 V1A V2B V3B".split()
        line_ready = Signal() # all fields read
        self.comb += line_ready.eq(optree("|", [
                    (state == states[st]) & (fp.typ == j)
                    for j, st in enumerate(short_send)]))
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
        line = Record(line_layout)
        self.line = line
        self.busy = Signal()
        self.comb += self.busy.eq(~self.line_in.ack | self.line_in.stb)

        self.trigger = Signal()
        self.arm = Signal()
        self.aux = Signal(4)

        self.data = Signal(16)
        self.comb += self.data.eq(line.v0[32:])
        dt_dec = Signal(16)
        line_dt_dec = Signal(16)
        next_triggers = Signal()
        self.comb += next_triggers.eq(self.line_in.stb &
                self.line_in.payload.trigger)
        hold = Signal()
        self.comb += hold.eq(~self.arm | (line.wait &
            ~(self.trigger | next_triggers)))
        self.comb += self.line_in.ack.eq((line.dt <= 1) & (dt_dec <= 1)
                & ~hold)
        complete = Signal()
        self.comb += complete.eq(self.line_in.stb & self.line_in.ack)

        lp = self.line_in.payload
        self.sync += [
                If(dt_dec > 1,
                    dt_dec.eq(dt_dec - 1),
                ).Else(
                    If(line.dt > 1,
                        If(line.typ >= 2, line.v0.eq(line.v0 + line.v1)),
                        If(line.typ >= 3, line.v1.eq(line.v1 + line.v2)),
                        If(line.typ >= 4, line.v2.eq(line.v2 + line.v3)),
                        line.dt.eq(line.dt - 1),
                        dt_dec.eq(line_dt_dec),
                    ).Elif(complete,
                        Cat(line.typ, line.wait, line.trigger, line.dt).eq(
                            Cat(lp.typ, lp.wait, lp.trigger, lp.dt)),
                        self.aux.eq(lp.aux),
                        dt_dec.eq(1<<lp.shift),
                        line_dt_dec.eq(1<<lp.shift),
                        If(lp.typ >= 1, line.v0.eq(lp.v0)),
                        If(lp.typ >= 2, line.v1.eq(lp.v1)),
                        If(lp.typ >= 3, line.v2.eq(lp.v2)),
                        If(lp.typ >= 4, line.v3.eq(lp.v3)),
                    ),
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
            s.wr(self.dac.out.arm, 1)
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
    tim = np.arange(out.shape[0])/50e6
    spframe = interpolate.splrep(t, v, s=0, k=3)
    tt = np.arange(t[0], t[-1], 1/p.freq)
    vv = interpolate.splev(tt, spframe, der=0)
    plt.plot(t, v, "xk")
    plt.plot(tt, vv, ",g")
    plt.plot(tim-30/50e6, out, "-r")
    plt.show()


if __name__ == "__main__":
    main()
