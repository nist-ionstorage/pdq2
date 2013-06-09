from migen.fhdl.std import *
from migen.genlib.record import Record
from migen.genlib.misc import optree
from migen.flow.actor import Source, Sink
from migen.flow.network import CompositeActor, DataFlowGraph


line_layout = [
        ("typ", 4),
        ("wait", 1),
        ("force", 1),
        ("shift", 4),
        ("reserved", 6),
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
        self.comb += self.busy.eq(~self.line_out.stb)

        # MEM:
        #   IRQ0_ADR 16
        #   ...
        #   IRQ7_ADR 16
        #   JUMP_ADR 16
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
        #     FORCE 1
        #     DT_SHIFT 4
        #     RESERVED 6
        #   DT 16
        #   V0 16
        #   (V1 32)
        #   (V2 48)
        #   (V3 48)

        repeat = Signal(8)
        next_frame = Signal(8)
        mode = Cat(next_frame, repeat)
        length = Signal(16)

        header = Cat(fp.typ, fp.wait, fp.force, fp.shift, fp.reserved)

        states = dict((v, i) for i, v in enumerate("WAIT0 IRQ WAIT1 "
            "MODE LENGTH HEADER "
            "DT V0 V1 V1A V2 V2A V2B V3 V3A V3B".split()))
        state = Signal(max=len(states))

        def read_next(reg, next_state, inc):
            ret = []
            if inc:
                ret.append(read.adr.eq(read.adr + inc))
            if reg:
                ret.append(reg.eq(read.dat_r[:flen(reg)]))
            if next_state:
                ret.append(state.eq(states[next_state]))
            return ret

        read_seq = [
                ("WAIT0", None, 0), ("IRQ", read.adr, 0), ("WAIT1", None, 1),
                ("MODE", mode, 1), ("LENGTH", length, 1),
                ("HEADER", header, 1),
                ("DT", fp.dt, 1),
                ("V0", fp.v0[32:], 1),
                ("V1", fp.v1[16:32], 1), ("V1A", fp.v1[32:], 1),
                ("V2", fp.v2[:16], 1), ("V2A", fp.v2[16:32], 1),
                    ("V2B", fp.v2[32:], 1),
                ("V3", fp.v3[:16], 1), ("V3A", fp.v3[16:32], 1),
                    ("V3B", fp.v3[32:], 1),
                (None,),
                ]
        seq_actions = dict((states[st], read_next(reg, read_seq[i+1][0], inc))
                for i, (st, reg, inc) in enumerate(read_seq[:-1]))

        save_data = Signal(16)
        delayed = Signal()
        complete = Signal()
        self.comb += complete.eq(self.line_out.stb &
                self.line_out.ack)
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

        short_send = "V0 V1A V2B V3B".split()
        line_ready = Signal()
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
                    state.eq(states["WAIT0"]),
                    fp.force.eq(1), # ?
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
            ),
            If(complete,
                self.line_out.stb.eq(0),
            ),
            ]


class DacOut(Module):
    def __init__(self):
        self.line_in = Sink(line_layout)
        line = Record(line_layout)
        self.line = line
        self.busy = Signal()
        self.comb += self.busy.eq(~self.line_in.ack)

        self.trigger = Signal()
        self.data = Signal(16)
        self.comb += self.data.eq(line.v0[-16:])
        dt_dec = Signal(16)
        line_dt_dec = Signal(16)
        self.comb += self.line_in.ack.eq((line.dt <= 1) & (dt_dec <= 1) &
                (self.line_in.payload.force | self.trigger | ~line.wait))
        self.sync += [
                If(dt_dec > 1,
                    dt_dec.eq(dt_dec - 1),
                ).Else(
                    If(line.dt > 1,
                        line.v0.eq(line.v0 + line.v1),
                        line.v1.eq(line.v1 + line.v2),
                        line.v2.eq(line.v2 + line.v3),
                        line.dt.eq(line.dt - 1),
                        dt_dec.eq(line_dt_dec),
                    ).Elif(self.line_in.stb & self.line_in.ack,
                        line.eq(self.line_in.payload),
                        dt_dec.eq(1<<self.line_in.payload.shift),
                        line_dt_dec.eq(1<<self.line_in.payload.shift),
                    ),
                ),
                ]


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
        #if s.cycle_counter == 0:
        #    s.wr(self.dac.parser.interrupt, 2)
        if s.cycle_counter == 100:
            s.wr(self.dac.out.trigger, 1)
        #if s.cycle_counter == 200:
        #    s.wr(self.dac.parser.interrupt, 0)
        if (s.rd(self.dac.out.line_in.ack) and
                s.rd(self.dac.out.line_in.stb)):
            f = self.dac.out.line
            print("line {} {}".format(s.rd(f.dt), s.rd(f.v0)))


def main():
    from migen.fhdl import verilog
    from migen.sim.generic import Simulator, TopLevel
    from migen.actorlib import sim
    from matplotlib import pyplot as plt
    import numpy as np
    from scipy import interpolate
    import pdq

    #print(verilog.convert(Dac()))

    t = np.arange(0, 6) * .28e-6
    v = 9*(1-np.cos(t/t[-1]*np.pi))/2
    p = pdq.Pdq("dac_test")
    mem = bytes([0x01, 0x00]) + \
            p.serialize_frame(t, v, derivatives=3,
            trigger=True, next_frame=1, repeat=2)
    mem = np.fromstring(mem, "<u2")

    tb = TB(mem)
    sim = Simulator(tb, TopLevel("dac.vcd"))
    n = 400
    sim.run(n)
    out = np.array(tb.outputs, np.uint16).view(np.int16)*20./(1<<16)
    tim = np.arange(out.shape[0])/50e6
    spframe = interpolate.splrep(t, v, s=0, k=3)
    tt = np.arange(t[0], t[-1], 1/p.freq)
    vv = interpolate.splev(tt, spframe, der=0)
    plt.plot(t, v, "xk")
    plt.plot(tt, vv, ",g")
    plt.plot(tim-12/50e6, out, "-r")
    plt.show()


if __name__ == "__main__":
    main()
