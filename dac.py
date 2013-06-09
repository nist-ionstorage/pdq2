from migen.fhdl.std import *
from migen.genlib.record import Record
from migen.genlib.misc import optree
from migen.flow.actor import Source, Sink
from migen.flow.network import CompositeActor, DataFlowGraph


frame_layout = [
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


class DacReader(Module):
    def __init__(self, mem_data_width=16, mem_adr_width=16,
            mem_depth=4*(1<<10)): # XC3S500E: 20BRAMS 18bits 1024loc
        self.specials.mem = Memory(width=mem_data_width, depth=mem_depth)
        read = self.mem.get_port()
        self.specials.read = read

        self.frame_out = Source(frame_layout)
        fp = self.frame_out.payload
        self.busy = Signal()
        self.comb += self.busy.eq(~self.frame_out.stb)

        # MEM:
        #   HEADER
        #   LINE ... LINE
        # HEADER:
        #   IRQ0 ... IRQ7
        #   OTHER_LINE_ADRS
        # LINE:
        #   MODE:
        #       NEXT_LINE 8
        #       REPEAT 8
        #   LENGTH
        #   FRAME ... FRAME
        # FRAME:
        #   HEADER: 
        #     TYP 4
        #     WAIT 1
        #     FORCE 1
        #     DT_SHIFT 4
        #     RESERVED 6
        #   DT 16
        #   V0 16
        #   V1 32
        #   V2 48
        #   V3 48

        repeat = Signal(8)
        next_line = Signal(8)
        mode = Cat(next_line, repeat)
        length = Signal(16)

        interrupt = Signal(4)
        self.interrupt = interrupt
        save_interrupt = Signal(4)
        self.sync += If(interrupt != save_interrupt,
                    next_line.eq(interrupt),
                    save_interrupt.eq(interrupt),
                )

        header = Signal(16)
        self.comb += Cat(fp.typ, fp.wait, fp.force,
                fp.shift, fp.reserved).eq(header)

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
        seq_actions[states["HEADER"]] = [ # override
                save_data.eq(read.dat_r),
                If(self.frame_out.stb, # sending
                    If(~self.frame_out.ack, # not sent
                        save_data.eq(save_data), # keep
                        state.eq(states["HEADER"]),
                    ).Else( # sent with delay
                        read.adr.eq(read.adr + 1),
                        header.eq(save_data), # kept
                        state.eq(states["DT"]),
                    )
                ).Else( # already sent
                    read.adr.eq(read.adr + 1),
                    header.eq(read.dat_r),
                    state.eq(states["DT"]),
                )]
        self.sync += Case(state, seq_actions)
        
        cur_line = Signal(4)

        short_send = "V0 V1A V2B V3B".split()
        self.sync += [
            If(optree("|", [
                    (state == states[st]) & (fp.typ == j)
                    for j, st in enumerate(short_send)]),
                self.frame_out.stb.eq(1),
                length.eq(length - 1),
                If(length <= 1,
                    repeat.eq(repeat - 1),
                    If(repeat <= 1,
                        cur_line.eq(next_line),
                        read.adr.eq(next_line),
                    ).Else(
                        read.adr.eq(cur_line),
                    ),
                    state.eq(states["WAIT0"]),
                ).Else(
                    If(self.frame_out.ack,
                        read.adr.eq(read.adr + 1),
                    ),
                    state.eq(states["HEADER"]),
                )
            ),
            If(self.frame_out.stb & self.frame_out.ack,
                self.frame_out.stb.eq(0),
            ),
            ]


class DacOut(Module):
    def __init__(self):
        self.frame_in = Sink(frame_layout)
        frame = Record(frame_layout)
        self.frame = frame
        self.busy = Signal()
        self.comb += self.busy.eq(~self.frame_in.ack)

        self.trigger = Signal()
        self.data = Signal(16)
        self.comb += self.data.eq(frame.v0[-16:])
        self.comb += self.frame_in.ack.eq((frame.dt <= 1) &
                (self.frame_in.payload.force | self.trigger | ~frame.wait))
        t_dec = Signal(16)
        self.sync += [
                If(t_dec > 1,
                    t_dec.eq(t_dec - 1),
                ).Else(
                    If(frame.dt > 1,
                        frame.v0.eq(frame.v0 + frame.v1),
                        frame.v1.eq(frame.v1 + frame.v2),
                        frame.v2.eq(frame.v2 + frame.v3),
                        frame.dt.eq(frame.dt - 1),
                    ).Elif(self.frame_in.stb & self.frame_in.ack,
                        frame.eq(self.frame_in.payload),
                        t_dec.eq(1<<self.frame_in.payload.shift),
                    ),
                ),
                ]


class Dac(Module):
    def __init__(self, **kwargs):
        self.reader = DacReader(**kwargs)
        self.out = DacOut()
        g = DataFlowGraph()
        g.add_connection(self.reader, self.out)
        self.submodules.graph = CompositeActor(g)


class TB(Module):
    def __init__(self, mem=None):
        self.submodules.dac = Dac()
        if mem is not None:
            self.dac.reader.mem.init = mem
        self.outputs = []

    def do_simulation(self, s):
        self.outputs.append(s.rd(self.dac.out.data))
        if s.cycle_counter == 0:
            s.wr(self.dac.out.trigger, 1)
            s.wr(self.dac.reader.interrupt, 1)
        if (s.rd(self.dac.out.frame_in.ack) and
                s.rd(self.dac.out.frame_in.stb)):
            f = self.dac.out.frame
            print("frame {} {}".format(s.rd(f.dt), s.rd(f.v0)))


def main():
    from migen.sim.generic import Simulator, TopLevel
    from migen.actorlib import sim
    from matplotlib import pyplot as plt
    import numpy as np
    from scipy import interpolate
    import pdq
    pdq.Ftdi = pdq.FileFtdi

    t = np.arange(0, 6) * .25e-6
    v = [None] * 8
    v[1] = (1-np.cos(t/t[-1]*np.pi))/2
    p = pdq.Pdq("dac_test")
    p.prepare_simple(t, v, channel=4, mode=3, trigger=True)
    p.dev.fil.close()
    fil = "pdq_dac_test_ftdi.bin"
    skip = 37
    mem = np.fromstring(
            open(fil, "rb").read()[skip:],
            dtype=np.dtype("<u2"))

    tb = TB(mem)
    sim = Simulator(tb, TopLevel("dac.vcd"))
    n = 200
    sim.run(n)
    out = np.array(tb.outputs, np.uint16).view(np.int16)*20./(1<<16)
    tim = np.arange(out.shape[0])/50e6
    spline = interpolate.splrep(t, v[1], s=0, k=3)
    tt = np.arange(t[0], t[-1], 1/p.freq)
    vv = interpolate.splev(tt, spline, der=0)
    plt.plot(t, v[1], "xk")
    plt.plot(tt, vv, ",g")
    plt.plot(tim-12/50e6, out, "-r")
    plt.show()


if __name__ == "__main__":
    main()
