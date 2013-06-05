from migen.fhdl.std import *
from migen.genlib.record import Record
from migen.flow.actor import Source, Sink
from migen.flow.network import CompositeActor, DataFlowGraph


frame_layout = [
        ("wait", 1),
        ("dt", 15),
        ("v0", 48),
        ("v1", 48),
        ("v2", 48),
        ("v3", 48),
        ]


class DacReader(Module):
    def __init__(self, mem_data_width=16, mem_adr_width=13,
            mem_depth=4*(1<<10)): # XC3S500E: 20BRAMS 18bits 1024loc
        self.specials.mem = Memory(width=mem_data_width, depth=mem_depth)
        read = self.mem.get_port()
        self.specials.read = read
        self.order = Signal(2)
        self.branch = Signal(3)
        self.branch_adrs = Array(Signal(16) for _ in range(8))
        branch_start = Signal(16)
        branch_end = Signal(16)
        self.comb += [
                If(self.branch == 0,
                    branch_start.eq(0),
                ).Else(
                    branch_start.eq(self.branch_adrs[self.branch-1]),
                ),
                branch_end.eq(self.branch_adrs[self.branch]),
                ]

        self.frame_out = Source(frame_layout)
        fp = self.frame_out.payload
        self.busy = ~self.frame_out.stb

        states = dict((v, i) for i, v in enumerate(
            "IDLE V0 DT V1A V1B V2A V2B V2C V3A V3B V3C".split()))
        state = Signal(max=len(states))
        self.state = state

        read_next = lambda reg, s: [
                reg.eq(read.dat_r[:16]),
                read.adr.eq(read.adr + 1),
                state.eq(states[s]),
                ]
        short_send = lambda order: [
                If(self.order == order,
                    self.frame_out.stb.eq(1),
                    state.eq(states["IDLE"]),
                )]

        actions = {
                "IDLE": [
                    If(self.frame_out.ack | ~self.frame_out.stb,
                        self.frame_out.stb.eq(0),
                        fp.eq(0),
                        read.adr.eq(read.adr + 1),
                        state.eq(states["V0"]),
                    )],
                "V0": read_next(fp.v0[32:], "DT") + short_send(0),
                "DT": read_next(fp.dt, "V1A") + short_send(1) + [
                    fp.wait.eq(read.dat_r[15]),
                    If(read.dat_r[15],
                        fp.dt.eq(1 + ~read.dat_r[:15]),
                    )],
                "V1A": read_next(fp.v1[16:32], "V1B"),
                "V1B": read_next(fp.v1[32:], "V2A"),
                "V2A": read_next(fp.v2[:16], "V2B"),
                "V2B": read_next(fp.v2[16:32], "V2C"),
                "V2C": read_next(fp.v2[32:], "V3A") + short_send(2),
                "V3A": read_next(fp.v3[:16], "V3B"),
                "V3B": read_next(fp.v3[16:32], "V3C"),
                "V3C": read_next(fp.v3[32:], "IDLE") + short_send(3) + [
                    # 2-cycle delay, undo incr, will be redone in IDLE
                    read.adr.eq(read.adr),
                    If((read.adr < branch_start[:mem_adr_width]) |
                       (read.adr >= branch_end[:mem_adr_width]),
                        read.adr.eq(branch_start[:mem_adr_width]),
                    )],
                }
        actions = dict((states[k], _) for k, _ in actions.items())
        self.sync += Case(state, actions)
       

class DacOut(Module):
    def __init__(self):
        self.frame_in = Sink(frame_layout)
        frame = Record(frame_layout)
        self.frame = frame
        self.busy = frame.dt > 0

        self.trigger = Signal()
        self.freerun = Signal()
        self.data = Signal(16)

        self.sync += [
                If((frame.dt <= 2) &
                        (self.freerun | self.trigger | ~frame.wait),
                    self.frame_in.ack.eq(1),
                ),
                If(frame.dt > 1,
                    frame.v0.eq(frame.v0 + frame.v1),
                    frame.v1.eq(frame.v1 + frame.v2),
                    frame.v2.eq(frame.v2 + frame.v3),
                    frame.dt.eq(frame.dt - 1),
                ).Elif(self.frame_in.stb & self.frame_in.ack,
                    self.frame_in.ack.eq(0),
                    frame.eq(self.frame_in.payload),
                ),
                self.data.eq(frame.v0[32:]),
                ]


class Dac(Module):
    def __init__(self):
        g = DataFlowGraph()
        self.reader = DacReader()
        self.out = DacOut()
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
            #s.wr(self.dac.out.trigger, 1)
            s.wr(self.dac.reader.branch, 1)
            s.wr(self.dac.reader.order, 3)
            s.wr(self.dac.reader.branch_adrs[1],
                    len(self.dac.reader.mem.init))
        if (s.rd(self.dac.out.frame_in.ack) and
                s.rd(self.dac.out.frame_in.stb)):
            f = self.dac.out.frame
            print("frame {} {} {}".format(s.rd(f.wait), s.rd(f.dt),
                s.rd(f.v0)))


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
