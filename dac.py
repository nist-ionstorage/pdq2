from migen.fhdl.std import *
from migen.genlib.fsm import FSM
from migen.genlib.record import Record
from migen.flow.actor import *
from migen.flow.network import *
from migen.actorlib import sim


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
            mem_depth=6*(1<<10)): # XC3S500E: 20BRAMS 18bits 1024loc
        self.specials.mem = Memory(width=mem_data_width, depth=mem_depth)
        read = self.mem.get_port()
        self.specials.read = read
        cur_adr = Signal(mem_adr_width)
        self.sync += cur_adr.eq(read.adr)
        self.order = Signal(2)
        self.branch = Signal(3)
        self.branch_adrs = Array(Signal(mem_adr_width) for _ in range(8))
        branch_start = Signal(mem_adr_width)
        branch_end = Signal(mem_adr_width)
        self.comb += [
                If(self.branch == 0,
                    branch_start.eq(0),
                ).Else(
                    branch_start.eq(self.branch_adrs[self.branch-1]),
                ),
                branch_end.eq(self.branch_adrs[self.branch]),
                ]

        self.frame_out = Source(frame_layout)
        fp_send = self.frame_out.payload
        fp = Record(frame_layout)
        self.sync += fp_send.raw_bits().eq(fp.raw_bits())
        self.busy = ~self.frame_out.stb

        states = "INIT V0 DT V1A V1B V2A V2B V2C V3A V3B V3C IDLE".split()
        fsm = FSM(*states)
        self.submodules.fsm = fsm

        read_states = states[1:]
        for i, state in enumerate(read_states[:-1]):
            fsm.act(getattr(fsm, state),
                read.adr.eq(cur_adr + 1),
                fsm.next_state(getattr(fsm, read_states[i+1])),
                fp.raw_bits().eq(fp_send.raw_bits()),
                )
        
        fsm.act(fsm.INIT,
                If((cur_adr < branch_start) | 
                   (cur_adr >= branch_end),
                    read.adr.eq(branch_start),
                ).Else(
                    read.adr.eq(cur_adr),
                ),
                fsm.next_state(fsm.V0),
                )
        fsm.act(fsm.V0,
                fp.v0[32:].eq(read.dat_r[:16]),
                If(self.order == 0,
                    fp.wait.eq(0),
                    fp.dt.eq(3), # INIT, V0, IDLE
                    fp.v1.eq(0),
                    fp.v2.eq(0),
                    fp.v3.eq(0),
                    fsm.next_state(fsm.IDLE),
                ))
        fsm.act(fsm.DT,
                fp.wait.eq(read.dat_r[15]),
                If(read.dat_r[15],
                    fp.dt.eq(1 + ~read.dat_r[:15]),
                ).Else(
                    fp.dt.eq(read.dat_r[:15]),
                ),
                If(self.order == 1,
                    fp.v1.eq(0),
                    fp.v2.eq(0),
                    fp.v3.eq(0),
                    fsm.next_state(fsm.IDLE),
                ))
        fsm.act(fsm.V1A,
                fp.v1[16:32].eq(read.dat_r[:16]),
                )
        fsm.act(fsm.V1B,
                fp.v1[32:].eq(read.dat_r[:16]),
                )
        fsm.act(fsm.V2A,
                fp.v2[:16].eq(read.dat_r[:16]),
                )
        fsm.act(fsm.V2B,
                fp.v2[16:32].eq(read.dat_r[:16]),
                )
        fsm.act(fsm.V2C,
                fp.v2[32:].eq(read.dat_r[:16]),
                If(self.order == 2,
                    fp.v3.eq(0),
                    fsm.next_state(fsm.IDLE),
                ))
        fsm.act(fsm.V3A,
                fp.v3[:16].eq(read.dat_r[:16]),
                )
        fsm.act(fsm.V3B,
                fp.v3[16:32].eq(read.dat_r[:16]),
                )
        fsm.act(fsm.V3C,
                fp.v3[32:].eq(read.dat_r[:16]),
                If(self.order == 3,
                    fsm.next_state(fsm.IDLE),
                ),
                )
        fsm.act(fsm.IDLE,
                read.adr.eq(cur_adr),
                self.frame_out.stb.eq(1),
                fp.raw_bits().eq(fp_send.raw_bits()),
                If(self.frame_out.ack,
                    fsm.next_state(fsm.INIT),
                ),
                )


class DacOut(Module):
    def __init__(self):
        self.frame_in = Sink(frame_layout)
        frame = Record(frame_layout)
        self.frame = frame

        self.trigger = Signal()
        t = Signal(15)
        self.t = t

        self.freerun = Signal()
        
        self.busy = t < frame.dt

        self.sync += [
                If(t < frame.dt,
                    self.frame_in.ack.eq(0),
                    frame.v0.eq(frame.v0 + frame.v1),
                    frame.v1.eq(frame.v1 + frame.v2),
                    frame.v2.eq(frame.v2 + frame.v3),
                    t.eq(t + 1),
                ),
                If((frame.dt == 0) | (t + 1 >= frame.dt),
                    If(self.frame_in.stb &
                        (self.freerun | self.trigger | ~frame.wait),
                        self.frame_in.ack.eq(1),
                        frame.raw_bits().eq(self.frame_in.payload.raw_bits()),
                        t.eq(0),
                    ),
                ),
                ]

        self.out = Signal(16)
        self.sync += self.out.eq(frame.v0[-16:])


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
        self.outputs.append(s.rd(self.dac.out.out))
        if s.cycle_counter == 0:
            # s.wr(self.dac.out.trigger, 1)
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
    from matplotlib import pyplot as plt
    import numpy as np
    import pdq
    pdq.Ftdi = pdq.FileFtdi

    t = np.linspace(0, 3e-6, 11)
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
    plt.plot(t, v[1])
    plt.plot(tim, out)
    plt.show()


if __name__ == "__main__":
    main()
