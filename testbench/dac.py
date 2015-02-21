from migen.fhdl.std import *
from migen.sim.generic import run_simulation

from matplotlib import pyplot as plt
import numpy as np
from scipy import interpolate

from gateware.dac import Dac
from host import pdq2


class TB(Module):
    def __init__(self, mem=None):
        self.submodules.dac = Dac()
        if mem is not None:
            self.dac.parser.mem.init = mem
        self.outputs = []
        self.dac.parser.frame.reset = 0

    def do_simulation(self, selfp):
        self.outputs.append(selfp.dac.out.data)
        if selfp.simulator.cycle_counter == 5:
            selfp.dac.parser.start = 1
            selfp.dac.parser.arm = 1
            selfp.dac.out.arm = 1
        elif selfp.simulator.cycle_counter == 20:
            selfp.dac.out.trigger = 1
        elif selfp.simulator.cycle_counter == 21:
            selfp.dac.out.trigger = 0
        if selfp.dac.out.sink.ack and selfp.dac.out.sink.stb:
            print("cycle {} data {}".format(
                selfp.simulator.cycle_counter, selfp.dac.out.data))


def _main():
    # from migen.fhdl import verilog
    # print(verilog.convert(Dac()))

    t = np.arange(7) * 18
    t = t.astype(np.int32)
    v = (1 << 14)*(1 - np.cos(t/t[-1]*2*np.pi))/2
    v = v.astype(np.int16)
    k = 3
    p = pdq2.Pdq2(dev="dummy")
    c = p.channels[0]
    s = c.new_segment()
    s.dac(t, v, order=k, first=dict(trigger=True))
    s.dds(2*t, (v/c.cordic_gain).astype(np.int16),
          0*t + (1 << 14), (t/t[-1]*(1 << 13)).astype(np.int16),
          first=dict(trigger=False))
    mem = c.serialize()
    tb = TB(list(np.fromstring(mem, "<u2")))
    run_simulation(tb, ncycles=400, vcd_name="dac.vcd")

    plt.plot(t, v, "xk")

    sp = interpolate.splrep(t, v, k=k, s=0)
    tt = np.arange(t[-1])
    vv = interpolate.splev(tt, sp)
    plt.plot(tt, vv, "+g")

    vv1 = []
    widths = np.array([0, 1, 2, 2])
    dv = pdq2.Segment.interpolate(t, v, k, t[:-1], widths)
    dv = dv/2**(16*widths)

    for i, (ti, dvi) in enumerate(zip(t, dv)):
        dt = 0
        while ti + dt < t[i + 1]:
            dt += 1
            vv1.append(dvi[0])
            for ki in range(k):
                dvi[ki] += dvi[ki + 1]
    plt.step(tt + 1, vv1, "-g")

    out = np.array(tb.outputs, np.uint16).view(np.int16)
    plt.step(np.arange(len(out)) - 22, out, "-r")
    plt.show()


if __name__ == "__main__":
    _main()
