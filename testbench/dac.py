from migen.fhdl.std import *
from migen.sim.generic import run_simulation

from matplotlib import pyplot as plt
import numpy as np
from scipy import interpolate

from gateware.dac import Dac
from host import pdq2
pdq2.Ftdi = pdq2.FileFtdi


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
    #from migen.fhdl import verilog
    #print(verilog.convert(Dac()))

    t = np.arange(0, 5) * .12e-6
    v = 9*(1-np.cos(t/t[-1]*2*np.pi))/2
    p = pdq2.Pdq2()
    p.freq = 100e6
    k = 3
    mem = p.map_frames([b"".join([
            p.frame(t, v, order=k, end=False),
            p.frame(2*t, v, 0*t+np.pi/2, 20e6*t/t[-1], trigger=False)
    ])])
    tb = TB(list(np.fromstring(mem, "<u2")))
    run_simulation(tb, ncycles=250, vcd_name="dac.vcd")

    plt.plot(t, v, "xk")

    sp = interpolate.splrep(t, v, k=k)
    tt = np.arange(t[0], t[-1], 1/p.freq)

    vv = interpolate.splev(tt, sp)
    plt.plot(tt, vv, "+g")

    vv1 = []
    dv = p.interpolate(t*p.freq, v, order=k)
    j = 0
    for i, tti in enumerate(tt):
        if tti >= t[j]:
            v = [dvi[j] for dvi in dv]
            k = np.searchsorted(tt, t[j + 1])
            j += 1
        vv1.append(v[0])
        for k in range(len(v) - 1):
            v[k] += v[k+1]
    plt.step(tt + 1/p.freq, vv1, "-g")

    out = np.array(tb.outputs, np.uint16).view(np.int16)*20./(1<<16)
    tim = np.arange(out.shape[0])/p.freq
    plt.step(tim - 22/p.freq, out, "-r")
    plt.show()


if __name__ == "__main__":
    _main()
