from migen.fhdl.std import *
from migen.fhdl import verilog
from migen.genlib.record import Record
from migen.flow.actor import Source, Sink
from migen.flow.transactions import Token
from migen.sim.generic import run_simulation
from migen.actorlib.sim import SimActor

from matplotlib import pyplot as plt
import numpy as np
from scipy import interpolate

from gateware.ft245r import SimFt245r_rx
from gateware.dac import Dac
from gateware.comm import Comm


class SimReader(SimActor):
    def __init__(self, data):
        self.source = Source(bus_layout)
        SimActor.__init__(self, self.data_gen(data))

    def data_gen(self, data):
        for msg in data:
            yield Token("data_out", {"data": msg})


class TB(Module):
    comm_pads = [
            ("rxfl", 1),
            ("rdl", 1),
            ("rd_in", 1),
            ("rd_out", 1),
            ("data", 8),
            ("adr", 4),
            ("aux", 1),
            ("frame", 3),
            ("trigger", 1),
            ("reset", 1),
            ("go2_in", 1),
            ("go2_out", 1),
    ]

    def __init__(self, mem=None):
        self.pads = Record(self.comm_pads)
        self.pads.adr.reset = 15
        self.pads.trigger.reset = 1
        if mem is not None:
            #reader = SimReader(mem)
            simin = SimFt245r_rx(self.pads, list(mem))
            self.submodules += simin
        dacs = [InsertReset(Dac()) for i in range(3)]
        self.submodules.comm = InsertReset(Comm(self.pads, dacs), ["sys"])
        self.comb += self.comm.reset_sys.eq(self.comm.ctrl.reset)
        for i, dac in enumerate(dacs):
            setattr(self.submodules, "dac{}".format(i), dac)
            self.comb += dac.reset.eq(self.comm.ctrl.reset)
        self.outputs = []

    def do_simulation(self, selfp):
        self.outputs.append([
            getattr(selfp, "dac{}".format(i)).out.data for i in range(3)])
    do_simulation.passive = True


class SimFtdi:
    def __init__(self, serial=None):
        self.buffer = b""

    def write(self, data):
        self.buffer += data
        return len(data)

    def close(self):
        tb = TB(self.buffer)
        run_simulation(tb, vcd_name="pdq.vcd", ncycles=6000)
        out = np.array(tb.outputs, np.uint16).view(np.int16)*20./(1<<16)
        tim = np.arange(out.shape[0])/100e6
        plt.plot(tim, out)
        plt.show()


if __name__ == "__main__":
    from host import pdq2
    pdq2.Ftdi = SimFtdi
    pdq2._main()
