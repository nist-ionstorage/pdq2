from migen.fhdl.std import *

class Dac(Module):
    def __init__(self, pads):
        self.specials.mem = Memory(width=18, depth=6144)
        mem_read = self.mem.get_port()
        self.specials += mem_read
        self.data = Signal(16)
        self.mode = Signal(2)
        self.dt = Signal(16)
        self.t = Signal(16)
        self.v0 = Signal(48)
        self.v1 = Signal(48)
        self.v2 = Signal(48)
        self.v3 = Signal(48)

        self.comb += [
                self.data.eq(self.v0[-16:]),
                    ]
        self.sync += [
                If(self.t < self.dt, self.t.eq(self.t + 1)),
                If(self.mode >= 1, self.v0.eq(self.v0 + self.v1)),
                If(self.mode >= 2, self.v1.eq(self.v1 + self.v2)),
                If(self.mode >= 3, self.v2.eq(self.v2 + self.v3)),
                    ]
