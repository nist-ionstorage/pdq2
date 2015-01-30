# Robert Jordens <jordens@gmail.com>, 2012-2015

import logging
import struct
import warnings

import numpy as np
from scipy import interpolate
import serial

logger = logging.getLogger(__name__)


class Frame:
    max_val = 1 << 15  # signed 16 bit DAC
    max_out = 10.
    max_time = 1 << 16  # unsigned 16 bit timer
    cordic_gain = 1.
    for i in range(16):
        cordic_gain *= np.sqrt(1 + 2**(-2*i))

    def __init__(self):
        self.data = b""

    @staticmethod
    def line(typ, dt, data, trigger=False, silence=False, aux=False,
             shift=0, end=False, clear=False, wait=False):
        assert len(data) % 2 == 0, data
        assert len(data)//2 <= 14
        #assert dt*(1 << shift) > 1 + len(data)//2
        head = (
            1 + len(data)//2 | (typ << 4) | (trigger << 6) | (silence << 7) |
            (aux << 8) | (shift << 9) | (end << 13) | (clear << 14) |
            (wait << 15)
        )
        return struct.pack("<HH", head, dt) + data

    @staticmethod
    def pack(widths, values):
        fmt = "<"
        ud = []
        for width, value in zip(widths, values):
            if width > 2:
                ud.append(value & 0xffff)
                fmt += "H"
                value >>= 16
                width -= 1
            ud.append(value)
            fmt += "hi"[width - 1]
        return struct.pack(fmt, *ud)

    def dac_line(self, dt, *u, **kwargs):
        # u0 u1 u2 u3
        data = self.pack([1, 2, 3, 3], u)
        self.data += self.line(0, dt, data, **kwargs)

    def dds_line(self, dt, *upf, **kwargs):
        # u0 u1 u2 u3 p0 f0 f1
        data = self.pack([1, 2, 3, 3, 1, 2, 2], upf)
        self.data += self.line(1, dt, data, **kwargs)

    def segment(self, typ, dt, widths, v, first={}, mid={}, last={}):
        n = len(dt) - 1
        for i, vi in enumerate(zip(dt, *v)):
            dti, vi = vi[0], vi[1:]
            data = self.pack(widths, vi)
            if i == 0:
                opts = first
            elif i == n:
                opts = last
            else:
                opts = mid
            self.data += self.line(typ, dti, data, **opts)

    @staticmethod
    def interpolate(t, v, order, t_eval):
        """Spline interpolating derivatives for t,v.
        The returned spline coefficients are one shorter than t
        """
        s = interpolate.splrep(t, v, k=order)
        dv = np.array(interpolate.spalde(t_eval, s))
        # correct for adder chain latency
        if order > 1:
            dv[:, 1] += dv[:, 2]/2
        if order > 2:
            dv[:, 1] += dv[:, 3]/6
            dv[:, 2] += dv[:, 3]
        return np.rint(dv).astype("i8")

    def line_times(self, t, freq, shift, tr):
        t = t*freq/2**shift
        if tr is None:
            tr = t
        else:
            tr = tr*freq/2**shift
        tr = np.rint(tr).astype(np.uint16)
        dt = np.diff(tr)
        assert np.all(dt >= 0)
        assert np.all(dt < 1 << 16)
        return t, tr[:-1], dt

    def dac_segment(self, t, v,
                    first={}, mid={}, last={},
                    freq=50e6, shift=0, tr=None, order=3, stop=True):
        t, tr, dt = self.line_times(t, freq, shift, tr)
        v = np.clip(v/self.max_out, -1, 1)*self.max_val
        # FIXME: does not ensure that interpolates do not clip
        order = min(order, len(t) - 1)
        dv = self.interpolate(t, v, order, tr)
        self.segment(0, dt, [1, 2, 3, 3], dv, first, mid,
                     mid if stop else last)
        if stop:
            self.line(0, 1, self.pack([1], [int(v[-1])]), **last)

    def dds_segment(self, t, v, p=None, f=None,
                    first={}, mid={}, last={},
                    freq=50e6, shift=0, tr=None, order=3, stop=True):
        if order < 3:
            assert p is None and f is None
        t, tr, dt = self.line_times(t, freq, shift, tr)
        v = np.clip(v/self.max_out, -1, 1)*self.max_val/self.cordic_gain
        if p is not None:
            p = np.rint(p/np.pi*self.max_val).astype(np.int16)
        if v is not None:
            f = f/freq*self.max_val
        # FIXME: does not ensure that interpolates do not clip
        dv = self.interpolate(t, v, order, tr)
        if p is not None:
            dv = np.concatenate((dv, p[:-1, None]), axis=1)  # FIXME: use tr
        if f is not None:
            dv = np.concatenate((dv, self.interpolate(t, f, 1, tr)), axis=1)
        self.segment(1, dt, [1, 2, 3, 3, 1, 2, 2], dv, first, mid,
                     mid if stop else last)
        if stop:
            data = self.pack([1, 2, 3, 3, 1, 2, 2],
                             [int(v[-1]), 0, 0, 0, int(p[-1]), int(f[-1]), 0])
            self.line(0, 1, data, **last)


class Channel:
    max_data = 4*(1 << 10)  # 8kx16 8kx16 4kx16
    num_frames = 8

    def __init__(self):
        self.frames = []

    def place(self):
        addr = self.num_frames
        for frame in self.frames:
            frame.addr = addr
            addr += len(frame.data)//2
        assert addr <= self.max_data, addr
        return addr

    def table(self, frames):
        table = [0] * self.num_frames
        for i, frame in enumerate(frames):
            table[i] = frame.addr
        return struct.pack("<" + "H"*self.num_frames, *table)

    def serialize(self):
        self.place()
        frames = b"".join([frame.data for frame in self.frames])
        return self.table(self.frames) + frames

    def frame(self):
        frame = Frame()
        self.frames.append(frame)
        return frame


class Pdq2:
    """
    PDQ DAC (a.k.a. QC_Waveform)
    """
    num_dacs = 3
    num_boards = 3
    num_channels = num_dacs*num_boards
    freq = 50e6  # samples/s

    _escape = b"\xa5"
    _commands = {
        "RESET":   0,
        "TRIGGER": 1,
        "ARM":     2,
        "DCM":     3,
        "START":   4,
    }

    def __init__(self, url=None, dev=None):
        if dev is None:
            dev = serial.serial_for_url(url)
        self.dev = dev
        self.channels = [Channel() for i in range(self.num_channels)]

    def close(self):
        self.dev.close()
        del self.dev

    def write(self, data):
        written = self.dev.write(data)
        assert written == len(data)

    def cmd(self, cmd, enable):
        cmd = self._commands[cmd] << 1
        if not enable:
            cmd |= 1
        self.write(self._escape + bytes([cmd]))

    def write_cmd(self, cmd):
        warnings.warn("deprecated, use cmd()", DeprecationWarning)
        if cmd.endswith("_EN"):
            self.cmd(cmd[:-len("_EN")], True)
        else:
            self.cmd(cmd[:-len("_DIS")], False)

    def write_mem(self, channel, data, start_addr=0):
        board, dac = divmod(channel, self.num_dacs)
        data = struct.pack("<HHH", (board << 4) | dac, start_addr,
                           start_addr + len(data)//2 - 1) + data
        data = data.replace(self._escape, self._escape + self._escape)
        self.write(data)

    def write_all(self):
        for i, channel in enumerate(self.channels):
            self.write_mem(i, channel.serialize())

    def write_table(self, channel, frames):
        # no frame placement
        # no frame writing
        self.write_mem(channel, self.channels[channel].table(frames))

    def write_frame(self, channel, frame):
        # no collision check
        f = self.channels[channel].frames[frame]
        self.write_mem(channel, f.data, f.adr)

    def frame(self, t, v, p=None, f=None,
              order=3, aux=False, shift=0, trigger=True, end=True,
              silence=False, stop=True, clear=True, wait=False):
        warnings.warn("deprecated, use channels[i].open_frame() etc",
                      DeprecationWarning)
        frame = Frame()
        first = dict(trigger=trigger, clear=clear, aux=aux)
        mid = dict(aux=aux)
        last = dict(silence=silence, end=end, wait=wait, aux=aux)
        if p is None and f is None:
            frame.dac_segment(t, v, first, mid, last, self.freq, shift,
                              order=order, stop=stop)
        else:
            frame.dds_segment(t, v, p, f, first, mid, last, self.freq, shift,
                              order=order, stop=stop)
        return frame.data

    def map_frames(self, frames):
        warnings.warn("deprecated", DeprecationWarning)
        c = Channel()
        for fd in frames:
            c.frame().data = fd
        return c.serialize()

    def multi_frame(self, times_voltages, channel, map=None, **kwargs):
        warnings.warn("deprecated, use channels[i].ope_frame() etc",
                      DeprecationWarning)
        c = Channel()
        c.frames = [self.frame(t, v, **kwargs) for t, v in times_voltages]
        self.write_mem(channel, c.serialize())
