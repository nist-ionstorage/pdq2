# Robert Jordens <jordens@gmail.com>, 2012-2015

import logging
import struct
import warnings

import numpy as np
from scipy import interpolate
import serial

logger = logging.getLogger(__name__)


class Segment:
    max_val = 1 << 15  # signed 16 bit DAC
    max_time = 1 << 16  # unsigned 16 bit timer
    cordic_gain = 1.
    for i in range(16):
        cordic_gain *= np.sqrt(1 + 2**(-2*i))

    def __init__(self):
        self.data = b""

    def line(self, typ, dt, data, trigger=False, silence=False,
             aux=False, shift=0, end=False, clear=False, wait=False):
        assert len(data) % 2 == 0, data
        assert len(data)//2 <= 14
        #assert dt*(1 << shift) > 1 + len(data)//2
        head = (
            1 + len(data)//2 | (typ << 4) | (trigger << 6) | (silence << 7) |
            (aux << 8) | (shift << 9) | (end << 13) | (clear << 14) |
            (wait << 15)
        )
        self.data += struct.pack("<HH", head, dt) + data

    @staticmethod
    def pack(widths, values):
        fmt = "<"
        ud = []
        for width, value in zip(widths, values):
            if width == 3:
                ud.append(value & 0xffff)
                fmt += "H"
                value >>= 16
                width -= 1
            ud.append(value)
            fmt += " hi"[width]
        return struct.pack(fmt, *ud)

    def lines(self, typ, dt, widths, v, first={}, mid={}, last={}, shift=0):
        n = len(dt) - 1
        for i, (dti, vi) in enumerate(zip(dt, v)):
            opts = mid
            if i == 0:
                opts = first
            elif i == n:
                opts = last
            data = self.pack(widths, vi)
            self.line(typ, dti, data, shift=shift, **opts)

    @staticmethod
    def interpolate(t, v, order, t_eval, widths=None):
        """Spline interpolating derivatives for t,v.
        The returned spline coefficients are one shorter than t
        """
        # FIXME: does not ensure that interpolates do not clip
        s = interpolate.splrep(t, v, k=order)
        dv = np.array(interpolate.spalde(t_eval, s))
        # correct for adder chain latency
        if order > 1:
            dv[:, 1] += dv[:, 2]/2
        if order > 2:
            dv[:, 1] += dv[:, 3]/6
            dv[:, 2] += dv[:, 3]
        if widths is not None:
            dv *= 1 << widths
        return np.rint(dv).astype(np.int64)

    def line_times(self, t):
        dt = np.diff(t)
        assert np.all(dt >= 0)
        assert np.all(dt < self.max_time)
        return t[:-1], dt

    def dac(self, t, v, first={}, mid={}, last={},
            shift=0, tr=None, order=3, stop=True):
        if tr is None:
            tr = np.rint(t).astype(np.uint32)
        tr, dt = self.line_times(tr)
        widths = np.array([1, 2, 3, 3])
        dv = self.interpolate(t, v, order, tr, 16*(widths[:order + 1] - 1))
        self.lines(0, dt, widths, dv, first, mid, mid if stop else last, shift)
        if stop:
            self.line(0, 2, self.pack([1], v[-1:]), **last)

    def dds(self, t, v, p=None, f=None, first={}, mid={}, last={},
            shift=0, tr=None, order=3, stop=True):
        if order < 3:
            assert p is None
        if tr is None:
            tr = np.rint(t).astype(np.uint32)
        tr, dt = self.line_times(tr)
        widths = np.array([1, 2, 3, 3, 1, 2, 2])
        dv = self.interpolate(t, v, order, tr, 16*(widths[:order + 1] - 1))
        if p is not None:
            dp = self.interpolate(t, p, 1, tr)[:, :1]
            dv = np.concatenate((dv, dp), axis=1)
            if f is not None:
                df = self.interpolate(t, f, 1, tr, 16*(widths[-2:] - 1))
                dv = np.concatenate((dv, df), axis=1)
        self.lines(1, dt, widths, dv, first, mid, mid if stop else last, shift)
        if stop:
            data = self.pack([1, 2, 3, 3, 1, 2],
                             [v[-1], 0, 0, 0, p[-1], f[-1]])
            self.line(1, 2, data, **last)


class Channel:
    max_data = 4*(1 << 10)  # 8kx16 8kx16 4kx16
    num_frames = 8

    def __init__(self):
        self.segments = []

    def new_segment(self):
        # assert len(self.segments) < self.num_frames
        segment = Segment()
        self.segments.append(segment)
        return segment

    def place(self):
        addr = self.num_frames
        for segment in self.segments:
            segment.addr = addr
            addr += len(segment.data)//2
        assert addr <= self.max_data, addr
        return addr

    def table(self, entry=None):
        table = [0] * self.num_frames
        if entry is None:
            entry = self.segments
        for i, frame in enumerate(entry):
            if frame is not None:
                table[i] = frame.addr
        return struct.pack("<" + "H"*self.num_frames, *table)

    def serialize(self, entry=None):
        self.place()
        data = b"".join([segment.data for segment in self.segments])
        return self.table(entry) + data


class Pdq2:
    """
    PDQ DAC (a.k.a. QC_Waveform)
    """
    num_dacs = 3
    num_boards = 3
    num_channels = num_dacs*num_boards
    freq = 50e6  # samples/s
    max_out = 10.

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

    def write_channel(self, channel, entry=None):
        self.write_mem(channel, self.channels[channel].serialize(entry))

    def write_all(self):
        for i, channel in enumerate(self.channels):
            self.write_mem(i, channel.serialize())

    def write_table(self, channel, segments=None):
        # no segment placement
        # no segment writing
        self.write_mem(channel, self.channels[channel].table(segments))

    def write_segment(self, channel, segment):
        # no collision check
        s = self.channels[channel].segments[segment]
        self.write_mem(channel, s.data, s.adr)

    def segment(self, t, v, p=None, f=None,
                order=3, aux=False, shift=0, trigger=True, end=True,
                silence=False, stop=True, clear=True, wait=False):
        warnings.warn("deprecated", DeprecationWarning)
        segment = Segment()
        first = dict(trigger=trigger, clear=clear, aux=aux)
        mid = dict(aux=aux)
        last = dict(silence=silence, end=end, wait=wait, aux=aux)
        t = (t*(self.freq/2**shift)).astype(np.int)
        v = (np.clip(v/self.max_out, -1, 1)*segment.max_val).astype(np.int16)
        order = min(order, len(t) - 1)
        if p is None:
            segment.dac(t, v, first, mid, last, shift, order=order, stop=stop)
        else:
            v = (v/segment.cordic_gain).astype(np.int16)
            p = (p/np.pi*segment.max_val).astype(np.int16)
            if f is not None:
                f = (f/self.freq*segment.max_val).astype(np.int16)
            segment.dds(t, v, p, f, first, mid, last, shift, order=order)
        return segment

    def multi_segment(self, times_voltages, channel, map=None, **kwargs):
        warnings.warn("deprecated", DeprecationWarning)
        c = self.channels[channel]
        c.segments = [self.segment(t, v, **kwargs) for t, v in times_voltages]
        return c.serialize(map)
