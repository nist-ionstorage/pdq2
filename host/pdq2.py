# Copyright 2013-2015 Robert Jordens <jordens@gmail.com>
#
# This file is part of pdq2.
#
# pdq2 is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# pdq2 is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with pdq2.  If not, see <http://www.gnu.org/licenses/>.

from math import log, sqrt
import logging
import struct

import serial


logger = logging.getLogger(__name__)


def discrete_compensate(c):
    l = len(c)
    if l > 2:
        c[1] += c[2]/2.
    if l > 3:
        c[1] += c[3]/6.
        c[2] += c[3]
    if l > 4:
        raise ValueError("only third-order splines supported")


class Segment:
    max_time = 1 << 16  # uint16 timer
    max_val = 1 << 15  # int16 DAC
    max_out = 10.  # Volt
    out_scale = max_val/max_out
    cordic_gain = 1.
    for i in range(16):
        cordic_gain *= sqrt(1 + 2**(-2*i))

    def __init__(self):
        self.data = b""
        self.addr = None

    def line(self, typ, duration, data, trigger=False, silence=False,
             aux=False, shift=0, jump=False, clear=False, wait=False):
        assert len(data) % 2 == 0, data
        assert len(data)//2 <= 14
        # assert dt*(1 << shift) > 1 + len(data)//2
        header = (
            1 + len(data)//2 | (typ << 4) | (trigger << 6) | (silence << 7) |
            (aux << 8) | (shift << 9) | (jump << 13) | (clear << 14) |
            (wait << 15)
        )
        self.data += struct.pack("<HH", header, duration) + data

    @staticmethod
    def pack(widths, values):
        fmt = "<"
        ud = []
        for width, value in zip(widths, values):
            value = int(round(value * (1 << 16*width)))
            if width == 2:
                ud.append(value & 0xffff)
                fmt += "H"
                value >>= 16
                width -= 1
            ud.append(value)
            fmt += "hi"[width]
        try:
            return struct.pack(fmt, *ud)
        except struct.error as e:
            logger.error("can not pack %s as %s (%s as %s): %s",
                         values, widths, ud, fmt, e)
            raise e

    def bias(self, amplitude=[], **kwargs):
        """Append a bias line to this segment.

        Amplitude in volts
        """
        coef = [self.out_scale*a for a in amplitude]
        discrete_compensate(coef)
        data = self.pack([0, 1, 2, 2], coef)
        self.line(typ=0, data=data, **kwargs)

    def dds(self, amplitude=[], phase=[], **kwargs):
        """Append a dds line to this segment.

        Amplitude in volts,
        phase[0] in turns,
        phase[1] in turns*sample_rate,
        phase[2] in turns*(sample_rate/2**shift)**2
        """
        scale = self.out_scale/self.cordic_gain
        coef = [scale*a for a in amplitude]
        discrete_compensate(coef)
        if phase:
            assert len(amplitude) == 4
        coef += [p*self.max_val*2 for p in phase]
        data = self.pack([0, 1, 2, 2, 0, 1, 1], coef)
        self.line(typ=1, data=data, **kwargs)


class Channel:
    num_frames = 8
    max_data = 4*(1 << 10)  # 8kx16 8kx16 4kx16

    def __init__(self):
        self.segments = []

    def clear(self):
        self.segments.clear()

    def new_segment(self):
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

    _escape = b"\xa5"
    _commands = "RESET TRIGGER ARM DCM START".split()

    def __init__(self, url=None, dev=None, num_boards=3):
        if dev is None:
            dev = serial.serial_for_url(url)
        self.dev = dev
        self.num_boards = num_boards
        self.num_channels = self.num_dacs * self.num_boards
        self.channels = [Channel() for i in range(self.num_channels)]

    def close(self):
        self.dev.close()
        del self.dev

    def write(self, data):
        logger.debug("> %r", data)
        written = self.dev.write(data)
        if isinstance(written, int):
            assert written == len(data)

    def cmd(self, cmd, enable):
        cmd = self._commands.index(cmd) << 1
        if not enable:
            cmd |= 1
        self.write(struct.pack("cb", self._escape, cmd))

    def write_mem(self, channel, data, start_addr=0):
        board, dac = divmod(channel, self.num_dacs)
        data = struct.pack("<HHH", (board << 4) | dac, start_addr,
                           start_addr + len(data)//2 - 1) + data
        data = data.replace(self._escape, self._escape + self._escape)
        self.write(data)

    def program_segments(self, segments, data):
        for i, line in enumerate(data):
            dac_divider = line.get("dac_divider", 1)
            shift = int(log(dac_divider, 2))
            if 2**shift != dac_divider:
                raise ValueError("only power-of-two dac_dividers supported")
            duration = line["duration"]
            trigger = line.get("trigger", False)
            for segment, data in zip(segments, line["channel_data"]):
                if len(data) != 1:
                    raise ValueError("only one target per channel and line "
                                     "supported")
                for target, target_data in data.items():
                    getattr(segment, target)(
                        shift=shift, duration=duration, trigger=trigger,
                        **target_data)

    def program(self, program, channels=None):
        if channels is None:
            channels = range(self.num_channels)
        chs = [self.channels[i] for i in channels]
        for channel in chs:
            channel.clear()
        for frame in program:
            segments = [c.new_segment() for c in chs]
            for segment in segments:
                segment.line(typ=3, data=b"", trigger=True, duration=1, aux=1)
            self.program_segments(segments, frame)
            # append an empty line to stall the memory reader before jumping
            # through the frame table (`wait` does not prevent reading
            # the next line)
            for segment in segments:
                segment.line(typ=3, data=b"", trigger=True, duration=1, aux=1,
                             jump=True)
        for channel, ch in zip(channels, chs):
            self.write_mem(channel, ch.serialize())
