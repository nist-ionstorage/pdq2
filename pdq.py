#!/usr/bin/python
# -*- coding: utf8 -*-
#
# Robert Jordens <jordens@gmail.com>, 2012
#

import logging, struct
import numpy as np
from scipy import interpolate


class PyFtdi(object):
    def __init__(self, serial=None):
        self.dev = pylibftdi.Device(device_id=serial)

    def write(self, data):
        written = self.dev.write(data)
        if written < 0:
            raise pylibftdi.FtdiError(written,
                    self.dev.get_error_string())
        return written


class D2xxFtdi(object):
    def __init__(self, serial=None):
        if serial is not None:
            self.dev = ftd2xx.openEx(serial)
        else:
            self.dev = ftd2xx.open()
        self.dev.setTimeouts(read=5000, write=5000)

    def write(self, data):
        written = self.dev.write(str(data))
        return written


class FileFtdi(object):
    def __init__(self, serial="unknown"):
        self.fil = open("pdq_%s_ftdi.bin" % serial, "wb")
    def write(self, data):
        written = self.fil.write(data)
        return len(data)


try:
    import pylibftdi
    Ftdi = PyFtdi
except ImportError:
    try:
        import ftd2xx
        Ftdi = D2xxFtdi
    except ImportError:
        Ftdi = FileFtdi



class Pdq(object):
    """
    PDQ DAC (a.k.a. QC_Waveform)
    """
    range = (1<<15)-1 # signed 16 bit DAC
    scale = range/10. # LSB/V
    freq = 50e6 # samples/s
    min_time = 11 # processing time for a mode=3 point
    max_time = range # signed 16 bit timer
    num_boards = 3
    num_dacs = 3
    max_data = 4*(1<<10) # 6144 data buffer size per channel
    escape = 0x5a

    commands = {
            "RESET_EN":    0x00,
            "RESET_DIS":   0x01,
            "TRIGGER_EN":  0x02,
            "TRIGGER_DIS": 0x03,
            "ARM_EN":      0x04,
            "ARM_DIS":     0x05,
            }
        
    # DEV_ADR:
    #   BOARD 8
    #   DAC 8
    # MEM_ADR 16
    # DATA_LEN 16
    # MEM_DATA
    #   
    # MEM:
    #   IRQ0_ADR 16
    #   ...
    #   IRQ7_ADR 16
    #   JUMP_ADR 16
    #   ...
    #   FRAME
    #   ...
    # FRAME:
    #   MODE 16:
    #       NEXT_FRAME 8
    #       REPEAT 8
    #   LENGTH 16
    #   LINE
    #   ...
    # LINE:
    #   HEADER 16:
    #     TYP 4
    #     WAIT 1
    #     TRIGGER 1
    #     SHIFT 4
    #     AUX 4
    #     RESERVED 2
    #   DT 16
    #   (V0 16)
    #   (V1 32)
    #   (V2 48)
    #   (V3 48)


    def __init__(self, serial=None):
        self.dev = Ftdi(serial)

    def cmd(self, cmd):
        return bytes([self.escape, self.commands[cmd]])

    @staticmethod
    def interpolate(times, voltages, mode):
        """
        calculate b-spframe interpolation derivatives for voltage data
        according to interpolation mode
        returns times, voltages and derivatives suitable for passing to
        frame()
        also removes the first times point (implicitly shifts to 0) and
        the last voltages/derivatives (irrelevant since the section ends
        here)
        """
        derivatives = []
        if mode in (1, 2, 3):
            spframe = interpolate.splrep(times, voltages, s=0, k=mode)
            derivatives = [interpolate.splev(times[:-1], spframe, der=i+1)
                    for i in range(mode)]
        voltages = voltages[:-1]
        times = times[1:]
        return times, voltages, derivatives

    def frame(self, times, voltages, derivatives=None, aux=None,
            time_shift=0, trigger_first=False, wait_last=True):
        """
        serialize frame data
        voltages in volts, times in seconds,
        derivatives can be a number in which case the derivatives will
        be the interpolation, else it should be a list of derivatives
        """
        if type(derivatives) is type(1):
            times, voltages, derivatives = self.interpolate(times,
                    voltages, derivatives)
        order = len(derivatives)
        length = len(times)
        voltages = [voltages] + list(derivatives)

        frame = []

        head = np.zeros((length,), "u2")
        head[:] |= (order+1)<<0
        head[-1] |= wait_last<<4
        head[0] |= trigger_first<<5
        head[:] |= time_shift<<6
        if aux is not None:
            head[:] |= aux[:length]<<10
        #head[:] |= reserved<<14
        frame.append(head)

        dt = np.diff(np.r_[0, times])*(self.freq/(1<<time_shift))
        # FIXME: clipped intervals cumulatively skew
        # subsequent ones
        np.clip(dt, self.min_time, self.max_time, out=dt)
        frame.append(dt.astype("i2"))

        byts = [2, 4, 6, 6]
        for i, (v, byt) in enumerate(zip(voltages, byts)):
            scale = self.scale*(1<<(8*(byt-2)))/self.freq**i
            v = np.rint(scale*v).astype("i8")
            # np.clip(v, -self.range, self.range, out=v)
            if byt <= 4:
                frame.append(v.astype("i%i" % byt))
            else: # no i6 type
                frame.append(v.astype("i4"))
                frame.append((v>>32).astype("i%i" % (byt-4)))

        frame = np.rec.fromarrays(frame, byteorder="<") # interleave
        data_length = len(frame.data.tobytes())
        logging.debug("frame %s dtype %s shape %s length %s",
                frame, frame.dtype, frame.shape, data_length)
        bytes_per_line = {0: 2+2+2, 1: 2+2+2+4, 2: 2+2+2+4+6, 3: 2+2+2+4+6+6}
        bpl = bytes_per_line[order]
        assert data_length == bpl*length, (data_length, bpl, length,
                frame.shape)
        return frame

    @staticmethod
    def add_frame_header(frame, repeat, next):
        head = struct.pack("<BBH", next, repeat, frame.shape[0])
        logging.debug("frame header %r", head)
        return head + frame.data.tobytes()

    def combine_frames(self, frames):
        lens = [len(frame)//2 for frame in frames[:-1]] # 16 bits
        mems = np.cumsum([len(frames)] + lens)
        chunk = mems.astype("<u2").data.tobytes()
        logging.debug("mem len %i, %r", len(chunk), chunk)
        for frame in frames:
            chunk += frame
        assert len(chunk) <= self.max_data
        return chunk

    def add_mem_header(self, board, dac, mem_adr, chunk):
        length = len(chunk)//2 # 16 bit memory
        assert dac in range(self.num_dacs)
        assert board in range(self.num_boards)
        head = struct.pack("<BBHH", board, dac, mem_adr, length)
        logging.debug("mem header %r", head)
        return head + chunk

    def multi_frame(self, times_voltages, channel, mode=3, repeat=1,
            next=None, **frame_kwargs):
        frames = []
        for i, (t, v) in enumerate(times_voltages):
            n = i if next is None else next
            frame = self.frame(t, v, derivatives=mode, **frame_kwargs)
            frames.append(self.add_frame_header(frame, repeat, n))
        data = self.combine_frames(frames)
        board, dac = divmod(channel, self.num_dacs)
        data = self.add_mem_header(board, dac, 0, data)
        logging.debug("write %s, len %i", list(map(hex, data)), len(data))
        return data

    def single_frame(self, times, voltages, **kwargs):
        """
        interpolate, serialize and upload 8 irq data for one channel
        """
        return self.multi_frame([(times, voltages)], **kwargs)

    def write(self, *segments):
        """
        writes segments to device
        segments is a flat sequence of channel header and branch data,
        pseudo-regex: (header branch*)*
        """
        for segment in segments:
            # escape escape
            segment = segment.replace(bytes([self.escape]),
                    bytes([self.escape, self.escape]))
            written = self.dev.write(segment)
            if written < len(segment):
                logging.error("wrote only %i of %i", written, len(segment))

    def set_interrupt(self, board, channel, interrupt, addr):
        data = struct.pack("<H", addr)
        data = self.add_mem_header(board, channel, interrupt, data)
        return self.write(data)

    def write_cmd(self, cmd):
        return self.write(self.cmd(cmd))
    

def main():
    import argparse
    parser = argparse.ArgumentParser(description="""PDQ DAC frontend.
            Evaluates times and voltages, interpolates them and uploads
            them to one section in one branch for one channel.""")
    parser.add_argument("-s", "--serial", default=None,
            help="device (FT245R) serial string [first]")
    parser.add_argument("-c", "--channel", default=0, type=int,
            help="channel: 3*fpga_num+dac_num [%(default)s]")
    parser.add_argument("-i", "--interrupt", default=0, type=int,
            help="interrupt number [%(default)s]")
    parser.add_argument("-f", "--free", default=False,
            action="store_true",
            help="soft triggered, free running [%(default)s]")
    parser.add_argument("-n", "--disarm", default=False,
            action="store_true",
            help="disarm external trigger [%(default)s]")
    parser.add_argument("-t", "--times",
            default="np.linspace(0, 1e-3, 11)",
            help="sample times (s) [%(default)s]")
    parser.add_argument("-v", "--voltages",
            default="(1-np.cos(t/t[-1]*np.pi))/2",
            help="sample voltages (V) [%(default)s]")
    parser.add_argument("-m", "--mode", default=3, type=int,
            help="interpolation (0: v_only, 1: const, 2: quad, 3: cubic)"
                 " [%(default)s]")
    parser.add_argument("-x", "--test", default=False, action="store_true",
            help="test mode, assign all channels and all branches the"
                 "same waveform shifted by channel+.1*branch [%(default)s]")
    parser.add_argument("-p", "--plot",
            help="plot to file [%(default)s]")
    parser.add_argument("-d", "--debug", default=False,
            action="store_true",
            help="debug communications")

    args = parser.parse_args()

    if args.debug:
        logging.basicConfig(level=logging.DEBUG)
    else:
        logging.basicConfig(level=logging.WARNING)

    times = eval(args.times)
    voltages = eval(args.voltages, globals(), dict(t=times))

    if args.plot:
        from matplotlib import pyplot as plt
        times -= times[0]
        spframe = interpolate.splrep(times, voltages,
                s=0, k=args.mode)
        ttimes = np.arange(0, times[-1], 1/Pdq.freq)
        vvoltages = interpolate.splev(ttimes, spframe, der=0)
        fig, ax0 = plt.subplots()
        ax0.plot(times, voltages, "xk", label="points")
        ax0.plot(ttimes, vvoltages, ",b", label="interpolation")
        fig.savefig(args.plot)

    dev = Pdq(serial=args.serial)
    channels = (args.channel == -1) and range(9) or [args.channel]
    for channel in channels:
        if args.interrupt == -1:
            v = [.1*interrupt+channel+voltages for interrupt in range(8)]
            t = [times] * 8
            data = dev.multi_frame(zip(t, v), channel, args.mode)
        else:
            data = dev.single_frame(times, voltages, channel=channel,
                    mode=args.mode)
        dev.write(data)
    dev.write_cmd("ARM_DIS" if args.disarm else "ARM_EN")
    dev.write_cmd("TRIGGER_EN" if args.free else "TRIGGER_DIS")

if __name__ == "__main__":
    main()
