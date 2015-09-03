import numpy as np
from scipy.interpolate import splev, splrep

    def build_program(self, program_data, order=3):
        # data = [ # list of frames [ # list of segments
        # (t, u) # ((n_times, ), (n_channels, n_times))
        # ]]
        program = []
        for frame_data in program_data:
            frame = []
            program.append(frame)
            for t, u in frame_data:
                segment = [{} for i in range(len(t) - 1)]
                frame.append(segment)
                self.build_segment_bias_program(segment, t, u, order)
        return program
    
    def build_segment_bias_program(self, segment, t, u, order=3):

