PDQ
===


New dac features
----------------

* 100 MHz operation by doubling the clock
* supports digital aux out channel
* dds mode with phase, frequency, amplitude and their sweeps
* supports frame repetition and frame chaining
* supports more (256) frames
* supports longer wait times up to 2**32 (43 seconds)
* replaces zero-dt constant voltage mode with finite dt constant voltage
mode (in line of the higher derivatives modes)
* supports aux-only mode
* each line has its own mode
* complete simulation and testbench support for ft245r, communications,
controller, dac, full system


Communications
--------------

* supports "escape" commands. escape char is 0xaa: commands are reset,
trigger, arm
* ft245 timings adapted to what the datasheet allows
* comm protocol simplified to only support commands interleaved with
writes to memory (board address, dac address, memory start and data
length).


Memory map
----------

* memory format is frame address, followed by many "frames"
* each frame is a pointer to the next, a repeat counter, its
length and finally multiple "lines"
* frame entry addresses obtained by indirect jumps (frame N jump address is
stored at mem[N]).
* each frame has a pointer to another frame that will be executed after it
* each frame has a repeat counter
* first 8 frames (interrupts) can be jumped to (interrupting the current
frame, after the current line) by hardware
* each line contains metadata, duration and voltage derivatives
* metadata is whether to wait for trigger, aux out value, max derivative
order, whether to override pending trigger, and a time scaling factor
* clock is slowed down by 2**shift (to achieve longer line and frame duration,
up to 2**32 cycles).


# MEM:
#   IRQ0_ADR 16
#   ...
#   IRQ7_ADR 16
#   ADDITIONAL_ADR 16
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
#     LENGTH 4
#     TYP 2
#     WAIT 1
#     TRIGGER 1
#     SHIFT 4
#     AUX 1
#     RESERVED 3
#   DT 16
#   UNION DATA 0-15*16
#     VOLT-TYP:
#       V0 16
#       V1 32
#       V2 48
#       V3 48
#     DDS-TYP:
#       Z0 16
#       X0 16
#       Z1 32
#       X1 32
#       Z2 48
