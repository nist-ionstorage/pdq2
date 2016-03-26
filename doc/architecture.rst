Architecture
============

The PDQ2 is an interpolating, scalable, high speed arbitrary waveform generator.

    * Outputs: 16 bit DACs, +- 10V
    * Sample rate and interpolation speed: 50 MHz or 100 MHz online selectable.
    * Scalability: Three DACs per board.
      Up to 16 boards stackable to provide 48 channels per USB device.
      Number of PDQ2 stacks limited by maximum number of USB devices per computer.
    * Memory: 16 KiB or 8 KiB per channel. Compact partitionable data format.
    * Interpolation: DC bias B-spline: constant, linear, quadratic, or cubic.
      Selectable for each spline knot, each channel.
    * DDS output per channel: 32 bit frequency, 16 bit phase offset, 48 bit frequency chirp.
      Cubic spline amplitude modulation, aligned with frequency/phase modulator.
      DDS output added to DC bias spline.
    * Digital outputs: One AUX channel per board, synchronous to spline knots.
    * External control, synchronization: One TTL trigger control input to trigger the execution of marked spline knots.
    * Frame selection: Eight separate frames each describing a waveform. Selectable in hard real-time using three TTL frame select signals.

Spline Interpolation
--------------------

Many use cases of analog voltages in physics experiments do not continuously need large bandwidth analog signals yet the signals need to be clean and with very small content of spurious frequencies.
Either a large bandwidth at very small duty cycle or a very small bandwidth at longer duty cycles is sufficient.
It is therefore prudent to generate, represent, transfer, and store the output waveform data in a compressed format.

The method of compression chosen here is a polynomial basis spline (B-spline).
The data consists of a sequence of knots.
Each knot is described by a duration :math:`\Delta t` and spline coefficients :math:`u_n` up to order :math:`k`.
If the knot is evaluated starting at time :math:`t_0`, the output :math:`u(t)` for :math:`t \in [t_0, t_0 + \Delta t]` is:

.. math::
    u(t) = \sum_{n=0}^k \frac{u_n}{n!} (t - t_0)^n
    = u_0 + u_1 (t - t_0) + \frac{u_2}{2} (t - t_0)^2 + \dots

A sequence of such knots describes a spline waveform.
Such a polynomial segment can be evaluated and evolved very efficiently using only iterative accumulation (recursive addition) without the need for any multiplications and powers that would require scarce resources on programmable logic.
From one discrete time :math:`i` to the next :math:`i + 1` each accumulators :math:`v_{n, i}` is incremented by the value of the next higher order accumulator:

.. math::
    v_{n, i + 1} = v_{n, i} + v_{n + 1, i}

For a cubic spline the mapping between the accumulators' initial values :math:`v_{n, 0}` and the polynomial derivatives or spline coefficients :math:`u_n` can be done off-line and ahead of time.
The mapping includes corrections due to the finite time step size :math:`\tau`.

.. math::
    t_i &= t_0 + i\tau\\
    v_{n, i} &= u_n(t_i)\\
    v_{0, 0} &= u_0\\
    v_{1, 0} &= u_1\tau + \frac{u_2 \tau^2}{2} + \frac{u_3 \tau^3}{6}\\
    v_{2, 0} &= u_2\tau^2 + u_3\tau^3\\
    v_{3, 0} &= u_3\tau^3

The data for each knot is then described by the integer duration :math:`T = \Delta t/\tau` and the initial values :math:`v_{n, 0}`.

This representation allows both very fast transient high bandwidth waveforms and slow but smooth  large duty cycle waveforms to be described efficiently.

CORDIC
------

Trigonometric functions can also be represented efficiently on programmable logic using only additions, comparisons and bit shifts.
See :mod:`gateware.cordic` for a full documentation of the features, capabilities, and constraints.


.. _features:

Features
--------

Each PDQ2 card contains one FPGA that feeds three DAC channels.
Multiple PDQ2 cards can be combined into a stack.
There is one data connection and one set of digital control lines connected to a stack, common to all cards, all FPGAs, and all channels in that stack.

Each channel of the PDQ2 can generate a waveform :math:`w(t)` that is the sum of a cubic spline :math:`a(t)` and a sinusoid modulated in amplitude by a cubic spline :math:`b(t)` and in phase/frequency by a quadratic spline :math:`c(t)`:

.. math::
    w(t) = a(t) + b(t) \cos(c(t))

The data is sampled at 50 MHz or 100 MHz and 16 bit resolution.
The higher order spline coefficients (including the frequency offset :math:`c_1`) receive successively wider data words due to their higher dynamic range.

The duration of a spline knot is of the form:

.. math::
    \Delta t = 2^E T \tau_0 = T \tau

Here, :math:`T` is a 16 bit unsigned integer and :math:`E` is a 4 bit unsigned integer.
The spline time step is accordingly scaled to :math:`\tau = 2^E \tau_0` where :math:`\tau_0` = 20 ns or 10 ns to accommodate the corresponding change in dynamic range of the coefficients.
The only exception to the scaling is the frequency offset :math:`c_1` which is always unscaled.
At 100 MHz sampling rate, this allows for knot durations anywhere between up to 655 µs at 10 ns resolution and up to 43 s at 655 µs resolution.
The encoding of the spline coefficients and associated metadata is described in :ref:`data-format`.

The execution of a knot can be delayed until a trigger signal is received.
The trigger signal is common to all channels of all cards in a stack.

Each channel can play waveforms from any of eight frames, selected by the three frame signals that are common to all channels and all boards of a stack.
All frames of a channel share the same memory.
The memory layout is described in :ref:`memory-layout`.
Transitions between frames happen at the end of frames.
Frames can be aborted at the end of a spline knot by disarming the stack.

Each channel also has one digital output `aux` that can be set or cleared at each knot.
The logical OR of each set of three channels is mapped to the F5 output of each PDQ2 card.

The waveform data is written into the channel memories over a full speed USB link.
Each channel memory can be accessed individually.
Data or status messages can not be read back.

The USB channel also carries in-band control commands to switch the clock speed between 50 MHz and 100 MHz, reset the device, arm or disarm the device, enable or disable soft triggering, and enable or disable the starting of new frames.
The USB protocol is described in :ref:`usb-protocol`.

The host side software receives waveform data in an easy-to generate, portable, and human readable format that is then encoded and written to the channels attached to a device.
This wavesynth format is described in :ref:`wavesynth-format`.
