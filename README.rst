=============
PDQ2 Overview
=============

.. image:: https://zenodo.org/badge/doi/10.5281/zenodo.11567.png
  :target: http://dx.doi.org/10.5281/zenodo.11567
  :alt: Zenodo DOI URL
.. image:: https://travis-ci.org/m-labs/pdq2.svg?branch=master
  :target: https://travis-ci.org/m-labs/pdq2
  :alt: Continuous integration build and test
.. image:: http://readthedocs.org/projects/pdq2/badge/?version=latest
  :target: http://pdq2.readthedocs.org/en/latest/?badge=latest
  :alt: Documentation Status



A pretty darn quick interpolating arbitrary waveform generator.


Build
=====

Requirements:

  * The legacy branch of Migen (https://github.com/m-labs/migen/tree/legacy)
  * Xilinx ISE (WebPack is sufficient)

Installation of Migen differs depending on what packaging system is used (or if one is used at all). Migen can be installed using ``pip``::

  $ pip install -e git://github.com/m-labs/migen.git@legacy#egg=migen

Then to build the gateware::

  $ python make.py

The HTML documentation can be built with::

  $ pip install -r doc/requirements.txt
  $ make -C doc html


Programming
===========

Once the device has been programmed with the gateware and powered up, it can be used to generate waveforms.

See the :class:`host.pdq2.Pdq2` class for how to access a stack of PDQ2 board programmatically, how to submit commands, and how prepare, serialize, and program segments, frames, and channels.

An example how :class:`host.pdq2.Pdq2` can be used is the command line test interface to the PDQ2 in :func:`host.cli.main`.

Individual commands are described in the manual in :ref:`usb-protocol`.

The wavesynth format is described with examples in :ref:`wavesynth-format`.

To communicate with the device, run the testbenches and generate the data,
the following additional packages are required::

  * ``pyserial``
  * ``scipy``


Testbenches
===========

::

  $ python3 -m testbench.escape
  $ python3 -m testbench.cli


References
==========

Arbitrary waveform generator for quantum information processing with trapped
ions; R. Bowler, U. Warring, J. W. Britton, B. C. Sawyer and J. Amini;
Rev. Sci. Instrum. 84, 033108 (2013);
http://dx.doi.org/10.1063/1.4795552
http://tf.boulder.nist.gov/general/pdf/2668.pdf

Coherent Diabatic Ion Transport and Separation in a Multizone Trap Array;
R. Bowler, J. Gaebler, Y. Lin, T. R. Tan, D. Hanneke, J. D. Jost, J. P. Home,
D. Leibfried, and D. J. Wineland; Phys. Rev. Lett. 109, 080502;
http://dx.doi.org/10.1103/PhysRevLett.109.080502
http://tf.boulder.nist.gov/general/pdf/2624.pdf
