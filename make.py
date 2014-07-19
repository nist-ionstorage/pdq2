# Robert Jordens <jordens@gmail.com> 2013

from gateware.platform import Platform
from gateware.pdq import Pdq


def _main():
    platform = Platform()
    soc = Pdq(platform, fast=True)
    platform.build_cmdline(soc)


if __name__ == "__main__":
    _main()
