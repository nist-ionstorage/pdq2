# Robert Jordens <jordens@gmail.com> 2013

from gateware.platform import Platform
from gateware.pdq import Pdq


def _main():
    platform = Platform()
    pdq = Pdq(platform)
    platform.build_cmdline(pdq, build_name="pdq")


if __name__ == "__main__":
    _main()
