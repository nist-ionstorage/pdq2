# Robert Jordens <jordens@gmail.com> 2013

from gateware.platform import Platform
from gateware.pdq import Pdq2


def _main():
    platform = Platform()
    pdq = Pdq2(platform)
    platform.build_cmdline(pdq, build_name="pdq2")


if __name__ == "__main__":
    _main()
