# Robert Jordens <jordens@gmail.com> 2013

from platform import Platform
from top import Soc


def main():
    platform = Platform()
    soc = Soc(platform, fast=True)
    platform.build_cmdline(soc)


if __name__ == "__main__":
    main()
