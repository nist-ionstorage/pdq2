# Robert Jordens <jordens@gmail.com> 2013

from mibuild.generic_platform import *
from mibuild.xilinx.ise import XilinxISEPlatform
from mibuild.crg import SimpleCRG


_io = [
        ("clk50", 0, Pins("P80"), IOStandard("LVCMOS25")),

        ("comm", 0,
            Subsignal("data", Pins("P101 P91 P76 P72 P71 P60 P58 P57")),
            Subsignal("rdl", Pins("P97")),
            Subsignal("rxfl", Pins("P54")),

            Subsignal("rd_in", Pins("P159")), #GO_1
            Subsignal("rd_out", Pins("P102")), #G1

            Subsignal("reset", Pins("P96")), # dac_reset

            Subsignal("adr", Pins("P32 P6 P14 P43")), # active low
            Subsignal("trigger", Pins("P110")), #F1
            Subsignal("frame", Pins("P118 P124 P98")), #F2 F3 F4
            Subsignal("aux", Pins("P99")), #F5 out

            Subsignal("go2_in", Pins("P169")), #GO_2 in
            Subsignal("go2_out", Pins("P100")), #G2 out
            IOStandard("LVCMOS25")
        ),

        ("dac", 0,
            Subsignal("clk_p", Pins("P89"), IOStandard("LVCMOS25")),
            Subsignal("clk_n", Pins("P90"), IOStandard("LVCMOS25")),
            Subsignal("data_p", Pins(
                "P106 P108 P112 P115 P119 P122 P126 P128 "
                "P132 P134 P137 P139 P144 P146 P150 P152"),
                IOStandard("LVDS_25")),
            Subsignal("data_n", Pins(
                "P107 P109 P113 P116 P120 P123 P127 P129 "
                "P133 P135 P138 P140 P145 P147 P151 P153"),
                IOStandard("LVDS_25")),
            Subsignal("data_clk_p", Pins("P93"), IOStandard("LVDS_25")),
            Subsignal("data_clk_n", Pins("P94"), IOStandard("LVDS_25")),
        ),

        ("dac", 1,
            Subsignal("clk_p", Pins("P68"), IOStandard("LVCMOS25")),
            Subsignal("clk_n", Pins("P69"), IOStandard("LVCMOS25")),
            Subsignal("data_p", Pins(
                "P160 P162 P164 P167 P171 P82 P177 P180 "
                "P74 P185 P189 P192 P196 P199 P202 P205"),
                IOStandard("LVDS_25")),
            Subsignal("data_n", Pins(
                "P161 P163 P165 P168 P172 P83 P178 P181 "
                "P75 P186 P190 P193 P197 P200 P203 P206"),
                IOStandard("LVDS_25")),
            Subsignal("data_clk_p", Pins("P77"), IOStandard("LVDS_25")),
            Subsignal("data_clk_n", Pins("P78"), IOStandard("LVDS_25")),
        ),

        ("dac", 2,
            Subsignal("clk_p", Pins("P62"), IOStandard("LVCMOS25")),
            Subsignal("clk_n", Pins("P63"), IOStandard("LVCMOS25")),
            Subsignal("data_p", Pins(
                "P2 P4 P8 P11 P15 P18 P22 P24 "
                "P28 P30 P33 P35 P39 P41 P47 P49"),
                IOStandard("LVDS_25")),
            Subsignal("data_n", Pins(
                "P3 P5 P9 P12 P16 P19 P23 P25 "
                "P29 P31 P34 P36 P40 P42 P48 P50"),
                IOStandard("LVDS_25")),
            Subsignal("data_clk_p", Pins("P64"), IOStandard("LVDS_25")),
            Subsignal("data_clk_n", Pins("P65"), IOStandard("LVDS_25")),
        ),
]


class Platform(XilinxISEPlatform):
    xst_opt = """-ifmt MIXED
-opt_level 2
-opt_mode SPEED
-register_balancing yes"""
    bitgen_opt = "-g GTS_cycle:3 -g LCK_cycle:4 -g GWE_cycle:5 " \
            "-g DONE_cycle:6 -g Binary:Yes -w"
    ise_commands = """
trce -v 12 -fastpaths -o {build_name} {build_name}.ncd {build_name}.pcf
promgen -w -spi -c FF -p mcs -o {build_name}.mcs -u 0 {build_name}.bit
"""
    def __init__(self):
        XilinxISEPlatform.__init__(self, "xc3s500e-4pq208", _io,
                lambda p: SimpleCRG(p, "clk50", None))

    def do_finalize(self, fragment):
        try:
            self.add_period_constraint(self.lookup_request("clk50"), 20)
        except ConstraintError:
            pass
