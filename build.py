from platform import Platform
from top import Soc

def main():
    platform = Platform()
    soc = Soc(platform)

    platform.add_platform_command("""
NET "{clk50}" TNM_NET = "grp_clk50";
TIMESPEC "ts_grp_clk50" = PERIOD "grp_clk50" 20 ns HIGH 50%;
#NET "clk100" TNM_NET = "grp_clk100";
#TIMESPEC "ts_grp_clk100" = PERIOD "grp_clk100" 10 ns HIGH 50%;
""", clk50=platform.lookup_request("clk50"))
    
    platform.build_cmdline(soc)

if __name__ == "__main__":
    main()
