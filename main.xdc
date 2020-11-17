#####################################################################################
set_property -dict { PACKAGE_PIN E3    IOSTANDARD LVCMOS33 } [get_ports { clk_in }]
set_property -dict { PACKAGE_PIN C2    IOSTANDARD LVCMOS33 } [get_ports { rstx_in }]
set_property -dict { PACKAGE_PIN A9    IOSTANDARD LVCMOS33 } [get_ports { uart_rxd }]
set_property -dict { PACKAGE_PIN D10   IOSTANDARD LVCMOS33 } [get_ports { uart_txd }]

# create_clock -add -name sys_clk -period 10.00 [get_ports {clk_in}];


# ddr3 ports are defined in
# dram/mig_7series_0/mig_7series_0/user_design/constraints/mig_7series_0.xdc
# (mig_7series_0.xdc is generated during the synthesis of the ip core)

#####################################################################################
# used in implementation
create_generated_clock -name user_design_clk [get_pins dmem/dram/clkgen/inst/mmcm_adv_inst/CLKOUT0]
set_clock_groups -asynchronous -group {user_design_clk}
#####################################################################################
