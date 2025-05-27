# Clock definition
#create_clock -period 10.000 -name clk_osc -waveform {0.000 5.000} [get_ports BASE_CLKP]
create_clock -period 8.000 -name clk_gmod -waveform {0.000 4.000} [get_pins u_BUFG/O]

#create_clock -period 8 -name clk_mod -waveform {0.000 4.000} [get_pins u_cbtlane/BUFG_modclk/O]
#create_clock -period 10 -name clk_mod -waveform {0.000 5.000} [get_pins u_cbtlane/BUFG_modclk/O]
#set_input_jitter clk_b2tt 0.100

#set_clock_groups -name async_input -physically_exclusive -group [get_clocks clk_osc] -group [get_clocks clk_hul]

#create_clock -period 1.600 -name clk_fast -waveform {0.000 0.800} [get_pins u_BUFG_Fast_inst/O]
#set_input_jitter clk_fast 0.030
#
#create_clock -period 8.000 -name clk_slow -waveform {0.000 4.000} [get_pins u_BUFG_Slow_inst/O]
#set_input_jitter clk_slow 0.030

#create_clock -period 2.000 -name clk_fast -waveform {0.000 1.000} [get_ports CLK_FASTP]
#set_input_jitter clk_fast 0.030

#create_clock -period 8.000 -name clk_slow -waveform {0.000 4.000} [get_ports CLK_SLOWP]
#set_input_jitter clk_slow 0.030

set_case_analysis 0 [get_pins u_CdcmMan_Inst/inst/mmcm_adv_inst/CLKINSEL]
#set_case_analysis 0 [get_pins BUFGMUX_C6C_inst/S]
#set_case_analysis 1 [get_pins BUFGMUX_FAST_inst/S]
#set_case_analysis 1 [get_pins BUFGMUX_SLOW_inst/S]

create_generated_clock -name clk_sys [get_pins u_ClkMan_Inst/inst/mmcm_adv_inst/CLKOUT0]
create_generated_clock -name clk_indep [get_pins u_ClkMan_Inst/inst/mmcm_adv_inst/CLKOUT1]
create_generated_clock -name clk_spi [get_pins u_ClkMan_Inst/inst/mmcm_adv_inst/CLKOUT2]

create_generated_clock -name clk_slow [get_pins u_CdcmMan_Inst/inst/mmcm_adv_inst/CLKOUT0]
create_generated_clock -name clk_fast [get_pins u_CdcmMan_Inst/inst/mmcm_adv_inst/CLKOUT1]
create_generated_clock -name clk_tdc0 [get_pins u_CdcmMan_Inst/inst/mmcm_adv_inst/CLKOUT2]
create_generated_clock -name clk_tdc1 [get_pins u_CdcmMan_Inst/inst/mmcm_adv_inst/CLKOUT3]
create_generated_clock -name clk_tdc2 [get_pins u_CdcmMan_Inst/inst/mmcm_adv_inst/CLKOUT4]
create_generated_clock -name clk_tdc3 [get_pins u_CdcmMan_Inst/inst/mmcm_adv_inst/CLKOUT5]

set_multicycle_path -setup -from [get_clocks clk_tdc3] -to [get_clocks clk_tdc0] 2


set_clock_groups -name async_sys_gmii -asynchronous -group clk_sys -group clk_indep -group clk_spi -group {clk_fast clk_slow clk_tdc0 clk_tdc1 ckl_tdc2 clk_tdc3} -group GTX_REFCLK_P

set_false_path -through [get_ports {LED[1]}]
set_false_path -through [get_ports {LED[2]}]
set_false_path -through [get_ports {LED[3]}]
set_false_path -through [get_ports {LED[4]}]


