onerror {quit -code 1 -force}
onbreak {}
set root [file normalize ../..]
set base [file normalize ../..]
if {[file exists [file join $root ipcore sfp_rx_32x2048 rtl ipml_fifo_v1_9_sfp_rx_32x2048.v]]} {set base $root}
set ip [file join $base ipcore sfp_rx_32x2048]
set vendor C:/pango/PDS_2022.2-SP6.4/arch/vendor/pango/verilog/simulation
if {![file exists modelsim.ini]} {vmap -c}
if {![file isdirectory work_sfp_register]} {vlib work_sfp_register}
vmap work_sfp_register work_sfp_register
vlog -sv -work work_sfp_register [file join $vendor GTP_GRS.v] [file join $vendor GTP_DRM18K.v]
vlog -sv -work work_sfp_register [file join $ip rtl ipml_fifo_ctrl_v1_4_sfp_rx_32x2048.v] [file join $ip rtl ipml_sdpram_v1_9_sfp_rx_32x2048.v] [file join $ip rtl ipml_fifo_v1_9_sfp_rx_32x2048.v] [file join $ip sfp_rx_32x2048.v]
vlog -sv -work work_sfp_register [file join $base source src img sfp_raster_read_ctrl.v] [file join $root source sfp_8b10b_top sfp_decode.v] sfp_decode_reference.v sfp_register_tb.sv
vsim -voptargs=+acc -onfinish stop work_sfp_register.sfp_register_tb
run -all
if {[examine -radix unsigned sim:/sfp_register_tb/test_passed] != 1} {quit -code 1 -force}
quit -code 0 -force
