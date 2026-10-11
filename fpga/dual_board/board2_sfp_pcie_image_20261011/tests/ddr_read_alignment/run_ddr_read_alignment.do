onerror {quit -code 1 -force}
onbreak {}
set root [file normalize ../..]
set base [file normalize ../..]
if {[file exists [file join $root source read_ddr_fifo read_ddr_fifo.v]]} {set base $root}
set vendor C:/pango/PDS_2022.2-SP6.4/arch/vendor/pango/verilog/simulation
if {![file exists modelsim.ini]} {vmap -c}
if {![file isdirectory work_ddr_read_alignment]} {vlib work_ddr_read_alignment}
vmap work_ddr_read_alignment work_ddr_read_alignment
vlog -sv -work work_ddr_read_alignment [file join $vendor GTP_GRS.v] [file join $vendor GTP_DRM9K.v] [file join $vendor GTP_DRM18K.v]
foreach fifo {read_ddr_fifo write_ddr_fifo} {
    set ip [file join $base source $fifo]
    vlog -sv -work work_ddr_read_alignment [file join $ip rtl ipml_fifo_ctrl_v1_4_${fifo}.v] [file join $ip rtl ipml_sdpram_v1_9_${fifo}.v] [file join $ip rtl ipml_fifo_v1_9_${fifo}.v] [file join $ip ${fifo}.v]
}
vlog -sv -work work_ddr_read_alignment [file join $base source source rw_fifo_ctrl.v] [file join $base source source sync_generator.v] ddr_read_alignment_tb.sv
if {![info exists full_hd]} {set full_hd 0}
vsim -gFULL_HD=$full_hd -voptargs=+acc -onfinish stop work_ddr_read_alignment.ddr_read_alignment_tb
run -all
if {[examine -radix unsigned sim:/ddr_read_alignment_tb/test_passed] != 1} {quit -code 1 -force}
quit -code 0 -force
