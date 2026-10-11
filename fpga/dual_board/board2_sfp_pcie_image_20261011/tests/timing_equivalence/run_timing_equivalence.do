onerror {quit -code 1 -force}
onbreak {}
if {![file isdirectory work_timing]} {vlib work_timing}
vmap work_timing work_timing
vlog -sv -work work_timing ../../source/src/common/frame_config_delay.v ../../source/src/common/sky_protect.v ../../source/src/common/ycbcr2rgb.v sky_reference.v ycbcr_reference.v timing_equivalence_tb.sv
vsim -voptargs=+acc -onfinish stop work_timing.timing_equivalence_tb
run -all
if {[examine -radix unsigned sim:/timing_equivalence_tb/test_passed] != 1} {quit -code 1 -force}
quit -code 0 -force
