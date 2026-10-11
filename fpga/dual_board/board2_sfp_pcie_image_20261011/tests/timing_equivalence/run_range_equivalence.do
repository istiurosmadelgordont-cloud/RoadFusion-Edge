onerror {quit -code 1 -force}
onbreak {}
if {![file isdirectory work_range]} {vlib work_range}
vmap work_range work_range
vlog -sv -work work_range ../../source/src/common/Bilateral_Range_LUT.v bilateral_reference.v range_equivalence_tb.sv
vsim -voptargs=+acc -onfinish stop work_range.range_equivalence_tb
run -all
if {[examine -radix unsigned sim:/range_equivalence_tb/test_passed] != 1} {quit -code 1 -force}
quit -code 0 -force
