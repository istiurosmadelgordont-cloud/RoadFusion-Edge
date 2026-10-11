onerror {quit -code 1 -force}
onbreak {}
quietly set base [file normalize ../../source/src/common]
if {![file exists modelsim.ini]} {vmap -c}
if {![file isdirectory work_sky]} {vlib work_sky}
vmap work_sky work_sky
vlog -sv -work work_sky $base/frame_config_delay.v $base/sky_protect.v sky_before_carry.v ../timing_equivalence/sky_reference.v sky_carry_tb.sv
vsim -c -voptargs=+acc -onfinish stop work_sky.sky_carry_tb
run -all
if {[examine -radix unsigned sim:/sky_carry_tb/test_passed] != 1} {quit -code 1 -force}
quit -code 0 -force
