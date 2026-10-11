onerror {quit -code 1 -force}
onbreak {}
if {![file exists modelsim.ini]} {vmap -c}
if {![file isdirectory work_atmosphere_stage]} {vlib work_atmosphere_stage}
vmap work_atmosphere_stage work_atmosphere_stage
vlog -work work_atmosphere_stage -sv ../../source/src/img/atmosphere_temporal.v atmosphere_temporal_reference.v atmosphere_temporal_tb.sv
vsim -c -voptargs=+acc -onfinish stop work_atmosphere_stage.atmosphere_temporal_tb
run -all
if {[examine -radix unsigned sim:/atmosphere_temporal_tb/test_passed] != 1} {quit -code 1 -force}
quit -code 0 -force
