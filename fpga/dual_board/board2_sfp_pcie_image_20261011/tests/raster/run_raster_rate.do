onerror {quit -code 1 -force}
onbreak {}
if {![file exists modelsim.ini]} {vmap -c}
if {![file isdirectory work_raster_rate]} {vlib work_raster_rate}
vmap work_raster_rate work_raster_rate
vlog -sv -work work_raster_rate ../../source/src/img/sfp_raster_read_ctrl.v raster_rate_tb.sv
vsim -c -voptargs=+acc -onfinish stop work_raster_rate.raster_rate_tb
run -all
if {[examine -radix unsigned sim:/raster_rate_tb/test_passed] != 1} {quit -code 1 -force}
quit -code 0 -force
