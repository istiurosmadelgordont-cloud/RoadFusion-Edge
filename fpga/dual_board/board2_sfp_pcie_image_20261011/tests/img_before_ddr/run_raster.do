onerror {quit -code 1 -force}
onbreak {}
vmap -c
vlib work_raster
vmap work_raster work_raster
vlog -work work_raster -sv ../../source/src/img/sfp_raster_read_ctrl.v raster_read_tb.v
vsim -c -onfinish stop work_raster.raster_read_tb
run -all
quit -code 0 -force
