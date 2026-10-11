onerror {quit -code 1 -force}
onbreak {}
quietly set base [file normalize ../../source/src/common]
if {![file exists modelsim.ini]} {vmap -c}
if {![file isdirectory work_clahe]} {vlib work_clahe}
vmap work_clahe work_clahe
vlog -sv -work work_clahe $base/frame_config_delay.v $base/clahe_histogram.v $base/clahe_clipper.v $base/clahe_cdf_store.v $base/clahe_tiles.v $base/clahe_tiles_first.v $base/clahe_luma.v clahe_luma_reference.v clahe_equivalence_tb.sv
vsim -c -voptargs=+acc -onfinish stop work_clahe.clahe_equivalence_tb
run -all
if {[examine -radix unsigned sim:/clahe_equivalence_tb/test_passed] != 1} {quit -code 1 -force}
quit -code 0 -force
