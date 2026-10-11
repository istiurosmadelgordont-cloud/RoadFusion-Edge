onerror {quit -code 1 -force}
onbreak {}
set root [file normalize ../..]
if {![file exists modelsim.ini]} {vmap -c}
if {![file isdirectory work_command_guard]} {vlib work_command_guard}
vmap work_command_guard work_command_guard
vlog -sv -work work_command_guard [file join $root source source pcie_dma_ctrl.v] pcie_dma_ctrl_timing_reference.v fifo_model.v command_guard_tb.sv
transcript file command_guard.log
vsim -gIMAGE_PACKETS=2 -voptargs=+acc -onfinish stop work_command_guard.command_guard_tb
run -all
if {[examine -radix unsigned sim:/command_guard_tb/test_passed] != 1} {quit -code 1 -force}
quit -sim
transcript file command_guard_full.log
vsim -gIMAGE_PACKETS=64800 -voptargs=+acc -onfinish stop work_command_guard.command_guard_tb
run -all
if {[examine -radix unsigned sim:/command_guard_tb/test_passed] != 1} {quit -code 1 -force}
quit -code 0 -force
