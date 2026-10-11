onerror {quit -code 1 -force}
onbreak {}
set root [file normalize ../..]
if {![file exists modelsim.ini]} {vmap -c}
if {![file isdirectory work_dma_reset]} {vlib work_dma_reset}
vmap work_dma_reset work_dma_reset
vlog -sv -work work_dma_reset $root/source/source/pcie_dma_ctrl.v pcie_dma_ctrl_reference.v fifo_reset_model.v sync_generator_reference.v fifo_reset_tb.sv
vsim -voptargs=+acc -onfinish stop work_dma_reset.fifo_reset_tb
run -all
if {[examine -radix unsigned sim:/fifo_reset_tb/test_passed] != 1} {quit -code 1 -force}
quit -code 0 -force
