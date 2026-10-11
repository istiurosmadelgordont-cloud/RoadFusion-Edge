onerror {quit -code 1 -force}
onbreak {}
set root [file normalize ../..]
if {![file exists modelsim.ini]} {vmap -c}
if {![file isdirectory work_dma]} {vlib work_dma}
vmap work_dma work_dma
vlog -sv -work work_dma $root/source/source/pcie_dma_ctrl.v fifo_model.v handshake_tb.sv
vsim -voptargs=+acc -onfinish stop work_dma.handshake_tb
run -all
if {[examine -radix unsigned sim:/handshake_tb/test_passed] != 1} {quit -code 1 -force}
quit -code 0 -force
