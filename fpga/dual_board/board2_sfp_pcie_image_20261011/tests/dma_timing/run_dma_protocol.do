onerror {quit -code 1 -force}
onbreak {}
set root [file normalize ../..]
if {![file exists modelsim.ini]} {vmap -c}
if {![file isdirectory work_dma_protocol]} {vlib work_dma_protocol}
vmap work_dma_protocol work_dma_protocol
vlog -sv -work work_dma_protocol [file join $root source source pcie_dma_ctrl.v] fifo_model.v handshake_tb.sv
vsim -voptargs=+acc -onfinish stop work_dma_protocol.handshake_tb
run -all
if {[examine -radix unsigned sim:/handshake_tb/test_passed] != 1} {quit -code 1 -force}
quit -code 0 -force
