onerror {quit -code 1 -force}
onbreak {}
if {![file exists modelsim.ini]} {vmap -c}
set installed [file normalize ../..]
set root [file normalize ../payload_target]
if {[file exists $installed/source/src/img/pcie_image_control.v]} {set root $installed}
if {![file isdirectory work_pcie_control]} {vlib work_pcie_control}
vmap work_pcie_control work_pcie_control
vlog -sv -work work_pcie_control $root/source/source/pcie_dma_ctrl.v
vlog -sv -work work_pcie_control $root/source/src/img/pcie_image_control.v
vlog -sv -work work_pcie_control $root/source/src/img/pcie_tlp_arbiter.v
vlog -sv -work work_pcie_control $root/source/src/img/image_runtime_control.v
vlog -sv -work work_pcie_control fifo_model.v pcie_image_control_tb.sv
vsim -voptargs=+acc -onfinish stop work_pcie_control.pcie_image_control_tb
run -all
if {[examine -radix unsigned sim:/pcie_image_control_tb/test_passed] != 1} {quit -code 1 -force}
quit -code 0 -force
