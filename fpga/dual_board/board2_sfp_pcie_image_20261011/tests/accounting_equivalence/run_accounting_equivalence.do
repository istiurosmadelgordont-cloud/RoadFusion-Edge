onerror {quit -code 1 -force}
onbreak {}
if {![file exists modelsim.ini]} {vmap -c}
if {![file isdirectory work_accounting]} {vlib work_accounting}
vmap work_accounting work_accounting
vlog -sv -work work_accounting ../../source/src/img/pcie_image_control.v pcie_image_control_reference.v accounting_equivalence_tb.sv
vsim -voptargs=+acc -onfinish stop work_accounting.accounting_equivalence_tb
run -all
if {[examine -radix unsigned sim:/accounting_equivalence_tb/test_passed] != 1} {quit -code 1 -force}
quit -code 0 -force
