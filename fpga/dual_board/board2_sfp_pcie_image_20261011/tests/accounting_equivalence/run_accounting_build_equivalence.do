onerror {quit -code 1 -force}
onbreak {}
if {![file exists modelsim.ini]} {vmap -c}
if {![file isdirectory work_accounting_build]} {vlib work_accounting_build}
vmap work_accounting_build work_accounting_build
vlog -sv -work work_accounting_build ../../source/src/img/pcie_image_control.v pcie_image_control_build_reference.v accounting_build_equivalence_tb.sv
vsim -voptargs=+acc -onfinish stop work_accounting_build.accounting_build_equivalence_tb
run -all
if {[examine -radix unsigned sim:/accounting_build_equivalence_tb/test_passed] != 1} {quit -code 1 -force}
quit -code 0 -force
