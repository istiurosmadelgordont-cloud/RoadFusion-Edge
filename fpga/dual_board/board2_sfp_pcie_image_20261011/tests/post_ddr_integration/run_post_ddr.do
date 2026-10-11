onerror {quit -code 1 -force}
onbreak {}
set target [file normalize ../..]
set base $target
if {![file exists modelsim.ini]} {vmap -c}
if {![file isdirectory work_post_ddr]} {vlib work_post_ddr}
vmap work_post_ddr work_post_ddr
set sources [dict create]
foreach group {common img} {
    foreach root [list $base $target] {
        foreach source [glob -nocomplain $root/source/src/$group/*.v] {
            dict set sources [file tail $source] $source
        }
    }
}
set rtl {}
dict for {name source} $sources {
    if {$name ne "image_processing_dvio_top.v" && $name ne "pcie_tlp_arbiter.v" && $name ne "pcie_image_control.v"} {
        lappend rtl $source
    }
}
vlog -sv -work work_post_ddr {*}$rtl $base/source/source/sync_generator.v peripheral_witnesses.v \
    $target/source/hdmi_ddr_pcie.v post_ddr_integration_tb.sv
vsim -c -voptargs=+acc -onfinish stop work_post_ddr.post_ddr_integration_tb
run -all
if {[examine -radix unsigned sim:/post_ddr_integration_tb/test_passed] != 1} {quit -code 1 -force}
quit -code 0 -force
