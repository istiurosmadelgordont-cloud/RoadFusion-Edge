onerror {quit -code 1 -force}
onbreak {}
quietly set target [file normalize ../payload_target]
quietly set board [file normalize ../payload_board]
quietly set base C:/Users/hero/Desktop/pds_prj/06_hdmi_loop/06_hdmi_loop
quietly set installed [file normalize ../..]
if {[file exists $installed/source/src/img/image_processing_top.v]} {
    quietly set base $installed
    quietly set target $installed
    quietly set board $installed
}
if {![file exists modelsim.ini]} {vmap -c}
if {![file isdirectory work_serial]} {vlib work_serial}
vmap work_serial work_serial
quietly set rtl {}
foreach group {common img} {
    quietly set sources [dict create]
    foreach root [list $base $target $board] {
        foreach source [glob -nocomplain $root/source/src/$group/*.v] {
            dict set sources [file tail $source] $source
        }
    }
    dict for {name source} $sources {
        if {$name ne "image_processing_dvio_top.v" && $name ne "pcie_tlp_arbiter.v"} {
            lappend rtl $source
        }
    }
}
vlog -sv -work work_serial {*}$rtl image_serial_tb.sv
vsim -c -voptargs=+acc -onfinish stop work_serial.image_serial_tb
run -all
if {[examine -radix unsigned sim:/image_serial_tb/test_passed] != 1} {quit -code 1 -force}
quit -code 0 -force
