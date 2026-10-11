onerror {quit -code 1 -force}
onbreak {}
set here [pwd]
set target C:/Users/hero/Desktop/pds_prj/06_hdmi_loop/06_hdmi_loop
set payload [file normalize ../../payload_board]
set installed [file normalize ../..]
if {[file exists $installed/source/src/img/image_processing_top.v]} {
    set target $installed
    set payload $installed
}
if {![file exists modelsim.ini]} {vmap -c}
if {![file isdirectory work_tone]} {vlib work_tone}
vmap work_tone work_tone
set rtl {}
foreach group {common img} {
    set seen [dict create]
    foreach source [glob $target/source/src/$group/*.v] {
        set name [file tail $source]
        dict set seen $name 1
        if {[file exists $payload/source/src/$group/$name]} {
            lappend rtl $payload/source/src/$group/$name
        } else {
            lappend rtl $source
        }
    }
    foreach source [glob -nocomplain $payload/source/src/$group/*.v] {
        if {![dict exists $seen [file tail $source]]} {lappend rtl $source}
    }
}
vlog -sv -work work_tone {*}$rtl tone_luma_tb.sv tone_pipeline_tb.sv
vsim -c -voptargs=+acc -onfinish stop work_tone.tone_luma_tb
run -all
if {[examine -radix unsigned sim:/tone_luma_tb/test_passed] != 1} {quit -code 1 -force}
quit -sim
vsim -c -voptargs=+acc -onfinish stop work_tone.tone_pipeline_tb
run -all
if {[examine -radix unsigned sim:/tone_pipeline_tb/test_passed] != 1} {quit -code 1 -force}
quit -code 0 -force
