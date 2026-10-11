onerror {quit -code 1 -force}
onbreak {}
set root [file normalize ../..]
set base [file normalize ../..]
if {[file exists [file join $root ipcore hdmi_pcie_fifo hdmi_pcie_fifo.v]]} {set base $root}
set vendor C:/pango/PDS_2022.2-SP6.4/arch/vendor/pango/verilog/simulation
set ip [file join $base ipcore hdmi_pcie_fifo]
if {![file exists modelsim.ini]} {vmap -c}
if {![file isdirectory work_dma_vendor]} {vlib work_dma_vendor}
vmap work_dma_vendor work_dma_vendor
vlog -sv -work work_dma_vendor [file join $vendor GTP_GRS.v] [file join $vendor GTP_DRM18K.v]
vlog -sv -work work_dma_vendor [file join $ip rtl ipml_fifo_ctrl_v1_3.v] [file join $ip rtl ipml_reg_fifo_v1_0.v] [file join $ip rtl ipml_prefetch_fifo_v1_6_hdmi_pcie_fifo.v] [file join $ip rtl ipml_sdpram_v1_6_hdmi_pcie_fifo.v] [file join $ip rtl ipml_fifo_v1_6_hdmi_pcie_fifo.v] [file join $ip hdmi_pcie_fifo.v]
vlog -sv -work work_dma_vendor [file join $root source source pcie_dma_ctrl.v] pcie_dma_ctrl_timing_reference.v [file join $base source source sync_generator.v] dma_timing_vendor_tb.sv
if {![info exists image_packets]} {set image_packets 120}
vsim -gIMAGE_PACKETS=$image_packets -voptargs=+acc -onfinish stop work_dma_vendor.dma_timing_vendor_tb
run -all
if {[examine -radix unsigned sim:/dma_timing_vendor_tb/test_passed] != 1} {quit -code 1 -force}
quit -code 0 -force
