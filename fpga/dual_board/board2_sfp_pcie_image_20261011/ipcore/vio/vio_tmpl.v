// Created by IP Generator (Version 2022.2-SP6.4 build 146967)
// Instantiation Template
//
// Insert the following codes into your Verilog file.
//   * Change the_instance_name to your own instance name.
//   * Change the signal names in the port associations


vio the_instance_name (
  .drck_in(drck_in),          // input
  .hub_tdi(hub_tdi),          // input
  .id_i(id_i),                // input [8:0]
  .capt_i(capt_i),            // input
  .shift_i(shift_i),          // input
  .conf_sel(conf_sel),        // input
  .hub_tdo(hub_tdo),          // output
  .clk(clk),                  // input
  .resetn_i(resetn_i),        // input
  .probe_in0(probe_in0),      // input [31:0]
  .probe_in1(probe_in1),      // input [31:0]
  .probe_in2(probe_in2),      // input [31:0]
  .probe_in3(probe_in3),      // input [31:0]
  .probe_in4(probe_in4),      // input [31:0]
  .probe_in5(probe_in5),      // input [7:0]
  .probe_out0(probe_out0),    // output [31:0]
  .probe_out1(probe_out1),    // output [31:0]
  .probe_out2(probe_out2),    // output [31:0]
  .probe_out3(probe_out3),    // output [31:0]
  .probe_out4(probe_out4),    // output [31:0]
  .probe_out5(probe_out5)     // output
);
