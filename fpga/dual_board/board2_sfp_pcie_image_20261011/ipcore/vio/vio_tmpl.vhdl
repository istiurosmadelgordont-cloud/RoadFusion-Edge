-- Created by IP Generator (Version 2022.2-SP6.4 build 146967)
-- Instantiation Template
--
-- Insert the following codes into your VHDL file.
--   * Change the_instance_name to your own instance name.
--   * Change the net names in the port map.


COMPONENT vio
  PORT (
    drck_in : IN STD_LOGIC;
    hub_tdi : IN STD_LOGIC;
    id_i : IN STD_LOGIC_VECTOR(8 DOWNTO 0);
    capt_i : IN STD_LOGIC;
    shift_i : IN STD_LOGIC;
    conf_sel : IN STD_LOGIC;
    hub_tdo : OUT STD_LOGIC;
    clk : IN STD_LOGIC;
    resetn_i : IN STD_LOGIC;
    probe_in0 : IN STD_LOGIC_VECTOR(31 DOWNTO 0);
    probe_in1 : IN STD_LOGIC_VECTOR(31 DOWNTO 0);
    probe_in2 : IN STD_LOGIC_VECTOR(31 DOWNTO 0);
    probe_in3 : IN STD_LOGIC_VECTOR(31 DOWNTO 0);
    probe_in4 : IN STD_LOGIC_VECTOR(31 DOWNTO 0);
    probe_in5 : IN STD_LOGIC_VECTOR(7 DOWNTO 0);
    probe_out0 : OUT STD_LOGIC_VECTOR(31 DOWNTO 0);
    probe_out1 : OUT STD_LOGIC_VECTOR(31 DOWNTO 0);
    probe_out2 : OUT STD_LOGIC_VECTOR(31 DOWNTO 0);
    probe_out3 : OUT STD_LOGIC_VECTOR(31 DOWNTO 0);
    probe_out4 : OUT STD_LOGIC_VECTOR(31 DOWNTO 0);
    probe_out5 : OUT STD_LOGIC
  );
END COMPONENT;


the_instance_name : vio
  PORT MAP (
    drck_in => drck_in,
    hub_tdi => hub_tdi,
    id_i => id_i,
    capt_i => capt_i,
    shift_i => shift_i,
    conf_sel => conf_sel,
    hub_tdo => hub_tdo,
    clk => clk,
    resetn_i => resetn_i,
    probe_in0 => probe_in0,
    probe_in1 => probe_in1,
    probe_in2 => probe_in2,
    probe_in3 => probe_in3,
    probe_in4 => probe_in4,
    probe_in5 => probe_in5,
    probe_out0 => probe_out0,
    probe_out1 => probe_out1,
    probe_out2 => probe_out2,
    probe_out3 => probe_out3,
    probe_out4 => probe_out4,
    probe_out5 => probe_out5
  );
