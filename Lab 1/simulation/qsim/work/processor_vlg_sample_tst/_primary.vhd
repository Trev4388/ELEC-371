library verilog;
use verilog.vl_types.all;
entity processor_vlg_sample_tst is
    port(
        clk             : in     vl_logic;
        mem_data_in     : in     vl_logic_vector(31 downto 0);
        reset_n         : in     vl_logic;
        sampler_tx      : out    vl_logic
    );
end processor_vlg_sample_tst;
