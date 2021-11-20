library verilog;
use verilog.vl_types.all;
entity processor_vlg_check_tst is
    port(
        ifetch_out      : in     vl_logic;
        mem_addr_out    : in     vl_logic_vector(31 downto 0);
        mem_data_out    : in     vl_logic_vector(31 downto 0);
        mem_read        : in     vl_logic;
        mem_write       : in     vl_logic;
        sampler_rx      : in     vl_logic
    );
end processor_vlg_check_tst;
