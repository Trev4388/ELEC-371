library verilog;
use verilog.vl_types.all;
entity processor is
    port(
        clk             : in     vl_logic;
        reset_n         : in     vl_logic;
        mem_addr_out    : out    vl_logic_vector(31 downto 0);
        mem_data_out    : out    vl_logic_vector(31 downto 0);
        mem_data_in     : in     vl_logic_vector(31 downto 0);
        mem_read        : out    vl_logic;
        mem_write       : out    vl_logic;
        ifetch_out      : out    vl_logic
    );
end processor;
