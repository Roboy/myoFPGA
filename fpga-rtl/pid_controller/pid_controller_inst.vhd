	component pid_controller is
		port (
			address     : in  std_logic_vector(3 downto 0)  := (others => 'X'); -- address
			write       : in  std_logic                     := 'X';             -- write
			writedata   : in  std_logic_vector(31 downto 0) := (others => 'X'); -- writedata
			read        : in  std_logic                     := 'X';             -- read
			readdata    : out std_logic_vector(31 downto 0);                    -- readdata
			waitrequest : out std_logic;                                        -- waitrequest
			clock       : in  std_logic                     := 'X';             -- clk
			o_output    : out std_logic_vector(31 downto 0);                    -- export
			reset       : in  std_logic                     := 'X'              -- reset
		);
	end component pid_controller;

	u0 : component pid_controller
		port map (
			address     => CONNECTED_TO_address,     -- avalon_slave_0.address
			write       => CONNECTED_TO_write,       --               .write
			writedata   => CONNECTED_TO_writedata,   --               .writedata
			read        => CONNECTED_TO_read,        --               .read
			readdata    => CONNECTED_TO_readdata,    --               .readdata
			waitrequest => CONNECTED_TO_waitrequest, --               .waitrequest
			clock       => CONNECTED_TO_clock,       --     clock_sink.clk
			o_output    => CONNECTED_TO_o_output,    --    conduit_end.export
			reset       => CONNECTED_TO_reset        --          reset.reset
		);

