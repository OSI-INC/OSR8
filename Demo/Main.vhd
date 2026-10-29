-- RISC Microcontroller, Five-Bit Opcodes (RMC5) Demonstration Program

-- V1.0 [28-OCT-26] The microcontroller data bus and control signals are connected to 
-- input-output pins, LCMXO2-256ZE in TQFP-100.


library ieee;  
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity main is 
	port (
		prog_cntr : out std_logic_vector(11 downto 0);
		prog_data : in std_logic_vector(7 downto 0);
		cpu_data : inout std_logic_vector(7 downto 0);
		cpu_addr : out std_logic_vector(9 downto 0);
		CPUDS : inout std_logic;
		CPUWR : inout std_logic;
		ISRV : out std_logic;
		CK, IRQ, RESET : in std_logic
	);
end;

architecture behavior of main is

-- Configuration of OSR8 CPU.
	constant prog_cntr_len : integer := 12;
	constant cpu_addr_len : integer := 10;
	constant start_pc : integer := 0;
	constant interrupt_pc : integer := 3;
	constant ram_addr_len : integer := 10;

-- Microcontroller data in and out.
	signal cpu_data_in, cpu_data_out : std_logic_vector(7 downto 0);
	signal wr_boolean, ds_boolean, isrv_boolean : boolean;

-- Attributes that request signals to be retained instead of distributed.
	attribute syn_keep : boolean;
	attribute nomerge : string;
	
-- Functions and Procedures	
	function to_std_logic (v: boolean) return std_ulogic is
	begin if v then return('1'); else return('0'); end if; end function;

begin
	cpu_data <= cpu_data_out when CPUWR else (others => 'Z');
	cpu_data_in <= cpu_data;
	CPUDS <= to_std_logic(ds_boolean);
	CPUWR <= to_std_logic(wr_boolean);
	ISRV <= to_std_logic(isrv_boolean);	

	CPU : entity OSR8_CPU 
		generic map (
			prog_cntr_len => prog_cntr_len,
			cpu_addr_len => cpu_addr_len,
			start_pc => start_pc,
			interrupt_pc => interrupt_pc
		)
		port map (
			prog_data => prog_data,
			prog_cntr => prog_cntr,
			cpu_data_out => cpu_data_out,
			cpu_data_in => cpu_data_in,
			cpu_addr => cpu_addr,
			WR => wr_boolean,
			DS => ds_boolean,
			IRQ => (IRQ = '1'),
			ISRV => isrv_boolean,
			RESET => RESET,
			CK => CK
		);
		
end behavior;