-- RISC Microcontroller, Five-Bit Opcodes (RMC5) Demonstration Program

-- V1.0 [28-OCT-26] The microcontroller data buses, address bus, and 
-- control signals are connected to input-output pins.


library ieee;  
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity main is 
	generic (
		prog_cntr_len : positive := 12;
		cpu_addr_len : positive := 11;
		start_pc : natural := 0;
		interrupt_pc : natural := 3	);
	port (
		prog_cntr : out std_logic_vector(prog_cntr_len-1 downto 0);
		prog_data : in std_logic_vector(7 downto 0);
		cpu_addr : out std_logic_vector(cpu_addr_len-1 downto 0);
		cpu_data_out : out std_logic_vector(7 downto 0);
		cpu_data_in : in std_logic_vector(7 downto 0);
		WR : inout std_logic;
		ISRV : out std_logic;
		CK, IRQ, RESET : in std_logic
	);
end;

architecture behavior of main is

-- Microcontroller data in and out.
	signal wr_boolean, isrv_boolean : boolean;

-- Attributes that request signals to be retained instead of distributed.
	attribute syn_keep : boolean;
	attribute nomerge : string;
	
-- Functions and Procedures	
	function to_std_logic (v: boolean) return std_ulogic is
	begin if v then return('1'); else return('0'); end if; end function;

begin
	WR <= to_std_logic(wr_boolean);
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
			IRQ => (IRQ = '1'),
			ISRV => isrv_boolean,
			RESET => RESET,
			CK => CK
		);
		
end behavior;