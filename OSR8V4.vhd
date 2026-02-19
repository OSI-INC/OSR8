-- Open-Source Reconfigurable Eight-Bit (OSR8) Central Processing Unit (CPU)
--
-- Copyright (C) 2020-2026 Kevan Hashemi, Open Source Instruments Inc.
--
-- This program is free software; you can redistribute it and/orpr
-- modify it under the terms of the GNU General Public License
-- as published by the Free Software Foundation; either version 2
-- of the License, or (at your option) any later version.
--
-- This program is distributed in the hope that it will be useful,
-- but WITHOUT ANY WARRANTY; without even the implied warranty of
-- MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
-- GNU General Public License for more details.
--
-- You should have received a copy of the GNU General Public License
-- along with this program; if not, write to the Free Software
-- Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.

-- V4.1, [10-FEB-26] Eliminate the intermediate variable prog_cntr, change all
-- occurances of prog_addr to prog_cntr. These changes are cosmetic only,
-- but also have the effect of making the OSR8V4 incompatible with a port
-- interface created for the OSR8V3.

-- [15-FEB-26] In the face of chronic instability, we resolve to constrain
-- the behavior of all registered and combinatorial logic. In the case of-- registered logic, we make sure all our case statements include "others null"
-- to make it clear to the compiler that all registers should hold their value
-- unless stated otherwise. in combinatorial logic, we provide default values
-- for all signals, and include "others null" in case statements as well, so 
-- that there is no state in which the compiler does not know what the correct
-- value of the signal should be. If we do not tell the compiler the correct
-- value of Y for input X, the compiler either proves to itself that X can never
-- occur, or it fails in this proof and creates latches to preserve the state
-- of Y when X arises. These latches are vulnerable to glitches, and they will
-- appear and disappear from our logic as the compiler's optimization fails 
-- or succeeds in proving that X will never occur. The addition of these 
-- constraints increases the code size by 50 LUTs, making it impossible to
-- fit the constrained OSR8 in the P3041 logic chip.

-- We must reduce the size of the OSR8. We see no examples of our using
-- instructions inc E, dec E, inc H, dec H, inc L, dec L, inc SP, or dec SP.
-- All of these can be created by combinations of other instructions. For
-- example, "inc SP" could be "push A", because pushing a byte onto the stack
-- increments the stack pointer. We can do "inc E" with "push A, push E, pop A,
-- inc A, push A, pop E, pop A". We eliminate these instructions and the code 
-- shrinks by 100 LUTs, and now fits in the P3041.

-- [18-FEB-26] Restore missing C-Flag definition. Assign values to the SIG
-- signals: read opcode flag, executing interrupt flag, jump instruction flag,
-- and a combined flag that shows any increment or decrement of a register,
-- any jump that occurs, and any constant addition or subtraction from register
-- A, but not an addition or subtraction of register B. Subject to multiple
-- adjustments and re-compilations on an A3041 platform. Stable and robust.

-- V4.2, [18-FEB-26] Start new version. Introduce a signals opcode_saved and
-- state for the CPU. Separate the ALU input multiplexer into its own process.
-- find and fix bugs in the MUX from use of different copies of opcode. Add
-- Interrupt Service (ISRV) flag that indicates the CPU is servicing an
-- interrupt.

library ieee;  
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

-- The CPU requires seprate program and process memory. Process and program memory 
-- can be anything from 1 kByte to 64 kByte.

-- The program memory read cycles must be clocked on the falling edge of CK. 
-- The program memory write cycles, if supported by peripherals, are how the
-- user can upload code into the embedded controller. The CPU must write to
-- a dual-port program memory on the rising edge of CK, on account of this same
-- memory being read by the CPU on the falling edge. Dual-port memory does not-- function correctly unless the write and read clocks are offset.

-- The process memory must be clocked on the falling edge of CK. The CPU increments 
-- the program counter on the rising edge of CK. When it asserts WR and DS, it does
-- so for one CK cycle, starting and ending on the rising edges. These signals are
-- always valid before and after the falling edge of CK. On a read cycle, the CPU 
-- expects data to be ready before the next rising edge of CK after it asserts DS. On
-- a write cycle, the CPU expects the write to take place on the falling edge of
-- CK after it asserts DS and WR. 

-- The IRQ signal is the interrupt request line from external logic. The memory 
-- management unit (MMU) must provide a way for the CPU to clear the IRQ signal 
-- so it can get back to its main program.

-- The RESET signal will put the CPU back to program location zero and clear the 
-- stack pointer. Execution begins at location start_pc. Most likely, the program
-- will provide a three-byte un-conditional jump instruction at this location. 
-- Interrupt execution begins at interrupt_pc, where we will most likely fine
-- another jp_nn instruction. 

-- The stack pointer is upward-going and initialized to zero. Byte ordering for 
-- pointers is big-endian. The most significant, or HI byte is at the lower address 
-- and the lease significant, or LO byte, is at the higher address. The HI byte of 
-- the pointer will contain from one to eight neccessary bits, depending upon the 
-- memory available to the CPU. 

entity OSR8_CPU is 
	generic (
		prog_cntr_len : integer := 12;
		cpu_addr_len : integer := 11;
		start_pc : integer := 0;
		interrupt_pc : integer := 3	);
	port (
		prog_data : in std_logic_vector(7 downto 0); -- Program Data
		prog_cntr : inout std_logic_vector(prog_cntr_len-1 downto 0); -- Program Address
		cpu_data_out : out std_logic_vector(7 downto 0); -- Outgoing CPU Data Bus
		cpu_data_in : in std_logic_vector(7 downto 0); -- Incoming CPU Data Bus
		cpu_addr : out std_logic_vector(cpu_addr_len-1 downto 0); -- Outgoing CPU Address Bus
		WR : out boolean; -- Write Cycle
		DS : out boolean; -- Data Strobe
		IRQ : in boolean; -- Interrupt Request
		ISRV : out boolean; -- Interrupt Service
		SIG : out std_logic_vector(3 downto 0); -- Signals for Debugging
		RESET : in std_logic; -- Hard Reset
		CK : in std_logic); -- The clock, duty cycle 50%.
end;

architecture behavior of OSR8_CPU is 

-- Program location constants in bytes.
	constant pa_top : integer := prog_cntr_len-1;
	constant ca_top : integer := cpu_addr_len-1;

-- Definition of operation codes. We have the constant name we will be using
-- in the VHDL definition of the CPU to refer to an operation, followed by 
-- the hexadecimal byte value by which this operation will be invoked as a 
-- program is being executed, and the instruction pneumonic that shows what
-- the instruction does, and how it is to be written in the OSR8 assembly 
-- language. We use "n" for an eight-bit constant, "nn" for a sixteen-bit
-- constant, "(nn)" for the location with address nn, "(IX)" for the location
-- with address given by index register IX, "jp nz" for jump if not zero,
-- "ld" for "load", and so on. Our syntax for assembler code follows that
-- of the venerable Z80 microprocessors.

subtype opcode_type is std_logic_vector(6 downto 0);
	
constant nop      : opcode_type := "0000000"; -- 00 nop
constant jp_nn    : opcode_type := "0000001"; -- 01 jp nn
constant jp_nz_nn : opcode_type := "0000010"; -- 02 jp nz,nn
constant jp_z_nn  : opcode_type := "0000011"; -- 03 jp z,nn
constant jp_nc_nn : opcode_type := "0000100"; -- 04 jp nc,nn
constant jp_c_nn  : opcode_type := "0000101"; -- 05 jp c,nn
constant jp_np_nn : opcode_type := "0000110"; -- 06 jp np,nn
constant jp_p_nn  : opcode_type := "0000111"; -- 07 jp p,nn
constant call_nn  : opcode_type := "0001000"; -- 08 call nn
constant sw_int   : opcode_type := "0001001"; -- 09 int
constant ret_cll  : opcode_type := "0001010"; -- 0A ret
constant ret_int  : opcode_type := "0001011"; -- 0B rti
constant cpu_wt   : opcode_type := "0001100"; -- 0C wait
constant clr_iflg : opcode_type := "0001101"; -- 0D clri
constant set_iflg : opcode_type := "0001110"; -- 0E seti

constant ld_A_n   : opcode_type := "0010000"; -- 10 ld A,n
constant ld_IX_nn : opcode_type := "0010001"; -- 11 ld IX,nn
constant ld_IY_nn : opcode_type := "0010010"; -- 12 ld IY,nn
constant ld_HL_nn : opcode_type := "0010011"; -- 13 ld HL,nn
constant ld_A_mm  : opcode_type := "0010100"; -- 14 ld A,(nn)
constant ld_mm_A  : opcode_type := "0010101"; -- 15 ld (nn),A
constant ld_A_ix  : opcode_type := "0010110"; -- 16 ld A,(IX)
constant ld_A_iy  : opcode_type := "0010111"; -- 17 ld A,(IY)
constant ld_ix_A  : opcode_type := "0011000"; -- 18 ld (IX),A
constant ld_iy_A  : opcode_type := "0011001"; -- 19 ld (IY),A
constant ld_HL_SP : opcode_type := "0011010"; -- 1A ld HL,SP
constant ld_SP_HL : opcode_type := "0011011"; -- 1B ld SP,HL
constant ld_HL_PC : opcode_type := "0011100"; -- 1C ld HL,PC
constant ld_PC_HL : opcode_type := "0011101"; -- 1D ld PC,HL

constant push_A   : opcode_type := "0100000"; -- 20 push A
constant push_B   : opcode_type := "0100001"; -- 21 push B
constant push_C   : opcode_type := "0100010"; -- 22 push C
constant push_D   : opcode_type := "0100011"; -- 23 push D
constant push_E   : opcode_type := "0100100"; -- 24 push E
constant push_H   : opcode_type := "0100101"; -- 25 push H
constant push_L   : opcode_type := "0100110"; -- 26 push L
constant push_F   : opcode_type := "0100111"; -- 27 push F
constant push_IX  : opcode_type := "0101000"; -- 28 push IX
constant push_IY  : opcode_type := "0101001"; -- 29 push IY

constant pop_A    : opcode_type := "0110000"; -- 30 pop A
constant pop_B    : opcode_type := "0110001"; -- 31 pop B
constant pop_C    : opcode_type := "0110010"; -- 32 pop C
constant pop_D    : opcode_type := "0110011"; -- 33 pop D
constant pop_E    : opcode_type := "0110100"; -- 34 pop E
constant pop_H    : opcode_type := "0110101"; -- 35 pop H
constant pop_L    : opcode_type := "0110110"; -- 36 pop L
constant pop_F    : opcode_type := "0110111"; -- 37 pop F
constant pop_IX   : opcode_type := "0111000"; -- 38 pop IX
constant pop_IY   : opcode_type := "0111001"; -- 39 pop IY

constant add_A_B  : opcode_type := "1000000"; -- 40 add A,B
constant add_A_n  : opcode_type := "1000001"; -- 41 add A,n
constant adc_A_B  : opcode_type := "1000010"; -- 42 adc A,B
constant adc_A_n  : opcode_type := "1000011"; -- 43 adc A,n
constant sub_A_B  : opcode_type := "1000100"; -- 44 sub A,B
constant sub_A_n  : opcode_type := "1000101"; -- 45 sub A,n
constant sbc_A_B  : opcode_type := "1000110"; -- 46 sbc A,B
constant sbc_A_n  : opcode_type := "1000111"; -- 47 sbc A,n
constant clr_aflg : opcode_type := "1001111"; -- 4F clrf

constant inc_A    : opcode_type := "1010000"; -- 50 inc A
constant inc_B    : opcode_type := "1010001"; -- 51 inc B
constant inc_C    : opcode_type := "1010010"; -- 52 inc C
constant inc_D    : opcode_type := "1010011"; -- 53 inc D
constant inc_IX   : opcode_type := "1011001"; -- 59 inc IX
constant inc_IY   : opcode_type := "1011010"; -- 5A inc IY

constant dec_A    : opcode_type := "1100000"; -- 60 dec A
constant dec_B    : opcode_type := "1100001"; -- 61 dec B
constant dec_C    : opcode_type := "1100010"; -- 62 dec C
constant dec_D    : opcode_type := "1100011"; -- 63 dec D
constant dly_A    : opcode_type := "1100111"; -- 67 dly A
constant dec_IX   : opcode_type := "1101001"; -- 69 dec IX
constant dec_IY   : opcode_type := "1101010"; -- 6A dec IY

constant and_A_B  : opcode_type := "1110000"; -- 70 and A,B
constant and_A_n  : opcode_type := "1110001"; -- 71 and A,n
constant or_A_B   : opcode_type := "1110010"; -- 72 or A,B
constant or_A_n   : opcode_type := "1110011"; -- 73 or A,n
constant xor_A_B  : opcode_type := "1110100"; -- 74 xor A,B
constant xor_A_n  : opcode_type := "1110101"; -- 75 xor A,n

constant rl_A     : opcode_type := "1111000"; -- 78 rl A
constant rlc_A    : opcode_type := "1111001"; -- 79 rlc A
constant rr_A     : opcode_type := "1111010"; -- 7A rr A
constant rrc_A    : opcode_type := "1111011"; -- 7B rrc A
constant sla_A    : opcode_type := "1111100"; -- 7C sla A
constant sra_A    : opcode_type := "1111101"; -- 7D sra A
constant srl_A    : opcode_type := "1111110"; -- 7E srl A
	
-- Attributes to guide the compiler.
	attribute syn_keep : boolean;
	attribute nomerge : string;

-- Register types for the CPU, ALU, and MUX.
	subtype cpu_reg_type is unsigned(7 downto 0);
	subtype cpu_ca_type is std_logic_vector(ca_top downto 0);
	subtype cpu_pc_type is std_logic_vector(pa_top downto 0);
	subtype alu_result_type is unsigned(8 downto 0);
	subtype alu_ctrl_type is unsigned(3 downto 0);
	
-- ALU inputs and control codes.
	signal alu_in_x, alu_in_y : cpu_reg_type;
	signal alu_cin : boolean;
	signal alu_ctrl : alu_ctrl_type;
	constant alu_cmd_add  : alu_ctrl_type := "0000"; -- Add X and Y
	constant alu_cmd_sub  : alu_ctrl_type := "0001"; -- Subtract Y from X
	constant alu_cmd_and  : alu_ctrl_type := "0100"; -- Bitwise AND of X and Y
	constant alu_cmd_xor  : alu_ctrl_type := "0101"; -- Bitwise XOR of X and Y
	constant alu_cmd_or   : alu_ctrl_type := "0110"; -- Bitwise OR of X and Y
	constant alu_cmd_rl   : alu_ctrl_type := "1000"; -- Rotate Left of X
	constant alu_cmd_rlc  : alu_ctrl_type := "1001"; -- Rotate Left Circuiar of X
	constant alu_cmd_sla  : alu_ctrl_type := "1010"; -- Shift Left Arithmetic of X
	constant alu_cmd_rr   : alu_ctrl_type := "1100"; -- Rotate Right of X
	constant alu_cmd_rrc  : alu_ctrl_type := "1101"; -- Rotate Right Circular of X
	constant alu_cmd_sra  : alu_ctrl_type := "1110"; -- Shift Right Arithmetic of X
	constant alu_cmd_srl  : alu_ctrl_type := "1111"; -- Shift Right Logical of X
	
-- ALU Outputs. We must force the compiler to keep the main alu output signal.-- If we allow the compiler to merge the output with the CPU and MUX, the-- OSR8 code swells by roughly one sixth in size. We do not force it to keep
-- the carry output, however, because we have seen no decrease in size from
-- doing so.
	signal alu_out : cpu_reg_type;
	signal alu_cout : boolean;
	attribute syn_keep of alu_out : signal is true;
	attribute nomerge of alu_out : signal is "";	

-- CPU Registers. The Accumulator, or Register A, in which we get the result 
-- of eight-bit arithmetic operations, logical operations, and shifts and 
-- rotations. When such an operation requires a second operand, we can use 
-- either Register B or a one-byte constant.
	signal reg_A : cpu_reg_type;

-- Register B will operate with the accumulator for eight-bit operatins that
-- require a second operand, such as addition, subtraction, and logical AND.
	signal reg_B  : cpu_reg_type;

-- General-purpose eight-bit registers C, D, H, and L. The last two have 
-- one special function: to act as a loading platform for the stack pointer.
	signal reg_C, reg_D, reg_E, reg_H, reg_L : cpu_reg_type;
	
-- The flags are bits in the Flag Register, but we work with them as separate
-- boolean signals so as to simplify our code. We have the Zero flag, Carry
-- flag, Sign flag, and Interrupt flag.
	signal flag_Z, flag_C, flag_S, flag_I : boolean;

-- The index registers IX and IY are intended for use as pointers to bytes
-- in memory. We can increment them and push them onto the stack, as well
-- as load them with a constant.
	signal reg_IX, reg_IY : cpu_ca_type;

-- The Stack Pointer (SP) we use to manage an upward-growing stack. The
-- Stack Pointer points to the top of the stack, which is the byte most
-- recently pushed onto the stack, and at the highest address of all the 
-- bytes on the stack. When we push a byte onto the stack, we increment
-- the stack pointer, then perform the write. When we pop from the stack, we 
-- read from the stack and then decrement the stack pointer.
	signal reg_SP : cpu_ca_type;
-- State variables and registers for the CPU and its ALU.
	signal opcode_saved : opcode_type;
	
-- The state of the CPU.
	signal state : integer range 0 to 15;
	
-- This signal is the one the CPU sets on the rising edge of CK to
-- indicated that it is executing an interrupt routine. We will delay
-- until the falling edge of CK to make ISRV.
	signal RISRV : boolean;
	
-- The state values. We use single-bit values to simplify the state
-- machine logic. We get faster and more compact logic even though we
-- use eight bits for the state rather than three.
	constant read_opcode : integer := 1;
	constant read_first_operand : integer := 2;
	constant read_second_operand : integer := 3;
	constant read_first_byte : integer := 4;
	constant read_second_byte : integer := 5;
	constant write_second_byte : integer := 6;
	constant read_second_opcode : integer := 7;
	constant incr_pc : integer := 8;
		
-- Functions and Procedures	
	function to_std_logic (v: boolean) return std_ulogic is
	begin if v then return('1'); else return('0'); end if; end function;

begin 

-- The Arithmetic Logic Unit provides an eight-bit adder-subtractor with 
-- carry in and carry out, as well as logical operations AND, OR, and XOR.
	ALU : process (all) is
	variable result : alu_result_type;
	begin 
		result := ('0' & alu_in_x) + ('0' & alu_in_y);
		
		case alu_ctrl is
		
		-- Add X and Y with Carry
		when alu_cmd_add =>
			if alu_cin then 
				result := ('0' & alu_in_x) + ('0' & alu_in_y) + "000000001";
			else 
				result := ('0' & alu_in_x) + ('0' & alu_in_y); 
			end if;
			
		-- Subtract Y from X with Borrow
		when alu_cmd_sub =>
			if alu_cin then 
				result := ('0' & alu_in_x) - ('0' & alu_in_y) - "000000001";
			else 
				result := ('0' & alu_in_x) - ('0' & alu_in_y); 
			end if;
			
		-- Bit-Wise Exclusive OR of A and B
		when alu_cmd_xor =>
			result := '0' & (alu_in_x xor alu_in_y);
			
		-- Bit-Wise OR of A and B
		when alu_cmd_or =>
			result := '0' & (alu_in_x or alu_in_y);
					
		-- Bit-Wise Logical AND of X and Y
		when alu_cmd_and =>
			result := '0' & (alu_in_x and alu_in_y);
					
		-- Rotate Left of X.
		when alu_cmd_rl  => 
			result(8 downto 1) := alu_in_x(7 downto 0);
			result(0) := to_std_logic(alu_cin);	
			
		 -- Rotate Left Circuiar of X
		when alu_cmd_rlc => 
			result(7 downto 1) := alu_in_x(6 downto 0);
			result(0) := alu_in_x(7);	
			result(8) := alu_in_x(7);
			
		-- Rotate Right of X
		when alu_cmd_rr  =>
			result(7) := to_std_logic(alu_cin);
			result(6 downto 0) := alu_in_x(7 downto 1);
			result(8) := alu_in_x(0);		
			
		-- Rotate Right Circular of X
		when alu_cmd_rrc =>  
			result(6 downto 0) := alu_in_x(7 downto 1);
			result(7) := alu_in_x(0);		
			result(8) := alu_in_x(0);		
			
		-- Shift Left Arithmetic of X
		when alu_cmd_sla =>  
			result(8 downto 1) := alu_in_x(7 downto 0);
			result(0) :='0';	
			
		-- Shift Right Arithmetic of X
		when alu_cmd_sra =>  
			result(6 downto 0) := alu_in_x(7 downto 1);
			result(7) := alu_in_x(7);		
			result(8) := alu_in_x(0);		
			
		-- Shift Right Logical of X
		when alu_cmd_srl => 
			result(6 downto 0) := alu_in_x(7 downto 1);
			result(7) := '0';		
			result(8) := alu_in_x(0);		

		-- Let the default values stand.
		when others =>
			null;
			
		end case;
		
		alu_out <= result(7 downto 0);
		alu_cout <= (result(8) = '1');
	end process;

-- The Central Processing Unit reads and writes from an external memory space with
-- its address bus, cpu_addr, and its eight-bit data busses, cpu_data_in and 
-- cpu_data_out, and its control signals CPU Data Strobe (DS), and CPU Write (WR). 
-- Its program memory is separate and private. The CPU instruction set is sufficient 
-- for all integer arithmetic, register exchanges, recursive subroutines, concurrent 
-- processes, and efficient block moves. 
	CPU : process (CK, RESET) is
	
		-- The next_r variables we use to set up the value of A that will be asserted 
		-- on the next rising edge of CK.
		variable next_A, next_B, next_C, next_D, 
			next_E, next_H, next_L : cpu_reg_type;
		
		-- The next values of the flag bits.
		variable next_flag_Z, next_flag_C, next_flag_S, next_flag_I : boolean;
		
		-- The next values of the pointer registers.
		variable next_IX, next_IY, next_SP : std_logic_vector(ca_top downto 0);
				
		-- Variables for the microcode state machine, and constants for the state
		-- names.
		variable opcode : opcode_type;
		variable first_operand, second_operand : cpu_reg_type;
		variable next_state : integer range 0 to 15;
		
		-- We have next_pc to set up the next program counter value.
		variable next_pc : cpu_pc_type;
				
		-- A variable to track changes in the diagnostic outputs.
		variable next_SIG : std_logic_vector(3 downto 0);
		variable next_RISRV : boolean;
						
	begin

		-- Reset the cpu state and program counter until we enter standby mode.
		if (RESET ='1') then 
			state <= read_opcode;
			prog_cntr <= std_logic_vector(to_unsigned(start_pc,prog_cntr_len)); 
			opcode := nop;
			opcode_saved <= nop;
			reg_SP <= (others => '0'); 
			flag_Z <= false;
			flag_C <= false;
			flag_S <= false;
			flag_I <= false;
			RISRV <= false;
			WR <= false;
			DS <= false;
			SIG <= (others => '0');
			
		-- Otherwise we repond to the rising edge of CK.
		elsif rising_edge(CK) then
		
			-- Define default next values. We end up stating explicitly what
			-- the next state and next program counter will be.
			next_state := read_opcode;
			next_pc := std_logic_vector(unsigned(prog_cntr)+1);
			next_A := reg_A;
			next_B := reg_B;
			next_C := reg_C;
			next_D := reg_D;
			next_E := reg_E;
			next_H := reg_H;
			next_L := reg_L;
			next_IX := reg_IX;
			next_IY := reg_IY;
			next_SP := reg_SP;
			next_flag_C := flag_C;
			next_flag_Z := flag_Z;
			next_flag_S := flag_S;
			next_flag_I := flag_I;
			next_RISRV := RISRV;
			WR <= false;
			DS <= false;
			next_SIG(0) := '0';    -- Read opcode signal.
			next_SIG(1) := '0';    -- Executing nop, wait, or dly.
			next_SIG(2) := '0';    -- Jump instruction.
			next_SIG(3) := '0';    -- Jump occurs, any decrement or increment.
			
			-- Read the first byte of the instruction. If the instruction operates only 
			-- upon register, with no constants involved, it will execute here in one state. 
			-- If we have an interrupt request, we deal with that now, before we start 
			-- decoding this instruction, by executing a non-maskable interrupt instruction 
			-- instead of the instruction pointed to by the program counter.
			if (state = read_opcode) then
					
				-- We override the opcode provided by the program data when we have an
				-- interrupt request and we are not currently servicing an interrupt, this
				-- latter condition being indicated by flag_I.
				if IRQ and (not flag_I) then
					opcode := sw_int;
					opcode_saved <= sw_int;
					next_RISRV := true;
				else
					opcode := prog_data(6 downto 0);
					opcode_saved <= prog_data(6 downto 0);
				end if;
				
				-- Make some signals.
				next_SIG(0) := '1';	
				if (opcode = dec_A) or (opcode = dec_B) 
					or (opcode = dec_C) or (opcode = dec_D)
					or (opcode = inc_A) or (opcode = inc_B) 
					or (opcode = inc_C) or (opcode = inc_D)
					or (opcode = inc_IX) or (opcode = inc_IY) 
					or (opcode = dec_IX) or (opcode = dec_IY)
					or (opcode = add_A_n) or (opcode = adc_A_n)
					or (opcode = sub_A_n) or (opcode = sbc_A_n) then
					next_SIG(3) := '1';
				end if;
				if (opcode = nop) or (opcode = cpu_wt) or (opcode = dly_A) then
					next_SIG(1) := '1';
				end if;
					
				-- Decode the instruction.
				case opcode is
				
				-- The no-operation instruction. Clock Cycles = 1.
				when nop => 
					next_state := read_opcode;
					next_pc := std_logic_vector(unsigned(prog_cntr)+1);
					
				-- The wait operation stays in this one place until we receive an
				-- interrupt. It clears the interrupt flag, and so must not be
				-- executed within an interrupt. Clock Cycles = 1.
				when cpu_wt =>
					next_flag_I := false;
					next_pc := prog_cntr;
					next_state := read_opcode;
					
				-- The delay instruction decrements A until it reaches zero, then moves on.
				-- It sets no flags because its end point is always a zero value in A.
				when dly_A => 
					next_A := alu_out;
					if (reg_A = 0) then 
						next_pc := std_logic_vector(unsigned(prog_cntr)+1);
					else
						next_pc := prog_cntr; 
					end if;
					next_state := read_opcode;
					
				-- We handle an interrupt by freezing the program counter and 
				-- pushing its value onto the stack before jumping to the hard-wired
				-- interrupt handler address. We begin by pushing the HI byte of 
				-- the program counter onto the stack. We set flag_I so that the
				-- interrupt cannot itself be interrupted. The interrupt routine 
				-- must do something to clear the Interrupt Request signal (IRQ) 
				-- or else the main process will be interrupted again upon return. 
				-- The interrupt routine should use the return from interrupt 
				-- instruction (ret_int or "rti"), which will clear flag_I upon 
				-- return and execute the interrupted instruction. Clock Cycles = 2.
				when sw_int =>
					next_SP := std_logic_vector(unsigned(reg_SP)+1);
					cpu_addr <= std_logic_vector(unsigned(reg_SP)+1);
					WR <= true;
					DS <= true;
					cpu_data_out <= (others => '0');
					cpu_data_out(pa_top-8 downto 0) <= prog_cntr(pa_top downto 8);
					next_flag_I := true;
					next_state := write_second_byte;
					next_pc := prog_cntr;

				-- Return from subroutine, we pop the LO byte of the program counter 
				-- off the stack, then go on to pop the HI byte and load it into the
				-- program counter. Clock Cycles = 4 except for a return from interrupt
				-- initiated by IRQ, which takes only 3.
				when ret_cll| ret_int =>
					next_SP := std_logic_vector(unsigned(reg_SP)-1);
					cpu_addr <= reg_SP;
					WR <= false;
					DS <= true;
					next_state := read_first_byte;
					next_pc := prog_cntr;
			
				-- Stack Pointer load to and from HL. We use H for the upper bits 
				-- of SP and L for the lower bits. Clock Cycles = 1.
				when ld_HL_SP =>
					next_H(7 downto ca_top-7) := (others => '0');
					next_H(ca_top-8 downto 0) := unsigned(reg_SP(ca_top downto 8));
					next_L := unsigned(reg_SP(7 downto 0));
					next_state := read_opcode;
					next_pc := std_logic_vector(unsigned(prog_cntr)+1);
				when ld_SP_HL =>
					next_SP(ca_top downto 8) := std_logic_vector(reg_H(ca_top-8 downto 0));
					next_SP(7 downto 0) := std_logic_vector(reg_L);
					next_state := read_opcode;
					next_pc := std_logic_vector(unsigned(prog_cntr)+1);

				-- Program Counter load to and from HL. We use H for the upper bits of 
				-- PC. These instructions allow us, in principle, to implement relative 
				-- jumps, by copying the program counter, adding to it, then loading the 
				-- program counter with the result, as in: ld HL,PC; push L; pop A; 
				-- add A,n; push A; pop L; push H; pop A; adc A,n; push A; pop H; 
				-- ld PC,HL; for a total of 16 clock cycles compared to 3 for an
				-- absolute, unconditional jump. Clock Cycles = 1.
				when ld_HL_PC =>
					next_H(7 downto pa_top-7) := (others => '0');
					next_H(pa_top-8 downto 0) := unsigned(prog_cntr(pa_top downto 8));
					next_L := unsigned(prog_cntr(7 downto 0));
					next_state := read_opcode;
					next_pc := std_logic_vector(unsigned(prog_cntr)+1);
				when ld_PC_HL =>
					next_pc(pa_top downto 8) := std_logic_vector(reg_H(pa_top-8 downto 0));
					next_pc(7 downto 0) := std_logic_vector(reg_L);
					next_state := read_opcode;
					next_pc := std_logic_vector(unsigned(prog_cntr)+1);
					
				-- Push operations for eight-bit registers. We immediately generate 
				-- the value SP-1 with our combinatorial decrementer. We drive SP+1 onto 
				-- the address lines on the next rising edge of the clock. On the same
				-- rising edge, SP will decrement to SP-1, and we drive the value we
				-- want to store in the stack onto the data lines. The value will be
				-- stored on the falling edge of the clock in the middle of the next
				-- clock period. By that time, the CPU will be processing the next
				-- instruction. The single-register push instruction, and its converse,
				-- the single-register pop instruction, together allow us to move any
				-- register into any other register, and indeed to re-arrange registers
				-- however we like. If we want to move B into C, C into D, and D into
				-- B we use: push B; push C; push D; pop B; pop D; pop C to perform
				-- the rotation in 9 clock cycles. Clock Cycles = 1.
				when push_A | push_B | push_C | push_D | push_E | push_H | push_L | push_F => 
					next_SP := std_logic_vector(unsigned(reg_SP)+1);
					cpu_addr <= std_logic_vector(unsigned(reg_SP)+1);
					WR <= true;
					DS <= true;
					case opcode is
						when push_A => cpu_data_out <= std_logic_vector(reg_A);
						when push_B => cpu_data_out <= std_logic_vector(reg_B);
						when push_C => cpu_data_out <= std_logic_vector(reg_C);
						when push_D => cpu_data_out <= std_logic_vector(reg_D);
						when push_E => cpu_data_out <= std_logic_vector(reg_E);
						when push_H => cpu_data_out <= std_logic_vector(reg_H);
						when push_L => cpu_data_out <= std_logic_vector(reg_L);
						when push_F =>
							cpu_data_out <= (others => '0');
							cpu_data_out(7) <= to_std_logic(flag_I);
							cpu_data_out(2) <= to_std_logic(flag_C);
							cpu_data_out(1) <= to_std_logic(flag_S);
							cpu_data_out(0) <= to_std_logic(flag_Z);
						when others => null;
					end case;
					next_state := read_opcode;
					next_pc := std_logic_vector(unsigned(prog_cntr)+1);
				
				-- Push operations for index registers. We start by pushing the HI byte onto
				-- the stack, then we push the LO byte. Clock Cycles = 2.
				when push_IX | push_IY => 
					next_SP := std_logic_vector(unsigned(reg_SP)+1);
					cpu_addr <= std_logic_vector(unsigned(reg_SP)+1);
					WR <= true;
					DS <= true;
					cpu_data_out <= (others => '0');
					case opcode is
						when push_IX => cpu_data_out(ca_top-8 downto 0) <= reg_IX(ca_top downto 8);
						when push_IY => cpu_data_out(ca_top-8 downto 0) <= reg_IY(ca_top downto 8);
						when others => null;
					end case;
					next_state := write_second_byte;
					next_pc := std_logic_vector(unsigned(prog_cntr)+1);
					
				-- Pop instructions for eight-bit registers. We immediately generate SP+1
				-- at the output of our combinatorial incrementer. On the next rising 
				-- edge of the clock, we drive SP onto the address lines, and at the same
				-- time increment SP to SP+1. But is is SP that remains on the address lines,
				-- even as the stack pointer register itself increments. On that same rising 
				-- edge we assert DS and unassert WR. The stack will respond on the next 
				-- falling edge by driving a byte onto the data lines. We read the byte on 
				-- the rising edge after that, so we need another clock cycle to complete the 
				-- pop. We move to read_first_byte to perform the read. Clock Cycles = 2.
				when pop_A | pop_B | pop_C | pop_D | pop_E | pop_H | pop_L | pop_F =>
					next_SP := std_logic_vector(unsigned(reg_SP)-1);
					cpu_addr <= reg_SP;
					WR <= false;
					DS <= true;
					next_state := read_first_byte;	
					next_pc := std_logic_vector(unsigned(prog_cntr)+1);
					
				-- Pop operations for index registers. We begin with an eight-bit pop of the
				-- LO byte of the register, and then POP the HI byte in read_second_byte.
				-- Clock Cycles = 3.
				when pop_IX | pop_IY =>
					next_SP := std_logic_vector(unsigned(reg_SP)-1);
					cpu_addr <= reg_SP;
					WR <= false;
					DS <= true;
					next_state := read_first_byte;
					next_pc := std_logic_vector(unsigned(prog_cntr)+1);
					
				-- Eight-bit indirect write operations with index register. We drive the index
				-- register onto the address lines on the next rising edge of the clock and
				-- assert DS and WR. We drive the value of A onto the data lines. On the falling
				-- edge of the clock in the middle of the next clock cycle, the target location
				-- will be written. By that time, we will be processing the next instruction.
				-- Clock Cycles = 1.
				when ld_ix_A =>
					cpu_data_out <= std_logic_vector(reg_A);
					cpu_addr <= reg_IX;
					WR <= true;
					DS <= true;
					next_state := read_opcode;
					next_pc := std_logic_vector(unsigned(prog_cntr)+1);
				when ld_iy_A =>
					cpu_data_out <= std_logic_vector(reg_A);
					cpu_addr <= reg_IY;
					WR <= true;
					DS <= true;
					next_state := read_opcode;
					next_pc := std_logic_vector(unsigned(prog_cntr)+1);
					
				-- Eight-bit indirect read operations with index register. We drive the index
				-- register onto the address lines on the next rising edge of the clcok and
				-- assert DS and unassert WR. On the falling edge in the middle of the next
				-- clock cycle, the target location's value will be driven onto the data bus.
				-- We read that value in a subsequent state, read_first_byte. Clock Cycles = 2.
				when ld_A_ix =>
					cpu_addr <= reg_IX;
					WR <= false;
					DS <= true;
					next_state := read_first_byte;
					next_pc := std_logic_vector(unsigned(prog_cntr)+1);
				when ld_A_iy =>
					cpu_addr <= reg_IY;
					WR <= false;
					DS <= true;
					next_state := read_first_byte;
					next_pc := std_logic_vector(unsigned(prog_cntr)+1);
					
				-- Eight-bit register increments and decrements, which use the combinatorial
				-- eight-bit adder to performm the adjustment. These operations set the zero and
				-- sign flags, but not the carry flag. Clock cycles = 1.
				when dec_A | inc_A => 
					next_A := alu_out;
					next_flag_Z := (alu_out = 0);
					next_flag_S := (alu_out >= 128);
					next_state := read_opcode;
					next_pc := std_logic_vector(unsigned(prog_cntr)+1);
				when dec_B | inc_B => 
					next_B := alu_out;
					next_flag_Z := (alu_out = 0);
					next_flag_S := (alu_out >= 128);
					next_state := read_opcode;
					next_pc := std_logic_vector(unsigned(prog_cntr)+1);
				when dec_C | inc_C => 
					next_C := alu_out;
					next_flag_Z := (alu_out = 0);
					next_flag_S := (alu_out >= 128);
					next_state := read_opcode;
					next_pc := std_logic_vector(unsigned(prog_cntr)+1);
				when dec_D | inc_D => 
					next_D := alu_out;
					next_flag_Z := (alu_out = 0);
					next_flag_S := (alu_out >= 128);
					next_state := read_opcode;
					next_pc := std_logic_vector(unsigned(prog_cntr)+1);
					
				-- Address pointer increments and decrements. These do not affect the
				-- carry, sign, or zero flags. We use our combinatorial incrementer to
				-- add or subtract one from the pointer value. Clock Cycles = 1.
				when inc_IX => 
					next_IX := std_logic_vector(unsigned(reg_IX)+1);
					next_state := read_opcode;
					next_pc := std_logic_vector(unsigned(prog_cntr)+1);
				when dec_IX => 
					next_IX := std_logic_vector(unsigned(reg_IX)-1);
					next_state := read_opcode;
					next_pc := std_logic_vector(unsigned(prog_cntr)+1);
				when inc_IY => 
					next_IY := std_logic_vector(unsigned(reg_IY)+1);
					next_state := read_opcode;
					next_pc := std_logic_vector(unsigned(prog_cntr)+1);
				when dec_IY => 
					next_IY := std_logic_vector(unsigned(reg_IY)-1);
					next_state := read_opcode;
					next_pc := std_logic_vector(unsigned(prog_cntr)+1);
					
				-- Arithmetic operations, in which wse can add B register to the 
				-- accumulator, A, or subtract B, with carry or without. The carry, 
				-- sign, and zero flags are all set or cleared by these operations. 
				-- The calculation is performed by our combinatorial eight-bit 
				-- adder. Clock Cycles = 1.
				when add_A_B | sub_A_B | adc_A_B | sbc_A_B =>
					next_A := alu_out;
					next_flag_Z := (alu_out = 0);
					next_flag_C := alu_cout;
					next_flag_S := (alu_out >= 128);
					next_state := read_opcode;
					next_pc := std_logic_vector(unsigned(prog_cntr)+1);

				-- Eight-bit logical operations in the accumulator with register
				-- B as the second operand. Sets the Z flag only. Clock Cycles = 1.
				when and_A_B | or_A_B | xor_A_B => 
					next_A := alu_out;
					next_flag_Z := (alu_out = 0);
					next_state := read_opcode;
					next_pc := std_logic_vector(unsigned(prog_cntr)+1);
					
				-- Clear arithmetic flags: a single instruction clears the sign (S), 
				-- zero (Z), and carry (C) flags, but not the interrupt (I) flag;
				when clr_aflg =>
					next_flag_Z := false;
					next_flag_C := false;
					next_flag_S := false;
					next_state := read_opcode;
					next_pc := std_logic_vector(unsigned(prog_cntr)+1);

				-- Clear interrupt flag, so as to enable interrupts.
				when clr_iflg =>
					next_flag_I := false;
					next_state := read_opcode;
					next_pc := std_logic_vector(unsigned(prog_cntr)+1);

				-- Set interrupt flag, so as to disable interrupts.
				when set_iflg =>
					next_flag_I := true;
					next_state := read_opcode;
					next_pc := std_logic_vector(unsigned(prog_cntr)+1);
					
				-- Shift and rotate operations in the accumulator. All affect the
				-- carry bit, but none affect the zero or sign bits. Clock Cycles = 1.
				when rl_A | rlc_A | rr_A | rrc_A | sla_A | sra_A | srl_A =>
					next_A := alu_out;
					next_flag_C := alu_cout;
					next_state := read_opcode;
					next_pc := std_logic_vector(unsigned(prog_cntr)+1);
					
				-- All other instructions need an operand. We don't bother to decode them
				-- separately at this point.
				when others => 
					next_state := read_first_operand;
					next_pc := std_logic_vector(unsigned(prog_cntr)+1);
					
				end case;

			-- Instructions have either one or two operands, which will be provided HI-byte
			-- first, as the program counter increments through the program space, and our
			-- byte ordering is big-endian. Here we read the first operand. If we have enough
			-- information to perform a write cycle, we do so immediately. We increment
			-- the program counter, either to point to the first byte of the next instruction, 
			-- or to point to the second operand, which will be the LO byte of a parameter.
			elsif (state = read_first_operand) then
				first_operand := unsigned(prog_data);
				
				case opcode_saved is
				
				-- We load a constant into the accumulator. Clock Cycles = 2.
				when ld_A_n => 
					next_A := first_operand;
					next_state := read_opcode;
					next_pc := std_logic_vector(unsigned(prog_cntr)+1);
				
				-- We use our combinatorial adder to combine a constant and the
				-- accumulator value, and clock this new value into the accumulator
				-- on the next rising edge of the clock. Sets the Z, C, and S 
				-- flags. Clock Cycles = 2.
				when add_A_n | sub_A_n | adc_A_n | sbc_A_n =>
					next_A := alu_out;
					next_flag_Z := (alu_out = 0);
					next_flag_C := alu_cout;
					next_flag_S := (alu_out >= 128);
					next_state := read_opcode;
					next_pc := std_logic_vector(unsigned(prog_cntr)+1);
					
				-- Logical operations upon the accumulator with a constant as the
				-- second operand. Sets the Z flag only. Clock Cycles = 2.
				when and_A_n | or_A_n | xor_A_n => 
					next_A := alu_out;
					next_flag_Z := (alu_out = 0);
					next_state := read_opcode;
					next_pc := std_logic_vector(unsigned(prog_cntr)+1);

				-- All others require a second operand, so we don't decode them yet.
				when others => 
					next_state := read_second_operand;
					next_pc := std_logic_vector(unsigned(prog_cntr)+1);
					
				end case;
			
			-- Read a second operand, which will be the LO byte of a sixteen-bite 
			-- value. If we have enough information to begin a write cycle, we do so.
			-- We increment the program counter to point to the next instruction opcode.
			elsif (state = read_second_operand) then
				second_operand := unsigned(prog_data);
				
				case opcode_saved is 
				
				-- Indirect eight-bit read operations using operands as address. The first
				-- operand, being the one we loaded from a lower address, is the HI byte
				-- of the address, and the second operand is the LO byte. We drive the 
				-- address onto the address lines on the next rising edge, along with 
				-- DS and not WR. We read the value of the target location in the next
				-- clock cycle, in the read_first_byte state. Clock Cycles = 4.
				when ld_A_mm =>
					cpu_addr(ca_top downto 8) <= 
						std_logic_vector(first_operand(ca_top-8 downto 0));
					cpu_addr(7 downto 0) <= 
						std_logic_vector(second_operand);
					WR <= false;
					DS <= true;
					next_state := read_first_byte;
					next_pc := std_logic_vector(unsigned(prog_cntr)+1);
					
				-- Indirect eight-bit write operations using operands as address. Same as
				-- the read cycle, except we assert WR on the next rising edge, and drive
				-- the data lines with the value of A, and then leave the targe location
				-- to be written in the middle of the next cycle, while we get on with
				-- decoding the next instruction. Clock Cycles = 3.
				when ld_mm_A =>
					cpu_addr(ca_top downto 8) <= 
						std_logic_vector(first_operand(ca_top-8 downto 0));
					cpu_addr(7 downto 0) <= std_logic_vector(second_operand);
					cpu_data_out <= std_logic_vector(reg_A);
					WR <= true;
					DS <= true;
					next_state := read_opcode;
					next_pc := std_logic_vector(unsigned(prog_cntr)+1);
					
				-- Direct load of constants into index registers. Clock Cycles = 3.
				when ld_IX_nn =>
					next_IX(ca_top downto 8) := 
						std_logic_vector(first_operand(ca_top-8 downto 0));
					next_IX(7 downto 0) := std_logic_vector(second_operand);
					next_state := read_opcode;
					next_pc := std_logic_vector(unsigned(prog_cntr)+1);
				when ld_IY_nn =>
					next_IY(ca_top downto 8) := 
						std_logic_vector(first_operand(ca_top-8 downto 0));
					next_IY(7 downto 0) := std_logic_vector(second_operand);
					next_state := read_opcode;
					next_pc := std_logic_vector(unsigned(prog_cntr)+1);
					
				-- Direct load of sixteen-bit constant into HL. Clock Cycles = 3;
				when ld_HL_nn =>
					next_H := first_operand;
					next_L := second_operand;
					next_state := read_opcode;
					next_pc := std_logic_vector(unsigned(prog_cntr)+1);
					
				-- Call a subroutine. We now have the full address to jump to, and we don't
				-- have to increment the program counter to read any more constants, so we
				-- can push the LO byte onto the stack, and push the HI byte in the next
				-- clock cycle, in the write_second_byte state. We save the program counter
				-- HI byte to push onto the stack in the next clock cycle. Clock Cycles = 4.
				when call_nn =>
					next_SP := std_logic_vector(unsigned(reg_SP)+1);
					cpu_addr <= std_logic_vector(unsigned(reg_SP)+1);
					WR <= true;
					DS <= true;
					cpu_data_out <= (others => '0');
					cpu_data_out(pa_top-8 downto 0) <= prog_cntr(pa_top downto 8);
					next_state := write_second_byte;
					next_pc := prog_cntr;
				
				-- Decide if we should jump to the address specified by the two operands. If
				-- so, we will jump immediately by loading the program counter with a new 
				-- value. Clock Cycles = 3.
				when jp_nn | jp_z_nn | jp_nz_nn | jp_c_nn | jp_nc_nn | jp_p_nn | jp_np_nn =>
					next_SIG(2) := '1';
					if (opcode_saved = jp_nn) or 
						((opcode_saved = jp_z_nn) and flag_Z) or
						((opcode_saved = jp_nz_nn) and (not flag_Z)) or
						((opcode_saved = jp_c_nn) and flag_C) or
						((opcode_saved = jp_nc_nn) and (not flag_C)) or
						((opcode_saved = jp_p_nn) and (not flag_S)) or
						((opcode_saved = jp_np_nn) and flag_S) then
						next_SIG(3) := '1';
						next_pc(pa_top downto 8) := 
							std_logic_vector(first_operand(pa_top-8 downto 0));
						next_pc(7 downto 0) := 
							std_logic_vector(second_operand);
					else 
						next_pc := std_logic_vector(unsigned(prog_cntr)+1);
					end if;
					next_state := read_opcode;
					
				-- Let the defaults stand;
				when others => null;					
					
				end case;
				
			-- Read the first byte of data from the data bus. This could be a user memory access
			-- or a stack access. We never increment the program counter from this state because
			-- we have already obtained the full instruction.
			elsif (state = read_first_byte) then			
				case opcode_saved is
				
				-- By now, the address lines have selected the target location, and half-way
				-- through this clock cycle, the target byte will be driven onto the data
				-- bus. On the next rising edge, even as we start decoding the next instruction,
				-- we will load the new value into the accumulator. Clock cycles = 2.
				when ld_A_mm | ld_A_ix | ld_A_iy => 
					next_A := unsigned(cpu_data_in);
					next_pc := prog_cntr;
					next_state := read_opcode;
					
				-- We have selected a byte in the stack with the stack pointer, and half-way
				-- through this cycle it will appear on the data bus. We store it in the
				-- destination register on the next rising edge of the clock. 
				when pop_A | pop_B | pop_C | pop_D | pop_E | pop_H | pop_L | pop_F =>
					case opcode_saved is
						when pop_A => next_A := unsigned(cpu_data_in);
						when pop_B => next_B := unsigned(cpu_data_in);
						when pop_C => next_C := unsigned(cpu_data_in);
						when pop_D => next_D := unsigned(cpu_data_in);
						when pop_E => next_E := unsigned(cpu_data_in);
						when pop_H => next_H := unsigned(cpu_data_in);
						when pop_L => next_L := unsigned(cpu_data_in);
						when pop_F => 
							next_flag_I := (cpu_data_in(7) = '1');
							next_flag_C := (cpu_data_in(2) = '1');		
							next_flag_S := (cpu_data_in(1) = '1');
							next_flag_Z := (cpu_data_in(0) = '1');
						when others => null;
					end case;
					next_pc := prog_cntr;
					next_state := read_opcode;
					
				-- Read the LO byte of the index register from the stack, and prepare to read
				-- the HI byte in the next clock cycle, in the read_second_byte state. 
				when pop_IX | pop_IY =>
					next_SP := std_logic_vector(unsigned(reg_SP)-1);
					cpu_addr <= reg_SP;
					WR <= false;
					DS <= true;
					case opcode_saved is
						when pop_IX => next_IX(7 downto 0) := cpu_data_in(7 downto 0);
						when pop_IY => next_IY(7 downto 0) := cpu_data_in(7 downto 0);
						when others => null;
					end case;
					next_pc := prog_cntr;
					next_state := read_second_byte;	
					
				-- Read the LO byte of the program counter from the stack and prepare to read 
				-- the HI byte in read_second_byte.
				when ret_cll | ret_int =>
					next_SP := std_logic_vector(unsigned(reg_SP)-1);
					cpu_addr <= reg_SP;
					WR <= false;
					DS <= true;
					next_pc := (others => '0');
					next_pc(7 downto 0) := cpu_data_in;
					next_state := read_second_byte;	
					
				-- Let the defaults stand;
				when others => null;

			end case;
	
			-- Read the second byte of data. We never increment the program counter from
			-- this state, but we will set it to a the value popped off the stack by a
			-- return instruction.
			elsif (state = read_second_byte) then
				case opcode_saved is
				
				-- Read the HI byte of index pointer from the stack.
				when pop_IX => 
					next_IX(ca_top downto 8) := cpu_data_in(ca_top-8 downto 0);
					next_pc := prog_cntr;
					next_state := read_opcode;
				when pop_IY => 
					next_IY(ca_top downto 8) := cpu_data_in(ca_top-8 downto 0);
					next_pc := prog_cntr;
					next_state := read_opcode;
	
				-- We are returning from a subroutine. We have already read the
				-- LO byte of the program counter from the stack, decremented
				-- the stack pointer, and now we are reading the HI byte. We use the 
				-- incr_pc state to increment the program counter so as to select
				-- the next instruction when we are returning from call_nn. But if
				-- we are returning from an interrupt, we leave the program counter 
				-- as it is and clear flag_I so we can execute the instruction that 
				-- was interrupted and be ready for the next interrupt.
				when ret_cll | ret_int =>
					next_pc(pa_top downto 8) := cpu_data_in(pa_top-8 downto 0);
					next_pc(7 downto 0) := prog_cntr(7 downto 0);	
					if opcode_saved = ret_cll then
						next_state := incr_pc;
					else 
						next_flag_I := false;
						next_RISRV := false;
						next_state := read_opcode;
					end if;
				
				-- Let the defaults stand;
				when others => null;
				
				end case;
			
			-- Write the second byte of data. We have no "write first byte" state because
			-- we write the first byte immediately upon decoding the instruction. We never
			-- increment the program counter in this state, but we will set it to the value
			-- specified by the two operands in a call instruction, having saved the previous
			-- program counter to the stack.
			elsif (state = write_second_byte) then
				case opcode_saved is
				
				-- We increment the stack pointer and write the LO byte of the index 
				-- pointers to the stack, having already pushed the HI byte to the
				-- lower address. 
				when push_IX | push_IY =>
					next_SP := std_logic_vector(unsigned(reg_SP)+1);
					cpu_addr <= std_logic_vector(unsigned(reg_SP)+1);
					WR <= true;
					DS <= true;
					case opcode_saved is
						when push_IX => cpu_data_out(7 downto 0) <= reg_IX(7 downto 0);
						when push_IY => cpu_data_out(7 downto 0) <= reg_IY(7 downto 0);
						when others => null;
					end case;
					next_pc := prog_cntr;
					next_state := read_opcode;
					
				-- Here we are completing a call or sw_int by pushing the LO byte of the 
				-- program counter onto the stack, and setting the next value of the program 
				-- counter to the address given by the first operand (HI byte) and second
				-- operand (LO byte).
				when call_nn | sw_int =>
					next_SP := std_logic_vector(unsigned(reg_SP)+1);
					cpu_addr <= std_logic_vector(unsigned(reg_SP)+1);
					WR <= true;
					DS <= true;
					cpu_data_out <= prog_cntr(7 downto 0);
					if (opcode_saved = call_nn) then
						next_pc(pa_top downto 8) := 
							std_logic_vector(first_operand(pa_top-8 downto 0));
						next_pc(7 downto 0) := 
							std_logic_vector(second_operand);
					else 
						next_pc := std_logic_vector(to_unsigned(interrupt_pc, prog_cntr_len));
					end if;
					next_state := read_opcode;
				
				-- Let the defaults stand;
				when others => null;
									
				end case;
				
			-- The increment-program-counter state increments the program counter and moves
			-- to the read-opcode state. We use it when returning from a subroutine or a
			-- software interrupt, to increment the program counter we popped off the stack
			-- so that it points to the next instruction after the call or software interrupt.
			elsif (state = incr_pc) then
				next_state := read_opcode;
				next_pc := std_logic_vector(unsigned(prog_cntr)+1);	
			end if;
			
			-- Assert the new state, program counter, and register values for the next clock cycle.
			prog_cntr <= next_pc; 
			state <= next_state;					
			reg_A <= next_A;
			reg_B <= next_B;
			reg_C <= next_C;
			reg_D <= next_D;
			reg_E <= next_E;
			reg_H <= next_H;
			reg_L <= next_L;
			reg_IX <= next_IX;
			reg_IY <= next_IY;
			reg_SP <= next_SP;
			flag_C <= next_flag_C;
			flag_S <= next_flag_S;
			flag_Z <= next_flag_Z;
			flag_I <= next_flag_I;
			SIG <= next_SIG;
			RISRV <= next_RISRV;
		end if;
	end process;

	-- Here we have combinatorial logic that applies the Arithmetic Logic Unit to eight-bit 
	-- registers and constants. One of the two inputs to the ALU is always the accumulator,
	-- which we name reg_A in the code. We use the ALU to increment and decrement registers
	-- as well as add, subtract, shift, rotate, and combine them logically. We begin with 
	-- the behavior of the ALU when we are in the read_opcode state. We use the current 
	-- value of the program data as the opcode that controls the function of the ALU.
	MUX : process (all) is
		variable opcode_now : opcode_type;
		variable data_now : cpu_reg_type;

	begin		opcode_now := prog_data(6 downto 0);
		data_now := unsigned(prog_data(7 downto 0));
		alu_in_x <= reg_A;
		alu_in_y <= reg_B;
		alu_cin  <= false;
		alu_ctrl <= alu_cmd_and;
		
		if (state = read_opcode) then
		
			case opcode_now is
			
			-- Eight-bit register increment and decrement.
			when inc_A | dec_A =>
				alu_in_x <= reg_A;
				alu_in_y <= "00000001";
				alu_cin <= false;
				if (opcode_now = inc_A) then alu_ctrl <= alu_cmd_add;
				else alu_ctrl <= alu_cmd_sub; end if;	
			when inc_B | dec_B =>
				alu_in_x <= reg_B;
				alu_in_y <= "00000001";
				alu_cin <= false;
				if (opcode_now = inc_B) then alu_ctrl <= alu_cmd_add; 
				else alu_ctrl <= alu_cmd_sub; end if;	
			when inc_C | dec_C =>
				alu_in_x <= reg_C;
				alu_in_y <= "00000001";
				alu_cin <= false;
				if (opcode_now = inc_C) then alu_ctrl <= alu_cmd_add; 
				else alu_ctrl <= alu_cmd_sub; end if;	
			when inc_D | dec_D =>
				alu_in_x <= reg_D;
				alu_in_y <= "00000001";
				alu_cin <= false;
				if (opcode_now = inc_D) then alu_ctrl <= alu_cmd_add;
				else alu_ctrl <= alu_cmd_sub; end if;	
				
			-- The delay instruction decrements A until it is zero.
			when dly_A =>
				alu_in_x <= reg_A;
				alu_in_y <= "00000001";
				alu_cin <= false;
				alu_ctrl <= alu_cmd_sub;
			
			-- Mathematical operations with A and B.
			when add_A_B | sub_A_B | adc_A_B | sbc_A_B =>
				alu_in_x <= reg_A;
				alu_in_y <= reg_B;
				alu_cin <= flag_C and ((opcode_now = adc_A_B) or (opcode_now = sbc_A_B));
				if (opcode_now = add_A_B) or (opcode_now = adc_A_B) then
					alu_ctrl <= alu_cmd_add;
				else 
					alu_ctrl <= alu_cmd_sub;
				end if;
				
			-- Logical operations on A with B.
			when and_A_B =>
				alu_in_x <= reg_A;
				alu_in_y <= reg_B;
				alu_ctrl <= alu_cmd_and;
			when or_A_B =>
				alu_in_x <= reg_A;
				alu_in_y <= reg_B;
				alu_ctrl <= alu_cmd_or;
			when xor_A_B =>
				alu_in_x <= reg_A;
				alu_in_y <= reg_B;
				alu_ctrl <= alu_cmd_xor;
			
			-- Shift and Rotate Operations on A. We specify reg_B for alu_in_Y
			-- because it reduces our logic footprint.
			when rl_A | rlc_A | rr_A | rrc_A | sla_A | sra_A | srl_A =>
				alu_in_x <= reg_A;
				alu_in_y <= reg_B;
				alu_cin <= flag_C;
				case opcode_now is 
					when rl_A  => alu_ctrl <= alu_cmd_rl;
					when rlc_A => alu_ctrl <= alu_cmd_rlc;
					when rr_A  => alu_ctrl <= alu_cmd_rr;
					when rrc_A => alu_ctrl <= alu_cmd_rrc;
					when sla_A => alu_ctrl <= alu_cmd_sla;
					when sra_A => alu_ctrl <= alu_cmd_sra;
					when srl_A => alu_ctrl <= alu_cmd_srl;	
					when others => null;
				end case;
			
			-- In all other cases, we allow defaults to stand.
			when others =>
				null;
			
			end case;
			
		-- If we are not in the read_opcode state, we have a signal "opcode_saved" that 
		-- holds the value of the opcode that was presented in the most recent read_opcode 
		-- state. We use this variable to control the behavior of the ALU. The operand we are 
		-- going to use for the Y-input of the ALU will always be the current value of the
		-- program data, so we do not use the state machine's "first_operand" variable, but
		-- the program data directly.
		else 
			case opcode_saved is 			
			-- Operations with A and a constant.
			when add_A_n | sub_A_n | adc_A_n | sbc_A_n =>
				alu_in_x <= reg_A;
				alu_in_y <= data_now;
				alu_cin <= flag_C and ((opcode_saved = adc_A_n) or (opcode_saved = sbc_A_n));
				if (opcode_saved = add_A_n) or (opcode_saved = adc_A_n) then
					alu_ctrl <= alu_cmd_add;
				else 
					alu_ctrl <= alu_cmd_sub;
				end if;		
				
			-- Logical operations on A and a constant. We specify reg_B for alu_in_Y
			-- because it reduces our logic footprint.
			when and_A_n =>
				alu_in_x <= reg_A;
				alu_in_y <= data_now;
				alu_ctrl <= alu_cmd_and;
			when or_A_n =>
				alu_in_x <= reg_A;
				alu_in_y <= data_now;
				alu_ctrl <= alu_cmd_or;
			when xor_A_n =>
				alu_in_x <= reg_A;
				alu_in_y <= data_now;
				alu_ctrl <= alu_cmd_xor;
				
			-- In all other cases, allow defaults to stand.
			when others =>
				null;
			
			end case;
		end if;
	end process;
	
	-- The ISRV Delay takes RISRV generated on the rising edge of CK
	-- and delays until the falling edge, so as to match signals written
	-- to registers by the CPU. The resulting ISRV signal can be used
	-- to boost the CPU clock in the same way that a write to a BOOST
	-- and ENFCK register can do the same.
	ISRV_Delay : process (RESET, CK) is
	begin
		if (RESET = '1') then
			ISRV <= false;
		elsif falling_edge(CK) then
			ISRV <= RISRV;
		end if;
	end process;
end behavior;

