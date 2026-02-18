# OSR8
Open Source Reconfigurable Eight-Bit Processor (OSR8)

The OSR8 is an open-source, reconfigurable eight-bit processor described in 
VHDL (Virtual Hardware Description Language). For a full description of the
processor, its instruction set, its development, and its performance when
deployed in Lattice Semiconductor's MachXO2 family of FPGAs, see the
[OSR8 Manual](https://www.opensourceinstruments.com/Electronics/A3035/OSR8.html).

Each time we release a new version of the OSR8 we give it a new version suffix
to its name. Within each version, we make improvements and fix bugs, but we do
nothing deliberate that will render the code incompatible with an existing project.
The OSR8V1 and OSR8V2 versions are obsolete. The currently-supported version are
OSR8V3 and OSR8V4.

- OSR8V3: Finalized 18-JUN-22. Deployed in hundreds of implantable devices and
motherboards. The processor source file does not provide complete constraints for 
all signals and variables. Although the OSR8V3 contains no known bugs, it is hard
to work with because any error in our peripheral logic code combines with the
incomplete constraints of the OSR8V3 to produce erratice behavior. This behavior
often gives very little clue as to which parts of our peripheral logic are going
wrong. A fully-constrained version of the OSR8V3 was too large to fit in the 
1280-LUT logic chip we use for our implants along with all the peripheral logic
required by the implant functions, so we relaxed the OSR8V3 constraints, which 
reduced its size. We recommend against using the OSR8V3 in new projects. Later
versions are fully-constrained, after removing eight instructions we never used
in any of our applications.

- OSR8V4: Initialized on 10-FEB-26. Requires renaming prog_addr to prog_cntr in 
the port map when moving from OSR8V3 to OSR8V4. Removed eight instructions from
the OSR8V3 in order reduce the size of the compiled code. Added complete
constraints to all variables and signals so as to make behavior less erratic in
the presence of bugs in peripheral logic.