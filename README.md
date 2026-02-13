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

- OSR8V3: Introduced in 2022, stable and in continuous use through 2025. No known
bugs. Generic constants for address lengths allow deployment in a wide range
of sizes. The 4-KByte program memory, 2-KByte user memory version takes up
800 LUTs in a MachXO2.

- OSR8V4: Started in 2026, planning addition of an interrupt execution flag
to support automatic clock boost during interrupt service routines. No known 
bugs. Still backward-compatible with OSR8V3, but may not remain so.
