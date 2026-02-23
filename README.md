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
motherboards. A fully-constrained version of the OSR8V3 was too large to fit in
the 1280-LUT logic chip we use for our implants along with all the peripheral
logic required by the implant functions, so we relaxed the OSR8V3 constraints,
which reduced its size. But the incomplete constraints made the OSR8V3 hard to
work with. Any error in our peripheral logic combines with the incomplete
constraints of signals within the OSR8V3 to produce erratic behavior. We might
route a signal to a test point and the logic will work, and then route a
different signal to the same test point and the logic will stop working. We
recommend against using the OSR8V3 in new projects. Later versions are
fully-constrained and no larger than the OSR8V3 after removing eight
instructions we never used.

- OSR8V4: Initialized on 10-FEB-26. Requires renaming prog_addr to prog_cntr in
the port map when moving from OSR8V3 to OSR8V4. Reduced the CPU's size by
removing eight never-used instructions from its instruction set. Added complete
constraints of all variables and signals. Errors in peripheral logic code no
longer cause erratic behavior of the processor. Add an Interrupt Service (ISRV)
output that the peripheral logic can use to boost the CPU clock during
interrupts. We plan to add to the OSR8V4 a Kernel Request (KRQ) input and a
Kernal Service (KSRV) output to allow a non-interruptable kernel process to run
on the CPU. The kernel process will allow devices to attend to incoming commands
even when executing user code that has put the CPU in an infinite loop inside a
normal-priority interrupt.