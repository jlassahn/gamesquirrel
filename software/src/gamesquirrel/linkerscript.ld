
ENTRY(Reset_Handler)

/* Reserve 12K for stack */
STACK_SIZE = 0x3000;

MEMORY
{
	XDRAM (rw)  : ORIGIN = 0x903FC000, LENGTH = 0x00004000
	DRAM  (rw)  : ORIGIN = 0x90000000, LENGTH = 0x003FC000
	XSRAM (rw)  : ORIGIN = 0x20043000, LENGTH = 0x00001000
	SRAM  (rw)  : ORIGIN = 0x20000000, LENGTH = 0x00043000
	FLASH (xr)  : ORIGIN = 0x08000000, LENGTH = 0x00080000
	IRAM  (xr)  : ORIGIN = 0x04000000, LENGTH = 0x003FC000
}


SECTIONS
{
	/* System tables at the start of flash */
	.sys :
	{
		/* Interrupt vector table */
		KEEP(*(.isr_vector))

		/* System library table at fixed address 0x08000400 */
		. = ORIGIN(FLASH) + 0x0400;
		KEEP(*(.syslib))
	} > FLASH

	/* Executable code */
	.text :
	{
		*(.text*)
		*(.text)
		*(.eh_frame)
		KEEP (*(.init))
		KEEP (*(.fini))

		/* _etext = .;  */       /* FIXME Do we need this? */
	} > FLASH

	/* Read-only data */
	.rodata :
	{
		*(.rodata)
		*(.rodata*)
	} > FLASH

	/* Constructors, exception handling, etc FIXME do we need? */
	.cppdata :
	{
		*(.ARM.extab* .gnu.linkonce.armextab.*)
		__exidx_start = .;
		*(.ARM.exidx*)
		__exidx_end = .;

		PROVIDE_HIDDEN (__preinit_array_start = .);
		KEEP (*(.preinit_array*))
		PROVIDE_HIDDEN (__preinit_array_end = .);

		PROVIDE_HIDDEN (__init_array_start = .);
		KEEP (*(SORT(.init_array.*)))
		KEEP (*(.init_array*))
		PROVIDE_HIDDEN (__init_array_end = .);

		/* FIXME definitely don't need fini, we're never going to shut down. */
		PROVIDE_HIDDEN (__fini_array_start = .);
		KEEP (*(SORT(.fini_array.*)))
		KEEP (*(.fini_array*))
		PROVIDE_HIDDEN (__fini_array_end = .);
	} > FLASH

	/* Reserve space for stack */
	.stack (NOLOAD):
	{
		. = . + STACK_SIZE;
		_estack = .;
	} >SRAM

	/* Initialized data */
	.data :
	{
		_sdata = .;
		*(.data)
		*(.data*)
		_edata = .;
	} >SRAM AT> FLASH
	_sidata = LOADADDR(.data);

	/* Uninitialized data */
	.bss :
	{
		_sbss = .;
		*(.bss)
		*(.bss*)
		*(COMMON)
		. = ALIGN(4);
		_ebss = .;
		end = _ebss; /* Beginning of heap. */
	} >SRAM

	/* Remove information from the compiler libraries */
	/DISCARD/ :
	{
		libc.a ( * )
		libm.a ( * )
		libgcc.a ( * )
	}

	/* FIXME do we need? */
	.ARM.attributes 0 : { *(.ARM.attributes) }
}
