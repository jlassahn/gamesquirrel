
ENTRY(_start)

MEMORY
{
	DRAM (rw) : ORIGIN = 0x90000000, LENGTH = 0x003FC000
	IRAM (xr) : ORIGIN = 0x04000000, LENGTH = 0x003FC000
}

SECTIONS
{
	.text :
	{
		. = ALIGN(4);
		*(.text)           /* .text sections (code) */
		*(.text*)          /* .text* sections (code) */
		*(.glue_7)         /* glue arm to thumb code */
		*(.glue_7t)        /* glue thumb to arm code */
		*(.eh_frame)

		KEEP (*(.init))
		KEEP (*(.fini))

		*(.rodata)         /* .rodata sections (constants, strings, etc.) */
		*(.rodata*)        /* .rodata* sections (constants, strings, etc.) */
		. = ALIGN(4);

		. = ALIGN(4);
		*(.ARM.extab* .gnu.linkonce.armextab.*)
		. = ALIGN(4);

		. = ALIGN(4);
		__exidx_start = .;
		*(.ARM.exidx*)
		__exidx_end = .;
		. = ALIGN(4);

		. = ALIGN(4);
		PROVIDE_HIDDEN (__preinit_array_start = .);
		KEEP (*(.preinit_array*))
		PROVIDE_HIDDEN (__preinit_array_end = .);
		. = ALIGN(4);

		. = ALIGN(4);
		PROVIDE_HIDDEN (__init_array_start = .);
		KEEP (*(SORT(.init_array.*)))
		KEEP (*(.init_array*))
		PROVIDE_HIDDEN (__init_array_end = .);
		. = ALIGN(4);

		. = ALIGN(4);
		PROVIDE_HIDDEN (__fini_array_start = .);
		KEEP (*(SORT(.fini_array.*)))
		KEEP (*(.fini_array*))
		PROVIDE_HIDDEN (__fini_array_end = .);
		. = ALIGN(4);

	} >IRAM AT> DRAM

	.data :
	{
		. = ALIGN(4);
		*(.data)           /* .data sections */
		*(.data*)          /* .data* sections */
		*(.RamFunc)        /* .RamFunc sections */
		*(.RamFunc*)       /* .RamFunc* sections */

		. = ALIGN(4);
	} >DRAM

	.bss :
	{
		*(.bss)
		*(.bss*)
		*(COMMON)
	} >DRAM

	.notes 0 (TYPE = SHT_NOTE) :
	{
		LONG(14)
		LONG(4)
		LONG(1)
		ASCIZ "Squirrel.Stack\0"
		LONG(4096)
	}
}

