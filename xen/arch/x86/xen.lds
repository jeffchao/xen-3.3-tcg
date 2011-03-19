OUTPUT_FORMAT("elf32-i386", "elf32-i386", "elf32-i386")
OUTPUT_ARCH(i386)
ENTRY(start)
PHDRS
{
  text PT_LOAD ;
}
SECTIONS
{
  . = 0xFF000000 + 0x100000;
  _start = .;
  _stext = .;
  .text : {
 *(.text)
 *(.fixup)
 *(.gnu.warning)
 } :text =0x9090
  .text.lock : { *(.text.lock) } :text
  _etext = .;
  .rodata : { *(.rodata) *(.rodata.*) } :text
  . = ALIGN(32);
  __start___ex_table = .;
  __ex_table : { *(__ex_table) } :text
  __stop___ex_table = .;
  . = ALIGN(32);
  __start___pre_ex_table = .;
  __pre_ex_table : { *(__pre_ex_table) } :text
  __stop___pre_ex_table = .;
  .data : {
 *(.data)
 CONSTRUCTORS
 } :text
  . = ALIGN(128);
  .data.read_mostly : { *(.data.read_mostly) } :text
  . = ALIGN(4096);
  __init_begin = .;
  _sinittext = .;
  .init.text : { *(.init.text) } :text
  _einittext = .;
  .init.data : { *(.init.data) } :text
  . = ALIGN(32);
  __setup_start = .;
  .init.setup : { *(.init.setup) } :text
  __setup_end = .;
  __initcall_start = .;
  .initcall.init : { *(.initcall1.init) } :text
  __initcall_end = .;
  __xsm_initcall_start = .;
  .xsm_initcall.init : { *(.xsm_initcall.init) } :text
  __xsm_initcall_end = .;
  . = ALIGN((1 << 12));
  __init_end = .;
  __per_cpu_shift = 12;
  __per_cpu_start = .;
  .data.percpu : { *(.data.percpu) } :text
  __per_cpu_data_end = .;
  . = __per_cpu_start + (32 << 12);
  . = ALIGN((1 << 12));
  __per_cpu_end = .;
  __bss_start = .;
  .bss : {
 . = ALIGN(((1 << 12) << 2));
 *(.bss.stack_aligned)
 . = ALIGN((1 << 12));
 *(.bss.page_aligned)
 *(.bss)
 } :text
  _end = . ;
  /DISCARD/ : {
 *(.exit.text)
 *(.exit.data)
 *(.exitcall.exit)
 }
  .stab 0 : { *(.stab) }
  .stabstr 0 : { *(.stabstr) }
  .stab.excl 0 : { *(.stab.excl) }
  .stab.exclstr 0 : { *(.stab.exclstr) }
  .stab.index 0 : { *(.stab.index) }
  .stab.indexstr 0 : { *(.stab.indexstr) }
  .comment 0 : { *(.comment) }
}
