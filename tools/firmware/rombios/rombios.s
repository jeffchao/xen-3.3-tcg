! 1 
! 1 # 1 "_rombios_.c"
! 1 struct bios_info {
! 2     unsigned char com1_present:1;
!BCC_EOS
! 3     unsigned char com2_present:1;
!BCC_EOS
! 4     unsigned char hpet_present:1;
!BCC_EOS
! 5     unsigned long pci_min, pci_len;
!BCC_EOS
! 6     unsigned short xen_pfiob;
!BCC_EOS
! 7 };
!BCC_EOS
! 8 #asm
!BCC_ASM
.rom
.org 0x0000
use16 386
MACRO HALT
  ;; the HALT macro is called with the line number of the HALT call.
  ;; The line number is then sent to the 0x400, causing Bochs/Plex
  ;; to print a BX_PANIC message. This will normally halt the simulation
  ;; with a message such as "BIOS panic at rombios.c, line 4091".
  ;; However, users can choose to make panics non-fatal and continue.
  mov dx,#0x400
  mov ax,#?1
  out dx,ax
MEND
MACRO JMP_AP
  db 0xea
  dw ?2
  dw ?1
MEND
MACRO SET_INT_VECTOR
  mov ax, ?3
  mov ?1*4, ax
  mov ax, ?2
  mov ?1*4+2, ax
MEND
! 33 endasm
!BCC_ENDASM
! 34 typedef unsigned char Bit8u;
!BCC_EOS
! 35 typedef unsigned short Bit16u;
!BCC_EOS
! 36 typedef unsigned short bx_bool;
!BCC_EOS
! 37 typedef unsigned long Bit32u;
!BCC_EOS
! 38   void memsetb(seg,offset,value,count);
!BCC_EOS
! 39   void memcpyb(dseg,doffset,sseg,soffset,count);
!BCC_EOS
! 40   void memcpyd(dseg,doffset,sseg,soffset,count);
!BCC_EOS
! 41     void
! 42   memsetb(seg,offset,value,count)
! 43     Bit16u seg;
export	_memsetb
_memsetb:
!BCC_EOS
! 44     Bit16u offset;
!BCC_EOS
! 45     Bit16u value;
!BCC_EOS
! 46     Bit16u count;
!BCC_EOS
! 47   {
! 48 #asm
!BCC_ASM
_memsetb.count	set	8
_memsetb.seg	set	2
_memsetb.value	set	6
_memsetb.offset	set	4
    push bp
    mov bp, sp
      push ax
      push cx
      push es
      push di
      mov cx, 10[bp] ; count
      cmp cx, #0x00
      je memsetb_end
      mov ax, 4[bp] ; segment
      mov es, ax
      mov ax, 6[bp] ; offset
      mov di, ax
      mov al, 8[bp] ; value
      cld
      rep
       stosb
  memsetb_end:
      pop di
      pop es
      pop cx
      pop ax
    pop bp
! 72 endasm
!BCC_ENDASM
! 73   }
ret
! 74     void
! 75   memcpyb(dseg,doffset,sseg,soffset,count)
! 76     Bit16u dseg;
export	_memcpyb
_memcpyb:
!BCC_EOS
! 77     Bit16u doffset;
!BCC_EOS
! 78     Bit16u sseg;
!BCC_EOS
! 79     Bit16u soffset;
!BCC_EOS
! 80     Bit16u count;
!BCC_EOS
! 81   {
! 82 #asm
!BCC_ASM
_memcpyb.count	set	$A
_memcpyb.sseg	set	6
_memcpyb.soffset	set	8
_memcpyb.dseg	set	2
_memcpyb.doffset	set	4
    push bp
    mov bp, sp
      push ax
      push cx
      push es
      push di
      push ds
      push si
      mov cx, 12[bp] ; count
      cmp cx, #0x0000
      je memcpyb_end
      mov ax, 4[bp] ; dsegment
      mov es, ax
      mov ax, 6[bp] ; doffset
      mov di, ax
      mov ax, 8[bp] ; ssegment
      mov ds, ax
      mov ax, 10[bp] ; soffset
      mov si, ax
      cld
      rep
       movsb
  memcpyb_end:
      pop si
      pop ds
      pop di
      pop es
      pop cx
      pop ax
    pop bp
! 113 endasm
!BCC_ENDASM
! 114   }
ret
! 115   static Bit32u read_dword();
!BCC_EOS
! 116   static void write_dword();
!BCC_EOS
! 117     Bit32u
! 118   read_dword(seg, offset)
! 119     Bit16u seg;
export	_read_dword
_read_dword:
!BCC_EOS
! 120     Bit16u offset;
!BCC_EOS
! 121   {
! 122 #asm
!BCC_ASM
_read_dword.seg	set	2
_read_dword.offset	set	4
    push bp
    mov bp, sp
      push bx
      push ds
      mov ax, 4[bp] ; segment
      mov ds, ax
      mov bx, 6[bp] ; offset
      mov ax, [bx]
      inc bx
      inc bx
      mov dx, [bx]
      ;; ax = return value (word)
      ;; dx = return value (word)
      pop ds
      pop bx
    pop bp
! 139 endasm
!BCC_ENDASM
! 140   }
ret
! 141     void
! 142   write_dword(seg, offset, data)
! 143     Bit16u seg;
export	_write_dword
_write_dword:
!BCC_EOS
! 144     Bit16u offset;
!BCC_EOS
! 145     Bit32u data;
!BCC_EOS
! 146   {
! 147 #asm
!BCC_ASM
_write_dword.seg	set	2
_write_dword.data	set	6
_write_dword.offset	set	4
    push bp
    mov bp, sp
      push ax
      push bx
      push ds
      mov ax, 4[bp] ; segment
      mov ds, ax
      mov bx, 6[bp] ; offset
      mov ax, 8[bp] ; data word
      mov [bx], ax ; write data word
      inc bx
      inc bx
      mov ax, 10[bp] ; data word
      mov [bx], ax ; write data word
      pop ds
      pop bx
      pop ax
    pop bp
! 166 endasm
!BCC_ENDASM
! 167   }
ret
! 168 #asm
!BCC_ASM
_write_dword.seg	set	2
_write_dword.data	set	6
_write_dword.offset	set	4
  ;; and function
  landl:
  landul:
    SEG SS
      and ax,[di]
    SEG SS
      and bx,2[di]
    ret
  ;; add function
  laddl:
  laddul:
    SEG SS
      add ax,[di]
    SEG SS
      adc bx,2[di]
    ret
  ;; cmp function
  lcmpl:
  lcmpul:
    and eax, #0x0000FFFF
    shl ebx, #16
    add eax, ebx
    shr ebx, #16
    SEG SS
      cmp eax, dword ptr [di]
    ret
  ;; sub function
  lsubl:
  lsubul:
    SEG SS
    sub ax,[di]
    SEG SS
    sbb bx,2[di]
    ret
  ;; mul function
  lmull:
  lmulul:
    and eax, #0x0000FFFF
    shl ebx, #16
    add eax, ebx
    SEG SS
    mul eax, dword ptr [di]
    mov ebx, eax
    shr ebx, #16
    ret
  ;; dec function
  ldecl:
  ldecul:
    SEG SS
    dec dword ptr [bx]
    ret
  ;; or function
  lorl:
  lorul:
    SEG SS
    or ax,[di]
    SEG SS
    or bx,2[di]
    ret
  ;; inc function
  lincl:
  lincul:
    SEG SS
    inc dword ptr [bx]
    ret
  ;; tst function
  ltstl:
  ltstul:
    and eax, #0x0000FFFF
    shl ebx, #16
    add eax, ebx
    shr ebx, #16
    test eax, eax
    ret
  ;; sr function
  lsrul:
    mov cx,di
    jcxz lsr_exit
    and eax, #0x0000FFFF
    shl ebx, #16
    add eax, ebx
  lsr_loop:
    shr eax, #1
    loop lsr_loop
    mov ebx, eax
    shr ebx, #16
  lsr_exit:
    ret
  ;; sl function
  lsll:
  lslul:
    mov cx,di
    jcxz lsl_exit
    and eax, #0x0000FFFF
    shl ebx, #16
    add eax, ebx
  lsl_loop:
    shl eax, #1
    loop lsl_loop
    mov ebx, eax
    shr ebx, #16
  lsl_exit:
    ret
  idiv_:
    cwd
    idiv bx
    ret
  idiv_u:
    xor dx,dx
    div bx
    ret
  ldivul:
    and eax, #0x0000FFFF
    shl ebx, #16
    add eax, ebx
    xor edx, edx
    SEG SS
    mov bx, 2[di]
    shl ebx, #16
    SEG SS
    mov bx, [di]
    div ebx
    mov ebx, eax
    shr ebx, #16
    ret
! 294 endasm
!BCC_ENDASM
! 295 typedef struct {
! 296   unsigned char filler1[0x400];
!BCC_EOS
! 297   unsigned char filler2[0x6c];
!BCC_EOS
! 298   Bit16u ticks_low;
!BCC_EOS
! 299   Bit16u ticks_high;
!BCC_EOS
! 300   Bit8u midnight_flag;
!BCC_EOS
! 301   } bios_data_t;
!BCC_EOS
! 302   typedef struct {
! 303     Bit16u heads;
!BCC_EOS
! 304     Bit16u cylinders;
!BCC_EOS
! 305     Bit16u spt;
!BCC_EOS
! 306     } chs_t;
!BCC_EOS
! 307   typedef struct {
! 308     Bit16u iobase1;
!BCC_EOS
! 309     Bit16u iobase2;
!BCC_EOS
! 310     Bit8u prefix;
!BCC_EOS
! 311     Bit8u unused;
!BCC_EOS
! 312     Bit8u irq;
!BCC_EOS
! 313     Bit8u blkcount;
!BCC_EOS
! 314     Bit8u dma;
!BCC_EOS
! 315     Bit8u pio;
!BCC_EOS
! 316     Bit16u options;
!BCC_EOS
! 317     Bit16u reserved;
!BCC_EOS
! 318     Bit8u revision;
!BCC_EOS
! 319     Bit8u checksum;
!BCC_EOS
! 320     } dpte_t;
!BCC_EOS
! 321   typedef struct {
! 322     Bit8u iface;
!BCC_EOS
! 323     Bit16u iobase1;
!BCC_EOS
! 324     Bit16u iobase2;
!BCC_EOS
! 325     Bit8u irq;
!BCC_EOS
! 326     } ata_channel_t;
!BCC_EOS
! 327   typedef struct {
! 328     Bit8u type;
!BCC_EOS
! 329     Bit8u device;
!BCC_EOS
! 330     Bit8u removable;
!BCC_EOS
! 331     Bit8u lock;
!BCC_EOS
! 332     Bit8u mode;
!BCC_EOS
! 333     Bit16u blksize;
!BCC_EOS
! 334     Bit8u translation;
!BCC_EOS
! 335     chs_t lchs;
!BCC_EOS
! 336     chs_t pchs;
!BCC_EOS
! 337     Bit32u sectors;
!BCC_EOS
! 338     } ata_device_t;
!BCC_EOS
! 339   typedef struct {
! 340     ata_channel_t channels[4];
!BCC_EOS
! 341     ata_device_t devices[(4*2)];
!BCC_EOS
! 342     Bit8u hdcount, hdidmap[(4*2)];
!BCC_EOS
! 343     Bit8u cdcount, cdidmap[(4*2)];
!BCC_EOS
! 344     dpte_t dpte;
!BCC_EOS
! 345     Bit16u trsfsectors;
!BCC_EOS
! 346     Bit32u trsfbytes;
!BCC_EOS
! 347     } ata_t;
!BCC_EOS
! 348   typedef struct {
! 349     Bit8u active;
!BCC_EOS
! 350     Bit8u media;
!BCC_EOS
! 351     Bit8u emulated_drive;
!BCC_EOS
! 352     Bit8u controller_index;
!BCC_EOS
! 353     Bit16u device_spe
! 353 c;
!BCC_EOS
! 354     Bit32u ilba;
!BCC_EOS
! 355     Bit16u buffer_segment;
!BCC_EOS
! 356     Bit16u load_segment;
!BCC_EOS
! 357     Bit16u sector_count;
!BCC_EOS
! 358     chs_t vdevice;
!BCC_EOS
! 359     } cdemu_t;
!BCC_EOS
! 360 Bit32u TCGInterruptHandler( );
!BCC_EOS
! 361 void tcpa_acpi_init( );
!BCC_EOS
! 362 Bit32u tcpa_extend_acpi_log( );
!BCC_EOS
! 363 void tcpa_calling_int19h( );
!BCC_EOS
! 364 void tcpa_returned_int19h( );
!BCC_EOS
! 365 void tcpa_add_event_separators( );
!BCC_EOS
! 366 void tcpa_wake_event( );
!BCC_EOS
! 367 void tcpa_add_bootdevice( );
!BCC_EOS
! 368 void tcpa_start_option_rom_scan( );
!BCC_EOS
! 369 void tcpa_option_rom( );
!BCC_EOS
! 370 void tcpa_ipl( );
!BCC_EOS
! 371 void tcpa_measure_post( );
!BCC_EOS
! 372 Bit32u tcpa_initialize_tpm( );
!BCC_EOS
! 373 Bit32u get_s3_waking_vector( );
!BCC_EOS
! 374 void test_gateway();
!BCC_EOS
! 375 typedef struct {
! 376   Bit16u reg_ss;
!BCC_EOS
! 377   Bit16u reg_cs;
!BCC_EOS
! 378   Bit16u reg_ds;
!BCC_EOS
! 379   Bit16u reg_es;
!BCC_EOS
! 380   Bit16u esp_hi;
!BCC_EOS
! 381   Bit16u retaddr;
!BCC_EOS
! 382 } upcall_t;
!BCC_EOS
! 383   typedef struct {
! 384     unsigned char ebda_size;
!BCC_EOS
! 385     unsigned char cmos_shutdown_status;
!BCC_EOS
! 386     unsigned char filler1[0x3B];
!BCC_EOS
! 387     unsigned char fdpt0[0x10];
!BCC_EOS
! 388     unsigned char fdpt1[0x10];
!BCC_EOS
! 389     unsigned char filler2[0xC4];
!BCC_EOS
! 390     ata_t ata;
!BCC_EOS
! 391     cdemu_t cdemu;
!BCC_EOS
! 392     upcall_t upcall;
!BCC_EOS
! 393     } ebda_data_t;
!BCC_EOS
! 394   typedef struct {
! 395     Bit8u size;
!BCC_EOS
! 396     Bit8u reserved;
!BCC_EOS
! 397     Bit16u count;
!BCC_EOS
! 398     Bit16u offset;
!BCC_EOS
! 399     Bit16u segment;
!BCC_EOS
! 400     Bit32u lba1;
!BCC_EOS
! 401     Bit32u lba2;
!BCC_EOS
! 402     } int13ext_t;
!BCC_EOS
! 403   typedef struct {
! 404     Bit16u size;
!BCC_EOS
! 405     Bit16u infos;
!BCC_EOS
! 406     Bit32u cylinders;
!BCC_EOS
! 407     Bit32u heads;
!BCC_EOS
! 408     Bit32u spt;
!BCC_EOS
! 409     Bit32u sector_count1;
!BCC_EOS
! 410     Bit32u sector_count2;
!BCC_EOS
! 411     Bit16u blksize;
!BCC_EOS
! 412     Bit16u dpte_offset;
!BCC_EOS
! 413     Bit16u dpte_segment;
!BCC_EOS
! 414     Bit16u key;
!BCC_EOS
! 415     Bit8u dpi_length;
!BCC_EOS
! 416     Bit8u reserved1;
!BCC_EOS
! 417     Bit16u reserved2;
!BCC_EOS
! 418     Bit8u host_bus[4];
!BCC_EOS
! 419     Bit8u iface_type[8];
!BCC_EOS
! 420     Bit8u iface_path[8];
!BCC_EOS
! 421     Bit8u device_path[8];
!BCC_EOS
! 422     Bit8u reserved3;
!BCC_EOS
! 423     Bit8u checksum;
!BCC_EOS
! 424     } dpt_t;
!BCC_EOS
! 425 typedef struct {
! 426   union {
! 427     struct {
! 428       Bit16u di, si, bp, sp;
!BCC_EOS
! 429       Bit16u bx, dx, cx, ax;
!BCC_EOS
! 430       } r16;
!BCC_EOS
! 431     struct {
! 432       Bit16u filler[4];
!BCC_EOS
! 433       Bit8u bl, bh, dl, dh, cl, ch, al, ah;
!BCC_EOS
! 434       } r8;
!BCC_EOS
! 435     } u;
!BCC_EOS
! 436   } pusha_regs_t;
!BCC_EOS
! 437 typedef struct {
! 438  union {
! 439   struct {
! 440     Bit32u edi, esi, ebp, esp;
!BCC_EOS
! 441     Bit32u ebx, edx, ecx, eax;
!BCC_EOS
! 442     } r32;
!BCC_EOS
! 443   struct {
! 444     Bit16u di, filler1, si, filler2, bp, filler3, sp, filler4;
!BCC_EOS
! 445     Bit16u bx, filler5, dx
! 445 , filler6, cx, filler7, ax, filler8;
!BCC_EOS
! 446     } r16;
!BCC_EOS
! 447   struct {
! 448     Bit32u filler[4];
!BCC_EOS
! 449     Bit8u bl, bh;
!BCC_EOS
! 450     Bit16u filler1;
!BCC_EOS
! 451     Bit8u dl, dh;
!BCC_EOS
! 452     Bit16u filler2;
!BCC_EOS
! 453     Bit8u cl, ch;
!BCC_EOS
! 454     Bit16u filler3;
!BCC_EOS
! 455     Bit8u al, ah;
!BCC_EOS
! 456     Bit16u filler4;
!BCC_EOS
! 457     } r8;
!BCC_EOS
! 458   } u;
!BCC_EOS
! 459 } pushad_regs_t;
!BCC_EOS
! 460 typedef struct {
! 461   union {
! 462     struct {
! 463       Bit16u flags;
!BCC_EOS
! 464       } r16;
!BCC_EOS
! 465     struct {
! 466       Bit8u flagsl;
!BCC_EOS
! 467       Bit8u flagsh;
!BCC_EOS
! 468       } r8;
!BCC_EOS
! 469     } u;
!BCC_EOS
! 470   } flags_t;
!BCC_EOS
! 471 typedef struct {
! 472   Bit16u ip;
!BCC_EOS
! 473   Bit16u cs;
!BCC_EOS
! 474   flags_t flags;
!BCC_EOS
! 475   } iret_addr_t;
!BCC_EOS
! 476 static Bit8u inb();
!BCC_EOS
! 477 static Bit8u inb_cmos();
!BCC_EOS
! 478 static void outb();
!BCC_EOS
! 479 static void outb_cmos();
!BCC_EOS
! 480 static Bit16u inw();
!BCC_EOS
! 481 static void outw();
!BCC_EOS
! 482 static void init_rtc();
!BCC_EOS
! 483 static bx_bool rtc_updating();
!BCC_EOS
! 484 static Bit8u read_byte();
!BCC_EOS
! 485 static Bit16u read_word();
!BCC_EOS
! 486 static void write_byte();
!BCC_EOS
! 487 static void write_word();
!BCC_EOS
! 488 static void bios_printf();
!BCC_EOS
! 489 static void copy_e820_table();
!BCC_EOS
! 490 static Bit8u inhibit_mouse_int_and_events();
!BCC_EOS
! 491 static void enable_mouse_int_and_events();
!BCC_EOS
! 492 static Bit8u send_to_mouse_ctrl();
!BCC_EOS
! 493 static Bit8u get_mouse_data();
!BCC_EOS
! 494 static void set_kbd_command_byte();
!BCC_EOS
! 495 static void int09_function();
!BCC_EOS
! 496 static void int13_harddisk();
!BCC_EOS
! 497 static void int13_cdrom();
!BCC_EOS
! 498 static void int13_cdemu();
!BCC_EOS
! 499 static void int13_eltorito();
!BCC_EOS
! 500 static void int13_diskette_function();
!BCC_EOS
! 501 static void int14_function();
!BCC_EOS
! 502 static void int15_function();
!BCC_EOS
! 503 static void int16_function();
!BCC_EOS
! 504 static void int17_function();
!BCC_EOS
! 505 static void int18_function();
!BCC_EOS
! 506 static void int1a_function();
!BCC_EOS
! 507 static void int70_function();
!BCC_EOS
! 508 static void int74_function();
!BCC_EOS
! 509 static Bit16u get_CS();
!BCC_EOS
! 510 static Bit16u get_SS();
!BCC_EOS
! 511 static unsigned int enqueue_key();
!BCC_EOS
! 512 static unsigned int dequeue_key();
!BCC_EOS
! 513 static void get_hd_geometry();
!BCC_EOS
! 514 static void set_diskette_ret_status();
!BCC_EOS
! 515 static void set_diskette_current_cyl();
!BCC_EOS
! 516 static void determine_floppy_media();
!BCC_EOS
! 517 static bx_bool floppy_drive_exists();
!BCC_EOS
! 518 static bx_bool floppy_drive_recal();
!BCC_EOS
! 519 static bx_bool floppy_media_known();
!BCC_EOS
! 520 static bx_bool floppy_media_sense();
!BCC_EOS
! 521 static bx_bool set_enable_a20();
!BCC_EOS
! 522 static void debugger_on();
!BCC_EOS
! 523 static void debugger_off();
!BCC_EOS
! 524 static void keyboard_init();
!BCC_EOS
! 525 static void keyboard_panic();
!BCC_EOS
! 526 static void shutdown
! 526 _status_panic();
!BCC_EOS
! 527 static void nmi_handler_msg();
!BCC_EOS
! 528 static void print_bios_banner();
!BCC_EOS
! 529 static void print_boot_device();
!BCC_EOS
! 530 static void print_boot_failure();
!BCC_EOS
! 531 static void print_cdromboot_failure();
!BCC_EOS
! 532 void ata_init();
!BCC_EOS
! 533 void ata_detect();
!BCC_EOS
! 534 void ata_reset();
!BCC_EOS
! 535 Bit16u ata_cmd_non_data();
!BCC_EOS
! 536 Bit16u ata_cmd_data_in();
!BCC_EOS
! 537 Bit16u ata_cmd_data_out();
!BCC_EOS
! 538 Bit16u ata_cmd_packet();
!BCC_EOS
! 539 Bit16u atapi_get_sense();
!BCC_EOS
! 540 Bit16u atapi_is_ready();
!BCC_EOS
! 541 Bit16u atapi_is_cdrom();
!BCC_EOS
! 542 void cdemu_init();
!BCC_EOS
! 543 Bit8u cdemu_isactive();
!BCC_EOS
! 544 Bit8u cdemu_emulated_drive();
!BCC_EOS
! 545 Bit16u cdrom_boot();
!BCC_EOS
! 546 static char bios_cvs_version_string[] = "$Revision: 1.138 $";
.data
_bios_cvs_version_string:
.1:
.ascii	"$Revision: 1.138 $"
.byte	0
!BCC_EOS
! 547 static char bios_date_string[] = "$Date: 2005/05/07 15:55:26 $";
_bios_date_string:
.2:
.ascii	"$Date: 2005/05/07 15:55:26 $"
.byte	0
!BCC_EOS
! 548 static char CVSID[] = "$Id: rombios.c,v 1.138 2005/05/07 15:55:26 vruppert Exp $";
_CVSID:
.3:
.ascii	"$Id: rombios.c,v 1.138 2005/05/07 15:55:"
.ascii	"26 vruppert Exp $"
.byte	0
!BCC_EOS
! 549 static struct {
! 550   Bit16u normal;
!BCC_EOS
! 551   Bit16u shift;
!BCC_EOS
! 552   Bit16u control;
!BCC_EOS
! 553   Bit16u alt;
!BCC_EOS
! 554   Bit8u lock_flags;
!BCC_EOS
! 555   } scan_to_scanascii[0x58 + 1] = {
_scan_to_scanascii:
! 556       { 0, 0, 0, 0, 0 },
.word	0
.word	0
.word	0
.word	0
.byte	0
.blkb	1
! 557       { 0x011b, 0x011b, 0x011b, 0x0100, 0 },
.word	$11B
.word	$11B
.word	$11B
.word	$100
.byte	0
.blkb	1
! 558       { 0x0231, 0x0221, 0, 0x7800, 0 },
.word	$231
.word	$221
.word	0
.word	$7800
.byte	0
.blkb	1
! 559       { 0x0332, 0x0340, 0x0300, 0x7900, 0 },
.word	$332
.word	$340
.word	$300
.word	$7900
.byte	0
.blkb	1
! 560       { 0x0433, 0x0423, 0, 0x7a00, 0 },
.word	$433
.word	$423
.word	0
.word	$7A00
.byte	0
.blkb	1
! 561       { 0x0534, 0x0524, 0, 0x7b00, 0 },
.word	$534
.word	$524
.word	0
.word	$7B00
.byte	0
.blkb	1
! 562       { 0x0635, 0x0625, 0, 0x7c00, 0 },
.word	$635
.word	$625
.word	0
.word	$7C00
.byte	0
.blkb	1
! 563       { 0x0736, 0x075e, 0x071e, 0x7d00, 0 },
.word	$736
.word	$75E
.word	$71E
.word	$7D00
.byte	0
.blkb	1
! 564       { 0x0837, 0x0826, 0, 0x7e00, 0 },
.word	$837
.word	$826
.word	0
.word	$7E00
.byte	0
.blkb	1
! 565       { 0x0938, 0x092a, 0, 0x7f00, 0 },
.word	$938
.word	$92A
.word	0
.word	$7F00
.byte	0
.blkb	1
! 566       { 0x0a39, 0x0a28, 0, 0x8000, 0 },
.word	$A39
.word	$A28
.word	0
.word	$8000
.byte	0
.blkb	1
! 567       { 0x0b30, 0x0b29, 0, 0x8100, 0 },
.word	$B30
.word	$B29
.word	0
.word	$8100
.byte	0
.blkb	1
! 568       { 0x0c2d, 0x0c5f, 0x0c1f, 0x8200, 0 },
.word	$C2D
.word	$C5F
.word	$C1F
.word	$8200
.byte	0
.blkb	1
! 569       { 0x0d3d, 0x0d2b, 0, 0x8300, 0 },
.word	$D3D
.word	$D2B
.word	0
.word	$8300
.byte	0
.blkb	1
! 570       { 0x0e08, 0x0e08, 0x0e7f, 0, 0 },
.word	$E08
.word	$E08
.word	$E7F
.word	0
.byte	0
.blkb	1
! 571       { 0x0f09, 0x0f00, 0, 0, 0 },
.word	$F09
.word	$F00
.word	0
.word	0
.byte	0
.blkb	1
! 572       { 0x1071, 0x1051, 0x1011, 0x1000, 0x40 },
.word	$1071
.word	$1051
.word	$1011
.word	$1000
.byte	$40
.blkb	1
! 573       { 0x1177, 0x1157, 0x1117, 0x1100, 0x40 },
.word	$1177
.word	$1157
.word	$1117
.word	$1100
.byte	$40
.blkb	1
! 574       { 0x1265, 0x1245, 0x1205, 0x1200, 0x40 },
.word	$1265
.word	$1245
.word	$1205
.word	$1200
.byte	$40
.blkb	1
! 575       { 0x1372, 0x1352, 0x1312, 0x1300, 0x40 },
.word	$1372
.word	$1352
.word	$1312
.word	$1300
.byte	$40
.blkb	1
! 576       { 0x1474, 0x1454, 0x1414, 0x1400, 0x40 },
.word	$1474
.word	$1454
.word	$1414
.word	$1400
.byte	$40
.blkb	1
! 577       { 0x1579, 0x1559, 0x1519, 0x1500, 0x40 },
.word	$1579
.word	$1559
.word	$1519
.word	$1500
.byte	$40
.blkb	1
! 578       { 0x1675, 0x1655, 0x1615, 0x1600, 0x40 },
.word	$1675
.word	$1655
.word	$1615
.word	$1600
.byte	$40
.blkb	1
! 579       { 0x1769, 0x1749, 0x1709, 0x1700, 0x40 },
.word	$1769
.word	$1749
.word	$1709
.word	$1700
.byte	$40
.blkb	1
! 580       { 0x186f, 0x184f, 0x180f, 0x1800, 0x40 },
.word	$186F
.word	$184F
.word	$180F
.word	$1800
.byte	$40
.blkb	1
! 581       { 0x1970, 0x1950, 0x1910, 0x1900, 0x40 },
.word	$1970
.word	$1950
.word	$1910
.word	$1900
.byte	$40
.blkb	1
! 582       { 0x1a5b, 0x1a7b, 0x1a1b, 0, 0 },
.word	$1A5B
.word	$1A7B
.word	$1A1B
.word	0
.byte	0
.blkb	1
! 583       { 0x1b5d, 0x1b7d, 0
.word	$1B5D
.word	$1B7D
! 583 x1b1d, 0, 0 },
.word	$1B1D
.word	0
.byte	0
.blkb	1
! 584       { 0x1c0d, 0x1c0d, 0x1c0a, 0, 0 },
.word	$1C0D
.word	$1C0D
.word	$1C0A
.word	0
.byte	0
.blkb	1
! 585       { 0, 0, 0, 0, 0 },
.word	0
.word	0
.word	0
.word	0
.byte	0
.blkb	1
! 586       { 0x1e61, 0x1e41, 0x1e01, 0x1e00, 0x40 },
.word	$1E61
.word	$1E41
.word	$1E01
.word	$1E00
.byte	$40
.blkb	1
! 587       { 0x1f73, 0x1f53, 0x1f13, 0x1f00, 0x40 },
.word	$1F73
.word	$1F53
.word	$1F13
.word	$1F00
.byte	$40
.blkb	1
! 588       { 0x2064, 0x2044, 0x2004, 0x2000, 0x40 },
.word	$2064
.word	$2044
.word	$2004
.word	$2000
.byte	$40
.blkb	1
! 589       { 0x2166, 0x2146, 0x2106, 0x2100, 0x40 },
.word	$2166
.word	$2146
.word	$2106
.word	$2100
.byte	$40
.blkb	1
! 590       { 0x2267, 0x2247, 0x2207, 0x2200, 0x40 },
.word	$2267
.word	$2247
.word	$2207
.word	$2200
.byte	$40
.blkb	1
! 591       { 0x2368, 0x2348, 0x2308, 0x2300, 0x40 },
.word	$2368
.word	$2348
.word	$2308
.word	$2300
.byte	$40
.blkb	1
! 592       { 0x246a, 0x244a, 0x240a, 0x2400, 0x40 },
.word	$246A
.word	$244A
.word	$240A
.word	$2400
.byte	$40
.blkb	1
! 593       { 0x256b, 0x254b, 0x250b, 0x2500, 0x40 },
.word	$256B
.word	$254B
.word	$250B
.word	$2500
.byte	$40
.blkb	1
! 594       { 0x266c, 0x264c, 0x260c, 0x2600, 0x40 },
.word	$266C
.word	$264C
.word	$260C
.word	$2600
.byte	$40
.blkb	1
! 595       { 0x273b, 0x273a, 0, 0, 0 },
.word	$273B
.word	$273A
.word	0
.word	0
.byte	0
.blkb	1
! 596       { 0x2827, 0x2822, 0, 0, 0 },
.word	$2827
.word	$2822
.word	0
.word	0
.byte	0
.blkb	1
! 597       { 0x2960, 0x297e, 0, 0, 0 },
.word	$2960
.word	$297E
.word	0
.word	0
.byte	0
.blkb	1
! 598       { 0, 0, 0, 0, 0 },
.word	0
.word	0
.word	0
.word	0
.byte	0
.blkb	1
! 599       { 0x2b5c, 0x2b7c, 0x2b1c, 0, 0 },
.word	$2B5C
.word	$2B7C
.word	$2B1C
.word	0
.byte	0
.blkb	1
! 600       { 0x2c7a, 0x2c5a, 0x2c1a, 0x2c00, 0x40 },
.word	$2C7A
.word	$2C5A
.word	$2C1A
.word	$2C00
.byte	$40
.blkb	1
! 601       { 0x2d78, 0x2d58, 0x2d18, 0x2d00, 0x40 },
.word	$2D78
.word	$2D58
.word	$2D18
.word	$2D00
.byte	$40
.blkb	1
! 602       { 0x2e63, 0x2e43, 0x2e03, 0x2e00, 0x40 },
.word	$2E63
.word	$2E43
.word	$2E03
.word	$2E00
.byte	$40
.blkb	1
! 603       { 0x2f76, 0x2f56, 0x2f16, 0x2f00, 0x40 },
.word	$2F76
.word	$2F56
.word	$2F16
.word	$2F00
.byte	$40
.blkb	1
! 604       { 0x3062, 0x3042, 0x3002, 0x3000, 0x40 },
.word	$3062
.word	$3042
.word	$3002
.word	$3000
.byte	$40
.blkb	1
! 605       { 0x316e, 0x314e, 0x310e, 0x3100, 0x40 },
.word	$316E
.word	$314E
.word	$310E
.word	$3100
.byte	$40
.blkb	1
! 606       { 0x326d, 0x324d, 0x320d, 0x3200, 0x40 },
.word	$326D
.word	$324D
.word	$320D
.word	$3200
.byte	$40
.blkb	1
! 607       { 0x332c, 0x333c, 0, 0, 0 },
.word	$332C
.word	$333C
.word	0
.word	0
.byte	0
.blkb	1
! 608       { 0x342e, 0x343e, 0, 0, 0 },
.word	$342E
.word	$343E
.word	0
.word	0
.byte	0
.blkb	1
! 609       { 0x352f, 0x353f, 0, 0, 0 },
.word	$352F
.word	$353F
.word	0
.word	0
.byte	0
.blkb	1
! 610       { 0, 0, 0, 0, 0 },
.word	0
.word	0
.word	0
.word	0
.byte	0
.blkb	1
! 611       { 0x372a, 0x372a, 0, 0, 0 },
.word	$372A
.word	$372A
.word	0
.word	0
.byte	0
.blkb	1
! 612       { 0, 0, 0, 0, 0 },
.word	0
.word	0
.word	0
.word	0
.byte	0
.blkb	1
! 613       { 0x3920, 0x3920, 0x3920, 0x3920, 0 },
.word	$3920
.word	$3920
.word	$3920
.word	$3920
.byte	0
.blkb	1
! 614       { 0, 0, 0, 0, 0 },
.word	0
.word	0
.word	0
.word	0
.byte	0
.blkb	1
! 615       { 0x3b00, 0x5400, 0x5e00, 0x6800, 0 },
.word	$3B00
.word	$5400
.word	$5E00
.word	$6800
.byte	0
.blkb	1
! 616       { 0x3c00, 0x5500, 0x5f00, 0x6900, 0 },
.word	$3C00
.word	$5500
.word	$5F00
.word	$6900
.byte	0
.blkb	1
! 617       { 0x3d00, 0x5600, 0x6000, 0x6a00, 0 },
.word	$3D00
.word	$5600
.word	$6000
.word	$6A00
.byte	0
.blkb	1
! 618       { 0x3e00, 0x5700, 0x6100, 0x6b00, 0 },
.word	$3E00
.word	$5700
.word	$6100
.word	$6B00
.byte	0
.blkb	1
! 619       { 0x3f00, 0x5800, 0x6200, 0x6c00, 0 },
.word	$3F00
.word	$5800
.word	$6200
.word	$6C00
.byte	0
.blkb	1
! 620       { 0x4000, 0x5900, 0x6300, 0x6d00, 0 },
.word	$4000
.word	$5900
.word	$6300
.word	$6D00
.byte	0
.blkb	1
! 621       { 0x4100, 0x5a00, 0x6400, 0x6e00, 0 },
.word	$4100
.word	$5A00
.word	$6400
.word	$6E00
.byte	0
.blkb	1
! 622       { 0x4200, 0x5b00, 0x6500, 0x6f00, 0 },
.word	$4200
.word	$5B00
.word	$6500
.word	$6F00
.byte	0
.blkb	1
! 623       { 0x4300, 0x5c00, 0x6600, 0x7000, 0 },
.word	$4300
.word	$5C00
.word	$6600
.word	$7000
.byte	0
.blkb	1
! 624       { 0x4400, 0x5d00, 0x6700, 0x7100, 0 },
.word	$4400
.word	$5D00
.word	$6700
.word	$7100
.byte	0
.blkb	1
! 625       { 0, 0, 0, 0, 0 },
.word	0
.word	0
.word	0
.word	0
.byte	0
.blkb	1
! 626       { 0, 0, 0, 0, 0 },
.word	0
.word	0
.word	0
.word	0
.byte	0
.blkb	1
! 627       { 0x4700, 0x4737, 0x7700, 0, 0x20 },
.word	$4700
.word	$4737
.word	$7700
.word	0
.byte	$20
.blkb	1
! 628       { 0x4800, 0x4838, 0, 0, 0x20 },
.word	$4800
.word	$4838
.word	0
.word	0
.byte	$20
.blkb	1
! 629       { 0x4900, 0x4939, 0x8400, 0, 0x20 },
.word	$4900
.word	$4939
.word	$8400
.word	0
.byte	$20
.blkb	1
! 630       { 0x4a2d, 0x4a2d, 0, 0, 0 },
.word	$4A2D
.word	$4A2D
.word	0
.word	0
.byte	0
.blkb	1
! 631       { 0x4b00, 0x4b34, 0x7300, 0, 0x20 },
.word	$4B00
.word	$4B34
.word	$7300
.word	0
.byte	$20
.blkb	1
! 632       { 0x4c00, 0x4c35, 0, 0, 0x20 },
.word	$4C00
.word	$4C35
.word	0
.word	0
.byte	$20
.blkb	1
! 633       { 0x4d00, 0x4d36, 0x7400
.word	$4D00
.word	$4D36
! 633 , 0, 0x20 },
.word	$7400
.word	0
.byte	$20
.blkb	1
! 634       { 0x4e2b, 0x4e2b, 0, 0, 0 },
.word	$4E2B
.word	$4E2B
.word	0
.word	0
.byte	0
.blkb	1
! 635       { 0x4f00, 0x4f31, 0x7500, 0, 0x20 },
.word	$4F00
.word	$4F31
.word	$7500
.word	0
.byte	$20
.blkb	1
! 636       { 0x5000, 0x5032, 0, 0, 0x20 },
.word	$5000
.word	$5032
.word	0
.word	0
.byte	$20
.blkb	1
! 637       { 0x5100, 0x5133, 0x7600, 0, 0x20 },
.word	$5100
.word	$5133
.word	$7600
.word	0
.byte	$20
.blkb	1
! 638       { 0x5200, 0x5230, 0, 0, 0x20 },
.word	$5200
.word	$5230
.word	0
.word	0
.byte	$20
.blkb	1
! 639       { 0x5300, 0x532e, 0, 0, 0x20 },
.word	$5300
.word	$532E
.word	0
.word	0
.byte	$20
.blkb	1
! 640       { 0, 0, 0, 0, 0 },
.word	0
.word	0
.word	0
.word	0
.byte	0
.blkb	1
! 641       { 0, 0, 0, 0, 0 },
.word	0
.word	0
.word	0
.word	0
.byte	0
.blkb	1
! 642       { 0, 0, 0, 0, 0 },
.word	0
.word	0
.word	0
.word	0
.byte	0
.blkb	1
! 643       { 0x8500, 0x8700, 0x8900, 0x8b00, 0 },
.word	$8500
.word	$8700
.word	$8900
.word	$8B00
.byte	0
.blkb	1
! 644       { 0x8600, 0x8800, 0x8a00, 0x8c00, 0 },
.word	$8600
.word	$8800
.word	$8A00
.word	$8C00
.byte	0
.blkb	1
! 645       };
!BCC_EOS
! 646   Bit8u
! 647 inb(port)
! 648   Bit16u port;
.text
export	_inb
_inb:
!BCC_EOS
! 649 {
! 650 #asm
!BCC_ASM
_inb.port	set	2
  push bp
  mov bp, sp
    push dx
    mov dx, 4[bp]
    in al, dx
    pop dx
  pop bp
! 658 endasm
!BCC_ENDASM
! 659 }
ret
! 660   Bit16u
! 661 inw(port)
! 662   Bit16u port;
export	_inw
_inw:
!BCC_EOS
! 663 {
! 664 #asm
!BCC_ASM
_inw.port	set	2
  push bp
  mov bp, sp
    push dx
    mov dx, 4[bp]
    in ax, dx
    pop dx
  pop bp
! 672 endasm
!BCC_ENDASM
! 673 }
ret
! 674   void
! 675 outb(port, val)
! 676   Bit16u port;
export	_outb
_outb:
!BCC_EOS
! 677   Bit8u val;
!BCC_EOS
! 678 {
! 679 #asm
!BCC_ASM
_outb.val	set	4
_outb.port	set	2
  push bp
  mov bp, sp
    push ax
    push dx
    mov dx, 4[bp]
    mov al, 6[bp]
    out dx, al
    pop dx
    pop ax
  pop bp
! 690 endasm
!BCC_ENDASM
! 691 }
ret
! 692   void
! 693 outw(port, val)
! 694   Bit16u port;
export	_outw
_outw:
!BCC_EOS
! 695   Bit16u val;
!BCC_EOS
! 696 {
! 697 #asm
!BCC_ASM
_outw.val	set	4
_outw.port	set	2
  push bp
  mov bp, sp
    push ax
    push dx
    mov dx, 4[bp]
    mov ax, 6[bp]
    out dx, ax
    pop dx
    pop ax
  pop bp
! 708 endasm
!BCC_ENDASM
! 709 }
ret
! 710   void
! 711 outb_cmos(cmos_reg, val)
! 712   Bit8u cmos_reg;
export	_outb_cmos
_outb_cmos:
!BCC_EOS
! 713   Bit8u val;
!BCC_EOS
! 714 {
! 715 #asm
!BCC_ASM
_outb_cmos.cmos_reg	set	2
_outb_cmos.val	set	4
  push bp
  mov bp, sp
    mov al, 4[bp] ;; cmos_reg
    out 0x70, al
    mov al, 6[bp] ;; val
    out 0x71, al
  pop bp
! 723 endasm
!BCC_ENDASM
! 724 }
ret
! 725   Bit8u
! 726 inb_cmos(cmos_reg)
! 727   Bit8u cmos_reg;
export	_inb_cmos
_inb_cmos:
!BCC_EOS
! 728 {
! 729 #asm
!BCC_ASM
_inb_cmos.cmos_reg	set	2
  push bp
  mov bp, sp
    mov al, 4[bp] ;; cmos_reg
    out 0x70, al
    in al, 0x71
  pop bp
! 736 endasm
!BCC_ENDASM
! 737 }
ret
! 738   void
! 739 init_rtc()
! 740 {
export	_init_rtc
_init_rtc:
! 741   outb_cmos(0x0a, 0x26);
push	bp
mov	bp,sp
! Debug: list int = const $26 (used reg = )
mov	ax,*$26
push	ax
! Debug: list int = const $A (used reg = )
mov	ax,*$A
push	ax
! Debug: func () void = outb_cmos+0 (used reg = )
call	_outb_cmos
mov	sp,bp
!BCC_EOS
! 742   outb_cmos(0x0b, 0x02);
! Debug: list int = const 2 (used reg = )
mov	ax,*2
push	ax
! Debug: list int = const $B (used reg = )
mov	ax,*$B
push	ax
! Debug: func () void = outb_cmos+0 (used reg = )
call	_outb_cmos
mov	sp,bp
!BCC_EOS
! 743   inb_cmos(0x0c);
! Debug: list int = const $C (used reg = )
mov	ax,*$C
push	ax
! Debug: func () unsigned char = inb_cmos+0 (used reg = )
call	_inb_cmos
mov	sp,bp
!BCC_EOS
! 744   inb_cmos(0x0d);
! Debug: list int = const $D (used reg = )
mov	ax,*$D
push	ax
! Debug: func () unsigned char = inb_cmos+0 (used reg = )
call	_inb_cmos
mov	sp,bp
!BCC_EOS
! 745 }
pop	bp
ret
! 746   bx_bool
! 747 rtc_updating()
! 748 {
export	_rtc_updating
_rtc_updating:
! 749   Bit16u count;
!BCC_EOS
! 750   count = 25000;
push	bp
mov	bp,sp
dec	sp
dec	sp
! Debug: eq int = const $61A8 to unsigned short count = [S+4-4] (used reg = )
mov	ax,#$61A8
mov	-2[bp],ax
!BCC_EOS
! 751   while (--count != 0) {
jmp .5
.6:
! 752     if ( (inb_cmos(0x0a) & 0x80) == 0 )
! Debug: list int = const $A (used reg = )
mov	ax,*$A
push	ax
! Debug: func () unsigned char = inb_cmos+0 (used reg = )
call	_inb_cmos
inc	sp
inc	sp
! Debug: and int = const $80 to unsigned char = al+0 (used reg = )
and	al,#$80
! Debug: logeq int = const 0 to unsigned char = al+0 (used reg = )
test	al,al
jne 	.7
.8:
! 753       return(0);
xor	ax,ax
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 754     }
.7:
! 755   return(1);
.5:
! Debug: predec unsigned short count = [S+4-4] (used reg = )
mov	ax,-2[bp]
dec	ax
mov	-2[bp],ax
! Debug: ne int = const 0 to unsigned short = ax+0 (used reg = )
test	ax,ax
jne	.6
.9:
.4:
mov	ax,*1
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 756 }
! 757   Bit8u
! 758 read_byte(seg, offset)
! 759   Bit16u seg;
export	_read_byte
_read_byte:
!BCC_EOS
! 760   Bit16u offset;
!BCC_EOS
! 761 {
! 762 #asm
!BCC_ASM
_read_byte.seg	set	2
_read_byte.offset	set	4
  push bp
  mov bp, sp
    push bx
    push ds
    mov ax, 4[bp] ; segment
    mov ds, ax
    mov bx, 6[bp] ; offset
    mov al, [bx]
    ;; al = return value (byte)
    pop ds
    pop bx
  pop bp
! 775 endasm
!BCC_ENDASM
! 776 }
ret
! 777   Bit16u
! 778 read_word(seg, offset)
! 779   Bit16u seg;
export	_read_word
_read_word:
!BCC_EOS
! 780   Bit
! 780 16u offset;
!BCC_EOS
! 781 {
! 782 #asm
!BCC_ASM
_read_word.seg	set	2
_read_word.offset	set	4
  push bp
  mov bp, sp
    push bx
    push ds
    mov ax, 4[bp] ; segment
    mov ds, ax
    mov bx, 6[bp] ; offset
    mov ax, [bx]
    ;; ax = return value (word)
    pop ds
    pop bx
  pop bp
! 795 endasm
!BCC_ENDASM
! 796 }
ret
! 797   void
! 798 write_byte(seg, offset, data)
! 799   Bit16u seg;
export	_write_byte
_write_byte:
!BCC_EOS
! 800   Bit16u offset;
!BCC_EOS
! 801   Bit8u data;
!BCC_EOS
! 802 {
! 803 #asm
!BCC_ASM
_write_byte.seg	set	2
_write_byte.data	set	6
_write_byte.offset	set	4
  push bp
  mov bp, sp
    push ax
    push bx
    push ds
    mov ax, 4[bp] ; segment
    mov ds, ax
    mov bx, 6[bp] ; offset
    mov al, 8[bp] ; data byte
    mov [bx], al ; write data byte
    pop ds
    pop bx
    pop ax
  pop bp
! 818 endasm
!BCC_ENDASM
! 819 }
ret
! 820   void
! 821 write_word(seg, offset, data)
! 822   Bit16u seg;
export	_write_word
_write_word:
!BCC_EOS
! 823   Bit16u offset;
!BCC_EOS
! 824   Bit16u data;
!BCC_EOS
! 825 {
! 826 #asm
!BCC_ASM
_write_word.seg	set	2
_write_word.data	set	6
_write_word.offset	set	4
  push bp
  mov bp, sp
    push ax
    push bx
    push ds
    mov ax, 4[bp] ; segment
    mov ds, ax
    mov bx, 6[bp] ; offset
    mov ax, 8[bp] ; data word
    mov [bx], ax ; write data word
    pop ds
    pop bx
    pop ax
  pop bp
! 841 endasm
!BCC_ENDASM
! 842 }
ret
! 843   Bit16u
! 844 get_CS()
! 845 {
export	_get_CS
_get_CS:
! 846 #asm
!BCC_ASM
  mov ax, cs
! 848 endasm
!BCC_ENDASM
! 849 }
ret
! 850   Bit16u
! 851 get_SS()
! 852 {
export	_get_SS
_get_SS:
! 853 #asm
!BCC_ASM
  mov ax, ss
! 855 endasm
!BCC_ENDASM
! 856 }
ret
! 857 void
! 858 copy_e820_table()
! 859 {
export	_copy_e820_table
_copy_e820_table:
! 860   Bit8u nr_entries = read_byte(0x9000, 0x1e8);
push	bp
mov	bp,sp
dec	sp
! Debug: list int = const $1E8 (used reg = )
mov	ax,#$1E8
push	ax
! Debug: list unsigned int = const $9000 (used reg = )
mov	ax,#$9000
push	ax
! Debug: func () unsigned char = read_byte+0 (used reg = )
call	_read_byte
add	sp,*4
! Debug: eq unsigned char = al+0 to unsigned char nr_entries = [S+3-3] (used reg = )
mov	-1[bp],al
!BCC_EOS
! 861   Bit32u base_mem;
!BCC_EOS
! 862   if (nr_entries > 32)
add	sp,*-5
! Debug: gt int = const $20 to unsigned char nr_entries = [S+8-3] (used reg = )
mov	al,-1[bp]
cmp	al,*$20
jbe 	.A
.B:
! 863    nr_entries = 32;
! Debug: eq int = const $20 to unsigned char nr_entries = [S+8-3] (used reg = )
mov	al,*$20
mov	-1[bp],al
!BCC_EOS
! 864   write_word(0xe000, 0x8, nr_entries);
.A:
! Debug: list unsigned char nr_entries = [S+8-3] (used reg = )
mov	al,-1[bp]
xor	ah,ah
push	ax
! Debug: list int = const 8 (used reg = )
mov	ax,*8
push	ax
! Debug: list unsigned int = const $E000 (used reg = )
mov	ax,#$E000
push	ax
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
add	sp,*6
!BCC_EOS
! 865   memcpyb(0xe000, 0x10, 0x9000, 0x2d0, nr_entries * 0x14);
! Debug: mul int = const $14 to unsigned char nr_entries = [S+8-3] (used reg = )
mov	al,-1[bp]
xor	ah,ah
mov	cx,*$14
imul	cx
! Debug: list unsigned int = ax+0 (used reg = )
push	ax
! Debug: list int = const $2D0 (used reg = )
mov	ax,#$2D0
push	ax
! Debug: list unsigned int = const $9000 (used reg = )
mov	ax,#$9000
push	ax
! Debug: list int = const $10 (used reg = )
mov	ax,*$10
push	ax
! Debug: list unsigned int = const $E000 (used reg = )
mov	ax,#$E000
push	ax
! Debug: func () void = memcpyb+0 (used reg = )
call	_memcpyb
add	sp,*$A
!BCC_EOS
! 866   base_mem = read_dword(0x9000, 0x2d0 + 8);
! Debug: list int = const $2D8 (used reg = )
mov	ax,#$2D8
push	ax
! Debug: list unsigned int = const $9000 (used reg = )
mov	ax,#$9000
push	ax
! Debug: func () unsigned long = read_dword+0 (used reg = )
call	_read_dword
mov	bx,dx
add	sp,*4
! Debug: eq unsigned long = bx+0 to unsigned long base_mem = [S+8-8] (used reg = )
mov	-6[bp],ax
mov	-4[bp],bx
!BCC_EOS
! 867   write_word(0x40, 0x13, base_mem >> 10);
! Debug: sr int = const $A to unsigned long base_mem = [S+8-8] (used reg = )
mov	ax,-6[bp]
mov	bx,-4[bp]
mov	al,ah
mov	ah,bl
mov	bl,bh
sub	bh,bh
mov	di,*2
call	lsrul
! Debug: list unsigned long = bx+0 (used reg = )
push	bx
push	ax
! Debug: list int = const $13 (used reg = )
mov	ax,*$13
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
add	sp,*8
!BCC_EOS
! 868 }
mov	sp,bp
pop	bp
ret
! 869 void
! Register BX used in function copy_e820_table
! 870 set_rom_write_access(action)
! 871   Bit16u action;
export	_set_rom_write_access
_set_rom_write_access:
!BCC_EOS
! 872 {
! 873     Bit16u off = (Bit16u)&((struct bios_info *)0)->xen_pfiob;
push	bp
mov	bp,sp
dec	sp
dec	sp
! Debug: eq unsigned short = const $C to unsigned short off = [S+4-4] (used reg = )
mov	ax,*$C
mov	-2[bp],ax
!BCC_EOS
! 874 #asm
!BCC_EOS
!BCC_ASM
_set_rom_write_access.action	set	6
.set_rom_write_access.action	set	4
_set_rom_write_access.off	set	0
.set_rom_write_access.off	set	-2
    mov si,.set_rom_write_access.off[bp]
    push ds
    mov ax,#(0x000EA000 >> 4)
    mov ds,ax
    mov dx,[si]
    pop ds
    mov ax,.set_rom_write_access.action[bp]
    out dx,al
! 883 endasm
!BCC_ENDASM
!BCC_EOS
! 884 }
mov	sp,bp
pop	bp
ret
! 885 void enable_rom_write_access()
! 886 {
export	_enable_rom_write_access
_enable_rom_write_access:
! 887     set_rom_write_access(0);
push	bp
mov	bp,sp
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: func () void = set_rom_write_access+0 (used reg = )
call	_set_rom_write_access
mov	sp,bp
!BCC_EOS
! 888 }
pop	bp
ret
! 889 void disable_rom_write_access()
! 890 {
export	_disable_rom_write_access
_disable_rom_write_access:
! 891     set_rom_write_access(1);
push	bp
mov	bp,sp
! Debug: list int = const 1 (used reg = )
mov	ax,*1
push	ax
! Debug: func () void = set_rom_write_access+0 (used reg = )
call	_set_rom_write_access
mov	sp,bp
!BCC_EOS
! 892 }
pop	bp
ret
! 893   void
! 894 wrch(c)
! 895   Bit8u c;
export	_wrch
_wrch:
!BCC_EOS
! 896 {
! 897 #asm
!BCC_ASM
_wrch.c	set	2
  push bp
  mov bp, sp
  push bx
  mov ah, #0x0e
  mov al, 4[bp]
  xor bx,bx
  int #0x10
  pop bx
  pop bp
! 907 endasm
!BCC_ENDASM
! 908 }
ret
! 909   void
! 910 send(action, c)
! 911   Bit16u action;
export	_send
_send:
!BCC_EOS
! 912   Bit8u c;
!BCC_EOS
! 913 {
! 914   outb(0xE9, c);
push	bp
mov	bp,sp
! Debug: list unsigned char c = [S+2+4] (used reg = )
mov	al,6[bp]
xor	ah,ah
push	ax
! Debug: list int = const $E9 (used reg = )
mov	ax,#$E9
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
mov	sp,bp
!BCC_EOS
! 915   if (action & 8) outb(0x403, c);
! Debug: and int = const 8 to unsigned short action = [S+2+2] (used reg = )
mov	al,4[bp]
and	al,*8
test	al,al
je  	.C
.D:
! Debug: list unsigned char c = [S+2+4] (used reg = )
mov	al,6[bp]
xor	ah,ah
push	ax
! Debug: list int = const $403 (used reg = )
mov	ax,#$403
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
mov	sp,bp
!BCC_EOS
! 916   if (action & 4) outb(0x402, c);
.C:
! Debug: and int = const 4 to unsigned short action = [S+2+2] (used reg = )
mov	al,4[bp]
and	al,*4
test	al,al
je  	.E
.F:
! Debug: list unsigned char c = [S+2+4] (used reg = )
mov	al,6[bp]
xor	ah,ah
push	ax
! Debug: list int = const $402 (used reg = )
mov	ax,#$402
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
mov	sp,bp
!BCC_EOS
! 917   if (
.E:
! 917 action & 2) {
! Debug: and int = const 2 to unsigned short action = [S+2+2] (used reg = )
mov	al,4[bp]
and	al,*2
test	al,al
je  	.10
.11:
! 918     if (c == '\n') wrch('\r');
! Debug: logeq int = const $A to unsigned char c = [S+2+4] (used reg = )
mov	al,6[bp]
cmp	al,*$A
jne 	.12
.13:
! Debug: list int = const $D (used reg = )
mov	ax,*$D
push	ax
! Debug: func () void = wrch+0 (used reg = )
call	_wrch
mov	sp,bp
!BCC_EOS
! 919     wrch(c);
.12:
! Debug: list unsigned char c = [S+2+4] (used reg = )
mov	al,6[bp]
xor	ah,ah
push	ax
! Debug: func () void = wrch+0 (used reg = )
call	_wrch
mov	sp,bp
!BCC_EOS
! 920   }
! 921 }
.10:
pop	bp
ret
! 922   void
! 923 put_int(action, val, width, neg)
! 924   Bit16u action;
export	_put_int
_put_int:
!BCC_EOS
! 925   short val, width;
!BCC_EOS
! 926   bx_bool neg;
!BCC_EOS
! 927 {
! 928   short nval = val / 10;
push	bp
mov	bp,sp
dec	sp
dec	sp
! Debug: div int = const $A to short val = [S+4+4] (used reg = )
mov	ax,6[bp]
mov	bx,*$A
cwd
idiv	bx
! Debug: eq int = ax+0 to short nval = [S+4-4] (used reg = )
mov	-2[bp],ax
!BCC_EOS
! 929   if (nval)
mov	ax,-2[bp]
test	ax,ax
je  	.14
.15:
! 930     put_int(action, nval, width - 1, neg);
! Debug: list unsigned short neg = [S+4+8] (used reg = )
push	$A[bp]
! Debug: sub int = const 1 to short width = [S+6+6] (used reg = )
mov	ax,8[bp]
! Debug: list int = ax-1 (used reg = )
dec	ax
push	ax
! Debug: list short nval = [S+8-4] (used reg = )
push	-2[bp]
! Debug: list unsigned short action = [S+$A+2] (used reg = )
push	4[bp]
! Debug: func () void = put_int+0 (used reg = )
call	_put_int
add	sp,*8
!BCC_EOS
! 931   else {
jmp .16
.14:
! 932     while (--width > 0) send(action, ' ');
jmp .18
.19:
! Debug: list int = const $20 (used reg = )
mov	ax,*$20
push	ax
! Debug: list unsigned short action = [S+6+2] (used reg = )
push	4[bp]
! Debug: func () void = send+0 (used reg = )
call	_send
add	sp,*4
!BCC_EOS
! 933     if (neg) send(action, '-');
.18:
! Debug: predec short width = [S+4+6] (used reg = )
mov	ax,8[bp]
dec	ax
mov	8[bp],ax
! Debug: gt int = const 0 to short = ax+0 (used reg = )
test	ax,ax
jg 	.19
.1A:
.17:
mov	ax,$A[bp]
test	ax,ax
je  	.1B
.1C:
! Debug: list int = const $2D (used reg = )
mov	ax,*$2D
push	ax
! Debug: list unsigned short action = [S+6+2] (used reg = )
push	4[bp]
! Debug: func () void = send+0 (used reg = )
call	_send
add	sp,*4
!BCC_EOS
! 934   }
.1B:
! 935   send(action, val - (nval * 10) + '0');
.16:
! Debug: mul int = const $A to short nval = [S+4-4] (used reg = )
mov	ax,-2[bp]
mov	dx,ax
shl	ax,*1
shl	ax,*1
add	ax,dx
shl	ax,*1
! Debug: sub int = ax+0 to short val = [S+4+4] (used reg = )
push	ax
mov	ax,6[bp]
sub	ax,-4[bp]
inc	sp
inc	sp
! Debug: add int = const $30 to int = ax+0 (used reg = )
! Debug: list int = ax+$30 (used reg = )
add	ax,*$30
push	ax
! Debug: list unsigned short action = [S+6+2] (used reg = )
push	4[bp]
! Debug: func () void = send+0 (used reg = )
call	_send
add	sp,*4
!BCC_EOS
! 936 }
mov	sp,bp
pop	bp
ret
! 937   void
! Register BX used in function put_int
! 938 put_uint(action, val, width, neg)
! 939   Bit16u action;
export	_put_uint
_put_uint:
!BCC_EOS
! 940   unsigned short val;
!BCC_EOS
! 941   short width;
!BCC_EOS
! 942   bx_bool neg;
!BCC_EOS
! 943 {
! 944   unsigned short nval = val / 10;
push	bp
mov	bp,sp
dec	sp
dec	sp
! Debug: div int = const $A to unsigned short val = [S+4+4] (used reg = )
mov	ax,6[bp]
mov	bx,*$A
call	idiv_u
! Debug: eq unsigned int = ax+0 to unsigned short nval = [S+4-4] (used reg = )
mov	-2[bp],ax
!BCC_EOS
! 945   if (nval)
mov	ax,-2[bp]
test	ax,ax
je  	.1D
.1E:
! 946     put_uint(action, nval, width - 1, neg);
! Debug: list unsigned short neg = [S+4+8] (used reg = )
push	$A[bp]
! Debug: sub int = const 1 to short width = [S+6+6] (used reg = )
mov	ax,8[bp]
! Debug: list int = ax-1 (used reg = )
dec	ax
push	ax
! Debug: list unsigned short nval = [S+8-4] (used reg = )
push	-2[bp]
! Debug: list unsigned short action = [S+$A+2] (used reg = )
push	4[bp]
! Debug: func () void = put_uint+0 (used reg = )
call	_put_uint
add	sp,*8
!BCC_EOS
! 947   else {
jmp .1F
.1D:
! 948     while (--width > 0) send(action, ' ');
jmp .21
.22:
! Debug: list int = const $20 (used reg = )
mov	ax,*$20
push	ax
! Debug: list unsigned short action = [S+6+2] (used reg = )
push	4[bp]
! Debug: func () void = send+0 (used reg = )
call	_send
add	sp,*4
!BCC_EOS
! 949     if (neg) send(action, '-');
.21:
! Debug: predec short width = [S+4+6] (used reg = )
mov	ax,8[bp]
dec	ax
mov	8[bp],ax
! Debug: gt int = const 0 to short = ax+0 (used reg = )
test	ax,ax
jg 	.22
.23:
.20:
mov	ax,$A[bp]
test	ax,ax
je  	.24
.25:
! Debug: list int = const $2D (used reg = )
mov	ax,*$2D
push	ax
! Debug: list unsigned short action = [S+6+2] (used reg = )
push	4[bp]
! Debug: func () void = send+0 (used reg = )
call	_send
add	sp,*4
!BCC_EOS
! 950   }
.24:
! 951   send(action, val - (nval * 10) + '0');
.1F:
! Debug: mul int = const $A to unsigned short nval = [S+4-4] (used reg = )
mov	ax,-2[bp]
mov	dx,ax
shl	ax,*1
shl	ax,*1
add	ax,dx
shl	ax,*1
! Debug: sub unsigned int = ax+0 to unsigned short val = [S+4+4] (used reg = )
push	ax
mov	ax,6[bp]
sub	ax,-4[bp]
inc	sp
inc	sp
! Debug: add int = const $30 to unsigned int = ax+0 (used reg = )
! Debug: list unsigned int = ax+$30 (used reg = )
add	ax,*$30
push	ax
! Debug: list unsigned short action = [S+6+2] (used reg = )
push	4[bp]
! Debug: func () void = send+0 (used reg = )
call	_send
add	sp,*4
!BCC_EOS
! 952 }
mov	sp,bp
pop	bp
ret
! 953   void
! Register BX used in function put_uint
! 954 bios_printf(action, s)
! 955   Bit16u action;
export	_bios_printf
_bios_printf:
!BCC_EOS
! 956   Bit8u *s;
!BCC_EOS
! 957 {
! 958   Bit8u c, format_char;
!BCC_EOS
! 959   bx_bool in_format;
!BCC_EOS
! 960   short i;
!BCC_EOS
! 961   Bit16u *arg_ptr;
!BCC_EOS
! 962   Bit16u arg_seg, arg, nibble, shift_count, format_width;
!BCC_EOS
! 963   arg_ptr = &s;
push	bp
mov	bp,sp
add	sp,*-$12
! Debug: eq * * unsigned char s = S+$14+4 to * unsigned short arg_ptr = [S+$14-$A] (used reg = )
lea	bx,6[bp]
mov	-8[bp],bx
!BCC_EOS
! 964   arg_seg = get_SS();
! Debug: func () unsigned short = get_SS+0 (used reg = )
call	_get_SS
! Debug: eq unsigned short = ax+0 to unsigned short arg_seg = [S+$14-$C] (used reg = )
mov	-$A[bp],ax
!BCC_EOS
! 965   in_format = 0;
! Debug: eq int = const 0 to unsigned short in_format = [S+$14-6] (used reg = )
xor	ax,ax
mov	-4[bp],ax
!BCC_EOS
! 966   format_width = 0;
! Debug: eq int = const 0 to unsigned short format_width = [S+$14-$14] (used reg = )
xor	ax,ax
mov	-$12[bp],ax
!BCC_EOS
! 967   if ((action & (2 | 4 | 1)) == (2 | 4 | 1)) {
! Debug: and int = const 7 to unsigned short action = [S+$14+2] (used reg = )
mov	al,4[bp]
and	al,*7
! Debug: logeq int = const 7 to unsigned char = al+0 (used reg = )
cmp	al,*7
jne 	.26
.27:
! 968     outb(0x401, 0x00);
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: list int = const $401 (used reg = )
mov	ax,#$401
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 969     bios_printf (2, "FATAL: ");
! Debug: list * char = .28+0 (used reg = )
mov	bx,#.28
push	bx
! Debug: list int = const 2 (used reg = )
mov	ax,*2
push	ax
! Debug: func () void = bios_printf+0 (used reg = )
call	_bios_printf
add	sp,*4
!BCC_EOS
! 970   }
! 971   while (c = read_byte(get_CS(), s)) {
.26:
br 	.2A
.2B:
! 972     if ( c == '%' ) {
! Debug: logeq int = const $25 to unsigned char c = [S+$14-3] (used reg = )
mov	al,-1[bp]
cmp	al,*$25
jne 	.2C
.2D:
! 973       in_format = 1;
! Debug: eq int = const 1 to unsigned short in_format = [S+$14-6] (used reg = )
mov	ax,*1
mov	-4[bp],ax
!BCC_EOS
! 974       format_width = 0;
! Debug: eq int = const 0 to unsigned short format_width = [S+$14-$14] (used reg = )
xor	ax,ax
mov	-$12[bp],ax
!BCC_EOS
! 975       }
! 976     else if (in_format) {
br 	.2E
.2C:
mov	ax,-4[bp]
test	ax,ax
beq 	.2F
.30:
! 977       if ( (c>='0') && (c<='9') ) {
! Debug: ge int = const $30 to unsigned char c = [S+$14-3] (used reg = )
mov	al,-1[bp]
cmp	al,*$30
jb  	.31
.33:
! Debug: le int = const $39 to unsigned char c = [S+$14-3] (used reg = )
mov	al,-1[bp]
cmp	al,*$39
ja  	.31
.32:
! 978         format_width = (format_width * 10) + (c - '0');
! Debug: sub int = const $30 to unsigned char c = [S+$14-3] (used reg = )
mov	al,-1[bp]
xor	ah,ah
add	ax,*-$30
push	ax
! Debug: mul int = const $A to unsigned short format_width = [S+$16-$14] (used reg = )
mov	ax,-$12[bp]
mov	dx,ax
shl	ax,*1
shl	ax,*1
add	ax,dx
shl	ax,*1
! Debug: add unsigned int (temp) = [S+$16-$16] to unsigned int = ax+0 (used reg = )
add	ax,-$14[bp]
inc	sp
inc	sp
! Debug: eq unsigned int = ax+0 to unsigned short format_width = [S+$14-$14] (used reg = )
mov	-$12[bp],ax
!BCC_EOS
! 979         }
! 980       else {
br 	.34
.31:
! 981         arg_ptr++;
! Debug: postinc * unsigned short arg_ptr = [S+$14-$A] (used reg = )
mov	bx,-8[bp]
inc	bx
inc	bx
mov	-8[bp],bx
!BCC_EOS
! 982         arg = read_word(arg_seg, arg_ptr);
! Debug: list * unsigned short arg_ptr = [S+$14-$A] (used reg = )
push	-8[bp]
! Debug: list unsigned short arg_seg = [S+$16-$C] (used reg = )
push	-$A[bp]
! Debug: func () unsigned short = read_word+0 (used reg = )
call	_read_word
add	sp,*4
! Debug: eq unsigned short = ax+0 to unsigned short arg = [S+$14-$E] (used reg = )
mov	-$C[bp],ax
!BCC_EOS
! 983         if (c == 'x') {
! Debug: logeq int = const $78 to unsigned char c = [S+$14-3] (used reg = )
mov	al,-1[bp]
cmp	al,*$78
jne 	.35
.36:
! 984           if (format_width == 0)
! Debug: logeq int = const 0 to unsigned short format_width = [S+$14-$14] (used reg = )
mov	ax,-$12[bp]
test	ax,ax
jne 	.37
.38:
! 985             format_width = 4;
! Debug: eq int = const 4 to unsigned short format_width = [S+$14-$14] (used reg = )
mov	ax,*4
mov	-$12[bp],ax
!BCC_EOS
! 986           for (i=format_width-1; i>=0; i--) {
.37:
! Debug: sub int = const 1 to unsigned short format_width = [S+$14-$14] (used reg = )
mov	ax,-$12[bp]
! Debug: eq unsigned int = ax-1 to short i = [S+$14-8] (used reg = )
dec	ax
mov	-6[bp],ax
!BCC_EOS
!BCC_EOS
jmp .3B
.3C:
! 987             nibble = (arg >> (4 * i)) & 0x000f;
! Debug: mul short i = [S+$14-8] to int = const 4 (used reg = )
! Debug: expression subtree swapping
mov	ax,-6[bp]
shl	ax,*1
shl	ax,*1
! Debug: sr int = ax+0 to unsigned short arg = [S+$14-$E] (used reg = )
mov	bx,ax
mov	ax,-$C[bp]
mov	cx,bx
shr	ax,cl
! Debug: and int = const $F to unsigned int = ax+0 (used reg = )
and	al,*$F
! Debug: eq unsigned char = al+0 to unsigned short nibble = [S+$14-$10] (used reg = )
xor	ah,ah
mov	-$E[bp],ax
!BCC_EOS
! 988             send (action, (nibble<=9)? (nibble+'0') : (nibble-10+'A'));
! Debug: le int = const 9 to unsigned short nibble = [S+$14-$10] (used reg = )
mov	ax,-$E[bp]
cmp	ax,*9
ja  	.3D
.3E:
! Debug: add int = const $30 to unsigned short nibble = [S+$14-$10] (used reg = )
mov	ax,-$E[bp]
add	ax,*$30
jmp .3F
.3D:
! Debug: sub int = const $A to unsigned short nibble = [S+$14-$10] (used reg = )
mov	ax,-$E[bp]
! Debug: add int = const $41 to unsigned int = ax-$A (used reg = )
add	ax,*$37
.3F:
! Debug: list unsigned int = ax+0 (used reg = )
push	ax
! Debug: list unsigned short action = [S+$16+2] (used reg = )
push	4[bp]
! Debug: func () void = send+0 (used reg = )
call	_send
add	sp,*4
!BCC_EOS
! 989             }
! 990           }
.3A:
! Debug: postdec short i = [S+$14-8] (used reg = )
mov	ax,-6[bp]
dec	ax
mov	-6[bp],ax
.3B:
! Debug: ge int = const 0 to short i = [S+$14-8] (used reg = )
mov	ax,-6[bp]
test	ax,ax
jge	.3C
.40:
.39:
! 991         else if (c == 'u') {
br 	.41
.35:
! Debug: logeq int = const $75 to unsigned char c = [S+$14-3] (used reg = )
mov	al,-1[bp]
cmp	al,*$75
jne 	.42
.43:
! 992           put_uint(action, arg, format_width, 0);
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: list unsigned short format_width = [S+$16-$14] (used reg = )
push	-$12[bp]
! Debug: list unsigned short arg = [S+$18-$E] (used reg = )
push	-$C[bp]
! Debug: list unsigned short action = [S+$1A+2] (used reg = )
push	4[bp]
! Debug: func () void = put_uint+0 (used reg = )
call	_put_uint
add	sp,*8
!BCC_EOS
! 993           }
! 994         else if (c == 'd') {
jmp .44
.42:
! Debug: logeq int = const $64 to unsigned char c = [S+$14-3] (used reg = )
mov	al,-1[bp]
cmp	al,*$64
jne 	.45
.46:
! 995           if (arg & 0x8000)
! Debug: and unsigned int = const $8000 to unsigned short arg = [S+$14-$E] (used reg = )
mov	ax,-$C[bp]
and	ax,#$8000
test	ax,ax
je  	.47
.48:
! 996             put_int(action, -arg, format_width - 1, 1);
! Debug: list int = const 1 (used reg = )
mov	ax,*1
push	ax
! Debug: sub int = const 1 to unsigned short format_width = [S+$16-$14] (used reg = )
mov	ax,-$12[bp]
! Debug: list unsigned int = ax-1 (used reg = )
dec	ax
push	ax
! Debug: neg unsigned short arg = [S+$18-$E] (used reg = )
xor	ax,ax
sub	ax,-$C[bp]
! Debug: list unsigned int = ax+0 (used reg = )
push	ax
! Debug: list unsigned short action = [S+$1A+2] (used reg = )
push	4[bp]
! Debug: func () void = put_int+0 (used reg = )
call	_put_int
add	sp,*8
!BCC_EOS
! 997           else
! 998             put_int(action, arg, format_width, 0);
jmp .49
.47:
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: list unsigned short format_width = [S+$16-$14] (used reg = )
push	-$12[bp]
! Debug: list unsigned short arg = [S+$18-$E] (used reg = )
push	-$C[bp]
! Debug: list unsigned short action = [S+$1A+2] (used reg = )
push	4[bp]
! Debug: func () void = put_int+0 (used reg = )
call	_put_int
add	sp,*8
!BCC_EOS
! 999           }
.49:
! 1000         else if (c == 's') {
jmp .4A
.45:
! Debug: logeq int = const $73 to unsigned char c = [S+$14-3] (used reg = )
mov	al,-1[bp]
cmp	al,*$73
jne 	.4B
.4C:
! 1001           bios_printf(action & (~1), arg);
! Debug: list unsigned short arg = [S+$14-$E] (used reg = )
push	-$C[bp]
! Debug: and int = const -2 to unsigned short action = [S+$16+2] (used reg = )
mov	ax,4[bp]
and	al,#$FE
! Debug: list unsigned int = ax+0 (used reg = )
push	ax
! Debug: func () void = bios_printf+0 (used reg = )
call	_bios_printf
add	sp,*4
!BCC_EOS
! 1002           }
! 1003         e
! 1003 lse if (c == 'c') {
jmp .4D
.4B:
! Debug: logeq int = const $63 to unsigned char c = [S+$14-3] (used reg = )
mov	al,-1[bp]
cmp	al,*$63
jne 	.4E
.4F:
! 1004           send(action, arg);
! Debug: list unsigned short arg = [S+$14-$E] (used reg = )
push	-$C[bp]
! Debug: list unsigned short action = [S+$16+2] (used reg = )
push	4[bp]
! Debug: func () void = send+0 (used reg = )
call	_send
add	sp,*4
!BCC_EOS
! 1005           }
! 1006         else
! 1007           bios_printf((2 | 4 | 1), "bios_printf: unknown format\n");
jmp .50
.4E:
! Debug: list * char = .51+0 (used reg = )
mov	bx,#.51
push	bx
! Debug: list int = const 7 (used reg = )
mov	ax,*7
push	ax
! Debug: func () void = bios_printf+0 (used reg = )
call	_bios_printf
add	sp,*4
!BCC_EOS
! 1008           in_format = 0;
.50:
.4D:
.4A:
.44:
.41:
! Debug: eq int = const 0 to unsigned short in_format = [S+$14-6] (used reg = )
xor	ax,ax
mov	-4[bp],ax
!BCC_EOS
! 1009         }
! 1010       }
.34:
! 1011     else {
jmp .52
.2F:
! 1012       send(action, c);
! Debug: list unsigned char c = [S+$14-3] (used reg = )
mov	al,-1[bp]
xor	ah,ah
push	ax
! Debug: list unsigned short action = [S+$16+2] (used reg = )
push	4[bp]
! Debug: func () void = send+0 (used reg = )
call	_send
add	sp,*4
!BCC_EOS
! 1013       }
! 1014     s ++;
.52:
.2E:
! Debug: postinc * unsigned char s = [S+$14+4] (used reg = )
mov	bx,6[bp]
inc	bx
mov	6[bp],bx
!BCC_EOS
! 1015     }
! 1016   if (action & 1) {
.2A:
! Debug: list * unsigned char s = [S+$14+4] (used reg = )
push	6[bp]
! Debug: func () unsigned short = get_CS+0 (used reg = )
call	_get_CS
! Debug: list unsigned short = ax+0 (used reg = )
push	ax
! Debug: func () unsigned char = read_byte+0 (used reg = )
call	_read_byte
add	sp,*4
! Debug: eq unsigned char = al+0 to unsigned char c = [S+$14-3] (used reg = )
mov	-1[bp],al
test	al,al
bne 	.2B
.53:
.29:
! Debug: and int = const 1 to unsigned short action = [S+$14+2] (used reg = )
mov	al,4[bp]
and	al,*1
test	al,al
je  	.54
.55:
! 1017 #asm
!BCC_EOS
!BCC_ASM
_bios_printf.format_width	set	0
.bios_printf.format_width	set	-$12
_bios_printf.format_char	set	$10
.bios_printf.format_char	set	-2
_bios_printf.arg_ptr	set	$A
.bios_printf.arg_ptr	set	-8
_bios_printf.action	set	$16
.bios_printf.action	set	4
_bios_printf.i	set	$C
.bios_printf.i	set	-6
_bios_printf.arg_seg	set	8
.bios_printf.arg_seg	set	-$A
_bios_printf.shift_count	set	2
.bios_printf.shift_count	set	-$10
_bios_printf.in_format	set	$E
.bios_printf.in_format	set	-4
_bios_printf.s	set	$18
.bios_printf.s	set	6
_bios_printf.nibble	set	4
.bios_printf.nibble	set	-$E
_bios_printf.c	set	$11
.bios_printf.c	set	-1
_bios_printf.arg	set	6
.bios_printf.arg	set	-$C
    cli
 halt2_loop:
    hlt
    jmp halt2_loop
! 1022 endasm
!BCC_ENDASM
!BCC_EOS
! 1023     }
! 1024 }
.54:
mov	sp,bp
pop	bp
ret
! 1025   void
! Register BX used in function bios_printf
! 1026 keyboard_init()
! 1027 {
export	_keyboard_init
_keyboard_init:
! 1028     Bit16u max;
!BCC_EOS
! 1029     max=0xffff;
push	bp
mov	bp,sp
dec	sp
dec	sp
! Debug: eq unsigned int = const $FFFF to unsigned short max = [S+4-4] (used reg = )
mov	ax,#$FFFF
mov	-2[bp],ax
!BCC_EOS
! 1030     while ( (inb(0x64) & 0x02) && (--max>0)) outb(0x80, 0x00);
jmp .57
.58:
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: list int = const $80 (used reg = )
mov	ax,#$80
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 1031     max=0x2000;
.57:
! Debug: list int = const $64 (used reg = )
mov	ax,*$64
push	ax
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
inc	sp
inc	sp
! Debug: and int = const 2 to unsigned char = al+0 (used reg = )
and	al,*2
test	al,al
je  	.59
.5A:
! Debug: predec unsigned short max = [S+4-4] (used reg = )
mov	ax,-2[bp]
dec	ax
mov	-2[bp],ax
! Debug: gt int = const 0 to unsigned short = ax+0 (used reg = )
test	ax,ax
jne	.58
.59:
.56:
! Debug: eq int = const $2000 to unsigned short max = [S+4-4] (used reg = )
mov	ax,#$2000
mov	-2[bp],ax
!BCC_EOS
! 1032     while (--max > 0) {
jmp .5C
.5D:
! 1033         outb(0x80, 0x00);
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: list int = const $80 (used reg = )
mov	ax,#$80
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 1034         if (inb(0x64) & 0x01) {
! Debug: list int = const $64 (used reg = )
mov	ax,*$64
push	ax
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
inc	sp
inc	sp
! Debug: and int = const 1 to unsigned char = al+0 (used reg = )
and	al,*1
test	al,al
je  	.5E
.5F:
! 1035             inb(0x60);
! Debug: list int = const $60 (used reg = )
mov	ax,*$60
push	ax
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
inc	sp
inc	sp
!BCC_EOS
! 1036             max = 0x2000;
! Debug: eq int = const $2000 to unsigned short max = [S+4-4] (used reg = )
mov	ax,#$2000
mov	-2[bp],ax
!BCC_EOS
! 1037             }
! 1038         }
.5E:
! 1039     outb(0x64, 0xaa);
.5C:
! Debug: predec unsigned short max = [S+4-4] (used reg = )
mov	ax,-2[bp]
dec	ax
mov	-2[bp],ax
! Debug: gt int = const 0 to unsigned short = ax+0 (used reg = )
test	ax,ax
jne	.5D
.60:
.5B:
! Debug: list int = const $AA (used reg = )
mov	ax,#$AA
push	ax
! Debug: list int = const $64 (used reg = )
mov	ax,*$64
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 1040     max=0xffff;
! Debug: eq unsigned int = const $FFFF to unsigned short max = [S+4-4] (used reg = )
mov	ax,#$FFFF
mov	-2[bp],ax
!BCC_EOS
! 1041     while ( (inb(0x64) & 0x02) && (--max>0)) outb(0x80, 0x00);
jmp .62
.63:
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: list int = const $80 (used reg = )
mov	ax,#$80
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 1042     if (max==0x0) keyboard_panic(00);
.62:
! Debug: list int = const $64 (used reg = )
mov	ax,*$64
push	ax
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
inc	sp
inc	sp
! Debug: and int = const 2 to unsigned char = al+0 (used reg = )
and	al,*2
test	al,al
je  	.64
.65:
! Debug: predec unsigned short max = [S+4-4] (used reg = )
mov	ax,-2[bp]
dec	ax
mov	-2[bp],ax
! Debug: gt int = const 0 to unsigned short = ax+0 (used reg = )
test	ax,ax
jne	.63
.64:
.61:
! Debug: logeq int = const 0 to unsigned short max = [S+4-4] (used reg = )
mov	ax,-2[bp]
test	ax,ax
jne 	.66
.67:
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: func () void = keyboard_panic+0 (used reg = )
call	_keyboard_panic
inc	sp
inc	sp
!BCC_EOS
! 1043     max=0xffff;
.66:
! Debug: eq unsigned int = const $FFFF to unsigned short max = [S+4-4] (used reg = )
mov	ax,#$FFFF
mov	-2[bp],ax
!BCC_EOS
! 1044     while ( ((inb(0x64) & 0x01) == 0) && (--max>0) ) outb(0x80, 0x01);
jmp .69
.6A:
! Debug: list int = const 1 (used reg = )
mov	ax,*1
push	ax
! Debug: list int = const $80 (used reg = )
mov	ax,#$80
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 1045     if (max==0x0) keyboard_panic(01);
.69:
! Debug: list int = const $64 (used reg = )
mov	ax,*$64
push	ax
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
inc	sp
inc	sp
! Debug: and int = const 1 to unsigned char = al+0 (used reg = )
and	al,*1
! Debug: logeq int = const 0 to unsigned char = al+0 (used reg = )
test	al,al
jne 	.6B
.6C:
! Debug: predec unsigned short max = [S+4-4] (used reg = )
mov	ax,-2[bp]
dec	ax
mov	-2[bp],ax
! Debug: gt int = const 0 to unsigned short = ax+0 (used reg = )
test	ax,ax
jne	.6A
.6B:
.68:
! Debug: logeq int = const 0 to unsigned short max = [S+4-4] (used reg = )
mov	ax,-2[bp]
test	ax,ax
jne 	.6D
.6E:
! Debug: list int = const 1 (used reg = )
mov	ax,*1
push	ax
! Debug: func () void = keyboard_panic+0 (used reg = )
call	_keyboard_panic
inc	sp
inc	sp
!BCC_EOS
! 1046     if ((inb(0x60) != 0x55)){
.6D:
! Debug: list int = const $60 (used reg = )
mov	ax,*$60
push	ax
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
inc	sp
inc	sp
! Debug: ne int = const $55 to unsigned char = al+0 (used reg = )
cmp	al,*$55
je  	.6F
.70:
! 1047         keyboard_panic(991);
! Debug: list int = const $3DF (used reg = )
mov	ax,#$3DF
push	ax
! Debug: func () void = keyboard_panic+0 (used reg = )
call	_keyboard_panic
inc	sp
inc	sp
!BCC_EOS
! 1048     }
! 1049     outb(0x64,0xab);
.6F:
! Debug: list int = const $AB (used reg = )
mov	ax,#$AB
push	ax
! Debug: list int = const $64 (used reg = )
mov	ax,*$64
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 1050     max=0xffff;
! Debug: eq unsigned int = const $FFFF to unsigned short max = [S+4-4] (used reg = )
mov	ax,#$FFFF
mov	-2[bp],ax
!BCC_EOS
! 1051     while ((inb(0x64) & 0x02) && (--max>0)) outb(0x80, 0x10);
jmp .72
.73:
! Debug: list int = const $10 (used reg = )
mov	ax,*$10
push	ax
! Debug: list int = const $80 (used reg = )
mov	ax,#$80
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 1052     if (max==0x0) keyboard_panic(10);
.72:
! Debug: list int = const $64 (used reg = )
mov	ax,*$64
push	ax
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
inc	sp
inc	sp
! Debug: and int = const 2 to unsigned char = al+0 (used reg = )
and	al,*2
test	al,al
je  	.74
.75:
! Debug: predec unsigned short max = [S+4-4] (used reg = )
mov	ax,-2[bp]
dec	ax
mov	-2[bp],ax
! Debug: gt int = const 0 to unsigned short = ax+0 (used reg = )
test	ax,ax
jne	.73
.74:
.71:
! Debug: logeq int = const 0 to unsigned short max = [S+4-4] (used reg = )
mov	ax,-2[bp]
test	ax,ax
jne 	.76
.77:
! Debug: list int = const $A (used reg = )
mov	ax,*$A
push	ax
! Debug: func () void = keyboard_panic+0 (used reg = )
call	_keyboard_panic
inc	sp
inc	sp
!BCC_EOS
! 1053     max=0xffff;
.76:
! Debug: eq unsigned int = const $FFFF to unsigned short max = [S+4-4] (used reg = )
mov	ax,#$FFFF
mov	-2[bp],ax
!BCC_EOS
! 1054     while ( ((inb(0x64) & 0x01) == 0) && (--max>0) ) outb(0x80, 0x11);
jmp .79
.7A:
! Debug: list int = const $11 (used reg = )
mov	ax,*$11
push	ax
! Debug: list int = const $80 (used reg = )
mov	ax,#$80
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 1055     if (max==0x0) keyboard_panic(11);
.79:
! Debug: list int = const $64 (used reg = )
mov	ax,*$64
push	ax
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
inc	sp
inc	sp
! Debug: and int = const 1 to unsigned char = al+0 (used reg = )
and	al,*1
! Debug: logeq int = const 0 to unsigned char = al+0 (used reg = )
test	al,al
jne 	.7B
.7C:
! Debug: predec unsigned short max = [S+4-4] (used reg = )
mov	ax,-2[bp]
dec	ax
mov	-2[bp],ax
! Debug: gt int = const 0 to unsigned short = ax+0 (used reg = )
test	ax,ax
jne	.7A
.7B:
.78:
! Debug: logeq int = const 0 to unsigned short max = [S+4-4] (used reg = )
mov	ax,-2[bp]
test	ax,ax
jne 	.7D
.7E:
! Debug: list int = const $B (used reg = )
mov	ax,*$B
push	ax
! Debug: func () void = keyboard_panic+0 (used reg = )
call	_keyboard_panic
inc	sp
inc	sp
!BCC_EOS
! 1056     if ((inb(0x60) != 0x00)) {
.7D:
! Debug: list int = const $60 (used reg = )
mov	ax,*$60
push	ax
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
inc	sp
inc	sp
! Debug: ne int = const 0 to unsigned char = al+0 (used reg = )
test	al,al
je  	.7F
.80:
! 1057         keyboard_panic(992);
! Debug: list int = const $3E0 (used reg = )
mov	ax,#$3E0
push	ax
! Debug: func () void = keyboard_panic+0 (used reg = )
call	_keyboard_panic
inc	sp
inc	sp
!BCC_EOS
! 1058     }
! 1059     outb(0x64,0xae);
.7F:
! Debug: list int = const $AE (used reg = )
mov	ax,#$AE
push	ax
! Debug: list int = const $64 (used reg = )
mov	ax,*$64
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 1060     outb(0x64,0xa8);
! Debug: list int = const $A8 (used reg = )
mov	ax,#$A8
push	ax
! Debug: list int = const $64 (used reg = )
mov	ax,*$64
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 1061     outb(0x60, 0xff);
! Debug: list int = const $FF (used reg = )
mov	ax,#$FF
push	ax
! Debug: list int = const $60 (used reg = )
mov	ax,*$60
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 1062     max=0xffff;
! Debug: eq unsigned int = const $FFFF to unsigned short max = [S+4-4] (used reg = )
mov	ax,#$FFFF
mov	-2[bp],ax
!BCC_EOS
! 1063     while ((inb(0x64) & 0x02) && (--max>0)) outb(0x80, 0x20);
jmp .82
.83:
! Debug: list int = const $20 (used reg = )
mov	ax,*$20
push	ax
! Debug: list int = const $80 (used reg = )
mov	ax,#$80
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 1064     if (max==0x0) keyboard_panic(20);
.82:
! Debug: list int = const $64 (used reg = )
mov	ax,*$64
push	ax
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
inc	sp
inc	sp
! Debug: and int = const 2 to unsigned char = al+0 (used reg = )
and	al,*2
test	al,al
je  	.84
.85:
! Debug: predec unsigned short max = [S+4-4] (used reg = )
mov	ax,-2[bp]
dec	ax
mov	-2[bp],ax
! Debug: gt int = const 0 to unsigned short = ax+0 (used reg = )
test	ax,ax
jne	.83
.84:
.81:
! Debug: logeq int = const 0 to unsigned short max = [S+4-4] (used reg = )
mov	ax,-2[bp]
test	ax,ax
jne 	.86
.87:
! Debug: list int = const $14 (used reg = )
mov	ax,*$14
push	ax
! Debug: func () void = keyboard_panic+0 (used reg = )
call	_keyboard_panic
inc	sp
inc	sp
!BCC_EOS
! 1065     max=0xffff;
.86:
! Debug: eq unsigned int = const $FFFF to unsigned short max = [S+4-4] (used reg = )
mov	ax,#$FFFF
mov	-2[bp],ax
!BCC_EOS
! 1066     while ( ((inb(0x64) & 0x01) == 0) && (--max>0) ) outb(0x80, 0x21);
jmp .89
.8A:
! Debug: list int = const $21 (used reg = )
mov	ax,*$21
push	ax
! Debug: list int = const $80 (used reg = )
mov	ax,#$80
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 1067     if (max==0x0) keyboard_panic(21);
.89:
! Debug: list int = const $64 (used reg = )
mov	ax,*$64
push	ax
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
inc	sp
inc	sp
! Debug: and int = const 1 to unsigned char = al+0 (used reg = )
and	al,*1
! Debug: logeq int = const 0 to unsigned char = al+0 (used reg = )
test	al,al
jne 	.8B
.8C:
! Debug: predec unsigned short max = [S+4-4] (used reg = )
mov	ax,-2[bp]
dec	ax
mov	-2[bp],ax
! Debug: gt int = const 0 to unsigned short = ax+0 (used reg = )
test	ax,ax
jne	.8A
.8B:
.88:
! Debug: logeq int = const 0 to unsigned short max = [S+4-4] (used reg = )
mov	ax,-2[bp]
test	ax,ax
jne 	.8D
.8E:
! Debug: list int = const $15 (used reg = )
mov	ax,*$15
push	ax
! Debug: func () void = keyboard_panic+0 (used reg = )
call	_keyboard_panic
inc	sp
inc	sp
!BCC_EOS
! 1068     if ((inb(0x60) != 0xfa)) {
.8D:
! Debug: list int = const $60 (used reg = )
mov	ax,*$60
push	ax
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
inc	sp
inc	sp
! Debug: ne int = const $FA to unsigned char = al+0 (used reg = )
cmp	al,#$FA
je  	.8F
.90:
! 1069         keyboard_panic(993);
! Debug: list int = const $3E1 (used reg = )
mov	ax,#$3E1
push	ax
! Debug: func () void = keyboard_panic+0 (used reg = )
call	_keyboard_panic
inc	sp
inc	sp
!BCC_EOS
! 1070     }
! 1071     max=0xffff;
.8F:
! Debug: eq unsigned int = const $FFFF to unsigned short max = [S+4-4] (used reg = )
mov	ax,#$FFFF
mov	-2[bp],ax
!BCC_EOS
! 1072     while ( ((inb(0x64) & 0x01) == 0) && (--max>0) ) outb(0x80, 0x31);
jmp .92
.93:
! Debug: list int = const $31 (used reg = )
mov	ax,*$31
push	ax
! Debug: list int = const $80 (used reg = )
mov	ax,#$80
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 1073     if (max==0x0) keyboard_panic(31);
.92:
! Debug: list int = const $64 (used reg = )
mov	ax,*$64
push	ax
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
inc	sp
inc	sp
! Debug: and int = const 1 to unsigned char = al+0 (used reg = )
and	al,*1
! Debug: logeq int = const 0 to unsigned char = al+0 (used reg = )
test	al,al
jne 	.94
.95:
! Debug: predec unsigned short max = [S+4-4] (used reg = )
mov	ax,-2[bp]
dec	ax
mov	-2[bp],ax
! Debug: gt int = const 0 to unsigned short = ax+0 (used reg = )
test	ax,ax
jne	.93
.94:
.91:
! Debug: logeq int = const 0 to unsigned short max = [S+4-4] (used reg = )
mov	ax,-2[bp]
test	ax,ax
jne 	.96
.97:
! Debug: list int = const $1F (used reg = )
mov	ax,*$1F
push	ax
! Debug: func () void = keyboard_panic+0 (used reg = )
call	_keyboard_panic
inc	sp
inc	sp
!BCC_EOS
! 1074     if ((inb(0x60) != 0xaa)) {
.96:
! Debug: list int = const $60 (used reg = )
mov	ax,*$60
push	ax
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
inc	sp
inc	sp
! Debug: ne int = const $AA to unsigned char = al+0 (used reg = )
cmp	al,#$AA
je  	.98
.99:
! 1075         keyboard_panic(994);
! Debug: list int = const $3E2 (used reg = )
mov	ax,#$3E2
push	ax
! Debug: func () void = keyboard_panic+0 (used reg = )
call	_keyboard_panic
inc	sp
inc	sp
!BCC_EOS
! 1076     }
! 1077     outb(0x60, 0xf5);
.98:
! Debug: list int = const $F5 (used reg = )
mov	ax,#$F5
push	ax
! Debug: list int = const $60 (used reg = )
mov	ax,*$60
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 1078     max=0xffff;
! Debug: eq unsigned int = const $FFFF to unsigned short max = [S+4-4] (used reg = )
mov	ax,#$FFFF
mov	-2[bp],ax
!BCC_EOS
! 1079     while ((inb(0x64) & 0x02) && (--max>0)) outb(0x80, 0x40);
jmp .9B
.9C:
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: list int = const $80 (used reg = )
mov	ax,#$80
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 1080     if (max==0x0) keyboard_panic(40);
.9B:
! Debug: list int = const $64 (used reg = )
mov	ax,*$64
push	ax
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
inc	sp
inc	sp
! Debug: and int = const 2 to unsigned char = al+0 (used reg = )
and	al,*2
test	al,al
je  	.9D
.9E:
! Debug: predec unsigned short max = [S+4-4] (used reg = )
mov	ax,-2[bp]
dec	ax
mov	-2[bp],ax
! Debug: gt int = const 0 to unsigned short = ax+0 (used reg = )
test	ax,ax
jne	.9C
.9D:
.9A:
! Debug: logeq int = const 0 to unsigned short max = [S+4-4] (used reg = )
mov	ax,-2[bp]
test	ax,ax
jne 	.9F
.A0:
! Debug: list int = const $28 (used reg = )
mov	ax,*$28
push	ax
! Debug: func () void = keyboard_panic+0 (used reg = )
call	_keyboard_panic
inc	sp
inc	sp
!BCC_EOS
! 1081     max=0xffff;
.9F:
! Debug: eq unsigned int = const $FFFF to unsigned short max = [S+4-4] (used reg = )
mov	ax,#$FFFF
mov	-2[bp],ax
!BCC_EOS
! 1082     while ( ((inb(0x64) & 0x01) == 0) && (--max>0) 
! 1082 ) outb(0x80, 0x41);
jmp .A2
.A3:
! Debug: list int = const $41 (used reg = )
mov	ax,*$41
push	ax
! Debug: list int = const $80 (used reg = )
mov	ax,#$80
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 1083     if (max==0x0) keyboard_panic(41);
.A2:
! Debug: list int = const $64 (used reg = )
mov	ax,*$64
push	ax
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
inc	sp
inc	sp
! Debug: and int = const 1 to unsigned char = al+0 (used reg = )
and	al,*1
! Debug: logeq int = const 0 to unsigned char = al+0 (used reg = )
test	al,al
jne 	.A4
.A5:
! Debug: predec unsigned short max = [S+4-4] (used reg = )
mov	ax,-2[bp]
dec	ax
mov	-2[bp],ax
! Debug: gt int = const 0 to unsigned short = ax+0 (used reg = )
test	ax,ax
jne	.A3
.A4:
.A1:
! Debug: logeq int = const 0 to unsigned short max = [S+4-4] (used reg = )
mov	ax,-2[bp]
test	ax,ax
jne 	.A6
.A7:
! Debug: list int = const $29 (used reg = )
mov	ax,*$29
push	ax
! Debug: func () void = keyboard_panic+0 (used reg = )
call	_keyboard_panic
inc	sp
inc	sp
!BCC_EOS
! 1084     if ((inb(0x60) != 0xfa)) {
.A6:
! Debug: list int = const $60 (used reg = )
mov	ax,*$60
push	ax
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
inc	sp
inc	sp
! Debug: ne int = const $FA to unsigned char = al+0 (used reg = )
cmp	al,#$FA
je  	.A8
.A9:
! 1085         keyboard_panic(995);
! Debug: list int = const $3E3 (used reg = )
mov	ax,#$3E3
push	ax
! Debug: func () void = keyboard_panic+0 (used reg = )
call	_keyboard_panic
inc	sp
inc	sp
!BCC_EOS
! 1086     }
! 1087     outb(0x64, 0x60);
.A8:
! Debug: list int = const $60 (used reg = )
mov	ax,*$60
push	ax
! Debug: list int = const $64 (used reg = )
mov	ax,*$64
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 1088     max=0xffff;
! Debug: eq unsigned int = const $FFFF to unsigned short max = [S+4-4] (used reg = )
mov	ax,#$FFFF
mov	-2[bp],ax
!BCC_EOS
! 1089     while ((inb(0x64) & 0x02) && (--max>0)) outb(0x80, 0x50);
jmp .AB
.AC:
! Debug: list int = const $50 (used reg = )
mov	ax,*$50
push	ax
! Debug: list int = const $80 (used reg = )
mov	ax,#$80
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 1090     if (max==0x0) keyboard_panic(50);
.AB:
! Debug: list int = const $64 (used reg = )
mov	ax,*$64
push	ax
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
inc	sp
inc	sp
! Debug: and int = const 2 to unsigned char = al+0 (used reg = )
and	al,*2
test	al,al
je  	.AD
.AE:
! Debug: predec unsigned short max = [S+4-4] (used reg = )
mov	ax,-2[bp]
dec	ax
mov	-2[bp],ax
! Debug: gt int = const 0 to unsigned short = ax+0 (used reg = )
test	ax,ax
jne	.AC
.AD:
.AA:
! Debug: logeq int = const 0 to unsigned short max = [S+4-4] (used reg = )
mov	ax,-2[bp]
test	ax,ax
jne 	.AF
.B0:
! Debug: list int = const $32 (used reg = )
mov	ax,*$32
push	ax
! Debug: func () void = keyboard_panic+0 (used reg = )
call	_keyboard_panic
inc	sp
inc	sp
!BCC_EOS
! 1091     outb(0x60, 0x61);
.AF:
! Debug: list int = const $61 (used reg = )
mov	ax,*$61
push	ax
! Debug: list int = const $60 (used reg = )
mov	ax,*$60
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 1092     max=0xffff;
! Debug: eq unsigned int = const $FFFF to unsigned short max = [S+4-4] (used reg = )
mov	ax,#$FFFF
mov	-2[bp],ax
!BCC_EOS
! 1093     while ((inb(0x64) & 0x02) && (--max>0)) outb(0x80, 0x60);
jmp .B2
.B3:
! Debug: list int = const $60 (used reg = )
mov	ax,*$60
push	ax
! Debug: list int = const $80 (used reg = )
mov	ax,#$80
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 1094     if (max==0x0) keyboard_panic(60);
.B2:
! Debug: list int = const $64 (used reg = )
mov	ax,*$64
push	ax
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
inc	sp
inc	sp
! Debug: and int = const 2 to unsigned char = al+0 (used reg = )
and	al,*2
test	al,al
je  	.B4
.B5:
! Debug: predec unsigned short max = [S+4-4] (used reg = )
mov	ax,-2[bp]
dec	ax
mov	-2[bp],ax
! Debug: gt int = const 0 to unsigned short = ax+0 (used reg = )
test	ax,ax
jne	.B3
.B4:
.B1:
! Debug: logeq int = const 0 to unsigned short max = [S+4-4] (used reg = )
mov	ax,-2[bp]
test	ax,ax
jne 	.B6
.B7:
! Debug: list int = const $3C (used reg = )
mov	ax,*$3C
push	ax
! Debug: func () void = keyboard_panic+0 (used reg = )
call	_keyboard_panic
inc	sp
inc	sp
!BCC_EOS
! 1095     outb(0x60, 0xf4);
.B6:
! Debug: list int = const $F4 (used reg = )
mov	ax,#$F4
push	ax
! Debug: list int = const $60 (used reg = )
mov	ax,*$60
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 1096     max=0xffff;
! Debug: eq unsigned int = const $FFFF to unsigned short max = [S+4-4] (used reg = )
mov	ax,#$FFFF
mov	-2[bp],ax
!BCC_EOS
! 1097     while ((inb(0x64) & 0x02) && (--max>0)) outb(0x80, 0x70);
jmp .B9
.BA:
! Debug: list int = const $70 (used reg = )
mov	ax,*$70
push	ax
! Debug: list int = const $80 (used reg = )
mov	ax,#$80
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 1098     if (max==0x0) keyboard_panic(70);
.B9:
! Debug: list int = const $64 (used reg = )
mov	ax,*$64
push	ax
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
inc	sp
inc	sp
! Debug: and int = const 2 to unsigned char = al+0 (used reg = )
and	al,*2
test	al,al
je  	.BB
.BC:
! Debug: predec unsigned short max = [S+4-4] (used reg = )
mov	ax,-2[bp]
dec	ax
mov	-2[bp],ax
! Debug: gt int = const 0 to unsigned short = ax+0 (used reg = )
test	ax,ax
jne	.BA
.BB:
.B8:
! Debug: logeq int = const 0 to unsigned short max = [S+4-4] (used reg = )
mov	ax,-2[bp]
test	ax,ax
jne 	.BD
.BE:
! Debug: list int = const $46 (used reg = )
mov	ax,*$46
push	ax
! Debug: func () void = keyboard_panic+0 (used reg = )
call	_keyboard_panic
inc	sp
inc	sp
!BCC_EOS
! 1099     max=0xffff;
.BD:
! Debug: eq unsigned int = const $FFFF to unsigned short max = [S+4-4] (used reg = )
mov	ax,#$FFFF
mov	-2[bp],ax
!BCC_EOS
! 1100     while ( ((inb(0x64) & 0x01) == 0) && (--max>0) ) outb(0x80, 0x71);
jmp .C0
.C1:
! Debug: list int = const $71 (used reg = )
mov	ax,*$71
push	ax
! Debug: list int = const $80 (used reg = )
mov	ax,#$80
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 1101     if (max==0x0) keyboard_panic(70);
.C0:
! Debug: list int = const $64 (used reg = )
mov	ax,*$64
push	ax
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
inc	sp
inc	sp
! Debug: and int = const 1 to unsigned char = al+0 (used reg = )
and	al,*1
! Debug: logeq int = const 0 to unsigned char = al+0 (used reg = )
test	al,al
jne 	.C2
.C3:
! Debug: predec unsigned short max = [S+4-4] (used reg = )
mov	ax,-2[bp]
dec	ax
mov	-2[bp],ax
! Debug: gt int = const 0 to unsigned short = ax+0 (used reg = )
test	ax,ax
jne	.C1
.C2:
.BF:
! Debug: logeq int = const 0 to unsigned short max = [S+4-4] (used reg = )
mov	ax,-2[bp]
test	ax,ax
jne 	.C4
.C5:
! Debug: list int = const $46 (used reg = )
mov	ax,*$46
push	ax
! Debug: func () void = keyboard_panic+0 (used reg = )
call	_keyboard_panic
inc	sp
inc	sp
!BCC_EOS
! 1102     if ((inb(0x60) != 0xfa)) {
.C4:
! Debug: list int = const $60 (used reg = )
mov	ax,*$60
push	ax
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
inc	sp
inc	sp
! Debug: ne int = const $FA to unsigned char = al+0 (used reg = )
cmp	al,#$FA
je  	.C6
.C7:
! 1103         keyboard_panic(996);
! Debug: list int = const $3E4 (used reg = )
mov	ax,#$3E4
push	ax
! Debug: func () void = keyboard_panic+0 (used reg = )
call	_keyboard_panic
inc	sp
inc	sp
!BCC_EOS
! 1104     }
! 1105     outb(0x80, 0x77);
.C6:
! Debug: list int = const $77 (used reg = )
mov	ax,*$77
push	ax
! Debug: list int = const $80 (used reg = )
mov	ax,#$80
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 1106 }
mov	sp,bp
pop	bp
ret
! 1107   void
! 1108 keyboard_panic(status)
! 1109   Bit16u status;
export	_keyboard_panic
_keyboard_panic:
!BCC_EOS
! 1110 {
! 1111   bios_printf((2 | 4 | 1), "Keyboard error:%u\n",status);
push	bp
mov	bp,sp
! Debug: list unsigned short status = [S+2+2] (used reg = )
push	4[bp]
! Debug: list * char = .C8+0 (used reg = )
mov	bx,#.C8
push	bx
! Debug: list int = const 7 (used reg = )
mov	ax,*7
push	ax
! Debug: func () void = bios_printf+0 (used reg = )
call	_bios_printf
mov	sp,bp
!BCC_EOS
! 1112 }
pop	bp
ret
! 1113   void
! Register BX used in function keyboard_panic
! 1114 machine_reset()
! 1115 {
export	_machine_reset
_machine_reset:
! 1116 #asm
!BCC_ASM
;we must check whether 0xFE is set or not
;if it is s3 resume, just jmp back to normal Post Entry
;below port io will prevent s3 resume
  mov al, #0x0f
  out 0x70, al
  in al, 0x71
  cmp al, #0xFE
  jz post
! 1125 endasm
!BCC_ENDASM
! 1126   outb(0x64, 0x60);
push	bp
mov	bp,sp
! Debug: list int = const $60 (used reg = )
mov	ax,*$60
push	ax
! Debug: list int = const $64 (used reg = )
mov	ax,*$64
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
mov	sp,bp
!BCC_EOS
! 1127   outb(0x60, 0x14);
! Debug: list int = const $14 (used reg = )
mov	ax,*$14
push	ax
! Debug: list int = const $60 (used reg = )
mov	ax,*$60
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
mov	sp,bp
!BCC_EOS
! 1128   outb(0x64, 0xfe);
! Debug: list int = const $FE (used reg = )
mov	ax,#$FE
push	ax
! Debug: list int = const $64 (used reg = )
mov	ax,*$64
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
mov	sp,bp
!BCC_EOS
! 1129   bios_printf((2 | 4 | 1), "Couldn't reset the machine\n");
! Debug: list * char = .C9+0 (used reg = )
mov	bx,#.C9
push	bx
! Debug: list int = const 7 (used reg = )
mov	ax,*7
push	ax
! Debug: func () void = bios_printf+0 (used reg = )
call	_bios_printf
mov	sp,bp
!BCC_EOS
! 1130 }
pop	bp
ret
! 1131   void
! Register BX used in function machine_reset
! 1132 clobber_entry_point()
! 1133 {
export	_clobber_entry_point
_clobber_entry_point:
! 1134     write_word(0xffff, 0x0001, machine_reset);
push	bp
mov	bp,sp
! Debug: cast * () void = const 0 to () void = machine_reset+0 (used reg = )
! Debug: list * () void = machine_reset+0 (used reg = )
mov	bx,#_machine_reset
push	bx
! Debug: list int = const 1 (used reg = )
mov	ax,*1
push	ax
! Debug: list unsigned int = const $FFFF (used reg = )
mov	ax,#$FFFF
push	ax
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
mov	sp,bp
!BCC_EOS
! 1135 }
pop	bp
ret
! 1136   void
! Register BX used in function clobber_entry_point
! 1137 shutdown_status_panic(status)
! 1138   Bit16u status;
export	_shutdown_status_panic
_shutdown_status_panic:
!BCC_EOS
! 1139 {
! 1140   bios_printf((2 | 4 | 1), "Unimplemented shutdown status: %02x\n",(Bit8u)status);
push	bp
mov	bp,sp
! Debug: list unsigned char status = [S+2+2] (used reg = )
mov	al,4[bp]
xor	ah,ah
push	ax
! Debug: list * char = .CA+0 (used reg = )
mov	bx,#.CA
push	bx
! Debug: list int = const 7 (used reg = )
mov	ax,*7
push	ax
! Debug: func () void = bios_printf+0 (used reg = )
call	_bios_printf
mov	sp,bp
!BCC_EOS
! 1141 }
pop	bp
ret
! 1142 void
! Register BX used in function shutdown_status_panic
! 1143 print_bios_banner()
! 1144 {
export	_print_bios_banner
_print_bios_banner:
! 1145   bios_printf(2, "HVMAssist"" BIOS, %d cpu%s, ", 1, 1>1?"s":"");
push	bp
mov	bp,sp
! Debug: list * char = .CD+0 (used reg = )
mov	bx,#.CD
push	bx
! Debug: list int = const 1 (used reg = )
mov	ax,*1
push	ax
! Debug: list * char = .CB+0 (used reg = )
mov	bx,#.CB
push	bx
! Debug: list int = const 2 (used reg = )
mov	ax,*2
push	ax
! Debug: func () void = bios_printf+0 (used reg = )
call	_bios_printf
mov	sp,bp
!BCC_EOS
! 1146   bios_printf(2, "%s %s\n", bios_cvs_version_string, bios_date_string);
! Debug: list * char = bios_date_string+0 (used reg = )
mov	bx,#_bios_date_string
push	bx
! Debug: list * char = bios_cvs_version_string+0 (used reg = )
mov	bx,#_bios_cvs_version_string
push	bx
! Debug: list * char = .CE+0 (used reg = )
mov	bx,#.CE
push	bx
! Debug: list int = const 2 (used reg = )
mov	ax,*2
push	ax
! Debug: func () void = bios_printf+0 (used reg = )
call	_bios_printf
mov	sp,bp
!BCC_EOS
! 1147   bios_printf(2, "TCG-enabled BIOS.\n");
! Debug: list * char = .CF+0 (used reg = )
mov	bx,#.CF
push	bx
! Debug: list int = const 2 (used reg = )
mov	ax,*2
push	ax
! Debug: func () void = bios_printf+0 (used reg = )
call	_bios_printf
mov	sp,bp
!BCC_EOS
! 1148   bios_printf(2, "\n");
! Debug: list * char = .D0+0 (used reg = )
mov	bx,#.D0
push	bx
! Debug: list int = const 2 (used reg = )
mov	ax,*2
push	ax
! Debug: func () void = bios_printf+0 (used reg = )
call	_bios_printf
mov	sp,bp
!BCC_EOS
! 1149 }
pop	bp
ret
! 1150 struct ipl_entry {
! Register BX used in function print_bios_banner
! 1151   Bit16u type;
!BCC_EOS
! 1152   Bit16u flags;
!BCC_EOS
! 1153   Bit32u vector;
!BCC_EOS
! 1154   Bit32u description;
!BCC_EOS
! 1155   Bit32u reserved;
!BCC_EOS
! 1156 };
!BCC_EOS
! 1157 static void
! 1158 init_boot_vectors()
! 1159 {
_init_boot_vectors:
! 1160   struct ipl_entry e;
!BCC_EOS
! 1161   Bit16u count = 0;
push	bp
mov	bp,sp
add	sp,*-$12
! Debug: eq int = const 0 to unsigned short count = [S+$14-$14] (used reg = )
xor	ax,ax
mov	-$12[bp],ax
!BCC_EOS
! 1162   Bit16u ss = get_SS();
dec	sp
dec	sp
! Debug: func () unsigned short = get_SS+0 (used reg = )
call	_get_SS
! Debug: eq unsigned short = ax+0 to unsigned short ss = [S+$16-$16] (used reg = )
mov	-$14[bp],ax
!BCC_EOS
! 1163   memsetb(0x9ff0, 0x0000, 0, 0xff);
! Debug: list int = const $FF (used reg = )
mov	ax,#$FF
push	ax
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: list unsigned int = const $9FF0 (used reg = )
mov	ax,#$9FF0
push	ax
! Debug: func () void = memsetb+0 (used reg = )
call	_memsetb
add	sp,*8
!BCC_EOS
! 1164   e.type = 1; e.flags = 0; e.vector = 0; e.description = 0; e.reserved = 0;
! Debug: eq int = const 1 to unsigned short e = [S+$16-$12] (used reg = )
mov	ax,*1
mov	-$10[bp],ax
!BCC_EOS
! Debug: eq int = const 0 to unsigned short e = [S+$16-$10] (used reg = )
xor	ax,ax
mov	-$E[bp],ax
!BCC_EOS
! Debug: eq int = const 0 to unsigned long e = [S+$16-$E] (used reg = )
xor	ax,ax
xor	bx,bx
mov	-$C[bp],ax
mov	-$A[bp],bx
!BCC_EOS
! Debug: eq int = const 0 to unsigned long e = [S+$16-$A] (used reg = )
xor	ax,ax
xor	bx,bx
mov	-8[bp],ax
mov	-6[bp],bx
!BCC_EOS
! Debug: eq int = const 0 to unsigned long e = [S+$16-6] (used reg = )
xor	ax,ax
xor	bx,bx
mov	-4[bp],ax
mov	-2[bp],bx
!BCC_EOS
! 1165   memcpyb(0x9ff0, 0x0000 + count * sizeof (e
! 1165 ), ss, &e, sizeof (e));
! Debug: list int = const $10 (used reg = )
mov	ax,*$10
push	ax
! Debug: list * struct ipl_entry e = S+$18-$12 (used reg = )
lea	bx,-$10[bp]
push	bx
! Debug: list unsigned short ss = [S+$1A-$16] (used reg = )
push	-$14[bp]
! Debug: mul int = const $10 to unsigned short count = [S+$1C-$14] (used reg = )
mov	ax,-$12[bp]
mov	cl,*4
shl	ax,cl
! Debug: add unsigned int = ax+0 to int = const 0 (used reg = )
! Debug: expression subtree swapping
! Debug: list unsigned int = ax+0 (used reg = )
push	ax
! Debug: list unsigned int = const $9FF0 (used reg = )
mov	ax,#$9FF0
push	ax
! Debug: func () void = memcpyb+0 (used reg = )
call	_memcpyb
add	sp,*$A
!BCC_EOS
! 1166   count++;
! Debug: postinc unsigned short count = [S+$16-$14] (used reg = )
mov	ax,-$12[bp]
inc	ax
mov	-$12[bp],ax
!BCC_EOS
! 1167   e.type = 2; e.flags = 0; e.vector = 0; e.description = 0; e.reserved = 0;
! Debug: eq int = const 2 to unsigned short e = [S+$16-$12] (used reg = )
mov	ax,*2
mov	-$10[bp],ax
!BCC_EOS
! Debug: eq int = const 0 to unsigned short e = [S+$16-$10] (used reg = )
xor	ax,ax
mov	-$E[bp],ax
!BCC_EOS
! Debug: eq int = const 0 to unsigned long e = [S+$16-$E] (used reg = )
xor	ax,ax
xor	bx,bx
mov	-$C[bp],ax
mov	-$A[bp],bx
!BCC_EOS
! Debug: eq int = const 0 to unsigned long e = [S+$16-$A] (used reg = )
xor	ax,ax
xor	bx,bx
mov	-8[bp],ax
mov	-6[bp],bx
!BCC_EOS
! Debug: eq int = const 0 to unsigned long e = [S+$16-6] (used reg = )
xor	ax,ax
xor	bx,bx
mov	-4[bp],ax
mov	-2[bp],bx
!BCC_EOS
! 1168   memcpyb(0x9ff0, 0x0000 + count * sizeof (e), ss, &e, sizeof (e));
! Debug: list int = const $10 (used reg = )
mov	ax,*$10
push	ax
! Debug: list * struct ipl_entry e = S+$18-$12 (used reg = )
lea	bx,-$10[bp]
push	bx
! Debug: list unsigned short ss = [S+$1A-$16] (used reg = )
push	-$14[bp]
! Debug: mul int = const $10 to unsigned short count = [S+$1C-$14] (used reg = )
mov	ax,-$12[bp]
mov	cl,*4
shl	ax,cl
! Debug: add unsigned int = ax+0 to int = const 0 (used reg = )
! Debug: expression subtree swapping
! Debug: list unsigned int = ax+0 (used reg = )
push	ax
! Debug: list unsigned int = const $9FF0 (used reg = )
mov	ax,#$9FF0
push	ax
! Debug: func () void = memcpyb+0 (used reg = )
call	_memcpyb
add	sp,*$A
!BCC_EOS
! 1169   count++;
! Debug: postinc unsigned short count = [S+$16-$14] (used reg = )
mov	ax,-$12[bp]
inc	ax
mov	-$12[bp],ax
!BCC_EOS
! 1170   e.type = 3; e.flags = 0; e.vector = 0; e.description = 0; e.reserved = 0;
! Debug: eq int = const 3 to unsigned short e = [S+$16-$12] (used reg = )
mov	ax,*3
mov	-$10[bp],ax
!BCC_EOS
! Debug: eq int = const 0 to unsigned short e = [S+$16-$10] (used reg = )
xor	ax,ax
mov	-$E[bp],ax
!BCC_EOS
! Debug: eq int = const 0 to unsigned long e = [S+$16-$E] (used reg = )
xor	ax,ax
xor	bx,bx
mov	-$C[bp],ax
mov	-$A[bp],bx
!BCC_EOS
! Debug: eq int = const 0 to unsigned long e = [S+$16-$A] (used reg = )
xor	ax,ax
xor	bx,bx
mov	-8[bp],ax
mov	-6[bp],bx
!BCC_EOS
! Debug: eq int = const 0 to unsigned long e = [S+$16-6] (used reg = )
xor	ax,ax
xor	bx,bx
mov	-4[bp],ax
mov	-2[bp],bx
!BCC_EOS
! 1171   memcpyb(0x9ff0, 0x0000 + count * sizeof (e), ss, &e, sizeof (e));
! Debug: list int = const $10 (used reg = )
mov	ax,*$10
push	ax
! Debug: list * struct ipl_entry e = S+$18-$12 (used reg = )
lea	bx,-$10[bp]
push	bx
! Debug: list unsigned short ss = [S+$1A-$16] (used reg = )
push	-$14[bp]
! Debug: mul int = const $10 to unsigned short count = [S+$1C-$14] (used reg = )
mov	ax,-$12[bp]
mov	cl,*4
shl	ax,cl
! Debug: add unsigned int = ax+0 to int = const 0 (used reg = )
! Debug: expression subtree swapping
! Debug: list unsigned int = ax+0 (used reg = )
push	ax
! Debug: list unsigned int = const $9FF0 (used reg = )
mov	ax,#$9FF0
push	ax
! Debug: func () void = memcpyb+0 (used reg = )
call	_memcpyb
add	sp,*$A
!BCC_EOS
! 1172   count++;
! Debug: postinc unsigned short count = [S+$16-$14] (used reg = )
mov	ax,-$12[bp]
inc	ax
mov	-$12[bp],ax
!BCC_EOS
! 1173   write_word(0x9ff0, 0x0080, count);
! Debug: list unsigned short count = [S+$16-$14] (used reg = )
push	-$12[bp]
! Debug: list int = const $80 (used reg = )
mov	ax,#$80
push	ax
! Debug: list unsigned int = const $9FF0 (used reg = )
mov	ax,#$9FF0
push	ax
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
add	sp,*6
!BCC_EOS
! 1174   write_word(0x9ff0, 0x0082, 0xffff);
! Debug: list unsigned int = const $FFFF (used reg = )
mov	ax,#$FFFF
push	ax
! Debug: list int = const $82 (used reg = )
mov	ax,#$82
push	ax
! Debug: list unsigned int = const $9FF0 (used reg = )
mov	ax,#$9FF0
push	ax
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
add	sp,*6
!BCC_EOS
! 1175 }
mov	sp,bp
pop	bp
ret
! 1176 static Bit8u
! Register BX used in function init_boot_vectors
! 1177 get_boot_vector(i, e)
! 1178 Bit16u i; struct ipl_entry *e;
_get_boot_vector:
!BCC_EOS
!BCC_EOS
! 1179 {
! 1180   Bit16u count;
!BCC_EOS
! 1181   Bit16u ss = get_SS();
push	bp
mov	bp,sp
add	sp,*-4
! Debug: func () unsigned short = get_SS+0 (used reg = )
call	_get_SS
! Debug: eq unsigned short = ax+0 to unsigned short ss = [S+6-6] (used reg = )
mov	-4[bp],ax
!BCC_EOS
! 1182   count = read_word(0x9ff0, 0x0080);
! Debug: list int = const $80 (used reg = )
mov	ax,#$80
push	ax
! Debug: list unsigned int = const $9FF0 (used reg = )
mov	ax,#$9FF0
push	ax
! Debug: func () unsigned short = read_word+0 (used reg = )
call	_read_word
add	sp,*4
! Debug: eq unsigned short = ax+0 to unsigned short count = [S+6-4] (used reg = )
mov	-2[bp],ax
!BCC_EOS
! 1183   if (i >= count) return 0;
! Debug: ge unsigned short count = [S+6-4] to unsigned short i = [S+6+2] (used reg = )
mov	ax,4[bp]
cmp	ax,-2[bp]
jb  	.D1
.D2:
xor	al,al
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 1184   memcpyb(ss, e, 0x9ff0, 0x0000 + i * sizeof (*e), sizeof (*e));
.D1:
! Debug: list int = const $10 (used reg = )
mov	ax,*$10
push	ax
! Debug: mul int = const $10 to unsigned short i = [S+8+2] (used reg = )
mov	ax,4[bp]
mov	cl,*4
shl	ax,cl
! Debug: add unsigned int = ax+0 to int = const 0 (used reg = )
! Debug: expression subtree swapping
! Debug: list unsigned int = ax+0 (used reg = )
push	ax
! Debug: list unsigned int = const $9FF0 (used reg = )
mov	ax,#$9FF0
push	ax
! Debug: list * struct ipl_entry e = [S+$C+4] (used reg = )
push	6[bp]
! Debug: list unsigned short ss = [S+$E-6] (used reg = )
push	-4[bp]
! Debug: func () void = memcpyb+0 (used reg = )
call	_memcpyb
add	sp,*$A
!BCC_EOS
! 1185   return 1;
mov	al,*1
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 1186 }
! 1187 static char drivetypes[][10]={"", "Floppy","Hard Disk","CD-Rom", "Network"};
.data
_drivetypes:
.D3:
.byte	0
.blkb	9
.D4:
.ascii	"Floppy"
.byte	0
.blkb	3
.D5:
.ascii	"Hard Disk"
.byte	0
.D6:
.ascii	"CD-Rom"
.byte	0
.blkb	3
.D7:
.ascii	"Network"
.byte	0
.blkb	2
!BCC_EOS
! 1188 void
! 1189 print_boot_device(type)
! 1190   Bit16u type;
.text
export	_print_boot_device
_print_boot_device:
!BCC_EOS
! 1191 {
! 1192   if (type == 0x80 ) type = 0x4;
push	bp
mov	bp,sp
! Debug: logeq int = const $80 to unsigned short type = [S+2+2] (used reg = )
mov	ax,4[bp]
cmp	ax,#$80
jne 	.D8
.D9:
! Debug: eq int = const 4 to unsigned short type = [S+2+2] (used reg = )
mov	ax,*4
mov	4[bp],ax
!BCC_EOS
! 1193   if (type == 0 || type > 0x4) bios_printf((2 | 4 | 1), "Bad drive type\n");
.D8:
! Debug: logeq int = const 0 to unsigned short type = [S+2+2] (used reg = )
mov	ax,4[bp]
test	ax,ax
je  	.DB
.DC:
! Debug: gt int = const 4 to unsigned short type = [S+2+2] (used reg = )
mov	ax,4[bp]
cmp	ax,*4
jbe 	.DA
.DB:
! Debug: list * char = .DD+0 (used reg = )
mov	bx,#.DD
push	bx
! Debug: list int = const 7 (used reg = )
mov	ax,*7
push	ax
! Debug: func () void = bios_printf+0 (used reg = )
call	_bios_printf
mov	sp,bp
!BCC_EOS
! 1194   bios_printf(2, "Booting from %s...\n", drivetypes[type]);
.DA:
! Debug: ptradd unsigned short type = [S+2+2] to [5] [$A] char = drivetypes+0 (used reg = )
mov	bx,4[bp]
mov	dx,bx
shl	bx,*1
shl	bx,*1
add	bx,dx
shl	bx,*1
! Debug: cast * char = const 0 to [$A] char = bx+_drivetypes+0 (used reg = )
! Debug: list * char = bx+_drivetypes+0 (used reg = )
add	bx,#_drivetypes
push	bx
! Debug: list * char = .DE+0 (used reg = )
mov	bx,#.DE
push	bx
! Debug: list int = const 2 (used reg = )
mov	ax,*2
push	ax
! Debug: func () void = bios_printf+0 (used reg = )
call	_bios_printf
mov	sp,bp
!BCC_EOS
! 1195 }
pop	bp
ret
! 1196   void
! Register BX used in function print_boot_device
! 1197 print_boot_failure(type, reason)
! 1198   Bit16u type; Bit8u reason;
export	_print_boot_failure
_print_boot_failure:
!BCC_EOS
!BCC_EOS
! 1199 {
! 1200   if (type == 0 || type > 0x3) bios_printf((2 | 4 | 1), "Bad drive type\n");
push	bp
mov	bp,sp
! Debug: logeq int = const 0 to unsigned short type = [S+2+2] (used reg = )
mov	ax,4[bp]
test	ax,ax
je  	.E0
.E1:
! Debug: gt int = const 3 to unsigned short type = [S+2+2] (used reg = )
mov	ax,4[bp]
cmp	ax,*3
jbe 	.DF
.E0:
! Debug: list * char = .E2+0 (used reg = )
mov	bx,#.E2
push	bx
! Debug: list int = const 7 (used reg = )
mov	ax,*7
push	ax
! Debug: func () void = bios_printf+0 (used reg = )
call	_bios_printf
mov	sp,bp
!BCC_EOS
! 1201   bios_printf(2, "Boot from %s failed", drivetypes[type]);
.DF:
! Debug: ptradd unsigned short type = [S+2+2] to [5] [$A] char = drivetypes+0 (used reg = )
mov	bx,4[bp]
mov	dx,bx
shl	bx,*1
shl	bx,*1
add	bx,dx
shl	bx,*1
! Debug: cast * char = const 0 to [$A] char = bx+_drivetypes+0 (used reg = )
! Debug: list * char = bx+_drivetypes+0 (used reg = )
add	bx,#_drivetypes
push	bx
! Debug: list * char = .E3+0 (used reg = )
mov	bx,#.E3
push	bx
! Debug: list int = const 2 (used reg = )
mov	ax,*2
push	ax
! Debug: func () void = bios_printf+0 (used reg = )
call	_bios_printf
mov	sp,bp
!BCC_EOS
! 1202   if (type < 4) {
! Debug: lt int = const 4 to unsigned short type = [S+2+2] (used reg = )
mov	ax,4[bp]
cmp	ax,*4
jae 	.E4
.E5:
! 1203   if (reason==0)
! Debug: logeq int = const 0 to unsigned char reason = [S+2+4] (used reg = )
mov	al,6[bp]
test	al,al
jne 	.E6
.E7:
! 1204     bios_printf(2, ": not a bootable disk");
! Debug: list * char = .E8+0 (used reg = )
mov	bx,#.E8
push	bx
! Debug: list int = const 2 (used reg = )
mov	ax,*2
push	ax
! Debug: func () void = bios_printf+0 (used reg = )
call	_bios_printf
mov	sp,bp
!BCC_EOS
! 1205   else
! 1206     bios_printf(2, ": could not read the boot disk");
jmp .E9
.E6:
! Debug: list * char = .EA+0 (used reg = )
mov	bx,#.EA
push	bx
! Debug: list int = const 2 (used reg = )
mov	ax,*2
push	ax
! Debug: func () void = bios_printf+0 (used reg = )
call	_bios_printf
mov	sp,bp
!BCC_EOS
! 1207   }
.E9:
! 1208   bios_printf(2, "\n");
.E4:
! Debug: list * char = .EB+0 (used reg = )
mov	bx,#.EB
push	bx
! Debug: list int = const 2 (used reg = )
mov	ax,*2
push	ax
! Debug: func () void = bios_printf+0 (used reg = )
call	_bios_printf
mov	sp,bp
!BCC_EOS
! 1209 }
pop	bp
ret
! 1210   void
! Register BX used in function print_boot_failure
! 1211 print_cdromboot_failure( code )
! 1212   Bit16u code;
export	_print_cdromboot_failure
_print_cdromboot_failure:
!BCC_EOS
! 1213 {
! 1214   bios_printf(2 | 4, "CDROM boot failure code : %04x\n",code);
push	bp
mov	bp,sp
! Debug: list unsigned short code = [S+2+2] (used reg = )
push	4[bp]
! Debug: list * char = .EC+0 (used reg = )
mov	bx,#.EC
push	bx
! Debug: list int = const 6 (used reg = )
mov	ax,*6
push	ax
! Debug: func () void = bios_printf+0 (used reg = )
call	_bios_printf
mov	sp,bp
!BCC_EOS
! 1215   return;
pop	bp
ret
!BCC_EOS
! 1216 }
! 1217 Bit8u check_for_keystroke()
! Register BX used in function print_cdromboot_failure
! 1218 {
export	_check_for_keystroke
_check_for_keystroke:
! 1219 #asm
!BCC_ASM
    mov ax, #0x100
    int #0x16
    jz no_key
    mov al, #1
    jmp done
no_key:
    xor al, al
done:
! 1228 endasm
!BCC_ENDASM
! 1229 }
ret
! 1230 Bit8u get_keystroke()
! 1231 {
export	_get_keystroke
_get_keystroke:
! 1232 #asm
!BCC_ASM
    mov ax, #0x0
    int #0x16
    xchg ah, al
! 1236 endasm
!BCC_ENDASM
! 1237 }
ret
! 1238 Bit8u wait(ticks, stop_on_key)
! 1239   Bit16u ticks;
export	_wait
_wait:
!BCC_EOS
! 1240   Bit8u stop_on_key;
!BCC_EOS
! 1241 {
! 1242     long ticks_to_wait, delta;
!BCC_EOS
! 1243     Bit32u prev_ticks, t;
!BCC_EOS
! 1244     Bit8u scan_code = 0;
push	bp
mov	bp,sp
add	sp,*-$11
! Debug: eq int = const 0 to unsigned char scan_code = [S+$13-$13] (used reg = )
xor	al,al
mov	-$11[bp],al
!BCC_EOS
! 1245     ticks_to_wait = ticks;
dec	sp
! Debug: eq unsigned short ticks = [S+$14+2] to long ticks_to_wait = [S+$14-6] (used reg = )
mov	ax,4[bp]
xor	bx,bx
mov	-4[bp],ax
mov	-2[bp],bx
!BCC_EOS
! 1246     prev_ticks = read_dword(0x0, 0x46c);
! Debug: list int = const $46C (used reg = )
mov	ax,#$46C
push	ax
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: func () unsigned long = read_dword+0 (used reg = )
call	_read_dword
mov	bx,dx
add	sp,*4
! Debug: eq unsigned long = bx+0 to unsigned long prev_ticks = [S+$14-$E] (used reg = )
mov	-$C[bp],ax
mov	-$A[bp],bx
!BCC_EOS
! 1247     do
! 1248     {
.EF:
! 1249         t = read_dword(0x0, 0x46c);
! Debug: list int = const $46C (used reg = )
mov	ax,#$46C
push	ax
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: func () unsigned long = read_dword+0 (used reg = )
call	_read_dword
mov	bx,dx
add	sp,*4
! Debug: eq unsigned long = bx+0 to unsigned long t = [S+$14-$12] (used reg = )
mov	-$10[bp],ax
mov	-$E[bp],bx
!BCC_EOS
! 1250         if (t > prev_ticks)
! Debug: gt unsigned long prev_ticks = [S+$14-$E] to unsigned long t = [S+$14-$12] (used reg = )
mov	ax,-$C[bp]
mov	bx,-$A[bp]
lea	di,-$10[bp]
call	lcmpul
jae 	.F0
.F1:
! 1251         {
! 1252             delta = t - prev
! 1252 _ticks;
! Debug: sub unsigned long prev_ticks = [S+$14-$E] to unsigned long t = [S+$14-$12] (used reg = )
mov	ax,-$10[bp]
mov	bx,-$E[bp]
lea	di,-$C[bp]
call	lsubul
! Debug: eq unsigned long = bx+0 to long delta = [S+$14-$A] (used reg = )
mov	-8[bp],ax
mov	-6[bp],bx
!BCC_EOS
! 1253             ticks_to_wait -= delta;
! Debug: subab long delta = [S+$14-$A] to long ticks_to_wait = [S+$14-6] (used reg = )
mov	ax,-4[bp]
mov	bx,-2[bp]
lea	di,-8[bp]
call	lsubl
mov	-4[bp],ax
mov	-2[bp],bx
!BCC_EOS
! 1254         }
! 1255         else if (t < prev_ticks)
jmp .F2
.F0:
! Debug: lt unsigned long prev_ticks = [S+$14-$E] to unsigned long t = [S+$14-$12] (used reg = )
mov	ax,-$C[bp]
mov	bx,-$A[bp]
lea	di,-$10[bp]
call	lcmpul
jbe 	.F3
.F4:
! 1256             ticks_to_wait -= t;
! Debug: subab unsigned long t = [S+$14-$12] to long ticks_to_wait = [S+$14-6] (used reg = )
mov	ax,-4[bp]
mov	bx,-2[bp]
lea	di,-$10[bp]
call	lsubul
mov	-4[bp],ax
mov	-2[bp],bx
!BCC_EOS
! 1257         prev_ticks = t;
.F3:
.F2:
! Debug: eq unsigned long t = [S+$14-$12] to unsigned long prev_ticks = [S+$14-$E] (used reg = )
mov	ax,-$10[bp]
mov	bx,-$E[bp]
mov	-$C[bp],ax
mov	-$A[bp],bx
!BCC_EOS
! 1258         if (check_for_keystroke())
! Debug: func () unsigned char = check_for_keystroke+0 (used reg = )
call	_check_for_keystroke
test	al,al
je  	.F5
.F6:
! 1259         {
! 1260             scan_code = get_keystroke();
! Debug: func () unsigned char = get_keystroke+0 (used reg = )
call	_get_keystroke
! Debug: eq unsigned char = al+0 to unsigned char scan_code = [S+$14-$13] (used reg = )
mov	-$11[bp],al
!BCC_EOS
! 1261             bios_printf(8, "Key pressed: %x\n", scan_code);
! Debug: list unsigned char scan_code = [S+$14-$13] (used reg = )
mov	al,-$11[bp]
xor	ah,ah
push	ax
! Debug: list * char = .F7+0 (used reg = )
mov	bx,#.F7
push	bx
! Debug: list int = const 8 (used reg = )
mov	ax,*8
push	ax
! Debug: func () void = bios_printf+0 (used reg = )
call	_bios_printf
add	sp,*6
!BCC_EOS
! 1262             if (stop_on_key)
mov	al,6[bp]
test	al,al
je  	.F8
.F9:
! 1263                 return scan_code;
mov	al,-$11[bp]
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 1264         }
.F8:
! 1265     } while (ticks_to_wait > 0);
.F5:
.EE:
! Debug: gt long = const 0 to long ticks_to_wait = [S+$14-6] (used reg = )
xor	ax,ax
xor	bx,bx
lea	di,-4[bp]
call	lcmpl
blt 	.EF
.FA:
!BCC_EOS
! 1266     return scan_code;
.ED:
mov	al,-$11[bp]
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 1267 }
! 1268 static void clearscreen() {
! Register BX used in function wait
_clearscreen:
! 1269 #asm
!BCC_ASM
        push bx
        push cx
        push dx
        mov ax, #0x100
        mov cx, #0x1000
        int #0x10
        mov ax, #0x700
        mov bh, #7
        xor cx, cx
        mov dx, #0x184f
        int #0x10
        mov ax, #0x200
        xor bx, bx
        xor dx, dx
        int #0x10
        pop dx
        pop cx
        pop bx
! 1288 endasm
!BCC_ENDASM
! 1289 }
ret
! 1290 int bootmenu(selected)
! 1291   int selected;
export	_bootmenu
_bootmenu:
!BCC_EOS
! 1292 {
! 1293     Bit8u scode;
!BCC_EOS
! 1294     int max;
!BCC_EOS
! 1295     max = read_word(0x9ff0, 0x0080);
push	bp
mov	bp,sp
add	sp,*-4
! Debug: list int = const $80 (used reg = )
mov	ax,#$80
push	ax
! Debug: list unsigned int = const $9FF0 (used reg = )
mov	ax,#$9FF0
push	ax
! Debug: func () unsigned short = read_word+0 (used reg = )
call	_read_word
add	sp,*4
! Debug: eq unsigned short = ax+0 to int max = [S+6-6] (used reg = )
mov	-4[bp],ax
!BCC_EOS
! 1296     for(;;) {
!BCC_EOS
!BCC_EOS
.FD:
! 1297         if (selected > max || selected < 1) selected = 1;
! Debug: gt int max = [S+6-6] to int selected = [S+6+2] (used reg = )
mov	ax,4[bp]
cmp	ax,-4[bp]
jg  	.FF
.100:
! Debug: lt int = const 1 to int selected = [S+6+2] (used reg = )
mov	ax,4[bp]
cmp	ax,*1
jge 	.FE
.FF:
! Debug: eq int = const 1 to int selected = [S+6+2] (used reg = )
mov	ax,*1
mov	4[bp],ax
!BCC_EOS
! 1298         clearscreen();
.FE:
! Debug: func () void = clearscreen+0 (used reg = )
call	_clearscreen
!BCC_EOS
! 1299         bios_printf(2 | 4, "\n\n\n\n\n\n\n");
! Debug: list * char = .101+0 (used reg = )
mov	bx,#.101
push	bx
! Debug: list int = const 6 (used reg = )
mov	ax,*6
push	ax
! Debug: func () void = bios_printf+0 (used reg = )
call	_bios_printf
add	sp,*4
!BCC_EOS
! 1300         bios_printf(2 | 4, "          Select boot device\n\n");
! Debug: list * char = .102+0 (used reg = )
mov	bx,#.102
push	bx
! Debug: list int = const 6 (used reg = )
mov	ax,*6
push	ax
! Debug: func () void = bios_printf+0 (used reg = )
call	_bios_printf
add	sp,*4
!BCC_EOS
! 1301         bios_printf(2 | 4, "            1. Floppy\n");
! Debug: list * char = .103+0 (used reg = )
mov	bx,#.103
push	bx
! Debug: list int = const 6 (used reg = )
mov	ax,*6
push	ax
! Debug: func () void = bios_printf+0 (used reg = )
call	_bios_printf
add	sp,*4
!BCC_EOS
! 1302         bios_printf(2 | 4, "            2. Hard drive\n");
! Debug: list * char = .104+0 (used reg = )
mov	bx,#.104
push	bx
! Debug: list int = const 6 (used reg = )
mov	ax,*6
push	ax
! Debug: func () void = bios_printf+0 (used reg = )
call	_bios_printf
add	sp,*4
!BCC_EOS
! 1303         bios_printf(2 | 4, "            3. CD-ROM\n");
! Debug: list * char = .105+0 (used reg = )
mov	bx,#.105
push	bx
! Debug: list int = const 6 (used reg = )
mov	ax,*6
push	ax
! Debug: func () void = bios_printf+0 (used reg = )
call	_bios_printf
add	sp,*4
!BCC_EOS
! 1304         if (max == 4)
! Debug: logeq int = const 4 to int max = [S+6-6] (used reg = )
mov	ax,-4[bp]
cmp	ax,*4
jne 	.106
.107:
! 1305             bios_printf(2 | 4, "            4. Network\n");
! Debug: list * char = .108+0 (used reg = )
mov	bx,#.108
push	bx
! Debug: list int = const 6 (used reg = )
mov	ax,*6
push	ax
! Debug: func () void = bios_printf+0 (used reg = )
call	_bios_printf
add	sp,*4
!BCC_EOS
! 1306         bios_printf(2 | 4, "\n\n          Currently selected: %d\n", selected);
.106:
! Debug: list int selected = [S+6+2] (used reg = )
push	4[bp]
! Debug: list * char = .109+0 (used reg = )
mov	bx,#.109
push	bx
! Debug: list int = const 6 (used reg = )
mov	ax,*6
push	ax
! Debug: func () void = bios_printf+0 (used reg = )
call	_bios_printf
add	sp,*6
!BCC_EOS
! 1307         do {
.10C:
! 1308             scode = wait(18, 1);
! Debug: list int = const 1 (used reg = )
mov	ax,*1
push	ax
! Debug: list int = const $12 (used reg = )
mov	ax,*$12
push	ax
! Debug: func () unsigned char = wait+0 (used reg = )
call	_wait
add	sp,*4
! Debug: eq unsigned char = al+0 to unsigned char scode = [S+6-3] (used reg = )
mov	-1[bp],al
!BCC_EOS
! 1309         } while (scode == 0);
.10B:
! Debug: logeq int = const 0 to unsigned char scode = [S+6-3] (used reg = )
mov	al,-1[bp]
test	al,al
je 	.10C
.10D:
!BCC_EOS
! 1310         switch(scode) {
.10A:
mov	al,-1[bp]
jmp .110
! 1311         case 0x02:
! 1312         case 0x03:
.111:
! 1313         case 0x04:
.112:
! 1314             selected = scode - 1;
.113:
! Debug: sub int = const 1 to unsigned char scode = [S+6-3] (used reg = )
mov	al,-1[bp]
xor	ah,ah
! Debug: eq unsigned int = ax-1 to int selected = [S+6+2] (used reg = )
dec	ax
mov	4[bp],ax
!BCC_EOS
! 1315             break;
jmp .10E
!BCC_EOS
! 1316         case 0x05:
! 1317             if (max == 4)
.114:
! Debug: logeq int = const 4 to int max = [S+6-6] (used reg = )
mov	ax,-4[bp]
cmp	ax,*4
jne 	.115
.116:
! 1318                 selected = scode -1 ;
! Debug: sub int = const 1 to unsigned char scode = [S+6-3] (used reg = )
mov	al,-1[bp]
xor	ah,ah
! Debug: eq unsigned int = ax-1 to int selected = [S+6+2] (used reg = )
dec	ax
mov	4[bp],ax
!BCC_EOS
! 1319             else
! 1320                 scode = 0;
jmp .117
.115:
! Debug: eq int = const 0 to unsigned char scode = [S+6-3] (used reg = )
xor	al,al
mov	-1[bp],al
!BCC_EOS
! 1321             break;
.117:
jmp .10E
!BCC_EOS
! 1322         case 0x48:
! 1323             selected -= 1;
.118:
! Debug: subab int = const 1 to int selected = [S+6+2] (used reg = )
mov	ax,4[bp]
dec	ax
mov	4[bp],ax
!BCC_EOS
! 1324             if (selected < 1)
! Debug: lt int = const 1 to int selected = [S+6+2] (used reg = )
mov	ax,4[bp]
cmp	ax,*1
jge 	.119
.11A:
! 1325                 selected = 1;
! Debug: eq int = const 1 to int selected = [S+6+2] (used reg = )
mov	ax,*1
mov	4[bp],ax
!BCC_EOS
! 1326             scode = 0;
.119:
! Debug: eq int = const 0 to unsigned char scode = [S+6-3] (used reg = )
xor	al,al
mov	-1[bp],al
!BCC_EOS
! 1327             break;
jmp .10E
!BCC_EOS
! 1328         case 0x50:
! 1329             selected += 1;
.11B:
! Debug: addab int = const 1 to int selected = [S+6+2] (used reg = )
mov	ax,4[bp]
inc	ax
mov	4[bp],ax
!BCC_EOS
! 1330             if (selected > max)
! Debug: gt int max = [S+6-6] to int selected = [S+6+2] (used reg = )
mov	ax,4[bp]
cmp	ax,-4[bp]
jle 	.11C
.11D:
! 1331                 sele
! 1331 cted = max;
! Debug: eq int max = [S+6-6] to int selected = [S+6+2] (used reg = )
mov	ax,-4[bp]
mov	4[bp],ax
!BCC_EOS
! 1332             scode = 0;
.11C:
! Debug: eq int = const 0 to unsigned char scode = [S+6-3] (used reg = )
xor	al,al
mov	-1[bp],al
!BCC_EOS
! 1333             break;
jmp .10E
!BCC_EOS
! 1334         case 0x1c:
! 1335             break;
.11E:
jmp .10E
!BCC_EOS
! 1336         default:
! 1337             scode = 0;
.11F:
! Debug: eq int = const 0 to unsigned char scode = [S+6-3] (used reg = )
xor	al,al
mov	-1[bp],al
!BCC_EOS
! 1338             break;
jmp .10E
!BCC_EOS
! 1339         }
! 1340         if (scode != 0)
jmp .10E
.110:
sub	al,*2
je 	.111
sub	al,*1
je 	.112
sub	al,*1
je 	.113
sub	al,*1
je 	.114
sub	al,*$17
je 	.11E
sub	al,*$2C
je 	.118
sub	al,*8
je 	.11B
jmp	.11F
.10E:
..FFFF	=	-6
! Debug: ne int = const 0 to unsigned char scode = [S+6-3] (used reg = )
mov	al,-1[bp]
test	al,al
je  	.120
.121:
! 1341             break;
jmp .FB
!BCC_EOS
! 1342     }
.120:
! 1343     switch (selected) {
.FC:
br 	.FD
.FB:
mov	ax,4[bp]
jmp .124
! 1344     case 1:
! 1345         return 0x3D;
.125:
mov	ax,*$3D
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 1346     case 2:
! 1347         return 0x3E;
.126:
mov	ax,*$3E
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 1348     case 3:
! 1349         return 0x3F;
.127:
mov	ax,*$3F
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 1350     case 4:
! 1351         return 0x58;
.128:
mov	ax,*$58
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 1352     default:
! 1353         return 0;
.129:
xor	ax,ax
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 1354     }
! 1355 }
jmp .122
.124:
sub	ax,*1
je 	.125
sub	ax,*1
je 	.126
sub	ax,*1
je 	.127
sub	ax,*1
je 	.128
jmp	.129
.122:
..FFFE	=	-6
mov	sp,bp
pop	bp
ret
! 1356 void interactive_bootkey()
! Register BX used in function bootmenu
! 1357 {
export	_interactive_bootkey
_interactive_bootkey:
! 1358     Bit16u i;
!BCC_EOS
! 1359     Bit8u scan = 0;
push	bp
mov	bp,sp
add	sp,*-3
! Debug: eq int = const 0 to unsigned char scan = [S+5-5] (used reg = )
xor	al,al
mov	-3[bp],al
!BCC_EOS
! 1360     bios_printf(2 | 4,
dec	sp
! 1361                 "\n\nPress F10 to select boot device.\n");
! Debug: list * char = .12A+0 (used reg = )
mov	bx,#.12A
push	bx
! Debug: list int = const 6 (used reg = )
mov	ax,*6
push	ax
! Debug: func () void = bios_printf+0 (used reg = )
call	_bios_printf
add	sp,*4
!BCC_EOS
! 1362     scan = wait(1, 0);
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: list int = const 1 (used reg = )
mov	ax,*1
push	ax
! Debug: func () unsigned char = wait+0 (used reg = )
call	_wait
add	sp,*4
! Debug: eq unsigned char = al+0 to unsigned char scan = [S+6-5] (used reg = )
mov	-3[bp],al
!BCC_EOS
! 1363     if (scan == 0x44)
! Debug: logeq int = const $44 to unsigned char scan = [S+6-5] (used reg = )
mov	al,-3[bp]
cmp	al,*$44
jne 	.12B
.12C:
! 1364         scan = bootmenu(inb_cmos(0x3d) & 0x0f);
! Debug: list int = const $3D (used reg = )
mov	ax,*$3D
push	ax
! Debug: func () unsigned char = inb_cmos+0 (used reg = )
call	_inb_cmos
inc	sp
inc	sp
! Debug: and int = const $F to unsigned char = al+0 (used reg = )
and	al,*$F
! Debug: list unsigned char = al+0 (used reg = )
xor	ah,ah
push	ax
! Debug: func () int = bootmenu+0 (used reg = )
call	_bootmenu
inc	sp
inc	sp
! Debug: eq int = ax+0 to unsigned char scan = [S+6-5] (used reg = )
mov	-3[bp],al
!BCC_EOS
! 1365     switch(scan) {
.12B:
mov	al,-3[bp]
jmp .12F
! 1366     case 0x3D:
! 1367         outb_cmos(0x3d, 0x01);
.130:
! Debug: list int = const 1 (used reg = )
mov	ax,*1
push	ax
! Debug: list int = const $3D (used reg = )
mov	ax,*$3D
push	ax
! Debug: func () void = outb_cmos+0 (used reg = )
call	_outb_cmos
add	sp,*4
!BCC_EOS
! 1368         break;
jmp .12D
!BCC_EOS
! 1369     case 0x3E:
! 1370         outb_cmos(0x3d, 0x02);
.131:
! Debug: list int = const 2 (used reg = )
mov	ax,*2
push	ax
! Debug: list int = const $3D (used reg = )
mov	ax,*$3D
push	ax
! Debug: func () void = outb_cmos+0 (used reg = )
call	_outb_cmos
add	sp,*4
!BCC_EOS
! 1371         break;
jmp .12D
!BCC_EOS
! 1372     case 0x3F:
! 1373         outb_cmos(0x3d, 0x03);
.132:
! Debug: list int = const 3 (used reg = )
mov	ax,*3
push	ax
! Debug: list int = const $3D (used reg = )
mov	ax,*$3D
push	ax
! Debug: func () void = outb_cmos+0 (used reg = )
call	_outb_cmos
add	sp,*4
!BCC_EOS
! 1374         break;
jmp .12D
!BCC_EOS
! 1375     case 0x58:
! 1376         outb_cmos(0x3d, 0x04);
.133:
! Debug: list int = const 4 (used reg = )
mov	ax,*4
push	ax
! Debug: list int = const $3D (used reg = )
mov	ax,*$3D
push	ax
! Debug: func () void = outb_cmos+0 (used reg = )
call	_outb_cmos
add	sp,*4
!BCC_EOS
! 1377         break;
jmp .12D
!BCC_EOS
! 1378     default:
! 1379         break;
.134:
jmp .12D
!BCC_EOS
! 1380     }
! 1381 }
jmp .12D
.12F:
sub	al,*$3D
je 	.130
sub	al,*1
je 	.131
sub	al,*1
je 	.132
sub	al,*$19
je 	.133
jmp	.134
.12D:
..FFFD	=	-6
mov	sp,bp
pop	bp
ret
! 1382 void
! Register BX used in function interactive_bootkey
! 1383 nmi_handler_msg()
! 1384 {
export	_nmi_handler_msg
_nmi_handler_msg:
! 1385   bios_printf((2 | 4 | 1), "NMI Handler called\n");
push	bp
mov	bp,sp
! Debug: list * char = .135+0 (used reg = )
mov	bx,#.135
push	bx
! Debug: list int = const 7 (used reg = )
mov	ax,*7
push	ax
! Debug: func () void = bios_printf+0 (used reg = )
call	_bios_printf
mov	sp,bp
!BCC_EOS
! 1386 }
pop	bp
ret
! 1387 void
! Register BX used in function nmi_handler_msg
! 1388 int18_panic_msg()
! 1389 {
export	_int18_panic_msg
_int18_panic_msg:
! 1390   bios_printf((2 | 4 | 1), "INT18: BOOT FAILURE\n");
push	bp
mov	bp,sp
! Debug: list * char = .136+0 (used reg = )
mov	bx,#.136
push	bx
! Debug: list int = const 7 (used reg = )
mov	ax,*7
push	ax
! Debug: func () void = bios_printf+0 (used reg = )
call	_bios_printf
mov	sp,bp
!BCC_EOS
! 1391 }
pop	bp
ret
! 1392 void
! Register BX used in function int18_panic_msg
! 1393 log_bios_start()
! 1394 {
export	_log_bios_start
_log_bios_start:
! 1395   bios_printf(4, "%s\n", (CVSID + 4));
push	bp
mov	bp,sp
! Debug: list * char = CVSID+4 (used reg = )
mov	bx,#_CVSID+4
push	bx
! Debug: list * char = .137+0 (used reg = )
mov	bx,#.137
push	bx
! Debug: list int = const 4 (used reg = )
mov	ax,*4
push	ax
! Debug: func () void = bios_printf+0 (used reg = )
call	_bios_printf
mov	sp,bp
!BCC_EOS
! 1396 }
pop	bp
ret
! 1397   bx_bool
! Register BX used in function log_bios_start
! 1398 set_enable_a20(val)
! 1399   bx_bool val;
export	_set_enable_a20
_set_enable_a20:
!BCC_EOS
! 1400 {
! 1401   Bit8u oldval;
!BCC_EOS
! 1402   oldval = inb(0x92);
push	bp
mov	bp,sp
dec	sp
dec	sp
! Debug: list int = const $92 (used reg = )
mov	ax,#$92
push	ax
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char oldval = [S+4-3] (used reg = )
mov	-1[bp],al
!BCC_EOS
! 1403   if (val)
mov	ax,4[bp]
test	ax,ax
je  	.138
.139:
! 1404     outb(0x92, oldval | 0x02);
! Debug: or int = const 2 to unsigned char oldval = [S+4-3] (used reg = )
mov	al,-1[bp]
or	al,*2
! Debug: list unsigned char = al+0 (used reg = )
xor	ah,ah
push	ax
! Debug: list int = const $92 (used reg = )
mov	ax,#$92
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 1405   else
! 1406     outb(0x92, oldval & 0xfd);
jmp .13A
.138:
! Debug: and int = const $FD to unsigned char oldval = [S+4-3] (used reg = )
mov	al,-1[bp]
and	al,#$FD
! Debug: list unsigned char = al+0 (used reg = )
xor	ah,ah
push	ax
! Debug: list int = const $92 (used reg = )
mov	ax,#$92
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 1407   return((oldval & 0x02) != 0);
.13A:
! Debug: and int = const 2 to unsigned char oldval = [S+4-3] (used reg = )
mov	al,-1[bp]
and	al,*2
! Debug: ne int = const 0 to unsigned char = al+0 (used reg = )
test	al,al
je 	.13B
mov	al,*1
jmp	.13C
.13B:
xor	al,al
.13C:
! Debug: cast unsigned short = const 0 to char = al+0 (used reg = )
xor	ah,ah
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 1408 }
! 1409   void
! 1410 debugger_on()
! 1411 {
export	_debugger_on
_debugger_on:
! 1412   outb(0xfedc, 0x01);
push	bp
mov	bp,sp
! Debug: list int = const 1 (used reg = )
mov	ax,*1
push	ax
! Debug: list unsigned int = const $FEDC (used reg = )
mov	ax,#$FEDC
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
mov	sp,bp
!BCC_EOS
! 1413 }
pop	bp
ret
! 1414   void
! 1415 debugger_off()
! 1416 {
export	_debugger_off
_debugger_off:
! 1417   outb(0xfedc, 0x00);
push	bp
mov	bp,sp
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: list unsigned int = const $FEDC (used reg = )
mov	ax,#$FEDC
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
mov	sp,bp
!BCC_EOS
! 1418 }
pop	bp
ret
! 1419 void
! 1420 s3_resume()
! 1421 {
export	_s3_resume
_s3_resume:
! 1422     Bit32u s3_wakeup_vector;
!BCC_EOS
! 1423     Bit16u s3_wakeup_ip, s3_wakeup_cs;
!BCC_EOS
! 1424     Bit8u cmos_shutdown_status;
!BCC_EOS
! 1425 #asm
push	bp
mov	bp,sp
add	sp,*-$A
!BCC_EOS
!BCC_ASM
_s3_resume.cmos_shutdown_status	set	1
.s3_resume.cmos_shutdown_status	set	-9
_s3_resume.s3_wakeup_ip	set	4
.s3_resume.s3_wakeup_ip	set	-6
_s3_resume.s3_wakeup_vector	set	6
.s3_resume.s3_wakeup_vector	set	-4
_s3_resume.s3_wakeup_cs	set	2
.s3_resume.s3_wakeup_cs	set	-8
    push ds
    push ax
    mov ax, #0x9FC0
    mov ds, ax
    mov al, [1]
    mov .s3_resume.cmos_shutdown_status[bp], al
    pop ax
    pop ds
! 1434 endasm
!BCC_ENDASM
!BCC_EOS
! 1435     if (cmos_shutdown_status != 0xFE)
! Debug: ne int = const $FE to unsigned char cmos_shutdown_status = [S+$C-$B] (used reg = )
mov	al,-9[bp]
cmp	al,#$FE
je  	.13D
.13E:
! 1436         return;
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 1437     s3_wakeup_vector = get_s3_waking_vector();
.13D:
! Debug: func () unsigned long = get_s3_waking_vector+0 (used reg = )
call	_get_s3_waking_vector
mov	bx,dx
! Debug: eq unsigned long = bx+0 to unsigned long s3_wakeup_vector = [S+$C-6] (used reg = )
mov	-4[bp],ax
mov	-2[bp],bx
!BCC_EOS
! 1438     if (!s3_wakeup_vector)
mov	ax,-4[bp]
mov	bx,-2[bp]
call	ltstl
jne 	.13F
.140:
! 1439         return;
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 1440     s3_wakeup_ip = s3_wakeup_vector & 0xF;
.13F:
! Debug: and unsigned long = const $F to unsigned long s3_wakeup_vector = [S+$C-6] (used reg = )
! Debug: expression subtree swapping
mov	ax,*$F
xor	bx,bx
lea	di,-4[bp]
call	landul
! Debug: eq unsigned long = bx+0 to unsigned short s3_wakeup_ip = [S+$C-8] (used reg = )
mov	-6[bp],ax
!BCC_EOS
! 1441     s3_wakeup_cs = s3_wakeup_vector >> 4;
! Debug: sr int = const 4 to unsigned long s3_wakeup_vector = [S+$C-6] (used reg = )
mov	ax,-4[bp]
mov	bx,-2[bp]
mov	di,*4
call	lsrul
! Debug: eq unsigned long = bx+0 to unsigned short s3_wakeup_cs = [S+$C-$A] (used reg = )
mov	-8[bp],ax
!BCC_EOS
! 1442 #asm
!BCC_EOS
!BCC_ASM
_s3_resume.cmos_shutdown_status	set	1
.s3_resume.cmos_shutdown_status	set	-9
_s3_resume.s3_wakeup_ip	set	4
.s3_resume.s3_wakeup_ip	set	-6
_s3_resume.s3_wakeup_vector	set	6
.s3_resume.s3_wakeup_vector	set	-4
_s3_resume.s3_wakeup_cs	set	2
.s3_resume.s3_wakeup_cs	set	-8
    push .s3_resume.s3_wakeup_cs[bp]
    push .s3_resume.s3_wakeup_ip[bp]
    retf
! 1446 endasm
!BCC_ENDASM
!BCC_EOS
! 1447 }
mov	sp,bp
pop	bp
ret
! 1448 void ata_init( )
! Register BX used in function s3_resume
! 1449 {
export	_ata_init
_ata_init:
! 1450   Bit16u ebda_seg=read_word(0x0040,0x000E);
push	bp
mov	bp,sp
dec	sp
dec	sp
! Debug: list int = const $E (used reg = )
mov	ax,*$E
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () unsigned short = read_word+0 (used reg = )
call	_read_word
add	sp,*4
! Debug: eq unsigned short = ax+0 to unsigned short ebda_seg = [S+4-4] (used reg = )
mov	-2[bp],ax
!BCC_EOS
! 1451   Bit8u channel, device;
!BCC_EOS
! 1452   for (channel=0; channel<4; channel++) {
dec	sp
dec	sp
! Debug: eq int = const 0 to unsigned char channel = [S+6-5] (used reg = )
xor	al,al
mov	-3[bp],al
!BCC_EOS
!BCC_EOS
br 	.143
.144:
! 1453     write_byte(ebda_seg,&((ebda_data_t *) 0)->ata.channels[channel].iface,0x00);
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: ptradd unsigned char channel = [S+8-5] to [4] struct  = const $122 (used reg = )
mov	al,-3[bp]
xor	ah,ah
mov	cl,*3
shl	ax,cl
mov	bx,ax
! Debug: address unsigned char = [bx+$122] (used reg = )
! Debug: list * unsigned char = bx+$122 (used reg = )
add	bx,#$122
push	bx
! Debug: list unsigned short ebda_seg = [S+$A-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 1454     write_word(ebda_seg,&((ebda_data_t *) 0)->ata.channels[channel].iobase1,0x0);
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: ptradd unsigned char channel = [S+8-5] to [4] struct  = const $122 (used reg = )
mov	al,-3[bp]
xor	ah,ah
mov	cl,*3
shl	ax,cl
mov	bx,ax
! Debug: address unsigned short = [bx+$124] (used reg = )
! Debug: list * unsigned short = bx+$124 (used reg = )
add	bx,#$124
push	bx
! Debug: list unsigned short ebda_seg = [S+$A-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
add	sp,*6
!BCC_EOS
! 1455     write_word(ebda_seg,&((ebda_data_t *) 0)->ata.channels[channel].iobase2,0x0);
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: ptradd unsigned char channel = [S+8-5] to [4] struct  = const $122 (used reg = )
mov	al,-3[bp]
xor	ah,ah
mov	cl,*3
shl	ax,cl
mov	bx,ax
! Debug: address unsigned short = [bx+$126] (used reg = )
! Debug: list * unsigned short = bx+$126 (used reg = )
add	bx,#$126
push	bx
! Debug: list unsigned short ebda_seg = [S+$A-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
add	sp,*6
!BCC_EOS
! 1456     write_byte(ebda_seg,&((ebda_data_t *) 0)->ata.channels[channel].irq,0);
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: ptradd unsigned char channel = [S+8-5] to [4] struct  = const $122 (used reg = )
mov	al,-3[bp]
xor	ah,ah
mov	cl,*3
shl	ax,cl
mov	bx,ax
! Debug: address unsigned char = [bx+$128] (used reg = )
! Debug: list * unsigned char = bx+$128 (used reg = )
add	bx,#$128
push	bx
! Debug: list unsigned short ebda_seg = [S+$A-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 1457     }
! 1458   for (device=0; device<(4*2); device++) {
.142:
! Debug: postinc unsigned char channel = [S+6-5] (used reg = )
mov	al,-3[bp]
inc	ax
mov	-3[bp],al
.143:
! Debug: lt int = const 4 to unsigned char channel = [S+6-5] (used reg = )
mov	al,-3[bp]
cmp	al,*4
jb 	.144
.145:
.141:
! Debug: eq int = const 0 to unsigned char device = [S+6-6] (used reg = )
xor	al,al
mov	-4[bp],al
!BCC_EOS
!BCC_EOS
br 	.148
.149:
! 1459     write_byte(ebda_seg,&((ebda_data_t *) 0)->ata.devices[device].type,0x00);
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: ptradd unsigned char device = [S+8-6] to [8] struct  = const $142 (used reg = )
mov	al,-4[bp]
xor	ah,ah
mov	cx,*$1A
imul	cx
mov	bx,ax
! Debug: address unsigned char = [bx+$142] (used reg = )
! Debug: list * unsigned char = bx+$142 (used reg = )
add	bx,#$142
push	bx
! Debug: list unsigned short ebda_seg = [S+$A-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 1460     write_byte(ebda_seg,&((ebda_data_t *) 0)->ata.devices[device].device,0x00);
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: ptradd unsigned char device = [S+8-6] to [8] struct  = const $142 (used reg = )
mov	al,-4[bp]
xor	ah,ah
mov	cx,*$1A
imul	cx
mov	bx,ax
! Debug: address unsigned char = [bx+$143] (used reg = )
! Debug: list * unsigned char = bx+$143 (used reg = )
add	bx,#$143
push	bx
! Debug: list unsigned short ebda_seg = [S+$A-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 1461     write_byte(ebda_seg,&((ebda_data_t *) 0)->ata.devices[device].removable,0);
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: ptradd unsigned char device = [S+8-6] to [8] struct  = const $142 (used reg = )
mov	al,-4[bp]
xor	ah,ah
mov	cx,*$1A
imul	cx
mov	bx,ax
! Debug: address unsigned char = [bx+$144] (used reg = )
! Debug: list * unsigned char = bx+$144 (used reg = )
add	bx,#$144
push	bx
! Debug: list unsigned short ebda_seg = [S+$A-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 1462     write_byte(ebda_seg,&((ebda_data_t *) 0)->ata.devices[device].lock,0);
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: ptradd unsigned char device = [S+8-6] to [8] struct  = const $142 (used reg = )
mov	al,-4[bp]
xor	ah,ah
mov	cx,*$1A
imul	cx
mov	bx,ax
! Debug: address unsigned char = [bx+$145] (used reg = )
! Debug: list * unsigned char = bx+$145 (used reg = )
add	bx,#$145
push	bx
! Debug: list unsigned short ebda_seg = [S+$A-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 1463     write_byte(ebda_seg,&((ebda_data_t *) 0)->ata.devices[device].mode,0x00);
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: ptradd unsigned char device = [S+8-6] to [8] struct  = const $142 (used reg = )
mov	al,-4[bp]
xor	ah,ah
mov	cx,*$1A
imul	cx
mov	bx,ax
! Debug: address unsigned char = [bx+$146] (used reg = )
! Debug: list * unsigned char = bx+$146 (used reg = )
add	bx,#$146
push	bx
! Debug: list unsigned short ebda_seg = [S+$A-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 1464     write_word(ebda_seg,&((ebda_data_t *) 0)->ata.devices[device].blksize,0);
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: ptradd unsigned char device = [S+8-6] to [8] struct  = const $142 (used reg = )
mov	al,-4[bp]
xor	ah,ah
mov	cx,*$1A
imul	cx
mov	bx,ax
! Debug: address unsigned short = [bx+$148] (used reg = )
! Debug: list * unsigned short = bx+$148 (used reg = )
add	bx,#$148
push	bx
! Debug: list unsigned short ebda_seg = [S+$A-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
add	sp,*6
!BCC_EOS
! 1465     write_byte(ebda_seg,&((ebda_data_t *) 0)->ata.devices[device].translation,0);
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: ptradd unsigned char device = [S+8-6] to [8] struct  = const $142 (used reg = )
mov	al,-4[bp]
xor	ah,ah
mov	cx,*$1A
imul	cx
mov	bx,ax
! Debug: address unsigned char = [bx+$14A] (used reg = )
! Debug: list * unsigned char = bx+$14A (used reg = )
add	bx,#$14A
push	bx
! Debug: list unsigned short ebda_seg = [S+$A-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 1466     write_word(ebda_seg,&((ebda_data_t *) 0)->ata.devices[device].lchs.heads,0);
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: ptradd unsigned char device = [S+8-6] to [8] struct  = const $142 (used reg = )
mov	al,-4[bp]
xor	ah,ah
mov	cx,*$1A
imul	cx
mov	bx,ax
! Debug: address unsigned short = [bx+$14C] (used reg = )
! Debug: list * unsigned short = bx+$14C (used reg = )
add	bx,#$14C
push	bx
! Debug: list unsigned short ebda_seg = [S+$A-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
add	sp,*6
!BCC_EOS
! 1467     write_word(ebda_seg,&((ebda_data_t *) 0)->ata.devices[device].lchs.cylinders,0);
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: ptradd unsigned char device = [S+8-6] to [8] struct  = const $142 (used reg = )
mov	al,-4[bp]
xor	ah,ah
mov	cx,*$1A
imul	cx
mov	bx,ax
! Debug: address unsigned short = [bx+$14E] (used reg = )
! Debug: list * unsigned short = bx+$14E (used reg = )
add	bx,#$14E
push	bx
! Debug: list unsigned short ebda_seg = [S+$A-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
add	sp,*6
!BCC_EOS
! 1468     write_word(ebda_seg,&((ebda_data_t *) 0)->ata.devices[device].lchs.spt,0);
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: ptradd unsigned char device = [S+8-6] to [8] struct  = const $142 (used reg = )
mov	al,-4[bp]
xor	ah,ah
mov	cx,*$1A
imul	cx
mov	bx,ax
! Debug: address unsigned short = [bx+$150] (used reg = )
! Debug: list * unsigned short = bx+$150 (used reg = )
add	bx,#$150
push	bx
! Debug: list unsigned short ebda_seg = [S+$A-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
add	sp,*6
!BCC_EOS
! 1469     write_word(ebda_seg,&((ebda_data_t *) 0)->ata.devices[device].pchs.heads,0);
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: ptradd unsigned char device = [S+8-6] to [8] struct  = const $142 (used reg = )
mov	al,-4[bp]
xor	ah,ah
mov	cx,*$1A
imul	cx
mov	bx,ax
! Debug: address unsigned short = [bx+$152] (used reg = )
! Debug: list * unsigned short = bx+$152 (used reg = )
add	bx,#$152
push	bx
! Debug: list unsigned short ebda_seg = [S+$A-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
add	sp,*6
!BCC_EOS
! 1470     write_word(ebda_seg,&((ebda_data_t *) 0)->ata.devices[device].pchs.cylinders,0);
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: ptradd unsigned char device = [S+8-6] to [8] struct  = const $142 (used reg = )
mov	al,-4[bp]
xor	ah,ah
mov	cx,*$1A
imul	cx
mov	bx,ax
! Debug: address unsigned short = [bx+$154] (used reg = )
! Debug: list * unsigned short = bx+$154 (used reg = )
add	bx,#$154
push	bx
! Debug: list unsigned short ebda_seg = [S+$A-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
add	sp,*6
!BCC_EOS
! 1471     write_word(ebda_seg,&((ebda_data_t *) 0)->ata.devices[device].pchs.spt,0);
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: ptradd unsigned char device = [S+8-6] to [8] struct  = const $142 (used reg = )
mov	al,-4[bp]
xor	ah,ah
mov	cx,*$1A
imul	cx
mov	bx,ax
! Debug: address unsigned short = [bx+$156] (used reg = )
! Debug: list * unsigned short = bx+$156 (used reg = )
add	bx,#$156
push	bx
! Debug: list unsigned short ebda_seg = [S+$A-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
add	sp,*6
!BCC_EOS
! 1472     write_dword(ebda_seg,&((ebda_data_t *) 0)->ata.devices[device].sectors,0L);
! Debug: list long = const 0 (used reg = )
xor	ax,ax
xor	bx,bx
push	bx
push	ax
! Debug: ptradd unsigned char device = [S+$A-6] to [8] struct  = const $142 (used reg = )
mov	al,-4[bp]
xor	ah,ah
mov	cx,*$1A
imul	cx
mov	bx,ax
! Debug: address unsigned long = [bx+$158] (used reg = )
! Debug: list * unsigned long = bx+$158 (used reg = )
add	bx,#$158
push	bx
! Debug: list unsigned short ebda_seg = [S+$C-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_dword+0 (used reg = )
call	_write_dword
add	sp,*8
!BCC_EOS
! 1473     }
! 1474   for (device=0; device<(4*2); device++) {
.147:
! Debug: postinc unsigned char device = [S+6-6] (used reg = )
mov	al,-4[bp]
inc	ax
mov	-4[bp],al
.148:
! Debug: lt int = const 8 to unsigned char device = [S+6-6] (used reg = )
mov	al,-4[bp]
cmp	al,*8
blo 	.149
.14A:
.146:
! Debug: eq int = const 0 to unsigned char device = [S+6-6] (used reg = )
xor	al,al
mov	-4[bp],al
!BCC_EOS
!BCC_EOS
jmp .14D
.14E:
! 1475     write_byte(ebda_seg,&((ebda_data_t *) 0)->ata.hdidmap[device],(4*2));
! Debug: list int = const 8 (used reg = )
mov	ax,*8
push	ax
! Debug: ptradd unsigned char device = [S+8-6] to [8] unsigned char = const $213 (used reg = )
mov	al,-4[bp]
xor	ah,ah
mov	bx,ax
! Debug: address unsigned char = [bx+$213] (used reg = )
! Debug: list * unsigned char = bx+$213 (used reg = )
add	bx,#$213
push	bx
! Debug: list unsigned short ebda_seg = [S+$A-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 1476     write_byte(ebda_seg,&((ebda_data_t *) 0)->ata.cdidmap[device],(4*2));
! Debug: list int = const 8 (used reg = )
mov	ax,*8
push	ax
! Debug: ptradd unsigned char device = [S+8-6] to [8] unsigned char = const $21C (used reg = )
mov	al,-4[bp]
xor	ah,ah
mov	bx,ax
! Debug: address unsigned char = [bx+$21C] (used reg = )
! Debug: list * unsigned char = bx+$21C (used reg = )
add	bx,#$21C
push	bx
! Debug: list unsigned short ebda_seg = [S+$A-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 1477     }
! 1478   write_byte(ebda_seg,&((ebda_data_t *) 0)->ata.hdcount,0);
.14C:
! Debug: postinc unsigned char device = [S+6-6] (used reg = )
mov	al,-4[bp]
inc	ax
mov	-4[bp],al
.14D:
! Debug: lt int = const 8 to unsigned char device = [S+6-6] (used reg = )
mov	al,-4[bp]
cmp	al,*8
jb 	.14E
.14F:
.14B:
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: list * unsigned char = const $212 (used reg = )
mov	ax,#$212
push	ax
! Debug: list unsigned short ebda_seg = [S+$A-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 1479   write_byte(ebda_seg,&((ebda_data_t *) 0)->ata.cdcount,0);
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: list * unsigned char = const $21B (used reg = )
mov	ax,#$21B
push	ax
! Debug: list unsigned short ebda_seg = [S+$A-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 1480 }
mov	sp,bp
pop	bp
ret
! 1481 void ata_detect( )
! Register BX used in function ata_init
! 1482 {
export	_ata_detect
_ata_detect:
! 1483   Bit16u ebda_seg=read_word(0x0040,0x000E);
push	bp
mov	bp,sp
dec	sp
dec	sp
! Debug: list int = const $E (used reg = )
mov	ax,*$E
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () unsigned short = read_word+0 (used reg = )
call	_read_word
add	sp,*4
! Debug: eq unsigned short = ax+0 to unsigned short ebda_seg = [S+4-4] (used reg = )
mov	-2[bp],ax
!BCC_EOS
! 1484   Bit8u hdcount, cdco
! 1484 unt, device, type;
!BCC_EOS
! 1485   Bit8u buffer[0x0200];
!BCC_EOS
! 1486   write_byte(ebda_seg,&((ebda_data_t *) 0)->ata.channels[0].iface,0x00);
add	sp,#-$204
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: list * unsigned char = const $122 (used reg = )
mov	ax,#$122
push	ax
! Debug: list unsigned short ebda_seg = [S+$20C-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 1487   write_word(ebda_seg,&((ebda_data_t *) 0)->ata.channels[0].iobase1,0x1f0);
! Debug: list int = const $1F0 (used reg = )
mov	ax,#$1F0
push	ax
! Debug: list * unsigned short = const $124 (used reg = )
mov	ax,#$124
push	ax
! Debug: list unsigned short ebda_seg = [S+$20C-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
add	sp,*6
!BCC_EOS
! 1488   write_word(ebda_seg,&((ebda_data_t *) 0)->ata.channels[0].iobase2,0x3f0);
! Debug: list int = const $3F0 (used reg = )
mov	ax,#$3F0
push	ax
! Debug: list * unsigned short = const $126 (used reg = )
mov	ax,#$126
push	ax
! Debug: list unsigned short ebda_seg = [S+$20C-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
add	sp,*6
!BCC_EOS
! 1489   write_byte(ebda_seg,&((ebda_data_t *) 0)->ata.channels[0].irq,14);
! Debug: list int = const $E (used reg = )
mov	ax,*$E
push	ax
! Debug: list * unsigned char = const $128 (used reg = )
mov	ax,#$128
push	ax
! Debug: list unsigned short ebda_seg = [S+$20C-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 1490   write_byte(ebda_seg,&((ebda_data_t *) 0)->ata.channels[1].iface,0x00);
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: list * unsigned char = const $12A (used reg = )
mov	ax,#$12A
push	ax
! Debug: list unsigned short ebda_seg = [S+$20C-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 1491   write_word(ebda_seg,&((ebda_data_t *) 0)->ata.channels[1].iobase1,0x170);
! Debug: list int = const $170 (used reg = )
mov	ax,#$170
push	ax
! Debug: list * unsigned short = const $12C (used reg = )
mov	ax,#$12C
push	ax
! Debug: list unsigned short ebda_seg = [S+$20C-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
add	sp,*6
!BCC_EOS
! 1492   write_word(ebda_seg,&((ebda_data_t *) 0)->ata.channels[1].iobase2,0x370);
! Debug: list int = const $370 (used reg = )
mov	ax,#$370
push	ax
! Debug: list * unsigned short = const $12E (used reg = )
mov	ax,#$12E
push	ax
! Debug: list unsigned short ebda_seg = [S+$20C-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
add	sp,*6
!BCC_EOS
! 1493   write_byte(ebda_seg,&((ebda_data_t *) 0)->ata.channels[1].irq,15);
! Debug: list int = const $F (used reg = )
mov	ax,*$F
push	ax
! Debug: list * unsigned char = const $130 (used reg = )
mov	ax,#$130
push	ax
! Debug: list unsigned short ebda_seg = [S+$20C-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 1494   write_byte(ebda_seg,&((ebda_data_t *) 0)->ata.channels[2].iface,0x00);
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: list * unsigned char = const $132 (used reg = )
mov	ax,#$132
push	ax
! Debug: list unsigned short ebda_seg = [S+$20C-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 1495   write_word(ebda_seg,&((ebda_data_t *) 0)->ata.channels[2].iobase1,0x1e8);
! Debug: list int = const $1E8 (used reg = )
mov	ax,#$1E8
push	ax
! Debug: list * unsigned short = const $134 (used reg = )
mov	ax,#$134
push	ax
! Debug: list unsigned short ebda_seg = [S+$20C-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
add	sp,*6
!BCC_EOS
! 1496   write_word(ebda_seg,&((ebda_data_t *) 0)->ata.channels[2].iobase2,0x3e0);
! Debug: list int = const $3E0 (used reg = )
mov	ax,#$3E0
push	ax
! Debug: list * unsigned short = const $136 (used reg = )
mov	ax,#$136
push	ax
! Debug: list unsigned short ebda_seg = [S+$20C-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
add	sp,*6
!BCC_EOS
! 1497   write_byte(ebda_seg,&((ebda_data_t *) 0)->ata.channels[2].irq,12);
! Debug: list int = const $C (used reg = )
mov	ax,*$C
push	ax
! Debug: list * unsigned char = const $138 (used reg = )
mov	ax,#$138
push	ax
! Debug: list unsigned short ebda_seg = [S+$20C-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 1498   write_byte(ebda_seg,&((ebda_data_t *) 0)->ata.channels[3].iface,0x00);
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: list * unsigned char = const $13A (used reg = )
mov	ax,#$13A
push	ax
! Debug: list unsigned short ebda_seg = [S+$20C-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 1499   write_word(ebda_seg,&((ebda_data_t *) 0)->ata.channels[3].iobase1,0x168);
! Debug: list int = const $168 (used reg = )
mov	ax,#$168
push	ax
! Debug: list * unsigned short = const $13C (used reg = )
mov	ax,#$13C
push	ax
! Debug: list unsigned short ebda_seg = [S+$20C-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
add	sp,*6
!BCC_EOS
! 1500   write_word(ebda_seg,&((ebda_data_t *) 0)->ata.channels[3].iobase2,0x360);
! Debug: list int = const $360 (used reg = )
mov	ax,#$360
push	ax
! Debug: list * unsigned short = const $13E (used reg = )
mov	ax,#$13E
push	ax
! Debug: list unsigned short ebda_seg = [S+$20C-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
add	sp,*6
!BCC_EOS
! 1501   write_byte(ebda_seg,&((ebda_data_t *) 0)->ata.channels[3].irq,11);
! Debug: list int = const $B (used reg = )
mov	ax,*$B
push	ax
! Debug: list * unsigned char = const $140 (used reg = )
mov	ax,#$140
push	ax
! Debug: list unsigned short ebda_seg = [S+$20C-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 1502   hdcount=cdcount=0;
! Debug: eq int = const 0 to unsigned char cdcount = [S+$208-6] (used reg = )
xor	al,al
mov	-4[bp],al
! Debug: eq unsigned char = al+0 to unsigned char hdcount = [S+$208-5] (used reg = )
mov	-3[bp],al
!BCC_EOS
! 1503   for(device=0; device<(4*2); device++) {
! Debug: eq int = const 0 to unsigned char device = [S+$208-7] (used reg = )
xor	al,al
mov	-5[bp],al
!BCC_EOS
!BCC_EOS
br 	.152
.153:
! 1504     Bit16u iobase1, iobase2;
!BCC_EOS
! 1505     Bit8u channel, slave, shift;
!BCC_EOS
! 1506     Bit8u sc, sn, cl, ch, st;
!BCC_EOS
! 1507     channel = device / 2;
add	sp,*-$C
! Debug: div int = const 2 to unsigned char device = [S+$214-7] (used reg = )
mov	al,-5[bp]
xor	ah,ah
shr	ax,*1
! Debug: eq unsigned int = ax+0 to unsigned char channel = [S+$214-$20D] (used reg = )
mov	-$20B[bp],al
!BCC_EOS
! 1508     slave = device % 2;
! Debug: mod int = const 2 to unsigned char device = [S+$214-7] (used reg = )
mov	al,-5[bp]
xor	ah,ah
and	al,*1
! Debug: eq unsigned char = al+0 to unsigned char slave = [S+$214-$20E] (used reg = )
mov	-$20C[bp],al
!BCC_EOS
! 1509     iobase1 =read_word(ebda_seg,&((ebda_data_t *) 0)->ata.channels[channel].iobase1);
! Debug: ptradd unsigned char channel = [S+$214-$20D] to [4] struct  = const $122 (used reg = )
mov	al,-$20B[bp]
xor	ah,ah
mov	cl,*3
shl	ax,cl
mov	bx,ax
! Debug: address unsigned short = [bx+$124] (used reg = )
! Debug: list * unsigned short = bx+$124 (used reg = )
add	bx,#$124
push	bx
! Debug: list unsigned short ebda_seg = [S+$216-4] (used reg = )
push	-2[bp]
! Debug: func () unsigned short = read_word+0 (used reg = )
call	_read_word
add	sp,*4
! Debug: eq unsigned short = ax+0 to unsigned short iobase1 = [S+$214-$20A] (used reg = )
mov	-$208[bp],ax
!BCC_EOS
! 1510     iobase2 =read_word(ebda_seg,&((ebda_data_t *) 0)->ata.channels[channel].iobase2);
! Debug: ptradd unsigned char channel = [S+$214-$20D] to [4] struct  = const $122 (used reg = )
mov	al,-$20B[bp]
xor	ah,ah
mov	cl,*3
shl	ax,cl
mov	bx,ax
! Debug: address unsigned short = [bx+$126] (used reg = )
! Debug: list * unsigned short = bx+$126 (used reg = )
add	bx,#$126
push	bx
! Debug: list unsigned short ebda_seg = [S+$216-4] (used reg = )
push	-2[bp]
! Debug: func () unsigned short = read_word+0 (used reg = )
call	_read_word
add	sp,*4
! Debug: eq unsigned short = ax+0 to unsigned short iobase2 = [S+$214-$20C] (used reg = )
mov	-$20A[bp],ax
!BCC_EOS
! 1511     outb(iobase2+6, 0x08 | 0x02);
! Debug: list int = const $A (used reg = )
mov	ax,*$A
push	ax
! Debug: add int = const 6 to unsigned short iobase2 = [S+$216-$20C] (used reg = )
mov	ax,-$20A[bp]
! Debug: list unsigned int = ax+6 (used reg = )
add	ax,*6
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 1512     outb(iobase1+6, slave ? 0xb0 : 0xa0);
mov	al,-$20C[bp]
test	al,al
je  	.154
.155:
mov	al,#$B0
jmp .156
.154:
mov	al,#$A0
.156:
! Debug: list char = al+0 (used reg = )
xor	ah,ah
push	ax
! Debug: add int = const 6 to unsigned short iobase1 = [S+$216-$20A] (used reg = )
mov	ax,-$208[bp]
! Debug: list unsigned int = ax+6 (used reg = )
add	ax,*6
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 1513     outb(iobase1+2, 0x55);
! Debug: list int = const $55 (used reg = )
mov	ax,*$55
push	ax
! Debug: add int = const 2 to unsigned short iobase1 = [S+$216-$20A] (used reg = )
mov	ax,-$208[bp]
! Debug: list unsigned int = ax+2 (used reg = )
inc	ax
inc	ax
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 1514     outb(iobase1+3, 0xaa);
! Debug: list int = const $AA (used reg = )
mov	ax,#$AA
push	ax
! Debug: add int = const 3 to unsigned short iobase1 = [S+$216-$20A] (used reg = )
mov	ax,-$208[bp]
! Debug: list unsigned int = ax+3 (used reg = )
add	ax,*3
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 1515     outb(iobase1+2, 0xaa);
! Debug: list int = const $AA (used reg = )
mov	ax,#$AA
push	ax
! Debug: add int = const 2 to unsigned short iobase1 = [S+$216-$20A] (used reg = )
mov	ax,-$208[bp]
! Debug: list unsigned int = ax+2 (used reg = )
inc	ax
inc	ax
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 1516     outb(iobase1+3, 0x55);
! Debug: list int = const $55 (used reg = )
mov	ax,*$55
push	ax
! Debug: add int = const 3 to unsigned short iobase1 = [S+$216-$20A] (used reg = )
mov	ax,-$208[bp]
! Debug: list unsigned int = ax+3 (used reg = )
add	ax,*3
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 1517     outb(iobase1+2, 0x55);
! Debug: list int = const $55 (used reg = )
mov	ax,*$55
push	ax
! Debug: add int = const 2 to unsigned short iobase1 = [S+$216-$20A] (used reg = )
mov	ax,-$208[bp]
! Debug: list unsigned int = ax+2 (used reg = )
inc	ax
inc	ax
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 1518     outb(iobase1+3, 0xaa);
! Debug: list int = const $AA (used reg = )
mov	ax,#$AA
push	ax
! Debug: add int = const 3 to unsigned short iobase1 = [S+$216-$20A] (used reg = )
mov	ax,-$208[bp]
! Debug: list unsigned int = ax+3 (used reg = )
add	ax,*3
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 1519     sc = inb(iobase1+2);
! Debug: add int = const 2 to unsigned short iobase1 = [S+$214-$20A] (used reg = )
mov	ax,-$208[bp]
! Debug: list unsigned int = ax+2 (used reg = )
inc	ax
inc	ax
push	ax
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char sc = [S+$214-$210] (used reg = )
mov	-$20E[bp],al
!BCC_EOS
! 1520     sn = inb(iobase1+3);
! Debug: add int = const 3 to unsigned short iobase1 = [S+$214-$20A] (used reg = )
mov	ax,-$208[bp]
! Debug: list unsigned int = ax+3 (used reg = )
add	ax,*3
push	ax
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char sn = [S+$214-$211] (used reg = )
mov	-$20F[bp],al
!BCC_EOS
! 1521     if ( (sc == 0x55) && (sn == 0xaa) ) {
! Debug: logeq int = const $55 to unsigned char sc = [S+$214-$210] (used reg = )
mov	al,-$20E[bp]
cmp	al,*$55
bne 	.157
.159:
! Debug: logeq int = const $AA to unsigned char sn = [S+$214-$211] (used reg = )
mov	al,-$20F[bp]
cmp	al,#$AA
bne 	.157
.158:
! 1522       write_byte(ebda_seg,&((ebda_data_t *) 0)->ata.devices[device].type,0x01);
! Debug: list int = const 1 (used reg = )
mov	ax,*1
push	ax
! Debug: ptradd unsigned char device = [S+$216-7] to [8] struct  = const $142 (used reg = )
mov	al,-5[bp]
xor	ah,ah
mov	cx,*$1A
imul	cx
mov	bx,ax
! Debug: address unsigned char = [bx+$142] (used reg = )
! Debug: list * unsigned char = bx+$142 (used reg = )
add	bx,#$142
push	bx
! Debug: list unsigned short ebda_seg = [S+$218-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 1523       ata_reset (device);
! Debug: list unsigned char device = [S+$214-7] (used reg = )
mov	al,-5[bp]
xor	ah,ah
push	ax
! Debug: func () void = ata_reset+0 (used reg = )
call	_ata_reset
inc	sp
inc	sp
!BCC_EOS
! 1524       outb(iobas
! 1524 e1+6, slave ? 0xb0 : 0xa0);
mov	al,-$20C[bp]
test	al,al
je  	.15A
.15B:
mov	al,#$B0
jmp .15C
.15A:
mov	al,#$A0
.15C:
! Debug: list char = al+0 (used reg = )
xor	ah,ah
push	ax
! Debug: add int = const 6 to unsigned short iobase1 = [S+$216-$20A] (used reg = )
mov	ax,-$208[bp]
! Debug: list unsigned int = ax+6 (used reg = )
add	ax,*6
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 1525       sc = inb(iobase1+2);
! Debug: add int = const 2 to unsigned short iobase1 = [S+$214-$20A] (used reg = )
mov	ax,-$208[bp]
! Debug: list unsigned int = ax+2 (used reg = )
inc	ax
inc	ax
push	ax
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char sc = [S+$214-$210] (used reg = )
mov	-$20E[bp],al
!BCC_EOS
! 1526       sn = inb(iobase1+3);
! Debug: add int = const 3 to unsigned short iobase1 = [S+$214-$20A] (used reg = )
mov	ax,-$208[bp]
! Debug: list unsigned int = ax+3 (used reg = )
add	ax,*3
push	ax
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char sn = [S+$214-$211] (used reg = )
mov	-$20F[bp],al
!BCC_EOS
! 1527       if ( (sc==0x01) && (sn==0x01) ) {
! Debug: logeq int = const 1 to unsigned char sc = [S+$214-$210] (used reg = )
mov	al,-$20E[bp]
cmp	al,*1
bne 	.15D
.15F:
! Debug: logeq int = const 1 to unsigned char sn = [S+$214-$211] (used reg = )
mov	al,-$20F[bp]
cmp	al,*1
bne 	.15D
.15E:
! 1528         cl = inb(iobase1+4);
! Debug: add int = const 4 to unsigned short iobase1 = [S+$214-$20A] (used reg = )
mov	ax,-$208[bp]
! Debug: list unsigned int = ax+4 (used reg = )
add	ax,*4
push	ax
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char cl = [S+$214-$212] (used reg = )
mov	-$210[bp],al
!BCC_EOS
! 1529         ch = inb(iobase1+5);
! Debug: add int = const 5 to unsigned short iobase1 = [S+$214-$20A] (used reg = )
mov	ax,-$208[bp]
! Debug: list unsigned int = ax+5 (used reg = )
add	ax,*5
push	ax
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char ch = [S+$214-$213] (used reg = )
mov	-$211[bp],al
!BCC_EOS
! 1530         st = inb(iobase1+7);
! Debug: add int = const 7 to unsigned short iobase1 = [S+$214-$20A] (used reg = )
mov	ax,-$208[bp]
! Debug: list unsigned int = ax+7 (used reg = )
add	ax,*7
push	ax
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char st = [S+$214-$214] (used reg = )
mov	-$212[bp],al
!BCC_EOS
! 1531         if ( (cl==0x14) && (ch==0xeb) ) {
! Debug: logeq int = const $14 to unsigned char cl = [S+$214-$212] (used reg = )
mov	al,-$210[bp]
cmp	al,*$14
jne 	.160
.162:
! Debug: logeq int = const $EB to unsigned char ch = [S+$214-$213] (used reg = )
mov	al,-$211[bp]
cmp	al,#$EB
jne 	.160
.161:
! 1532           write_byte(ebda_seg,&((ebda_data_t *) 0)->ata.devices[device].type,0x03);
! Debug: list int = const 3 (used reg = )
mov	ax,*3
push	ax
! Debug: ptradd unsigned char device = [S+$216-7] to [8] struct  = const $142 (used reg = )
mov	al,-5[bp]
xor	ah,ah
mov	cx,*$1A
imul	cx
mov	bx,ax
! Debug: address unsigned char = [bx+$142] (used reg = )
! Debug: list * unsigned char = bx+$142 (used reg = )
add	bx,#$142
push	bx
! Debug: list unsigned short ebda_seg = [S+$218-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 1533           }
! 1534         else if ( (cl==0x00) && (ch==0x00) && (st!=0x00) ) {
jmp .163
.160:
! Debug: logeq int = const 0 to unsigned char cl = [S+$214-$212] (used reg = )
mov	al,-$210[bp]
test	al,al
jne 	.164
.167:
! Debug: logeq int = const 0 to unsigned char ch = [S+$214-$213] (used reg = )
mov	al,-$211[bp]
test	al,al
jne 	.164
.166:
! Debug: ne int = const 0 to unsigned char st = [S+$214-$214] (used reg = )
mov	al,-$212[bp]
test	al,al
je  	.164
.165:
! 1535           write_byte(ebda_seg,&((ebda_data_t *) 0)->ata.devices[device].type,0x02);
! Debug: list int = const 2 (used reg = )
mov	ax,*2
push	ax
! Debug: ptradd unsigned char device = [S+$216-7] to [8] struct  = const $142 (used reg = )
mov	al,-5[bp]
xor	ah,ah
mov	cx,*$1A
imul	cx
mov	bx,ax
! Debug: address unsigned char = [bx+$142] (used reg = )
! Debug: list * unsigned char = bx+$142 (used reg = )
add	bx,#$142
push	bx
! Debug: list unsigned short ebda_seg = [S+$218-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 1536           }
! 1537         }
.164:
.163:
! 1538       }
.15D:
! 1539     type=read_byte(ebda_seg,&((ebda_data_t *) 0)->ata.devices[device].type);
.157:
! Debug: ptradd unsigned char device = [S+$214-7] to [8] struct  = const $142 (used reg = )
mov	al,-5[bp]
xor	ah,ah
mov	cx,*$1A
imul	cx
mov	bx,ax
! Debug: address unsigned char = [bx+$142] (used reg = )
! Debug: list * unsigned char = bx+$142 (used reg = )
add	bx,#$142
push	bx
! Debug: list unsigned short ebda_seg = [S+$216-4] (used reg = )
push	-2[bp]
! Debug: func () unsigned char = read_byte+0 (used reg = )
call	_read_byte
add	sp,*4
! Debug: eq unsigned char = al+0 to unsigned char type = [S+$214-8] (used reg = )
mov	-6[bp],al
!BCC_EOS
! 1540     if(type == 0x02) {
! Debug: logeq int = const 2 to unsigned char type = [S+$214-8] (used reg = )
mov	al,-6[bp]
cmp	al,*2
bne 	.168
.169:
! 1541       Bit32u sectors;
!BCC_EOS
! 1542       Bit16u cylinders, heads, spt, blksize;
!BCC_EOS
! 1543       Bit8u translation, removable, mode;
!BCC_EOS
! 1544       mode = 0x00;
add	sp,*-$10
! Debug: eq int = const 0 to unsigned char mode = [S+$224-$223] (used reg = )
xor	al,al
mov	-$221[bp],al
!BCC_EOS
! 1545       write_byte(ebda_seg,&((ebda_data_t *) 0)->ata.devices[device].device,0xFF);
! Debug: list int = const $FF (used reg = )
mov	ax,#$FF
push	ax
! Debug: ptradd unsigned char device = [S+$226-7] to [8] struct  = const $142 (used reg = )
mov	al,-5[bp]
xor	ah,ah
mov	cx,*$1A
imul	cx
mov	bx,ax
! Debug: address unsigned char = [bx+$143] (used reg = )
! Debug: list * unsigned char = bx+$143 (used reg = )
add	bx,#$143
push	bx
! Debug: list unsigned short ebda_seg = [S+$228-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 1546       write_byte(ebda_seg,&((ebda_data_t *) 0)->ata.devices[device].mode, 0x00);
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: ptradd unsigned char device = [S+$226-7] to [8] struct  = const $142 (used reg = )
mov	al,-5[bp]
xor	ah,ah
mov	cx,*$1A
imul	cx
mov	bx,ax
! Debug: address unsigned char = [bx+$146] (used reg = )
! Debug: list * unsigned char = bx+$146 (used reg = )
add	bx,#$146
push	bx
! Debug: list unsigned short ebda_seg = [S+$228-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 1547       if (ata_cmd_data_in(device,0xEC, 1, 0, 0, 0, 0L, get_SS(),buffer) !=0 )
! Debug: list * unsigned char buffer = S+$224-$208 (used reg = )
lea	bx,-$206[bp]
push	bx
! Debug: func () unsigned short = get_SS+0 (used reg = )
call	_get_SS
! Debug: list unsigned short = ax+0 (used reg = )
push	ax
! Debug: list long = const 0 (used reg = )
xor	ax,ax
xor	bx,bx
push	bx
push	ax
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: list int = const 1 (used reg = )
mov	ax,*1
push	ax
! Debug: list int = const $EC (used reg = )
mov	ax,#$EC
push	ax
! Debug: list unsigned char device = [S+$236-7] (used reg = )
mov	al,-5[bp]
xor	ah,ah
push	ax
! Debug: func () unsigned short = ata_cmd_data_in+0 (used reg = )
call	_ata_cmd_data_in
add	sp,*$14
! Debug: ne int = const 0 to unsigned short = ax+0 (used reg = )
test	ax,ax
je  	.16A
.16B:
! 1548         bios_printf((2 | 4 | 1), "ata-detect: Failed to detect ATA device\n");
! Debug: list * char = .16C+0 (used reg = )
mov	bx,#.16C
push	bx
! Debug: list int = const 7 (used reg = )
mov	ax,*7
push	ax
! Debug: func () void = bios_printf+0 (used reg = )
call	_bios_printf
add	sp,*4
!BCC_EOS
! 1549       removable = (read_byte(get_SS(),buffer+0) & 0x80) ? 1 : 0;
.16A:
! Debug: list * unsigned char buffer = S+$224-$208 (used reg = )
lea	bx,-$206[bp]
push	bx
! Debug: func () unsigned short = get_SS+0 (used reg = )
call	_get_SS
! Debug: list unsigned short = ax+0 (used reg = )
push	ax
! Debug: func () unsigned char = read_byte+0 (used reg = )
call	_read_byte
add	sp,*4
! Debug: and int = const $80 to unsigned char = al+0 (used reg = )
and	al,#$80
test	al,al
je  	.16D
.16E:
mov	al,*1
jmp .16F
.16D:
xor	al,al
.16F:
! Debug: eq char = al+0 to unsigned char removable = [S+$224-$222] (used reg = )
mov	-$220[bp],al
!BCC_EOS
! 1550       mode = read_byte(get_SS(),buffer+96) ? 0x01 : 0x00;
! Debug: list * unsigned char buffer = S+$224-$1A8 (used reg = )
lea	bx,-$1A6[bp]
push	bx
! Debug: func () unsigned short = get_SS+0 (used reg = )
call	_get_SS
! Debug: list unsigned short = ax+0 (used reg = )
push	ax
! Debug: func () unsigned char = read_byte+0 (used reg = )
call	_read_byte
add	sp,*4
test	al,al
je  	.170
.171:
mov	al,*1
jmp .172
.170:
xor	al,al
.172:
! Debug: eq char = al+0 to unsigned char mode = [S+$224-$223] (used reg = )
mov	-$221[bp],al
!BCC_EOS
! 1551       blksize = read_word(get_SS(),buffer+10);
! Debug: list * unsigned char buffer = S+$224-$1FE (used reg = )
lea	bx,-$1FC[bp]
push	bx
! Debug: func () unsigned short = get_SS+0 (used reg = )
call	_get_SS
! Debug: list unsigned short = ax+0 (used reg = )
push	ax
! Debug: func () unsigned short = read_word+0 (used reg = )
call	_read_word
add	sp,*4
! Debug: eq unsigned short = ax+0 to unsigned short blksize = [S+$224-$220] (used reg = )
mov	-$21E[bp],ax
!BCC_EOS
! 1552       cylinders = read_word(get_SS(),buffer+(1*2));
! Debug: list * unsigned char buffer = S+$224-$206 (used reg = )
lea	bx,-$204[bp]
push	bx
! Debug: func () unsigned short = get_SS+0 (used reg = )
call	_get_SS
! Debug: list unsigned short = ax+0 (used reg = )
push	ax
! Debug: func () unsigned short = read_word+0 (used reg = )
call	_read_word
add	sp,*4
! Debug: eq unsigned short = ax+0 to unsigned short cylinders = [S+$224-$21A] (used reg = )
mov	-$218[bp],ax
!BCC_EOS
! 1553       heads = read_word(get_SS(),buffer+(3*2));
! Debug: list * unsigned char buffer = S+$224-$202 (used reg = )
lea	bx,-$200[bp]
push	bx
! Debug: func () unsigned short = get_SS+0 (used reg = )
call	_get_SS
! Debug: list unsigned short = ax+0 (used reg = )
push	ax
! Debug: func () unsigned short = read_word+0 (used reg = )
call	_read_word
add	sp,*4
! Debug: eq unsigned short = ax+0 to unsigned short heads = [S+$224-$21C] (used reg = )
mov	-$21A[bp],ax
!BCC_EOS
! 1554       spt = read_word(get_SS(),buffer+(6*2));
! Debug: list * unsigned char buffer = S+$224-$1FC (used reg = )
lea	bx,-$1FA[bp]
push	bx
! Debug: func () unsigned short = get_SS+0 (used reg = )
call	_get_SS
! Debug: list unsigned short = ax+0 (used reg = )
push	ax
! Debug: func () unsigned short = read_word+0 (used reg = )
call	_read_word
add	sp,*4
! Debug: eq unsigned short = ax+0 to unsigned short spt = [S+$224-$21E] (used reg = )
mov	-$21C[bp],ax
!BCC_EOS
! 1555       sectors = read_dword(get_SS(),buffer+(60*2));
! Debug: list * unsigned char buffer = S+$224-$190 (used reg = )
lea	bx,-$18E[bp]
push	bx
! Debug: func () unsigned short = get_SS+0 (used reg = )
call	_get_SS
! Debug: list unsigned short = ax+0 (used reg = )
push	ax
! Debug: func () unsigned long = read_dword+0 (used reg = )
call	_read_dword
mov	bx,dx
add	sp,*4
! Debug: eq unsigned long = bx+0 to unsigned long sectors = [S+$224-$218] (used reg = )
mov	-$216[bp],ax
mov	-$214[bp],bx
!BCC_EOS
! 1556       write_byte(ebda_seg,&((ebda_data_t *) 0)->ata.devices[device].device,0xFF);
! Debug: list int = const $FF (used reg = )
mov	ax,#$FF
push	ax
! Debug: ptradd unsigned char device = [S+$226-7] to [8] struct  = const $142 (used reg = )
mov	al,-5[bp]
xor	ah,ah
mov	cx,*$1A
imul	cx
mov	bx,ax
! Debug: address unsigned char = [bx+$143] (used reg = )
! Debug: list * unsigned char = bx+$143 (used reg = )
add	bx,#$143
push	bx
! Debug: list unsigned short ebda_seg = [S+$228-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 1557       write_byte(ebda_seg,&((ebda_data_t *) 0)->ata.devices[device].removable, removable);
! Debug: list unsigned char removable = [S+$224-$222] (used reg = )
mov	al,-$220[bp]
xor	ah,ah
push	ax
! Debug: ptradd unsigned char device = [S+$226-7] to [8] struct  = const $142 (used reg = )
mov	al,-5[bp]
xor	ah,ah
mov	cx,*$1A
imul	cx
mov	bx,ax
! Debug: address unsigned char = [bx+$144] (used reg = )
! Debug: list * unsigned char = bx+$144 (used reg = )
add	bx,#$144
push	bx
! Debug: list unsigned short ebda_seg = [S+$228-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 1558       write_byte(ebda_seg,&((ebda_data_t *) 0)->ata.devices[device].mode, mode);
! Debug: list unsigned char mode = [S+$224-$223] (used reg = )
mov	al,-$221[bp]
xor	ah,ah
push	ax
! Debug: ptradd unsigned char device = [S+$226-7] to [8] struct  = const $142 (used reg = )
mov	al,-5[bp]
xor	ah,ah
mov	cx,*$1A
imul	cx
mov	bx,ax
! Debug: address unsigned char = [bx+$146] (used reg = )
! Debug: list * unsigned char = bx+$146 (used reg = )
add	bx,#$146
push	bx
! Debug: list unsigned short ebda_seg = [S+$228-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 1559       write_word(ebda_seg,&((ebda_data_t *) 0)->ata.devices[device].blksize, blksize);
! Debug: list unsigned short blksize = [S+$224-$220] (used reg = )
push	-$21E[bp]
! Debug: ptradd unsigned char device = [S+$226-7] to [8] struct  = const $142 (used reg = )
mov	al,-5[bp]
xor	ah,ah
mov	cx,*$1A
imul	cx
mov	bx,ax
! Debug: address unsigned short = [bx+$148] (used reg = )
! Debug: list * unsigned short = bx+$148 (used reg = )
add	bx,#$148
push	bx
! Debug: list unsigned short ebda_seg = [S+$228-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
add	sp,*6
!BCC_EOS
! 1560       write_word(ebda_seg,&((ebda_data_t *) 0)->ata.devices[device].pchs.heads, heads);
! Debug: list unsigned short heads = [S+$224-$21C] (used reg = )
push	-$21A[bp]
! Debug: ptradd unsigned char device = [S+$226-7] to [8] struct  = const $142 (used reg = )
mov	al,-5[bp]
xor	ah,ah
mov	cx,*$1A
imul	cx
mov	bx,ax
! Debug: address unsigned short = [bx+$152] (used reg = )
! Debug: list * unsigned short = bx+$152 (used reg = )
add	bx,#$152
push	bx
! Debug: list unsigned short ebda_seg = [S+$228-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
add	sp,*6
!BCC_EOS
! 1561       write_word(ebda_seg,&((ebda_data_t *) 0)->ata.devices[device].pchs.cylinders, cylinders);
! Debug: list unsigned short cylinders = [S+$224-$21A] (used reg = )
push	-$218[bp]
! Debug: ptradd unsigned char device = [S+$226-7] to [8] struct  = const $142 (used reg = )
mov	al,-5[bp]
xor	ah,ah
mov	cx,*$1A
imul	cx
mov	bx,ax
! Debug: address unsigned short = [bx+$154] (used reg = )
! Debug: list * unsigned short = bx+$154 (used reg = )
add	bx,#$154
push	bx
! Debug: list unsigned short ebda_seg = [S+$228-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
add	sp,*6
!BCC_EOS
! 1562       write_word(ebda_seg,&((ebda_data_t *) 0)->ata.devices[device].pchs.spt, spt);
! Debug: list unsigned short spt = [S+$224-$21E] (used reg = )
push	-$21C[bp]
! Debug: ptradd unsigned char device = [S+$226-7] to [8] struct  = const $142 (used reg = )
mov	al,-5[bp]
xor	ah,ah
mov	cx,*$1A
imul	cx
mov	bx,ax
! Debug: address unsigned short = [bx+$156] (used reg = )
! Debug: list * unsigned short = bx+$156 (used reg = )
add	bx,#$156
push	bx
! Debug: list unsigned short ebda_seg = [S+$228-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
add	sp,*6
!BCC_EOS
! 1563  
! 1563      write_dword(ebda_seg,&((ebda_data_t *) 0)->ata.devices[device].sectors, sectors);
! Debug: list unsigned long sectors = [S+$224-$218] (used reg = )
push	-$214[bp]
push	-$216[bp]
! Debug: ptradd unsigned char device = [S+$228-7] to [8] struct  = const $142 (used reg = )
mov	al,-5[bp]
xor	ah,ah
mov	cx,*$1A
imul	cx
mov	bx,ax
! Debug: address unsigned long = [bx+$158] (used reg = )
! Debug: list * unsigned long = bx+$158 (used reg = )
add	bx,#$158
push	bx
! Debug: list unsigned short ebda_seg = [S+$22A-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_dword+0 (used reg = )
call	_write_dword
add	sp,*8
!BCC_EOS
! 1564       bios_printf(4, "ata%d-%d: PCHS=%u/%d/%d translation=", channel, slave,cylinders, heads, spt);
! Debug: list unsigned short spt = [S+$224-$21E] (used reg = )
push	-$21C[bp]
! Debug: list unsigned short heads = [S+$226-$21C] (used reg = )
push	-$21A[bp]
! Debug: list unsigned short cylinders = [S+$228-$21A] (used reg = )
push	-$218[bp]
! Debug: list unsigned char slave = [S+$22A-$20E] (used reg = )
mov	al,-$20C[bp]
xor	ah,ah
push	ax
! Debug: list unsigned char channel = [S+$22C-$20D] (used reg = )
mov	al,-$20B[bp]
xor	ah,ah
push	ax
! Debug: list * char = .173+0 (used reg = )
mov	bx,#.173
push	bx
! Debug: list int = const 4 (used reg = )
mov	ax,*4
push	ax
! Debug: func () void = bios_printf+0 (used reg = )
call	_bios_printf
add	sp,*$E
!BCC_EOS
! 1565       translation = inb_cmos(0x39 + channel/2);
! Debug: div int = const 2 to unsigned char channel = [S+$224-$20D] (used reg = )
mov	al,-$20B[bp]
xor	ah,ah
shr	ax,*1
! Debug: add unsigned int = ax+0 to int = const $39 (used reg = )
! Debug: expression subtree swapping
! Debug: list unsigned int = ax+$39 (used reg = )
add	ax,*$39
push	ax
! Debug: func () unsigned char = inb_cmos+0 (used reg = )
call	_inb_cmos
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char translation = [S+$224-$221] (used reg = )
mov	-$21F[bp],al
!BCC_EOS
! 1566       for (shift=device%4; shift>0; shift--) translation >>= 2;
! Debug: mod int = const 4 to unsigned char device = [S+$224-7] (used reg = )
mov	al,-5[bp]
xor	ah,ah
and	al,*3
! Debug: eq unsigned char = al+0 to unsigned char shift = [S+$224-$20F] (used reg = )
mov	-$20D[bp],al
!BCC_EOS
!BCC_EOS
jmp .176
.177:
! Debug: srab int = const 2 to unsigned char translation = [S+$224-$221] (used reg = )
mov	al,-$21F[bp]
xor	ah,ah
shr	ax,*1
shr	ax,*1
mov	-$21F[bp],al
!BCC_EOS
! 1567       translation &= 0x03;
.175:
! Debug: postdec unsigned char shift = [S+$224-$20F] (used reg = )
mov	al,-$20D[bp]
dec	ax
mov	-$20D[bp],al
.176:
! Debug: gt int = const 0 to unsigned char shift = [S+$224-$20F] (used reg = )
mov	al,-$20D[bp]
test	al,al
jne	.177
.178:
.174:
! Debug: andab int = const 3 to unsigned char translation = [S+$224-$221] (used reg = )
mov	al,-$21F[bp]
and	al,*3
mov	-$21F[bp],al
!BCC_EOS
! 1568       write_byte(ebda_seg,&((ebda_data_t *) 0)->ata.devices[device].translation, translation);
! Debug: list unsigned char translation = [S+$224-$221] (used reg = )
mov	al,-$21F[bp]
xor	ah,ah
push	ax
! Debug: ptradd unsigned char device = [S+$226-7] to [8] struct  = const $142 (used reg = )
mov	al,-5[bp]
xor	ah,ah
mov	cx,*$1A
imul	cx
mov	bx,ax
! Debug: address unsigned char = [bx+$14A] (used reg = )
! Debug: list * unsigned char = bx+$14A (used reg = )
add	bx,#$14A
push	bx
! Debug: list unsigned short ebda_seg = [S+$228-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 1569       switch (translation) {
mov	al,-$21F[bp]
jmp .17B
! 1570         case 0:
! 1571           bios_printf(4, "none");
.17C:
! Debug: list * char = .17D+0 (used reg = )
mov	bx,#.17D
push	bx
! Debug: list int = const 4 (used reg = )
mov	ax,*4
push	ax
! Debug: func () void = bios_printf+0 (used reg = )
call	_bios_printf
add	sp,*4
!BCC_EOS
! 1572           break;
jmp .179
!BCC_EOS
! 1573         case 1:
! 1574           bios_printf(4, "lba");
.17E:
! Debug: list * char = .17F+0 (used reg = )
mov	bx,#.17F
push	bx
! Debug: list int = const 4 (used reg = )
mov	ax,*4
push	ax
! Debug: func () void = bios_printf+0 (used reg = )
call	_bios_printf
add	sp,*4
!BCC_EOS
! 1575           break;
jmp .179
!BCC_EOS
! 1576         case 2:
! 1577           bios_printf(4, "large");
.180:
! Debug: list * char = .181+0 (used reg = )
mov	bx,#.181
push	bx
! Debug: list int = const 4 (used reg = )
mov	ax,*4
push	ax
! Debug: func () void = bios_printf+0 (used reg = )
call	_bios_printf
add	sp,*4
!BCC_EOS
! 1578           break;
jmp .179
!BCC_EOS
! 1579         case 3:
! 1580           bios_printf(4, "r-echs");
.182:
! Debug: list * char = .183+0 (used reg = )
mov	bx,#.183
push	bx
! Debug: list int = const 4 (used reg = )
mov	ax,*4
push	ax
! Debug: func () void = bios_printf+0 (used reg = )
call	_bios_printf
add	sp,*4
!BCC_EOS
! 1581           break;
jmp .179
!BCC_EOS
! 1582         }
! 1583       switch (translation) {
jmp .179
.17B:
sub	al,*0
je 	.17C
sub	al,*1
je 	.17E
sub	al,*1
je 	.180
sub	al,*1
je 	.182
.179:
..FFFC	=	-$224
mov	al,-$21F[bp]
br 	.186
! 1584         case 0:
! 1585           break;
.187:
br 	.184
!BCC_EOS
! 1586         case 1:
! 1587           spt = 63;
.188:
! Debug: eq int = const $3F to unsigned short spt = [S+$224-$21E] (used reg = )
mov	ax,*$3F
mov	-$21C[bp],ax
!BCC_EOS
! 1588           sectors /= 63;
! Debug: divab unsigned long = const $3F to unsigned long sectors = [S+$224-$218] (used reg = )
mov	ax,*$3F
xor	bx,bx
push	bx
push	ax
mov	ax,-$216[bp]
mov	bx,-$214[bp]
lea	di,-2+..FFFB[bp]
call	ldivul
mov	-$216[bp],ax
mov	-$214[bp],bx
add	sp,*4
!BCC_EOS
! 1589           heads = sectors / 1024;
! Debug: div unsigned long = const $400 to unsigned long sectors = [S+$224-$218] (used reg = )
mov	ax,#$400
xor	bx,bx
push	bx
push	ax
mov	ax,-$216[bp]
mov	bx,-$214[bp]
lea	di,-2+..FFFB[bp]
call	ldivul
add	sp,*4
! Debug: eq unsigned long = bx+0 to unsigned short heads = [S+$224-$21C] (used reg = )
mov	-$21A[bp],ax
!BCC_EOS
! 1590           if (heads>128) heads = 255;
! Debug: gt int = const $80 to unsigned short heads = [S+$224-$21C] (used reg = )
mov	ax,-$21A[bp]
cmp	ax,#$80
jbe 	.189
.18A:
! Debug: eq int = const $FF to unsigned short heads = [S+$224-$21C] (used reg = )
mov	ax,#$FF
mov	-$21A[bp],ax
!BCC_EOS
! 1591           else if (heads>64) heads = 128;
jmp .18B
.189:
! Debug: gt int = const $40 to unsigned short heads = [S+$224-$21C] (used reg = )
mov	ax,-$21A[bp]
cmp	ax,*$40
jbe 	.18C
.18D:
! Debug: eq int = const $80 to unsigned short heads = [S+$224-$21C] (used reg = )
mov	ax,#$80
mov	-$21A[bp],ax
!BCC_EOS
! 1592           else if (heads>32) heads = 64;
jmp .18E
.18C:
! Debug: gt int = const $20 to unsigned short heads = [S+$224-$21C] (used reg = )
mov	ax,-$21A[bp]
cmp	ax,*$20
jbe 	.18F
.190:
! Debug: eq int = const $40 to unsigned short heads = [S+$224-$21C] (used reg = )
mov	ax,*$40
mov	-$21A[bp],ax
!BCC_EOS
! 1593           else if (heads>16) heads = 32;
jmp .191
.18F:
! Debug: gt int = const $10 to unsigned short heads = [S+$224-$21C] (used reg = )
mov	ax,-$21A[bp]
cmp	ax,*$10
jbe 	.192
.193:
! Debug: eq int = const $20 to unsigned short heads = [S+$224-$21C] (used reg = )
mov	ax,*$20
mov	-$21A[bp],ax
!BCC_EOS
! 1594           else heads=16;
jmp .194
.192:
! Debug: eq int = const $10 to unsigned short heads = [S+$224-$21C] (used reg = )
mov	ax,*$10
mov	-$21A[bp],ax
!BCC_EOS
! 1595           cylinders = sectors / heads;
.194:
.191:
.18E:
.18B:
! Debug: cast unsigned long = const 0 to unsigned short heads = [S+$224-$21C] (used reg = )
mov	ax,-$21A[bp]
xor	bx,bx
! Debug: div unsigned long = bx+0 to unsigned long sectors = [S+$224-$218] (used reg = )
push	bx
push	ax
mov	ax,-$216[bp]
mov	bx,-$214[bp]
lea	di,-2+..FFFB[bp]
call	ldivul
add	sp,*4
! Debug: eq unsigned long = bx+0 to unsigned short cylinders = [S+$224-$21A] (used reg = )
mov	-$218[bp],ax
!BCC_EOS
! 1596           break;
br 	.184
!BCC_EOS
! 1597         case 3:
! 1598           if (heads==16) {
.195:
! Debug: logeq int = const $10 to unsigned short heads = [S+$224-$21C] (used reg = )
mov	ax,-$21A[bp]
cmp	ax,*$10
jne 	.196
.197:
! 1599             if(cylinders>61439) cylinders=61439;
! Debug: cast unsigned long = const 0 to unsigned short cylinders = [S+$224-$21A] (used reg = )
mov	ax,-$218[bp]
xor	bx,bx
! Debug: gt long = const $EFFF to unsigned long = bx+0 (used reg = )
push	bx
push	ax
mov	ax,#$EFFF
xor	bx,bx
push	bx
push	ax
mov	ax,-2+..FFFB[bp]
mov	bx,0+..FFFB[bp]
lea	di,-6+..FFFB[bp]
call	lcmpul
lea	sp,2+..FFFB[bp]
jbe 	.198
.199:
! Debug: eq long = const $EFFF to unsigned short cylinders = [S+$224-$21A] (used reg = )
mov	ax,#$EFFF
mov	-$218[bp],ax
!BCC_EOS
! 1600             heads=15;
.198:
! Debug: eq int = const $F to unsigned short heads = [S+$224-$21C] (used reg = )
mov	ax,*$F
mov	-$21A[bp],ax
!BCC_EOS
! 1601             cylinders = (Bit16u)((Bit32u)(cylinders)*16/15);
! Debug: cast unsigned long = const 0 to unsigned short cylinders = [S+$224-$21A] (used reg = )
mov	ax,-$218[bp]
xor	bx,bx
! Debug: mul unsigned long = const $10 to unsigned long = bx+0 (used reg = )
push	bx
push	ax
mov	ax,*$10
xor	bx,bx
push	bx
push	ax
mov	ax,-2+..FFFB[bp]
mov	bx,0+..FFFB[bp]
lea	di,-6+..FFFB[bp]
call	lmulul
add	sp,*8
! Debug: div unsigned long = const $F to unsigned long = bx+0 (used reg = )
push	bx
push	ax
mov	ax,*$F
xor	bx,bx
push	bx
push	ax
mov	ax,-2+..FFFB[bp]
mov	bx,0+..FFFB[bp]
lea	di,-6+..FFFB[bp]
call	ldivul
add	sp,*8
! Debug: cast unsigned short = const 0 to unsigned long = bx+0 (used reg = )
! Debug: eq unsigned short = ax+0 to unsigned short cylinders = [S+$224-$21A] (used reg = )
mov	-$218[bp],ax
!BCC_EOS
! 1602             }
! 1603         case 2:
.196:
! 1604           while(cylinders > 1024) {
.19A:
jmp .19C
.19D:
! 1605             cylinders >>= 1;
! Debug: srab int = const 1 to unsigned short cylinders = [S+$224-$21A] (used reg = )
mov	ax,-$218[bp]
shr	ax,*1
mov	-$218[bp],ax
!BCC_EOS
! 1606             heads <<= 1;
! Debug: slab int = const 1 to unsigned short heads = [S+$224-$21C] (used reg = )
mov	ax,-$21A[bp]
shl	ax,*1
mov	-$21A[bp],ax
!BCC_EOS
! 1607             if (heads > 127) break;
! Debug: gt int = const $7F to unsigned short heads = [S+$224-$21C] (used reg = )
mov	ax,-$21A[bp]
cmp	ax,*$7F
jbe 	.19E
.19F:
jmp .19B
!BCC_EOS
! 1608           }
.19E:
! 1609           break;
.19C:
! Debug: gt int = const $400 to unsigned short cylinders = [S+$224-$21A] (used reg = )
mov	ax,-$218[bp]
cmp	ax,#$400
ja 	.19D
.1A0:
.19B:
jmp .184
!BCC_EOS
! 1610         }
! 1611       if (cylinders > 1024) cylinders=1024;
jmp .184
.186:
sub	al,*0
beq 	.187
sub	al,*1
beq 	.188
sub	al,*1
je 	.19A
sub	al,*1
beq 	.195
.184:
..FFFB	=	-$224
! Debug: gt int = const $400 to unsigned short cylinders = [S+$224-$21A] (used reg = )
mov	ax,-$218[bp]
cmp	ax,#$400
jbe 	.1A1
.1A2:
! Debug: eq int = const $400 to unsigned short cylinders = [S+$224-$21A] (used reg = )
mov	ax,#$400
mov	-$218[bp],ax
!BCC_EOS
! 1612       bios_printf(4, " LCHS=%d/%d/%d\n", cylinders, heads, spt);
.1A1:
! Debug: list unsigned short spt = [S+$224-$21E] (used reg = )
push	-$21C[bp]
! Debug: list unsigned short heads = [S+$226-$21C] (used reg = )
push	-$21A[bp]
! Debug: list unsigned short cylinders = [S+$228-$21A] (used reg = )
push	-$218[bp]
! Debug: list * char = .1A3+0 (used reg = )
mov	bx,#.1A3
push	bx
! Debug: list int = const 4 (used reg = )
mov	ax,*4
push	ax
! Debug: func () void = bios_printf+0 (used reg = )
call	_bios_printf
add	sp,*$A
!BCC_EOS
! 1613       write_word(ebda_seg,&((ebda_data_t *) 0)->ata.devices[device].lchs.heads, heads);
! Debug: list unsigned short heads = [S+$224-$21C] (used reg = )
push	-$21A[bp]
! Debug: ptradd unsigned char device = [S+$226-7] to [8] struct  = const $142 (used reg = )
mov	al,-5[bp]
xor	ah,ah
mov	cx,*$1A
imul	cx
mov	bx,ax
! Debug: address unsigned short = [bx+$14C] (used reg = )
! Debug: list * unsigned short = bx+$14C (used reg = )
add	bx,#$14C
push	bx
! Debug: list unsigned short ebda_seg = [S+$228-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
add	sp,*6
!BCC_EOS
! 1614       write_word(ebda_seg,&((ebda_data_t *) 0)->ata.devices[device].lchs.cylinders, cylinders);
! Debug: list unsigned short cylinders = [S+$224-$21A] (used reg = )
push	-$218[bp]
! Debug: ptradd unsigned char device = [S+$226-7] to [8] struct  = const $142 (used reg = )
mov	al,-5[bp]
xor	ah,ah
mov	cx,*$1A
imul	cx
mov	bx,ax
! Debug: address unsigned short = [bx+$14E] (used reg = )
! Debug: list * unsigned short = bx+$14E (used reg = )
add	bx,#$14E
push	bx
! Debug: list unsigned short ebda_seg = [S+$228-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
add	sp,*6
!BCC_EOS
! 1615       write_word(ebda_seg,&((ebda_data_t *) 0)->ata.devices[device].lchs.spt, spt);
! Debug: list unsigned short spt = [S+$224-$21E] (used reg = )
push	-$21C[bp]
! Debug: ptradd unsigned char device = [S+$226-7] to [8] struct  = const $142 (used reg = )
mov	al,-5[bp]
xor	ah,ah
mov	cx,*$1A
imul	cx
mov	bx,ax
! Debug: address unsigned short = [bx+$150] (used reg = )
! Debug: list * unsigned short = bx+$150 (used reg = )
add	bx,#$150
push	bx
! Debug: list unsigned short ebda_seg = [S+$228-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
add	sp,*6
!BCC_EOS
! 1616       write_byte(ebda_seg,&((ebda_data_t *) 0)->ata.hdidmap[hdcount], device);
! Debug: list unsigned char device = [S+$224-7] (used reg = )
mov	al,-5[bp]
xor	ah,ah
push	ax
! Debug: ptradd unsigned char hdcount = [S+$226-5] to [8] unsigned char = const $213 (used reg = )
mov	al,-3[bp]
xor	ah,ah
mov	bx,ax
! Debug: address unsigned char = [bx+$213] (used reg = )
! Debug: list * unsigned char = bx+$213 (used reg = )
add	bx,#$213
push	bx
! Debug: list unsigned short ebda_seg = [S+$228-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 1617       hdcount++;
! Debug: postinc unsigned char hdcount = [S+$224-5] (used reg = )
mov	al,-3[bp]
inc	ax
mov	-3[bp],al
!BCC_EOS
! 1618       }
add	sp,*$10
! 1619     if(type == 0x03) {
.168:
! Debug: logeq int = const 3 to unsigned char type = [S+$214-8] (used reg = )
mov	al,-6[bp]
cmp	al,*3
bne 	.1A4
.1A5:
! 1620       Bit8u type, removable, mode;
!BCC_EOS
! 1621       Bit
! 1621 16u blksize;
!BCC_EOS
! 1622       mode = 0x00;
add	sp,*-6
! Debug: eq int = const 0 to unsigned char mode = [S+$21A-$217] (used reg = )
xor	al,al
mov	-$215[bp],al
!BCC_EOS
! 1623       write_byte(ebda_seg,&((ebda_data_t *) 0)->ata.devices[device].device,0x05);
! Debug: list int = const 5 (used reg = )
mov	ax,*5
push	ax
! Debug: ptradd unsigned char device = [S+$21C-7] to [8] struct  = const $142 (used reg = )
mov	al,-5[bp]
xor	ah,ah
mov	cx,*$1A
imul	cx
mov	bx,ax
! Debug: address unsigned char = [bx+$143] (used reg = )
! Debug: list * unsigned char = bx+$143 (used reg = )
add	bx,#$143
push	bx
! Debug: list unsigned short ebda_seg = [S+$21E-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 1624       write_byte(ebda_seg,&((ebda_data_t *) 0)->ata.devices[device].mode, 0x00);
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: ptradd unsigned char device = [S+$21C-7] to [8] struct  = const $142 (used reg = )
mov	al,-5[bp]
xor	ah,ah
mov	cx,*$1A
imul	cx
mov	bx,ax
! Debug: address unsigned char = [bx+$146] (used reg = )
! Debug: list * unsigned char = bx+$146 (used reg = )
add	bx,#$146
push	bx
! Debug: list unsigned short ebda_seg = [S+$21E-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 1625       if (ata_cmd_data_in(device,0xA1, 1, 0, 0, 0, 0L, get_SS(),buffer) != 0)
! Debug: list * unsigned char buffer = S+$21A-$208 (used reg = )
lea	bx,-$206[bp]
push	bx
! Debug: func () unsigned short = get_SS+0 (used reg = )
call	_get_SS
! Debug: list unsigned short = ax+0 (used reg = )
push	ax
! Debug: list long = const 0 (used reg = )
xor	ax,ax
xor	bx,bx
push	bx
push	ax
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: list int = const 1 (used reg = )
mov	ax,*1
push	ax
! Debug: list int = const $A1 (used reg = )
mov	ax,#$A1
push	ax
! Debug: list unsigned char device = [S+$22C-7] (used reg = )
mov	al,-5[bp]
xor	ah,ah
push	ax
! Debug: func () unsigned short = ata_cmd_data_in+0 (used reg = )
call	_ata_cmd_data_in
add	sp,*$14
! Debug: ne int = const 0 to unsigned short = ax+0 (used reg = )
test	ax,ax
je  	.1A6
.1A7:
! 1626         bios_printf((2 | 4 | 1), "ata-detect: Failed to detect ATAPI device\n");
! Debug: list * char = .1A8+0 (used reg = )
mov	bx,#.1A8
push	bx
! Debug: list int = const 7 (used reg = )
mov	ax,*7
push	ax
! Debug: func () void = bios_printf+0 (used reg = )
call	_bios_printf
add	sp,*4
!BCC_EOS
! 1627       type = read_byte(get_SS(),buffer+1) & 0x1f;
.1A6:
! Debug: list * unsigned char buffer = S+$21A-$207 (used reg = )
lea	bx,-$205[bp]
push	bx
! Debug: func () unsigned short = get_SS+0 (used reg = )
call	_get_SS
! Debug: list unsigned short = ax+0 (used reg = )
push	ax
! Debug: func () unsigned char = read_byte+0 (used reg = )
call	_read_byte
add	sp,*4
! Debug: and int = const $1F to unsigned char = al+0 (used reg = )
and	al,*$1F
! Debug: eq unsigned char = al+0 to unsigned char type = [S+$21A-$215] (used reg = )
mov	-$213[bp],al
!BCC_EOS
! 1628       removable = (read_byte(get_SS(),buffer+0) & 0x80) ? 1 : 0;
! Debug: list * unsigned char buffer = S+$21A-$208 (used reg = )
lea	bx,-$206[bp]
push	bx
! Debug: func () unsigned short = get_SS+0 (used reg = )
call	_get_SS
! Debug: list unsigned short = ax+0 (used reg = )
push	ax
! Debug: func () unsigned char = read_byte+0 (used reg = )
call	_read_byte
add	sp,*4
! Debug: and int = const $80 to unsigned char = al+0 (used reg = )
and	al,#$80
test	al,al
je  	.1A9
.1AA:
mov	al,*1
jmp .1AB
.1A9:
xor	al,al
.1AB:
! Debug: eq char = al+0 to unsigned char removable = [S+$21A-$216] (used reg = )
mov	-$214[bp],al
!BCC_EOS
! 1629       mode = read_byte(get_SS(),buffer+96) ? 0x01 : 0x00;
! Debug: list * unsigned char buffer = S+$21A-$1A8 (used reg = )
lea	bx,-$1A6[bp]
push	bx
! Debug: func () unsigned short = get_SS+0 (used reg = )
call	_get_SS
! Debug: list unsigned short = ax+0 (used reg = )
push	ax
! Debug: func () unsigned char = read_byte+0 (used reg = )
call	_read_byte
add	sp,*4
test	al,al
je  	.1AC
.1AD:
mov	al,*1
jmp .1AE
.1AC:
xor	al,al
.1AE:
! Debug: eq char = al+0 to unsigned char mode = [S+$21A-$217] (used reg = )
mov	-$215[bp],al
!BCC_EOS
! 1630       blksize = 2048;
! Debug: eq int = const $800 to unsigned short blksize = [S+$21A-$21A] (used reg = )
mov	ax,#$800
mov	-$218[bp],ax
!BCC_EOS
! 1631       write_byte(ebda_seg,&((ebda_data_t *) 0)->ata.devices[device].device, type);
! Debug: list unsigned char type = [S+$21A-$215] (used reg = )
mov	al,-$213[bp]
xor	ah,ah
push	ax
! Debug: ptradd unsigned char device = [S+$21C-7] to [8] struct  = const $142 (used reg = )
mov	al,-5[bp]
xor	ah,ah
mov	cx,*$1A
imul	cx
mov	bx,ax
! Debug: address unsigned char = [bx+$143] (used reg = )
! Debug: list * unsigned char = bx+$143 (used reg = )
add	bx,#$143
push	bx
! Debug: list unsigned short ebda_seg = [S+$21E-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 1632       write_byte(ebda_seg,&((ebda_data_t *) 0)->ata.devices[device].removable, removable);
! Debug: list unsigned char removable = [S+$21A-$216] (used reg = )
mov	al,-$214[bp]
xor	ah,ah
push	ax
! Debug: ptradd unsigned char device = [S+$21C-7] to [8] struct  = const $142 (used reg = )
mov	al,-5[bp]
xor	ah,ah
mov	cx,*$1A
imul	cx
mov	bx,ax
! Debug: address unsigned char = [bx+$144] (used reg = )
! Debug: list * unsigned char = bx+$144 (used reg = )
add	bx,#$144
push	bx
! Debug: list unsigned short ebda_seg = [S+$21E-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 1633       write_byte(ebda_seg,&((ebda_data_t *) 0)->ata.devices[device].mode, mode);
! Debug: list unsigned char mode = [S+$21A-$217] (used reg = )
mov	al,-$215[bp]
xor	ah,ah
push	ax
! Debug: ptradd unsigned char device = [S+$21C-7] to [8] struct  = const $142 (used reg = )
mov	al,-5[bp]
xor	ah,ah
mov	cx,*$1A
imul	cx
mov	bx,ax
! Debug: address unsigned char = [bx+$146] (used reg = )
! Debug: list * unsigned char = bx+$146 (used reg = )
add	bx,#$146
push	bx
! Debug: list unsigned short ebda_seg = [S+$21E-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 1634       write_word(ebda_seg,&((ebda_data_t *) 0)->ata.devices[device].blksize, blksize);
! Debug: list unsigned short blksize = [S+$21A-$21A] (used reg = )
push	-$218[bp]
! Debug: ptradd unsigned char device = [S+$21C-7] to [8] struct  = const $142 (used reg = )
mov	al,-5[bp]
xor	ah,ah
mov	cx,*$1A
imul	cx
mov	bx,ax
! Debug: address unsigned short = [bx+$148] (used reg = )
! Debug: list * unsigned short = bx+$148 (used reg = )
add	bx,#$148
push	bx
! Debug: list unsigned short ebda_seg = [S+$21E-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
add	sp,*6
!BCC_EOS
! 1635       write_byte(ebda_seg,&((ebda_data_t *) 0)->ata.cdidmap[cdcount], device);
! Debug: list unsigned char device = [S+$21A-7] (used reg = )
mov	al,-5[bp]
xor	ah,ah
push	ax
! Debug: ptradd unsigned char cdcount = [S+$21C-6] to [8] unsigned char = const $21C (used reg = )
mov	al,-4[bp]
xor	ah,ah
mov	bx,ax
! Debug: address unsigned char = [bx+$21C] (used reg = )
! Debug: list * unsigned char = bx+$21C (used reg = )
add	bx,#$21C
push	bx
! Debug: list unsigned short ebda_seg = [S+$21E-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 1636       cdcount++;
! Debug: postinc unsigned char cdcount = [S+$21A-6] (used reg = )
mov	al,-4[bp]
inc	ax
mov	-4[bp],al
!BCC_EOS
! 1637       }
add	sp,*6
! 1638       {
.1A4:
! 1639       Bit32u sizeinmb;
!BCC_EOS
! 1640       Bit16u ataversion;
!BCC_EOS
! 1641       Bit8u c, i, version, model[41];
!BCC_EOS
! 1642       switch (type) {
add	sp,*-$32
mov	al,-6[bp]
br 	.1B1
! 1643         case 0x02:
! 1644           sizeinmb = read_dword(ebda_seg,&((ebda_data_t *) 0)->ata.devices[device].sectors);
.1B2:
! Debug: ptradd unsigned char device = [S+$246-7] to [8] struct  = const $142 (used reg = )
mov	al,-5[bp]
xor	ah,ah
mov	cx,*$1A
imul	cx
mov	bx,ax
! Debug: address unsigned long = [bx+$158] (used reg = )
! Debug: list * unsigned long = bx+$158 (used reg = )
add	bx,#$158
push	bx
! Debug: list unsigned short ebda_seg = [S+$248-4] (used reg = )
push	-2[bp]
! Debug: func () unsigned long = read_dword+0 (used reg = )
call	_read_dword
mov	bx,dx
add	sp,*4
! Debug: eq unsigned long = bx+0 to unsigned long sizeinmb = [S+$246-$218] (used reg = )
mov	-$216[bp],ax
mov	-$214[bp],bx
!BCC_EOS
! 1645           sizeinmb >>= 11;
! Debug: srab int = const $B to unsigned long sizeinmb = [S+$246-$218] (used reg = )
mov	ax,-$216[bp]
mov	bx,-$214[bp]
mov	al,ah
mov	ah,bl
mov	bl,bh
sub	bh,bh
mov	di,*3
call	lsrul
mov	-$216[bp],ax
mov	-$214[bp],bx
!BCC_EOS
! 1646         case 0x03:
! 1647           ataversion=((Bit16u)(read_byte(get_SS(),buffer+161))<<8)|read_byte(get_SS(),buffer+160);
.1B3:
! Debug: list * unsigned char buffer = S+$246-$168 (used reg = )
lea	bx,-$166[bp]
push	bx
! Debug: func () unsigned short = get_SS+0 (used reg = )
call	_get_SS
! Debug: list unsigned short = ax+0 (used reg = )
push	ax
! Debug: func () unsigned char = read_byte+0 (used reg = )
call	_read_byte
add	sp,*4
push	ax
! Debug: list * unsigned char buffer = S+$248-$167 (used reg = )
lea	bx,-$165[bp]
push	bx
! Debug: func () unsigned short = get_SS+0 (used reg = )
call	_get_SS
! Debug: list unsigned short = ax+0 (used reg = )
push	ax
! Debug: func () unsigned char = read_byte+0 (used reg = )
call	_read_byte
add	sp,*4
! Debug: cast unsigned short = const 0 to unsigned char = al+0 (used reg = )
xor	ah,ah
! Debug: sl int = const 8 to unsigned short = ax+0 (used reg = )
mov	ah,al
xor	al,al
! Debug: or unsigned char (temp) = [S+$248-$248] to unsigned int = ax+0 (used reg = )
or	al,0+..FFFA[bp]
inc	sp
inc	sp
! Debug: eq unsigned int = ax+0 to unsigned short ataversion = [S+$246-$21A] (used reg = )
mov	-$218[bp],ax
!BCC_EOS
! 1648           for(version=15;version>0;version--) {
! Debug: eq int = const $F to unsigned char version = [S+$246-$21D] (used reg = )
mov	al,*$F
mov	-$21B[bp],al
!BCC_EOS
!BCC_EOS
jmp .1B6
.1B7:
! 1649             if((ataversion&(1<<version))!=0)
! Debug: sl unsigned char version = [S+$246-$21D] to int = const 1 (used reg = )
mov	al,-$21B[bp]
xor	ah,ah
mov	bx,ax
mov	ax,*1
mov	cx,bx
shl	ax,cl
! Debug: and int = ax+0 to unsigned short ataversion = [S+$246-$21A] (used reg = )
! Debug: expression subtree swapping
and	ax,-$218[bp]
! Debug: ne int = const 0 to unsigned int = ax+0 (used reg = )
test	ax,ax
je  	.1B8
.1B9:
! 1650             break;
jmp .1B4
!BCC_EOS
! 1651             }
.1B8:
! 1652           for(i=0;i<20;i++){
.1B5:
! Debug: postdec unsigned char version = [S+$246-$21D] (used reg = )
mov	al,-$21B[bp]
dec	ax
mov	-$21B[bp],al
.1B6:
! Debug: gt int = const 0 to unsigned char version = [S+$246-$21D] (used reg = )
mov	al,-$21B[bp]
test	al,al
jne	.1B7
.1BA:
.1B4:
! Debug: eq int = const 0 to unsigned char i = [S+$246-$21C] (used reg = )
xor	al,al
mov	-$21A[bp],al
!BCC_EOS
!BCC_EOS
br 	.1BD
.1BE:
! 1653             write_byte(get_SS(),model+(i*2),read_byte(get_SS(),buffer+(i*2)+54+1));
! Debug: mul int = const 2 to unsigned char i = [S+$246-$21C] (used reg = )
mov	al,-$21A[bp]
xor	ah,ah
shl	ax,*1
! Debug: ptradd unsigned int = ax+0 to [$200] unsigned char buffer = S+$246-$208 (used reg = )
mov	bx,bp
add	bx,ax
! Debug: ptradd int = const $36 to [$200] unsigned char = bx-$206 (used reg = )
! Debug: ptradd int = const 1 to [$200] unsigned char = bx-$1D0 (used reg = )
! Debug: cast * unsigned char = const 0 to [$200] unsigned char = bx-$1CF (used reg = )
! Debug: list * unsigned char = bx-$1CF (used reg = )
add	bx,#-$1CF
push	bx
! Debug: func () unsigned short = get_SS+0 (used reg = )
call	_get_SS
! Debug: list unsigned short = ax+0 (used reg = )
push	ax
! Debug: func () unsigned char = read_byte+0 (used reg = )
call	_read_byte
add	sp,*4
! Debug: list unsigned char = al+0 (used reg = )
xor	ah,ah
push	ax
! Debug: mul int = const 2 to unsigned char i = [S+$248-$21C] (used reg = )
mov	al,-$21A[bp]
xor	ah,ah
shl	ax,*1
! Debug: ptradd unsigned int = ax+0 to [$29] unsigned char model = S+$248-$246 (used reg = )
mov	bx,bp
add	bx,ax
! Debug: cast * unsigned char = const 0 to [$29] unsigned char = bx-$244 (used reg = )
! Debug: list * unsigned char = bx-$244 (used reg = )
add	bx,#-$244
push	bx
! Debug: func () unsigned short = get_SS+0 (used reg = )
call	_get_SS
! Debug: list unsigned short = ax+0 (used reg = )
push	ax
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 1654             write_byte(get_SS(),model+(i*2)+1,read_byte(get_SS(),buffer+(i*2)+54));
! Debug: mul int = const 2 to unsigned char i = [S+$246-$21C] (used reg = )
mov	al,-$21A[bp]
xor	ah,ah
shl	ax,*1
! Debug: ptradd unsigned int = ax+0 to [$200] unsigned char buffer = S+$246-$208 (used reg = )
mov	bx,bp
add	bx,ax
! Debug: ptradd int = const $36 to [$200] unsigned char = bx-$206 (used reg = )
! Debug: cast * unsigned char = const 0 to [$200] unsigned char = bx-$1D0 (used reg = )
! Debug: list * unsigned char = bx-$1D0 (used reg = )
add	bx,#-$1D0
push	bx
! Debug: func () unsigned short = get_SS+0 (used reg = )
call	_get_SS
! Debug: list unsigned short = ax+0 (used reg = )
push	ax
! Debug: func () unsigned char = read_byte+0 (used reg = )
call	_read_byte
add	sp,*4
! Debug: list unsigned char = al+0 (used reg = )
xor	ah,ah
push	ax
! Debug: mul int = const 2 to unsigned char i = [S+$248-$21C] (used reg = )
mov	al,-$21A[bp]
xor	ah,ah
shl	ax,*1
! Debug: ptradd unsigned int = ax+0 to [$29] unsigned char model = S+$248-$246 (used reg = )
mov	bx,bp
add	bx,ax
! Debug: ptradd int = const 1 to [$29] unsigned char = bx-$244 (used reg = )
! Debug: cast * unsigned char = const 0 to [$29] unsigned char = bx-$243 (used reg = )
! Debug: list * unsigned char = bx-$243 (used reg = )
add	bx,#-$243
push	bx
! Debug: func () unsigned short = get_SS+0 (used reg = )
call	_get_SS
! Debug: list unsigned short = ax+0 (used reg = )
push	ax
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 1655             }
! 1656           write_byte(get_SS(),model+40,0x00);
.1BC:
! Debug: postinc unsigned char i = [S+$246-$21C] (used reg = )
mov	al,-$21A[bp]
inc	ax
mov	-$21A[bp],al
.1BD:
! Debug: lt int = const $14 to unsigned char i = [S+$246-$21C] (used reg = )
mov	al,-$21A[bp]
cmp	al,*$14
blo 	.1BE
.1BF:
.1BB:
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: list * unsigned char model = S+$248-$21E (used reg = )
lea	bx,-$21C[bp]
push	bx
! Debug: func () unsigned short = get_SS+0 (used reg = )
call	_get_SS
! Debug: list unsigned short = ax+0 (used reg = )
push	ax
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 1657           for(i=39;i>0;i--){
! Debug: eq int = const $27 to unsigned char i = [S+$246-$21C] (used reg = )
mov	al,*$27
mov	-$21A[bp],al
!BCC_EOS
!BCC_EOS
jmp .1C2
.1C3:
! 1658             if(read_byte(get_SS(),model+i)==0x20)
! Debug: ptradd unsigned char i = [S+$246-$21C] to [$29] unsigned char model = S+$246-$246 (used reg = )
mov	al,-$21A[bp]
xor	ah,ah
mov	bx,bp
add	bx,ax
! Debug: cast * unsigned char = const 0 to [$29] unsigned char = bx-$244 (used reg = )
! Debug: list * unsigned char = bx-$244 (used reg = )
add	bx,#-$244
push	bx
! Debug: func () unsigned short = get_SS+0 (used reg = )
call	_get_SS
! Debug: list unsigned short = ax+0 (used reg = )
push	ax
! Debug: func () unsigned char = read_byte+0 (used reg = )
call	_read_byte
add	sp,*4
! Debug: logeq int = const $20 to unsigned char = al+0 (used reg = )
cmp	al,*$20
jne 	.1C4
.1C5:
! 1659               write_byte(get_SS(),model+i,0x00);
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: ptradd unsigned char i = [S+$248-$21C] to [$29] unsigned char model = S+$248-$246 (used reg = )
mov	al,-$21A[bp]
xor	ah,ah
mov	bx,bp
add	bx,ax
! Debug: cast * unsigned char = const 0 to [$29] unsigned char = bx-$244 (used reg = )
! Debug: list * unsigned char = bx-$244 (used reg = )
add	bx,#-$244
push	bx
! Debug: func () unsigned short = get_SS+0 (used reg = )
call	_get_SS
! Debug: list unsigned short = ax+0 (used reg = )
push	ax
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 1660             else break;
jmp .1C6
.1C4:
jmp .1C0
!BCC_EOS
! 1661             }
.1C6:
! 1662           break;
.1C1:
! Debug: postdec unsigned char i = [S+$246-$21C] (used reg = )
mov	al,-$21A[bp]
dec	ax
mov	-$21A[bp],al
.1C2:
! Debug: gt int = const 0 to unsigned char i = [S+$246-$21C] (used reg = )
mov	al,-$21A[bp]
test	al,al
jne	.1C3
.1C7:
.1C0:
jmp .1AF
!BCC_EOS
! 1663         }
! 1664       switch (type) {
jmp .1AF
.1B1:
sub	al,*2
beq 	.1B2
sub	al,*1
beq 	.1B3
.1AF:
..FFFA	=	-$246
mov	al,-6[bp]
br 	.1CA
! 1665         case 0x02:
! 1666           bios_printf(2, "ata%d %s: ",channel,slave?" slave":"m
.1CB:
! 1666 aster");
mov	al,-$20C[bp]
test	al,al
je  	.1CF
.1D0:
mov	bx,#.1CD
jmp .1D1
.1CF:
mov	bx,#.1CE
.1D1:
! Debug: list * char = bx+0 (used reg = )
push	bx
! Debug: list unsigned char channel = [S+$248-$20D] (used reg = )
mov	al,-$20B[bp]
xor	ah,ah
push	ax
! Debug: list * char = .1CC+0 (used reg = )
mov	bx,#.1CC
push	bx
! Debug: list int = const 2 (used reg = )
mov	ax,*2
push	ax
! Debug: func () void = bios_printf+0 (used reg = )
call	_bios_printf
add	sp,*8
!BCC_EOS
! 1667           i=0; while(c=read_byte(get_SS(),model+i++)) bios_printf(2, "%c",c);
! Debug: eq int = const 0 to unsigned char i = [S+$246-$21C] (used reg = )
xor	al,al
mov	-$21A[bp],al
!BCC_EOS
jmp .1D3
.1D4:
! Debug: list unsigned char c = [S+$246-$21B] (used reg = )
mov	al,-$219[bp]
xor	ah,ah
push	ax
! Debug: list * char = .1D5+0 (used reg = )
mov	bx,#.1D5
push	bx
! Debug: list int = const 2 (used reg = )
mov	ax,*2
push	ax
! Debug: func () void = bios_printf+0 (used reg = )
call	_bios_printf
add	sp,*6
!BCC_EOS
! 1668           if (sizeinmb < 1UL<<16)
.1D3:
! Debug: postinc unsigned char i = [S+$246-$21C] (used reg = )
mov	al,-$21A[bp]
inc	ax
mov	-$21A[bp],al
! Debug: ptradd unsigned char = al-1 to [$29] unsigned char model = S+$246-$246 (used reg = )
dec	ax
xor	ah,ah
mov	bx,bp
add	bx,ax
! Debug: cast * unsigned char = const 0 to [$29] unsigned char = bx-$244 (used reg = )
! Debug: list * unsigned char = bx-$244 (used reg = )
add	bx,#-$244
push	bx
! Debug: func () unsigned short = get_SS+0 (used reg = )
call	_get_SS
! Debug: list unsigned short = ax+0 (used reg = )
push	ax
! Debug: func () unsigned char = read_byte+0 (used reg = )
call	_read_byte
add	sp,*4
! Debug: eq unsigned char = al+0 to unsigned char c = [S+$246-$21B] (used reg = )
mov	-$219[bp],al
test	al,al
jne	.1D4
.1D6:
.1D2:
! Debug: lt unsigned long = const $10000 to unsigned long sizeinmb = [S+$246-$218] (used reg = )
xor	ax,ax
mov	bx,*1
lea	di,-$216[bp]
call	lcmpul
jbe 	.1D7
.1D8:
! 1669             bios_printf(2, " ATA-%d Hard-Disk (%04u MBytes)\n",version,(Bit16u)sizeinmb);
! Debug: list unsigned short sizeinmb = [S+$246-$218] (used reg = )
push	-$216[bp]
! Debug: list unsigned char version = [S+$248-$21D] (used reg = )
mov	al,-$21B[bp]
xor	ah,ah
push	ax
! Debug: list * char = .1D9+0 (used reg = )
mov	bx,#.1D9
push	bx
! Debug: list int = const 2 (used reg = )
mov	ax,*2
push	ax
! Debug: func () void = bios_printf+0 (used reg = )
call	_bios_printf
add	sp,*8
!BCC_EOS
! 1670           else
! 1671             bios_printf(2, " ATA-%d Hard-Disk (%04u GBytes)\n",version,(Bit16u)(sizeinmb>>10));
jmp .1DA
.1D7:
! Debug: sr int = const $A to unsigned long sizeinmb = [S+$246-$218] (used reg = )
mov	ax,-$216[bp]
mov	bx,-$214[bp]
mov	al,ah
mov	ah,bl
mov	bl,bh
sub	bh,bh
mov	di,*2
call	lsrul
! Debug: cast unsigned short = const 0 to unsigned long = bx+0 (used reg = )
! Debug: list unsigned short = ax+0 (used reg = )
push	ax
! Debug: list unsigned char version = [S+$248-$21D] (used reg = )
mov	al,-$21B[bp]
xor	ah,ah
push	ax
! Debug: list * char = .1DB+0 (used reg = )
mov	bx,#.1DB
push	bx
! Debug: list int = const 2 (used reg = )
mov	ax,*2
push	ax
! Debug: func () void = bios_printf+0 (used reg = )
call	_bios_printf
add	sp,*8
!BCC_EOS
! 1672           break;
.1DA:
br 	.1C8
!BCC_EOS
! 1673         case 0x03:
! 1674           bios_printf(2, "ata%d %s: ",channel,slave?" slave":"master");
.1DC:
mov	al,-$20C[bp]
test	al,al
je  	.1E0
.1E1:
mov	bx,#.1DE
jmp .1E2
.1E0:
mov	bx,#.1DF
.1E2:
! Debug: list * char = bx+0 (used reg = )
push	bx
! Debug: list unsigned char channel = [S+$248-$20D] (used reg = )
mov	al,-$20B[bp]
xor	ah,ah
push	ax
! Debug: list * char = .1DD+0 (used reg = )
mov	bx,#.1DD
push	bx
! Debug: list int = const 2 (used reg = )
mov	ax,*2
push	ax
! Debug: func () void = bios_printf+0 (used reg = )
call	_bios_printf
add	sp,*8
!BCC_EOS
! 1675           i=0; while(c=read_byte(get_SS(),model+i++)) bios_printf(2, "%c",c);
! Debug: eq int = const 0 to unsigned char i = [S+$246-$21C] (used reg = )
xor	al,al
mov	-$21A[bp],al
!BCC_EOS
jmp .1E4
.1E5:
! Debug: list unsigned char c = [S+$246-$21B] (used reg = )
mov	al,-$219[bp]
xor	ah,ah
push	ax
! Debug: list * char = .1E6+0 (used reg = )
mov	bx,#.1E6
push	bx
! Debug: list int = const 2 (used reg = )
mov	ax,*2
push	ax
! Debug: func () void = bios_printf+0 (used reg = )
call	_bios_printf
add	sp,*6
!BCC_EOS
! 1676           if(read_byte(ebda_seg,&((ebda_data_t *) 0)->ata.devices[device].device)==0x05)
.1E4:
! Debug: postinc unsigned char i = [S+$246-$21C] (used reg = )
mov	al,-$21A[bp]
inc	ax
mov	-$21A[bp],al
! Debug: ptradd unsigned char = al-1 to [$29] unsigned char model = S+$246-$246 (used reg = )
dec	ax
xor	ah,ah
mov	bx,bp
add	bx,ax
! Debug: cast * unsigned char = const 0 to [$29] unsigned char = bx-$244 (used reg = )
! Debug: list * unsigned char = bx-$244 (used reg = )
add	bx,#-$244
push	bx
! Debug: func () unsigned short = get_SS+0 (used reg = )
call	_get_SS
! Debug: list unsigned short = ax+0 (used reg = )
push	ax
! Debug: func () unsigned char = read_byte+0 (used reg = )
call	_read_byte
add	sp,*4
! Debug: eq unsigned char = al+0 to unsigned char c = [S+$246-$21B] (used reg = )
mov	-$219[bp],al
test	al,al
jne	.1E5
.1E7:
.1E3:
! Debug: ptradd unsigned char device = [S+$246-7] to [8] struct  = const $142 (used reg = )
mov	al,-5[bp]
xor	ah,ah
mov	cx,*$1A
imul	cx
mov	bx,ax
! Debug: address unsigned char = [bx+$143] (used reg = )
! Debug: list * unsigned char = bx+$143 (used reg = )
add	bx,#$143
push	bx
! Debug: list unsigned short ebda_seg = [S+$248-4] (used reg = )
push	-2[bp]
! Debug: func () unsigned char = read_byte+0 (used reg = )
call	_read_byte
add	sp,*4
! Debug: logeq int = const 5 to unsigned char = al+0 (used reg = )
cmp	al,*5
jne 	.1E8
.1E9:
! 1677             bios_printf(2, " ATAPI-%d CD-Rom/DVD-Rom\n",version);
! Debug: list unsigned char version = [S+$246-$21D] (used reg = )
mov	al,-$21B[bp]
xor	ah,ah
push	ax
! Debug: list * char = .1EA+0 (used reg = )
mov	bx,#.1EA
push	bx
! Debug: list int = const 2 (used reg = )
mov	ax,*2
push	ax
! Debug: func () void = bios_printf+0 (used reg = )
call	_bios_printf
add	sp,*6
!BCC_EOS
! 1678           else
! 1679             bios_printf(2, " ATAPI-%d Device\n",version);
jmp .1EB
.1E8:
! Debug: list unsigned char version = [S+$246-$21D] (used reg = )
mov	al,-$21B[bp]
xor	ah,ah
push	ax
! Debug: list * char = .1EC+0 (used reg = )
mov	bx,#.1EC
push	bx
! Debug: list int = const 2 (used reg = )
mov	ax,*2
push	ax
! Debug: func () void = bios_printf+0 (used reg = )
call	_bios_printf
add	sp,*6
!BCC_EOS
! 1680           break;
.1EB:
jmp .1C8
!BCC_EOS
! 1681         case 0x01:
! 1682           bios_printf(2, "ata%d %s: Unknown device\n",channel,slave?" slave":"master");
.1ED:
mov	al,-$20C[bp]
test	al,al
je  	.1F1
.1F2:
mov	bx,#.1EF
jmp .1F3
.1F1:
mov	bx,#.1F0
.1F3:
! Debug: list * char = bx+0 (used reg = )
push	bx
! Debug: list unsigned char channel = [S+$248-$20D] (used reg = )
mov	al,-$20B[bp]
xor	ah,ah
push	ax
! Debug: list * char = .1EE+0 (used reg = )
mov	bx,#.1EE
push	bx
! Debug: list int = const 2 (used reg = )
mov	ax,*2
push	ax
! Debug: func () void = bios_printf+0 (used reg = )
call	_bios_printf
add	sp,*8
!BCC_EOS
! 1683           break;
jmp .1C8
!BCC_EOS
! 1684         }
! 1685       }
jmp .1C8
.1CA:
sub	al,*1
je 	.1ED
sub	al,*1
beq 	.1CB
sub	al,*1
beq 	.1DC
.1C8:
..FFF9	=	-$246
add	sp,*$32
! 1686     }
add	sp,*$C
! 1687   write_byte(ebda_seg,&((ebda_data_t *) 0)->ata.hdcount, hdcount);
.151:
! Debug: postinc unsigned char device = [S+$208-7] (used reg = )
mov	al,-5[bp]
inc	ax
mov	-5[bp],al
.152:
! Debug: lt int = const 8 to unsigned char device = [S+$208-7] (used reg = )
mov	al,-5[bp]
cmp	al,*8
blo 	.153
.1F4:
.150:
! Debug: list unsigned char hdcount = [S+$208-5] (used reg = )
mov	al,-3[bp]
xor	ah,ah
push	ax
! Debug: list * unsigned char = const $212 (used reg = )
mov	ax,#$212
push	ax
! Debug: list unsigned short ebda_seg = [S+$20C-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 1688   write_byte(ebda_seg,&((ebda_data_t *) 0)->ata.cdcount, cdcount);
! Debug: list unsigned char cdcount = [S+$208-6] (used reg = )
mov	al,-4[bp]
xor	ah,ah
push	ax
! Debug: list * unsigned char = const $21B (used reg = )
mov	ax,#$21B
push	ax
! Debug: list unsigned short ebda_seg = [S+$20C-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 1689   write_byte(0x40,0x75, hdcount);
! Debug: list unsigned char hdcount = [S+$208-5] (used reg = )
mov	al,-3[bp]
xor	ah,ah
push	ax
! Debug: list int = const $75 (used reg = )
mov	ax,*$75
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 1690   bios_printf(2, "\n");
! Debug: list * char = .1F5+0 (used reg = )
mov	bx,#.1F5
push	bx
! Debug: list int = const 2 (used reg = )
mov	ax,*2
push	ax
! Debug: func () void = bios_printf+0 (used reg = )
call	_bios_printf
add	sp,*4
!BCC_EOS
! 1691 }
mov	sp,bp
pop	bp
ret
! 1692 void ata_reset(device)
! Register BX used in function ata_detect
! 1693 Bit16u device;
export	_ata_reset
_ata_reset:
!BCC_EOS
! 1694 {
! 1695   Bit16u ebda_seg=read_word(0x0040,0x000E);
push	bp
mov	bp,sp
dec	sp
dec	sp
! Debug: list int = const $E (used reg = )
mov	ax,*$E
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () unsigned short = read_word+0 (used reg = )
call	_read_word
add	sp,*4
! Debug: eq unsigned short = ax+0 to unsigned short ebda_seg = [S+4-4] (used reg = )
mov	-2[bp],ax
!BCC_EOS
! 1696   Bit16u iobase1, iobase2;
!BCC_EOS
! 1697   Bit8u channel, slave, sn, sc;
!BCC_EOS
! 1698   Bit16u max;
!BCC_EOS
! 1699   channel = device / 2;
add	sp,*-$A
! Debug: div int = const 2 to unsigned short device = [S+$E+2] (used reg = )
mov	ax,4[bp]
shr	ax,*1
! Debug: eq unsigned int = ax+0 to unsigned char channel = [S+$E-9] (used reg = )
mov	-7[bp],al
!BCC_EOS
! 1700   slave = device % 2;
! Debug: mod int = const 2 to unsigned short device = [S+$E+2] (used reg = )
mov	ax,4[bp]
and	al,*1
! Debug: eq unsigned char = al+0 to unsigned char slave = [S+$E-$A] (used reg = )
mov	-8[bp],al
!BCC_EOS
! 1701   iobase1 = read_word(ebda_seg, &((ebda_data_t *) 0)->ata.channels[channel].iobase1);
! Debug: ptradd unsigned char channel = [S+$E-9] to [4] struct  = const $122 (used reg = )
mov	al,-7[bp]
xor	ah,ah
mov	cl,*3
shl	ax,cl
mov	bx,ax
! Debug: address unsigned short = [bx+$124] (used reg = )
! Debug: list * unsigned short = bx+$124 (used reg = )
add	bx,#$124
push	bx
! Debug: list unsigned short ebda_seg = [S+$10-4] (used reg = )
push	-2[bp]
! Debug: func () unsigned short = read_word+0 (used reg = )
call	_read_word
add	sp,*4
! Debug: eq unsigned short = ax+0 to unsigned short iobase1 = [S+$E-6] (used reg = )
mov	-4[bp],ax
!BCC_EOS
! 1702   iobase2 = read_word(ebda_seg, &((ebda_data_t *) 0)->ata.channels[channel].iobase2);
! Debug: ptradd unsigned char channel = [S+$E-9] to [4] struct  = const $122 (used reg = )
mov	al,-7[bp]
xor	ah,ah
mov	cl,*3
shl	ax,cl
mov	bx,ax
! Debug: address unsigned short = [bx+$126] (used reg = )
! Debug: list * unsigned short = bx+$126 (used reg = )
add	bx,#$126
push	bx
! Debug: list unsigned short ebda_seg = [S+$10-4] (used reg = )
push	-2[bp]
! Debug: func () unsigned short = read_word+0 (used reg = )
call	_read_word
add	sp,*4
! Debug: eq unsigned short = ax+0 to unsigned short iobase2 = [S+$E-8] (used reg = )
mov	-6[bp],ax
!BCC_EOS
! 1703   outb(iobase2+6, 0x08 | 0x02 | 0x04);
! Debug: list int = const $E (used reg = )
mov	ax,*$E
push	ax
! Debug: add int = const 6 to unsigned short iobase2 = [S+$10-8] (used reg = )
mov	ax,-6[bp]
! Debug: list unsigned int = ax+6 (used reg = )
add	ax,*6
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 1704   max=0xff;
! Debug: eq int = const $FF to unsigned short max = [S+$E-$E] (used reg = )
mov	ax,#$FF
mov	-$C[bp],ax
!BCC_EOS
! 1705   while(--max>0) {
jmp .1F7
.1F8:
! 1706     Bit8u status = inb(iobase1+7);
dec	sp
! Debug: add int = const 7 to unsigned short iobase1 = [S+$F-6] (used reg = )
mov	ax,-4[bp]
! Debug: list unsigned int = ax+7 (used reg = )
add	ax,*7
push	ax
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char status = [S+$F-$F] (used reg = )
mov	-$D[bp],al
!BCC_EOS
! 1707     if ((status & 0x80) != 0) break;
dec	sp
! Debug: and int = const $80 to unsigned char status = [S+$10-$F] (used reg = )
mov	al,-$D[bp]
and	al,#$80
! Debug: ne int = const 0 to unsigned char = al+0 (used reg = )
test	al,al
je  	.1F9
.1FA:
inc	sp
inc	sp
jmp .1F6
!BCC_EOS
! 1708   }
.1F9:
inc	sp
inc	sp
! 1709   outb(iobase2+6, 0x08 | 0x02);
.1F7:
! Debug: predec unsigned short max = [S+$E-$E] (used reg = )
mov	ax,-$C[bp]
dec	ax
mov	-$C[bp],ax
! Debug: gt int = const 0 to unsigned short = ax+0 (used reg = )
test	ax,ax
jne	.1F8
.1FB:
.1F6:
! Debug: list int = const $A (used reg = )
mov	ax,*$A
push	ax
! Debug: add int = const 6 to unsigned short iobase2 = [S+$10-8] (used reg = )
mov	ax,-6[bp]
! Debug: list unsigned int = ax+6 (used reg = )
add	ax,*6
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 1710   if (read_byte(ebda_seg,&((ebda_data_t *) 0)->ata.devices[device].type) != 0x00) {
! Debug: ptradd unsigned short device = [S+$E+2] to [8] struct  = const $142 (used reg = )
mov	ax,4[bp]
mov	cx,*$1A
imul	cx
mov	bx,ax
! Debug: address unsigned char = [bx+$142] (used reg = )
! Debug: list * unsigned char = bx+$142 (used reg = )
add	bx,#$142
push	bx
! Debug: list unsigned short ebda_seg = [S+$10-4] (used reg = )
push	-2[bp]
! Debug: func () unsigned char = read_byte+0 (used reg = )
call	_read_byte
add	sp,*4
! Debug: ne int = const 0 to unsigned char = al+0 (used reg = )
test	al,al
je  	.1FC
.1FD:
! 1711     outb(iobase1+6, slave?0xb0:0xa0);
mov	al,-8[bp]
test	al,al
je  	.1FE
.1FF:
mov	al,#$B0
jmp .200
.1FE:
mov	al,#$A0
.200:
! Debug: list char = al+0 (used reg = )
xor	ah,ah
push	ax
! Debug: add int = const 6 to unsigned short iobase1 = [S+$10-6] (used reg = )
mov	ax,-4[bp]
! Debug: list unsigned int = ax+6 (used reg = )
add	ax,*6
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 1712     sc = inb(iobase1+2);
! Debug: add int = const 2 to unsigned short iobase1 = [S+$E-6] (used reg = )
mov	ax,-4[bp]
! Debug: list unsigned int = ax+2 (used reg = )
inc	ax
inc	ax
push	ax
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char sc = [S+$E-$C] (used reg = )
mov	-$A[bp],al
!BCC_EOS
! 1713     sn = inb(iobase1+3);
! Debug: add int = const 3 to unsigned short iobase1 = [S+$E-6] (used reg = )
mov	ax,-4[bp]
! Debug: list unsigned int = ax+3 (used reg = )
add	ax,*3
push	ax
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char sn = [S+$E-$B] (used reg = )
mov	-9[bp],al
!BCC_EOS
! 1714     if ( (sc==0x01) && (sn==0x01) ) {
! Debug: logeq int = const 1 to unsigned char sc = [S+$E-$C] (used reg = )
mov	al,-$A[bp]
cmp	al,*1
jne 	.201
.203:
! Debug: logeq int = const 1 to unsigned char sn = [S+$E-$B] (used reg = )
mov	al,-9[bp]
cmp	al,*1
jne 	.201
.202:
! 1715       max=0xff;
! Debug: eq int = const $FF to unsigned short max = [S+$E-$E] (used reg = )
mov	ax,#$FF
mov	-$C[bp],ax
!BCC_EOS
! 1716       while(--max>0) {
jmp .205
.206:
! 1717         Bit8u status = inb(iobase1+7);
dec	sp
! Debug: add int = const 7 to unsigned short iobase1 = [S+$F-6] (used reg = )
mov	ax,-4[bp]
! Debug: list unsigned int = ax+7 (used reg = )
add	ax,*7
push	ax
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char status = [S+$F-$F] (used reg = )
mov	-$D[bp],al
!BCC_EOS
! 1718         if ((status & 0x80) == 0) break;
dec	sp
! Debug: and int = const $80 to unsigned char status = [S+$10-$F] (used reg = )
mov	al,-$D[bp]
and	al,#$80
! Debug: logeq int = const 0 to unsigned char = al+0 (used reg = )
test	al,al
jne 	.207
.208:
inc	sp
inc	sp
jmp .204
!BCC_EOS
! 1719         }
.207:
inc	sp
inc	sp
! 1720       }
.205:
! Debug: predec unsigned short max = [S+$E-$E] (used reg = )
mov	ax,-$C[bp]
dec	ax
mov	-$C[bp],ax
! Debug: gt int = const 0 to unsigned short = ax+0 (used reg = )
test	ax,ax
jne	.206
.209:
.204:
! 1721     }
.201:
! 1722   max=0xfff;
.1FC:
! Debug: eq int = const $FFF to unsigned short max = [S+$E-$E] (used reg = )
mov	ax,#$FFF
mov	-$C[bp],ax
!BCC_EOS
! 1723   while(--max>0) {
jmp .20B
.20C:
! 1724     Bit8u statu
! 1724 s = inb(iobase1+7);
dec	sp
! Debug: add int = const 7 to unsigned short iobase1 = [S+$F-6] (used reg = )
mov	ax,-4[bp]
! Debug: list unsigned int = ax+7 (used reg = )
add	ax,*7
push	ax
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char status = [S+$F-$F] (used reg = )
mov	-$D[bp],al
!BCC_EOS
! 1725       if ((status & 0x40) != 0) break;
dec	sp
! Debug: and int = const $40 to unsigned char status = [S+$10-$F] (used reg = )
mov	al,-$D[bp]
and	al,*$40
! Debug: ne int = const 0 to unsigned char = al+0 (used reg = )
test	al,al
je  	.20D
.20E:
inc	sp
inc	sp
jmp .20A
!BCC_EOS
! 1726   }
.20D:
inc	sp
inc	sp
! 1727   outb(iobase2+6, 0x08);
.20B:
! Debug: predec unsigned short max = [S+$E-$E] (used reg = )
mov	ax,-$C[bp]
dec	ax
mov	-$C[bp],ax
! Debug: gt int = const 0 to unsigned short = ax+0 (used reg = )
test	ax,ax
jne	.20C
.20F:
.20A:
! Debug: list int = const 8 (used reg = )
mov	ax,*8
push	ax
! Debug: add int = const 6 to unsigned short iobase2 = [S+$10-8] (used reg = )
mov	ax,-6[bp]
! Debug: list unsigned int = ax+6 (used reg = )
add	ax,*6
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 1728 }
mov	sp,bp
pop	bp
ret
! 1729 Bit16u ata_cmd_non_data()
! Register BX used in function ata_reset
! 1730 {return 0;}
export	_ata_cmd_non_data
_ata_cmd_non_data:
push	bp
mov	bp,sp
xor	ax,ax
pop	bp
ret
!BCC_EOS
! 1731 Bit16u ata_cmd_data_in(device, command, count, cylinder, head, sector, lba, segment, offset)
! 1732 Bit16u device, command, count, cylinder, head, sector, segment, offset;
export	_ata_cmd_data_in
_ata_cmd_data_in:
!BCC_EOS
! 1733 Bit32u lba;
!BCC_EOS
! 1734 {
! 1735   Bit16u ebda_seg=read_word(0x0040,0x000E);
push	bp
mov	bp,sp
dec	sp
dec	sp
! Debug: list int = const $E (used reg = )
mov	ax,*$E
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () unsigned short = read_word+0 (used reg = )
call	_read_word
add	sp,*4
! Debug: eq unsigned short = ax+0 to unsigned short ebda_seg = [S+4-4] (used reg = )
mov	-2[bp],ax
!BCC_EOS
! 1736   Bit16u iobase1, iobase2, blksize;
!BCC_EOS
! 1737   Bit8u channel, slave;
!BCC_EOS
! 1738   Bit8u status, current, mode;
!BCC_EOS
! 1739   channel = device / 2;
add	sp,*-$C
! Debug: div int = const 2 to unsigned short device = [S+$10+2] (used reg = )
mov	ax,4[bp]
shr	ax,*1
! Debug: eq unsigned int = ax+0 to unsigned char channel = [S+$10-$B] (used reg = )
mov	-9[bp],al
!BCC_EOS
! 1740   slave = device % 2;
! Debug: mod int = const 2 to unsigned short device = [S+$10+2] (used reg = )
mov	ax,4[bp]
and	al,*1
! Debug: eq unsigned char = al+0 to unsigned char slave = [S+$10-$C] (used reg = )
mov	-$A[bp],al
!BCC_EOS
! 1741   iobase1 = read_word(ebda_seg, &((ebda_data_t *) 0)->ata.channels[channel].iobase1);
! Debug: ptradd unsigned char channel = [S+$10-$B] to [4] struct  = const $122 (used reg = )
mov	al,-9[bp]
xor	ah,ah
mov	cl,*3
shl	ax,cl
mov	bx,ax
! Debug: address unsigned short = [bx+$124] (used reg = )
! Debug: list * unsigned short = bx+$124 (used reg = )
add	bx,#$124
push	bx
! Debug: list unsigned short ebda_seg = [S+$12-4] (used reg = )
push	-2[bp]
! Debug: func () unsigned short = read_word+0 (used reg = )
call	_read_word
add	sp,*4
! Debug: eq unsigned short = ax+0 to unsigned short iobase1 = [S+$10-6] (used reg = )
mov	-4[bp],ax
!BCC_EOS
! 1742   iobase2 = read_word(ebda_seg, &((ebda_data_t *) 0)->ata.channels[channel].iobase2);
! Debug: ptradd unsigned char channel = [S+$10-$B] to [4] struct  = const $122 (used reg = )
mov	al,-9[bp]
xor	ah,ah
mov	cl,*3
shl	ax,cl
mov	bx,ax
! Debug: address unsigned short = [bx+$126] (used reg = )
! Debug: list * unsigned short = bx+$126 (used reg = )
add	bx,#$126
push	bx
! Debug: list unsigned short ebda_seg = [S+$12-4] (used reg = )
push	-2[bp]
! Debug: func () unsigned short = read_word+0 (used reg = )
call	_read_word
add	sp,*4
! Debug: eq unsigned short = ax+0 to unsigned short iobase2 = [S+$10-8] (used reg = )
mov	-6[bp],ax
!BCC_EOS
! 1743   mode = read_byte(ebda_seg, &((ebda_data_t *) 0)->ata.devices[device].mode);
! Debug: ptradd unsigned short device = [S+$10+2] to [8] struct  = const $142 (used reg = )
mov	ax,4[bp]
mov	cx,*$1A
imul	cx
mov	bx,ax
! Debug: address unsigned char = [bx+$146] (used reg = )
! Debug: list * unsigned char = bx+$146 (used reg = )
add	bx,#$146
push	bx
! Debug: list unsigned short ebda_seg = [S+$12-4] (used reg = )
push	-2[bp]
! Debug: func () unsigned char = read_byte+0 (used reg = )
call	_read_byte
add	sp,*4
! Debug: eq unsigned char = al+0 to unsigned char mode = [S+$10-$F] (used reg = )
mov	-$D[bp],al
!BCC_EOS
! 1744   blksize = 0x200;
! Debug: eq int = const $200 to unsigned short blksize = [S+$10-$A] (used reg = )
mov	ax,#$200
mov	-8[bp],ax
!BCC_EOS
! 1745   if (mode == 0x01) blksize>>=2;
! Debug: logeq int = const 1 to unsigned char mode = [S+$10-$F] (used reg = )
mov	al,-$D[bp]
cmp	al,*1
jne 	.210
.211:
! Debug: srab int = const 2 to unsigned short blksize = [S+$10-$A] (used reg = )
mov	ax,-8[bp]
shr	ax,*1
shr	ax,*1
mov	-8[bp],ax
!BCC_EOS
! 1746   else blksize>>=1;
jmp .212
.210:
! Debug: srab int = const 1 to unsigned short blksize = [S+$10-$A] (used reg = )
mov	ax,-8[bp]
shr	ax,*1
mov	-8[bp],ax
!BCC_EOS
! 1747   write_word(ebda_seg, &((ebda_data_t *) 0)->ata.trsfsectors,0);
.212:
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: list * unsigned short = const $234 (used reg = )
mov	ax,#$234
push	ax
! Debug: list unsigned short ebda_seg = [S+$14-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
add	sp,*6
!BCC_EOS
! 1748   write_dword(ebda_seg, &((ebda_data_t *) 0)->ata.trsfbytes,0L);
! Debug: list long = const 0 (used reg = )
xor	ax,ax
xor	bx,bx
push	bx
push	ax
! Debug: list * unsigned long = const $236 (used reg = )
mov	ax,#$236
push	ax
! Debug: list unsigned short ebda_seg = [S+$16-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_dword+0 (used reg = )
call	_write_dword
add	sp,*8
!BCC_EOS
! 1749   current = 0;
! Debug: eq int = const 0 to unsigned char current = [S+$10-$E] (used reg = )
xor	al,al
mov	-$C[bp],al
!BCC_EOS
! 1750   status = inb(iobase1 + 7);
! Debug: add int = const 7 to unsigned short iobase1 = [S+$10-6] (used reg = )
mov	ax,-4[bp]
! Debug: list unsigned int = ax+7 (used reg = )
add	ax,*7
push	ax
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char status = [S+$10-$D] (used reg = )
mov	-$B[bp],al
!BCC_EOS
! 1751   if (status & 0x80) return 1;
! Debug: and int = const $80 to unsigned char status = [S+$10-$D] (used reg = )
mov	al,-$B[bp]
and	al,#$80
test	al,al
je  	.213
.214:
mov	ax,*1
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 1752   outb(iobase2 + 6, 0x08 | 0x02);
.213:
! Debug: list int = const $A (used reg = )
mov	ax,*$A
push	ax
! Debug: add int = const 6 to unsigned short iobase2 = [S+$12-8] (used reg = )
mov	ax,-6[bp]
! Debug: list unsigned int = ax+6 (used reg = )
add	ax,*6
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 1753   if (sector == 0) {
! Debug: logeq int = const 0 to unsigned short sector = [S+$10+$C] (used reg = )
mov	ax,$E[bp]
test	ax,ax
bne 	.215
.216:
! 1754     if ((count >= 1 << 8) || (lba + count >= 1UL << 28)) {
! Debug: ge int = const $100 to unsigned short count = [S+$10+6] (used reg = )
mov	ax,8[bp]
cmp	ax,#$100
jae 	.218
.219:
! Debug: cast unsigned long = const 0 to unsigned short count = [S+$10+6] (used reg = )
mov	ax,8[bp]
xor	bx,bx
! Debug: add unsigned long = bx+0 to unsigned long lba = [S+$10+$E] (used reg = )
! Debug: expression subtree swapping
lea	di,$10[bp]
call	laddul
! Debug: ge unsigned long = const $10000000 to unsigned long = bx+0 (used reg = )
push	bx
push	ax
xor	ax,ax
mov	bx,#$1000
push	bx
push	ax
mov	ax,-$12[bp]
mov	bx,-$10[bp]
lea	di,-$16[bp]
call	lcmpul
lea	sp,-$E[bp]
blo 	.217
.218:
! 1755       outb(iobase1 + 1, 0x00);
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: add int = const 1 to unsigned short iobase1 = [S+$12-6] (used reg = )
mov	ax,-4[bp]
! Debug: list unsigned int = ax+1 (used reg = )
inc	ax
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 1756       outb(iobase1 + 2, (count >> 8) & 0xff);
! Debug: sr int = const 8 to unsigned short count = [S+$10+6] (used reg = )
mov	ax,8[bp]
mov	al,ah
xor	ah,ah
! Debug: and int = const $FF to unsigned int = ax+0 (used reg = )
! Debug: list unsigned char = al+0 (used reg = )
xor	ah,ah
push	ax
! Debug: add int = const 2 to unsigned short iobase1 = [S+$12-6] (used reg = )
mov	ax,-4[bp]
! Debug: list unsigned int = ax+2 (used reg = )
inc	ax
inc	ax
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 1757       outb(iobase1 + 3, lba >> 24);
! Debug: sr int = const $18 to unsigned long lba = [S+$10+$E] (used reg = )
mov	ax,$10[bp]
mov	bx,$12[bp]
xchg	bx,ax
mov	al,ah
xor	ah,ah
xor	bx,bx
! Debug: list unsigned long = bx+0 (used reg = )
push	bx
push	ax
! Debug: add int = const 3 to unsigned short iobase1 = [S+$14-6] (used reg = )
mov	ax,-4[bp]
! Debug: list unsigned int = ax+3 (used reg = )
add	ax,*3
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*6
!BCC_EOS
! 1758       outb(iobase1 + 4, 0);
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: add int = const 4 to unsigned short iobase1 = [S+$12-6] (used reg = )
mov	ax,-4[bp]
! Debug: list unsigned int = ax+4 (used reg = )
add	ax,*4
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 1759       outb(iobase1 + 5, 0);
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: add int = const 5 to unsigned short iobase1 = [S+$12-6] (used reg = )
mov	ax,-4[bp]
! Debug: list unsigned int = ax+5 (used reg = )
add	ax,*5
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 1760       command |= 0x04;
! Debug: orab int = const 4 to unsigned short command = [S+$10+4] (used reg = )
mov	ax,6[bp]
or	al,*4
mov	6[bp],ax
!BCC_EOS
! 1761       count &= (1UL << 8) - 1;
! Debug: andab unsigned long = const $FF to unsigned short count = [S+$10+6] (used reg = )
mov	ax,8[bp]
xor	bx,bx
push	bx
push	ax
mov	ax,#$FF
xor	bx,bx
push	bx
push	ax
mov	ax,-$12[bp]
mov	bx,-$10[bp]
lea	di,-$16[bp]
call	landul
mov	8[bp],ax
add	sp,*8
!BCC_EOS
! 1762       lba &= (1UL << 24) - 1;
! Debug: andab unsigned long = const $FFFFFF to unsigned long lba = [S+$10+$E] (used reg = )
mov	ax,#$FFFF
mov	bx,#$FF
push	bx
push	ax
mov	ax,$10[bp]
mov	bx,$12[bp]
lea	di,-$12[bp]
call	landul
mov	$10[bp],ax
mov	$12[bp],bx
add	sp,*4
!BCC_EOS
! 1763       }
! 1764     sector = (Bit16u) (lba & 0x000000ffL);
.217:
! Debug: and long = const $FF to unsigned long lba = [S+$10+$E] (used reg = )
! Debug: expression subtree swapping
mov	ax,#$FF
xor	bx,bx
lea	di,$10[bp]
call	landul
! Debug: cast unsigned short = const 0 to unsigned long = bx+0 (used reg = )
! Debug: eq unsigned short = ax+0 to unsigned short sector = [S+$10+$C] (used reg = )
mov	$E[bp],ax
!BCC_EOS
! 1765     lba >>= 8;
! Debug: srab int = const 8 to unsigned long lba = [S+$10+$E] (used reg = )
mov	ax,$10[bp]
mov	bx,$12[bp]
mov	al,ah
mov	ah,bl
mov	bl,bh
sub	bh,bh
mov	$10[bp],ax
mov	$12[bp],bx
!BCC_EOS
! 1766     cylinder = (Bit16u) (lba & 0x0000ffffL);
! Debug: and long = const $FFFF to unsigned long lba = [S+$10+$E] (used reg = )
! Debug: expression subtree swapping
mov	ax,#$FFFF
xor	bx,bx
lea	di,$10[bp]
call	landul
! Debug: cast unsigned short = const 0 to unsigned long = bx+0 (used reg = )
! Debug: eq unsigned short = ax+0 to unsigned short cylinder = [S+$10+8] (used reg = )
mov	$A[bp],ax
!BCC_EOS
! 1767     lba >>= 16;
! Debug: srab int = const $10 to unsigned long lba = [S+$10+$E] (used reg = )
mov	ax,$10[bp]
mov	bx,$12[bp]
xchg	bx,ax
xor	bx,bx
mov	$10[bp],ax
mov	$12[bp],bx
!BCC_EOS
! 1768     head = ((Bit16u) (lba & 0x0000000fL)) | 0x40;
! Debug: and long = const $F to unsigned long lba = [S+$10+$E] (used reg = )
! Debug: expression subtree swapping
mov	ax,*$F
xor	bx,bx
lea	di,$10[bp]
call	landul
! Debug: cast unsigned short = const 0 to unsigned long = bx+0 (used reg = )
! Debug: or int = const $40 to unsigned short = ax+0 (used reg = )
or	al,*$40
! Debug: eq unsigned int = ax+0 to unsigned short head = [S+$10+$A] (used reg = )
mov	$C[bp],ax
!BCC_EOS
! 1769     }
! 1770   outb(iobase1 + 1, 0x00);
.215:
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: add int = const 1 to unsigned short iobase1 = [S+$12-6] (used reg = )
mov	ax,-4[bp]
! Debug: list unsigned int = ax+1 (used reg = )
inc	ax
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 1771   outb(iobase1 + 2, count);
! Debug: list unsigned short count = [S+$10+6] (used reg = )
push	8[bp]
! Debug: add int = const 2 to unsigned short iobase1 = [S+$12-6] (used reg = )
mov	ax,-4[bp]
! Debug: list unsigned int = ax+2 (used reg = )
inc	ax
inc	ax
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 1772   outb(iobase1 + 3, sector);
! Debug: list unsigned short sector = [S+$10+$C] (used reg = )
push	$E[bp]
! Debug: add int = const 3 to unsigned short iobase1 = [S+$12-6] (used reg = )
mov	ax,-4[bp]
! Debug: list unsigned int = ax+3 (used reg = )
add	ax,*3
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 1773   outb(iobase1 + 4, cylinder & 0x00ff);
! Debug: and int = const $FF to unsigned short cylinder = [S+$10+8] (used reg = )
mov	al,$A[bp]
! Debug: list unsigned char = al+0 (used reg = )
xor	ah,ah
push	ax
! Debug: add int = const 4 to unsigned short iobase1 = [S+$12-6] (used reg = )
mov	ax,-4[bp]
! Debug: list unsigned int = ax+4 (used reg = )
add	ax,*4
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 1774   outb(iobase1 + 5, cylinder >> 8);
! Debug: sr int = const 8 to unsigned short cylinder = [S+$10+8] (used reg = )
mov	ax,$A[bp]
mov	al,ah
xor	ah,ah
! Debug: list unsigned int = ax+0 (used reg = )
push	ax
! Debug: add int = const 5 to unsigned short iobase1 = [S+$12-6] (used reg = )
mov	ax,-4[bp]
! Debug: list unsigned int = ax+5 (used reg = )
add	ax,*5
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 1775   outb(iobase1 + 6, (slave ? 0xb0 : 0xa0) | (Bit8u) head );
mov	al,-$A[bp]
test	al,al
je  	.21A
.21B:
mov	al,#$B0
jmp .21C
.21A:
mov	al,#$A0
.21C:
! Debug: or unsigned char head = [S+$10+$A] to char = al+0 (used reg = )
or	al,$C[bp]
! Debug: list unsigned char = al+0 (used reg = )
xor	ah,ah
push	ax
! Debug: add int = const 6 to unsigned short iobase1 = [S+$12-6] (used reg = )
mov	ax,-4[bp]
! Debug: list unsigned int = ax+6 (used reg = )
add	ax,*6
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 1776   outb(iobase1 + 7, command);
! Debug: list unsigned short command = [S+$10+4] (used reg = )
push	6[bp]
! Debug: add int = const 7 to unsigned short iobase1 = [S+$12-6] (used reg = )
mov	ax,-4[bp]
! Debug: list unsigned int = ax+7 (used reg = )
add	ax,*7
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 1777   while (1) {
.21F:
! 1778     status = inb(iobase1 + 7);
! Debug: add int = const 7 to unsigned short iobase1 = [S+$10-6] (used reg = )
mov	ax,-4[bp]
! Debug: list unsigned int = ax+7 (used reg = )
add	ax,*7
push	ax
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char status = [S+$10-$D] (used reg = )
mov	-$B[bp],al
!BCC_EOS
! 1779     if ( !(status & 0x80) ) break;
! Debug: and int = const $80 to unsigned char status = [S+$10-$D] (used reg = )
mov	al,-$B[bp]
and	al,#$80
test	al,al
jne 	.220
.221:
jmp .21D
!BCC_EOS
! 1780     }
.220:
! 1781   if (status & 0x01) {
.21E:
jmp	.21F
.222:
.21D:
! Debug: and int = const 1 to unsigned char status = [S+$10-$D] (used reg = )
mov	al,-$B[bp]
and	al,*1
test	al,al
je  	.223
.224:
! 1782     ;
!BCC_EOS
! 1783     return 2;
mov	ax,*2
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 1784     } else if ( !(status & 0x08) ) {
jmp .225
.223:
! Debug: and int = const 8 to unsigned char status = [S+$10-$D] (used reg = )
mov	al,-$B[bp]
and	al,*8
test	al,al
jne 	.226
.227:
! 1785     ;
!BCC_EOS
! 1786     return 3;
mov	ax,*3
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 1787   }
! 1788 #asm
.226:
.225:
!BCC_EOS
!BCC_ASM
_ata_cmd_data_in.count	set	$16
.ata_cmd_data_in.count	set	8
_ata_cmd_data_in.segment	set	$22
.ata_cmd_data_in.segment	set	$14
_ata_cmd_data_in.iobase1	set	$A
.ata_cmd_data_in.iobase1	set	-4
_ata_cmd_data_in.channel	set	5
.ata_cmd_data_in.channel	set	-9
_ata_cmd_data_in.sector	set	$1C
.ata_cmd_data_in.sector	set	$E
_ata_cmd_data_in.blksize	set	6
.ata_cmd_data_in.blksize	set	-8
_ata_cmd_data_in.head	set	$1A
.ata_cmd_data_in.head	set	$C
_ata_cmd_data_in.cylinder	set	$18
.ata_cmd_data_in.cylinder	set	$A
_ata_cmd_data_in.device	set	$12
.ata_cmd_data_in.device	set	4
_ata_cmd_data_in.ebda_seg	set	$C
.ata_cmd_data_in.ebda_seg	set	-2
_ata_cmd_data_in.status	set	3
.ata_cmd_data_in.status	set	-$B
_ata_cmd_data_in.lba	set	$1E
.ata_cmd_data_in.lba	set	$10
_ata_cmd_data_in.current	set	2
.ata_cmd_data_in.current	set	-$C
_ata_cmd_data_in.command	set	$14
.ata_cmd_data_in.command	set	6
_ata_cmd_data_in.mode	set	1
.ata_cmd_data_in.mode	set	-$D
_ata_cmd_data_in.iobase2	set	8
.ata_cmd_data_in.iobase2	set	-6
_ata_cmd_data_in.offset	set	$24
.ata_cmd_data_in.offset	set	$16
_ata_cmd_data_in.slave	set	4
.ata_cmd_data_in.slave	set	-$A
        sti ;; enable higher priority interrupts
! 1790 endasm
!BCC_ENDASM
!BCC_EOS
! 1791   while (1) {
.22A:
! 1792 #asm
!BCC_EOS
!BCC_ASM
_ata_cmd_data_in.count	set	$16
.ata_cmd_data_in.count	set	8
_ata_cmd_data_in.segment	set	$22
.ata_cmd_data_in.segment	set	$14
_ata_cmd_data_in.iobase1	set	$A
.ata_cmd_data_in.iobase1	set	-4
_ata_cmd_data_in.channel	set	5
.ata_cmd_data_in.channel	set	-9
_ata_cmd_data_in.sector	set	$1C
.ata_cmd_data_in.sector	set	$E
_ata_cmd_data_in.blksize	set	6
.ata_cmd_data_in.blksize	set	-8
_ata_cmd_data_in.head	set	$1A
.ata_cmd_data_in.head	set	$C
_ata_cmd_data_in.cylinder	set	$18
.ata_cmd_data_in.cylinder	set	$A
_ata_cmd_data_in.device	set	$12
.ata_cmd_data_in.device	set	4
_ata_cmd_data_in.ebda_seg	set	$C
.ata_cmd_data_in.ebda_seg	set	-2
_ata_cmd_data_in.status	set	3
.ata_cmd_data_in.status	set	-$B
_ata_cmd_data_in.lba	set	$1E
.ata_cmd_data_in.lba	set	$10
_ata_cmd_data_in.current	set	2
.ata_cmd_data_in.current	set	-$C
_ata_cmd_data_in.command	set	$14
.ata_cmd_data_in.command	set	6
_ata_cmd_data_in.mode	set	1
.ata_cmd_data_in.mode	set	-$D
_ata_cmd_data_in.iobase2	set	8
.ata_cmd_data_in.iobase2	set	-6
_ata_cmd_data_in.offset	set	$24
.ata_cmd_data_in.offset	set	$16
_ata_cmd_data_in.slave	set	4
.ata_cmd_data_in.slave	set	-$A
        push bp
        mov bp, sp
        mov di, _ata_cmd_data_in.offset + 2[bp]
        mov ax, _ata_cmd_data_in.segment + 2[bp]
        mov cx, _ata_cmd_data_in.blksize + 2[bp]
        ;; adjust if there will be an overrun. 2K max sector size
        cmp di, #0xf800 ;;
        jbe ata_in_no_adjust
ata_in_adjust:
        sub di, #0x0800 ;; sub 2 kbytes from offset
        add ax, #0x0080 ;; add 2 Kbytes to segment
ata_in_no_adjust:
        mov es, ax ;; segment in es
        mov dx, _ata_cmd_data_in.iobase1 + 2[bp] ;; ATA data read port
        mov ah, _ata_cmd_data_in.mode + 2[bp]
        cmp ah, #0x01
        je ata_in_32
ata_in_16:
        rep
          insw ;; CX words transfered from port(DX) to ES:[DI]
        jmp ata_in_done
ata_in_32:
        rep
          insd ;; CX dwords transfered from port(DX) to ES:[DI]
ata_in_done:
        mov _ata_cmd_data_in.offset + 2[bp], di
        mov _ata_cmd_data_in.segment + 2[bp], es
        pop bp
! 1821 endasm
!BCC_ENDASM
!BCC_EOS
! 1822     current++;
! Debug: postinc unsigned char current = [S+$10-$E] (used reg = )
mov	al,-$C[bp]
inc	ax
mov	-$C[bp],al
!BCC_EOS
! 1823     write_word(ebda_seg, &((ebda_data_t *) 0)->ata.trsfsectors,current);
! Debug: list unsigned char current = [S+$10-$E] (used reg = )
mov	al,-$C[bp]
xor	ah,ah
push	ax
! Debug: list * unsigned short = const $234 (used reg = )
mov	ax,#$234
push	ax
! Debug: list unsigned short ebda_seg = [S+$14-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
add	sp,*6
!BCC_EOS
! 1824     count--;
! Debug: postdec unsigned short count = [S+$10+6] (used reg = )
mov	ax,8[bp]
dec	ax
mov	8[bp],ax
!BCC_EOS
! 1825     status = inb(iobase1 + 7);
! Debug: add int = const 7 to unsigned short iobase1 = [S+$10-6] (used reg = )
mov	ax,-4[bp]
! Debug: list unsigned int = ax+7 (used reg = )
add	ax,*7
push	ax
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char status = [S+$10-$D] (used reg = )
mov	-$B[bp],al
!BCC_EOS
! 1826     if (count == 0) {
! Debug: logeq int = const 0 to unsigned short count = [S+$10+6] (used reg = )
mov	ax,8[bp]
test	ax,ax
jne 	.22B
.22C:
! 1827       if ( (status & (0x80 | 0x40 | 0x08 | 0x01) )
! 1828           != 0x40 ) {
! Debug: and int = const $C9 to unsigned char status = [S+$10-$D] (used reg = )
mov	al,-$B[bp]
and	al,#$C9
! Debug: ne int = const $40 to unsigned char = al+0 (used reg = )
cmp	al,*$40
je  	.22D
.22E:
! 1829         ;
!BCC_EOS
! 1830         return 4;
mov	ax,*4
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 1831         }
! 1832       break;
.22D:
jmp .228
!BCC_EOS
! 1833       }
! 1834     else {
jmp .22F
.22B:
! 1835       if ( (status & (0x80 | 0x40 | 0x08 | 0x01) )
! 1836           != (0x40 | 0x08) ) {
! Debug: and int = const $C9 to unsigned char status = [S+$10-$D] (used reg = )
mov	al,-$B[bp]
and	al,#$C9
! Debug: ne int = const $48 to unsigned char = al+0 (used reg = )
cmp	al,*$48
je  	.230
.231:
! 1837         ;
!BCC_EOS
! 1838         return 5;
mov	ax,*5
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 1839       }
! 1840       continue;
.230:
jmp .229
!BCC_EOS
! 1841     }
! 1842   }
.22F:
! 1843   outb(iobase2+6, 0x08);
.229:
jmp	.22A
.232:
.228:
! Debug: list int = const 8 (used reg = )
mov	ax,*8
push	ax
! Debug: add int = const 6 to unsigned short iobase2 = [S+$12-8] (used reg = )
mov	ax,-6[bp]
! Debug: list unsigned int = ax+6 (used reg = )
add	ax,*6
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 1844   return 0;
xor	ax,ax
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 1845 }
! 1846 Bit16u ata_cmd_data_out(device, command, count, cylinder, head, sector, lba, segment, offset)
! Register BX used in function ata_cmd_data_in
! 1847 Bit16u device, command, count, cylinder, head, sector, segment, offset;
export	_ata_cmd_data_out
_ata_cmd_data_out:
!BCC_EOS
! 1848 Bit32u lba;
!BCC_EOS
! 1849 {
! 1850   Bit16u ebda_seg=read_word(0x0040,0x000E);
push	bp
mov	bp,sp
dec	sp
dec	sp
! Debug: list int = const $E (used reg = )
mov	ax,*$E
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () unsigned short = read_word+0 (used reg = )
call	_read_word
add	sp,*4
! Debug: eq unsigned short = ax+0 to unsigned short ebda_seg = [S+4-4] (used reg = )
mov	-2[bp],ax
!BCC_EOS
! 1851   Bit16u iobase1, iobase2, blksize;
!BCC_EOS
! 1852   Bit8u channel, slave;
!BCC_EOS
! 1853   Bit8u status, current, mode;
!BCC_EOS
! 1854   channel = device / 2;
add	sp,*-$C
! Debug: div int = const 2 to unsigned short device = [S+$10+2] (used reg = )
mov	ax,4[bp]
shr	ax,*1
! Debug: eq unsigned int = ax+0 to unsigned char channel = [S+$10-$B] (used reg = )
mov	-9[bp],al
!BCC_EOS
! 1855   slave = device % 2;
! Debug: mod int = const 2 to unsigned short device = [S+$10+2] (used reg = )
mov	ax,4[bp]
and	al,*1
! Debug: eq unsigned char = al+0 to unsigned char slave = [S+$10-$C] (used reg = )
mov	-$A[bp],al
!BCC_EOS
! 1856   iobase1 = read_word(ebda_seg, &((ebda_data_t *) 0)->ata.channels[channel].iobase1);
! Debug: ptradd unsigned char channel = [S+$10-$B] to [4] struct  = const $122 (used reg = )
mov	al,-9[bp]
xor	ah,ah
mov	cl,*3
shl	ax,cl
mov	bx,ax
! Debug: address unsigned short = [bx+$124] (used reg = )
! Debug: list * unsigned short = bx+$124 (used reg = )
add	bx,#$124
push	bx
! Debug: list unsigned short ebda_seg = [S+$12-4] (used reg = )
push	-2[bp]
! Debug: func () unsigned short = read_word+0 (used reg = )
call	_read_word
add	sp,*4
! Debug: eq unsigned short = ax+0 to unsigned short iobase1 = [S+$10-6] (used reg = )
mov	-4[bp],ax
!BCC_EOS
! 1857   iobase2 = read_word(ebda_seg, &((ebda_data_t *) 0)->ata.channels[channel].iobase2);
! Debug: ptradd unsigned char channel = [S+$10-$B] to [4] struct  = const $122 (used reg = )
mov	al,-9[bp]
xor	ah,ah
mov	cl,*3
shl	ax,cl
mov	bx,ax
! Debug: address unsigned short = [bx+$126] (used reg = )
! Debug: list * unsigned short = bx+$126 (used reg = )
add	bx,#$126
push	bx
! Debug: list unsigned short ebda_seg = [S+$12-4] (used reg = )
push	-2[bp]
! Debug: func () unsigned short = read_word+0 (used reg = )
call	_read_word
add	sp,*4
! Debug: eq unsigned short = ax+0 to unsigned short iobase2 = [S+$10-8] (used reg = )
mov	-6[bp],ax
!BCC_EOS
! 1858   mode = read_byte(ebda_seg, &((
! 1858 ebda_data_t *) 0)->ata.devices[device].mode);
! Debug: ptradd unsigned short device = [S+$10+2] to [8] struct  = const $142 (used reg = )
mov	ax,4[bp]
mov	cx,*$1A
imul	cx
mov	bx,ax
! Debug: address unsigned char = [bx+$146] (used reg = )
! Debug: list * unsigned char = bx+$146 (used reg = )
add	bx,#$146
push	bx
! Debug: list unsigned short ebda_seg = [S+$12-4] (used reg = )
push	-2[bp]
! Debug: func () unsigned char = read_byte+0 (used reg = )
call	_read_byte
add	sp,*4
! Debug: eq unsigned char = al+0 to unsigned char mode = [S+$10-$F] (used reg = )
mov	-$D[bp],al
!BCC_EOS
! 1859   blksize = 0x200;
! Debug: eq int = const $200 to unsigned short blksize = [S+$10-$A] (used reg = )
mov	ax,#$200
mov	-8[bp],ax
!BCC_EOS
! 1860   if (mode == 0x01) blksize>>=2;
! Debug: logeq int = const 1 to unsigned char mode = [S+$10-$F] (used reg = )
mov	al,-$D[bp]
cmp	al,*1
jne 	.233
.234:
! Debug: srab int = const 2 to unsigned short blksize = [S+$10-$A] (used reg = )
mov	ax,-8[bp]
shr	ax,*1
shr	ax,*1
mov	-8[bp],ax
!BCC_EOS
! 1861   else blksize>>=1;
jmp .235
.233:
! Debug: srab int = const 1 to unsigned short blksize = [S+$10-$A] (used reg = )
mov	ax,-8[bp]
shr	ax,*1
mov	-8[bp],ax
!BCC_EOS
! 1862   write_word(ebda_seg, &((ebda_data_t *) 0)->ata.trsfsectors,0);
.235:
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: list * unsigned short = const $234 (used reg = )
mov	ax,#$234
push	ax
! Debug: list unsigned short ebda_seg = [S+$14-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
add	sp,*6
!BCC_EOS
! 1863   write_dword(ebda_seg, &((ebda_data_t *) 0)->ata.trsfbytes,0L);
! Debug: list long = const 0 (used reg = )
xor	ax,ax
xor	bx,bx
push	bx
push	ax
! Debug: list * unsigned long = const $236 (used reg = )
mov	ax,#$236
push	ax
! Debug: list unsigned short ebda_seg = [S+$16-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_dword+0 (used reg = )
call	_write_dword
add	sp,*8
!BCC_EOS
! 1864   current = 0;
! Debug: eq int = const 0 to unsigned char current = [S+$10-$E] (used reg = )
xor	al,al
mov	-$C[bp],al
!BCC_EOS
! 1865   status = inb(iobase1 + 7);
! Debug: add int = const 7 to unsigned short iobase1 = [S+$10-6] (used reg = )
mov	ax,-4[bp]
! Debug: list unsigned int = ax+7 (used reg = )
add	ax,*7
push	ax
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char status = [S+$10-$D] (used reg = )
mov	-$B[bp],al
!BCC_EOS
! 1866   if (status & 0x80) return 1;
! Debug: and int = const $80 to unsigned char status = [S+$10-$D] (used reg = )
mov	al,-$B[bp]
and	al,#$80
test	al,al
je  	.236
.237:
mov	ax,*1
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 1867   outb(iobase2 + 6, 0x08 | 0x02);
.236:
! Debug: list int = const $A (used reg = )
mov	ax,*$A
push	ax
! Debug: add int = const 6 to unsigned short iobase2 = [S+$12-8] (used reg = )
mov	ax,-6[bp]
! Debug: list unsigned int = ax+6 (used reg = )
add	ax,*6
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 1868   if (sector == 0) {
! Debug: logeq int = const 0 to unsigned short sector = [S+$10+$C] (used reg = )
mov	ax,$E[bp]
test	ax,ax
bne 	.238
.239:
! 1869     if ((count >= 1 << 8) || (lba + count >= 1UL << 28)) {
! Debug: ge int = const $100 to unsigned short count = [S+$10+6] (used reg = )
mov	ax,8[bp]
cmp	ax,#$100
jae 	.23B
.23C:
! Debug: cast unsigned long = const 0 to unsigned short count = [S+$10+6] (used reg = )
mov	ax,8[bp]
xor	bx,bx
! Debug: add unsigned long = bx+0 to unsigned long lba = [S+$10+$E] (used reg = )
! Debug: expression subtree swapping
lea	di,$10[bp]
call	laddul
! Debug: ge unsigned long = const $10000000 to unsigned long = bx+0 (used reg = )
push	bx
push	ax
xor	ax,ax
mov	bx,#$1000
push	bx
push	ax
mov	ax,-$12[bp]
mov	bx,-$10[bp]
lea	di,-$16[bp]
call	lcmpul
lea	sp,-$E[bp]
blo 	.23A
.23B:
! 1870       outb(iobase1 + 1, 0x00);
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: add int = const 1 to unsigned short iobase1 = [S+$12-6] (used reg = )
mov	ax,-4[bp]
! Debug: list unsigned int = ax+1 (used reg = )
inc	ax
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 1871       outb(iobase1 + 2, (count >> 8) & 0xff);
! Debug: sr int = const 8 to unsigned short count = [S+$10+6] (used reg = )
mov	ax,8[bp]
mov	al,ah
xor	ah,ah
! Debug: and int = const $FF to unsigned int = ax+0 (used reg = )
! Debug: list unsigned char = al+0 (used reg = )
xor	ah,ah
push	ax
! Debug: add int = const 2 to unsigned short iobase1 = [S+$12-6] (used reg = )
mov	ax,-4[bp]
! Debug: list unsigned int = ax+2 (used reg = )
inc	ax
inc	ax
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 1872       outb(iobase1 + 3, lba >> 24);
! Debug: sr int = const $18 to unsigned long lba = [S+$10+$E] (used reg = )
mov	ax,$10[bp]
mov	bx,$12[bp]
xchg	bx,ax
mov	al,ah
xor	ah,ah
xor	bx,bx
! Debug: list unsigned long = bx+0 (used reg = )
push	bx
push	ax
! Debug: add int = const 3 to unsigned short iobase1 = [S+$14-6] (used reg = )
mov	ax,-4[bp]
! Debug: list unsigned int = ax+3 (used reg = )
add	ax,*3
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*6
!BCC_EOS
! 1873       outb(iobase1 + 4, 0);
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: add int = const 4 to unsigned short iobase1 = [S+$12-6] (used reg = )
mov	ax,-4[bp]
! Debug: list unsigned int = ax+4 (used reg = )
add	ax,*4
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 1874       outb(iobase1 + 5, 0);
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: add int = const 5 to unsigned short iobase1 = [S+$12-6] (used reg = )
mov	ax,-4[bp]
! Debug: list unsigned int = ax+5 (used reg = )
add	ax,*5
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 1875       command |= 0x04;
! Debug: orab int = const 4 to unsigned short command = [S+$10+4] (used reg = )
mov	ax,6[bp]
or	al,*4
mov	6[bp],ax
!BCC_EOS
! 1876       count &= (1UL << 8) - 1;
! Debug: andab unsigned long = const $FF to unsigned short count = [S+$10+6] (used reg = )
mov	ax,8[bp]
xor	bx,bx
push	bx
push	ax
mov	ax,#$FF
xor	bx,bx
push	bx
push	ax
mov	ax,-$12[bp]
mov	bx,-$10[bp]
lea	di,-$16[bp]
call	landul
mov	8[bp],ax
add	sp,*8
!BCC_EOS
! 1877       lba &= (1UL << 24) - 1;
! Debug: andab unsigned long = const $FFFFFF to unsigned long lba = [S+$10+$E] (used reg = )
mov	ax,#$FFFF
mov	bx,#$FF
push	bx
push	ax
mov	ax,$10[bp]
mov	bx,$12[bp]
lea	di,-$12[bp]
call	landul
mov	$10[bp],ax
mov	$12[bp],bx
add	sp,*4
!BCC_EOS
! 1878       }
! 1879     sector = (Bit16u) (lba & 0x000000ffL);
.23A:
! Debug: and long = const $FF to unsigned long lba = [S+$10+$E] (used reg = )
! Debug: expression subtree swapping
mov	ax,#$FF
xor	bx,bx
lea	di,$10[bp]
call	landul
! Debug: cast unsigned short = const 0 to unsigned long = bx+0 (used reg = )
! Debug: eq unsigned short = ax+0 to unsigned short sector = [S+$10+$C] (used reg = )
mov	$E[bp],ax
!BCC_EOS
! 1880     lba >>= 8;
! Debug: srab int = const 8 to unsigned long lba = [S+$10+$E] (used reg = )
mov	ax,$10[bp]
mov	bx,$12[bp]
mov	al,ah
mov	ah,bl
mov	bl,bh
sub	bh,bh
mov	$10[bp],ax
mov	$12[bp],bx
!BCC_EOS
! 1881     cylinder = (Bit16u) (lba & 0x0000ffffL);
! Debug: and long = const $FFFF to unsigned long lba = [S+$10+$E] (used reg = )
! Debug: expression subtree swapping
mov	ax,#$FFFF
xor	bx,bx
lea	di,$10[bp]
call	landul
! Debug: cast unsigned short = const 0 to unsigned long = bx+0 (used reg = )
! Debug: eq unsigned short = ax+0 to unsigned short cylinder = [S+$10+8] (used reg = )
mov	$A[bp],ax
!BCC_EOS
! 1882     lba >>= 16;
! Debug: srab int = const $10 to unsigned long lba = [S+$10+$E] (used reg = )
mov	ax,$10[bp]
mov	bx,$12[bp]
xchg	bx,ax
xor	bx,bx
mov	$10[bp],ax
mov	$12[bp],bx
!BCC_EOS
! 1883     head = ((Bit16u) (lba & 0x0000000fL)) | 0x40;
! Debug: and long = const $F to unsigned long lba = [S+$10+$E] (used reg = )
! Debug: expression subtree swapping
mov	ax,*$F
xor	bx,bx
lea	di,$10[bp]
call	landul
! Debug: cast unsigned short = const 0 to unsigned long = bx+0 (used reg = )
! Debug: or int = const $40 to unsigned short = ax+0 (used reg = )
or	al,*$40
! Debug: eq unsigned int = ax+0 to unsigned short head = [S+$10+$A] (used reg = )
mov	$C[bp],ax
!BCC_EOS
! 1884     }
! 1885   outb(iobase1 + 1, 0x00);
.238:
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: add int = const 1 to unsigned short iobase1 = [S+$12-6] (used reg = )
mov	ax,-4[bp]
! Debug: list unsigned int = ax+1 (used reg = )
inc	ax
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 1886   outb(iobase1 + 2, count);
! Debug: list unsigned short count = [S+$10+6] (used reg = )
push	8[bp]
! Debug: add int = const 2 to unsigned short iobase1 = [S+$12-6] (used reg = )
mov	ax,-4[bp]
! Debug: list unsigned int = ax+2 (used reg = )
inc	ax
inc	ax
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 1887   outb(iobase1 + 3, sector);
! Debug: list unsigned short sector = [S+$10+$C] (used reg = )
push	$E[bp]
! Debug: add int = const 3 to unsigned short iobase1 = [S+$12-6] (used reg = )
mov	ax,-4[bp]
! Debug: list unsigned int = ax+3 (used reg = )
add	ax,*3
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 1888   outb(iobase1 + 4, cylinder & 0x00ff);
! Debug: and int = const $FF to unsigned short cylinder = [S+$10+8] (used reg = )
mov	al,$A[bp]
! Debug: list unsigned char = al+0 (used reg = )
xor	ah,ah
push	ax
! Debug: add int = const 4 to unsigned short iobase1 = [S+$12-6] (used reg = )
mov	ax,-4[bp]
! Debug: list unsigned int = ax+4 (used reg = )
add	ax,*4
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 1889   outb(iobase1 + 5, cylinder >> 8);
! Debug: sr int = const 8 to unsigned short cylinder = [S+$10+8] (used reg = )
mov	ax,$A[bp]
mov	al,ah
xor	ah,ah
! Debug: list unsigned int = ax+0 (used reg = )
push	ax
! Debug: add int = const 5 to unsigned short iobase1 = [S+$12-6] (used reg = )
mov	ax,-4[bp]
! Debug: list unsigned int = ax+5 (used reg = )
add	ax,*5
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 1890   outb(iobase1 + 6, (slave ? 0xb0 : 0xa0) | (Bit8u) head );
mov	al,-$A[bp]
test	al,al
je  	.23D
.23E:
mov	al,#$B0
jmp .23F
.23D:
mov	al,#$A0
.23F:
! Debug: or unsigned char head = [S+$10+$A] to char = al+0 (used reg = )
or	al,$C[bp]
! Debug: list unsigned char = al+0 (used reg = )
xor	ah,ah
push	ax
! Debug: add int = const 6 to unsigned short iobase1 = [S+$12-6] (used reg = )
mov	ax,-4[bp]
! Debug: list unsigned int = ax+6 (used reg = )
add	ax,*6
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 1891   outb(iobase1 + 7, command);
! Debug: list unsigned short command = [S+$10+4] (used reg = )
push	6[bp]
! Debug: add int = const 7 to unsigned short iobase1 = [S+$12-6] (used reg = )
mov	ax,-4[bp]
! Debug: list unsigned int = ax+7 (used reg = )
add	ax,*7
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 1892   while (1) {
.242:
! 1893     status = inb(iobase1 + 7);
! Debug: add int = const 7 to unsigned short iobase1 = [S+$10-6] (used reg = )
mov	ax,-4[bp]
! Debug: list unsigned int = ax+7 (used reg = )
add	ax,*7
push	ax
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char status = [S+$10-$D] (used reg = )
mov	-$B[bp],al
!BCC_EOS
! 1894     if ( !(status & 0x80) ) break;
! Debug: and int = const $80 to unsigned char status = [S+$10-$D] (used reg = )
mov	al,-$B[bp]
and	al,#$80
test	al,al
jne 	.243
.244:
jmp .240
!BCC_EOS
! 1895     }
.243:
! 1896   if (status & 0x01) {
.241:
jmp	.242
.245:
.240:
! Debug: and int = const 1 to unsigned char status = [S+$10-$D] (used reg = )
mov	al,-$B[bp]
and	al,*1
test	al,al
je  	.246
.247:
! 1897     ;
!BCC_EOS
! 1898     return 2;
mov	ax,*2
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 1899     } else if ( !(status & 0x08) ) {
jmp .248
.246:
! Debug: and int = const 8 to unsigned char status = [S+$10-$D] (used reg = )
mov	al,-$B[bp]
and	al,*8
test	al,al
jne 	.249
.24A:
! 1900     ;
!BCC_EOS
! 1901     return 3;
mov	ax,*3
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 1902     }
! 1903 #asm
.249:
.248:
!BCC_EOS
!BCC_ASM
_ata_cmd_data_out.count	set	$16
.ata_cmd_data_out.count	set	8
_ata_cmd_data_out.segment	set	$22
.ata_cmd_data_out.segment	set	$14
_ata_cmd_data_out.iobase1	set	$A
.ata_cmd_data_out.iobase1	set	-4
_ata_cmd_data_out.channel	set	5
.ata_cmd_data_out.channel	set	-9
_ata_cmd_data_out.sector	set	$1C
.ata_cmd_data_out.sector	set	$E
_ata_cmd_data_out.blksize	set	6
.ata_cmd_data_out.blksize	set	-8
_ata_cmd_data_out.head	set	$1A
.ata_cmd_data_out.head	set	$C
_ata_cmd_data_out.cylinder	set	$18
.ata_cmd_data_out.cylinder	set	$A
_ata_cmd_data_out.device	set	$12
.ata_cmd_data_out.device	set	4
_ata_cmd_data_out.ebda_seg	set	$C
.ata_cmd_data_out.ebda_seg	set	-2
_ata_cmd_data_out.status	set	3
.ata_cmd_data_out.status	set	-$B
_ata_cmd_data_out.lba	set	$1E
.ata_cmd_data_out.lba	set	$10
_ata_cmd_data_out.current	set	2
.ata_cmd_data_out.current	set	-$C
_ata_cmd_data_out.command	set	$14
.ata_cmd_data_out.command	set	6
_ata_cmd_data_out.mode	set	1
.ata_cmd_data_out.mode	set	-$D
_ata_cmd_data_out.iobase2	set	8
.ata_cmd_data_out.iobase2	set	-6
_ata_cmd_data_out.offset	set	$24
.ata_cmd_data_out.offset	set	$16
_ata_cmd_data_out.slave	set	4
.ata_cmd_data_out.slave	set	-$A
        sti ;; enable higher priority interrupts
! 1905 endasm
!BCC_ENDASM
!BCC_EOS
! 1906   while (1) {
.24D:
! 1907 #asm
!BCC_EOS
!BCC_ASM
_ata_cmd_data_out.count	set	$16
.ata_cmd_data_out.count	set	8
_ata_cmd_data_out.segment	set	$22
.ata_cmd_data_out.segment	set	$14
_ata_cmd_data_out.iobase1	set	$A
.ata_cmd_data_out.iobase1	set	-4
_ata_cmd_data_out.channel	set	5
.ata_cmd_data_out.channel	set	-9
_ata_cmd_data_out.sector	set	$1C
.ata_cmd_data_out.sector	set	$E
_ata_cmd_data_out.blksize	set	6
.ata_cmd_data_out.blksize	set	-8
_ata_cmd_data_out.head	set	$1A
.ata_cmd_data_out.head	set	$C
_ata_cmd_data_out.cylinder	set	$18
.ata_cmd_data_out.cylinder	set	$A
_ata_cmd_data_out.device	set	$12
.ata_cmd_data_out.device	set	4
_ata_cmd_data_out.ebda_seg	set	$C
.ata_cmd_data_out.ebda_seg	set	-2
_ata_cmd_data_out.status	set	3
.ata_cmd_data_out.status	set	-$B
_ata_cmd_data_out.lba	set	$1E
.ata_cmd_data_out.lba	set	$10
_ata_cmd_data_out.current	set	2
.ata_cmd_data_out.current	set	-$C
_ata_cmd_data_out.command	set	$14
.ata_cmd_data_out.command	set	6
_ata_cmd_data_out.mode	set	1
.ata_cmd_data_out.mode	set	-$D
_ata_cmd_data_out.iobase2	set	8
.ata_cmd_data_out.iobase2	set	-6
_ata_cmd_data_out.offset	set	$24
.ata_cmd_data_out.offset	set	$16
_ata_cmd_data_out.slave	set	4
.ata_cmd_data_out.slave	set	-$A
        push bp
        mov bp, sp
        mov si, _ata_cmd_data_out.offset + 2[bp]
        mov ax, _ata_cmd_data_out.segment + 2[bp]
        mov cx, _ata_cmd_data_out.blksize + 2[bp]
        ;; adjust if there will be an overrun. 2K max sector size
        cmp si, #0xf800 ;;
        jbe ata_out_no_adjust
ata_out_adjust:
        sub si, #0x0800 ;; sub 2 kbytes from offset
        add ax, #0x0080 ;; add 2 Kbytes to segment
ata_out_no_adjust:
        mov es, ax ;; segment in es
        mov dx, _ata_cmd_data_out.iobase1 + 2[bp] ;; ATA data write port
        mov ah, _ata_cmd_data_out.mode + 2[bp]
        cmp ah, #0x01
        je ata_out_32
ata_out_16:
        seg ES
        rep
          outsw ;; CX words transfered from port(DX) to ES:[SI]
        jmp ata_out_done
ata_out_32:
        seg ES
        rep
          outsd ;; CX dwords transfered from port(DX) to ES:[SI]
ata_out_done:
        mov _ata_cmd_data_out.offset + 2[bp], si
        mov _ata_cmd_data_out.segment + 2[bp], es
        pop bp
! 1938 endasm
!BCC_ENDASM
!BCC_EOS
! 1939     current++;
! Debug: postinc unsigned char current = [S+$10-$E] (used reg = )
mov	al,-$C[bp]
inc	ax
mov	-$C[bp],al
!BCC_EOS
! 1940     write_word(ebda_seg, &((ebda_data_t *) 0)->ata.trsfsectors,current);
! Debug: list unsigned char current = [S+$10-$E] (used reg = )
mov	al,-$C[bp]
xor	ah,ah
push	ax
! Debug: list * unsigned short = const $234 (used reg = )
mov	ax,#$234
push	ax
! Debug: list unsigned short ebda_seg = [S+$14-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
add	sp,*6
!BCC_EOS
! 1941     count--;
! Debug: postdec unsigned short count = [S+$10+6] (used reg = )
mov	ax,8[bp]
dec	ax
mov	8[bp],ax
!BCC_EOS
! 1942     status = inb(iobase1 + 7);
! Debug: add int = const 7 to unsigned short iobase1 = [S+$10-6] (used reg = )
mov	ax,-4[bp]
! Debug: list unsigned int = ax+7 (used reg = )
add	ax,*7
push	ax
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char status = [S+$10-$D] (used reg = )
mov	-$B[bp],al
!BCC_EOS
! 1943     if (count == 0) {
! Debug: logeq int = const 0 to unsigned short count = [S+$10+6] (used reg = )
mov	ax,8[bp]
test	ax,ax
jne 	.24E
.24F:
! 1944       if ( (status & (0x80 | 0x40 | 0x20 | 0x08 | 0x01) )
! 1945           != 0x40 ) {
! Debug: and int = const $E9 to unsigned char status = [S+$10-$D] (used reg = )
mov	al,-$B[bp]
and	al,#$E9
! Debug: ne int = const $40 to unsigned char = al+0 (used reg = )
cmp	al,*$40
je  	.250
.251:
! 1946         ;
!BCC_EOS
! 1947         return 6;
mov	ax,*6
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 1948         }
! 1949       break;
.250:
jmp .24B
!BCC_EOS
! 1950       }
! 1951     else {
jmp .252
.24E:
! 1952       if ( (status & (0x80 | 0x40 | 0x08 | 0x01) )
! 1953           != (0x40 | 0x08) ) {
! Debug: and int = const $C9 to unsigned char status = [S+$10-$D] (used reg = )
mov	al,-$B[bp]
and	al,#$C9
! Debug: ne int = const $48 to unsigned char = al+0 (used reg = )
cmp	al,*$48
je  	.253
.254:
! 1954         ;
!BCC_EOS
! 1955         return 7;
mov	ax,*7
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 1956       }
! 1957       continue;
.253:
jmp .24C
!BCC_EOS
! 1958     }
! 1959   }
.252:
! 1960   outb(iobase2+6, 0x08);
.24C:
jmp	.24D
.255:
.24B:
! Debug: list int = const 8 (used reg = )
mov	ax,*8
push	ax
! Debug: add int = const 6 to unsigned short iobase2 = [S+$12-8] (used reg = )
mov	ax,-6[bp]
! Debug: list unsigned int = ax+6 (used reg = )
add	ax,*6
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 1961   return 0;
xor	ax,ax
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 1962 }
! 1963 Bit16u ata_cmd_packet(device, cmdlen, cmdseg, cmdoff, header, length, inout, bufseg, bufoff)
! Register BX used in function ata_cmd_data_out
! 1964 Bit8u cmdlen,inout;
export	_ata_cmd_packet
_ata_cmd_packet:
!BCC_EOS
! 1965 Bit16u device,cmdseg, cmdoff, bufseg, bufoff;
!BCC_EOS
! 1966 Bit16u header;
!BCC_EOS
! 1967 Bit32u length;
!BCC_EOS
! 1968 {
! 1969   Bit16u ebda_seg=read_word(0x0040,0x000E);
push	bp
mov	bp,sp
dec	sp
dec	sp
! Debug: list int = const $E (used reg = )
mov	ax,*$E
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () unsigned short = read_word+0 (used reg = )
call	_read_word
add	sp,*4
! Debug: eq unsigned short = ax+0 to unsigned short ebda_seg = [S+4-4] (used reg = )
mov	-2[bp],ax
!BCC_EOS
! 1970   Bit16u iobase1, iobase2;
!BCC_EOS
! 1971   Bit16u lcount, lbefore, lafter, count;
!BCC_EOS
! 1972   Bit8u channel, slave;
!BCC_EOS
! 1973   Bit8u status, mode, lmode;
!BCC_EOS
! 1974   Bit32u total, transfer;
!BCC_EOS
! 1975   channel = device / 2;
add	sp,*-$1A
! Debug: div int = const 2 to unsigned short device = [S+$1E+2] (used reg = )
mov	ax,4[bp]
shr	ax,*1
! Debug: eq unsigned int = ax+0 to unsigned char channel = [S+$1E-$11] (used reg = )
mov	-$F[bp],al
!BCC_EOS
! 1976   slave = device % 2;
! Debug: mod int = const 2 to unsigned short device = [S+$1E+2] (used reg = )
mov	ax,4[bp]
and	al,*1
! Debug: eq unsigned char = al+0 to unsigned char slave = [S+$1E-$12] (used reg = )
mov	-$10[bp],al
!BCC_EOS
! 1977   if (inout == 0x02) {
! Debug: logeq int = const 2 to unsigned char inout = [S+$1E+$10] (used reg = )
mov	al,$12[bp]
cmp	al,*2
jne 	.256
.257:
! 1978     bios_printf(4, "ata_cmd_packet: DATA_OUT not supported yet\n");
! Debug: list * char = .258+0 (used reg = )
mov	bx,#.258
push	bx
! Debug: list int = const 4 (used reg = )
mov	ax,*4
push	ax
! Debug: func () void = bios_printf+0 (used reg = )
call	_bios_printf
add	sp,*4
!BCC_EOS
! 1979     return 1;
mov	ax,*1
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 1980     }
! 1981   if (header & 1) {
.256:
! Debug: and int = const 1 to unsigned short header = [S+$1E+$A] (used reg = )
mov	al,$C[bp]
and	al,*1
test	al,al
je  	.259
.25A:
! 1982     ;
!BCC_EOS
! 1983     return 1;
mov	ax,*1
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 1984     }
! 1985   iobase1 = read_word(ebda_seg, &((ebda_data_t *) 0)->ata.channels[channel].iobase1);
.259:
! Debug: ptradd unsigned char channel = [S+$1E-$11] to [4] struct  = const $122 (used reg = )
mov	al,-$F[bp]
xor	ah,ah
mov	cl,*3
shl	ax,cl
mov	bx,ax
! Debug: address unsigned short = [bx+$124] (used reg = )
! Debug: list * unsigned short = bx+$124 (used reg = )
add	bx,#$124
push	bx
! Debug: list unsigned short ebda_seg = [S+$20-4] (used reg = )
push	-2[bp]
! Debug: func () unsigned short = read_word+0 (used reg = )
call	_read_word
add	sp,*4
! Debug: eq unsigned short = ax+0 to unsigned short iobase1 = [S+$1E-6] (used reg = )
mov	-4[bp],ax
!BCC_EOS
! 1986   iobase2 = read_word(ebda_seg, &((ebda_data_t *) 0)->ata.channels[channel].iobase2);
! Debug: ptradd unsigned char channel = [S+$1E-$11] to [4] struct  = const $122 (used reg = )
mov	al,-$F[bp]
xor	ah,ah
mov	cl,*3
shl	ax,cl
mov	bx,ax
! Debug: address unsigned short = [bx+$126] (used reg = )
! Debug: list * unsigned short = bx+$126 (used reg = )
add	bx,#$126
push	bx
! Debug: list unsigned short ebda_seg = [S+$20-4] (used reg = )
push	-2[bp]
! Debug: func () unsigned short = read_word+0 (used reg = )
call	_read_word
add	sp,*4
! Debug: eq unsigned short = ax+0 to unsigned short iobase2 = [S+$1E-8] (used reg = )
mov	-6[bp],ax
!BCC_EOS
! 1987   mode = read_byte(ebda_seg, &((ebda_data_t *) 0)->ata.devices[device].mode);
! Debug: ptradd unsigned short device = [S+$1E+2] to [8] struct  = const $142 (used reg = )
mov	ax,4[bp]
mov	cx,*$1A
imul	cx
mov	bx,ax
! Debug: address unsigned char = [bx+$146] (used reg = )
! Debug: list * unsigned char = bx+$146 (used reg = )
add	bx,#$146
push	bx
! Debug: list unsigned short ebda_seg = [S+$20-4] (used reg = )
push	-2[bp]
! Debug: func () unsigned char = read_byte+0 (used reg = )
call	_read_byte
add	sp,*4
! Debug: eq unsigned char = al+0 to unsigned char mode = [S+$1E-$14] (used reg = )
mov	-$12[bp],al
!BCC_EOS
! 1988   transfer= 0L;
! Debug: eq long = const 0 to unsigned long transfer = [S+$1E-$1E] (used reg = )
xor	ax,ax
xor	bx,bx
mov	-$1C[bp],ax
mov	-$1A[bp],bx
!BCC_EOS
! 1989   if (cmdlen < 12) cmdlen=12;
! Debug: lt int = const $C to unsigned char cmdlen = [S+$1E+4] (used reg = )
mov	al,6[bp]
cmp	al,*$C
jae 	.25B
.25C:
! Debug: eq int = const $C to unsigned char cmdlen = [S+$1E+4] (used reg = )
mov	al,*$C
mov	6[bp],al
!BCC_EOS
! 1990   if (cmdlen > 12) cmdlen=16;
.25B:
! Debug: gt int = const $C to unsigned char cmdlen = [S+$1E+4] (used reg = )
mov	al,6[bp]
cmp	al,*$C
jbe 	.25D
.25E:
! Debug: eq int = const $10 to unsigned char cmdlen = [S+$1E+4] (used reg = )
mov	al,*$10
mov	6[bp],al
!BCC_EOS
! 1991   cmdlen>>=1;
.25D:
! Debug: srab int = const 1 to unsigned char cmdlen = [S+$1E+4] (used reg = )
mov	al,6[bp]
xor	ah,ah
shr	ax,*1
mov	6[bp],al
!BCC_EOS
! 1992   write_word(ebda_seg, &((ebda_data_t *) 0)->ata.trsfsectors,0);
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: list * unsigned short = const $234 (used reg = )
mov	ax,#$234
push	ax
! Debug: list unsigned short ebda_seg = [S+$22-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
add	sp,*6
!BCC_EOS
! 1993   write_dword(ebda_seg, &((ebda_data_t *) 0)->ata.trsfbytes,0L);
! Debug: list long = const 0 (used reg = )
xor	ax,ax
xor	bx,bx
push	bx
push	ax
! Debug: list * unsigned long = const $236 (used reg = )
mov	ax,#$236
push	ax
! Debug: list unsigned short ebda_seg = [S+$24-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_dword+0 (used reg = )
call	_write_dword
add	sp,*8
!BCC_EOS
! 1994   status = inb(iobase1 + 7);
! Debug: add int = const 7 to unsigned short iobase1 = [S+$1E-6] (used reg = )
mov	ax,-4[bp]
! Debug: list unsigned int = ax+7 (used reg = )
add	ax,*7
push	ax
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char status = [S+$1E-$13] (used reg = )
mov	-$11[bp],al
!BCC_EOS
! 1995   if (status & 0x80) return 2;
! Debug: and int = const $80 to unsigned char status = [S+$1E-$13] (used reg = )
mov	al,-$11[bp]
and	al,#$80
test	al,al
je  	.25F
.260:
mov	ax,*2
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 1996   outb(iobase2 + 6, 0x08 | 0x02);
.25F:
! Debug: list int = const $A (used reg = )
mov	ax,*$A
push	ax
! Debug: add int = const 6 to unsigned short iobase2 = [S+$20-8] (used reg = )
mov	ax,-6[bp]
! Debug: list unsigned int = ax+6 (used reg = )
add	ax,*6
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 1997   outb(iobase1 + 4, 0xfff0 & 0x00ff);
! Debug: list unsigned int = const $F0 (used reg = )
mov	ax,#$F0
push	ax
! Debug: add int = const 4 to unsigned short iobase1 = [S+$20-6] (used reg = )
mov	ax,-4[bp]
! Debug: list unsigned int = ax+4 (used reg = )
add	ax,*4
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 1998   outb(iobase
! 1998 1 + 5, 0xfff0 >> 8);
! Debug: list unsigned int = const $FF (used reg = )
mov	ax,#$FF
push	ax
! Debug: add int = const 5 to unsigned short iobase1 = [S+$20-6] (used reg = )
mov	ax,-4[bp]
! Debug: list unsigned int = ax+5 (used reg = )
add	ax,*5
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 1999   outb(iobase1 + 6, slave ? 0xb0 : 0xa0);
mov	al,-$10[bp]
test	al,al
je  	.261
.262:
mov	al,#$B0
jmp .263
.261:
mov	al,#$A0
.263:
! Debug: list char = al+0 (used reg = )
xor	ah,ah
push	ax
! Debug: add int = const 6 to unsigned short iobase1 = [S+$20-6] (used reg = )
mov	ax,-4[bp]
! Debug: list unsigned int = ax+6 (used reg = )
add	ax,*6
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 2000   outb(iobase1 + 7, 0xA0);
! Debug: list int = const $A0 (used reg = )
mov	ax,#$A0
push	ax
! Debug: add int = const 7 to unsigned short iobase1 = [S+$20-6] (used reg = )
mov	ax,-4[bp]
! Debug: list unsigned int = ax+7 (used reg = )
add	ax,*7
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 2001   while (1) {
.266:
! 2002     status = inb(iobase1 + 7);
! Debug: add int = const 7 to unsigned short iobase1 = [S+$1E-6] (used reg = )
mov	ax,-4[bp]
! Debug: list unsigned int = ax+7 (used reg = )
add	ax,*7
push	ax
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char status = [S+$1E-$13] (used reg = )
mov	-$11[bp],al
!BCC_EOS
! 2003     if ( !(status & 0x80) ) break;
! Debug: and int = const $80 to unsigned char status = [S+$1E-$13] (used reg = )
mov	al,-$11[bp]
and	al,#$80
test	al,al
jne 	.267
.268:
jmp .264
!BCC_EOS
! 2004     }
.267:
! 2005   if (status & 0x01) {
.265:
jmp	.266
.269:
.264:
! Debug: and int = const 1 to unsigned char status = [S+$1E-$13] (used reg = )
mov	al,-$11[bp]
and	al,*1
test	al,al
je  	.26A
.26B:
! 2006     ;
!BCC_EOS
! 2007     return 3;
mov	ax,*3
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 2008     } else if ( !(status & 0x08) ) {
jmp .26C
.26A:
! Debug: and int = const 8 to unsigned char status = [S+$1E-$13] (used reg = )
mov	al,-$11[bp]
and	al,*8
test	al,al
jne 	.26D
.26E:
! 2009     ;
!BCC_EOS
! 2010     return 4;
mov	ax,*4
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 2011     }
! 2012   cmdseg += (cmdoff / 16);
.26D:
.26C:
! Debug: div int = const $10 to unsigned short cmdoff = [S+$1E+8] (used reg = )
mov	ax,$A[bp]
mov	cl,*4
shr	ax,cl
! Debug: addab unsigned int = ax+0 to unsigned short cmdseg = [S+$1E+6] (used reg = )
add	ax,8[bp]
mov	8[bp],ax
!BCC_EOS
! 2013   cmdoff %= 16;
! Debug: modab int = const $10 to unsigned short cmdoff = [S+$1E+8] (used reg = )
mov	ax,$A[bp]
and	al,*$F
xor	ah,ah
mov	$A[bp],ax
!BCC_EOS
! 2014 #asm
!BCC_EOS
!BCC_ASM
_ata_cmd_packet.cmdoff	set	$26
.ata_cmd_packet.cmdoff	set	$A
_ata_cmd_packet.header	set	$28
.ata_cmd_packet.header	set	$C
_ata_cmd_packet.count	set	$E
.ata_cmd_packet.count	set	-$E
_ata_cmd_packet.lafter	set	$10
.ata_cmd_packet.lafter	set	-$C
_ata_cmd_packet.iobase1	set	$18
.ata_cmd_packet.iobase1	set	-4
_ata_cmd_packet.channel	set	$D
.ata_cmd_packet.channel	set	-$F
_ata_cmd_packet.cmdseg	set	$24
.ata_cmd_packet.cmdseg	set	8
_ata_cmd_packet.cmdlen	set	$22
.ata_cmd_packet.cmdlen	set	6
_ata_cmd_packet.lmode	set	9
.ata_cmd_packet.lmode	set	-$13
_ata_cmd_packet.device	set	$20
.ata_cmd_packet.device	set	4
_ata_cmd_packet.ebda_seg	set	$1A
.ata_cmd_packet.ebda_seg	set	-2
_ata_cmd_packet.lcount	set	$14
.ata_cmd_packet.lcount	set	-8
_ata_cmd_packet.total	set	4
.ata_cmd_packet.total	set	-$18
_ata_cmd_packet.status	set	$B
.ata_cmd_packet.status	set	-$11
_ata_cmd_packet.mode	set	$A
.ata_cmd_packet.mode	set	-$12
_ata_cmd_packet.bufoff	set	$32
.ata_cmd_packet.bufoff	set	$16
_ata_cmd_packet.transfer	set	0
.ata_cmd_packet.transfer	set	-$1C
_ata_cmd_packet.iobase2	set	$16
.ata_cmd_packet.iobase2	set	-6
_ata_cmd_packet.lbefore	set	$12
.ata_cmd_packet.lbefore	set	-$A
_ata_cmd_packet.bufseg	set	$30
.ata_cmd_packet.bufseg	set	$14
_ata_cmd_packet.slave	set	$C
.ata_cmd_packet.slave	set	-$10
_ata_cmd_packet.inout	set	$2E
.ata_cmd_packet.inout	set	$12
_ata_cmd_packet.length	set	$2A
.ata_cmd_packet.length	set	$E
      sti ;; enable higher priority interrupts
      push bp
      mov bp, sp
      mov si, _ata_cmd_packet.cmdoff + 2[bp]
      mov ax, _ata_cmd_packet.cmdseg + 2[bp]
      mov cx, _ata_cmd_packet.cmdlen + 2[bp]
      mov es, ax ;; segment in es
      mov dx, _ata_cmd_packet.iobase1 + 2[bp] ;; ATA data write port
      seg ES
      rep
        outsw ;; CX words transfered from port(DX) to ES:[SI]
      pop bp
! 2027 endasm
!BCC_ENDASM
!BCC_EOS
! 2028   if (inout == 0x00) {
! Debug: logeq int = const 0 to unsigned char inout = [S+$1E+$10] (used reg = )
mov	al,$12[bp]
test	al,al
jne 	.26F
.270:
! 2029     status = inb(iobase1 + 7);
! Debug: add int = const 7 to unsigned short iobase1 = [S+$1E-6] (used reg = )
mov	ax,-4[bp]
! Debug: list unsigned int = ax+7 (used reg = )
add	ax,*7
push	ax
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char status = [S+$1E-$13] (used reg = )
mov	-$11[bp],al
!BCC_EOS
! 2030     }
! 2031   else {
br 	.271
.26F:
! 2032   while (1) {
.274:
! 2033       status = inb(iobase1 + 7);
! Debug: add int = const 7 to unsigned short iobase1 = [S+$1E-6] (used reg = )
mov	ax,-4[bp]
! Debug: list unsigned int = ax+7 (used reg = )
add	ax,*7
push	ax
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char status = [S+$1E-$13] (used reg = )
mov	-$11[bp],al
!BCC_EOS
! 2034       if ( (status & (0x80 | 0x08) ) ==0 ) break;
! Debug: and int = const $88 to unsigned char status = [S+$1E-$13] (used reg = )
mov	al,-$11[bp]
and	al,#$88
! Debug: logeq int = const 0 to unsigned char = al+0 (used reg = )
test	al,al
jne 	.275
.276:
br 	.272
!BCC_EOS
! 2035       if (status & 0x01) {
.275:
! Debug: and int = const 1 to unsigned char status = [S+$1E-$13] (used reg = )
mov	al,-$11[bp]
and	al,*1
test	al,al
je  	.277
.278:
! 2036         ;
!BCC_EOS
! 2037         return 3;
mov	ax,*3
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 2038       }
! 2039       if ( (status & (0x80 | 0x40 | 0x08 | 0x01) )
.277:
! 2040             != (0x40 | 0x08) ) {
! Debug: and int = const $C9 to unsigned char status = [S+$1E-$13] (used reg = )
mov	al,-$11[bp]
and	al,#$C9
! Debug: ne int = const $48 to unsigned char = al+0 (used reg = )
cmp	al,*$48
je  	.279
.27A:
! 2041         ;
!BCC_EOS
! 2042         return 4;
mov	ax,*4
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 2043         }
! 2044       bufseg += (bufoff / 16);
.279:
! Debug: div int = const $10 to unsigned short bufoff = [S+$1E+$14] (used reg = )
mov	ax,$16[bp]
mov	cl,*4
shr	ax,cl
! Debug: addab unsigned int = ax+0 to unsigned short bufseg = [S+$1E+$12] (used reg = )
add	ax,$14[bp]
mov	$14[bp],ax
!BCC_EOS
! 2045       bufoff %= 16;
! Debug: modab int = const $10 to unsigned short bufoff = [S+$1E+$14] (used reg = )
mov	ax,$16[bp]
and	al,*$F
xor	ah,ah
mov	$16[bp],ax
!BCC_EOS
! 2046       lcount = ((Bit16u)(inb(iobase1 + 5))<<8)+inb(iobase1 + 4);
! Debug: add int = const 4 to unsigned short iobase1 = [S+$1E-6] (used reg = )
mov	ax,-4[bp]
! Debug: list unsigned int = ax+4 (used reg = )
add	ax,*4
push	ax
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
inc	sp
inc	sp
push	ax
! Debug: add int = const 5 to unsigned short iobase1 = [S+$20-6] (used reg = )
mov	ax,-4[bp]
! Debug: list unsigned int = ax+5 (used reg = )
add	ax,*5
push	ax
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
inc	sp
inc	sp
! Debug: cast unsigned short = const 0 to unsigned char = al+0 (used reg = )
xor	ah,ah
! Debug: sl int = const 8 to unsigned short = ax+0 (used reg = )
mov	ah,al
xor	al,al
! Debug: add unsigned char (temp) = [S+$20-$20] to unsigned int = ax+0 (used reg = )
add	al,-$1E[bp]
adc	ah,*0
inc	sp
inc	sp
! Debug: eq unsigned int = ax+0 to unsigned short lcount = [S+$1E-$A] (used reg = )
mov	-8[bp],ax
!BCC_EOS
! 2047       if(header>lcount) {
! Debug: gt unsigned short lcount = [S+$1E-$A] to unsigned short header = [S+$1E+$A] (used reg = )
mov	ax,$C[bp]
cmp	ax,-8[bp]
jbe 	.27B
.27C:
! 2048          lbefore=lcount;
! Debug: eq unsigned short lcount = [S+$1E-$A] to unsigned short lbefore = [S+$1E-$C] (used reg = )
mov	ax,-8[bp]
mov	-$A[bp],ax
!BCC_EOS
! 2049          header-=lcount;
! Debug: subab unsigned short lcount = [S+$1E-$A] to unsigned short header = [S+$1E+$A] (used reg = )
mov	ax,$C[bp]
sub	ax,-8[bp]
mov	$C[bp],ax
!BCC_EOS
! 2050          lcount=0;
! Debug: eq int = const 0 to unsigned short lcount = [S+$1E-$A] (used reg = )
xor	ax,ax
mov	-8[bp],ax
!BCC_EOS
! 2051          }
! 2052       else {
jmp .27D
.27B:
! 2053         lbefore=header;
! Debug: eq unsigned short header = [S+$1E+$A] to unsigned short lbefore = [S+$1E-$C] (used reg = )
mov	ax,$C[bp]
mov	-$A[bp],ax
!BCC_EOS
! 2054         header=0;
! Debug: eq int = const 0 to unsigned short header = [S+$1E+$A] (used reg = )
xor	ax,ax
mov	$C[bp],ax
!BCC_EOS
! 2055         lcount-=lbefore;
! Debug: subab unsigned short lbefore = [S+$1E-$C] to unsigned short lcount = [S+$1E-$A] (used reg = )
mov	ax,-8[bp]
sub	ax,-$A[bp]
mov	-8[bp],ax
!BCC_EOS
! 2056         }
! 2057       if(lcount>length) {
.27D:
! Debug: cast unsigned long = const 0 to unsigned short lcount = [S+$1E-$A] (used reg = )
mov	ax,-8[bp]
xor	bx,bx
! Debug: gt unsigned long length = [S+$1E+$C] to unsigned long = bx+0 (used reg = )
lea	di,$E[bp]
call	lcmpul
jbe 	.27E
.27F:
! 2058         lafter=lcount-length;
! Debug: cast unsigned long = const 0 to unsigned short lcount = [S+$1E-$A] (used reg = )
mov	ax,-8[bp]
xor	bx,bx
! Debug: sub unsigned long length = [S+$1E+$C] to unsigned long = bx+0 (used reg = )
lea	di,$E[bp]
call	lsubul
! Debug: eq unsigned long = bx+0 to unsigned short lafter = [S+$1E-$E] (used reg = )
mov	-$C[bp],ax
!BCC_EOS
! 2059         lcount=length;
! Debug: eq unsigned long length = [S+$1E+$C] to unsigned short lcount = [S+$1E-$A] (used reg = )
mov	ax,$E[bp]
mov	-8[bp],ax
!BCC_EOS
! 2060         length=0;
! Debug: eq int = const 0 to unsigned long length = [S+$1E+$C] (used reg = )
xor	ax,ax
xor	bx,bx
mov	$E[bp],ax
mov	$10[bp],bx
!BCC_EOS
! 2061         }
! 2062       else {
jmp .280
.27E:
! 2063         lafter=0;
! Debug: eq int = const 0 to unsigned short lafter = [S+$1E-$E] (used reg = )
xor	ax,ax
mov	-$C[bp],ax
!BCC_EOS
! 2064         length-=lcount;
! Debug: cast unsigned long = const 0 to unsigned short lcount = [S+$1E-$A] (used reg = )
mov	ax,-8[bp]
xor	bx,bx
! Debug: subab unsigned long = bx+0 to unsigned long length = [S+$1E+$C] (used reg = )
push	bx
push	ax
mov	ax,$E[bp]
mov	bx,$10[bp]
lea	di,-$20[bp]
call	lsubul
mov	$E[bp],ax
mov	$10[bp],bx
add	sp,*4
!BCC_EOS
! 2065         }
! 2066       count = lcount;
.280:
! Debug: eq unsigned short lcount = [S+$1E-$A] to unsigned short count = [S+$1E-$10] (used reg = )
mov	ax,-8[bp]
mov	-$E[bp],ax
!BCC_EOS
! 2067       ;
!BCC_EOS
! 2068       ;
!BCC_EOS
! 2069       lmode = mode;
! Debug: eq unsigned char mode = [S+$1E-$14] to unsigned char lmode = [S+$1E-$15] (used reg = )
mov	al,-$12[bp]
mov	-$13[bp],al
!BCC_EOS
! 2070       if (lbefore & 0x03) lmode=0x00;
! Debug: and int = const 3 to unsigned short lbefore = [S+$1E-$C] (used reg = )
mov	al,-$A[bp]
and	al,*3
test	al,al
je  	.281
.282:
! Debug: eq int = const 0 to unsigned char lmode = [S+$1E-$15] (used reg = )
xor	al,al
mov	-$13[bp],al
!BCC_EOS
! 2071       if (lcount & 0x03) lmode=0x00;
.281:
! Debug: and int = const 3 to unsigned short lcount = [S+$1E-$A] (used reg = )
mov	al,-8[bp]
and	al,*3
test	al,al
je  	.283
.284:
! Debug: eq int = const 0 to unsigned char lmode = [S+$1E-$15] (used reg = )
xor	al,al
mov	-$13[bp],al
!BCC_EOS
! 2072       if (lafter & 0x03) lmode=0x00;
.283:
! Debug: and int = const 3 to unsigned short lafter = [S+$1E-$E] (used reg = )
mov	al,-$C[bp]
and	al,*3
test	al,al
je  	.285
.286:
! Debug: eq int = const 0 to unsigned char lmode = [S+$1E-$15] (used reg = )
xor	al,al
mov	-$13[bp],al
!BCC_EOS
! 2073       if (lcount & 0x01) {
.285:
! Debug: and int = const 1 to unsigned short lcount = [S+$1E-$A] (used reg = )
mov	al,-8[bp]
and	al,*1
test	al,al
je  	.287
.288:
! 2074         lcount+=1;
! Debug: addab int = const 1 to unsigned short lcount = [S+$1E-$A] (used reg = )
mov	ax,-8[bp]
inc	ax
mov	-8[bp],ax
!BCC_EOS
! 2075         if ((lafter > 0) && (lafter & 0x01)) {
! Debug: gt int = const 0 to unsigned short lafter = [S+$1E-$E] (used reg = )
mov	ax,-$C[bp]
test	ax,ax
je  	.289
.28B:
! Debug: and int = const 1 to unsigned short lafter = [S+$1E-$E] (used reg = )
mov	al,-$C[bp]
and	al,*1
test	al,al
je  	.289
.28A:
! 2076           lafter-=1;
! Debug: subab int = const 1 to unsigned short lafter = [S+$1E-$E] (used reg = )
mov	ax,-$C[bp]
dec	ax
mov	-$C[bp],ax
!BCC_EOS
! 2077           }
! 2078         }
.289:
! 2079       if (lmode == 0x01) {
.287:
! Debug: logeq int = const 1 to unsigned char lmode = [S+$1E-$15] (used reg = )
mov	al,-$13[bp]
cmp	al,*1
jne 	.28C
.28D:
! 2080         lcount>>=2; lbefore>>=2; lafter>>=2;
! Debug: srab int = const 2 to unsigned short lcount = [S+$1E-$A] (used reg = )
mov	ax,-8[bp]
shr	ax,*1
shr	ax,*1
mov	-8[bp],ax
!BCC_EOS
! Debug: srab int = const 2 to unsigned short lbefore = [S+$1E-$C] (used reg = )
mov	ax,-$A[bp]
shr	ax,*1
shr	ax,*1
mov	-$A[bp],ax
!BCC_EOS
! Debug: srab int = const 2 to unsigned short lafter = [S+$1E-$E] (used reg = )
mov	ax,-$C[bp]
shr	ax,*1
shr	ax,*1
mov	-$C[bp],ax
!BCC_EOS
! 2081         }
! 2082       else {
jmp .28E
.28C:
! 2083         lcount>>=1; lbefore>>=1; lafter>>=1;
! Debug: srab int = const 1 to unsigned short lcount = [S+$1E-$A] (used reg = )
mov	ax,-8[bp]
shr	ax,*1
mov	-8[bp],ax
!BCC_EOS
! Debug: srab int = const 1 to unsigned short lbefore = [S+$1E-$C] (used reg = )
mov	ax,-$A[bp]
shr	ax,*1
mov	-$A[bp],ax
!BCC_EOS
! Debug: srab int = const 1 to unsigned short lafter = [S+$1E-$E] (used reg = )
mov	ax,-$C[bp]
shr	ax,*1
mov	-$C[bp],ax
!BCC_EOS
! 2084         }
! 2085      
! 2085   ;
.28E:
!BCC_EOS
! 2086 #asm
!BCC_EOS
!BCC_ASM
_ata_cmd_packet.cmdoff	set	$26
.ata_cmd_packet.cmdoff	set	$A
_ata_cmd_packet.header	set	$28
.ata_cmd_packet.header	set	$C
_ata_cmd_packet.count	set	$E
.ata_cmd_packet.count	set	-$E
_ata_cmd_packet.lafter	set	$10
.ata_cmd_packet.lafter	set	-$C
_ata_cmd_packet.iobase1	set	$18
.ata_cmd_packet.iobase1	set	-4
_ata_cmd_packet.channel	set	$D
.ata_cmd_packet.channel	set	-$F
_ata_cmd_packet.cmdseg	set	$24
.ata_cmd_packet.cmdseg	set	8
_ata_cmd_packet.cmdlen	set	$22
.ata_cmd_packet.cmdlen	set	6
_ata_cmd_packet.lmode	set	9
.ata_cmd_packet.lmode	set	-$13
_ata_cmd_packet.device	set	$20
.ata_cmd_packet.device	set	4
_ata_cmd_packet.ebda_seg	set	$1A
.ata_cmd_packet.ebda_seg	set	-2
_ata_cmd_packet.lcount	set	$14
.ata_cmd_packet.lcount	set	-8
_ata_cmd_packet.total	set	4
.ata_cmd_packet.total	set	-$18
_ata_cmd_packet.status	set	$B
.ata_cmd_packet.status	set	-$11
_ata_cmd_packet.mode	set	$A
.ata_cmd_packet.mode	set	-$12
_ata_cmd_packet.bufoff	set	$32
.ata_cmd_packet.bufoff	set	$16
_ata_cmd_packet.transfer	set	0
.ata_cmd_packet.transfer	set	-$1C
_ata_cmd_packet.iobase2	set	$16
.ata_cmd_packet.iobase2	set	-6
_ata_cmd_packet.lbefore	set	$12
.ata_cmd_packet.lbefore	set	-$A
_ata_cmd_packet.bufseg	set	$30
.ata_cmd_packet.bufseg	set	$14
_ata_cmd_packet.slave	set	$C
.ata_cmd_packet.slave	set	-$10
_ata_cmd_packet.inout	set	$2E
.ata_cmd_packet.inout	set	$12
_ata_cmd_packet.length	set	$2A
.ata_cmd_packet.length	set	$E
        push bp
        mov bp, sp
        mov dx, _ata_cmd_packet.iobase1 + 2[bp] ;; ATA data read port
        mov cx, _ata_cmd_packet.lbefore + 2[bp]
        jcxz ata_packet_no_before
        mov ah, _ata_cmd_packet.lmode + 2[bp]
        cmp ah, #0x01
        je ata_packet_in_before_32
ata_packet_in_before_16:
        in ax, dx
        loop ata_packet_in_before_16
        jmp ata_packet_no_before
ata_packet_in_before_32:
        push eax
ata_packet_in_before_32_loop:
        in eax, dx
        loop ata_packet_in_before_32_loop
        pop eax
ata_packet_no_before:
        mov cx, _ata_cmd_packet.lcount + 2[bp]
        jcxz ata_packet_after
        mov di, _ata_cmd_packet.bufoff + 2[bp]
        mov ax, _ata_cmd_packet.bufseg + 2[bp]
        mov es, ax
        mov ah, _ata_cmd_packet.lmode + 2[bp]
        cmp ah, #0x01
        je ata_packet_in_32
ata_packet_in_16:
        rep
          insw ;; CX words transfered tp port(DX) to ES:[DI]
        jmp ata_packet_after
ata_packet_in_32:
        rep
          insd ;; CX dwords transfered to port(DX) to ES:[DI]
ata_packet_after:
        mov cx, _ata_cmd_packet.lafter + 2[bp]
        jcxz ata_packet_done
        mov ah, _ata_cmd_packet.lmode + 2[bp]
        cmp ah, #0x01
        je ata_packet_in_after_32
ata_packet_in_after_16:
        in ax, dx
        loop ata_packet_in_after_16
        jmp ata_packet_done
ata_packet_in_after_32:
        push eax
ata_packet_in_after_32_loop:
        in eax, dx
        loop ata_packet_in_after_32_loop
        pop eax
ata_packet_done:
        pop bp
! 2139 endasm
!BCC_ENDASM
!BCC_EOS
! 2140       bufoff += count;
! Debug: addab unsigned short count = [S+$1E-$10] to unsigned short bufoff = [S+$1E+$14] (used reg = )
mov	ax,$16[bp]
add	ax,-$E[bp]
mov	$16[bp],ax
!BCC_EOS
! 2141       transfer += count;
! Debug: cast unsigned long = const 0 to unsigned short count = [S+$1E-$10] (used reg = )
mov	ax,-$E[bp]
xor	bx,bx
! Debug: addab unsigned long = bx+0 to unsigned long transfer = [S+$1E-$1E] (used reg = )
lea	di,-$1C[bp]
call	laddul
mov	-$1C[bp],ax
mov	-$1A[bp],bx
!BCC_EOS
! 2142       write_dword(ebda_seg, &((ebda_data_t *) 0)->ata.trsfbytes,transfer);
! Debug: list unsigned long transfer = [S+$1E-$1E] (used reg = )
push	-$1A[bp]
push	-$1C[bp]
! Debug: list * unsigned long = const $236 (used reg = )
mov	ax,#$236
push	ax
! Debug: list unsigned short ebda_seg = [S+$24-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_dword+0 (used reg = )
call	_write_dword
add	sp,*8
!BCC_EOS
! 2143       }
! 2144     }
.273:
br 	.274
.28F:
.272:
! 2145   if ( (status & (0x80 | 0x40 | 0x20 | 0x08 | 0x01) )
.271:
! 2146          != 0x40 ) {
! Debug: and int = const $E9 to unsigned char status = [S+$1E-$13] (used reg = )
mov	al,-$11[bp]
and	al,#$E9
! Debug: ne int = const $40 to unsigned char = al+0 (used reg = )
cmp	al,*$40
je  	.290
.291:
! 2147     ;
!BCC_EOS
! 2148     return 4;
mov	ax,*4
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 2149     }
! 2150   outb(iobase2+6, 0x08);
.290:
! Debug: list int = const 8 (used reg = )
mov	ax,*8
push	ax
! Debug: add int = const 6 to unsigned short iobase2 = [S+$20-8] (used reg = )
mov	ax,-6[bp]
! Debug: list unsigned int = ax+6 (used reg = )
add	ax,*6
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 2151   return 0;
xor	ax,ax
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 2152 }
! 2153   Bit16u
! Register BX used in function ata_cmd_packet
! 2154 atapi_get_sense(device)
! 2155   Bit16u device;
export	_atapi_get_sense
_atapi_get_sense:
!BCC_EOS
! 2156 {
! 2157   Bit8u atacmd[12];
!BCC_EOS
! 2158   Bit8u buffer[16];
!BCC_EOS
! 2159   Bit8u i;
!BCC_EOS
! 2160   memsetb(get_SS(),atacmd,0,12);
push	bp
mov	bp,sp
add	sp,*-$1E
! Debug: list int = const $C (used reg = )
mov	ax,*$C
push	ax
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: list * unsigned char atacmd = S+$24-$E (used reg = )
lea	bx,-$C[bp]
push	bx
! Debug: func () unsigned short = get_SS+0 (used reg = )
call	_get_SS
! Debug: list unsigned short = ax+0 (used reg = )
push	ax
! Debug: func () void = memsetb+0 (used reg = )
call	_memsetb
add	sp,*8
!BCC_EOS
! 2161   atacmd[0]=0x03;
! Debug: eq int = const 3 to unsigned char atacmd = [S+$20-$E] (used reg = )
mov	al,*3
mov	-$C[bp],al
!BCC_EOS
! 2162   atacmd[4]=0x20;
! Debug: eq int = const $20 to unsigned char atacmd = [S+$20-$A] (used reg = )
mov	al,*$20
mov	-8[bp],al
!BCC_EOS
! 2163   if (ata_cmd_packet(device, 1
! 2163 2, get_SS(), atacmd, 0, 16L, 0x01, get_SS(), buffer) != 0)
! Debug: list * unsigned char buffer = S+$20-$1E (used reg = )
lea	bx,-$1C[bp]
push	bx
! Debug: func () unsigned short = get_SS+0 (used reg = )
call	_get_SS
! Debug: list unsigned short = ax+0 (used reg = )
push	ax
! Debug: list int = const 1 (used reg = )
mov	ax,*1
push	ax
! Debug: list long = const $10 (used reg = )
mov	ax,*$10
xor	bx,bx
push	bx
push	ax
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: list * unsigned char atacmd = S+$2C-$E (used reg = )
lea	bx,-$C[bp]
push	bx
! Debug: func () unsigned short = get_SS+0 (used reg = )
call	_get_SS
! Debug: list unsigned short = ax+0 (used reg = )
push	ax
! Debug: list int = const $C (used reg = )
mov	ax,*$C
push	ax
! Debug: list unsigned short device = [S+$32+2] (used reg = )
push	4[bp]
! Debug: func () unsigned short = ata_cmd_packet+0 (used reg = )
call	_ata_cmd_packet
add	sp,*$14
! Debug: ne int = const 0 to unsigned short = ax+0 (used reg = )
test	ax,ax
je  	.292
.293:
! 2164     return 0x0002;
mov	ax,*2
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 2165   if ((buffer[0] & 0x7e) == 0x70) {
.292:
! Debug: and int = const $7E to unsigned char buffer = [S+$20-$1E] (used reg = )
mov	al,-$1C[bp]
and	al,*$7E
! Debug: logeq int = const $70 to unsigned char = al+0 (used reg = )
cmp	al,*$70
jne 	.294
.295:
! 2166     return (((Bit16u)buffer[2]&0x0f)*0x100)+buffer[12];
! Debug: cast unsigned short = const 0 to unsigned char buffer = [S+$20-$1C] (used reg = )
mov	al,-$1A[bp]
xor	ah,ah
! Debug: and int = const $F to unsigned short = ax+0 (used reg = )
and	al,*$F
! Debug: mul int = const $100 to unsigned char = al+0 (used reg = )
xor	ah,ah
mov	cx,#$100
imul	cx
! Debug: add unsigned char buffer = [S+$20-$12] to unsigned int = ax+0 (used reg = )
add	al,-$10[bp]
adc	ah,*0
! Debug: cast unsigned short = const 0 to unsigned int = ax+0 (used reg = )
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 2167     }
! 2168   return 0;
.294:
xor	ax,ax
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 2169 }
! 2170   Bit16u
! Register BX used in function atapi_get_sense
! 2171 atapi_is_ready(device)
! 2172   Bit16u device;
export	_atapi_is_ready
_atapi_is_ready:
!BCC_EOS
! 2173 {
! 2174   Bit8u atacmd[12];
!BCC_EOS
! 2175   Bit8u buffer[];
!BCC_EOS
! 2176   memsetb(get_SS(),atacmd,0,12);
push	bp
mov	bp,sp
add	sp,*-$C
! Debug: list int = const $C (used reg = )
mov	ax,*$C
push	ax
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: list * unsigned char atacmd = S+$12-$E (used reg = )
lea	bx,-$C[bp]
push	bx
! Debug: func () unsigned short = get_SS+0 (used reg = )
call	_get_SS
! Debug: list unsigned short = ax+0 (used reg = )
push	ax
! Debug: func () void = memsetb+0 (used reg = )
call	_memsetb
add	sp,*8
!BCC_EOS
! 2177   if (ata_cmd_packet(device, 12, get_SS(), atacmd, 0, 0L, 0x00, get_SS(), buffer) != 0)
! Debug: list * unsigned char buffer = S+$E-$E (used reg = )
lea	bx,-$C[bp]
push	bx
! Debug: func () unsigned short = get_SS+0 (used reg = )
call	_get_SS
! Debug: list unsigned short = ax+0 (used reg = )
push	ax
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: list long = const 0 (used reg = )
xor	ax,ax
xor	bx,bx
push	bx
push	ax
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: list * unsigned char atacmd = S+$1A-$E (used reg = )
lea	bx,-$C[bp]
push	bx
! Debug: func () unsigned short = get_SS+0 (used reg = )
call	_get_SS
! Debug: list unsigned short = ax+0 (used reg = )
push	ax
! Debug: list int = const $C (used reg = )
mov	ax,*$C
push	ax
! Debug: list unsigned short device = [S+$20+2] (used reg = )
push	4[bp]
! Debug: func () unsigned short = ata_cmd_packet+0 (used reg = )
call	_ata_cmd_packet
add	sp,*$14
! Debug: ne int = const 0 to unsigned short = ax+0 (used reg = )
test	ax,ax
je  	.296
.297:
! 2178     return 0x000f;
mov	ax,*$F
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 2179   if (atapi_get_sense(device) !=0 ) {
.296:
! Debug: list unsigned short device = [S+$E+2] (used reg = )
push	4[bp]
! Debug: func () unsigned short = atapi_get_sense+0 (used reg = )
call	_atapi_get_sense
inc	sp
inc	sp
! Debug: ne int = const 0 to unsigned short = ax+0 (used reg = )
test	ax,ax
je  	.298
.299:
! 2180     memsetb(get_SS(),atacmd,0,12);
! Debug: list int = const $C (used reg = )
mov	ax,*$C
push	ax
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: list * unsigned char atacmd = S+$12-$E (used reg = )
lea	bx,-$C[bp]
push	bx
! Debug: func () unsigned short = get_SS+0 (used reg = )
call	_get_SS
! Debug: list unsigned short = ax+0 (used reg = )
push	ax
! Debug: func () void = memsetb+0 (used reg = )
call	_memsetb
add	sp,*8
!BCC_EOS
! 2181     if (ata_cmd_packet(device, 12, get_SS(), atacmd, 0, 0L, 0x00, get_SS(), buffer) != 0)
! Debug: list * unsigned char buffer = S+$E-$E (used reg = )
lea	bx,-$C[bp]
push	bx
! Debug: func () unsigned short = get_SS+0 (used reg = )
call	_get_SS
! Debug: list unsigned short = ax+0 (used reg = )
push	ax
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: list long = const 0 (used reg = )
xor	ax,ax
xor	bx,bx
push	bx
push	ax
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: list * unsigned char atacmd = S+$1A-$E (used reg = )
lea	bx,-$C[bp]
push	bx
! Debug: func () unsigned short = get_SS+0 (used reg = )
call	_get_SS
! Debug: list unsigned short = ax+0 (used reg = )
push	ax
! Debug: list int = const $C (used reg = )
mov	ax,*$C
push	ax
! Debug: list unsigned short device = [S+$20+2] (used reg = )
push	4[bp]
! Debug: func () unsigned short = ata_cmd_packet+0 (used reg = )
call	_ata_cmd_packet
add	sp,*$14
! Debug: ne int = const 0 to unsigned short = ax+0 (used reg = )
test	ax,ax
je  	.29A
.29B:
! 2182       return 0x000f;
mov	ax,*$F
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 2183     return atapi_get_sense(device);
.29A:
! Debug: list unsigned short device = [S+$E+2] (used reg = )
push	4[bp]
! Debug: func () unsigned short = atapi_get_sense+0 (used reg = )
call	_atapi_get_sense
inc	sp
inc	sp
! Debug: cast unsigned short = const 0 to unsigned short = ax+0 (used reg = )
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 2184     }
! 2185   return 0;
.298:
xor	ax,ax
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 2186 }
! 2187   Bit16u
! Register BX used in function atapi_is_ready
! 2188 atapi_is_cdrom(device)
! 2189   Bit8u device;
export	_atapi_is_cdrom
_atapi_is_cdrom:
!BCC_EOS
! 2190 {
! 2191   Bit16u ebda_seg=read_word(0x0040,0x000E);
push	bp
mov	bp,sp
dec	sp
dec	sp
! Debug: list int = const $E (used reg = )
mov	ax,*$E
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () unsigned short = read_word+0 (used reg = )
call	_read_word
add	sp,*4
! Debug: eq unsigned short = ax+0 to unsigned short ebda_seg = [S+4-4] (used reg = )
mov	-2[bp],ax
!BCC_EOS
! 2192   if (device >= (4*2))
! Debug: ge int = const 8 to unsigned char device = [S+4+2] (used reg = )
mov	al,4[bp]
cmp	al,*8
jb  	.29C
.29D:
! 2193     return 0;
xor	ax,ax
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 2194   if (read_byte(ebda_seg,&((ebda_data_t *) 0)->ata.devices[device].type) != 0x03)
.29C:
! Debug: ptradd unsigned char device = [S+4+2] to [8] struct  = const $142 (used reg = )
mov	al,4[bp]
xor	ah,ah
mov	cx,*$1A
imul	cx
mov	bx,ax
! Debug: address unsigned char = [bx+$142] (used reg = )
! Debug: list * unsigned char = bx+$142 (used reg = )
add	bx,#$142
push	bx
! Debug: list unsigned short ebda_seg = [S+6-4] (used reg = )
push	-2[bp]
! Debug: func () unsigned char = read_byte+0 (used reg = )
call	_read_byte
add	sp,*4
! Debug: ne int = const 3 to unsigned char = al+0 (used reg = )
cmp	al,*3
je  	.29E
.29F:
! 2195     return 0;
xor	ax,ax
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 2196   if (read_byte(ebda_seg,&((ebda_data_t *) 0)->ata.devices[device].device) != 0x05)
.29E:
! Debug: ptradd unsigned char device = [S+4+2] to [8] struct  = const $142 (used reg = )
mov	al,4[bp]
xor	ah,ah
mov	cx,*$1A
imul	cx
mov	bx,ax
! Debug: address unsigned char = [bx+$143] (used reg = )
! Debug: list * unsigned char = bx+$143 (used reg = )
add	bx,#$143
push	bx
! Debug: list unsigned short ebda_seg = [S+6-4] (used reg = )
push	-2[bp]
! Debug: func () unsigned char = read_byte+0 (used reg = )
call	_read_byte
add	sp,*4
! Debug: ne int = const 5 to unsigned char = al+0 (used reg = )
cmp	al,*5
je  	.2A0
.2A1:
! 2197     return 0;
xor	ax,ax
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 2198   return 1;
.2A0:
mov	ax,*1
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 2199 }
! 2200   void
! Register BX used in function atapi_is_cdrom
! 2201 cdemu_init()
! 2202 {
export	_cdemu_init
_cdemu_init:
! 2203   Bit16u ebda_seg=read_word(0x0040,0x000E);
push	bp
mov	bp,sp
dec	sp
dec	sp
! Debug: list int = const $E (used reg = )
mov	ax,*$E
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () unsigned short = read_word+0 (used reg = )
call	_read_word
add	sp,*4
! Debug: eq unsigned short = ax+0 to unsigned short ebda_seg = [S+4-4] (used reg = )
mov	-2[bp],ax
!BCC_EOS
! 2204   write_byte(ebda_seg,&((ebda_data_t *) 0)->cdemu.active,0x00);
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: list * unsigned char = const $23A (used reg = )
mov	ax,#$23A
push	ax
! Debug: list unsigned short ebda_seg = [S+8-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 2205 }
mov	sp,bp
pop	bp
ret
! 2206   Bit8u
! 2207 cdemu_isactive()
! 2208 {
export	_cdemu_isactive
_cdemu_isactive:
! 2209   Bit16u ebda_seg=read_word(0x0040,0x000E);
push	bp
mov	bp,sp
dec	sp
dec	sp
! Debug: list int = const $E (used reg = )
mov	ax,*$E
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () unsigned short = read_word+0 (used reg = )
call	_read_word
add	sp,*4
! Debug: eq unsigned short = ax+0 to unsigned short ebda_seg = [S+4-4] (used reg = )
mov	-2[bp],ax
!BCC_EOS
! 2210   return(read_byte(ebda_seg,&((ebda_data_t *) 0)->cdemu.active));
! Debug: list * unsigned char = const $23A (used reg = )
mov	ax,#$23A
push	ax
! Debug: list unsigned short ebda_seg = [S+6-4] (used reg = )
push	-2[bp]
! Debug: func () unsigned char = read_byte+0 (used reg = )
call	_read_byte
add	sp,*4
! Debug: cast unsigned char = const 0 to unsigned char = al+0 (used reg = )
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 2211 }
! 2212   Bit8u
! 2213 cdemu_emulated_drive()
! 2214 {
export	_cdemu_emulated_drive
_cdemu_emulated_drive:
! 2215   Bit16u ebda_seg=read_word(0x0040,0x000E);
push	bp
mov	bp,sp
dec	sp
dec	sp
! Debug: list int = const $E (used reg = )
mov	ax,*$E
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () unsigned short = read_word+0 (used reg = )
call	_read_word
add	sp,*4
! Debug: eq unsigned short = ax+0 to unsigned short ebda_seg = [S+4-4] (used reg = )
mov	-2[bp],ax
!BCC_EOS
! 2216   return(read_byte(ebda_seg,&((ebda_data_t *) 0)->cdemu.emulated_drive));
! Debug: list * unsigned char = const $23C (used reg = )
mov	ax,#$23C
push	ax
! Debug: list unsigned short ebda_seg = [S+6-4] (used reg = )
push	-2[bp]
! Debug: func () unsigned char = read_byte+0 (used reg = )
call	_read_byte
add	sp,*4
! Debug: cast unsigned char = const 0 to unsigned char = al+0 (used reg = )
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 2217 }
! 2218 static char isotag[6]="CD001";
.data
_isotag:
.2A2:
.ascii	"CD001"
.byte	0
!BCC_EOS
! 2219 static char eltorito[24]="EL TORITO SPECIFICATION";
_eltorito:
.2A3:
.ascii	"EL TORITO SPECIFICATION"
.byte	0
!BCC_EOS
! 2220   Bit16u
! 2221 cdrom_boot()
! 2222 {
.text
export	_cdrom_boot
_cdrom_boot:
! 2223   Bit16u ebda_seg=read_word(0x0040,0x000E);
push	bp
mov	bp,sp
dec	sp
dec	sp
! Debug: list int = const $E (used reg = )
mov	ax,*$E
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () unsigned short = read_word+0 (used reg = )
call	_read_word
add	sp,*4
! Debug: eq unsigned short = ax+0 to unsigned short ebda_seg = [S+4-4] (used reg = )
mov	-2[bp],ax
!BCC_EOS
! 2224   Bit8u atacmd[12], buffer[2048];
!BCC_EOS
! 2225   Bit32u lba;
!BCC_EOS
! 2226   Bit16u boot_segment, nbsectors, i, error;
!BCC_EOS
! 2227   Bit8u device;
!BCC_EOS
! 2228   for (device=0; device<(4*2);device++) {
add	sp,#-$81A
! Debug: eq int = const 0 to unsigned char device = [S+$81E-$81D] (used reg = )
xor	al,al
mov	-$81B[bp],al
!BCC_EOS
!BCC_EOS
jmp .2A6
.2A7:
! 2229     if (atapi_is_cdrom(device)) break;
! Debug: list unsigned char device = [S+$81E-$81D] (used reg = )
mov	al,-$81B[bp]
xor	ah,ah
push	ax
! Debug: func () unsigned short = atapi_is_cdrom+0 (used reg = )
call	_atapi_is_cdrom
inc	sp
inc	sp
test	ax,ax
je  	.2A8
.2A9:
jmp .2A4
!BCC_EOS
! 2230     }
.2A8:
! 2231   if(device >= (4*2)) return 2;
.2A5:
! Debug: postinc unsigned char device = [S+$81E-$81D] (used reg = )
mov	al,-$81B[bp]
inc	ax
mov	-$81B[bp],al
.2A6:
! Debug: lt int = const 8 to unsigned char device = [S+$81E-$81D] (used reg = )
mov	al,-$81B[bp]
cmp	al,*8
jb 	.2A7
.2AA:
.2A4:
! Debug: ge int = const 8 to unsigned char device = [S+$81E-$81D] (used reg = )
mov	al,-$81B[bp]
cmp	al,*8
jb  	.2AB
.2AC:
mov	ax,*2
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 2232   memsetb(get_SS(),atacmd,0,12);
.2AB:
! Debug: list int = const $C (used reg = )
mov	ax,*$C
push	ax
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: list * unsigned char atacmd = S+$822-$10 (used reg = )
lea	bx,-$E[bp]
push	bx
! Debug: func () unsigned short = get_SS+0 (used reg = )
call	_get_SS
! Debug: list unsigned short = ax+0 (used reg = )
push	ax
! Debug: func () void = memsetb+0 (used reg = )
call	_memsetb
add	sp,*8
!BCC_EOS
! 2233   atacmd[0]=0x28;
! Debug: eq int = const $28 to unsigned char atacmd = [S+$81E-$10] (used reg = )
mov	al,*$28
mov	-$E[bp],al
!BCC_EOS
! 2234   atacmd[7]=(0x01 & 0xff00) >> 8;
! Debug: eq unsigned int = const 0 to unsigned char atacmd = [S+$81E-9] (used reg = )
xor	al,al
mov	-7[bp],al
!BCC_EOS
! 2235   atacmd[8]=(0x01 & 0x00ff);
! Debug: eq int = const 1 to unsigned char atacmd = [S+$81E-8] (used reg = )
mov	al,*1
mov	-6[bp],al
!BCC_EOS
! 2236   atacmd[2]=(0x11 & 0xff000000) >> 24;
! Debug: eq unsigned long = const 0 to unsigned char atacmd = [S+$81E-$E] (used reg = )
xor	al,al
mov	-$C[bp],al
!BCC_EOS
! 2237   atacmd[3]=(0x11 & 0x00ff0000) >> 16;
! Debug: eq long = const 0 to unsigned char atacmd = [S+$81E-$D] (used reg = )
xor	al,al
mov	-$B[bp],al
!BCC_EOS
! 2238   atacmd[4]=(0x11 & 0x0000ff00) >> 8;
! Debug: eq unsigned int = const 0 to unsigned char atacmd = [S+$81E-$C] (used reg = )
xor	al,al
mov	-$A[bp],al
!BCC_EOS
! 2239   atacmd[5]=(0x11 
! 2239 & 0x000000ff);
! Debug: eq int = const $11 to unsigned char atacmd = [S+$81E-$B] (used reg = )
mov	al,*$11
mov	-9[bp],al
!BCC_EOS
! 2240   if((error = ata_cmd_packet(device, 12, get_SS(), atacmd, 0, 2048L, 0x01, get_SS(), buffer)) != 0)
! Debug: list * unsigned char buffer = S+$81E-$810 (used reg = )
lea	bx,-$80E[bp]
push	bx
! Debug: func () unsigned short = get_SS+0 (used reg = )
call	_get_SS
! Debug: list unsigned short = ax+0 (used reg = )
push	ax
! Debug: list int = const 1 (used reg = )
mov	ax,*1
push	ax
! Debug: list long = const $800 (used reg = )
mov	ax,#$800
xor	bx,bx
push	bx
push	ax
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: list * unsigned char atacmd = S+$82A-$10 (used reg = )
lea	bx,-$E[bp]
push	bx
! Debug: func () unsigned short = get_SS+0 (used reg = )
call	_get_SS
! Debug: list unsigned short = ax+0 (used reg = )
push	ax
! Debug: list int = const $C (used reg = )
mov	ax,*$C
push	ax
! Debug: list unsigned char device = [S+$830-$81D] (used reg = )
mov	al,-$81B[bp]
xor	ah,ah
push	ax
! Debug: func () unsigned short = ata_cmd_packet+0 (used reg = )
call	_ata_cmd_packet
add	sp,*$14
! Debug: eq unsigned short = ax+0 to unsigned short error = [S+$81E-$81C] (used reg = )
mov	-$81A[bp],ax
! Debug: ne int = const 0 to unsigned short = ax+0 (used reg = )
test	ax,ax
je  	.2AD
.2AE:
! 2241     return 3;
mov	ax,*3
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 2242   if(buffer[0]!=0)return 4;
.2AD:
! Debug: ne int = const 0 to unsigned char buffer = [S+$81E-$810] (used reg = )
mov	al,-$80E[bp]
test	al,al
je  	.2AF
.2B0:
mov	ax,*4
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 2243   for(i=0;i<5;i++){
.2AF:
! Debug: eq int = const 0 to unsigned short i = [S+$81E-$81A] (used reg = )
xor	ax,ax
mov	-$818[bp],ax
!BCC_EOS
!BCC_EOS
jmp .2B3
.2B4:
! 2244     if(buffer[1+i]!=read_byte(0xf000,&isotag[i]))return 5;
! Debug: ptradd unsigned short i = [S+$81E-$81A] to [6] char = isotag+0 (used reg = )
mov	bx,-$818[bp]
! Debug: address char = [bx+_isotag+0] (used reg = )
! Debug: list * char = bx+_isotag+0 (used reg = )
add	bx,#_isotag
push	bx
! Debug: list unsigned int = const $F000 (used reg = )
mov	ax,#$F000
push	ax
! Debug: func () unsigned char = read_byte+0 (used reg = )
call	_read_byte
add	sp,*4
push	ax
! Debug: add unsigned short i = [S+$820-$81A] to int = const 1 (used reg = )
! Debug: expression subtree swapping
mov	ax,-$818[bp]
! Debug: ptradd unsigned int = ax+1 to [$800] unsigned char buffer = S+$820-$810 (used reg = )
inc	ax
mov	bx,bp
add	bx,ax
! Debug: ne unsigned char (temp) = [S+$820-$820] to unsigned char = [bx-$80E] (used reg = )
mov	al,-$80E[bx]
cmp	al,-$81E[bp]
lea	sp,-$81C[bp]
je  	.2B5
.2B6:
mov	ax,*5
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 2245    }
.2B5:
! 2246   for(i=0;i<23;i++)
.2B2:
! Debug: postinc unsigned short i = [S+$81E-$81A] (used reg = )
mov	ax,-$818[bp]
inc	ax
mov	-$818[bp],ax
.2B3:
! Debug: lt int = const 5 to unsigned short i = [S+$81E-$81A] (used reg = )
mov	ax,-$818[bp]
cmp	ax,*5
jb 	.2B4
.2B7:
.2B1:
! Debug: eq int = const 0 to unsigned short i = [S+$81E-$81A] (used reg = )
xor	ax,ax
mov	-$818[bp],ax
!BCC_EOS
!BCC_EOS
! 2247     if(buffer[7+i]!=read_byte(0xf000,&eltorito[i]))return 6;
jmp .2BA
.2BB:
! Debug: ptradd unsigned short i = [S+$81E-$81A] to [$18] char = eltorito+0 (used reg = )
mov	bx,-$818[bp]
! Debug: address char = [bx+_eltorito+0] (used reg = )
! Debug: list * char = bx+_eltorito+0 (used reg = )
add	bx,#_eltorito
push	bx
! Debug: list unsigned int = const $F000 (used reg = )
mov	ax,#$F000
push	ax
! Debug: func () unsigned char = read_byte+0 (used reg = )
call	_read_byte
add	sp,*4
push	ax
! Debug: add unsigned short i = [S+$820-$81A] to int = const 7 (used reg = )
! Debug: expression subtree swapping
mov	ax,-$818[bp]
! Debug: ptradd unsigned int = ax+7 to [$800] unsigned char buffer = S+$820-$810 (used reg = )
add	ax,*7
mov	bx,bp
add	bx,ax
! Debug: ne unsigned char (temp) = [S+$820-$820] to unsigned char = [bx-$80E] (used reg = )
mov	al,-$80E[bx]
cmp	al,-$81E[bp]
lea	sp,-$81C[bp]
je  	.2BC
.2BD:
mov	ax,*6
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 2248   lba=buffer[0x4A]*0x1000000+buffer[0x49]*0x10000+buffer[0x48]*0x100+buffer[0x47];
.2BC:
.2B9:
! Debug: postinc unsigned short i = [S+$81E-$81A] (used reg = )
mov	ax,-$818[bp]
inc	ax
mov	-$818[bp],ax
.2BA:
! Debug: lt int = const $17 to unsigned short i = [S+$81E-$81A] (used reg = )
mov	ax,-$818[bp]
cmp	ax,*$17
jb 	.2BB
.2BE:
.2B8:
! Debug: cast unsigned long = const 0 to unsigned char buffer = [S+$81E-$7C9] (used reg = )
mov	al,-$7C7[bp]
xor	ah,ah
xor	bx,bx
push	bx
push	ax
! Debug: mul int = const $100 to unsigned char buffer = [S+$822-$7C8] (used reg = )
mov	al,-$7C6[bp]
xor	ah,ah
mov	cx,#$100
imul	cx
! Debug: cast unsigned long = const 0 to unsigned int = ax+0 (used reg = )
xor	bx,bx
push	bx
push	ax
! Debug: cast unsigned long = const 0 to unsigned char buffer = [S+$826-$7C7] (used reg = )
mov	al,-$7C5[bp]
xor	ah,ah
xor	bx,bx
! Debug: mul long = const $10000 to unsigned long = bx+0 (used reg = )
push	bx
push	ax
xor	ax,ax
mov	bx,*1
push	bx
push	ax
mov	ax,-$828[bp]
mov	bx,-$826[bp]
lea	di,-$82C[bp]
call	lmulul
add	sp,*8
push	bx
push	ax
! Debug: cast unsigned long = const 0 to unsigned char buffer = [S+$82A-$7C6] (used reg = )
mov	al,-$7C4[bp]
xor	ah,ah
xor	bx,bx
! Debug: mul long = const $1000000 to unsigned long = bx+0 (used reg = )
push	bx
push	ax
xor	ax,ax
mov	bx,#$100
push	bx
push	ax
mov	ax,-$82C[bp]
mov	bx,-$82A[bp]
lea	di,-$830[bp]
call	lmulul
add	sp,*8
! Debug: add unsigned long (temp) = [S+$82A-$82A] to unsigned long = bx+0 (used reg = )
lea	di,-$828[bp]
call	laddul
add	sp,*4
! Debug: add unsigned long (temp) = [S+$826-$826] to unsigned long = bx+0 (used reg = )
lea	di,-$824[bp]
call	laddul
add	sp,*4
! Debug: add unsigned long (temp) = [S+$822-$822] to unsigned long = bx+0 (used reg = )
lea	di,-$820[bp]
call	laddul
add	sp,*4
! Debug: eq unsigned long = bx+0 to unsigned long lba = [S+$81E-$814] (used reg = )
mov	-$812[bp],ax
mov	-$810[bp],bx
!BCC_EOS
! 2249   memsetb(get_SS(),atacmd,0,12);
! Debug: list int = const $C (used reg = )
mov	ax,*$C
push	ax
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: list * unsigned char atacmd = S+$822-$10 (used reg = )
lea	bx,-$E[bp]
push	bx
! Debug: func () unsigned short = get_SS+0 (used reg = )
call	_get_SS
! Debug: list unsigned short = ax+0 (used reg = )
push	ax
! Debug: func () void = memsetb+0 (used reg = )
call	_memsetb
add	sp,*8
!BCC_EOS
! 2250   atacmd[0]=0x28;
! Debug: eq int = const $28 to unsigned char atacmd = [S+$81E-$10] (used reg = )
mov	al,*$28
mov	-$E[bp],al
!BCC_EOS
! 2251   atacmd[7]=(0x01 & 0xff00) >> 8;
! Debug: eq unsigned int = const 0 to unsigned char atacmd = [S+$81E-9] (used reg = )
xor	al,al
mov	-7[bp],al
!BCC_EOS
! 2252   atacmd[8]=(0x01 & 0x00ff);
! Debug: eq int = const 1 to unsigned char atacmd = [S+$81E-8] (used reg = )
mov	al,*1
mov	-6[bp],al
!BCC_EOS
! 2253   atacmd[2]=(lba & 0xff000000) >> 24;
! Debug: and unsigned long = const $FF000000 to unsigned long lba = [S+$81E-$814] (used reg = )
! Debug: expression subtree swapping
xor	ax,ax
mov	bx,#$FF00
lea	di,-$812[bp]
call	landul
! Debug: sr int = const $18 to unsigned long = bx+0 (used reg = )
xchg	bx,ax
mov	al,ah
xor	ah,ah
xor	bx,bx
! Debug: eq unsigned long = bx+0 to unsigned char atacmd = [S+$81E-$E] (used reg = )
mov	-$C[bp],al
!BCC_EOS
! 2254   atacmd[3]=(lba & 0x00ff0000) >> 16;
! Debug: and long = const $FF0000 to unsigned long lba = [S+$81E-$814] (used reg = )
! Debug: expression subtree swapping
xor	ax,ax
mov	bx,#$FF
lea	di,-$812[bp]
call	landul
! Debug: sr int = const $10 to unsigned long = bx+0 (used reg = )
xchg	bx,ax
xor	bx,bx
! Debug: eq unsigned long = bx+0 to unsigned char atacmd = [S+$81E-$D] (used reg = )
mov	-$B[bp],al
!BCC_EOS
! 2255   atacmd[4]=(lba & 0x0000ff00) >> 8;
! Debug: and unsigned long = const $FF00 to unsigned long lba = [S+$81E-$814] (used reg = )
! Debug: expression subtree swapping
mov	ax,#$FF00
xor	bx,bx
lea	di,-$812[bp]
call	landul
! Debug: sr int = const 8 to unsigned long = bx+0 (used reg = )
mov	al,ah
mov	ah,bl
mov	bl,bh
sub	bh,bh
! Debug: eq unsigned long = bx+0 to unsigned char atacmd = [S+$81E-$C] (used reg = )
mov	-$A[bp],al
!BCC_EOS
! 2256   atacmd[5]=(lba & 0x000000ff);
! Debug: and unsigned long = const $FF to unsigned long lba = [S+$81E-$814] (used reg = )
! Debug: expression subtree swapping
mov	ax,#$FF
xor	bx,bx
lea	di,-$812[bp]
call	landul
! Debug: eq unsigned long = bx+0 to unsigned char atacmd = [S+$81E-$B] (used reg = )
mov	-9[bp],al
!BCC_EOS
! 2257   if((error = ata_cmd_packet(device, 12, get_SS(), atacmd, 0, 2048L, 0x01, get_SS(), buffer)) != 0)
! Debug: list * unsigned char buffer = S+$81E-$810 (used reg = )
lea	bx,-$80E[bp]
push	bx
! Debug: func () unsigned short = get_SS+0 (used reg = )
call	_get_SS
! Debug: list unsigned short = ax+0 (used reg = )
push	ax
! Debug: list int = const 1 (used reg = )
mov	ax,*1
push	ax
! Debug: list long = const $800 (used reg = )
mov	ax,#$800
xor	bx,bx
push	bx
push	ax
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: list * unsigned char atacmd = S+$82A-$10 (used reg = )
lea	bx,-$E[bp]
push	bx
! Debug: func () unsigned short = get_SS+0 (used reg = )
call	_get_SS
! Debug: list unsigned short = ax+0 (used reg = )
push	ax
! Debug: list int = const $C (used reg = )
mov	ax,*$C
push	ax
! Debug: list unsigned char device = [S+$830-$81D] (used reg = )
mov	al,-$81B[bp]
xor	ah,ah
push	ax
! Debug: func () unsigned short = ata_cmd_packet+0 (used reg = )
call	_ata_cmd_packet
add	sp,*$14
! Debug: eq unsigned short = ax+0 to unsigned short error = [S+$81E-$81C] (used reg = )
mov	-$81A[bp],ax
! Debug: ne int = const 0 to unsigned short = ax+0 (used reg = )
test	ax,ax
je  	.2BF
.2C0:
! 2258     return 7;
mov	ax,*7
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 2259   if(buffer[0x00]!=0x01)return 8;
.2BF:
! Debug: ne int = const 1 to unsigned char buffer = [S+$81E-$810] (used reg = )
mov	al,-$80E[bp]
cmp	al,*1
je  	.2C1
.2C2:
mov	ax,*8
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 2260   if(buffer[0x01]!=0x00)return 9;
.2C1:
! Debug: ne int = const 0 to unsigned char buffer = [S+$81E-$80F] (used reg = )
mov	al,-$80D[bp]
test	al,al
je  	.2C3
.2C4:
mov	ax,*9
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 2261   if(buffer[0x1E]!=0x55)return 10;
.2C3:
! Debug: ne int = const $55 to unsigned char buffer = [S+$81E-$7F2] (used reg = )
mov	al,-$7F0[bp]
cmp	al,*$55
je  	.2C5
.2C6:
mov	ax,*$A
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 2262   if(buffer[0x1F]!=0xAA)return 10;
.2C5:
! Debug: ne int = const $AA to unsigned char buffer = [S+$81E-$7F1] (used reg = )
mov	al,-$7EF[bp]
cmp	al,#$AA
je  	.2C7
.2C8:
mov	ax,*$A
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 2263   if(buffer[0x20]!=0x88)return 11;
.2C7:
! Debug: ne int = const $88 to unsigned char buffer = [S+$81E-$7F0] (used reg = )
mov	al,-$7EE[bp]
cmp	al,#$88
je  	.2C9
.2CA:
mov	ax,*$B
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 2264   tcpa_add_bootdevice((Bit32u)1L, (Bit32u)0L);
.2C9:
! Debug: list unsigned long = const 0 (used reg = )
xor	ax,ax
xor	bx,bx
push	bx
push	ax
! Debug: list unsigned long = const 1 (used reg = )
mov	ax,*1
xor	bx,bx
push	bx
push	ax
! Debug: func () void = tcpa_add_bootdevice+0 (used reg = )
call	_tcpa_add_bootdevice
add	sp,*8
!BCC_EOS
! 2265   tcpa_ipl((Bit32u)2L,(Bit32u)get_SS(),(Bit32u)buffer,(Bit32u)2048L);
! Debug: list unsigned long = const $800 (used reg = )
mov	ax,#$800
xor	bx,bx
push	bx
push	ax
! Debug: cast unsigned long = const 0 to [$800] unsigned char buffer = S+$822-$810 (used reg = )
mov	ax,bp
add	ax,#-$80E
xor	bx,bx
! Debug: list unsigned long = bx+0 (used reg = )
push	bx
push	ax
! Debug: func () unsigned short = get_SS+0 (used reg = )
call	_get_SS
! Debug: cast unsigned long = const 0 to unsigned short = ax+0 (used reg = )
xor	bx,bx
! Debug: list unsigned long = bx+0 (used reg = )
push	bx
push	ax
! Debug: list unsigned long = const 2 (used reg = )
mov	ax,*2
xor	bx,bx
push	bx
push	ax
! Debug: func () void = tcpa_ipl+0 (used reg = )
call	_tcpa_ipl
add	sp,*$10
!BCC_EOS
! 2266   write_byte(ebda_seg,&((ebda_data_t *) 0)->cdemu.media,buffer[0x21]);
! Debug: list unsigned char buffer = [S+$81E-$7EF] (used reg = )
mov	al,-$7ED[bp]
xor	ah,ah
push	ax
! Debug: list * unsigned char = const $23B (used reg = )
mov	ax,#$23B
push	ax
! Debug: list unsigned short ebda_seg = [S+$822-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 2267   if(buffer[0x21]==0){
! Debug: logeq int = const 0 to unsigned char buffer = [S+$81E-$7EF] (used reg = )
mov	al,-$7ED[bp]
test	al,al
jne 	.2CB
.2CC:
! 2268     write_byte(ebda_seg,&((ebda_data_t *) 0)->cdemu.emulated_drive,0xE0);
! Debug: list int = const $E0 (used reg = )
mov	ax,#$E0
push	ax
! Debug: list * unsigned char = const $23C (used reg = )
mov	ax,#$23C
push	ax
! Debug: list unsigned short ebda_seg = [S+$822-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 2269     }
! 2270   else if(buffer[0x21]<4)
jmp .2CD
.2CB:
! Debug: lt int = const 4 to unsigned char buffer = [S+$81E-$7EF] (used reg = )
mov	al,-$7ED[bp]
cmp	al,*4
jae 	.2CE
.2CF:
! 2271     write_byte(ebda_seg,&((ebda_data_t *) 0)->cdemu.emulated_drive,0x00);
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: list * unsigned char = const $23C (used reg = )
mov	ax,#$23C
push	ax
! Debug: list unsigned short ebda_seg = [S+$822-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 2272   else
! 2273     write_byte(ebda_seg,&((ebda_data_t *) 0)->cdemu.emulated_drive,0x80);
jmp .2D0
.2CE:
! Debug: list int = const $80 (used reg = )
mov	ax,#$80
push	ax
! Debug: list * unsigned char = const $23C (used reg = )
mov	ax,#$23C
push	ax
! Debug: list unsigned short ebda_seg = [S+$822-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 2274   write_byte(ebda_seg,&((ebda_data_t *) 0)->cdemu.controller_index,device/2);
.2D0:
.2CD:
! Debug: div int = const 2 to unsigned char device = [S+$81E-$81D] (used reg = )
mov	al,-$81B[bp]
xor	ah,ah
shr	ax,*1
! Debug: list unsigned int = ax+0 (used reg = )
push	ax
! Debug: list * unsigned char = const $23D (used reg = )
mov	ax,#$23D
push	ax
! Debug: list unsigned short ebda_seg = [S+$822-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 2275   write_byte(ebda_seg,&((ebda_data_t *) 0)->cdemu.device_spec,device%2);
! Debug: mod int = const 2 to unsigned char device = [S+$81E-$81D] (used reg = )
mov	al,-$81B[bp]
xor	ah,ah
and	al,*1
! Debug: list unsigned char = al+0 (used reg = )
xor	ah,ah
push	ax
! Debug: list * unsigned short = const $23E (used reg = )
mov	ax,#$23E
push	ax
! Debug: list unsigned short ebda_seg = [S+$822-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 2276   boot_segment=buffer[0x23]*0x100+buffer[0x22];
! Debug: mul int = const $100 to unsigned char buffer = [S+$81E-$7ED] (used reg = )
mov	al,-$7EB[bp]
xor	ah,ah
mov	cx,#$100
imul	cx
! Debug: add unsigned char buffer = [S+$81E-$7EE] to unsigned int = ax+0 (used reg = )
add	al,-$7EC[bp]
adc	ah,*0
! Debug: eq unsigned int = ax+0 to unsigned short boot_segment = [S+$81E-$816] (used reg = )
mov	-$814[bp],ax
!BCC_EOS
! 2277   if(boot_segment==0x0000)boot_segment=0x07C0;
! Debug: logeq int = const 0 to unsigned short boot_segment = [S+$81E-$816] (used reg = )
mov	ax,-$814[bp]
test	ax,ax
jne 	.2D1
.2D2:
! Debug: eq int = const $7C0 to unsigned short boot_segment = [S+$81E-$816] (used reg = )
mov	ax,#$7C0
mov	-$814[bp],ax
!BCC_EOS
! 2278   write_word(ebda_seg,&((ebda_data_t *) 0)->cdemu.load_segment,boot_segment);
.2D1:
! Debug: list unsigned short boot_segment = [S+$81E-$816] (used reg = )
push	-$814[bp]
! Debug: list * unsigned short = const $246 (used reg = )
mov	ax,#$246
push	ax
! Debug: list unsigned short ebda_seg = [S+$822-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
add	sp,*6
!BCC_EOS
! 2279   write_word(ebda_seg,&((ebda_data_t *) 0)->cdemu.buffer_segment,0x0000);
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: list * unsigned short = const $244 (used reg = )
mov	ax,#$244
push	ax
! Debug: list unsigned short ebda_seg = [S+$822-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
add	sp,*6
!BCC_EOS
! 2280   nbsectors=buffer[0x27]*0x100+buffer[0x26];
! Debug: mul int = const $100 to unsigned char buffer = [S+$81E-$7E9] (used reg = )
mov	al,-$7E7[bp]
xor	ah,ah
mov	cx,#$100
imul	cx
! Debug: add unsigned char buffer = [S+$81E-$7EA] to unsigned int = ax+0 (used reg = )
add	al,-$7E8[bp]
adc	ah,*0
! Debug: eq unsigned int = ax+0 to unsigned short nbsectors = [S+$81E-$818] (used reg = )
mov	-$816[bp],ax
!BCC_EOS
! 2281   write_word(ebda_seg,&((ebda_data_t *) 0)->cdemu.sector_count,nbsectors);
! Debug: list unsigned short nbsectors = [S+$81E-$818] (used reg = )
push	-$816[bp]
! Debug: list * unsigned short = const $248 (used reg = )
mov	ax,#$248
push	ax
! Debug: list unsigned short ebda_seg = [S+$822-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
add	sp,*6
!BCC_EOS
! 2282   lba=buffer[0x2B]*0x1000000+buffer[0x2A]*0x10000+buffer[0x29]*0x100+buffer[0x28];
! Debug: cast unsigned long = const 0 to unsigned char buffer = [S+$81E-$7E8] (used reg = )
mov	al,-$7E6[bp]
xor	ah,ah
xor	bx,bx
push	bx
push	ax
! Debug: mul int = const $100 to unsigned char buffer = [S+$822-$7E7] (used reg = )
mov	al,-$7E5[bp]
xor	ah,ah
mov	cx,#$100
imul	cx
! Debug: cast unsigned long = const 0 to unsigned int = ax+0 (used reg = )
xor	bx,bx
push	bx
push	ax
! Debug: cast unsigned long = const 0 to unsigned char buffer = [S+$826-$7E6] (used reg = )
mov	al,-$7E4[bp]
xor	ah,ah
xor	bx,bx
! Debug: mul long = const $10000 to unsigned long = bx+0 (used reg = )
push	bx
push	ax
xor	ax,ax
mov	bx,*1
push	bx
push	ax
mov	ax,-$828[bp]
mov	bx,-$826[bp]
lea	di,-$82C[bp]
call	lmulul
add	sp,*8
push	bx
push	ax
! Debug: cast unsigned long = const 0 to unsigned char buffer = [S+$82A-$7E5] (used reg = )
mov	al,-$7E3[bp]
xor	ah,ah
xor	bx,bx
! Debug: mul long = const $1000000 to unsigned long = bx+0 (used reg = )
push	bx
push	ax
xor	ax,ax
mov	bx,#$100
push	bx
push	ax
mov	ax,-$82C[bp]
mov	bx,-$82A[bp]
lea	di,-$830[bp]
call	lmulul
add	sp,*8
! Debug: add unsigned long (temp) = [S+$82A-$82A] to unsigned long = bx+0 (used reg = )
lea	di,-$828[bp]
call	laddul
add	sp,*4
! Debug: add unsigned long (temp) = [S+$826-$826] to unsigned long = bx+0 (used reg = )
lea	di,-$824[bp]
call	laddul
add	sp,*4
! Debug: add unsigned long (temp) = [S+$822-$822] to unsigned long = bx+0 (used reg = )
lea	di,-$820[bp]
call	laddul
add	sp,*4
! Debug: eq unsigned long = bx+0 to unsigned long lba = [S+$81E-$814] (used reg = )
mov	-$812[bp],ax
mov	-$810[bp],bx
!BCC_EOS
! 2283   write_dword(ebda_seg,&
! 2283 ((ebda_data_t *) 0)->cdemu.ilba,lba);
! Debug: list unsigned long lba = [S+$81E-$814] (used reg = )
push	-$810[bp]
push	-$812[bp]
! Debug: list * unsigned long = const $240 (used reg = )
mov	ax,#$240
push	ax
! Debug: list unsigned short ebda_seg = [S+$824-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_dword+0 (used reg = )
call	_write_dword
add	sp,*8
!BCC_EOS
! 2284   memsetb(get_SS(),atacmd,0,12);
! Debug: list int = const $C (used reg = )
mov	ax,*$C
push	ax
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: list * unsigned char atacmd = S+$822-$10 (used reg = )
lea	bx,-$E[bp]
push	bx
! Debug: func () unsigned short = get_SS+0 (used reg = )
call	_get_SS
! Debug: list unsigned short = ax+0 (used reg = )
push	ax
! Debug: func () void = memsetb+0 (used reg = )
call	_memsetb
add	sp,*8
!BCC_EOS
! 2285   atacmd[0]=0x28;
! Debug: eq int = const $28 to unsigned char atacmd = [S+$81E-$10] (used reg = )
mov	al,*$28
mov	-$E[bp],al
!BCC_EOS
! 2286   atacmd[7]=((1+(nbsectors-1)/4) & 0xff00) >> 8;
! Debug: sub int = const 1 to unsigned short nbsectors = [S+$81E-$818] (used reg = )
mov	ax,-$816[bp]
! Debug: div int = const 4 to unsigned int = ax-1 (used reg = )
dec	ax
shr	ax,*1
shr	ax,*1
! Debug: add unsigned int = ax+0 to int = const 1 (used reg = )
! Debug: expression subtree swapping
! Debug: and unsigned int = const $FF00 to unsigned int = ax+1 (used reg = )
inc	ax
xor	al,al
! Debug: sr int = const 8 to unsigned int = ax+0 (used reg = )
mov	al,ah
xor	ah,ah
! Debug: eq unsigned int = ax+0 to unsigned char atacmd = [S+$81E-9] (used reg = )
mov	-7[bp],al
!BCC_EOS
! 2287   atacmd[8]=((1+(nbsectors-1)/4) & 0x00ff);
! Debug: sub int = const 1 to unsigned short nbsectors = [S+$81E-$818] (used reg = )
mov	ax,-$816[bp]
! Debug: div int = const 4 to unsigned int = ax-1 (used reg = )
dec	ax
shr	ax,*1
shr	ax,*1
! Debug: add unsigned int = ax+0 to int = const 1 (used reg = )
! Debug: expression subtree swapping
! Debug: and int = const $FF to unsigned int = ax+1 (used reg = )
inc	ax
! Debug: eq unsigned char = al+0 to unsigned char atacmd = [S+$81E-8] (used reg = )
mov	-6[bp],al
!BCC_EOS
! 2288   atacmd[2]=(lba & 0xff000000) >> 24;
! Debug: and unsigned long = const $FF000000 to unsigned long lba = [S+$81E-$814] (used reg = )
! Debug: expression subtree swapping
xor	ax,ax
mov	bx,#$FF00
lea	di,-$812[bp]
call	landul
! Debug: sr int = const $18 to unsigned long = bx+0 (used reg = )
xchg	bx,ax
mov	al,ah
xor	ah,ah
xor	bx,bx
! Debug: eq unsigned long = bx+0 to unsigned char atacmd = [S+$81E-$E] (used reg = )
mov	-$C[bp],al
!BCC_EOS
! 2289   atacmd[3]=(lba & 0x00ff0000) >> 16;
! Debug: and long = const $FF0000 to unsigned long lba = [S+$81E-$814] (used reg = )
! Debug: expression subtree swapping
xor	ax,ax
mov	bx,#$FF
lea	di,-$812[bp]
call	landul
! Debug: sr int = const $10 to unsigned long = bx+0 (used reg = )
xchg	bx,ax
xor	bx,bx
! Debug: eq unsigned long = bx+0 to unsigned char atacmd = [S+$81E-$D] (used reg = )
mov	-$B[bp],al
!BCC_EOS
! 2290   atacmd[4]=(lba & 0x0000ff00) >> 8;
! Debug: and unsigned long = const $FF00 to unsigned long lba = [S+$81E-$814] (used reg = )
! Debug: expression subtree swapping
mov	ax,#$FF00
xor	bx,bx
lea	di,-$812[bp]
call	landul
! Debug: sr int = const 8 to unsigned long = bx+0 (used reg = )
mov	al,ah
mov	ah,bl
mov	bl,bh
sub	bh,bh
! Debug: eq unsigned long = bx+0 to unsigned char atacmd = [S+$81E-$C] (used reg = )
mov	-$A[bp],al
!BCC_EOS
! 2291   atacmd[5]=(lba & 0x000000ff);
! Debug: and unsigned long = const $FF to unsigned long lba = [S+$81E-$814] (used reg = )
! Debug: expression subtree swapping
mov	ax,#$FF
xor	bx,bx
lea	di,-$812[bp]
call	landul
! Debug: eq unsigned long = bx+0 to unsigned char atacmd = [S+$81E-$B] (used reg = )
mov	-9[bp],al
!BCC_EOS
! 2292   if((error = ata_cmd_packet(device, 12, get_SS(), atacmd, 0, nbsectors*512L, 0x01, boot_segment,0)) != 0)
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: list unsigned short boot_segment = [S+$820-$816] (used reg = )
push	-$814[bp]
! Debug: list int = const 1 (used reg = )
mov	ax,*1
push	ax
! Debug: cast unsigned long = const 0 to unsigned short nbsectors = [S+$824-$818] (used reg = )
mov	ax,-$816[bp]
xor	bx,bx
! Debug: mul long = const $200 to unsigned long = bx+0 (used reg = )
push	bx
push	ax
mov	ax,#$200
xor	bx,bx
push	bx
push	ax
mov	ax,-$826[bp]
mov	bx,-$824[bp]
lea	di,-$82A[bp]
call	lmulul
add	sp,*8
! Debug: list unsigned long = bx+0 (used reg = )
push	bx
push	ax
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: list * unsigned char atacmd = S+$82A-$10 (used reg = )
lea	bx,-$E[bp]
push	bx
! Debug: func () unsigned short = get_SS+0 (used reg = )
call	_get_SS
! Debug: list unsigned short = ax+0 (used reg = )
push	ax
! Debug: list int = const $C (used reg = )
mov	ax,*$C
push	ax
! Debug: list unsigned char device = [S+$830-$81D] (used reg = )
mov	al,-$81B[bp]
xor	ah,ah
push	ax
! Debug: func () unsigned short = ata_cmd_packet+0 (used reg = )
call	_ata_cmd_packet
add	sp,*$14
! Debug: eq unsigned short = ax+0 to unsigned short error = [S+$81E-$81C] (used reg = )
mov	-$81A[bp],ax
! Debug: ne int = const 0 to unsigned short = ax+0 (used reg = )
test	ax,ax
je  	.2D3
.2D4:
! 2293     return 12;
mov	ax,*$C
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 2294   tcpa_ipl((Bit32u)1L,(Bit32u)boot_segment,(Bit32u)0L,(Bit32u)512L);
.2D3:
! Debug: list unsigned long = const $200 (used reg = )
mov	ax,#$200
xor	bx,bx
push	bx
push	ax
! Debug: list unsigned long = const 0 (used reg = )
xor	ax,ax
xor	bx,bx
push	bx
push	ax
! Debug: cast unsigned long = const 0 to unsigned short boot_segment = [S+$826-$816] (used reg = )
mov	ax,-$814[bp]
xor	bx,bx
! Debug: list unsigned long = bx+0 (used reg = )
push	bx
push	ax
! Debug: list unsigned long = const 1 (used reg = )
mov	ax,*1
xor	bx,bx
push	bx
push	ax
! Debug: func () void = tcpa_ipl+0 (used reg = )
call	_tcpa_ipl
add	sp,*$10
!BCC_EOS
! 2295   switch(read_byte(ebda_seg,&((ebda_data_t *) 0)->cdemu.media)) {
! Debug: list * unsigned char = const $23B (used reg = )
mov	ax,#$23B
push	ax
! Debug: list unsigned short ebda_seg = [S+$820-4] (used reg = )
push	-2[bp]
! Debug: func () unsigned char = read_byte+0 (used reg = )
call	_read_byte
add	sp,*4
br 	.2D7
! 2296     case 0x01:
! 2297       write_word(ebda_seg,&((ebda_data_t *) 0)->cdemu.vdevice.spt,15);
.2D8:
! Debug: list int = const $F (used reg = )
mov	ax,*$F
push	ax
! Debug: list * unsigned short = const $24E (used reg = )
mov	ax,#$24E
push	ax
! Debug: list unsigned short ebda_seg = [S+$822-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
add	sp,*6
!BCC_EOS
! 2298       write_word(ebda_seg,&((ebda_data_t *) 0)->cdemu.vdevice.cylinders,80);
! Debug: list int = const $50 (used reg = )
mov	ax,*$50
push	ax
! Debug: list * unsigned short = const $24C (used reg = )
mov	ax,#$24C
push	ax
! Debug: list unsigned short ebda_seg = [S+$822-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
add	sp,*6
!BCC_EOS
! 2299       write_word(ebda_seg,&((ebda_data_t *) 0)->cdemu.vdevice.heads,2);
! Debug: list int = const 2 (used reg = )
mov	ax,*2
push	ax
! Debug: list * unsigned short = const $24A (used reg = )
mov	ax,#$24A
push	ax
! Debug: list unsigned short ebda_seg = [S+$822-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
add	sp,*6
!BCC_EOS
! 2300       break;
br 	.2D5
!BCC_EOS
! 2301     case 0x02:
! 2302       write_word(ebda_seg,&((ebda_data_t *) 0)->cdemu.vdevice.spt,18);
.2D9:
! Debug: list int = const $12 (used reg = )
mov	ax,*$12
push	ax
! Debug: list * unsigned short = const $24E (used reg = )
mov	ax,#$24E
push	ax
! Debug: list unsigned short ebda_seg = [S+$822-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
add	sp,*6
!BCC_EOS
! 2303       write_word(ebda_seg,&((ebda_data_t *) 0)->cdemu.vdevice.cylinders,80);
! Debug: list int = const $50 (used reg = )
mov	ax,*$50
push	ax
! Debug: list * unsigned short = const $24C (used reg = )
mov	ax,#$24C
push	ax
! Debug: list unsigned short ebda_seg = [S+$822-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
add	sp,*6
!BCC_EOS
! 2304       write_word(ebda_seg,&((ebda_data_t *) 0)->cdemu.vdevice.heads,2);
! Debug: list int = const 2 (used reg = )
mov	ax,*2
push	ax
! Debug: list * unsigned short = const $24A (used reg = )
mov	ax,#$24A
push	ax
! Debug: list unsigned short ebda_seg = [S+$822-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
add	sp,*6
!BCC_EOS
! 2305       break;
br 	.2D5
!BCC_EOS
! 2306     case 0x03:
! 2307       write_word(ebda_seg,&((ebda_data_t *) 0)->cdemu.vdevice.spt,36);
.2DA:
! Debug: list int = const $24 (used reg = )
mov	ax,*$24
push	ax
! Debug: list * unsigned short = const $24E (used reg = )
mov	ax,#$24E
push	ax
! Debug: list unsigned short ebda_seg = [S+$822-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
add	sp,*6
!BCC_EOS
! 2308       write_word(ebda_seg,&((ebda_data_t *) 0)->cdemu.vdevice.cylinders,80);
! Debug: list int = const $50 (used reg = )
mov	ax,*$50
push	ax
! Debug: list * unsigned short = const $24C (used reg = )
mov	ax,#$24C
push	ax
! Debug: list unsigned short ebda_seg = [S+$822-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
add	sp,*6
!BCC_EOS
! 2309       write_word(ebda_seg,&((ebda_data_t *) 0)->cdemu.vdevice.heads,2);
! Debug: list int = const 2 (used reg = )
mov	ax,*2
push	ax
! Debug: list * unsigned short = const $24A (used reg = )
mov	ax,#$24A
push	ax
! Debug: list unsigned short ebda_seg = [S+$822-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
add	sp,*6
!BCC_EOS
! 2310       break;
br 	.2D5
!BCC_EOS
! 2311     case 0x04:
! 2312       write_word(ebda_seg,&((ebda_data_t *) 0)->cdemu.vdevice.spt,read_byte(boot_segment,446+6)&0x3f);
.2DB:
! Debug: list int = const $1C4 (used reg = )
mov	ax,#$1C4
push	ax
! Debug: list unsigned short boot_segment = [S+$820-$816] (used reg = )
push	-$814[bp]
! Debug: func () unsigned char = read_byte+0 (used reg = )
call	_read_byte
add	sp,*4
! Debug: and int = const $3F to unsigned char = al+0 (used reg = )
and	al,*$3F
! Debug: list unsigned char = al+0 (used reg = )
xor	ah,ah
push	ax
! Debug: list * unsigned short = const $24E (used reg = )
mov	ax,#$24E
push	ax
! Debug: list unsigned short ebda_seg = [S+$822-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
add	sp,*6
!BCC_EOS
! 2313       write_word(ebda_seg,&((ebda_data_t *) 0)->cdemu.vdevice.cylinders,
! 2314        (read_byte(boot_segment,446+6)<<2) + read_byte(boot_segment,446+7) + 1);
! Debug: list int = const $1C5 (used reg = )
mov	ax,#$1C5
push	ax
! Debug: list unsigned short boot_segment = [S+$820-$816] (used reg = )
push	-$814[bp]
! Debug: func () unsigned char = read_byte+0 (used reg = )
call	_read_byte
add	sp,*4
push	ax
! Debug: list int = const $1C4 (used reg = )
mov	ax,#$1C4
push	ax
! Debug: list unsigned short boot_segment = [S+$822-$816] (used reg = )
push	-$814[bp]
! Debug: func () unsigned char = read_byte+0 (used reg = )
call	_read_byte
add	sp,*4
! Debug: sl int = const 2 to unsigned char = al+0 (used reg = )
xor	ah,ah
shl	ax,*1
shl	ax,*1
! Debug: add unsigned char (temp) = [S+$820-$820] to unsigned int = ax+0 (used reg = )
add	al,0+..FFF8[bp]
adc	ah,*0
inc	sp
inc	sp
! Debug: add int = const 1 to unsigned int = ax+0 (used reg = )
! Debug: list unsigned int = ax+1 (used reg = )
inc	ax
push	ax
! Debug: list * unsigned short = const $24C (used reg = )
mov	ax,#$24C
push	ax
! Debug: list unsigned short ebda_seg = [S+$822-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
add	sp,*6
!BCC_EOS
! 2315       write_word(ebda_seg,&((ebda_data_t *) 0)->cdemu.vdevice.heads,read_byte(boot_segment,446+5) + 1);
! Debug: list int = const $1C3 (used reg = )
mov	ax,#$1C3
push	ax
! Debug: list unsigned short boot_segment = [S+$820-$816] (used reg = )
push	-$814[bp]
! Debug: func () unsigned char = read_byte+0 (used reg = )
call	_read_byte
add	sp,*4
! Debug: add int = const 1 to unsigned char = al+0 (used reg = )
xor	ah,ah
! Debug: list unsigned int = ax+1 (used reg = )
inc	ax
push	ax
! Debug: list * unsigned short = const $24A (used reg = )
mov	ax,#$24A
push	ax
! Debug: list unsigned short ebda_seg = [S+$822-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
add	sp,*6
!BCC_EOS
! 2316       break;
jmp .2D5
!BCC_EOS
! 2317    }
! 2318   if(read_byte(ebda_seg,&((ebda_data_t *) 0)->cdemu.media)!=0) {
jmp .2D5
.2D7:
sub	al,*1
beq 	.2D8
sub	al,*1
beq 	.2D9
sub	al,*1
beq 	.2DA
sub	al,*1
beq 	.2DB
.2D5:
..FFF8	=	-$81E
! Debug: list * unsigned char = const $23B (used reg = )
mov	ax,#$23B
push	ax
! Debug: list unsigned short ebda_seg = [S+$820-4] (used reg = )
push	-2[bp]
! Debug: func () unsigned char = read_byte+0 (used reg = )
call	_read_byte
add	sp,*4
! Debug: ne int = const 0 to unsigned char = al+0 (used reg = )
test	al,al
je  	.2DC
.2DD:
! 2319     if(read_byte(ebda_seg,&((ebda_data_t *) 0)->cdemu.emulated_drive)==0x00)
! Debug: list * unsigned char = const $23C (used reg = )
mov	ax,#$23C
push	ax
! Debug: list unsigned short ebda_seg = [S+$820-4] (used reg = )
push	-2[bp]
! Debug: func () unsigned char = read_byte+0 (used reg = )
call	_read_byte
add	sp,*4
! Debug: logeq int = const 0 to unsigned char = al+0 (used reg = )
test	al,al
jne 	.2DE
.2DF:
! 2320       write_byte(0x40,0x10,read_byte(0x40,0x10)|0x41);
! Debug: list int = const $10 (used reg = )
mov	ax,*$10
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () unsigned char = read_byte+0 (used reg = )
call	_read_byte
add	sp,*4
! Debug: or int = const $41 to unsigned char = al+0 (used reg = )
or	al,*$41
! Debug: list unsigned char = al+0 (used reg = )
xor	ah,ah
push	ax
! Debug: list int = const $10 (used reg = )
mov	ax,*$10
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 2321     else
! 2322       write_byte(ebda_seg, &((ebda_data_t *) 0)->ata.hdcount, read_byte(ebda_seg, &((ebda_data_t *) 0)->ata.hdcount) + 1)
jmp .2E0
.2DE:
! 2322 ;
! Debug: list * unsigned char = const $212 (used reg = )
mov	ax,#$212
push	ax
! Debug: list unsigned short ebda_seg = [S+$820-4] (used reg = )
push	-2[bp]
! Debug: func () unsigned char = read_byte+0 (used reg = )
call	_read_byte
add	sp,*4
! Debug: add int = const 1 to unsigned char = al+0 (used reg = )
xor	ah,ah
! Debug: list unsigned int = ax+1 (used reg = )
inc	ax
push	ax
! Debug: list * unsigned char = const $212 (used reg = )
mov	ax,#$212
push	ax
! Debug: list unsigned short ebda_seg = [S+$822-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 2323    }
.2E0:
! 2324   if(read_byte(ebda_seg,&((ebda_data_t *) 0)->cdemu.media)!=0)
.2DC:
! Debug: list * unsigned char = const $23B (used reg = )
mov	ax,#$23B
push	ax
! Debug: list unsigned short ebda_seg = [S+$820-4] (used reg = )
push	-2[bp]
! Debug: func () unsigned char = read_byte+0 (used reg = )
call	_read_byte
add	sp,*4
! Debug: ne int = const 0 to unsigned char = al+0 (used reg = )
test	al,al
je  	.2E1
.2E2:
! 2325     write_byte(ebda_seg,&((ebda_data_t *) 0)->cdemu.active,0x01);
! Debug: list int = const 1 (used reg = )
mov	ax,*1
push	ax
! Debug: list * unsigned char = const $23A (used reg = )
mov	ax,#$23A
push	ax
! Debug: list unsigned short ebda_seg = [S+$822-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 2326   return (read_byte(ebda_seg,&((ebda_data_t *) 0)->cdemu.emulated_drive)*0x100)+0;
.2E1:
! Debug: list * unsigned char = const $23C (used reg = )
mov	ax,#$23C
push	ax
! Debug: list unsigned short ebda_seg = [S+$820-4] (used reg = )
push	-2[bp]
! Debug: func () unsigned char = read_byte+0 (used reg = )
call	_read_byte
add	sp,*4
! Debug: mul int = const $100 to unsigned char = al+0 (used reg = )
xor	ah,ah
mov	cx,#$100
imul	cx
! Debug: add int = const 0 to unsigned int = ax+0 (used reg = )
! Debug: cast unsigned short = const 0 to unsigned int = ax+0 (used reg = )
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 2327 }
! 2328   void
! Register BX used in function cdrom_boot
! 2329 int14_function(regs, ds, iret_addr)
! 2330   pusha_regs_t regs;
export	_int14_function
_int14_function:
!BCC_EOS
! 2331   Bit16u ds;
!BCC_EOS
! 2332   iret_addr_t iret_addr;
!BCC_EOS
! 2333 {
! 2334   Bit16u addr,timer,val16;
!BCC_EOS
! 2335   Bit8u timeout;
!BCC_EOS
! 2336 #asm
push	bp
mov	bp,sp
add	sp,*-8
!BCC_EOS
!BCC_ASM
_int14_function.ds	set	$1C
.int14_function.ds	set	$14
_int14_function.timer	set	4
.int14_function.timer	set	-4
_int14_function.timeout	set	1
.int14_function.timeout	set	-7
_int14_function.iret_addr	set	$1E
.int14_function.iret_addr	set	$16
_int14_function.addr	set	6
.int14_function.addr	set	-2
_int14_function.val16	set	2
.int14_function.val16	set	-6
_int14_function.regs	set	$C
.int14_function.regs	set	4
  sti
! 2338 endasm
!BCC_ENDASM
!BCC_EOS
! 2339   addr = read_word(0x0040, (regs.u.r16.dx << 1));
! Debug: sl int = const 1 to unsigned short regs = [S+$A+$C] (used reg = )
mov	ax,$E[bp]
shl	ax,*1
! Debug: list unsigned int = ax+0 (used reg = )
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () unsigned short = read_word+0 (used reg = )
call	_read_word
add	sp,*4
! Debug: eq unsigned short = ax+0 to unsigned short addr = [S+$A-4] (used reg = )
mov	-2[bp],ax
!BCC_EOS
! 2340   timeout = read_byte(0x0040, 0x007C + regs.u.r16.dx);
! Debug: add unsigned short regs = [S+$A+$C] to int = const $7C (used reg = )
! Debug: expression subtree swapping
mov	ax,$E[bp]
! Debug: list unsigned int = ax+$7C (used reg = )
add	ax,*$7C
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () unsigned char = read_byte+0 (used reg = )
call	_read_byte
add	sp,*4
! Debug: eq unsigned char = al+0 to unsigned char timeout = [S+$A-9] (used reg = )
mov	-7[bp],al
!BCC_EOS
! 2341   if ((regs.u.r16.dx < 4) && (addr > 0)) {
! Debug: lt int = const 4 to unsigned short regs = [S+$A+$C] (used reg = )
mov	ax,$E[bp]
cmp	ax,*4
bhis	.2E3
.2E5:
! Debug: gt int = const 0 to unsigned short addr = [S+$A-4] (used reg = )
mov	ax,-2[bp]
test	ax,ax
beq 	.2E3
.2E4:
! 2342     switch (regs.u.r8.ah) {
mov	al,$13[bp]
br 	.2E8
! 2343       case 0:
! 2344         outb(addr+3, inb(addr+3) | 0x80);
.2E9:
! Debug: add int = const 3 to unsigned short addr = [S+$A-4] (used reg = )
mov	ax,-2[bp]
! Debug: list unsigned int = ax+3 (used reg = )
add	ax,*3
push	ax
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
inc	sp
inc	sp
! Debug: or int = const $80 to unsigned char = al+0 (used reg = )
or	al,#$80
! Debug: list unsigned char = al+0 (used reg = )
xor	ah,ah
push	ax
! Debug: add int = const 3 to unsigned short addr = [S+$C-4] (used reg = )
mov	ax,-2[bp]
! Debug: list unsigned int = ax+3 (used reg = )
add	ax,*3
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 2345         if (regs.u.r8.al & 0xE0 == 0) {
! Debug: and int = const 0 to unsigned char regs = [S+$A+$10] (used reg = )
mov	al,$12[bp]
xor	al,al
test	al,al
je  	.2EA
.2EB:
! 2346           outb(addr, 0x17);
! Debug: list int = const $17 (used reg = )
mov	ax,*$17
push	ax
! Debug: list unsigned short addr = [S+$C-4] (used reg = )
push	-2[bp]
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 2347           outb(addr+1, 0x04);
! Debug: list int = const 4 (used reg = )
mov	ax,*4
push	ax
! Debug: add int = const 1 to unsigned short addr = [S+$C-4] (used reg = )
mov	ax,-2[bp]
! Debug: list unsigned int = ax+1 (used reg = )
inc	ax
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 2348         } else {
jmp .2EC
.2EA:
! 2349           val16 = 0x600 >> ((regs.u.r8.al & 0xE0) >> 5);
! Debug: and int = const $E0 to unsigned char regs = [S+$A+$10] (used reg = )
mov	al,$12[bp]
and	al,#$E0
! Debug: sr int = const 5 to unsigned char = al+0 (used reg = )
xor	ah,ah
mov	cl,*5
shr	ax,cl
! Debug: sr unsigned int = ax+0 to int = const $600 (used reg = )
mov	bx,ax
mov	ax,#$600
mov	cx,bx
sar	ax,cl
! Debug: eq int = ax+0 to unsigned short val16 = [S+$A-8] (used reg = )
mov	-6[bp],ax
!BCC_EOS
! 2350           outb(addr, val16 & 0xFF);
! Debug: and int = const $FF to unsigned short val16 = [S+$A-8] (used reg = )
mov	al,-6[bp]
! Debug: list unsigned char = al+0 (used reg = )
xor	ah,ah
push	ax
! Debug: list unsigned short addr = [S+$C-4] (used reg = )
push	-2[bp]
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 2351           outb(addr+1, val16 >> 8);
! Debug: sr int = const 8 to unsigned short val16 = [S+$A-8] (used reg = )
mov	ax,-6[bp]
mov	al,ah
xor	ah,ah
! Debug: list unsigned int = ax+0 (used reg = )
push	ax
! Debug: add int = const 1 to unsigned short addr = [S+$C-4] (used reg = )
mov	ax,-2[bp]
! Debug: list unsigned int = ax+1 (used reg = )
inc	ax
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 2352         }
! 2353         outb(addr+3, regs.u.r8.al & 0x1F);
.2EC:
! Debug: and int = const $1F to unsigned char regs = [S+$A+$10] (used reg = )
mov	al,$12[bp]
and	al,*$1F
! Debug: list unsigned char = al+0 (used reg = )
xor	ah,ah
push	ax
! Debug: add int = const 3 to unsigned short addr = [S+$C-4] (used reg = )
mov	ax,-2[bp]
! Debug: list unsigned int = ax+3 (used reg = )
add	ax,*3
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 2354         regs.u.r8.ah = inb(addr+5);
! Debug: add int = const 5 to unsigned short addr = [S+$A-4] (used reg = )
mov	ax,-2[bp]
! Debug: list unsigned int = ax+5 (used reg = )
add	ax,*5
push	ax
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char regs = [S+$A+$11] (used reg = )
mov	$13[bp],al
!BCC_EOS
! 2355         regs.u.r8.al = inb(addr+6);
! Debug: add int = const 6 to unsigned short addr = [S+$A-4] (used reg = )
mov	ax,-2[bp]
! Debug: list unsigned int = ax+6 (used reg = )
add	ax,*6
push	ax
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char regs = [S+$A+$10] (used reg = )
mov	$12[bp],al
!BCC_EOS
! 2356         iret_addr.flags.u.r8.flagsl &= 0xfe;
! Debug: andab int = const $FE to unsigned char iret_addr = [S+$A+$18] (used reg = )
mov	al,$1A[bp]
and	al,#$FE
mov	$1A[bp],al
!BCC_EOS
! 2357         break;
br 	.2E6
!BCC_EOS
! 2358       case 1:
! 2359         timer = read_word(0x0040, 0x006C);
.2ED:
! Debug: list int = const $6C (used reg = )
mov	ax,*$6C
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () unsigned short = read_word+0 (used reg = )
call	_read_word
add	sp,*4
! Debug: eq unsigned short = ax+0 to unsigned short timer = [S+$A-6] (used reg = )
mov	-4[bp],ax
!BCC_EOS
! 2360         while (((inb(addr+5) & 0x60) != 0x60) && (timeout)) {
jmp .2EF
.2F0:
! 2361           val16 = read_word(0x0040, 0x006C);
! Debug: list int = const $6C (used reg = )
mov	ax,*$6C
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () unsigned short = read_word+0 (used reg = )
call	_read_word
add	sp,*4
! Debug: eq unsigned short = ax+0 to unsigned short val16 = [S+$A-8] (used reg = )
mov	-6[bp],ax
!BCC_EOS
! 2362           if (val16 != timer) {
! Debug: ne unsigned short timer = [S+$A-6] to unsigned short val16 = [S+$A-8] (used reg = )
mov	ax,-6[bp]
cmp	ax,-4[bp]
je  	.2F1
.2F2:
! 2363             timer = val16;
! Debug: eq unsigned short val16 = [S+$A-8] to unsigned short timer = [S+$A-6] (used reg = )
mov	ax,-6[bp]
mov	-4[bp],ax
!BCC_EOS
! 2364             timeout--;
! Debug: postdec unsigned char timeout = [S+$A-9] (used reg = )
mov	al,-7[bp]
dec	ax
mov	-7[bp],al
!BCC_EOS
! 2365             }
! 2366           }
.2F1:
! 2367         if (timeout) outb(addr, regs.u.r8.al);
.2EF:
! Debug: add int = const 5 to unsigned short addr = [S+$A-4] (used reg = )
mov	ax,-2[bp]
! Debug: list unsigned int = ax+5 (used reg = )
add	ax,*5
push	ax
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
inc	sp
inc	sp
! Debug: and int = const $60 to unsigned char = al+0 (used reg = )
and	al,*$60
! Debug: ne int = const $60 to unsigned char = al+0 (used reg = )
cmp	al,*$60
je  	.2F3
.2F4:
mov	al,-7[bp]
test	al,al
jne	.2F0
.2F3:
.2EE:
mov	al,-7[bp]
test	al,al
je  	.2F5
.2F6:
! Debug: list unsigned char regs = [S+$A+$10] (used reg = )
mov	al,$12[bp]
xor	ah,ah
push	ax
! Debug: list unsigned short addr = [S+$C-4] (used reg = )
push	-2[bp]
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 2368         regs.u.r8.ah = inb(addr+5);
.2F5:
! Debug: add int = const 5 to unsigned short addr = [S+$A-4] (used reg = )
mov	ax,-2[bp]
! Debug: list unsigned int = ax+5 (used reg = )
add	ax,*5
push	ax
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char regs = [S+$A+$11] (used reg = )
mov	$13[bp],al
!BCC_EOS
! 2369         if (!timeout) regs.u.r8.ah |= 0x80;
mov	al,-7[bp]
test	al,al
jne 	.2F7
.2F8:
! Debug: orab int = const $80 to unsigned char regs = [S+$A+$11] (used reg = )
mov	al,$13[bp]
or	al,#$80
mov	$13[bp],al
!BCC_EOS
! 2370         iret_addr.flags.u.r8.flagsl &= 0xfe;
.2F7:
! Debug: andab int = const $FE to unsigned char iret_addr = [S+$A+$18] (used reg = )
mov	al,$1A[bp]
and	al,#$FE
mov	$1A[bp],al
!BCC_EOS
! 2371         break;
br 	.2E6
!BCC_EOS
! 2372       case 2:
! 2373         timer = read_word(0x0040, 0x006C);
.2F9:
! Debug: list int = const $6C (used reg = )
mov	ax,*$6C
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () unsigned short = read_word+0 (used reg = )
call	_read_word
add	sp,*4
! Debug: eq unsigned short = ax+0 to unsigned short timer = [S+$A-6] (used reg = )
mov	-4[bp],ax
!BCC_EOS
! 2374         while (((inb(addr+5) & 0x01) == 0) && (timeout)) {
jmp .2FB
.2FC:
! 2375           val16 = read_word(0x0040, 0x006C);
! Debug: list int = const $6C (used reg = )
mov	ax,*$6C
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () unsigned short = read_word+0 (used reg = )
call	_read_word
add	sp,*4
! Debug: eq unsigned short = ax+0 to unsigned short val16 = [S+$A-8] (used reg = )
mov	-6[bp],ax
!BCC_EOS
! 2376           if (val16 != timer) {
! Debug: ne unsigned short timer = [S+$A-6] to unsigned short val16 = [S+$A-8] (used reg = )
mov	ax,-6[bp]
cmp	ax,-4[bp]
je  	.2FD
.2FE:
! 2377             timer = val16;
! Debug: eq unsigned short val16 = [S+$A-8] to unsigned short timer = [S+$A-6] (used reg = )
mov	ax,-6[bp]
mov	-4[bp],ax
!BCC_EOS
! 2378             timeout--;
! Debug: postdec unsigned char timeout = [S+$A-9] (used reg = )
mov	al,-7[bp]
dec	ax
mov	-7[bp],al
!BCC_EOS
! 2379             }
! 2380           }
.2FD:
! 2381         if (timeout) {
.2FB:
! Debug: add int = const 5 to unsigned short addr = [S+$A-4] (used reg = )
mov	ax,-2[bp]
! Debug: list unsigned int = ax+5 (used reg = )
add	ax,*5
push	ax
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
inc	sp
inc	sp
! Debug: and int = const 1 to unsigned char = al+0 (used reg = )
and	al,*1
! Debug: logeq int = const 0 to unsigned char = al+0 (used reg = )
test	al,al
jne 	.2FF
.300:
mov	al,-7[bp]
test	al,al
jne	.2FC
.2FF:
.2FA:
mov	al,-7[bp]
test	al,al
je  	.301
.302:
! 2382           regs.u.r8.ah = 0;
! Debug: eq int = const 0 to unsigned char regs = [S+$A+$11] (used reg = )
xor	al,al
mov	$13[bp],al
!BCC_EOS
! 2383           regs.u.r8.al = inb(addr);
! Debug: list unsigned short addr = [S+$A-4] (used reg = )
push	-2[bp]
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char regs = [S+$A+$10] (used reg = )
mov	$12[bp],al
!BCC_EOS
! 2384         } else {
jmp .303
.301:
! 2385           regs.u.r8.ah = inb(addr+5);
! Debug: add int = const 5 to unsigned short addr = [S+$A-4] (used reg = )
mov	ax,-2[bp]
! Debug: list unsigned int = ax+5 (used reg = )
add	ax,*5
push	ax
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char regs = [S+$A+$11] (used reg = )
mov	$13[bp],al
!BCC_EOS
! 2386           }
! 2387         iret_addr.flags.u.r8.flagsl &= 0xfe;
.303:
! Debug: andab int = const $FE to unsigned char iret_addr = [S+$A+$18] (used reg = )
mov	al,$1A[bp]
and	al,#$FE
mov	$1A[bp],al
!BCC_EOS
! 2388         break;
jmp .2E6
!BCC_EOS
! 2389       case 3:
! 2390         regs.u.r8.ah = inb(addr+5);
.304:
! Debug: add int = const 5 to unsigned short addr = [S+$A-4] (used reg = )
mov	ax,-2[bp]
! Debug: list unsigned int = ax+5 (used reg = )
add	ax,*5
push	ax
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char regs = [S+$A+$11] (used reg = )
mov	$13[bp],al
!BCC_EOS
! 2391        
! 2391  regs.u.r8.al = inb(addr+6);
! Debug: add int = const 6 to unsigned short addr = [S+$A-4] (used reg = )
mov	ax,-2[bp]
! Debug: list unsigned int = ax+6 (used reg = )
add	ax,*6
push	ax
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char regs = [S+$A+$10] (used reg = )
mov	$12[bp],al
!BCC_EOS
! 2392         iret_addr.flags.u.r8.flagsl &= 0xfe;
! Debug: andab int = const $FE to unsigned char iret_addr = [S+$A+$18] (used reg = )
mov	al,$1A[bp]
and	al,#$FE
mov	$1A[bp],al
!BCC_EOS
! 2393         break;
jmp .2E6
!BCC_EOS
! 2394       default:
! 2395         iret_addr.flags.u.r8.flagsl |= 0x01;
.305:
! Debug: orab int = const 1 to unsigned char iret_addr = [S+$A+$18] (used reg = )
mov	al,$1A[bp]
or	al,*1
mov	$1A[bp],al
!BCC_EOS
! 2396       }
! 2397   } else {
jmp .2E6
.2E8:
sub	al,*0
beq 	.2E9
sub	al,*1
beq 	.2ED
sub	al,*1
beq 	.2F9
sub	al,*1
je 	.304
jmp	.305
.2E6:
..FFF7	=	-$A
jmp .306
.2E3:
! 2398     iret_addr.flags.u.r8.flagsl |= 0x01;
! Debug: orab int = const 1 to unsigned char iret_addr = [S+$A+$18] (used reg = )
mov	al,$1A[bp]
or	al,*1
mov	$1A[bp],al
!BCC_EOS
! 2399     }
! 2400 }
.306:
mov	sp,bp
pop	bp
ret
! 2401   void
! Register BX used in function int14_function
! 2402 int15_function(regs, ES, DS, FLAGS)
! 2403   pusha_regs_t regs;
export	_int15_function
_int15_function:
!BCC_EOS
! 2404   Bit16u ES, DS, FLAGS;
!BCC_EOS
! 2405 {
! 2406   Bit16u ebda_seg=read_word(0x0040,0x000E);
push	bp
mov	bp,sp
dec	sp
dec	sp
! Debug: list int = const $E (used reg = )
mov	ax,*$E
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () unsigned short = read_word+0 (used reg = )
call	_read_word
add	sp,*4
! Debug: eq unsigned short = ax+0 to unsigned short ebda_seg = [S+4-4] (used reg = )
mov	-2[bp],ax
!BCC_EOS
! 2407   bx_bool prev_a20_enable;
!BCC_EOS
! 2408   Bit16u base15_00;
!BCC_EOS
! 2409   Bit8u base23_16;
!BCC_EOS
! 2410   Bit16u ss;
!BCC_EOS
! 2411   Bit16u CX,DX;
!BCC_EOS
! 2412   Bit16u bRegister;
!BCC_EOS
! 2413   Bit8u irqDisable;
!BCC_EOS
! 2414 ;
add	sp,*-$10
!BCC_EOS
! 2415   switch (regs.u.r8.ah) {
mov	al,$13[bp]
br 	.309
! 2416     case 0x24:
! 2417       switch (regs.u.r8.al) {
.30A:
mov	al,$12[bp]
br 	.30D
! 2418         case 0x00:
! 2419           set_enable_a20(0);
.30E:
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: func () unsigned short = set_enable_a20+0 (used reg = )
call	_set_enable_a20
inc	sp
inc	sp
!BCC_EOS
! 2420           FLAGS &= 0xfffe;
! Debug: andab unsigned int = const $FFFE to unsigned short FLAGS = [S+$14+$16] (used reg = )
mov	ax,$18[bp]
and	al,#$FE
mov	$18[bp],ax
!BCC_EOS
! 2421           regs.u.r8.ah = 0;
! Debug: eq int = const 0 to unsigned char regs = [S+$14+$11] (used reg = )
xor	al,al
mov	$13[bp],al
!BCC_EOS
! 2422           break;
br 	.30B
!BCC_EOS
! 2423         case 0x01:
! 2424           set_enable_a20(1);
.30F:
! Debug: list int = const 1 (used reg = )
mov	ax,*1
push	ax
! Debug: func () unsigned short = set_enable_a20+0 (used reg = )
call	_set_enable_a20
inc	sp
inc	sp
!BCC_EOS
! 2425           FLAGS &= 0xfffe;
! Debug: andab unsigned int = const $FFFE to unsigned short FLAGS = [S+$14+$16] (used reg = )
mov	ax,$18[bp]
and	al,#$FE
mov	$18[bp],ax
!BCC_EOS
! 2426           regs.u.r8.ah = 0;
! Debug: eq int = const 0 to unsigned char regs = [S+$14+$11] (used reg = )
xor	al,al
mov	$13[bp],al
!BCC_EOS
! 2427           break;
jmp .30B
!BCC_EOS
! 2428         case 0x02:
! 2429           regs.u.r8.al = (inb(0x92) >> 1) & 0x01;
.310:
! Debug: list int = const $92 (used reg = )
mov	ax,#$92
push	ax
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
inc	sp
inc	sp
! Debug: sr int = const 1 to unsigned char = al+0 (used reg = )
xor	ah,ah
shr	ax,*1
! Debug: and int = const 1 to unsigned int = ax+0 (used reg = )
and	al,*1
! Debug: eq unsigned char = al+0 to unsigned char regs = [S+$14+$10] (used reg = )
mov	$12[bp],al
!BCC_EOS
! 2430           FLAGS &= 0xfffe;
! Debug: andab unsigned int = const $FFFE to unsigned short FLAGS = [S+$14+$16] (used reg = )
mov	ax,$18[bp]
and	al,#$FE
mov	$18[bp],ax
!BCC_EOS
! 2431           regs.u.r8.ah = 0;
! Debug: eq int = const 0 to unsigned char regs = [S+$14+$11] (used reg = )
xor	al,al
mov	$13[bp],al
!BCC_EOS
! 2432           break;
jmp .30B
!BCC_EOS
! 2433         case 0x03:
! 2434           FLAGS &= 0xfffe;
.311:
! Debug: andab unsigned int = const $FFFE to unsigned short FLAGS = [S+$14+$16] (used reg = )
mov	ax,$18[bp]
and	al,#$FE
mov	$18[bp],ax
!BCC_EOS
! 2435           regs.u.r8.ah = 0;
! Debug: eq int = const 0 to unsigned char regs = [S+$14+$11] (used reg = )
xor	al,al
mov	$13[bp],al
!BCC_EOS
! 2436           regs.u.r16.bx = 3;
! Debug: eq int = const 3 to unsigned short regs = [S+$14+$A] (used reg = )
mov	ax,*3
mov	$C[bp],ax
!BCC_EOS
! 2437           break;
jmp .30B
!BCC_EOS
! 2438         default:
! 2439           bios_printf(4, "int15: Func 24h, subfunc %02xh, A20 gate control not supported\n", (unsigned) regs.u.r8.al);
.312:
! Debug: list unsigned char regs = [S+$14+$10] (used reg = )
mov	al,$12[bp]
xor	ah,ah
push	ax
! Debug: list * char = .313+0 (used reg = )
mov	bx,#.313
push	bx
! Debug: list int = const 4 (used reg = )
mov	ax,*4
push	ax
! Debug: func () void = bios_printf+0 (used reg = )
call	_bios_printf
add	sp,*6
!BCC_EOS
! 2440           FLAGS |= 0x0001;
! Debug: orab int = const 1 to unsigned short FLAGS = [S+$14+$16] (used reg = )
mov	ax,$18[bp]
or	al,*1
mov	$18[bp],ax
!BCC_EOS
! 2441           regs.u.r8.ah = 0x86;
! Debug: eq int = const $86 to unsigned char regs = [S+$14+$11] (used reg = )
mov	al,#$86
mov	$13[bp],al
!BCC_EOS
! 2442       }
! 2443       break;
jmp .30B
.30D:
sub	al,*0
beq 	.30E
sub	al,*1
beq 	.30F
sub	al,*1
je 	.310
sub	al,*1
je 	.311
jmp	.312
.30B:
br 	.307
!BCC_EOS
! 2444     case 0x41:
! 2445       FLAGS |= 0x0001;
.314:
! Debug: orab int = const 1 to unsigned short FLAGS = [S+$14+$16] (used reg = )
mov	ax,$18[bp]
or	al,*1
mov	$18[bp],ax
!BCC_EOS
! 2446       regs.u.r8.ah = 0x86;
! Debug: eq int = const $86 to unsigned char regs = [S+$14+$11] (used reg = )
mov	al,#$86
mov	$13[bp],al
!BCC_EOS
! 2447       break;
br 	.307
!BCC_EOS
! 2448     case 0x4f:
! 2449       FLAGS |= 0x0001;
.315:
! Debug: orab int = const 1 to unsigned short FLAGS = [S+$14+$16] (used reg = )
mov	ax,$18[bp]
or	al,*1
mov	$18[bp],ax
!BCC_EOS
! 2450       break;
br 	.307
!BCC_EOS
! 2451     case 0x52:
! 2452       FLAGS &= 0xfffe;
.316:
! Debug: andab unsigned int = const $FFFE to unsigned short FLAGS = [S+$14+$16] (used reg = )
mov	ax,$18[bp]
and	al,#$FE
mov	$18[bp],ax
!BCC_EOS
! 2453       regs.u.r8.ah = 0;
! Debug: eq int = const 0 to unsigned char regs = [S+$14+$11] (used reg = )
xor	al,al
mov	$13[bp],al
!BCC_EOS
! 2454       break;
br 	.307
!BCC_EOS
! 2455     case 0x83: {
.317:
! 2456       if( regs.u.r8.al == 0 ) {
! Debug: logeq int = const 0 to unsigned char regs = [S+$14+$10] (used reg = )
mov	al,$12[bp]
test	al,al
bne 	.318
.319:
! 2457         if( ( read_byte( 0x40, 0xA0 ) & 1 ) == 0 ) {
! Debug: list int = const $A0 (used reg = )
mov	ax,#$A0
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () unsigned char = read_byte+0 (used reg = )
call	_read_byte
add	sp,*4
! Debug: and int = const 1 to unsigned char = al+0 (used reg = )
and	al,*1
! Debug: logeq int = const 0 to unsigned char = al+0 (used reg = )
test	al,al
bne 	.31A
.31B:
! 2458           write_byte( 0x40, 0xA0, 1 );
! Debug: list int = const 1 (used reg = )
mov	ax,*1
push	ax
! Debug: list int = const $A0 (used reg = )
mov	ax,#$A0
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 2459           write_word( 0x40, 0x98, ES );
! Debug: list unsigned short ES = [S+$14+$12] (used reg = )
push	$14[bp]
! Debug: list int = const $98 (used reg = )
mov	ax,#$98
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
add	sp,*6
!BCC_EOS
! 2460           write_word( 0x40, 0x9A, regs.u.r16.bx );
! Debug: list unsigned short regs = [S+$14+$A] (used reg = )
push	$C[bp]
! Debug: list int = const $9A (used reg = )
mov	ax,#$9A
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
add	sp,*6
!BCC_EOS
! 2461           write_word( 0x40, 0x9C, regs.u.r16.dx );
! Debug: list unsigned short regs = [S+$14+$C] (used reg = )
push	$E[bp]
! Debug: list int = const $9C (used reg = )
mov	ax,#$9C
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
add	sp,*6
!BCC_EOS
! 2462           write_word( 0x40, 0x9E, regs.u.r16.cx );
! Debug: list unsigned short regs = [S+$14+$E] (used reg = )
push	$10[bp]
! Debug: list int = const $9E (used reg = )
mov	ax,#$9E
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
add	sp,*6
!BCC_EOS
! 2463           FLAGS &= 0xfffe;
! Debug: andab unsigned int = const $FFFE to unsigned short FLAGS = [S+$14+$16] (used reg = )
mov	ax,$18[bp]
and	al,#$FE
mov	$18[bp],ax
!BCC_EOS
! 2464           irqDisable = inb( 0xA1 );
! Debug: list int = const $A1 (used reg = )
mov	ax,#$A1
push	ax
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char irqDisable = [S+$14-$13] (used reg = )
mov	-$11[bp],al
!BCC_EOS
! 2465           outb( 0xA1, irqDisable & 0xFE );
! Debug: and int = const $FE to unsigned char irqDisable = [S+$14-$13] (used reg = )
mov	al,-$11[bp]
and	al,#$FE
! Debug: list unsigned char = al+0 (used reg = )
xor	ah,ah
push	ax
! Debug: list int = const $A1 (used reg = )
mov	ax,#$A1
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 2466           bRegister = inb_cmos( 0xB );
! Debug: list int = const $B (used reg = )
mov	ax,*$B
push	ax
! Debug: func () unsigned char = inb_cmos+0 (used reg = )
call	_inb_cmos
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned short bRegister = [S+$14-$12] (used reg = )
xor	ah,ah
mov	-$10[bp],ax
!BCC_EOS
! 2467           outb_cmos( 0xB, bRegister | 0x40 );
! Debug: or int = const $40 to unsigned short bRegister = [S+$14-$12] (used reg = )
mov	ax,-$10[bp]
or	al,*$40
! Debug: list unsigned int = ax+0 (used reg = )
push	ax
! Debug: list int = const $B (used reg = )
mov	ax,*$B
push	ax
! Debug: func () void = outb_cmos+0 (used reg = )
call	_outb_cmos
add	sp,*4
!BCC_EOS
! 2468         } else {
jmp .31C
.31A:
! 2469           ;
!BCC_EOS
! 2470           FLAGS
! 2470  |= 0x0001;
! Debug: orab int = const 1 to unsigned short FLAGS = [S+$14+$16] (used reg = )
mov	ax,$18[bp]
or	al,*1
mov	$18[bp],ax
!BCC_EOS
! 2471           regs.u.r8.ah = 0x86;
! Debug: eq int = const $86 to unsigned char regs = [S+$14+$11] (used reg = )
mov	al,#$86
mov	$13[bp],al
!BCC_EOS
! 2472         }
! 2473       } else if( regs.u.r8.al == 1 ) {
.31C:
jmp .31D
.318:
! Debug: logeq int = const 1 to unsigned char regs = [S+$14+$10] (used reg = )
mov	al,$12[bp]
cmp	al,*1
jne 	.31E
.31F:
! 2474         write_byte( 0x40, 0xA0, 0 );
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: list int = const $A0 (used reg = )
mov	ax,#$A0
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 2475         FLAGS &= 0xfffe;
! Debug: andab unsigned int = const $FFFE to unsigned short FLAGS = [S+$14+$16] (used reg = )
mov	ax,$18[bp]
and	al,#$FE
mov	$18[bp],ax
!BCC_EOS
! 2476         bRegister = inb_cmos( 0xB );
! Debug: list int = const $B (used reg = )
mov	ax,*$B
push	ax
! Debug: func () unsigned char = inb_cmos+0 (used reg = )
call	_inb_cmos
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned short bRegister = [S+$14-$12] (used reg = )
xor	ah,ah
mov	-$10[bp],ax
!BCC_EOS
! 2477         outb_cmos( 0xB, bRegister & ~0x40 );
! Debug: and int = const -$41 to unsigned short bRegister = [S+$14-$12] (used reg = )
mov	ax,-$10[bp]
and	al,#$BF
! Debug: list unsigned int = ax+0 (used reg = )
push	ax
! Debug: list int = const $B (used reg = )
mov	ax,*$B
push	ax
! Debug: func () void = outb_cmos+0 (used reg = )
call	_outb_cmos
add	sp,*4
!BCC_EOS
! 2478       } else {
jmp .320
.31E:
! 2479         ;
!BCC_EOS
! 2480         FLAGS |= 0x0001;
! Debug: orab int = const 1 to unsigned short FLAGS = [S+$14+$16] (used reg = )
mov	ax,$18[bp]
or	al,*1
mov	$18[bp],ax
!BCC_EOS
! 2481         regs.u.r8.ah = 0x86;
! Debug: eq int = const $86 to unsigned char regs = [S+$14+$11] (used reg = )
mov	al,#$86
mov	$13[bp],al
!BCC_EOS
! 2482         regs.u.r8.al--;
! Debug: postdec unsigned char regs = [S+$14+$10] (used reg = )
mov	al,$12[bp]
dec	ax
mov	$12[bp],al
!BCC_EOS
! 2483       }
! 2484       break;
.320:
.31D:
br 	.307
!BCC_EOS
! 2485     }
! 2486     case 0x87:
! 2487 #asm
.321:
!BCC_EOS
!BCC_ASM
_int15_function.CX	set	6
.int15_function.CX	set	-$C
_int15_function.FLAGS	set	$2A
.int15_function.FLAGS	set	$18
_int15_function.irqDisable	set	1
.int15_function.irqDisable	set	-$11
_int15_function.DS	set	$28
.int15_function.DS	set	$16
_int15_function.DX	set	4
.int15_function.DX	set	-$E
_int15_function.base23_16	set	$B
.int15_function.base23_16	set	-7
_int15_function.bRegister	set	2
.int15_function.bRegister	set	-$10
_int15_function.ES	set	$26
.int15_function.ES	set	$14
_int15_function.ebda_seg	set	$10
.int15_function.ebda_seg	set	-2
_int15_function.base15_00	set	$C
.int15_function.base15_00	set	-6
_int15_function.ss	set	8
.int15_function.ss	set	-$A
_int15_function.regs	set	$16
.int15_function.regs	set	4
_int15_function.prev_a20_enable	set	$E
.int15_function.prev_a20_enable	set	-4
  cli
! 2489 endasm
!BCC_ENDASM
!BCC_EOS
! 2490       prev_a20_enable = set_enable_a20(1);
! Debug: list int = const 1 (used reg = )
mov	ax,*1
push	ax
! Debug: func () unsigned short = set_enable_a20+0 (used reg = )
call	_set_enable_a20
inc	sp
inc	sp
! Debug: eq unsigned short = ax+0 to unsigned short prev_a20_enable = [S+$14-6] (used reg = )
mov	-4[bp],ax
!BCC_EOS
! 2491       base15_00 = (ES << 4) + regs.u.r16.si;
! Debug: sl int = const 4 to unsigned short ES = [S+$14+$12] (used reg = )
mov	ax,$14[bp]
mov	cl,*4
shl	ax,cl
! Debug: add unsigned short regs = [S+$14+4] to unsigned int = ax+0 (used reg = )
add	ax,6[bp]
! Debug: eq unsigned int = ax+0 to unsigned short base15_00 = [S+$14-8] (used reg = )
mov	-6[bp],ax
!BCC_EOS
! 2492       base23_16 = ES >> 12;
! Debug: sr int = const $C to unsigned short ES = [S+$14+$12] (used reg = )
mov	ax,$14[bp]
mov	al,ah
xor	ah,ah
mov	cl,*4
shr	ax,cl
! Debug: eq unsigned int = ax+0 to unsigned char base23_16 = [S+$14-9] (used reg = )
mov	-7[bp],al
!BCC_EOS
! 2493       if (base15_00 < (ES<<4))
! Debug: sl int = const 4 to unsigned short ES = [S+$14+$12] (used reg = )
mov	ax,$14[bp]
mov	cl,*4
shl	ax,cl
! Debug: lt unsigned int = ax+0 to unsigned short base15_00 = [S+$14-8] (used reg = )
cmp	ax,-6[bp]
jbe 	.322
.323:
! 2494         base23_16++;
! Debug: postinc unsigned char base23_16 = [S+$14-9] (used reg = )
mov	al,-7[bp]
inc	ax
mov	-7[bp],al
!BCC_EOS
! 2495       write_word(ES, regs.u.r16.si+0x08+0, 47);
.322:
! Debug: list int = const $2F (used reg = )
mov	ax,*$2F
push	ax
! Debug: add int = const 8 to unsigned short regs = [S+$16+4] (used reg = )
mov	ax,6[bp]
! Debug: add int = const 0 to unsigned int = ax+8 (used reg = )
! Debug: list unsigned int = ax+8 (used reg = )
add	ax,*8
push	ax
! Debug: list unsigned short ES = [S+$18+$12] (used reg = )
push	$14[bp]
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
add	sp,*6
!BCC_EOS
! 2496       write_word(ES, regs.u.r16.si+0x08+2, base15_00);
! Debug: list unsigned short base15_00 = [S+$14-8] (used reg = )
push	-6[bp]
! Debug: add int = const 8 to unsigned short regs = [S+$16+4] (used reg = )
mov	ax,6[bp]
! Debug: add int = const 2 to unsigned int = ax+8 (used reg = )
! Debug: list unsigned int = ax+$A (used reg = )
add	ax,*$A
push	ax
! Debug: list unsigned short ES = [S+$18+$12] (used reg = )
push	$14[bp]
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
add	sp,*6
!BCC_EOS
! 2497       write_byte(ES, regs.u.r16.si+0x08+4, base23_16);
! Debug: list unsigned char base23_16 = [S+$14-9] (used reg = )
mov	al,-7[bp]
xor	ah,ah
push	ax
! Debug: add int = const 8 to unsigned short regs = [S+$16+4] (used reg = )
mov	ax,6[bp]
! Debug: add int = const 4 to unsigned int = ax+8 (used reg = )
! Debug: list unsigned int = ax+$C (used reg = )
add	ax,*$C
push	ax
! Debug: list unsigned short ES = [S+$18+$12] (used reg = )
push	$14[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 2498       write_byte(ES, regs.u.r16.si+0x08+5, 0x93);
! Debug: list int = const $93 (used reg = )
mov	ax,#$93
push	ax
! Debug: add int = const 8 to unsigned short regs = [S+$16+4] (used reg = )
mov	ax,6[bp]
! Debug: add int = const 5 to unsigned int = ax+8 (used reg = )
! Debug: list unsigned int = ax+$D (used reg = )
add	ax,*$D
push	ax
! Debug: list unsigned short ES = [S+$18+$12] (used reg = )
push	$14[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 2499       write_word(ES, regs.u.r16.si+0x08+6, 0x0000);
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: add int = const 8 to unsigned short regs = [S+$16+4] (used reg = )
mov	ax,6[bp]
! Debug: add int = const 6 to unsigned int = ax+8 (used reg = )
! Debug: list unsigned int = ax+$E (used reg = )
add	ax,*$E
push	ax
! Debug: list unsigned short ES = [S+$18+$12] (used reg = )
push	$14[bp]
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
add	sp,*6
!BCC_EOS
! 2500       write_word(ES, regs.u.r16.si+0x20+0, 0xffff);
! Debug: list unsigned int = const $FFFF (used reg = )
mov	ax,#$FFFF
push	ax
! Debug: add int = const $20 to unsigned short regs = [S+$16+4] (used reg = )
mov	ax,6[bp]
! Debug: add int = const 0 to unsigned int = ax+$20 (used reg = )
! Debug: list unsigned int = ax+$20 (used reg = )
add	ax,*$20
push	ax
! Debug: list unsigned short ES = [S+$18+$12] (used reg = )
push	$14[bp]
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
add	sp,*6
!BCC_EOS
! 2501       write_word(ES, regs.u.r16.si+0x20+2, 0x0000);
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: add int = const $20 to unsigned short regs = [S+$16+4] (used reg = )
mov	ax,6[bp]
! Debug: add int = const 2 to unsigned int = ax+$20 (used reg = )
! Debug: list unsigned int = ax+$22 (used reg = )
add	ax,*$22
push	ax
! Debug: list unsigned short ES = [S+$18+$12] (used reg = )
push	$14[bp]
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
add	sp,*6
!BCC_EOS
! 2502       write_byte(ES, regs.u.r16.si+0x20+4, 0x000f);
! Debug: list int = const $F (used reg = )
mov	ax,*$F
push	ax
! Debug: add int = const $20 to unsigned short regs = [S+$16+4] (used reg = )
mov	ax,6[bp]
! Debug: add int = const 4 to unsigned int = ax+$20 (used reg = )
! Debug: list unsigned int = ax+$24 (used reg = )
add	ax,*$24
push	ax
! Debug: list unsigned short ES = [S+$18+$12] (used reg = )
push	$14[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 2503       write_byte(ES, regs.u.r16.si+0x20+5, 0x9b);
! Debug: list int = const $9B (used reg = )
mov	ax,#$9B
push	ax
! Debug: add int = const $20 to unsigned short regs = [S+$16+4] (used reg = )
mov	ax,6[bp]
! Debug: add int = const 5 to unsigned int = ax+$20 (used reg = )
! Debug: list unsigned int = ax+$25 (used reg = )
add	ax,*$25
push	ax
! Debug: list unsigned short ES = [S+$18+$12] (used reg = )
push	$14[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 2504       write_word(ES, regs.u.r16.si+0x20+6, 0x0000);
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: add int = const $20 to unsigned short regs = [S+$16+4] (used reg = )
mov	ax,6[bp]
! Debug: add int = const 6 to unsigned int = ax+$20 (used reg = )
! Debug: list unsigned int = ax+$26 (used reg = )
add	ax,*$26
push	ax
! Debug: list unsigned short ES = [S+$18+$12] (used reg = )
push	$14[bp]
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
add	sp,*6
!BCC_EOS
! 2505       ss = get_SS();
! Debug: func () unsigned short = get_SS+0 (used reg = )
call	_get_SS
! Debug: eq unsigned short = ax+0 to unsigned short ss = [S+$14-$C] (used reg = )
mov	-$A[bp],ax
!BCC_EOS
! 2506       base15_00 = ss << 4;
! Debug: sl int = const 4 to unsigned short ss = [S+$14-$C] (used reg = )
mov	ax,-$A[bp]
mov	cl,*4
shl	ax,cl
! Debug: eq unsigned int = ax+0 to unsigned short base15_00 = [S+$14-8] (used reg = )
mov	-6[bp],ax
!BCC_EOS
! 2507       base23_16 = ss >> 12;
! Debug: sr int = const $C to unsigned short ss = [S+$14-$C] (used reg = )
mov	ax,-$A[bp]
mov	al,ah
xor	ah,ah
mov	cl,*4
shr	ax,cl
! Debug: eq unsigned int = ax+0 to unsigned char base23_16 = [S+$14-9] (used reg = )
mov	-7[bp],al
!BCC_EOS
! 2508       write_word(ES, regs.u.r16.si+0x28+0, 0xffff);
! Debug: list unsigned int = const $FFFF (used reg = )
mov	ax,#$FFFF
push	ax
! Debug: add int = const $28 to unsigned short regs = [S+$16+4] (used reg = )
mov	ax,6[bp]
! Debug: add int = const 0 to unsigned int = ax+$28 (used reg = )
! Debug: list unsigned int = ax+$28 (used reg = )
add	ax,*$28
push	ax
! Debug: list unsigned short ES = [S+$18+$12] (used reg = )
push	$14[bp]
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
add	sp,*6
!BCC_EOS
! 2509       write_word(ES, regs.u.r16.si+0x28+2, base15_00);
! Debug: list unsigned short base15_00 = [S+$14-8] (used reg = )
push	-6[bp]
! Debug: add int = const $28 to unsigned short regs = [S+$16+4] (used reg = )
mov	ax,6[bp]
! Debug: add int = const 2 to unsigned int = ax+$28 (used reg = )
! Debug: list unsigned int = ax+$2A (used reg = )
add	ax,*$2A
push	ax
! Debug: list unsigned short ES = [S+$18+$12] (used reg = )
push	$14[bp]
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
add	sp,*6
!BCC_EOS
! 2510       write_byte(ES, regs.u.r16.si+0x28+4, base23_16);
! Debug: list unsigned char base23_16 = [S+$14-9] (used reg = )
mov	al,-7[bp]
xor	ah,ah
push	ax
! Debug: add int = const $28 to unsigned short regs = [S+$16+4] (used reg = )
mov	ax,6[bp]
! Debug: add int = const 4 to unsigned int = ax+$28 (used reg = )
! Debug: list unsigned int = ax+$2C (used reg = )
add	ax,*$2C
push	ax
! Debug: list unsigned short ES = [S+$18+$12] (used reg = )
push	$14[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 2511       write_byte(ES, regs.u.r16.si+0x28+5, 0x93);
! Debug: list int = const $93 (used reg = )
mov	ax,#$93
push	ax
! Debug: add int = const $28 to unsigned short regs = [S+$16+4] (used reg = )
mov	ax,6[bp]
! Debug: add int = const 5 to unsigned int = ax+$28 (used reg = )
! Debug: list unsigned int = ax+$2D (used reg = )
add	ax,*$2D
push	ax
! Debug: list unsigned short ES = [S+$18+$12] (used reg = )
push	$14[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 2512       write_word(ES, regs.u.r16.si+0x28+6, 0x0000);
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: add int = const $28 to unsigned short regs = [S+$16+4] (used reg = )
mov	ax,6[bp]
! Debug: add int = const 6 to unsigned int = ax+$28 (used reg = )
! Debug: list unsigned int = ax+$2E (used reg = )
add	ax,*$2E
push	ax
! Debug: list unsigned short ES = [S+$18+$12] (used reg = )
push	$14[bp]
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
add	sp,*6
!BCC_EOS
! 2513       CX = regs.u.r16.cx;
! Debug: eq unsigned short regs = [S+$14+$E] to unsigned short CX = [S+$14-$E] (used reg = )
mov	ax,$10[bp]
mov	-$C[bp],ax
!BCC_EOS
! 2514 #asm
!BCC_EOS
!BCC_ASM
_int15_function.CX	set	6
.int15_function.CX	set	-$C
_int15_function.FLAGS	set	$2A
.int15_function.FLAGS	set	$18
_int15_function.irqDisable	set	1
.int15_function.irqDisable	set	-$11
_int15_function.DS	set	$28
.int15_function.DS	set	$16
_int15_function.DX	set	4
.int15_function.DX	set	-$E
_int15_function.base23_16	set	$B
.int15_function.base23_16	set	-7
_int15_function.bRegister	set	2
.int15_function.bRegister	set	-$10
_int15_function.ES	set	$26
.int15_function.ES	set	$14
_int15_function.ebda_seg	set	$10
.int15_function.ebda_seg	set	-2
_int15_function.base15_00	set	$C
.int15_function.base15_00	set	-6
_int15_function.ss	set	8
.int15_function.ss	set	-$A
_int15_function.regs	set	$16
.int15_function.regs	set	4
_int15_function.prev_a20_enable	set	$E
.int15_function.prev_a20_enable	set	-4
      mov bx, sp
      SEG SS
        mov cx, _int15_function.CX [bx]
      push eax
      xor eax, eax
      mov ds, ax
      mov 0x0469, ss
      mov 0x0467, sp
      SEG ES
        lgdt [si + 0x08]
      SEG CS
        lidt [pmode_IDT_info]
      ;; perhaps do something with IDT here
      ;; set PE bit in CR0
      mov eax, cr0
      or al, #0x01
      mov cr0, eax
      ;; far jump to flush CPU queue after transition to protected mode
      JMP_AP(0x0020, protected_mode)
protected_mode:
      ;; GDT points to valid descriptor table, now load SS, DS, ES
      mov ax, #0x28 ;; 101 000 = 5th descriptor in table, TI=GDT, RPL=00
      mov ss, ax
      mov ax, #0x10 ;; 010 000 = 2nd descriptor in table, TI=GDT, RPL=00
      mov ds, ax
      mov ax, #0x18 ;; 011 000 = 3rd descriptor in table, TI=GDT, RPL=00
      mov es, ax
      xor si, si
      xor di, di
      cld
      rep
        movsw ;; move CX words from DS:SI to ES:DI
      ;; make sure DS and ES limits are 64KB
      mov ax, #0x28
      mov ds, ax
      mov es, ax
      ;; reset PG bit in CR0 ???
      mov eax, cr0
      and al, #0xFE
      mov cr0, eax
      ;; far jump to flush CPU queue after transition to real mode
      JMP_AP(0xf000, real_mode)
real_mode:
      ;; restore IDT to normal real-mode defaults
      SEG CS
        lidt [rmode_IDT_info]
      xor ax, ax
      mov ds, ax
      mov ss, 0x0469
      mov sp, 0x0467
      pop eax
! 2566 endasm
!BCC_ENDASM
!BCC_EOS
! 2567       set_enable_a20(prev_a20_enable);
! Debug: list unsigned short prev_a20_enable = [S+$14-6] (used reg = )
push	-4[bp]
! Debug: func () unsigned short = set_enable_a20+0 (used reg = )
call	_set_enable_a20
inc	sp
inc	sp
!BCC_EOS
! 2568 #asm
!BCC_EOS
!BCC_ASM
_int15_function.CX	set	6
.int15_function.CX	set	-$C
_int15_function.FLAGS	set	$2A
.int15_function.FLAGS	set	$18
_int15_function.irqDisable	set	1
.int15_function.irqDisable	set	-$11
_int15_function.DS	set	$28
.int15_function.DS	set	$16
_int15_function.DX	set	4
.int15_function.DX	set	-$E
_int15_function.base23_16	set	$B
.int15_function.base23_16	set	-7
_int15_function.bRegister	set	2
.int15_function.bRegister	set	-$10
_int15_function.ES	set	$26
.int15_function.ES	set	$14
_int15_function.ebda_seg	set	$10
.int15_function.ebda_seg	set	-2
_int15_function.base15_00	set	$C
.int15_function.base15_00	set	-6
_int15_function.ss	set	8
.int15_function.ss	set	-$A
_int15_function.regs	set	$16
.int15_function.regs	set	4
_int15_function.prev_a20_enable	set	$E
.int15_function.prev_a20_enable	set	-4
  sti
! 2570 endasm
!BCC_ENDASM
!BCC_EOS
! 2571       regs.u.r8.ah = 0;
! Debug: eq int = const 0 to unsigned char regs = [S+$14+$11] (used reg = )
xor	al,al
mov	$13[bp],al
!BCC_EOS
! 2572       FLAGS &= 0xfffe;
! Debug: andab unsigned int = const $FFFE to unsigned short FLAGS = [S+$14+$16] (used reg = )
mov	ax,$18[bp]
and	al,#$FE
mov	$18[bp],ax
!BCC_EOS
! 2573       break;
br 	.307
!BCC_EOS
! 2574     case 0x88:
! 2575       regs.u.r8.al = inb_cmos(0x30);
.324:
! Debug: list int = const $30 (used reg = )
mov	ax,*$30
push	ax
! Debug: func () unsigned char = inb_cmos+0 (used reg = )
call	_inb_cmos
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char regs = [S+$14+$10] (used reg = )
mov	$12[bp],al
!BCC_EOS
! 2576       regs.u.r8.ah = inb_cmos(0x31);
! Debug: list int = const $31 (used reg = )
mov	ax,*$31
push	ax
! Debug: func () unsigned char = inb_cmos+0 (used reg = )
call	_inb_cmos
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char regs = [S+$14+$11] (used reg = )
mov	$13[bp],al
!BCC_EOS
! 2577       if(regs.u.r16.ax > 0x3c00)
! Debug: gt int = const $3C00 to unsigned short regs = [S+$14+$10] (used reg = )
mov	ax,$12[bp]
cmp	ax,#$3C00
jbe 	.325
.326:
! 2578         regs.u.r16.ax = 0x3c00;
! Debug: eq int = const $3C00 to unsigned short regs = [S+$14+$10] (used reg = )
mov	ax,#$3C00
mov	$12[bp],ax
!BCC_EOS
! 2579       FLAGS &= 0xfffe;
.325:
! Debug: andab unsigned int = const $FFFE to unsigned short FLAGS = [S+$14+$16] (used reg = )
mov	ax,$18[bp]
and	al,#$FE
mov	$18[bp],ax
!BCC_EOS
! 2580       break;
br 	.307
!BCC_EOS
! 2581     case 0x90:
! 2582       break;
.327:
br 	.307
!BCC_EOS
! 2583     case 0x91:
! 2584       break;
.328:
br 	.307
!BCC_EOS
! 2585     case 0xbf:
! 2586       bios_printf(4, "*** int 15h function AH=bf not yet supported!\n");
.329:
! Debug: list * char = .32A+0 (used reg = )
mov	bx,#.32A
push	bx
! Debug: list int = const 4 (used reg = )
mov	ax,*4
push	ax
! Debug: func () void = bios_printf+0 (used reg = )
call	_bios_printf
add	sp,*4
!BCC_EOS
! 2587       FLAGS |= 0x0001;
! Debug: orab int = const 1 to unsigned short FLAGS = [S+$14+$16] (used reg = )
mov	ax,$18[bp]
or	al,*1
mov	$18[bp],ax
!BCC_EOS
! 2588       regs.u.r8.ah = 0x86;
! Debug: eq int = const $86 to unsigned char regs = [S+$14+$11] (used reg = )
mov	al,#$86
mov	$13[bp],al
!BCC_EOS
! 2589       break;
br 	.307
!BCC_EOS
! 2590     case 0xC0:
! 2591       FLAGS &= 0xfffe;
.32B:
! Debug: andab unsigned int = const $FFFE to unsigned short FLAGS = [S+$14+$16] (used reg = )
mov	ax,$18[bp]
and	al,#$FE
mov	$18[bp],ax
!BCC_EOS
! 2592       regs.u.r8.ah = 0;
! Debug: eq int = const 0 to unsigned char regs = [S+$14+$11] (used reg = )
xor	al,al
mov	$13[bp],al
!BCC_EOS
! 2593       regs.u.r16.bx = 0xe6f5;
! Debug: eq unsigned int = const $E6F5 to unsigned short regs = [S+$14+$A] (used reg = )
mov	ax,#$E6F5
mov	$C[bp],ax
!BCC_EOS
! 2594       ES = 0xF000;
! Debug: eq unsigned int = const $F000 to unsigned short ES = [S+$14+$12] (used reg = )
mov	ax,#$F000
mov	$14[bp],ax
!BCC_EOS
! 2595       break;
br 	.307
!BCC_EOS
! 2596     case 0xc1:
! 2597       ES = ebda_seg;
.32C:
! Debug: eq unsigned short ebda_seg = [S+$14-4] to unsigned short ES = [S+$14+$12] (used reg = )
mov	ax,-2[bp]
mov	$14[bp],ax
!BCC_EOS
! 2598       FLAGS &= 0xfffe;
! Debug: andab unsigned int = const $FFFE to unsigned short FLAGS = [S+$14+$16] (used reg = )
mov	ax,$18[bp]
and	al,#$FE
mov	$18[bp],ax
!BCC_EOS
! 2599       break;
br 	.307
!BCC_EOS
! 2600     case 0xd8:
! 2601       bios_printf(8, "EISA BIOS not present\n");
.32D:
! Debug: list * char = .32E+0 (used reg = )
mov	bx,#.32E
push	bx
! Debug: list int = const 8 (used reg = )
mov	ax,*8
push	ax
! Debug: func () void = bios_printf+0 (used reg = )
call	_bios_printf
add	sp,*4
!BCC_EOS
! 2602       FLAGS |= 0x0001;
! Debug: orab int = const 1 to unsigned short FLAGS = [S+$14+$16] (used reg = )
mov	ax,$18[bp]
or	al,*1
mov	$18[bp],ax
!BCC_EOS
! 2603       regs.u.r8.ah = 0x86;
! Debug: eq int = const $86 to unsigned char regs = [S+$14+$11] (used reg = )
mov	al,#$86
mov	$13[bp],al
!BCC_EOS
! 2604       break;
br 	.307
!BCC_EOS
! 2605     default:
! 2606       bios_printf(4, "*** int 15h function AX=%04x, BX=%04x not yet supported!\n", (unsigned) regs.u.r16.ax, (unsigned) regs.u.r16.bx);
.32F:
! Debug: list unsigned short regs = [S+$14+$A] (used reg = )
push	$C[bp]
! Debug: list unsigned short regs = [S+$16+$10] (used reg = )
push	$12[bp]
! Debug: list * char = .330+0 (used reg = )
mov	bx,#.330
push	bx
! Debug: list int = const 4 (used reg = )
mov	ax,*4
push	ax
! Debug: func () void = bios_printf+0 (used reg = )
call	_bios_printf
add	sp,*8
!BCC_EOS
! 2607       FLAGS |= 0x0001;
! Debug: orab int = const 1 to unsigned short FLAGS = [S+$14+$16] (used reg = )
mov	ax,$18[bp]
or	al,*1
mov	$18[bp],ax
!BCC_EOS
! 2608       regs.u.r8.ah = 0x86;
! Debug: eq int = const $86 to unsigned char regs = [S+$14+$11] (used reg = )
mov	al,#$86
mov	$13[bp],al
!BCC_EOS
! 2609       break;
jmp .307
!BCC_EOS
! 2610     }
! 2611 }
jmp .307
.309:
sub	al,*$24
beq 	.30A
sub	al,*$1D
beq 	.314
sub	al,*$E
beq 	.315
sub	al,*3
beq 	.316
sub	al,*$31
beq 	.317
sub	al,*4
beq 	.321
sub	al,*1
beq 	.324
sub	al,*8
beq 	.327
sub	al,*1
beq 	.328
sub	al,*$2E
beq 	.329
sub	al,*1
beq 	.32B
sub	al,*1
beq 	.32C
sub	al,*$17
beq 	.32D
br 	.32F
.307:
..FFF6	=	-$14
mov	sp,bp
pop	bp
ret
! 2612   void
! Register BX used in function int15_function
! 2613 int15_function_mouse(regs, ES, DS, FLAGS)
! 2614   pusha_regs_t regs;
export	_int15_function_mouse
_int15_function_mouse:
!BCC_EOS
! 2615   Bit16u ES, DS, FLAGS;
!BCC_EOS
! 2616 {
! 2617   Bit16u ebda_seg=read_word(0x0040,0x000E);
push	bp
mov	bp,sp
dec	sp
dec	sp
! Debug: list int = const $E (used reg = )
mov	ax,*$E
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () unsigned short = read_word+0 (used reg = )
call	_read_word
add	sp,*4
! Debug: eq unsigned short = ax+0 to unsigned short ebda_seg = [S+4-4] (used reg = )
mov	-2[bp],ax
!BCC_EOS
! 2618   Bit8u mou
! 2618 se_flags_1, mouse_flags_2;
!BCC_EOS
! 2619   Bit16u mouse_driver_seg;
!BCC_EOS
! 2620   Bit16u mouse_driver_offset;
!BCC_EOS
! 2621   Bit8u comm_byte, prev_command_byte;
!BCC_EOS
! 2622   Bit8u ret, mouse_data1, mouse_data2, mouse_data3;
!BCC_EOS
! 2623 ;
add	sp,*-$C
!BCC_EOS
! 2624   switch (regs.u.r8.ah) {
mov	al,$13[bp]
br 	.333
! 2625     case 0xC2:
! 2626       switch (regs.u.r8.al) {
.334:
mov	al,$12[bp]
br 	.337
! 2627         case 0:
! 2628 ;
.338:
!BCC_EOS
! 2629           switch (regs.u.r8.bh) {
mov	al,$D[bp]
br 	.33B
! 2630             case 0:
! 2631 ;
.33C:
!BCC_EOS
! 2632               inhibit_mouse_int_and_events();
! Debug: func () unsigned char = inhibit_mouse_int_and_events+0 (used reg = )
call	_inhibit_mouse_int_and_events
!BCC_EOS
! 2633               ret = send_to_mouse_ctrl(0xF5);
! Debug: list int = const $F5 (used reg = )
mov	ax,#$F5
push	ax
! Debug: func () unsigned char = send_to_mouse_ctrl+0 (used reg = )
call	_send_to_mouse_ctrl
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char ret = [S+$10-$D] (used reg = )
mov	-$B[bp],al
!BCC_EOS
! 2634               if (ret == 0) {
! Debug: logeq int = const 0 to unsigned char ret = [S+$10-$D] (used reg = )
mov	al,-$B[bp]
test	al,al
jne 	.33D
.33E:
! 2635                 ret = get_mouse_data(&mouse_data1);
! Debug: list * unsigned char mouse_data1 = S+$10-$E (used reg = )
lea	bx,-$C[bp]
push	bx
! Debug: func () unsigned char = get_mouse_data+0 (used reg = )
call	_get_mouse_data
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char ret = [S+$10-$D] (used reg = )
mov	-$B[bp],al
!BCC_EOS
! 2636                 if ( (ret == 0) || (mouse_data1 == 0xFA) ) {
! Debug: logeq int = const 0 to unsigned char ret = [S+$10-$D] (used reg = )
mov	al,-$B[bp]
test	al,al
je  	.340
.341:
! Debug: logeq int = const $FA to unsigned char mouse_data1 = [S+$10-$E] (used reg = )
mov	al,-$C[bp]
cmp	al,#$FA
jne 	.33F
.340:
! 2637                   FLAGS &= 0xfffe;
! Debug: andab unsigned int = const $FFFE to unsigned short FLAGS = [S+$10+$16] (used reg = )
mov	ax,$18[bp]
and	al,#$FE
mov	$18[bp],ax
!BCC_EOS
! 2638                   regs.u.r8.ah = 0;
! Debug: eq int = const 0 to unsigned char regs = [S+$10+$11] (used reg = )
xor	al,al
mov	$13[bp],al
!BCC_EOS
! 2639                   return;
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 2640                   }
! 2641                 }
.33F:
! 2642               FLAGS |= 0x0001;
.33D:
! Debug: orab int = const 1 to unsigned short FLAGS = [S+$10+$16] (used reg = )
mov	ax,$18[bp]
or	al,*1
mov	$18[bp],ax
!BCC_EOS
! 2643               regs.u.r8.ah = ret;
! Debug: eq unsigned char ret = [S+$10-$D] to unsigned char regs = [S+$10+$11] (used reg = )
mov	al,-$B[bp]
mov	$13[bp],al
!BCC_EOS
! 2644               return;
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 2645               break;
br 	.339
!BCC_EOS
! 2646             case 1:
! 2647 ;
.342:
!BCC_EOS
! 2648               mouse_flags_2 = read_byte(ebda_seg, 0x0027);
! Debug: list int = const $27 (used reg = )
mov	ax,*$27
push	ax
! Debug: list unsigned short ebda_seg = [S+$12-4] (used reg = )
push	-2[bp]
! Debug: func () unsigned char = read_byte+0 (used reg = )
call	_read_byte
add	sp,*4
! Debug: eq unsigned char = al+0 to unsigned char mouse_flags_2 = [S+$10-6] (used reg = )
mov	-4[bp],al
!BCC_EOS
! 2649               if ( (mouse_flags_2 & 0x80) == 0 ) {
! Debug: and int = const $80 to unsigned char mouse_flags_2 = [S+$10-6] (used reg = )
mov	al,-4[bp]
and	al,#$80
! Debug: logeq int = const 0 to unsigned char = al+0 (used reg = )
test	al,al
jne 	.343
.344:
! 2650                 ;
!BCC_EOS
! 2651                 FLAGS |= 0x0001;
! Debug: orab int = const 1 to unsigned short FLAGS = [S+$10+$16] (used reg = )
mov	ax,$18[bp]
or	al,*1
mov	$18[bp],ax
!BCC_EOS
! 2652                 regs.u.r8.ah = 5;
! Debug: eq int = const 5 to unsigned char regs = [S+$10+$11] (used reg = )
mov	al,*5
mov	$13[bp],al
!BCC_EOS
! 2653                 return;
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 2654                 }
! 2655               inhibit_mouse_int_and_events();
.343:
! Debug: func () unsigned char = inhibit_mouse_int_and_events+0 (used reg = )
call	_inhibit_mouse_int_and_events
!BCC_EOS
! 2656               ret = send_to_mouse_ctrl(0xF4);
! Debug: list int = const $F4 (used reg = )
mov	ax,#$F4
push	ax
! Debug: func () unsigned char = send_to_mouse_ctrl+0 (used reg = )
call	_send_to_mouse_ctrl
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char ret = [S+$10-$D] (used reg = )
mov	-$B[bp],al
!BCC_EOS
! 2657               if (ret == 0) {
! Debug: logeq int = const 0 to unsigned char ret = [S+$10-$D] (used reg = )
mov	al,-$B[bp]
test	al,al
jne 	.345
.346:
! 2658                 ret = get_mouse_data(&mouse_data1);
! Debug: list * unsigned char mouse_data1 = S+$10-$E (used reg = )
lea	bx,-$C[bp]
push	bx
! Debug: func () unsigned char = get_mouse_data+0 (used reg = )
call	_get_mouse_data
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char ret = [S+$10-$D] (used reg = )
mov	-$B[bp],al
!BCC_EOS
! 2659                 if ( (ret == 0) && (mouse_data1 == 0xFA) ) {
! Debug: logeq int = const 0 to unsigned char ret = [S+$10-$D] (used reg = )
mov	al,-$B[bp]
test	al,al
jne 	.347
.349:
! Debug: logeq int = const $FA to unsigned char mouse_data1 = [S+$10-$E] (used reg = )
mov	al,-$C[bp]
cmp	al,#$FA
jne 	.347
.348:
! 2660                   enable_mouse_int_and_events();
! Debug: func () void = enable_mouse_int_and_events+0 (used reg = )
call	_enable_mouse_int_and_events
!BCC_EOS
! 2661                   FLAGS &= 0xfffe;
! Debug: andab unsigned int = const $FFFE to unsigned short FLAGS = [S+$10+$16] (used reg = )
mov	ax,$18[bp]
and	al,#$FE
mov	$18[bp],ax
!BCC_EOS
! 2662                   regs.u.r8.ah = 0;
! Debug: eq int = const 0 to unsigned char regs = [S+$10+$11] (used reg = )
xor	al,al
mov	$13[bp],al
!BCC_EOS
! 2663                   return;
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 2664                   }
! 2665                 }
.347:
! 2666               FLAGS |= 0x0001;
.345:
! Debug: orab int = const 1 to unsigned short FLAGS = [S+$10+$16] (used reg = )
mov	ax,$18[bp]
or	al,*1
mov	$18[bp],ax
!BCC_EOS
! 2667               regs.u.r8.ah = ret;
! Debug: eq unsigned char ret = [S+$10-$D] to unsigned char regs = [S+$10+$11] (used reg = )
mov	al,-$B[bp]
mov	$13[bp],al
!BCC_EOS
! 2668               return;
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 2669             default:
! 2670               ;
.34A:
!BCC_EOS
! 2671               FLAGS |= 0x0001;
! Debug: orab int = const 1 to unsigned short FLAGS = [S+$10+$16] (used reg = )
mov	ax,$18[bp]
or	al,*1
mov	$18[bp],ax
!BCC_EOS
! 2672               regs.u.r8.ah = 1;
! Debug: eq int = const 1 to unsigned char regs = [S+$10+$11] (used reg = )
mov	al,*1
mov	$13[bp],al
!BCC_EOS
! 2673               return;
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 2674             }
! 2675           break;
jmp .339
.33B:
sub	al,*0
beq 	.33C
sub	al,*1
beq 	.342
jmp	.34A
.339:
br 	.335
!BCC_EOS
! 2676         case 1:
! 2677         case 5:
.34B:
! 2678 ;
.34C:
!BCC_EOS
! 2679           if (regs.u.r8.al == 5) {
! Debug: logeq int = const 5 to unsigned char regs = [S+$10+$10] (used reg = )
mov	al,$12[bp]
cmp	al,*5
jne 	.34D
.34E:
! 2680             if (regs.u.r8.bh != 3) {
! Debug: ne int = const 3 to unsigned char regs = [S+$10+$B] (used reg = )
mov	al,$D[bp]
cmp	al,*3
je  	.34F
.350:
! 2681               FLAGS |= 0x0001;
! Debug: orab int = const 1 to unsigned short FLAGS = [S+$10+$16] (used reg = )
mov	ax,$18[bp]
or	al,*1
mov	$18[bp],ax
!BCC_EOS
! 2682               regs.u.r8.ah = 0x02;
! Debug: eq int = const 2 to unsigned char regs = [S+$10+$11] (used reg = )
mov	al,*2
mov	$13[bp],al
!BCC_EOS
! 2683               return;
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 2684             }
! 2685             mouse_flags_2 = read_byte(ebda_seg, 0x0027);
.34F:
! Debug: list int = const $27 (used reg = )
mov	ax,*$27
push	ax
! Debug: list unsigned short ebda_seg = [S+$12-4] (used reg = )
push	-2[bp]
! Debug: func () unsigned char = read_byte+0 (used reg = )
call	_read_byte
add	sp,*4
! Debug: eq unsigned char = al+0 to unsigned char mouse_flags_2 = [S+$10-6] (used reg = )
mov	-4[bp],al
!BCC_EOS
! 2686             mouse_flags_2 = (mouse_flags_2 & 0x00) | regs.u.r8.bh;
! 2686 
! Debug: and int = const 0 to unsigned char mouse_flags_2 = [S+$10-6] (used reg = )
mov	al,-4[bp]
xor	al,al
! Debug: or unsigned char regs = [S+$10+$B] to unsigned char = al+0 (used reg = )
or	al,$D[bp]
! Debug: eq unsigned char = al+0 to unsigned char mouse_flags_2 = [S+$10-6] (used reg = )
mov	-4[bp],al
!BCC_EOS
! 2687             mouse_flags_1 = 0x00;
! Debug: eq int = const 0 to unsigned char mouse_flags_1 = [S+$10-5] (used reg = )
xor	al,al
mov	-3[bp],al
!BCC_EOS
! 2688             write_byte(ebda_seg, 0x0026, mouse_flags_1);
! Debug: list unsigned char mouse_flags_1 = [S+$10-5] (used reg = )
mov	al,-3[bp]
xor	ah,ah
push	ax
! Debug: list int = const $26 (used reg = )
mov	ax,*$26
push	ax
! Debug: list unsigned short ebda_seg = [S+$14-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 2689             write_byte(ebda_seg, 0x0027, mouse_flags_2);
! Debug: list unsigned char mouse_flags_2 = [S+$10-6] (used reg = )
mov	al,-4[bp]
xor	ah,ah
push	ax
! Debug: list int = const $27 (used reg = )
mov	ax,*$27
push	ax
! Debug: list unsigned short ebda_seg = [S+$14-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 2690           }
! 2691           inhibit_mouse_int_and_events();
.34D:
! Debug: func () unsigned char = inhibit_mouse_int_and_events+0 (used reg = )
call	_inhibit_mouse_int_and_events
!BCC_EOS
! 2692           ret = send_to_mouse_ctrl(0xFF);
! Debug: list int = const $FF (used reg = )
mov	ax,#$FF
push	ax
! Debug: func () unsigned char = send_to_mouse_ctrl+0 (used reg = )
call	_send_to_mouse_ctrl
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char ret = [S+$10-$D] (used reg = )
mov	-$B[bp],al
!BCC_EOS
! 2693           if (ret == 0) {
! Debug: logeq int = const 0 to unsigned char ret = [S+$10-$D] (used reg = )
mov	al,-$B[bp]
test	al,al
bne 	.351
.352:
! 2694             ret = get_mouse_data(&mouse_data3);
! Debug: list * unsigned char mouse_data3 = S+$10-$10 (used reg = )
lea	bx,-$E[bp]
push	bx
! Debug: func () unsigned char = get_mouse_data+0 (used reg = )
call	_get_mouse_data
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char ret = [S+$10-$D] (used reg = )
mov	-$B[bp],al
!BCC_EOS
! 2695             if (mouse_data3 == 0xfe) {
! Debug: logeq int = const $FE to unsigned char mouse_data3 = [S+$10-$10] (used reg = )
mov	al,-$E[bp]
cmp	al,#$FE
jne 	.353
.354:
! 2696               FLAGS |= 0x0001;
! Debug: orab int = const 1 to unsigned short FLAGS = [S+$10+$16] (used reg = )
mov	ax,$18[bp]
or	al,*1
mov	$18[bp],ax
!BCC_EOS
! 2697               return;
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 2698             }
! 2699             if (mouse_data3 != 0xfa)
.353:
! Debug: ne int = const $FA to unsigned char mouse_data3 = [S+$10-$10] (used reg = )
mov	al,-$E[bp]
cmp	al,#$FA
je  	.355
.356:
! 2700               bios_printf((2 | 4 | 1), "Mouse reset returned %02x (should be ack)\n", (unsigned)mouse_data3);
! Debug: list unsigned char mouse_data3 = [S+$10-$10] (used reg = )
mov	al,-$E[bp]
xor	ah,ah
push	ax
! Debug: list * char = .357+0 (used reg = )
mov	bx,#.357
push	bx
! Debug: list int = const 7 (used reg = )
mov	ax,*7
push	ax
! Debug: func () void = bios_printf+0 (used reg = )
call	_bios_printf
add	sp,*6
!BCC_EOS
! 2701             if ( ret == 0 ) {
.355:
! Debug: logeq int = const 0 to unsigned char ret = [S+$10-$D] (used reg = )
mov	al,-$B[bp]
test	al,al
jne 	.358
.359:
! 2702               ret = get_mouse_data(&mouse_data1);
! Debug: list * unsigned char mouse_data1 = S+$10-$E (used reg = )
lea	bx,-$C[bp]
push	bx
! Debug: func () unsigned char = get_mouse_data+0 (used reg = )
call	_get_mouse_data
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char ret = [S+$10-$D] (used reg = )
mov	-$B[bp],al
!BCC_EOS
! 2703               if ( ret == 0 ) {
! Debug: logeq int = const 0 to unsigned char ret = [S+$10-$D] (used reg = )
mov	al,-$B[bp]
test	al,al
jne 	.35A
.35B:
! 2704                 ret = get_mouse_data(&mouse_data2);
! Debug: list * unsigned char mouse_data2 = S+$10-$F (used reg = )
lea	bx,-$D[bp]
push	bx
! Debug: func () unsigned char = get_mouse_data+0 (used reg = )
call	_get_mouse_data
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char ret = [S+$10-$D] (used reg = )
mov	-$B[bp],al
!BCC_EOS
! 2705                 if ( ret == 0 ) {
! Debug: logeq int = const 0 to unsigned char ret = [S+$10-$D] (used reg = )
mov	al,-$B[bp]
test	al,al
jne 	.35C
.35D:
! 2706                   enable_mouse_int_and_events();
! Debug: func () void = enable_mouse_int_and_events+0 (used reg = )
call	_enable_mouse_int_and_events
!BCC_EOS
! 2707                   FLAGS &= 0xfffe;
! Debug: andab unsigned int = const $FFFE to unsigned short FLAGS = [S+$10+$16] (used reg = )
mov	ax,$18[bp]
and	al,#$FE
mov	$18[bp],ax
!BCC_EOS
! 2708                   regs.u.r8.ah = 0;
! Debug: eq int = const 0 to unsigned char regs = [S+$10+$11] (used reg = )
xor	al,al
mov	$13[bp],al
!BCC_EOS
! 2709                   regs.u.r8.bl = mouse_data1;
! Debug: eq unsigned char mouse_data1 = [S+$10-$E] to unsigned char regs = [S+$10+$A] (used reg = )
mov	al,-$C[bp]
mov	$C[bp],al
!BCC_EOS
! 2710                   regs.u.r8.bh = mouse_data2;
! Debug: eq unsigned char mouse_data2 = [S+$10-$F] to unsigned char regs = [S+$10+$B] (used reg = )
mov	al,-$D[bp]
mov	$D[bp],al
!BCC_EOS
! 2711                   return;
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 2712                   }
! 2713                 }
.35C:
! 2714               }
.35A:
! 2715             }
.358:
! 2716           FLAGS |= 0x0001;
.351:
! Debug: orab int = const 1 to unsigned short FLAGS = [S+$10+$16] (used reg = )
mov	ax,$18[bp]
or	al,*1
mov	$18[bp],ax
!BCC_EOS
! 2717           regs.u.r8.ah = ret;
! Debug: eq unsigned char ret = [S+$10-$D] to unsigned char regs = [S+$10+$11] (used reg = )
mov	al,-$B[bp]
mov	$13[bp],al
!BCC_EOS
! 2718           return;
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 2719         case 2:
! 2720 ;
.35E:
!BCC_EOS
! 2721           switch (regs.u.r8.bh) {
mov	al,$D[bp]
jmp .361
! 2722             case 0: mouse_data1 = 10; break;
.362:
! Debug: eq int = const $A to unsigned char mouse_data1 = [S+$10-$E] (used reg = )
mov	al,*$A
mov	-$C[bp],al
!BCC_EOS
jmp .35F
!BCC_EOS
! 2723             case 1: mouse_data1 = 20; break;
.363:
! Debug: eq int = const $14 to unsigned char mouse_data1 = [S+$10-$E] (used reg = )
mov	al,*$14
mov	-$C[bp],al
!BCC_EOS
jmp .35F
!BCC_EOS
! 2724             case 2: mouse_data1 = 40; break;
.364:
! Debug: eq int = const $28 to unsigned char mouse_data1 = [S+$10-$E] (used reg = )
mov	al,*$28
mov	-$C[bp],al
!BCC_EOS
jmp .35F
!BCC_EOS
! 2725             case 3: mouse_data1 = 60; break;
.365:
! Debug: eq int = const $3C to unsigned char mouse_data1 = [S+$10-$E] (used reg = )
mov	al,*$3C
mov	-$C[bp],al
!BCC_EOS
jmp .35F
!BCC_EOS
! 2726             case 4: mouse_data1 = 80; break;
.366:
! Debug: eq int = const $50 to unsigned char mouse_data1 = [S+$10-$E] (used reg = )
mov	al,*$50
mov	-$C[bp],al
!BCC_EOS
jmp .35F
!BCC_EOS
! 2727             case 5: mouse_data1 = 100; break;
.367:
! Debug: eq int = const $64 to unsigned char mouse_data1 = [S+$10-$E] (used reg = )
mov	al,*$64
mov	-$C[bp],al
!BCC_EOS
jmp .35F
!BCC_EOS
! 2728             case 6: mouse_data1 = 200; break;
.368:
! Debug: eq int = const $C8 to unsigned char mouse_data1 = [S+$10-$E] (used reg = )
mov	al,#$C8
mov	-$C[bp],al
!BCC_EOS
jmp .35F
!BCC_EOS
! 2729             default: mouse_data1 = 0;
.369:
! Debug: eq int = const 0 to unsigned char mouse_data1 = [S+$10-$E] (used reg = )
xor	al,al
mov	-$C[bp],al
!BCC_EOS
! 2730           }
! 2731           if (mouse_data1 > 0) {
jmp .35F
.361:
sub	al,*0
jb 	.369
cmp	al,*6
ja  	.36A
xor	ah,ah
shl	ax,*1
mov	bx,ax
seg	cs
br	.36B[bx]
.36B:
.word	.362
.word	.363
.word	.364
.word	.365
.word	.366
.word	.367
.word	.368
.36A:
jmp	.369
.35F:
! Debug: gt int = const 0 to unsigned char mouse_data1 = [S+$10-$E] (used reg = )
mov	al,-$C[bp]
test	al,al
beq 	.36C
.36D:
! 2732             ret = send_to_mouse_ctrl(0xF3);
! Debug: list int = const $F3 (used reg = )
mov	ax,#$F3
push	ax
! Debug: func () unsigned char = send_to_mouse_ctrl+0 (used reg = )
call	_send_to_mouse_ctrl
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char ret = [S+$10-$D] (used reg = )
mov	-$B[bp],al
!BCC_EOS
! 2733             if (ret == 0) {
! Debug: logeq int = const 0 to unsigned char ret = [S+$10-$D] (used reg = )
mov	al,-$B[bp]
test	al,al
jne 	.36E
.36F:
! 2734               ret = get_mouse_data(&mouse_data2);
! Debug: list * unsigned char mouse_data2 = S+$10-$F (used reg = )
lea	bx,-$D[bp]
push	bx
! Debug: func () unsigned char = get_mouse_data+0 (used reg = )
call	_get_mouse_data
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char ret = [S+$10-$D] (used reg = )
mov	-$B[bp],al
!BCC_EOS
! 2735               ret = send_to_mouse_ctrl(mouse_data1);
! Debug: list unsigned char mouse_data1 = [S+$10-$E] (used reg = )
mov	al,-$C[bp]
xor	ah,ah
push	ax
! Debug: func () unsigned char = send_to_mouse_ctrl+0 (used reg = )
call	_send_to_mouse_ctrl
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char ret = [S+$10-$D] (used reg = )
mov	-$B[bp],al
!BCC_EOS
! 2736               ret = get_mouse_data(&mouse_data2);
! Debug: list * unsigned char mouse_data2 = S+$10-$F (used reg = )
lea	bx,-$D[bp]
push	bx
! Debug: func () unsigned char = get_mouse_data+0 (used reg = )
call	_get_mouse_data
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char ret = [S+$10-$D] (used reg = )
mov	-$B[bp],al
!BCC_EOS
! 2737               FLAGS &= 0xfffe;
! Debug: andab unsigned int = const $FFFE to unsigned short FLAGS = [S+$10+$16] (used reg = )
mov	ax,$18[bp]
and	al,#$FE
mov	$18[bp],ax
!BCC_EOS
! 2738               regs.u.r8.ah = 0;
! Debug: eq int = const 0 to unsigned char regs = [S+$10+$11] (used reg = )
xor	al,al
mov	$13[bp],al
!BCC_EOS
! 2739             } else {
jmp .370
.36E:
! 2740               FLAGS |= 0x0001;
! Debug: orab int = const 1 to unsigned short FLAGS = [S+$10+$16] (used reg = )
mov	ax,$18[bp]
or	al,*1
mov	$18[bp],ax
!BCC_EOS
! 2741               regs.u.r8.ah = 0x86;
! Debug: eq int = const $86 to unsigned char regs = [S+$10+$11] (used reg = )
mov	al,#$86
mov	$13[bp],al
!BCC_EOS
! 2742             }
! 2743           } else {
.370:
jmp .371
.36C:
! 2744             FLAGS |= 0x0001;
! Debug: orab int = const 1 to unsigned short FLAGS = [S+$10+$16] (used reg = )
mov	ax,$18[bp]
or	al,*1
mov	$18[bp],ax
!BCC_EOS
! 2745         
! 2745     regs.u.r8.ah = 0x86;
! Debug: eq int = const $86 to unsigned char regs = [S+$10+$11] (used reg = )
mov	al,#$86
mov	$13[bp],al
!BCC_EOS
! 2746           }
! 2747           break;
.371:
br 	.335
!BCC_EOS
! 2748         case 3:
! 2749 ;
.372:
!BCC_EOS
! 2750           FLAGS &= 0xfffe;
! Debug: andab unsigned int = const $FFFE to unsigned short FLAGS = [S+$10+$16] (used reg = )
mov	ax,$18[bp]
and	al,#$FE
mov	$18[bp],ax
!BCC_EOS
! 2751           regs.u.r8.ah = 0;
! Debug: eq int = const 0 to unsigned char regs = [S+$10+$11] (used reg = )
xor	al,al
mov	$13[bp],al
!BCC_EOS
! 2752           break;
br 	.335
!BCC_EOS
! 2753         case 4:
! 2754 ;
.373:
!BCC_EOS
! 2755           inhibit_mouse_int_and_events();
! Debug: func () unsigned char = inhibit_mouse_int_and_events+0 (used reg = )
call	_inhibit_mouse_int_and_events
!BCC_EOS
! 2756           ret = send_to_mouse_ctrl(0xF2);
! Debug: list int = const $F2 (used reg = )
mov	ax,#$F2
push	ax
! Debug: func () unsigned char = send_to_mouse_ctrl+0 (used reg = )
call	_send_to_mouse_ctrl
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char ret = [S+$10-$D] (used reg = )
mov	-$B[bp],al
!BCC_EOS
! 2757           if (ret == 0) {
! Debug: logeq int = const 0 to unsigned char ret = [S+$10-$D] (used reg = )
mov	al,-$B[bp]
test	al,al
jne 	.374
.375:
! 2758             ret = get_mouse_data(&mouse_data1);
! Debug: list * unsigned char mouse_data1 = S+$10-$E (used reg = )
lea	bx,-$C[bp]
push	bx
! Debug: func () unsigned char = get_mouse_data+0 (used reg = )
call	_get_mouse_data
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char ret = [S+$10-$D] (used reg = )
mov	-$B[bp],al
!BCC_EOS
! 2759             ret = get_mouse_data(&mouse_data2);
! Debug: list * unsigned char mouse_data2 = S+$10-$F (used reg = )
lea	bx,-$D[bp]
push	bx
! Debug: func () unsigned char = get_mouse_data+0 (used reg = )
call	_get_mouse_data
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char ret = [S+$10-$D] (used reg = )
mov	-$B[bp],al
!BCC_EOS
! 2760             FLAGS &= 0xfffe;
! Debug: andab unsigned int = const $FFFE to unsigned short FLAGS = [S+$10+$16] (used reg = )
mov	ax,$18[bp]
and	al,#$FE
mov	$18[bp],ax
!BCC_EOS
! 2761             regs.u.r8.ah = 0;
! Debug: eq int = const 0 to unsigned char regs = [S+$10+$11] (used reg = )
xor	al,al
mov	$13[bp],al
!BCC_EOS
! 2762             regs.u.r8.bh = mouse_data2;
! Debug: eq unsigned char mouse_data2 = [S+$10-$F] to unsigned char regs = [S+$10+$B] (used reg = )
mov	al,-$D[bp]
mov	$D[bp],al
!BCC_EOS
! 2763           } else {
jmp .376
.374:
! 2764             FLAGS |= 0x0001;
! Debug: orab int = const 1 to unsigned short FLAGS = [S+$10+$16] (used reg = )
mov	ax,$18[bp]
or	al,*1
mov	$18[bp],ax
!BCC_EOS
! 2765             regs.u.r8.ah = 0x86;
! Debug: eq int = const $86 to unsigned char regs = [S+$10+$11] (used reg = )
mov	al,#$86
mov	$13[bp],al
!BCC_EOS
! 2766           }
! 2767           break;
.376:
br 	.335
!BCC_EOS
! 2768         case 6:
! 2769 ;
.377:
!BCC_EOS
! 2770           switch (regs.u.r8.bh) {
mov	al,$D[bp]
br 	.37A
! 2771             case 0:
! 2772               comm_byte = inhibit_mouse_int_and_events();
.37B:
! Debug: func () unsigned char = inhibit_mouse_int_and_events+0 (used reg = )
call	_inhibit_mouse_int_and_events
! Debug: eq unsigned char = al+0 to unsigned char comm_byte = [S+$10-$B] (used reg = )
mov	-9[bp],al
!BCC_EOS
! 2773               ret = send_to_mouse_ctrl(0xE9);
! Debug: list int = const $E9 (used reg = )
mov	ax,#$E9
push	ax
! Debug: func () unsigned char = send_to_mouse_ctrl+0 (used reg = )
call	_send_to_mouse_ctrl
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char ret = [S+$10-$D] (used reg = )
mov	-$B[bp],al
!BCC_EOS
! 2774               if (ret == 0) {
! Debug: logeq int = const 0 to unsigned char ret = [S+$10-$D] (used reg = )
mov	al,-$B[bp]
test	al,al
bne 	.37C
.37D:
! 2775                 ret = get_mouse_data(&mouse_data1);
! Debug: list * unsigned char mouse_data1 = S+$10-$E (used reg = )
lea	bx,-$C[bp]
push	bx
! Debug: func () unsigned char = get_mouse_data+0 (used reg = )
call	_get_mouse_data
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char ret = [S+$10-$D] (used reg = )
mov	-$B[bp],al
!BCC_EOS
! 2776                 if (mouse_data1 != 0xfa)
! Debug: ne int = const $FA to unsigned char mouse_data1 = [S+$10-$E] (used reg = )
mov	al,-$C[bp]
cmp	al,#$FA
je  	.37E
.37F:
! 2777                   bios_printf((2 | 4 | 1), "Mouse status returned %02x (should be ack)\n", (unsigned)mouse_data1);
! Debug: list unsigned char mouse_data1 = [S+$10-$E] (used reg = )
mov	al,-$C[bp]
xor	ah,ah
push	ax
! Debug: list * char = .380+0 (used reg = )
mov	bx,#.380
push	bx
! Debug: list int = const 7 (used reg = )
mov	ax,*7
push	ax
! Debug: func () void = bios_printf+0 (used reg = )
call	_bios_printf
add	sp,*6
!BCC_EOS
! 2778                 if (ret == 0) {
.37E:
! Debug: logeq int = const 0 to unsigned char ret = [S+$10-$D] (used reg = )
mov	al,-$B[bp]
test	al,al
bne 	.381
.382:
! 2779                   ret = get_mouse_data(&mouse_data1);
! Debug: list * unsigned char mouse_data1 = S+$10-$E (used reg = )
lea	bx,-$C[bp]
push	bx
! Debug: func () unsigned char = get_mouse_data+0 (used reg = )
call	_get_mouse_data
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char ret = [S+$10-$D] (used reg = )
mov	-$B[bp],al
!BCC_EOS
! 2780                   if ( ret == 0 ) {
! Debug: logeq int = const 0 to unsigned char ret = [S+$10-$D] (used reg = )
mov	al,-$B[bp]
test	al,al
jne 	.383
.384:
! 2781                     ret = get_mouse_data(&mouse_data2);
! Debug: list * unsigned char mouse_data2 = S+$10-$F (used reg = )
lea	bx,-$D[bp]
push	bx
! Debug: func () unsigned char = get_mouse_data+0 (used reg = )
call	_get_mouse_data
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char ret = [S+$10-$D] (used reg = )
mov	-$B[bp],al
!BCC_EOS
! 2782                     if ( ret == 0 ) {
! Debug: logeq int = const 0 to unsigned char ret = [S+$10-$D] (used reg = )
mov	al,-$B[bp]
test	al,al
jne 	.385
.386:
! 2783                       ret = get_mouse_data(&mouse_data3);
! Debug: list * unsigned char mouse_data3 = S+$10-$10 (used reg = )
lea	bx,-$E[bp]
push	bx
! Debug: func () unsigned char = get_mouse_data+0 (used reg = )
call	_get_mouse_data
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char ret = [S+$10-$D] (used reg = )
mov	-$B[bp],al
!BCC_EOS
! 2784                       if ( ret == 0 ) {
! Debug: logeq int = const 0 to unsigned char ret = [S+$10-$D] (used reg = )
mov	al,-$B[bp]
test	al,al
jne 	.387
.388:
! 2785                         FLAGS &= 0xfffe;
! Debug: andab unsigned int = const $FFFE to unsigned short FLAGS = [S+$10+$16] (used reg = )
mov	ax,$18[bp]
and	al,#$FE
mov	$18[bp],ax
!BCC_EOS
! 2786                         regs.u.r8.ah = 0;
! Debug: eq int = const 0 to unsigned char regs = [S+$10+$11] (used reg = )
xor	al,al
mov	$13[bp],al
!BCC_EOS
! 2787                         regs.u.r8.bl = mouse_data1;
! Debug: eq unsigned char mouse_data1 = [S+$10-$E] to unsigned char regs = [S+$10+$A] (used reg = )
mov	al,-$C[bp]
mov	$C[bp],al
!BCC_EOS
! 2788                         regs.u.r8.cl = mouse_data2;
! Debug: eq unsigned char mouse_data2 = [S+$10-$F] to unsigned char regs = [S+$10+$E] (used reg = )
mov	al,-$D[bp]
mov	$10[bp],al
!BCC_EOS
! 2789                         regs.u.r8.dl = mouse_data3;
! Debug: eq unsigned char mouse_data3 = [S+$10-$10] to unsigned char regs = [S+$10+$C] (used reg = )
mov	al,-$E[bp]
mov	$E[bp],al
!BCC_EOS
! 2790                         set_kbd_command_byte(comm_byte);
! Debug: list unsigned char comm_byte = [S+$10-$B] (used reg = )
mov	al,-9[bp]
xor	ah,ah
push	ax
! Debug: func () void = set_kbd_command_byte+0 (used reg = )
call	_set_kbd_command_byte
inc	sp
inc	sp
!BCC_EOS
! 2791                         return;
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 2792                         }
! 2793                       }
.387:
! 2794                     }
.385:
! 2795                   }
.383:
! 2796                 }
.381:
! 2797               FLAGS |= 0x0001;
.37C:
! Debug: orab int = const 1 to unsigned short FLAGS = [S+$10+$16] (used reg = )
mov	ax,$18[bp]
or	al,*1
mov	$18[bp],ax
!BCC_EOS
! 2798               regs.u.r8.ah = ret;
! Debug: eq unsigned char ret = [S+$10-$D] to unsigned char regs = [S+$10+$11] (used reg = )
mov	al,-$B[bp]
mov	$13[bp],al
!BCC_EOS
! 2799               set_kbd_command_byte(comm_byte);
! Debug: list unsigned char comm_byte = [S+$10-$B] (used reg = )
mov	al,-9[bp]
xor	ah,ah
push	ax
! Debug: func () void = set_kbd_command_byte+0 (used reg = )
call	_set_kbd_command_byte
inc	sp
inc	sp
!BCC_EOS
! 2800               return;
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 2801             case 1:
! 2802             case 2:
.389:
! 2803               comm_byte = inhibit_mouse_int_and_events();
.38A:
! Debug: func () unsigned char = inhibit_mouse_int_and_events+0 (used reg = )
call	_inhibit_mouse_int_and_events
! Debug: eq unsigned char = al+0 to unsigned char comm_byte = [S+$10-$B] (used reg = )
mov	-9[bp],al
!BCC_EOS
! 2804               if (regs.u.r8.bh == 1) {
! Debug: logeq int = const 1 to unsigned char regs = [S+$10+$B] (used reg = )
mov	al,$D[bp]
cmp	al,*1
jne 	.38B
.38C:
! 2805                 ret = send_to_mous
! 2805 e_ctrl(0xE6);
! Debug: list int = const $E6 (used reg = )
mov	ax,#$E6
push	ax
! Debug: func () unsigned char = send_to_mouse_ctrl+0 (used reg = )
call	_send_to_mouse_ctrl
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char ret = [S+$10-$D] (used reg = )
mov	-$B[bp],al
!BCC_EOS
! 2806               } else {
jmp .38D
.38B:
! 2807                 ret = send_to_mouse_ctrl(0xE7);
! Debug: list int = const $E7 (used reg = )
mov	ax,#$E7
push	ax
! Debug: func () unsigned char = send_to_mouse_ctrl+0 (used reg = )
call	_send_to_mouse_ctrl
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char ret = [S+$10-$D] (used reg = )
mov	-$B[bp],al
!BCC_EOS
! 2808               }
! 2809               if (ret == 0) {
.38D:
! Debug: logeq int = const 0 to unsigned char ret = [S+$10-$D] (used reg = )
mov	al,-$B[bp]
test	al,al
jne 	.38E
.38F:
! 2810                 get_mouse_data(&mouse_data1);
! Debug: list * unsigned char mouse_data1 = S+$10-$E (used reg = )
lea	bx,-$C[bp]
push	bx
! Debug: func () unsigned char = get_mouse_data+0 (used reg = )
call	_get_mouse_data
inc	sp
inc	sp
!BCC_EOS
! 2811                 ret = (mouse_data1 != 0xFA);
! Debug: ne int = const $FA to unsigned char mouse_data1 = [S+$10-$E] (used reg = )
mov	al,-$C[bp]
cmp	al,#$FA
je 	.390
mov	al,*1
jmp	.391
.390:
xor	al,al
.391:
! Debug: eq char = al+0 to unsigned char ret = [S+$10-$D] (used reg = )
mov	-$B[bp],al
!BCC_EOS
! 2812               }
! 2813               if (ret == 0) {
.38E:
! Debug: logeq int = const 0 to unsigned char ret = [S+$10-$D] (used reg = )
mov	al,-$B[bp]
test	al,al
jne 	.392
.393:
! 2814                 FLAGS &= 0xfffe;
! Debug: andab unsigned int = const $FFFE to unsigned short FLAGS = [S+$10+$16] (used reg = )
mov	ax,$18[bp]
and	al,#$FE
mov	$18[bp],ax
!BCC_EOS
! 2815                 regs.u.r8.ah = 0;
! Debug: eq int = const 0 to unsigned char regs = [S+$10+$11] (used reg = )
xor	al,al
mov	$13[bp],al
!BCC_EOS
! 2816               } else {
jmp .394
.392:
! 2817                 FLAGS |= 0x0001;
! Debug: orab int = const 1 to unsigned short FLAGS = [S+$10+$16] (used reg = )
mov	ax,$18[bp]
or	al,*1
mov	$18[bp],ax
!BCC_EOS
! 2818                 regs.u.r8.ah = 0x86;
! Debug: eq int = const $86 to unsigned char regs = [S+$10+$11] (used reg = )
mov	al,#$86
mov	$13[bp],al
!BCC_EOS
! 2819               }
! 2820               set_kbd_command_byte(comm_byte);
.394:
! Debug: list unsigned char comm_byte = [S+$10-$B] (used reg = )
mov	al,-9[bp]
xor	ah,ah
push	ax
! Debug: func () void = set_kbd_command_byte+0 (used reg = )
call	_set_kbd_command_byte
inc	sp
inc	sp
!BCC_EOS
! 2821               break;
jmp .378
!BCC_EOS
! 2822             default:
! 2823               bios_printf((2 | 4 | 1), "INT 15h C2 AL=6, BH=%02x\n", (unsigned) regs.u.r8.bh);
.395:
! Debug: list unsigned char regs = [S+$10+$B] (used reg = )
mov	al,$D[bp]
xor	ah,ah
push	ax
! Debug: list * char = .396+0 (used reg = )
mov	bx,#.396
push	bx
! Debug: list int = const 7 (used reg = )
mov	ax,*7
push	ax
! Debug: func () void = bios_printf+0 (used reg = )
call	_bios_printf
add	sp,*6
!BCC_EOS
! 2824             }
! 2825           break;
jmp .378
.37A:
sub	al,*0
beq 	.37B
sub	al,*1
beq 	.389
sub	al,*1
beq 	.38A
jmp	.395
.378:
br 	.335
!BCC_EOS
! 2826         case 7:
! 2827 ;
.397:
!BCC_EOS
! 2828           mouse_driver_seg = ES;
! Debug: eq unsigned short ES = [S+$10+$12] to unsigned short mouse_driver_seg = [S+$10-8] (used reg = )
mov	ax,$14[bp]
mov	-6[bp],ax
!BCC_EOS
! 2829           mouse_driver_offset = regs.u.r16.bx;
! Debug: eq unsigned short regs = [S+$10+$A] to unsigned short mouse_driver_offset = [S+$10-$A] (used reg = )
mov	ax,$C[bp]
mov	-8[bp],ax
!BCC_EOS
! 2830           write_word(ebda_seg, 0x0022, mouse_driver_offset);
! Debug: list unsigned short mouse_driver_offset = [S+$10-$A] (used reg = )
push	-8[bp]
! Debug: list int = const $22 (used reg = )
mov	ax,*$22
push	ax
! Debug: list unsigned short ebda_seg = [S+$14-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
add	sp,*6
!BCC_EOS
! 2831           write_word(ebda_seg, 0x0024, mouse_driver_seg);
! Debug: list unsigned short mouse_driver_seg = [S+$10-8] (used reg = )
push	-6[bp]
! Debug: list int = const $24 (used reg = )
mov	ax,*$24
push	ax
! Debug: list unsigned short ebda_seg = [S+$14-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
add	sp,*6
!BCC_EOS
! 2832           mouse_flags_2 = read_byte(ebda_seg, 0x0027);
! Debug: list int = const $27 (used reg = )
mov	ax,*$27
push	ax
! Debug: list unsigned short ebda_seg = [S+$12-4] (used reg = )
push	-2[bp]
! Debug: func () unsigned char = read_byte+0 (used reg = )
call	_read_byte
add	sp,*4
! Debug: eq unsigned char = al+0 to unsigned char mouse_flags_2 = [S+$10-6] (used reg = )
mov	-4[bp],al
!BCC_EOS
! 2833           if (mouse_driver_offset == 0 && mouse_driver_seg == 0) {
! Debug: logeq int = const 0 to unsigned short mouse_driver_offset = [S+$10-$A] (used reg = )
mov	ax,-8[bp]
test	ax,ax
jne 	.398
.39A:
! Debug: logeq int = const 0 to unsigned short mouse_driver_seg = [S+$10-8] (used reg = )
mov	ax,-6[bp]
test	ax,ax
jne 	.398
.399:
! 2834             if ( (mouse_flags_2 & 0x80) != 0 ) {
! Debug: and int = const $80 to unsigned char mouse_flags_2 = [S+$10-6] (used reg = )
mov	al,-4[bp]
and	al,#$80
! Debug: ne int = const 0 to unsigned char = al+0 (used reg = )
test	al,al
je  	.39B
.39C:
! 2835               mouse_flags_2 &= ~0x80;
! Debug: andab int = const -$81 to unsigned char mouse_flags_2 = [S+$10-6] (used reg = )
mov	al,-4[bp]
and	al,*$7F
mov	-4[bp],al
!BCC_EOS
! 2836               inhibit_mouse_int_and_events();
! Debug: func () unsigned char = inhibit_mouse_int_and_events+0 (used reg = )
call	_inhibit_mouse_int_and_events
!BCC_EOS
! 2837               }
! 2838             }
.39B:
! 2839           else {
jmp .39D
.398:
! 2840             mouse_flags_2 |= 0x80;
! Debug: orab int = const $80 to unsigned char mouse_flags_2 = [S+$10-6] (used reg = )
mov	al,-4[bp]
or	al,#$80
mov	-4[bp],al
!BCC_EOS
! 2841             }
! 2842           write_byte(ebda_seg, 0x0027, mouse_flags_2);
.39D:
! Debug: list unsigned char mouse_flags_2 = [S+$10-6] (used reg = )
mov	al,-4[bp]
xor	ah,ah
push	ax
! Debug: list int = const $27 (used reg = )
mov	ax,*$27
push	ax
! Debug: list unsigned short ebda_seg = [S+$14-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 2843           FLAGS &= 0xfffe;
! Debug: andab unsigned int = const $FFFE to unsigned short FLAGS = [S+$10+$16] (used reg = )
mov	ax,$18[bp]
and	al,#$FE
mov	$18[bp],ax
!BCC_EOS
! 2844           regs.u.r8.ah = 0;
! Debug: eq int = const 0 to unsigned char regs = [S+$10+$11] (used reg = )
xor	al,al
mov	$13[bp],al
!BCC_EOS
! 2845           break;
jmp .335
!BCC_EOS
! 2846         default:
! 2847 ;
.39E:
!BCC_EOS
! 2848           regs.u.r8.ah = 1;
! Debug: eq int = const 1 to unsigned char regs = [S+$10+$11] (used reg = )
mov	al,*1
mov	$13[bp],al
!BCC_EOS
! 2849           FLAGS |= 0x0001;
! Debug: orab int = const 1 to unsigned short FLAGS = [S+$10+$16] (used reg = )
mov	ax,$18[bp]
or	al,*1
mov	$18[bp],ax
!BCC_EOS
! 2850         }
! 2851       break;
jmp .335
.337:
sub	al,*0
jb 	.39E
cmp	al,*7
ja  	.39F
xor	ah,ah
shl	ax,*1
mov	bx,ax
seg	cs
br	.3A0[bx]
.3A0:
.word	.338
.word	.34B
.word	.35E
.word	.372
.word	.373
.word	.34C
.word	.377
.word	.397
.39F:
jmp	.39E
.335:
jmp .331
!BCC_EOS
! 2852     default:
! 2853       bios_printf(4, "*** int 15h function AX=%04x, BX=%04x not yet supported!\n", (unsigned) regs.u.r16.ax, (unsigned) regs.u.r16.bx);
.3A1:
! Debug: list unsigned short regs = [S+$10+$A] (used reg = )
push	$C[bp]
! Debug: list unsigned short regs = [S+$12+$10] (used reg = )
push	$12[bp]
! Debug: list * char = .3A2+0 (used reg = )
mov	bx,#.3A2
push	bx
! Debug: list int = const 4 (used reg = )
mov	ax,*4
push	ax
! Debug: func () void = bios_printf+0 (used reg = )
call	_bios_printf
add	sp,*8
!BCC_EOS
! 2854       FLAGS |= 0x0001;
! Debug: orab int = const 1 to unsigned short FLAGS = [S+$10+$16] (used reg = )
mov	ax,$18[bp]
or	al,*1
mov	$18[bp],ax
!BCC_EOS
! 2855       regs.u.r8.ah = 0x86;
! Debug: eq int = const $86 to unsigned char regs = [S+$10+$11] (used reg = )
mov	al,#$86
mov	$13[bp],al
!BCC_EOS
! 2856       break;
jmp .331
!BCC_EOS
! 2857     }
! 2858 }
jmp .331
.333:
sub	al,#$C2
beq 	.334
jmp	.3A1
.331:
..FFF5	=	-$10
mov	sp,bp
pop	bp
ret
! 2859   void
! Register BX used in function int15_function_mouse
! 2860 int15_function32(regs, ES, DS, FLAGS)
! 2861   pushad_regs_t regs;
export	_int15_function32
_int15_function32:
!BCC_EOS
! 2862   Bit16u ES, DS, FLAGS;
!BCC_EOS
! 2863 {
! 2864   Bit32u extended_memory_size=0;
push	bp
mov	bp,sp
add	sp,*-4
! Debug: eq int = const 0 to unsigned long extended_memory_size = [S+6-6] (used reg = )
xor	ax,ax
xor	bx,bx
mov	-4[bp],ax
mov	-2[bp],bx
!BCC_EOS
! 2865   Bit16u CX,DX;
!BCC_EOS
! 2866   Bit16u off, e820_table_size;
!BCC_EOS
! 2867   Bit32u base, type, size;
!BCC_EOS
! 2868 ;
add	sp,*-$14
!BCC_EOS
! 2869   switch (regs.u.r8.ah) {
mov	al,$21[bp]
br 	.3A5
! 2870     case 0x86:
! 2871       CX = regs.u.r16.cx;
.3A6:
! Debug: eq unsigned short regs = [S+$1A+$1A] to unsigned short CX = [S+$1A-8] (used reg = )
mov	ax,$1C[bp]
mov	-6[bp],ax
!BCC_EOS
! 2872       DX = regs.u.r16.dx;
! Debug: eq unsigned short regs = [S+$1A+$16] to unsigned short DX = [S+$1A-$A] (used reg = )
mov	ax,$18[bp]
mov	-8[bp],ax
!BCC_EOS
! 2873 #asm
!BCC_EOS
!BCC_ASM
_int15_function32.CX	set	$12
.int15_function32.CX	set	-6
_int15_function32.extended_memory_size	set	$14
.int15_function32.extended_memory_size	set	-4
_int15_function32.FLAGS	set	$40
.int15_function32.FLAGS	set	$28
_int15_function32.type	set	4
.int15_function32.type	set	-$14
_int15_function32.DS	set	$3E
.int15_function32.DS	set	$26
_int15_function32.DX	set	$10
.int15_function32.DX	set	-8
_int15_function32.size	set	0
.int15_function32.size	set	-$18
_int15_function32.ES	set	$3C
.int15_function32.ES	set	$24
_int15_function32.e820_table_size	set	$C
.int15_function32.e820_table_size	set	-$C
_int15_function32.base	set	8
.int15_function32.base	set	-$10
_int15_function32.regs	set	$1C
.int15_function32.regs	set	4
_int15_function32.off	set	$E
.int15_function32.off	set	-$A
      ;; Get the count in eax
      mov ax, .int15_function32.CX [bp]
      shl eax, #16
      mov ax, .int15_function32.DX [bp]
      ;; convert to numbers of 15usec ticks
      mov ebx, #15
      xor edx, edx
      div eax, ebx
      mov ecx, eax
      ;; wait for ecx number of refresh requests
      in al, #0x61
      and al,#0x10
      mov ah, al
      or ecx, ecx
      je int1586_tick_end
int1586_tick:
      in al, #0x61
      and al,#0x10
      cmp al, ah
      je int1586_tick
      mov ah, al
      dec ecx
      jnz int1586_tick
int1586_tick_end:
! 2898 endasm
!BCC_ENDASM
!BCC_EOS
! 2899       break;
br 	.3A3
!BCC_EOS
! 2900     case 0xe8:
! 2901         switch(regs.u.r8.al)
.3A7:
mov	al,$20[bp]
! 2902         {
br 	.3AA
! 2903         case 0x20: {
.3AB:
! 2904             e820_table_size = read_word(0xe000, 0x8) * 0x14;
! Debug: list int = const 8 (used reg = )
mov	ax,*8
push	ax
! Debug: list unsigned int = const $E000 (used reg = )
mov	ax,#$E000
push	ax
! Debug: func () unsigned short = read_word+0 (used reg = )
call	_read_word
add	sp,*4
! Debug: mul int = const $14 to unsigned short = ax+0 (used reg = )
mov	cx,*$14
imul	cx
! Debug: eq unsigned int = ax+0 to unsigned short e820_table_size = [S+$1A-$E] (used reg = )
mov	-$C[bp],ax
!BCC_EOS
! 2905             if (regs.u.r32.edx != 0x534D4150)
! Debug: ne long = const $534D4150 to unsigned long regs = [S+$1A+$16] (used reg = )
! Debug: expression subtree swapping
mov	ax,#$4150
mov	bx,#$534D
push	bx
push	ax
mov	ax,$18[bp]
mov	bx,$1A[bp]
lea	di,-2+..FFF4[bp]
call	lcmpul
lea	sp,2+..FFF4[bp]
je  	.3AC
.3AD:
! 2906                 goto int15_unimplemented;
add	sp,#..FFF3-..FFF4
br 	.FFF3
!BCC_EOS
! 2907             if ((regs.u.r16.bx / 0x14) * 0x14 == regs.u.r16.bx) {
.3AC:
! Debug: div int = const $14 to unsigned short regs = [S+$1A+$12] (used reg = )
mov	ax,$14[bp]
mov	bx,*$14
call	idiv_u
! Debug: mul int = const $14 to unsigned int = ax+0 (used reg = )
mov	cx,*$14
imul	cx
! Debug: logeq unsigned short regs = [S+$1A+$12] to unsigned int = ax+0 (used reg = )
cmp	ax,$14[bp]
bne 	.3AE
.3AF:
! 2908                 if (regs.u.r16.bx + 0x14 <= e820_table_size)
! Debug: add int = const $14 to unsigned short regs = [S+$1A+$12] (used reg = )
mov	ax,$14[bp]
! Debug: le unsigned short e820_table_size = [S+$1A-$E] to unsigned int = ax+$14 (used reg = )
add	ax,*$14
cmp	ax,-$C[bp]
ja  	.3B0
.3B1:
! 2909                     memcpyb(ES, regs.u.r16.di,
! 2910                             0xe000, 0x10 + regs.u.r16.bx, 0x14);
! Debug: list int = const $14 (used reg = )
mov	ax,*$14
push	ax
! Debug: add unsigned short regs = [S+$1C+$12] to int = const $10 (used reg = )
! Debug: expression subtree swapping
mov	ax,$14[bp]
! Debug: list unsigned int = ax+$10 (used reg = )
add	ax,*$10
push	ax
! Debug: list unsigned int = const $E000 (used reg = )
mov	ax,#$E000
push	ax
! Debug: list unsigned short regs = [S+$20+2] (used reg = )
push	4[bp]
! Debug: list unsigned short ES = [S+$22+$22] (used reg = )
push	$24[bp]
! Debug: func () void = memcpyb+0 (used reg = )
call	_memcpyb
add	sp,*$A
!BCC_EOS
! 2911                 regs.u.r32.ebx += 0x14;
.3B0:
! Debug: addab unsigned long = const $14 to unsigned long regs = [S+$1A+$12] (used reg = )
mov	ax,*$14
xor	bx,bx
push	bx
push	ax
mov	ax,$14[bp]
mov	bx,$16[bp]
lea	di,-2+..FFF4[bp]
call	laddul
mov	$14[bp],ax
mov	$16[bp],bx
add	sp,*4
!BCC_EOS
! 2912                 if ((regs.u.r32.ebx + 0x14 - 1) > e820_table_size)
! Debug: cast unsigned long = const 0 to unsigned short e820_table_size = [S+$1A-$E] (used reg = )
mov	ax,-$C[bp]
xor	bx,bx
push	bx
push	ax
! Debug: add unsigned long = const $14 to unsigned long regs = [S+$1E+$12] (used reg = )
! Debug: expression subtree swapping
mov	ax,*$14
xor	bx,bx
lea	di,$14[bp]
call	laddul
! Debug: sub unsigned long = const 1 to unsigned long = bx+0 (used reg = )
push	bx
push	ax
mov	ax,*1
xor	bx,bx
push	bx
push	ax
mov	ax,-6+..FFF4[bp]
mov	bx,-4+..FFF4[bp]
lea	di,-$A+..FFF4[bp]
call	lsubul
add	sp,*8
! Debug: gt unsigned long (temp) = [S+$1E-$1E] to unsigned long = bx+0 (used reg = )
lea	di,-2+..FFF4[bp]
call	lcmpul
lea	sp,2+..FFF4[bp]
jbe 	.3B2
.3B3:
! 2913                     regs.u.r32.ebx = 0;
! Debug: eq int = const 0 to unsigned long regs = [S+$1A+$12] (used reg = )
xor	ax,ax
xor	bx,bx
mov	$14[bp],ax
mov	$16[bp],bx
!BCC_EOS
! 2914             } else if (regs.u.r16.bx == 1) {
.3B2:
br 	.3B4
.3AE:
! Debug: logeq int = const 1 to unsigned short regs = [S+$1A+$12] (used reg = )
mov	ax,$14[bp]
cmp	ax,*1
bne 	.3B5
.3B6:
! 2915                 for (off = 0; off < e820_table_size; off += 0x14) {
! Debug: eq int = const 0 to unsigned short off = [S+$1A-$C] (used reg = )
xor	ax,ax
mov	-$A[bp],ax
!BCC_EOS
!BCC_EOS
jmp .3B9
.3BA:
! 2916                     base = read_dword(0xe000, 0x10 + off);
! Debug: add unsigned short off = [S+$1A-$C] to int = const $10 (used reg = )
! Debug: expression subtree swapping
mov	ax,-$A[bp]
! Debug: list unsigned int = ax+$10 (used reg = )
add	ax,*$10
push	ax
! Debug: list unsigned int = const $E000 (used reg = )
mov	ax,#$E000
push	ax
! Debug: func () unsigned long = read_dword+0 (used reg = )
call	_read_dword
mov	bx,dx
add	sp,*4
! Debug: eq unsigned long = bx+0 to unsigned long base = [S+$1A-$12] (used reg = )
mov	-$10[bp],ax
mov	-$E[bp],bx
!BCC_EOS
! 2917                     type = read_dword(0xe000, 0x20 + off);
! Debug: add unsigned short off = [S+$1A-$C] to int = const $20 (used reg = )
! Debug: expression subtree swapping
mov	ax,-$A[bp]
! Debug: list unsigned int = ax+$20 (used reg = )
add	ax,*$20
push	ax
! Debug: list unsigned int = const $E000 (used reg = )
mov	ax,#$E000
push	ax
! Debug: func () unsigned long = read_dword+0 (used reg = )
call	_read_dword
mov	bx,dx
add	sp,*4
! Debug: eq unsigned long = bx+0 to unsigned long type = [S+$1A-$16] (used reg = )
mov	-$14[bp],ax
mov	-$12[bp],bx
!BCC_EOS
! 2918                     if ((base >= 0x100000) && (type == 1))
! Debug: ge long = const $100000 to unsigned long base = [S+$1A-$12] (used reg = )
xor	ax,ax
mov	bx,*$10
lea	di,-$10[bp]
call	lcmpul
ja  	.3BB
.3BD:
! Debug: logeq unsigned long = const 1 to unsigned long type = [S+$1A-$16] (used reg = )
! Debug: expression subtree swapping
mov	ax,*1
xor	bx,bx
push	bx
push	ax
mov	ax,-$14[bp]
mov	bx,-$12[bp]
lea	di,-2+..FFF4[bp]
call	lcmpul
lea	sp,2+..FFF4[bp]
jne 	.3BB
.3BC:
! 2919                         break;
jmp .3B7
!BCC_EOS
! 2920                 }
.3BB:
! 2921                 if (off == e820_table_size) {
.3B8:
! Debug: addab int = const $14 to unsigned short off = [S+$1A-$C] (used reg = )
mov	ax,-$A[bp]
add	ax,*$14
mov	-$A[bp],ax
.3B9:
! Debug: lt unsigned short e820_table_size = [S+$1A-$E] to unsigned short off = [S+$1A-$C] (used reg = )
mov	ax,-$A[bp]
cmp	ax,-$C[bp]
jb 	.3BA
.3BE:
.3B7:
! Debug: logeq unsigned short e820_table_size = [S+$1A-$E] to unsigned short off = [S+$1A-$C] (used reg = )
mov	ax,-$A[bp]
cmp	ax,-$C[bp]
jne 	.3BF
.3C0:
! 2922                     FLAGS |= 0x0001;
! Debug: orab int = const 1 to unsigned short FLAGS = [S+$1A+$26] (used reg = )
mov	ax,$28[bp]
or	al,*1
mov	$28[bp],ax
!BCC_EOS
! 2923                     break;
br 	.3A8
!BCC_EOS
! 2924                 }
! 2925                 memcpyb(ES, regs.u.r16.di, 0xe000, 0x10 + off, 0x14);
.3BF:
! Debug: list int = const $14 (used reg = )
mov	ax,*$14
push	ax
! Debug: add unsigned short off = [S+$1C-$C] to int = const $10 (used reg = )
! Debug: expression subtree swapping
mov	ax,-$A[bp]
! Debug: list unsigned int = ax+$10 (used reg = )
add	ax,*$10
push	ax
! Debug: list unsigned int = const $E000 (used reg = )
mov	ax,#$E000
push	ax
! Debug: list unsigned short regs = [S+$20+2] (used reg = )
push	4[bp]
! Debug: list unsigned short ES = [S+$22+$22] (used reg = )
push	$24[bp]
! Debug: func () void = memcpyb+0 (used reg = )
call	_memcpyb
add	sp,*$A
!BCC_EOS
! 2926                 regs.u.r32.ebx = 0;
! Debug: eq int = const 0 to unsigned long regs = [S+$1A+$12] (used reg = )
xor	ax,ax
xor	bx,bx
mov	$14[bp],ax
mov	$16[bp],bx
!BCC_EOS
! 2927             } else {
jmp .3C1
.3B5:
! 2928                 goto int15_unimplemented;
add	sp,#..FFF3-..FFF4
br 	.FFF3
!BCC_EOS
! 2929             }
! 2930             regs.u.r32.eax = 0x534D4150;
.3C1:
.3B4:
! Debug: eq long = const $534D4150 to unsigned long regs = [S+$1A+$1E] (used reg = )
mov	ax,#$4150
mov	bx,#$534D
mov	$20[bp],ax
mov	$22[bp],bx
!BCC_EOS
! 2931             regs.u.r32.ecx = 0x14;
! Debug: eq int = const $14 to unsigned long regs = [S+$1A+$1A] (used reg = )
mov	ax,*$14
xor	bx,bx
mov	$1C[bp],ax
mov	$1E[bp],bx
!BCC_EOS
! 2932             FLAGS &= 0xfffe;
! Debug: andab unsigned int = const $FFFE to unsigned short FLAGS = [S+$1A+$26] (used reg = )
mov	ax,$28[bp]
and	al,#$FE
mov	$28[bp],ax
!BCC_EOS
! 2933             break;
br 	.3A8
!BCC_EOS
! 2934         }
! 2935         case 0x01: {
.3C2:
! 2936             e820_table_size = read_word(0xe000, 0x8) * 0x14;
! Debug: list int = const 8 (used reg = )
mov	ax,*8
push	ax
! Debug: list unsigned int = const $E000 (used reg = )
mov	ax,#$E000
push	ax
! Debug: func () unsigned short = read_word+0 (used reg = )
call	_read_word
add	sp,*4
! Debug: mul int = const $14 to unsigned short = ax+0 (used reg = )
mov	cx,*$14
imul	cx
! Debug: eq unsigned int = ax+0 to unsigned short e820_table_size = [S+$1A-$E] (used reg = )
mov	-$C[bp],ax
!BCC_EOS
! 2937             FLAGS &= 0xfffe;
! Debug: andab unsigned int = const $FFFE to unsigned short FLAGS = [S+$1A+$26] (used reg = )
mov	ax,$28[bp]
and	al,#$FE
mov	$28[bp],ax
!BCC_EOS
! 2938             regs.u.r8.cl = inb_cmos(0x3
! 2938 0);
! Debug: list int = const $30 (used reg = )
mov	ax,*$30
push	ax
! Debug: func () unsigned char = inb_cmos+0 (used reg = )
call	_inb_cmos
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char regs = [S+$1A+$1A] (used reg = )
mov	$1C[bp],al
!BCC_EOS
! 2939             regs.u.r8.ch = inb_cmos(0x31);
! Debug: list int = const $31 (used reg = )
mov	ax,*$31
push	ax
! Debug: func () unsigned char = inb_cmos+0 (used reg = )
call	_inb_cmos
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char regs = [S+$1A+$1B] (used reg = )
mov	$1D[bp],al
!BCC_EOS
! 2940             if (regs.u.r16.cx > (15*1024))
! Debug: gt int = const $3C00 to unsigned short regs = [S+$1A+$1A] (used reg = )
mov	ax,$1C[bp]
cmp	ax,#$3C00
jbe 	.3C3
.3C4:
! 2941                 regs.u.r16.cx = 15*1024;
! Debug: eq int = const $3C00 to unsigned short regs = [S+$1A+$1A] (used reg = )
mov	ax,#$3C00
mov	$1C[bp],ax
!BCC_EOS
! 2942             for (off = 0; off < e820_table_size; off += 0x14) {
.3C3:
! Debug: eq int = const 0 to unsigned short off = [S+$1A-$C] (used reg = )
xor	ax,ax
mov	-$A[bp],ax
!BCC_EOS
!BCC_EOS
jmp .3C7
.3C8:
! 2943                 base = read_dword(0xe000, 0x10 + off);
! Debug: add unsigned short off = [S+$1A-$C] to int = const $10 (used reg = )
! Debug: expression subtree swapping
mov	ax,-$A[bp]
! Debug: list unsigned int = ax+$10 (used reg = )
add	ax,*$10
push	ax
! Debug: list unsigned int = const $E000 (used reg = )
mov	ax,#$E000
push	ax
! Debug: func () unsigned long = read_dword+0 (used reg = )
call	_read_dword
mov	bx,dx
add	sp,*4
! Debug: eq unsigned long = bx+0 to unsigned long base = [S+$1A-$12] (used reg = )
mov	-$10[bp],ax
mov	-$E[bp],bx
!BCC_EOS
! 2944                 type = read_dword(0xe000, 0x20 + off);
! Debug: add unsigned short off = [S+$1A-$C] to int = const $20 (used reg = )
! Debug: expression subtree swapping
mov	ax,-$A[bp]
! Debug: list unsigned int = ax+$20 (used reg = )
add	ax,*$20
push	ax
! Debug: list unsigned int = const $E000 (used reg = )
mov	ax,#$E000
push	ax
! Debug: func () unsigned long = read_dword+0 (used reg = )
call	_read_dword
mov	bx,dx
add	sp,*4
! Debug: eq unsigned long = bx+0 to unsigned long type = [S+$1A-$16] (used reg = )
mov	-$14[bp],ax
mov	-$12[bp],bx
!BCC_EOS
! 2945                 if ((base >= 0x100000) && (type == 1))
! Debug: ge long = const $100000 to unsigned long base = [S+$1A-$12] (used reg = )
xor	ax,ax
mov	bx,*$10
lea	di,-$10[bp]
call	lcmpul
ja  	.3C9
.3CB:
! Debug: logeq unsigned long = const 1 to unsigned long type = [S+$1A-$16] (used reg = )
! Debug: expression subtree swapping
mov	ax,*1
xor	bx,bx
push	bx
push	ax
mov	ax,-$14[bp]
mov	bx,-$12[bp]
lea	di,-2+..FFF4[bp]
call	lcmpul
lea	sp,2+..FFF4[bp]
jne 	.3C9
.3CA:
! 2946                     break;
jmp .3C5
!BCC_EOS
! 2947             }
.3C9:
! 2948             regs.u.r16.dx = 0;
.3C6:
! Debug: addab int = const $14 to unsigned short off = [S+$1A-$C] (used reg = )
mov	ax,-$A[bp]
add	ax,*$14
mov	-$A[bp],ax
.3C7:
! Debug: lt unsigned short e820_table_size = [S+$1A-$E] to unsigned short off = [S+$1A-$C] (used reg = )
mov	ax,-$A[bp]
cmp	ax,-$C[bp]
jb 	.3C8
.3CC:
.3C5:
! Debug: eq int = const 0 to unsigned short regs = [S+$1A+$16] (used reg = )
xor	ax,ax
mov	$18[bp],ax
!BCC_EOS
! 2949             if (off != e820_table_size) {
! Debug: ne unsigned short e820_table_size = [S+$1A-$E] to unsigned short off = [S+$1A-$C] (used reg = )
mov	ax,-$A[bp]
cmp	ax,-$C[bp]
je  	.3CD
.3CE:
! 2950                 size = base + read_dword(0xe000, 0x18 + off);
! Debug: add unsigned short off = [S+$1A-$C] to int = const $18 (used reg = )
! Debug: expression subtree swapping
mov	ax,-$A[bp]
! Debug: list unsigned int = ax+$18 (used reg = )
add	ax,*$18
push	ax
! Debug: list unsigned int = const $E000 (used reg = )
mov	ax,#$E000
push	ax
! Debug: func () unsigned long = read_dword+0 (used reg = )
call	_read_dword
mov	bx,dx
add	sp,*4
! Debug: add unsigned long = bx+0 to unsigned long base = [S+$1A-$12] (used reg = )
! Debug: expression subtree swapping
lea	di,-$10[bp]
call	laddul
! Debug: eq unsigned long = bx+0 to unsigned long size = [S+$1A-$1A] (used reg = )
mov	-$18[bp],ax
mov	-$16[bp],bx
!BCC_EOS
! 2951                 if (size > 0x1000000) {
! Debug: gt long = const $1000000 to unsigned long size = [S+$1A-$1A] (used reg = )
xor	ax,ax
mov	bx,#$100
lea	di,-$18[bp]
call	lcmpul
jae 	.3CF
.3D0:
! 2952                     size -= 0x1000000;
! Debug: subab long = const $1000000 to unsigned long size = [S+$1A-$1A] (used reg = )
xor	ax,ax
mov	bx,#$100
push	bx
push	ax
mov	ax,-$18[bp]
mov	bx,-$16[bp]
lea	di,-2+..FFF4[bp]
call	lsubul
mov	-$18[bp],ax
mov	-$16[bp],bx
add	sp,*4
!BCC_EOS
! 2953                     regs.u.r16.dx = (Bit16u)(size >> 16);
! Debug: sr int = const $10 to unsigned long size = [S+$1A-$1A] (used reg = )
mov	ax,-$18[bp]
mov	bx,-$16[bp]
xchg	bx,ax
xor	bx,bx
! Debug: cast unsigned short = const 0 to unsigned long = bx+0 (used reg = )
! Debug: eq unsigned short = ax+0 to unsigned short regs = [S+$1A+$16] (used reg = )
mov	$18[bp],ax
!BCC_EOS
! 2954                 }
! 2955             }
.3CF:
! 2956             regs.u.r16.ax = regs.u.r16.cx;
.3CD:
! Debug: eq unsigned short regs = [S+$1A+$1A] to unsigned short regs = [S+$1A+$1E] (used reg = )
mov	ax,$1C[bp]
mov	$20[bp],ax
!BCC_EOS
! 2957             regs.u.r16.bx = regs.u.r16.dx;
! Debug: eq unsigned short regs = [S+$1A+$16] to unsigned short regs = [S+$1A+$12] (used reg = )
mov	ax,$18[bp]
mov	$14[bp],ax
!BCC_EOS
! 2958             break;
jmp .3A8
!BCC_EOS
! 2959         }
! 2960  default:
! 2961             goto int15_unimplemented;
.3D1:
add	sp,#..FFF3-..FFF4
jmp .FFF3
!BCC_EOS
! 2962         }
! 2963         break;
jmp .3A8
.3AA:
sub	al,*1
beq 	.3C2
sub	al,*$1F
beq 	.3AB
jmp	.3D1
.3A8:
jmp .3A3
!BCC_EOS
! 2964     int15_unimplemented:
.FFF3:
! 2965     default:
! 2966       bios_printf(4, "*** int 15h function AX=%04x, BX=%04x not yet supported!\n", (unsigned) regs.u.r16.ax, (unsigned) regs.u.r16.bx);
.3D2:
! Debug: list unsigned short regs = [S+$1A+$12] (used reg = )
push	$14[bp]
! Debug: list unsigned short regs = [S+$1C+$1E] (used reg = )
push	$20[bp]
! Debug: list * char = .3D3+0 (used reg = )
mov	bx,#.3D3
push	bx
! Debug: list int = const 4 (used reg = )
mov	ax,*4
push	ax
! Debug: func () void = bios_printf+0 (used reg = )
call	_bios_printf
add	sp,*8
!BCC_EOS
! 2967       FLAGS |= 0x0001;
! Debug: orab int = const 1 to unsigned short FLAGS = [S+$1A+$26] (used reg = )
mov	ax,$28[bp]
or	al,*1
mov	$28[bp],ax
!BCC_EOS
! 2968       regs.u.r8.ah = 0x86;
! Debug: eq int = const $86 to unsigned char regs = [S+$1A+$1F] (used reg = )
mov	al,#$86
mov	$21[bp],al
!BCC_EOS
! 2969       break;
jmp .3A3
!BCC_EOS
! 2970     }
! 2971 }
jmp .3A3
.3A5:
sub	al,#$86
beq 	.3A6
sub	al,*$62
beq 	.3A7
jmp	.3D2
.3A3:
..FFF4	=	-$1A
..FFF3	=	-$1A
mov	sp,bp
pop	bp
ret
! 2972   void
! Register BX used in function int15_function32
! 2973 int16_function(DI, SI, BP, SP, BX, DX, CX, AX, FLAGS)
! 2974   Bit16u DI, SI, BP, SP, BX, DX, CX, AX, FLAGS;
export	_int16_function
_int16_function:
!BCC_EOS
! 2975 {
! 2976   Bit8u scan_code, ascii_code, shift_flags, count;
!BCC_EOS
! 2977   Bit16u kbd_code, max;
!BCC_EOS
! 2978   ;
push	bp
mov	bp,sp
add	sp,*-8
!BCC_EOS
! 2979   switch (( AX >> 8 )) {
! Debug: sr int = const 8 to unsigned short AX = [S+$A+$10] (used reg = )
mov	ax,$12[bp]
mov	al,ah
xor	ah,ah
br 	.3D6
! 2980     case 0x00:
! 2981       if ( !dequeue_key(&scan_code, &ascii_code, 1) ) {
.3D7:
! Debug: list int = const 1 (used reg = )
mov	ax,*1
push	ax
! Debug: list * unsigned char ascii_code = S+$C-4 (used reg = )
lea	bx,-2[bp]
push	bx
! Debug: list * unsigned char scan_code = S+$E-3 (used reg = )
lea	bx,-1[bp]
push	bx
! Debug: func () unsigned int = dequeue_key+0 (used reg = )
call	_dequeue_key
add	sp,*6
test	ax,ax
jne 	.3D8
.3D9:
! 2982         bios_printf((2 | 4 | 1), "KBD: int16h: out of keyboard input\n");
! Debug: list * char = .3DA+0 (used reg = )
mov	bx,#.3DA
push	bx
! Debug: list int = const 7 (used reg = )
mov	ax,*7
push	ax
! Debug: func () void = bios_printf+0 (used reg = )
call	_bios_printf
add	sp,*4
!BCC_EOS
! 2983         }
! 2984       if (scan_code !=0 && ascii_code == 0xF0) ascii_code = 0;
.3D8:
! Debug: ne int = const 0 to unsigned char scan_code = [S+$A-3] (used reg = )
mov	al,-1[bp]
test	al,al
je  	.3DB
.3DD:
! Debug: logeq int = const $F0 to unsigned char ascii_code = [S+$A-4] (used reg = )
mov	al,-2[bp]
cmp	al,#$F0
jne 	.3DB
.3DC:
! Debug: eq int = const 0 to unsigned char ascii_code = [S+$A-4] (used reg = )
xor	al,al
mov	-2[bp],al
!BCC_EOS
! 2985       else if (ascii_code == 0xE0) ascii_code = 0;
jmp .3DE
.3DB:
! Debug: logeq int = const $E0 to unsigned char ascii_code = [S+$A-4] (used reg = )
mov	al,-2[bp]
cmp	al,#$E0
jne 	.3DF
.3E0:
! Debug: eq int = const 0 to unsigned char ascii_code = [S+$A-4] (used reg = )
xor	al,al
mov	-2[bp],al
!BCC_EOS
! 2986       AX = (scan_code << 8) | ascii_code;
.3DF:
.3DE:
! Debug: sl int = const 8 to unsigned char scan_code = [S+$A-3] (used reg = )
mov	al,-1[bp]
xor	ah,ah
mov	ah,al
xor	al,al
! Debug: or unsigned char ascii_code = [S+$A-4] to unsigned int = ax+0 (used reg = )
or	al,-2[bp]
! Debug: eq unsigned int = ax+0 to unsigned short AX = [S+$A+$10] (used reg = )
mov	$12[bp],ax
!BCC_EOS
! 2987       break;
br 	.3D4
!BCC_EOS
! 2988     case 0x01:
! 2989       if ( !dequeue_key(&scan_code, &ascii_code, 0) ) {
.3E1:
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: list * unsigned char ascii_code = S+$C-4 (used reg = )
lea	bx,-2[bp]
push	bx
! Debug: list * unsigned char scan_code = S+$E-3 (used reg = )
lea	bx,-1[bp]
push	bx
! Debug: func () unsigned int = dequeue_key+0 (used reg = )
call	_dequeue_key
add	sp,*6
test	ax,ax
jne 	.3E2
.3E3:
! 2990         FLAGS |= 0x0040;
! Debug: orab int = const $40 to unsigned short FLAGS = [S+$A+$12] (used reg = )
mov	ax,$14[bp]
or	al,*$40
mov	$14[bp],ax
!BCC_EOS
! 2991         return;
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 2992         }
! 2993       if (scan_code !=0 && ascii_code == 0xF0) ascii_code = 0;
.3E2:
! Debug: ne int = const 0 to unsigned char scan_code = [S+$A-3] (used reg = )
mov	al,-1[bp]
test	al,al
je  	.3E4
.3E6:
! Debug: logeq int = const $F0 to unsigned char ascii_code = [S+$A-4] (used reg = )
mov	al,-2[bp]
cmp	al,#$F0
jne 	.3E4
.3E5:
! Debug: eq int = const 0 to unsigned char ascii_code = [S+$A-4] (used reg = )
xor	al,al
mov	-2[bp],al
!BCC_EOS
! 2994       else if (ascii_code == 0xE0) ascii_code = 0;
jmp .3E7
.3E4:
! Debug: logeq int = const $E0 to unsigned char ascii_code = [S+$A-4] (used reg = )
mov	al,-2[bp]
cmp	al,#$E0
jne 	.3E8
.3E9:
! Debug: eq int = const 0 to unsigned char ascii_code = [S+$A-4] (used reg = )
xor	al,al
mov	-2[bp],al
!BCC_EOS
! 2995       AX = (scan_code << 8) | ascii_code;
.3E8:
.3E7:
! Debug: sl int = const 8 to unsigned char scan_code = [S+$A-3] (used reg = )
mov	al,-1[bp]
xor	ah,ah
mov	ah,al
xor	al,al
! Debug: or unsigned char ascii_code = [S+$A-4] to unsigned int = ax+0 (used reg = )
or	al,-2[bp]
! Debug: eq unsigned int = ax+0 to unsigned short AX = [S+$A+$10] (used reg = )
mov	$12[bp],ax
!BCC_EOS
! 2996       FLAGS &= 0xffbf;
! Debug: andab unsigned int = const $FFBF to unsigned short FLAGS = [S+$A+$12] (used reg = )
mov	ax,$14[bp]
and	al,#$BF
mov	$14[bp],ax
!BCC_EOS
! 2997       break;
br 	.3D4
!BCC_EOS
! 2998     case 0x02:
! 2999       shift_flags = read_byte(0x0040, 0x17
.3EA:
! 2999 );
! Debug: list int = const $17 (used reg = )
mov	ax,*$17
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () unsigned char = read_byte+0 (used reg = )
call	_read_byte
add	sp,*4
! Debug: eq unsigned char = al+0 to unsigned char shift_flags = [S+$A-5] (used reg = )
mov	-3[bp],al
!BCC_EOS
! 3000       AX = ((AX & 0xff00) | (shift_flags));
! Debug: and unsigned int = const $FF00 to unsigned short AX = [S+$A+$10] (used reg = )
mov	ax,$12[bp]
xor	al,al
! Debug: or unsigned char shift_flags = [S+$A-5] to unsigned int = ax+0 (used reg = )
or	al,-3[bp]
! Debug: eq unsigned int = ax+0 to unsigned short AX = [S+$A+$10] (used reg = )
mov	$12[bp],ax
!BCC_EOS
! 3001       break;
br 	.3D4
!BCC_EOS
! 3002     case 0x05:
! 3003       if ( !enqueue_key(( CX >> 8 ), ( CX & 0x00ff )) ) {
.3EB:
! Debug: and int = const $FF to unsigned short CX = [S+$A+$E] (used reg = )
mov	al,$10[bp]
! Debug: list unsigned char = al+0 (used reg = )
xor	ah,ah
push	ax
! Debug: sr int = const 8 to unsigned short CX = [S+$C+$E] (used reg = )
mov	ax,$10[bp]
mov	al,ah
xor	ah,ah
! Debug: list unsigned int = ax+0 (used reg = )
push	ax
! Debug: func () unsigned int = enqueue_key+0 (used reg = )
call	_enqueue_key
add	sp,*4
test	ax,ax
jne 	.3EC
.3ED:
! 3004         AX = ((AX & 0xff00) | (1));
! Debug: and unsigned int = const $FF00 to unsigned short AX = [S+$A+$10] (used reg = )
mov	ax,$12[bp]
xor	al,al
! Debug: or int = const 1 to unsigned int = ax+0 (used reg = )
or	al,*1
! Debug: eq unsigned int = ax+0 to unsigned short AX = [S+$A+$10] (used reg = )
mov	$12[bp],ax
!BCC_EOS
! 3005         }
! 3006       else {
jmp .3EE
.3EC:
! 3007         AX = ((AX & 0xff00) | (0));
! Debug: and unsigned int = const $FF00 to unsigned short AX = [S+$A+$10] (used reg = )
mov	ax,$12[bp]
xor	al,al
! Debug: or int = const 0 to unsigned int = ax+0 (used reg = )
or	al,*0
! Debug: eq unsigned int = ax+0 to unsigned short AX = [S+$A+$10] (used reg = )
mov	$12[bp],ax
!BCC_EOS
! 3008         }
! 3009       break;
.3EE:
br 	.3D4
!BCC_EOS
! 3010     case 0x09:
! 3011       AX = ((AX & 0xff00) | (0x30));
.3EF:
! Debug: and unsigned int = const $FF00 to unsigned short AX = [S+$A+$10] (used reg = )
mov	ax,$12[bp]
xor	al,al
! Debug: or int = const $30 to unsigned int = ax+0 (used reg = )
or	al,*$30
! Debug: eq unsigned int = ax+0 to unsigned short AX = [S+$A+$10] (used reg = )
mov	$12[bp],ax
!BCC_EOS
! 3012       break;
br 	.3D4
!BCC_EOS
! 3013     case 0x0A:
! 3014       count = 2;
.3F0:
! Debug: eq int = const 2 to unsigned char count = [S+$A-6] (used reg = )
mov	al,*2
mov	-4[bp],al
!BCC_EOS
! 3015       kbd_code = 0x0;
! Debug: eq int = const 0 to unsigned short kbd_code = [S+$A-8] (used reg = )
xor	ax,ax
mov	-6[bp],ax
!BCC_EOS
! 3016       outb(0x60, 0xf2);
! Debug: list int = const $F2 (used reg = )
mov	ax,#$F2
push	ax
! Debug: list int = const $60 (used reg = )
mov	ax,*$60
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 3017       max=0xffff;
! Debug: eq unsigned int = const $FFFF to unsigned short max = [S+$A-$A] (used reg = )
mov	ax,#$FFFF
mov	-8[bp],ax
!BCC_EOS
! 3018       while ( ((inb(0x64) & 0x01) == 0) && (--max>0) ) outb(0x80, 0x00);
jmp .3F2
.3F3:
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: list int = const $80 (used reg = )
mov	ax,#$80
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 3019       if (max>0x0) {
.3F2:
! Debug: list int = const $64 (used reg = )
mov	ax,*$64
push	ax
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
inc	sp
inc	sp
! Debug: and int = const 1 to unsigned char = al+0 (used reg = )
and	al,*1
! Debug: logeq int = const 0 to unsigned char = al+0 (used reg = )
test	al,al
jne 	.3F4
.3F5:
! Debug: predec unsigned short max = [S+$A-$A] (used reg = )
mov	ax,-8[bp]
dec	ax
mov	-8[bp],ax
! Debug: gt int = const 0 to unsigned short = ax+0 (used reg = )
test	ax,ax
jne	.3F3
.3F4:
.3F1:
! Debug: gt int = const 0 to unsigned short max = [S+$A-$A] (used reg = )
mov	ax,-8[bp]
test	ax,ax
je  	.3F6
.3F7:
! 3020         if ((inb(0x60) == 0xfa)) {
! Debug: list int = const $60 (used reg = )
mov	ax,*$60
push	ax
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
inc	sp
inc	sp
! Debug: logeq int = const $FA to unsigned char = al+0 (used reg = )
cmp	al,#$FA
jne 	.3F8
.3F9:
! 3021           do {
.3FC:
! 3022             max=0xffff;
! Debug: eq unsigned int = const $FFFF to unsigned short max = [S+$A-$A] (used reg = )
mov	ax,#$FFFF
mov	-8[bp],ax
!BCC_EOS
! 3023             while ( ((inb(0x64) & 0x01) == 0) && (--max>0) ) outb(0x80, 0x00);
jmp .3FE
.3FF:
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: list int = const $80 (used reg = )
mov	ax,#$80
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 3024             if (max>0x0) {
.3FE:
! Debug: list int = const $64 (used reg = )
mov	ax,*$64
push	ax
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
inc	sp
inc	sp
! Debug: and int = const 1 to unsigned char = al+0 (used reg = )
and	al,*1
! Debug: logeq int = const 0 to unsigned char = al+0 (used reg = )
test	al,al
jne 	.400
.401:
! Debug: predec unsigned short max = [S+$A-$A] (used reg = )
mov	ax,-8[bp]
dec	ax
mov	-8[bp],ax
! Debug: gt int = const 0 to unsigned short = ax+0 (used reg = )
test	ax,ax
jne	.3FF
.400:
.3FD:
! Debug: gt int = const 0 to unsigned short max = [S+$A-$A] (used reg = )
mov	ax,-8[bp]
test	ax,ax
je  	.402
.403:
! 3025               kbd_code >>= 8;
! Debug: srab int = const 8 to unsigned short kbd_code = [S+$A-8] (used reg = )
mov	ax,-6[bp]
mov	al,ah
xor	ah,ah
mov	-6[bp],ax
!BCC_EOS
! 3026               kbd_code |= (inb(0x60) << 8);
! Debug: list int = const $60 (used reg = )
mov	ax,*$60
push	ax
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
inc	sp
inc	sp
! Debug: sl int = const 8 to unsigned char = al+0 (used reg = )
xor	ah,ah
mov	ah,al
xor	al,al
! Debug: orab unsigned int = ax+0 to unsigned short kbd_code = [S+$A-8] (used reg = )
or	ax,-6[bp]
mov	-6[bp],ax
!BCC_EOS
! 3027             }
! 3028           } while (--count>0);
.402:
.3FB:
! Debug: predec unsigned char count = [S+$A-6] (used reg = )
mov	al,-4[bp]
dec	ax
mov	-4[bp],al
! Debug: gt int = const 0 to unsigned char = al+0 (used reg = )
test	al,al
jne	.3FC
.404:
!BCC_EOS
! 3029  }
.3FA:
! 3030       }
.3F8:
! 3031       BX=kbd_code;
.3F6:
! Debug: eq unsigned short kbd_code = [S+$A-8] to unsigned short BX = [S+$A+$A] (used reg = )
mov	ax,-6[bp]
mov	$C[bp],ax
!BCC_EOS
! 3032       break;
br 	.3D4
!BCC_EOS
! 3033     case 0x10:
! 3034       if ( !dequeue_key(&scan_code, &ascii_code, 1) ) {
.405:
! Debug: list int = const 1 (used reg = )
mov	ax,*1
push	ax
! Debug: list * unsigned char ascii_code = S+$C-4 (used reg = )
lea	bx,-2[bp]
push	bx
! Debug: list * unsigned char scan_code = S+$E-3 (used reg = )
lea	bx,-1[bp]
push	bx
! Debug: func () unsigned int = dequeue_key+0 (used reg = )
call	_dequeue_key
add	sp,*6
test	ax,ax
jne 	.406
.407:
! 3035         bios_printf((2 | 4 | 1), "KBD: int16h: out of keyboard input\n");
! Debug: list * char = .408+0 (used reg = )
mov	bx,#.408
push	bx
! Debug: list int = const 7 (used reg = )
mov	ax,*7
push	ax
! Debug: func () void = bios_printf+0 (used reg = )
call	_bios_printf
add	sp,*4
!BCC_EOS
! 3036         }
! 3037       if (scan_code !=0 && ascii_code == 0xF0) ascii_code = 0;
.406:
! Debug: ne int = const 0 to unsigned char scan_code = [S+$A-3] (used reg = )
mov	al,-1[bp]
test	al,al
je  	.409
.40B:
! Debug: logeq int = const $F0 to unsigned char ascii_code = [S+$A-4] (used reg = )
mov	al,-2[bp]
cmp	al,#$F0
jne 	.409
.40A:
! Debug: eq int = const 0 to unsigned char ascii_code = [S+$A-4] (used reg = )
xor	al,al
mov	-2[bp],al
!BCC_EOS
! 3038       AX = (scan_code << 8) | ascii_code;
.409:
! Debug: sl int = const 8 to unsigned char scan_code = [S+$A-3] (used reg = )
mov	al,-1[bp]
xor	ah,ah
mov	ah,al
xor	al,al
! Debug: or unsigned char ascii_code = [S+$A-4] to unsigned int = ax+0 (used reg = )
or	al,-2[bp]
! Debug: eq unsigned int = ax+0 to unsigned short AX = [S+$A+$10] (used reg = )
mov	$12[bp],ax
!BCC_EOS
! 3039       break;
br 	.3D4
!BCC_EOS
! 3040     case 0x11:
! 3041       if ( !dequeue_key(&scan_code, &ascii_code, 0) ) {
.40C:
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: list * unsigned char ascii_code = S+$C-4 (used reg = )
lea	bx,-2[bp]
push	bx
! Debug: list * unsigned char scan_code = S+$E-3 (used reg = )
lea	bx,-1[bp]
push	bx
! Debug: func () unsigned int = dequeue_key+0 (used reg = )
call	_dequeue_key
add	sp,*6
test	ax,ax
jne 	.40D
.40E:
! 3042         FLAGS |= 0x0040;
! Debug: orab int = const $40 to unsigned short FLAGS = [S+$A+$12] (used reg = )
mov	ax,$14[bp]
or	al,*$40
mov	$14[bp],ax
!BCC_EOS
! 3043         return;
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 3044         }
! 3045       if (scan_code !=0 && ascii_code == 0xF0) ascii_code = 0;
.40D:
! Debug: ne int = const 0 to unsigned char scan_code = [S+$A-3] (used reg = )
mov	al,-1[bp]
test	al,al
je  	.40F
.411:
! Debug: logeq int = const $F0 to unsigned char ascii_code = [S+$A-4] (used reg = )
mov	al,-2[bp]
cmp	al,#$F0
jne 	.40F
.410:
! Debug: eq int = const 0 to unsigned char ascii_code = [S+$A-4] (used reg = )
xor	al,al
mov	-2[bp],al
!BCC_EOS
! 3046       AX = (scan_code << 8) | ascii_code;
.40F:
! Debug: sl int = const 8 to unsigned char scan_code = [S+$A-3] (used reg = )
mov	al,-1[bp]
xor	ah,ah
mov	ah,al
xor	al,al
! Debug: or unsigned char ascii_code = [S+$A-4] to unsigned int = ax+0 (used reg = )
or	al,-2[bp]
! Debug: eq unsigned int = ax+0 to unsigned short AX = [S+$A+$10] (used reg = )
mov	$12[bp],ax
!BCC_EOS
! 3047       FLAGS &= 0xffbf;
! Debug: andab unsigned int = const $FFBF to unsigned short FLAGS = [S+$A+$12] (used reg = )
mov	ax,$14[bp]
and	al,#$BF
mov	$14[bp],ax
!BCC_EOS
! 3048       break;
br 	.3D4
!BCC_EOS
! 3049     case 0x12:
! 3050       shift_flags = read_byte(0x0040, 0x17);
.412:
! Debug: list int = const $17 (used reg = )
mov	ax,*$17
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () unsigned char = read_byte+0 (used reg = )
call	_read_byte
add	sp,*4
! Debug: eq unsigned char = al+0 to unsigned char shift_flags = [S+$A-5] (used reg = )
mov	-3[bp],al
!BCC_EOS
! 3051       AX = ((AX & 0xff00) | (shift_flags));
! Debug: and unsigned int = const $FF00 to unsigned short AX = [S+$A+$10] (used reg = )
mov	ax,$12[bp]
xor	al,al
! Debug: or unsigned char shift_flags = [S+$A-5] to unsigned int = ax+0 (used reg = )
or	al,-3[bp]
! Debug: eq unsigned int = ax+0 to unsigned short AX = [S+$A+$10] (used reg = )
mov	$12[bp],ax
!BCC_EOS
! 3052       shift_flags = read_byte(0x0040, 0x18);
! Debug: list int = const $18 (used reg = )
mov	ax,*$18
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () unsigned char = read_byte+0 (used reg = )
call	_read_byte
add	sp,*4
! Debug: eq unsigned char = al+0 to unsigned char shift_flags = [S+$A-5] (used reg = )
mov	-3[bp],al
!BCC_EOS
! 3053       AX = ((AX & 0x00ff) | ((shift_flags) << 8));
! Debug: sl int = const 8 to unsigned char shift_flags = [S+$A-5] (used reg = )
mov	al,-3[bp]
xor	ah,ah
mov	ah,al
xor	al,al
push	ax
! Debug: and int = const $FF to unsigned short AX = [S+$C+$10] (used reg = )
mov	al,$12[bp]
! Debug: or unsigned int (temp) = [S+$C-$C] to unsigned char = al+0 (used reg = )
xor	ah,ah
or	ax,0+..FFF2[bp]
inc	sp
inc	sp
! Debug: eq unsigned int = ax+0 to unsigned short AX = [S+$A+$10] (used reg = )
mov	$12[bp],ax
!BCC_EOS
! 3054       ;
!BCC_EOS
! 3055       break;
br 	.3D4
!BCC_EOS
! 3056     case 0x92:
! 3057       AX = ((AX & 0x00ff) | ((0x80) << 8));
.413:
! Debug: and int = const $FF to unsigned short AX = [S+$A+$10] (used reg = )
mov	al,$12[bp]
! Debug: or int = const -$8000 to unsigned char = al+0 (used reg = )
xor	ah,ah
or	ax,#-$8000
! Debug: eq unsigned int = ax+0 to unsigned short AX = [S+$A+$10] (used reg = )
mov	$12[bp],ax
!BCC_EOS
! 3058       break;
jmp .3D4
!BCC_EOS
! 3059     case 0xA2:
! 3060       break;
.414:
jmp .3D4
!BCC_EOS
! 3061     case 0x6F:
! 3062       if (( AX & 0x00ff ) == 0x08)
.415:
! Debug: and int = const $FF to unsigned short AX = [S+$A+$10] (used reg = )
mov	al,$12[bp]
! Debug: logeq int = const 8 to unsigned char = al+0 (used reg = )
cmp	al,*8
jne 	.416
.417:
! 3063  AX = ((AX & 0x00ff) | ((0x02) << 8));
! Debug: and int = const $FF to unsigned short AX = [S+$A+$10] (used reg = )
mov	al,$12[bp]
! Debug: or int = const $200 to unsigned char = al+0 (used reg = )
xor	ah,ah
or	ax,#$200
! Debug: eq unsigned int = ax+0 to unsigned short AX = [S+$A+$10] (used reg = )
mov	$12[bp],ax
!BCC_EOS
! 3064     default:
.416:
! 3065       bios_printf(4, "KBD: unsupported int 16h function %02x\n", ( AX >> 8 ));
.418:
! Debug: sr int = const 8 to unsigned short AX = [S+$A+$10] (used reg = )
mov	ax,$12[bp]
mov	al,ah
xor	ah,ah
! Debug: list unsigned int = ax+0 (used reg = )
push	ax
! Debug: list * char = .419+0 (used reg = )
mov	bx,#.419
push	bx
! Debug: list int = const 4 (used reg = )
mov	ax,*4
push	ax
! Debug: func () void = bios_printf+0 (used reg = )
call	_bios_printf
add	sp,*6
!BCC_EOS
! 3066     }
! 3067 }
jmp .3D4
.3D6:
sub	ax,*0
jl 	.418
cmp	ax,*$12
ja  	.41A
shl	ax,*1
mov	bx,ax
seg	cs
br	.41B[bx]
.41B:
.word	.3D7
.word	.3E1
.word	.3EA
.word	.418
.word	.418
.word	.3EB
.word	.418
.word	.418
.word	.418
.word	.3EF
.word	.3F0
.word	.418
.word	.418
.word	.418
.word	.418
.word	.418
.word	.405
.word	.40C
.word	.412
.41A:
sub	ax,*$6F
je 	.415
sub	ax,*$23
je 	.413
sub	ax,*$10
je 	.414
jmp	.418
.3D4:
..FFF2	=	-$A
mov	sp,bp
pop	bp
ret
! 3068   unsigned int
! Register BX used in function int16_function
! 3069 dequeue_key(scan_code, ascii_code, incr)
! 3070   Bit8u *scan_code;
export	_dequeue_key
_dequeue_key:
!BCC_EOS
! 3071   Bit8u *ascii_code;
!BCC_EOS
! 3072   unsigned int incr;
!BCC_EOS
! 3073 {
! 3074   Bit16u buffer_start, buffer_end,
! 3074  buffer_head, buffer_tail;
!BCC_EOS
! 3075   Bit16u ss;
!BCC_EOS
! 3076   Bit8u acode, scode;
!BCC_EOS
! 3077   buffer_start = read_word(0x0040, 0x0080);
push	bp
mov	bp,sp
add	sp,*-$C
! Debug: list int = const $80 (used reg = )
mov	ax,#$80
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () unsigned short = read_word+0 (used reg = )
call	_read_word
add	sp,*4
! Debug: eq unsigned short = ax+0 to unsigned short buffer_start = [S+$E-4] (used reg = )
mov	-2[bp],ax
!BCC_EOS
! 3078   buffer_end = read_word(0x0040, 0x0082);
! Debug: list int = const $82 (used reg = )
mov	ax,#$82
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () unsigned short = read_word+0 (used reg = )
call	_read_word
add	sp,*4
! Debug: eq unsigned short = ax+0 to unsigned short buffer_end = [S+$E-6] (used reg = )
mov	-4[bp],ax
!BCC_EOS
! 3079   buffer_head = read_word(0x0040, 0x001a);
! Debug: list int = const $1A (used reg = )
mov	ax,*$1A
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () unsigned short = read_word+0 (used reg = )
call	_read_word
add	sp,*4
! Debug: eq unsigned short = ax+0 to unsigned short buffer_head = [S+$E-8] (used reg = )
mov	-6[bp],ax
!BCC_EOS
! 3080   buffer_tail = read_word(0x0040, 0x001c);
! Debug: list int = const $1C (used reg = )
mov	ax,*$1C
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () unsigned short = read_word+0 (used reg = )
call	_read_word
add	sp,*4
! Debug: eq unsigned short = ax+0 to unsigned short buffer_tail = [S+$E-$A] (used reg = )
mov	-8[bp],ax
!BCC_EOS
! 3081   if (buffer_head != buffer_tail) {
! Debug: ne unsigned short buffer_tail = [S+$E-$A] to unsigned short buffer_head = [S+$E-8] (used reg = )
mov	ax,-6[bp]
cmp	ax,-8[bp]
beq 	.41C
.41D:
! 3082     ss = get_SS();
! Debug: func () unsigned short = get_SS+0 (used reg = )
call	_get_SS
! Debug: eq unsigned short = ax+0 to unsigned short ss = [S+$E-$C] (used reg = )
mov	-$A[bp],ax
!BCC_EOS
! 3083     acode = read_byte(0x0040, buffer_head);
! Debug: list unsigned short buffer_head = [S+$E-8] (used reg = )
push	-6[bp]
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () unsigned char = read_byte+0 (used reg = )
call	_read_byte
add	sp,*4
! Debug: eq unsigned char = al+0 to unsigned char acode = [S+$E-$D] (used reg = )
mov	-$B[bp],al
!BCC_EOS
! 3084     scode = read_byte(0x0040, buffer_head+1);
! Debug: add int = const 1 to unsigned short buffer_head = [S+$E-8] (used reg = )
mov	ax,-6[bp]
! Debug: list unsigned int = ax+1 (used reg = )
inc	ax
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () unsigned char = read_byte+0 (used reg = )
call	_read_byte
add	sp,*4
! Debug: eq unsigned char = al+0 to unsigned char scode = [S+$E-$E] (used reg = )
mov	-$C[bp],al
!BCC_EOS
! 3085     write_byte(ss, ascii_code, acode);
! Debug: list unsigned char acode = [S+$E-$D] (used reg = )
mov	al,-$B[bp]
xor	ah,ah
push	ax
! Debug: list * unsigned char ascii_code = [S+$10+4] (used reg = )
push	6[bp]
! Debug: list unsigned short ss = [S+$12-$C] (used reg = )
push	-$A[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 3086     write_byte(ss, scan_code, scode);
! Debug: list unsigned char scode = [S+$E-$E] (used reg = )
mov	al,-$C[bp]
xor	ah,ah
push	ax
! Debug: list * unsigned char scan_code = [S+$10+2] (used reg = )
push	4[bp]
! Debug: list unsigned short ss = [S+$12-$C] (used reg = )
push	-$A[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 3087     if (incr) {
mov	ax,8[bp]
test	ax,ax
je  	.41E
.41F:
! 3088       buffer_head += 2;
! Debug: addab int = const 2 to unsigned short buffer_head = [S+$E-8] (used reg = )
mov	ax,-6[bp]
inc	ax
inc	ax
mov	-6[bp],ax
!BCC_EOS
! 3089       if (buffer_head >= buffer_end)
! Debug: ge unsigned short buffer_end = [S+$E-6] to unsigned short buffer_head = [S+$E-8] (used reg = )
mov	ax,-6[bp]
cmp	ax,-4[bp]
jb  	.420
.421:
! 3090         buffer_head = buffer_start;
! Debug: eq unsigned short buffer_start = [S+$E-4] to unsigned short buffer_head = [S+$E-8] (used reg = )
mov	ax,-2[bp]
mov	-6[bp],ax
!BCC_EOS
! 3091       write_word(0x0040, 0x001a, buffer_head);
.420:
! Debug: list unsigned short buffer_head = [S+$E-8] (used reg = )
push	-6[bp]
! Debug: list int = const $1A (used reg = )
mov	ax,*$1A
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
add	sp,*6
!BCC_EOS
! 3092       }
! 3093     return(1);
.41E:
mov	ax,*1
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 3094     }
! 3095   else {
jmp .422
.41C:
! 3096     return(0);
xor	ax,ax
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 3097     }
! 3098 }
.422:
mov	sp,bp
pop	bp
ret
! 3099 static char panic_msg_keyb_buffer_full[] = "%s: keyboard input buffer full\n";
.data
_panic_msg_keyb_buffer_full:
.423:
.ascii	"%s: keyboard input buffer full"
.byte	$A
.byte	0
!BCC_EOS
! 3100   Bit8u
! 3101 inhibit_mouse_int_and_events()
! 3102 {
.text
export	_inhibit_mouse_int_and_events
_inhibit_mouse_int_and_events:
! 3103   Bit8u command_byte, prev_command_byte;
!BCC_EOS
! 3104   if ( inb(0x64) & 0x02 )
push	bp
mov	bp,sp
dec	sp
dec	sp
! Debug: list int = const $64 (used reg = )
mov	ax,*$64
push	ax
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
inc	sp
inc	sp
! Debug: and int = const 2 to unsigned char = al+0 (used reg = )
and	al,*2
test	al,al
je  	.424
.425:
! 3105     bios_printf((2 | 4 | 1), panic_msg_keyb_buffer_full,"inhibmouse");
! Debug: list * char = .426+0 (used reg = )
mov	bx,#.426
push	bx
! Debug: list * char = panic_msg_keyb_buffer_full+0 (used reg = )
mov	bx,#_panic_msg_keyb_buffer_full
push	bx
! Debug: list int = const 7 (used reg = )
mov	ax,*7
push	ax
! Debug: func () void = bios_printf+0 (used reg = )
call	_bios_printf
add	sp,*6
!BCC_EOS
! 3106   outb(0x64, 0x20);
.424:
! Debug: list int = const $20 (used reg = )
mov	ax,*$20
push	ax
! Debug: list int = const $64 (used reg = )
mov	ax,*$64
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 3107   while ( (inb(0x64) & 0x01) != 0x01 );
jmp .428
.429:
!BCC_EOS
! 3108   prev_command_byte = inb(0x60);
.428:
! Debug: list int = const $64 (used reg = )
mov	ax,*$64
push	ax
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
inc	sp
inc	sp
! Debug: and int = const 1 to unsigned char = al+0 (used reg = )
and	al,*1
! Debug: ne int = const 1 to unsigned char = al+0 (used reg = )
cmp	al,*1
jne	.429
.42A:
.427:
! Debug: list int = const $60 (used reg = )
mov	ax,*$60
push	ax
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char prev_command_byte = [S+4-4] (used reg = )
mov	-2[bp],al
!BCC_EOS
! 3109   command_byte = prev_command_byte;
! Debug: eq unsigned char prev_command_byte = [S+4-4] to unsigned char command_byte = [S+4-3] (used reg = )
mov	al,-2[bp]
mov	-1[bp],al
!BCC_EOS
! 3110   if ( inb(0x64) & 0x02 )
! Debug: list int = const $64 (used reg = )
mov	ax,*$64
push	ax
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
inc	sp
inc	sp
! Debug: and int = const 2 to unsigned char = al+0 (used reg = )
and	al,*2
test	al,al
je  	.42B
.42C:
! 3111     bios_printf((2 | 4 | 1), panic_msg_keyb_buffer_full,"inhibmouse");
! Debug: list * char = .42D+0 (used reg = )
mov	bx,#.42D
push	bx
! Debug: list * char = panic_msg_keyb_buffer_full+0 (used reg = )
mov	bx,#_panic_msg_keyb_buffer_full
push	bx
! Debug: list int = const 7 (used reg = )
mov	ax,*7
push	ax
! Debug: func () void = bios_printf+0 (used reg = )
call	_bios_printf
add	sp,*6
!BCC_EOS
! 3112   command_byte &= 0xfd;
.42B:
! Debug: andab int = const $FD to unsigned char command_byte = [S+4-3] (used reg = )
mov	al,-1[bp]
and	al,#$FD
mov	-1[bp],al
!BCC_EOS
! 3113   command_byte |= 0x20;
! Debug: orab int = const $20 to unsigned char command_byte = [S+4-3] (used reg = )
mov	al,-1[bp]
or	al,*$20
mov	-1[bp],al
!BCC_EOS
! 3114   outb(0x64, 0x60);
! Debug: list int = const $60 (used reg = )
mov	ax,*$60
push	ax
! Debug: list int = const $64 (used reg = )
mov	ax,*$64
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 3115   outb(0x60, command_byte);
! Debug: list unsigned char command_byte = [S+4-3] (used reg = )
mov	al,-1[bp]
xor	ah,ah
push	ax
! Debug: list int = const $60 (used reg = )
mov	ax,*$60
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 3116   return(prev_command_byte);
mov	al,-2[bp]
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 3117 }
! 3118   void
! Register BX used in function inhibit_mouse_int_and_events
! 3119 enable_mouse_int_and_events()
! 3120 {
export	_enable_mouse_int_and_events
_enable_mouse_int_and_events:
! 3121   Bit8u command_byte;
!BCC_EOS
! 3122   if ( inb(0x64) & 0x02 )
push	bp
mov	bp,sp
dec	sp
dec	sp
! Debug: list int = const $64 (used reg = )
mov	ax,*$64
push	ax
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
inc	sp
inc	sp
! Debug: and int = const 2 to unsigned char = al+0 (used reg = )
and	al,*2
test	al,al
je  	.42E
.42F:
! 3123     bios_printf((2 | 4 | 1), panic_msg_keyb_buffer_full,"enabmouse");
! Debug: list * char = .430+0 (used reg = )
mov	bx,#.430
push	bx
! Debug: list * char = panic_msg_keyb_buffer_full+0 (used reg = )
mov	bx,#_panic_msg_keyb_buffer_full
push	bx
! Debug: list int = const 7 (used reg = )
mov	ax,*7
push	ax
! Debug: func () void = bios_printf+0 (used reg = )
call	_bios_printf
add	sp,*6
!BCC_EOS
! 3124   outb(0x64, 0x20);
.42E:
! Debug: list int = const $20 (used reg = )
mov	ax,*$20
push	ax
! Debug: list int = const $64 (used reg = )
mov	ax,*$64
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 3125   while ( (inb(0x64) & 0x01) != 0x01 );
jmp .432
.433:
!BCC_EOS
! 3126   command_byte = inb(0x60);
.432:
! Debug: list int = const $64 (used reg = )
mov	ax,*$64
push	ax
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
inc	sp
inc	sp
! Debug: and int = const 1 to unsigned char = al+0 (used reg = )
and	al,*1
! Debug: ne int = const 1 to unsigned char = al+0 (used reg = )
cmp	al,*1
jne	.433
.434:
.431:
! Debug: list int = const $60 (used reg = )
mov	ax,*$60
push	ax
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char command_byte = [S+4-3] (used reg = )
mov	-1[bp],al
!BCC_EOS
! 3127   if ( inb(0x64) & 0x02 )
! Debug: list int = const $64 (used reg = )
mov	ax,*$64
push	ax
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
inc	sp
inc	sp
! Debug: and int = const 2 to unsigned char = al+0 (used reg = )
and	al,*2
test	al,al
je  	.435
.436:
! 3128     bios_printf((2 | 4 | 1), panic_msg_keyb_buffer_full,"enabmouse");
! Debug: list * char = .437+0 (used reg = )
mov	bx,#.437
push	bx
! Debug: list * char = panic_msg_keyb_buffer_full+0 (used reg = )
mov	bx,#_panic_msg_keyb_buffer_full
push	bx
! Debug: list int = const 7 (used reg = )
mov	ax,*7
push	ax
! Debug: func () void = bios_printf+0 (used reg = )
call	_bios_printf
add	sp,*6
!BCC_EOS
! 3129   command_byte |= 0x02;
.435:
! Debug: orab int = const 2 to unsigned char command_byte = [S+4-3] (used reg = )
mov	al,-1[bp]
or	al,*2
mov	-1[bp],al
!BCC_EOS
! 3130   command_byte &= 0xdf;
! Debug: andab int = const $DF to unsigned char command_byte = [S+4-3] (used reg = )
mov	al,-1[bp]
and	al,#$DF
mov	-1[bp],al
!BCC_EOS
! 3131   outb(0x64, 0x60);
! Debug: list int = const $60 (used reg = )
mov	ax,*$60
push	ax
! Debug: list int = const $64 (used reg = )
mov	ax,*$64
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 3132   outb(0x60, command_byte);
! Debug: list unsigned char command_byte = [S+4-3] (used reg = )
mov	al,-1[bp]
xor	ah,ah
push	ax
! Debug: list int = const $60 (used reg = )
mov	ax,*$60
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 3133 }
mov	sp,bp
pop	bp
ret
! 3134   Bit8u
! Register BX used in function enable_mouse_int_and_events
! 3135 send_to_mouse_ctrl(sendbyte)
! 3136   Bit8u sendbyte;
export	_send_to_mouse_ctrl
_send_to_mouse_ctrl:
!BCC_EOS
! 3137 {
! 3138   Bit8u response;
!BCC_EOS
! 3139   if ( inb(0x64) & 0x02 )
push	bp
mov	bp,sp
dec	sp
dec	sp
! Debug: list int = const $64 (used reg = )
mov	ax,*$64
push	ax
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
inc	sp
inc	sp
! Debug: and int = const 2 to unsigned char = al+0 (used reg = )
and	al,*2
test	al,al
je  	.438
.439:
! 3140     bios_printf((2 | 4 | 1), panic_msg_keyb_buffer_full,"sendmouse");
! Debug: list * char = .43A+0 (used reg = )
mov	bx,#.43A
push	bx
! Debug: list * char = panic_msg_keyb_buffer_full+0 (used reg = )
mov	bx,#_panic_msg_keyb_buffer_full
push	bx
! Debug: list int = const 7 (used reg = )
mov	ax,*7
push	ax
! Debug: func () void = bios_printf+0 (used reg = )
call	_bios_printf
add	sp,*6
!BCC_EOS
! 3141   outb(0x64, 0xD4);
.438:
! Debug: list int = const $D4 (used reg = )
mov	ax,#$D4
push	ax
! Debug: list int = const $64 (used reg = )
mov	ax,*$64
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 3142   outb(0x60, sendbyte);
! Debug: list unsigned char sendbyte = [S+4+2] (used reg = )
mov	al,4[bp]
xor	ah,ah
push	ax
! Debug: list int = const $60 (used reg = )
mov	ax,*$60
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 3143   return(0);
xor	al,al
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 3144 }
! 3145   Bit8u
! Register BX used in function send_to_mouse_ctrl
! 3146 get_mouse_data(data)
! 3147   Bit8u *data;
export	_get_mouse_data
_get_mouse_data:
!BCC_EOS
! 3148 {
! 3149   Bit8u response;
!BCC_EOS
! 3150   Bit16u ss;
!BCC_EOS
! 3151   while ( (inb
push	bp
mov	bp,sp
add	sp,*-4
! 3151 (0x64) & 0x21) != 0x21 ) {
jmp .43C
.43D:
! 3152     }
! 3153   response = inb(0x60);
.43C:
! Debug: list int = const $64 (used reg = )
mov	ax,*$64
push	ax
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
inc	sp
inc	sp
! Debug: and int = const $21 to unsigned char = al+0 (used reg = )
and	al,*$21
! Debug: ne int = const $21 to unsigned char = al+0 (used reg = )
cmp	al,*$21
jne	.43D
.43E:
.43B:
! Debug: list int = const $60 (used reg = )
mov	ax,*$60
push	ax
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char response = [S+6-3] (used reg = )
mov	-1[bp],al
!BCC_EOS
! 3154   ss = get_SS();
! Debug: func () unsigned short = get_SS+0 (used reg = )
call	_get_SS
! Debug: eq unsigned short = ax+0 to unsigned short ss = [S+6-6] (used reg = )
mov	-4[bp],ax
!BCC_EOS
! 3155   write_byte(ss, data, response);
! Debug: list unsigned char response = [S+6-3] (used reg = )
mov	al,-1[bp]
xor	ah,ah
push	ax
! Debug: list * unsigned char data = [S+8+2] (used reg = )
push	4[bp]
! Debug: list unsigned short ss = [S+$A-6] (used reg = )
push	-4[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 3156   return(0);
xor	al,al
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 3157 }
! 3158   void
! 3159 set_kbd_command_byte(command_byte)
! 3160   Bit8u command_byte;
export	_set_kbd_command_byte
_set_kbd_command_byte:
!BCC_EOS
! 3161 {
! 3162   if ( inb(0x64) & 0x02 )
push	bp
mov	bp,sp
! Debug: list int = const $64 (used reg = )
mov	ax,*$64
push	ax
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
mov	sp,bp
! Debug: and int = const 2 to unsigned char = al+0 (used reg = )
and	al,*2
test	al,al
je  	.43F
.440:
! 3163     bios_printf((2 | 4 | 1), panic_msg_keyb_buffer_full,"setkbdcomm");
! Debug: list * char = .441+0 (used reg = )
mov	bx,#.441
push	bx
! Debug: list * char = panic_msg_keyb_buffer_full+0 (used reg = )
mov	bx,#_panic_msg_keyb_buffer_full
push	bx
! Debug: list int = const 7 (used reg = )
mov	ax,*7
push	ax
! Debug: func () void = bios_printf+0 (used reg = )
call	_bios_printf
mov	sp,bp
!BCC_EOS
! 3164   outb(0x64, 0xD4);
.43F:
! Debug: list int = const $D4 (used reg = )
mov	ax,#$D4
push	ax
! Debug: list int = const $64 (used reg = )
mov	ax,*$64
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
mov	sp,bp
!BCC_EOS
! 3165   outb(0x64, 0x60);
! Debug: list int = const $60 (used reg = )
mov	ax,*$60
push	ax
! Debug: list int = const $64 (used reg = )
mov	ax,*$64
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
mov	sp,bp
!BCC_EOS
! 3166   outb(0x60, command_byte);
! Debug: list unsigned char command_byte = [S+2+2] (used reg = )
mov	al,4[bp]
xor	ah,ah
push	ax
! Debug: list int = const $60 (used reg = )
mov	ax,*$60
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
mov	sp,bp
!BCC_EOS
! 3167 }
pop	bp
ret
! 3168   void
! Register BX used in function set_kbd_command_byte
! 3169 int09_function(DI, SI, BP, SP, BX, DX, CX, AX)
! 3170   Bit16u DI, SI, BP, SP, BX, DX, CX, AX;
export	_int09_function
_int09_function:
!BCC_EOS
! 3171 {
! 3172   Bit8u scancode, asciicode, shift_flags;
!BCC_EOS
! 3173   Bit8u mf2_flags, mf2_state, led_flags;
!BCC_EOS
! 3174   scancode = ( AX & 0x00ff );
push	bp
mov	bp,sp
add	sp,*-6
! Debug: and int = const $FF to unsigned short AX = [S+8+$10] (used reg = )
mov	al,$12[bp]
! Debug: eq unsigned char = al+0 to unsigned char scancode = [S+8-3] (used reg = )
mov	-1[bp],al
!BCC_EOS
! 3175   if (scancode == 0) {
! Debug: logeq int = const 0 to unsigned char scancode = [S+8-3] (used reg = )
mov	al,-1[bp]
test	al,al
jne 	.442
.443:
! 3176     bios_printf(4, "KBD: int09 handler: AL=0\n");
! Debug: list * char = .444+0 (used reg = )
mov	bx,#.444
push	bx
! Debug: list int = const 4 (used reg = )
mov	ax,*4
push	ax
! Debug: func () void = bios_printf+0 (used reg = )
call	_bios_printf
add	sp,*4
!BCC_EOS
! 3177     return;
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 3178     }
! 3179   shift_flags = read_byte(0x0040, 0x17);
.442:
! Debug: list int = const $17 (used reg = )
mov	ax,*$17
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () unsigned char = read_byte+0 (used reg = )
call	_read_byte
add	sp,*4
! Debug: eq unsigned char = al+0 to unsigned char shift_flags = [S+8-5] (used reg = )
mov	-3[bp],al
!BCC_EOS
! 3180   mf2_flags = read_byte(0x0040, 0x18);
! Debug: list int = const $18 (used reg = )
mov	ax,*$18
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () unsigned char = read_byte+0 (used reg = )
call	_read_byte
add	sp,*4
! Debug: eq unsigned char = al+0 to unsigned char mf2_flags = [S+8-6] (used reg = )
mov	-4[bp],al
!BCC_EOS
! 3181   mf2_state = read_byte(0x0040, 0x96);
! Debug: list int = const $96 (used reg = )
mov	ax,#$96
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () unsigned char = read_byte+0 (used reg = )
call	_read_byte
add	sp,*4
! Debug: eq unsigned char = al+0 to unsigned char mf2_state = [S+8-7] (used reg = )
mov	-5[bp],al
!BCC_EOS
! 3182   led_flags = read_byte(0x0040, 0x97);
! Debug: list int = const $97 (used reg = )
mov	ax,#$97
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () unsigned char = read_byte+0 (used reg = )
call	_read_byte
add	sp,*4
! Debug: eq unsigned char = al+0 to unsigned char led_flags = [S+8-8] (used reg = )
mov	-6[bp],al
!BCC_EOS
! 3183   asciicode = 0;
! Debug: eq int = const 0 to unsigned char asciicode = [S+8-4] (used reg = )
xor	al,al
mov	-2[bp],al
!BCC_EOS
! 3184   switch (scancode) {
mov	al,-1[bp]
br 	.447
! 3185     case 0x3a:
! 3186       shift_flags ^= 0x40;
.448:
! Debug: eorab int = const $40 to unsigned char shift_flags = [S+8-5] (used reg = )
mov	al,-3[bp]
xor	al,*$40
mov	-3[bp],al
!BCC_EOS
! 3187       write_byte(0x0040, 0x17, shift_flags);
! Debug: list unsigned char shift_flags = [S+8-5] (used reg = )
mov	al,-3[bp]
xor	ah,ah
push	ax
! Debug: list int = const $17 (used reg = )
mov	ax,*$17
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 3188       mf2_flags |= 0x40;
! Debug: orab int = const $40 to unsigned char mf2_flags = [S+8-6] (used reg = )
mov	al,-4[bp]
or	al,*$40
mov	-4[bp],al
!BCC_EOS
! 3189       write_byte(0x0040, 0x18, mf2_flags);
! Debug: list unsigned char mf2_flags = [S+8-6] (used reg = )
mov	al,-4[bp]
xor	ah,ah
push	ax
! Debug: list int = const $18 (used reg = )
mov	ax,*$18
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 3190       led_flags ^= 0x04;
! Debug: eorab int = const 4 to unsigned char led_flags = [S+8-8] (used reg = )
mov	al,-6[bp]
xor	al,*4
mov	-6[bp],al
!BCC_EOS
! 3191       write_byte(0x0040, 0x97, led_flags);
! Debug: list unsigned char led_flags = [S+8-8] (used reg = )
mov	al,-6[bp]
xor	ah,ah
push	ax
! Debug: list int = const $97 (used reg = )
mov	ax,#$97
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 3192       break;
br 	.445
!BCC_EOS
! 3193     case 0xba:
! 3194       mf2_flags &= ~0x40;
.449:
! Debug: andab int = const -$41 to unsigned char mf2_flags = [S+8-6] (used reg = )
mov	al,-4[bp]
and	al,#$BF
mov	-4[bp],al
!BCC_EOS
! 3195       write_byte(0x0040, 0x18, mf2_flags);
! Debug: list unsigned char mf2_flags = [S+8-6] (used reg = )
mov	al,-4[bp]
xor	ah,ah
push	ax
! Debug: list int = const $18 (used reg = )
mov	ax,*$18
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 3196       break;
br 	.445
!BCC_EOS
! 3197     case 0x2a:
! 3198       shift_flags |= 0x02;
.44A:
! Debug: orab int = const 2 to unsigned char shift_flags = [S+8-5] (used reg = )
mov	al,-3[bp]
or	al,*2
mov	-3[bp],al
!BCC_EOS
! 3199       write_byte(0x0040, 0x17, shift_flags);
! Debug: list unsigned char shift_flags = [S+8-5] (used reg = )
mov	al,-3[bp]
xor	ah,ah
push	ax
! Debug: list int = const $17 (used reg = )
mov	ax,*$17
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 3200       led_flags &= ~0x04;
! Debug: andab int = const -5 to unsigned char led_flags = [S+8-8] (used reg = )
mov	al,-6[bp]
and	al,#$FB
mov	-6[bp],al
!BCC_EOS
! 3201       write_byte(0x0040, 0x97, led_flags);
! Debug: list unsigned char led_flags = [S+8-8] (used reg = )
mov	al,-6[bp]
xor	ah,ah
push	ax
! Debug: list int = const $97 (used reg = )
mov	ax,#$97
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 3202       break;
br 	.445
!BCC_EOS
! 3203     case 0xaa:
! 3204       shift_flags &= ~0x02;
.44B:
! Debug: andab int = const -3 to unsigned char shift_flags = [S+8-5] (used reg = )
mov	al,-3[bp]
and	al,#$FD
mov	-3[bp],al
!BCC_EOS
! 3205       write_byte(0x0040, 0x17, shift_flags);
! Debug: list unsigned char shift_flags = [S+8-5] (used reg = )
mov	al,-3[bp]
xor	ah,ah
push	ax
! Debug: list int = const $17 (used reg = )
mov	ax,*$17
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 3206       break;
br 	.445
!BCC_EOS
! 3207     case 0x36:
! 3208       shift_flags |= 0x01;
.44C:
! Debug: orab int = const 1 to unsigned char shift_flags = [S+8-5] (used reg = )
mov	al,-3[bp]
or	al,*1
mov	-3[bp],al
!BCC_EOS
! 3209       write_byte(0x0040, 0x17, shift_flags);
! Debug: list unsigned char shift_flags = [S+8-5] (used reg = )
mov	al,-3[bp]
xor	ah,ah
push	ax
! Debug: list int = const $17 (used reg = )
mov	ax,*$17
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 3210       led_flags &= ~0x04;
! Debug: andab int = const -5 to unsigned char led_flags = [S+8-8] (used reg = )
mov	al,-6[bp]
and	al,#$FB
mov	-6[bp],al
!BCC_EOS
! 3211       write_byte(0x0040, 0x97, led_flags);
! Debug: list unsigned char led_flags = [S+8-8] (used reg = )
mov	al,-6[bp]
xor	ah,ah
push	ax
! Debug: list int = const $97 (used reg = )
mov	ax,#$97
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 3212       break;
br 	.445
!BCC_EOS
! 3213     case 0xb6:
! 3214       shift_flags &= ~0x01;
.44D:
! Debug: andab int = const -2 to unsigned char shift_flags = [S+8-5] (used reg = )
mov	al,-3[bp]
and	al,#$FE
mov	-3[bp],al
!BCC_EOS
! 3215       write_byte(0x0040, 0x17, shift_flags);
! Debug: list unsigned char shift_flags = [S+8-5] (used reg = )
mov	al,-3[bp]
xor	ah,ah
push	ax
! Debug: list int = const $17 (used reg = )
mov	ax,*$17
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 3216       break;
br 	.445
!BCC_EOS
! 3217     case 0x1d:
! 3218       shift_flags |= 0x04;
.44E:
! Debug: orab int = const 4 to unsigned char shift_flags = [S+8-5] (used reg = )
mov	al,-3[bp]
or	al,*4
mov	-3[bp],al
!BCC_EOS
! 3219       write_byte(0x0040, 0x17, shift_flags);
! Debug: list unsigned char shift_flags = [S+8-5] (used reg = )
mov	al,-3[bp]
xor	ah,ah
push	ax
! Debug: list int = const $17 (used reg = )
mov	ax,*$17
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 3220       if (mf2_state & 0x01) {
! Debug: and int = const 1 to unsigned char mf2_state = [S+8-7] (used reg = )
mov	al,-5[bp]
and	al,*1
test	al,al
je  	.44F
.450:
! 3221         mf2_flags |= 0x04;
! Debug: orab int = const 4 to unsigned char mf2_flags = [S+8-6] (used reg = )
mov	al,-4[bp]
or	al,*4
mov	-4[bp],al
!BCC_EOS
! 3222       } else {
jmp .451
.44F:
! 3223         mf2_flags |= 0x01;
! Debug: orab int = const 1 to unsigned char mf2_flags = [S+8-6] (used reg = )
mov	al,-4[bp]
or	al,*1
mov	-4[bp],al
!BCC_EOS
! 3224         }
! 3225       write_byte(0x0040, 0x18, mf2_flags);
.451:
! Debug: list unsigned char mf2_flags = [S+8-6] (used reg = )
mov	al,-4[bp]
xor	ah,ah
push	ax
! Debug: list int = const $18 (used reg = )
mov	ax,*$18
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 3226       break;
br 	.445
!BCC_EOS
! 3227     case 0x9d:
! 3228       shift_flags &= ~0x04;
.452:
! Debug: andab int = const -5 to unsigned char shift_flags = [S+8-5] (used reg = )
mov	al,-3[bp]
and	al,#$FB
mov	-3[bp],al
!BCC_EOS
! 3229       write_byte(0x0040, 0
! 3229 x17, shift_flags);
! Debug: list unsigned char shift_flags = [S+8-5] (used reg = )
mov	al,-3[bp]
xor	ah,ah
push	ax
! Debug: list int = const $17 (used reg = )
mov	ax,*$17
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 3230       if (mf2_state & 0x01) {
! Debug: and int = const 1 to unsigned char mf2_state = [S+8-7] (used reg = )
mov	al,-5[bp]
and	al,*1
test	al,al
je  	.453
.454:
! 3231         mf2_flags &= ~0x04;
! Debug: andab int = const -5 to unsigned char mf2_flags = [S+8-6] (used reg = )
mov	al,-4[bp]
and	al,#$FB
mov	-4[bp],al
!BCC_EOS
! 3232       } else {
jmp .455
.453:
! 3233         mf2_flags &= ~0x01;
! Debug: andab int = const -2 to unsigned char mf2_flags = [S+8-6] (used reg = )
mov	al,-4[bp]
and	al,#$FE
mov	-4[bp],al
!BCC_EOS
! 3234         }
! 3235       write_byte(0x0040, 0x18, mf2_flags);
.455:
! Debug: list unsigned char mf2_flags = [S+8-6] (used reg = )
mov	al,-4[bp]
xor	ah,ah
push	ax
! Debug: list int = const $18 (used reg = )
mov	ax,*$18
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 3236       break;
br 	.445
!BCC_EOS
! 3237     case 0x38:
! 3238       shift_flags |= 0x08;
.456:
! Debug: orab int = const 8 to unsigned char shift_flags = [S+8-5] (used reg = )
mov	al,-3[bp]
or	al,*8
mov	-3[bp],al
!BCC_EOS
! 3239       write_byte(0x0040, 0x17, shift_flags);
! Debug: list unsigned char shift_flags = [S+8-5] (used reg = )
mov	al,-3[bp]
xor	ah,ah
push	ax
! Debug: list int = const $17 (used reg = )
mov	ax,*$17
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 3240       if (mf2_state & 0x01) {
! Debug: and int = const 1 to unsigned char mf2_state = [S+8-7] (used reg = )
mov	al,-5[bp]
and	al,*1
test	al,al
je  	.457
.458:
! 3241         mf2_flags |= 0x08;
! Debug: orab int = const 8 to unsigned char mf2_flags = [S+8-6] (used reg = )
mov	al,-4[bp]
or	al,*8
mov	-4[bp],al
!BCC_EOS
! 3242       } else {
jmp .459
.457:
! 3243         mf2_flags |= 0x02;
! Debug: orab int = const 2 to unsigned char mf2_flags = [S+8-6] (used reg = )
mov	al,-4[bp]
or	al,*2
mov	-4[bp],al
!BCC_EOS
! 3244         }
! 3245       write_byte(0x0040, 0x18, mf2_flags);
.459:
! Debug: list unsigned char mf2_flags = [S+8-6] (used reg = )
mov	al,-4[bp]
xor	ah,ah
push	ax
! Debug: list int = const $18 (used reg = )
mov	ax,*$18
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 3246       break;
br 	.445
!BCC_EOS
! 3247     case 0xb8:
! 3248       shift_flags &= ~0x08;
.45A:
! Debug: andab int = const -9 to unsigned char shift_flags = [S+8-5] (used reg = )
mov	al,-3[bp]
and	al,#$F7
mov	-3[bp],al
!BCC_EOS
! 3249       write_byte(0x0040, 0x17, shift_flags);
! Debug: list unsigned char shift_flags = [S+8-5] (used reg = )
mov	al,-3[bp]
xor	ah,ah
push	ax
! Debug: list int = const $17 (used reg = )
mov	ax,*$17
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 3250       if (mf2_state & 0x01) {
! Debug: and int = const 1 to unsigned char mf2_state = [S+8-7] (used reg = )
mov	al,-5[bp]
and	al,*1
test	al,al
je  	.45B
.45C:
! 3251         mf2_flags &= ~0x08;
! Debug: andab int = const -9 to unsigned char mf2_flags = [S+8-6] (used reg = )
mov	al,-4[bp]
and	al,#$F7
mov	-4[bp],al
!BCC_EOS
! 3252       } else {
jmp .45D
.45B:
! 3253         mf2_flags &= ~0x02;
! Debug: andab int = const -3 to unsigned char mf2_flags = [S+8-6] (used reg = )
mov	al,-4[bp]
and	al,#$FD
mov	-4[bp],al
!BCC_EOS
! 3254         }
! 3255       write_byte(0x0040, 0x18, mf2_flags);
.45D:
! Debug: list unsigned char mf2_flags = [S+8-6] (used reg = )
mov	al,-4[bp]
xor	ah,ah
push	ax
! Debug: list int = const $18 (used reg = )
mov	ax,*$18
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 3256       break;
br 	.445
!BCC_EOS
! 3257     case 0x45:
! 3258       if ((mf2_state & 0x01) == 0) {
.45E:
! Debug: and int = const 1 to unsigned char mf2_state = [S+8-7] (used reg = )
mov	al,-5[bp]
and	al,*1
! Debug: logeq int = const 0 to unsigned char = al+0 (used reg = )
test	al,al
jne 	.45F
.460:
! 3259         mf2_flags |= 0x20;
! Debug: orab int = const $20 to unsigned char mf2_flags = [S+8-6] (used reg = )
mov	al,-4[bp]
or	al,*$20
mov	-4[bp],al
!BCC_EOS
! 3260         write_byte(0x0040, 0x18, mf2_flags);
! Debug: list unsigned char mf2_flags = [S+8-6] (used reg = )
mov	al,-4[bp]
xor	ah,ah
push	ax
! Debug: list int = const $18 (used reg = )
mov	ax,*$18
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 3261         shift_flags ^= 0x20;
! Debug: eorab int = const $20 to unsigned char shift_flags = [S+8-5] (used reg = )
mov	al,-3[bp]
xor	al,*$20
mov	-3[bp],al
!BCC_EOS
! 3262         led_flags ^= 0x02;
! Debug: eorab int = const 2 to unsigned char led_flags = [S+8-8] (used reg = )
mov	al,-6[bp]
xor	al,*2
mov	-6[bp],al
!BCC_EOS
! 3263         write_byte(0x0040, 0x17, shift_flags);
! Debug: list unsigned char shift_flags = [S+8-5] (used reg = )
mov	al,-3[bp]
xor	ah,ah
push	ax
! Debug: list int = const $17 (used reg = )
mov	ax,*$17
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 3264         write_byte(0x0040, 0x97, led_flags);
! Debug: list unsigned char led_flags = [S+8-8] (used reg = )
mov	al,-6[bp]
xor	ah,ah
push	ax
! Debug: list int = const $97 (used reg = )
mov	ax,#$97
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 3265         }
! 3266       break;
.45F:
br 	.445
!BCC_EOS
! 3267     case 0xc5:
! 3268       if ((mf2_state & 0x01) == 0) {
.461:
! Debug: and int = const 1 to unsigned char mf2_state = [S+8-7] (used reg = )
mov	al,-5[bp]
and	al,*1
! Debug: logeq int = const 0 to unsigned char = al+0 (used reg = )
test	al,al
jne 	.462
.463:
! 3269         mf2_flags &= ~0x20;
! Debug: andab int = const -$21 to unsigned char mf2_flags = [S+8-6] (used reg = )
mov	al,-4[bp]
and	al,#$DF
mov	-4[bp],al
!BCC_EOS
! 3270         write_byte(0x0040, 0x18, mf2_flags);
! Debug: list unsigned char mf2_flags = [S+8-6] (used reg = )
mov	al,-4[bp]
xor	ah,ah
push	ax
! Debug: list int = const $18 (used reg = )
mov	ax,*$18
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 3271         }
! 3272       break;
.462:
br 	.445
!BCC_EOS
! 3273     case 0x46:
! 3274       mf2_flags |= 0x10;
.464:
! Debug: orab int = const $10 to unsigned char mf2_flags = [S+8-6] (used reg = )
mov	al,-4[bp]
or	al,*$10
mov	-4[bp],al
!BCC_EOS
! 3275       write_byte(0x0040, 0x18, mf2_flags);
! Debug: list unsigned char mf2_flags = [S+8-6] (used reg = )
mov	al,-4[bp]
xor	ah,ah
push	ax
! Debug: list int = const $18 (used reg = )
mov	ax,*$18
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 3276       shift_flags ^= 0x10;
! Debug: eorab int = const $10 to unsigned char shift_flags = [S+8-5] (used reg = )
mov	al,-3[bp]
xor	al,*$10
mov	-3[bp],al
!BCC_EOS
! 3277       led_flags ^= 0x01;
! Debug: eorab int = const 1 to unsigned char led_flags = [S+8-8] (used reg = )
mov	al,-6[bp]
xor	al,*1
mov	-6[bp],al
!BCC_EOS
! 3278       write_byte(0x0040, 0x17, shift_flags);
! Debug: list unsigned char shift_flags = [S+8-5] (used reg = )
mov	al,-3[bp]
xor	ah,ah
push	ax
! Debug: list int = const $17 (used reg = )
mov	ax,*$17
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 3279       write_byte(0x0040, 0x97, led_flags);
! Debug: list unsigned char led_flags = [S+8-8] (used reg = )
mov	al,-6[bp]
xor	ah,ah
push	ax
! Debug: list int = const $97 (used reg = )
mov	ax,#$97
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 3280       break;
br 	.445
!BCC_EOS
! 3281     case 0xc6:
! 3282       mf2_flags &= ~0x10;
.465:
! Debug: andab int = const -$11 to unsigned char mf2_flags = [S+8-6] (used reg = )
mov	al,-4[bp]
and	al,#$EF
mov	-4[bp],al
!BCC_EOS
! 3283       write_byte(0x0040, 0x18, mf2_flags);
! Debug: list unsigned char mf2_flags = [S+8-6] (used reg = )
mov	al,-4[bp]
xor	ah,ah
push	ax
! Debug: list int = const $18 (used reg = )
mov	ax,*$18
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 3284       break;
br 	.445
!BCC_EOS
! 3285     case 0x53:
! 3286         if ((shift_flags & 0x0c) == 0x0c)
.466:
! Debug: and int = const $C to unsigned char shift_flags = [S+8-5] (used reg = )
mov	al,-3[bp]
and	al,*$C
! Debug: logeq int = const $C to unsigned char = al+0 (used reg = )
cmp	al,*$C
jne 	.467
.468:
! 3287             machine_reset();
! Debug: func () void = machine_reset+0 (used reg = )
call	_machine_reset
!BCC_EOS
! 3288     default:
.467:
! 3289       if (scancode & 0x80) return;
.469:
! Debug: and int = const $80 to unsigned char scancode = [S+8-3] (used reg = )
mov	al,-1[bp]
and	al,#$80
test	al,al
je  	.46A
.46B:
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 3290       if (scancode > 0x58) {
.46A:
! Debug: gt int = const $58 to unsigned char scancode = [S+8-3] (used reg = )
mov	al,-1[bp]
cmp	al,*$58
jbe 	.46C
.46D:
! 3291         bios_printf(4, "KBD: int09h_handler(): unknown scancode (%x) read!\n", scancode);
! Debug: list unsigned char scancode = [S+8-3] (used reg = )
mov	al,-1[bp]
xor	ah,ah
push	ax
! Debug: list * char = .46E+0 (used reg = )
mov	bx,#.46E
push	bx
! Debug: list int = const 4 (used reg = )
mov	ax,*4
push	ax
! Debug: func () void = bios_printf+0 (used reg = )
call	_bios_printf
add	sp,*6
!BCC_EOS
! 3292         return;
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 3293         }
! 3294       if (shift_flags & 0x08) {
.46C:
! Debug: and int = const 8 to unsigned char shift_flags = [S+8-5] (used reg = )
mov	al,-3[bp]
and	al,*8
test	al,al
je  	.46F
.470:
! 3295         asciicode = scan_to_scanascii[scancode].alt;
! Debug: ptradd unsigned char scancode = [S+8-3] to [$59] struct  = scan_to_scanascii+0 (used reg = )
mov	al,-1[bp]
xor	ah,ah
mov	dx,ax
shl	ax,*1
shl	ax,*1
add	ax,dx
shl	ax,*1
mov	bx,ax
add	bx,#_scan_to_scanascii
! Debug: eq unsigned short = [bx+6] to unsigned char asciicode = [S+8-4] (used reg = )
mov	al,6[bx]
mov	-2[bp],al
!BCC_EOS
! 3296         scancode = scan_to_scanascii[scancode].alt >> 8;
! Debug: ptradd unsigned char scancode = [S+8-3] to [$59] struct  = scan_to_scanascii+0 (used reg = )
mov	al,-1[bp]
xor	ah,ah
mov	dx,ax
shl	ax,*1
shl	ax,*1
add	ax,dx
shl	ax,*1
mov	bx,ax
add	bx,#_scan_to_scanascii
! Debug: sr int = const 8 to unsigned short = [bx+6] (used reg = )
mov	ax,6[bx]
mov	al,ah
xor	ah,ah
! Debug: eq unsigned int = ax+0 to unsigned char scancode = [S+8-3] (used reg = )
mov	-1[bp],al
!BCC_EOS
! 3297         }
! 3298       else if (shift_flags & 0x04) {
br 	.471
.46F:
! Debug: and int = const 4 to unsigned char shift_flags = [S+8-5] (used reg = )
mov	al,-3[bp]
and	al,*4
test	al,al
je  	.472
.473:
! 3299         asciicode = scan_to_scanascii[scancode].control;
! Debug: ptradd unsigned char scancode = [S+8-3] to [$59] struct  = scan_to_scanascii+0 (used reg = )
mov	al,-1[bp]
xor	ah,ah
mov	dx,ax
shl	ax,*1
shl	ax,*1
add	ax,dx
shl	ax,*1
mov	bx,ax
add	bx,#_scan_to_scanascii
! Debug: eq unsigned short = [bx+4] to unsigned char asciicode = [S+8-4] (used reg = )
mov	al,4[bx]
mov	-2[bp],al
!BCC_EOS
! 3300         scancode = scan_to_scanascii[scancode].control
! 3300  >> 8;
! Debug: ptradd unsigned char scancode = [S+8-3] to [$59] struct  = scan_to_scanascii+0 (used reg = )
mov	al,-1[bp]
xor	ah,ah
mov	dx,ax
shl	ax,*1
shl	ax,*1
add	ax,dx
shl	ax,*1
mov	bx,ax
add	bx,#_scan_to_scanascii
! Debug: sr int = const 8 to unsigned short = [bx+4] (used reg = )
mov	ax,4[bx]
mov	al,ah
xor	ah,ah
! Debug: eq unsigned int = ax+0 to unsigned char scancode = [S+8-3] (used reg = )
mov	-1[bp],al
!BCC_EOS
! 3301         }
! 3302       else if (shift_flags & 0x03) {
br 	.474
.472:
! Debug: and int = const 3 to unsigned char shift_flags = [S+8-5] (used reg = )
mov	al,-3[bp]
and	al,*3
test	al,al
beq 	.475
.476:
! 3303         if (shift_flags & scan_to_scanascii[scancode].lock_flags) {
! Debug: ptradd unsigned char scancode = [S+8-3] to [$59] struct  = scan_to_scanascii+0 (used reg = )
mov	al,-1[bp]
xor	ah,ah
mov	dx,ax
shl	ax,*1
shl	ax,*1
add	ax,dx
shl	ax,*1
mov	bx,ax
add	bx,#_scan_to_scanascii
! Debug: and unsigned char = [bx+8] to unsigned char shift_flags = [S+8-5] (used reg = )
mov	al,-3[bp]
and	al,8[bx]
test	al,al
je  	.477
.478:
! 3304           asciicode = scan_to_scanascii[scancode].normal;
! Debug: ptradd unsigned char scancode = [S+8-3] to [$59] struct  = scan_to_scanascii+0 (used reg = )
mov	al,-1[bp]
xor	ah,ah
mov	dx,ax
shl	ax,*1
shl	ax,*1
add	ax,dx
shl	ax,*1
mov	bx,ax
! Debug: eq unsigned short = [bx+_scan_to_scanascii+0] to unsigned char asciicode = [S+8-4] (used reg = )
mov	al,_scan_to_scanascii[bx]
mov	-2[bp],al
!BCC_EOS
! 3305           scancode = scan_to_scanascii[scancode].normal >> 8;
! Debug: ptradd unsigned char scancode = [S+8-3] to [$59] struct  = scan_to_scanascii+0 (used reg = )
mov	al,-1[bp]
xor	ah,ah
mov	dx,ax
shl	ax,*1
shl	ax,*1
add	ax,dx
shl	ax,*1
mov	bx,ax
! Debug: sr int = const 8 to unsigned short = [bx+_scan_to_scanascii+0] (used reg = )
mov	ax,_scan_to_scanascii[bx]
mov	al,ah
xor	ah,ah
! Debug: eq unsigned int = ax+0 to unsigned char scancode = [S+8-3] (used reg = )
mov	-1[bp],al
!BCC_EOS
! 3306           }
! 3307         else {
jmp .479
.477:
! 3308           asciicode = scan_to_scanascii[scancode].shift;
! Debug: ptradd unsigned char scancode = [S+8-3] to [$59] struct  = scan_to_scanascii+0 (used reg = )
mov	al,-1[bp]
xor	ah,ah
mov	dx,ax
shl	ax,*1
shl	ax,*1
add	ax,dx
shl	ax,*1
mov	bx,ax
add	bx,#_scan_to_scanascii
! Debug: eq unsigned short = [bx+2] to unsigned char asciicode = [S+8-4] (used reg = )
mov	al,2[bx]
mov	-2[bp],al
!BCC_EOS
! 3309           scancode = scan_to_scanascii[scancode].shift >> 8;
! Debug: ptradd unsigned char scancode = [S+8-3] to [$59] struct  = scan_to_scanascii+0 (used reg = )
mov	al,-1[bp]
xor	ah,ah
mov	dx,ax
shl	ax,*1
shl	ax,*1
add	ax,dx
shl	ax,*1
mov	bx,ax
add	bx,#_scan_to_scanascii
! Debug: sr int = const 8 to unsigned short = [bx+2] (used reg = )
mov	ax,2[bx]
mov	al,ah
xor	ah,ah
! Debug: eq unsigned int = ax+0 to unsigned char scancode = [S+8-3] (used reg = )
mov	-1[bp],al
!BCC_EOS
! 3310           }
! 3311         }
.479:
! 3312       else {
br 	.47A
.475:
! 3313         if (shift_flags & scan_to_scanascii[scancode].lock_flags) {
! Debug: ptradd unsigned char scancode = [S+8-3] to [$59] struct  = scan_to_scanascii+0 (used reg = )
mov	al,-1[bp]
xor	ah,ah
mov	dx,ax
shl	ax,*1
shl	ax,*1
add	ax,dx
shl	ax,*1
mov	bx,ax
add	bx,#_scan_to_scanascii
! Debug: and unsigned char = [bx+8] to unsigned char shift_flags = [S+8-5] (used reg = )
mov	al,-3[bp]
and	al,8[bx]
test	al,al
je  	.47B
.47C:
! 3314           asciicode = scan_to_scanascii[scancode].shift;
! Debug: ptradd unsigned char scancode = [S+8-3] to [$59] struct  = scan_to_scanascii+0 (used reg = )
mov	al,-1[bp]
xor	ah,ah
mov	dx,ax
shl	ax,*1
shl	ax,*1
add	ax,dx
shl	ax,*1
mov	bx,ax
add	bx,#_scan_to_scanascii
! Debug: eq unsigned short = [bx+2] to unsigned char asciicode = [S+8-4] (used reg = )
mov	al,2[bx]
mov	-2[bp],al
!BCC_EOS
! 3315           scancode = scan_to_scanascii[scancode].shift >> 8;
! Debug: ptradd unsigned char scancode = [S+8-3] to [$59] struct  = scan_to_scanascii+0 (used reg = )
mov	al,-1[bp]
xor	ah,ah
mov	dx,ax
shl	ax,*1
shl	ax,*1
add	ax,dx
shl	ax,*1
mov	bx,ax
add	bx,#_scan_to_scanascii
! Debug: sr int = const 8 to unsigned short = [bx+2] (used reg = )
mov	ax,2[bx]
mov	al,ah
xor	ah,ah
! Debug: eq unsigned int = ax+0 to unsigned char scancode = [S+8-3] (used reg = )
mov	-1[bp],al
!BCC_EOS
! 3316           }
! 3317         else {
jmp .47D
.47B:
! 3318           asciicode = scan_to_scanascii[scancode].normal;
! Debug: ptradd unsigned char scancode = [S+8-3] to [$59] struct  = scan_to_scanascii+0 (used reg = )
mov	al,-1[bp]
xor	ah,ah
mov	dx,ax
shl	ax,*1
shl	ax,*1
add	ax,dx
shl	ax,*1
mov	bx,ax
! Debug: eq unsigned short = [bx+_scan_to_scanascii+0] to unsigned char asciicode = [S+8-4] (used reg = )
mov	al,_scan_to_scanascii[bx]
mov	-2[bp],al
!BCC_EOS
! 3319           scancode = scan_to_scanascii[scancode].normal >> 8;
! Debug: ptradd unsigned char scancode = [S+8-3] to [$59] struct  = scan_to_scanascii+0 (used reg = )
mov	al,-1[bp]
xor	ah,ah
mov	dx,ax
shl	ax,*1
shl	ax,*1
add	ax,dx
shl	ax,*1
mov	bx,ax
! Debug: sr int = const 8 to unsigned short = [bx+_scan_to_scanascii+0] (used reg = )
mov	ax,_scan_to_scanascii[bx]
mov	al,ah
xor	ah,ah
! Debug: eq unsigned int = ax+0 to unsigned char scancode = [S+8-3] (used reg = )
mov	-1[bp],al
!BCC_EOS
! 3320           }
! 3321         }
.47D:
! 3322       if (scancode==0 && asciicode==0) {
.47A:
.474:
.471:
! Debug: logeq int = const 0 to unsigned char scancode = [S+8-3] (used reg = )
mov	al,-1[bp]
test	al,al
jne 	.47E
.480:
! Debug: logeq int = const 0 to unsigned char asciicode = [S+8-4] (used reg = )
mov	al,-2[bp]
test	al,al
jne 	.47E
.47F:
! 3323         bios_printf(4, "KBD: int09h_handler(): scancode & asciicode are zero?\n");
! Debug: list * char = .481+0 (used reg = )
mov	bx,#.481
push	bx
! Debug: list int = const 4 (used reg = )
mov	ax,*4
push	ax
! Debug: func () void = bios_printf+0 (used reg = )
call	_bios_printf
add	sp,*4
!BCC_EOS
! 3324         }
! 3325       enqueue_key(scancode, asciicode);
.47E:
! Debug: list unsigned char asciicode = [S+8-4] (used reg = )
mov	al,-2[bp]
xor	ah,ah
push	ax
! Debug: list unsigned char scancode = [S+$A-3] (used reg = )
mov	al,-1[bp]
xor	ah,ah
push	ax
! Debug: func () unsigned int = enqueue_key+0 (used reg = )
call	_enqueue_key
add	sp,*4
!BCC_EOS
! 3326       break;
jmp .445
!BCC_EOS
! 3327     }
! 3328   mf2_state &= ~0x01;
jmp .445
.447:
sub	al,*$1D
beq 	.44E
sub	al,*$D
beq 	.44A
sub	al,*$C
beq 	.44C
sub	al,*2
beq 	.456
sub	al,*2
beq 	.448
sub	al,*$B
beq 	.45E
sub	al,*1
beq 	.464
sub	al,*$D
beq 	.466
sub	al,*$4A
beq 	.452
sub	al,*$D
beq 	.44B
sub	al,*$C
beq 	.44D
sub	al,*2
beq 	.45A
sub	al,*2
beq 	.449
sub	al,*$B
beq 	.461
sub	al,*1
beq 	.465
br 	.469
.445:
..FFF1	=	-8
! Debug: andab int = const -2 to unsigned char mf2_state = [S+8-7] (used reg = )
mov	al,-5[bp]
and	al,#$FE
mov	-5[bp],al
!BCC_EOS
! 3329 }
mov	sp,bp
pop	bp
ret
! 3330   unsigned int
! Register BX used in function int09_function
! 3331 enqueue_key(scan_code, ascii_code)
! 3332   Bit8u scan_code, ascii_code;
export	_enqueue_key
_enqueue_key:
!BCC_EOS
! 3333 {
! 3334   Bit16u buffer_start, buffer_end, buffer_head, buffer_tail, temp_tail;
!BCC_EOS
! 3335   buffer_start = read_word(0x0040, 0x0080);
push	bp
mov	bp,sp
add	sp,*-$A
! Debug: list int = const $80 (used reg = )
mov	ax,#$80
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () unsigned short = read_word+0 (used reg = )
call	_read_word
add	sp,*4
! Debug: eq unsigned short = ax+0 to unsigned short buffer_start = [S+$C-4] (used reg = )
mov	-2[bp],ax
!BCC_EOS
! 3336   buffer_end = read_word(0x0040, 0x0082);
! Debug: list int = const $82 (used reg = )
mov	ax,#$82
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () unsigned short = read_word+0 (used reg = )
call	_read_word
add	sp,*4
! Debug: eq unsigned short = ax+0 to unsigned short buffer_end = [S+$C-6] (used reg = )
mov	-4[bp],ax
!BCC_EOS
! 3337   buffer_head = read_word(0x0040, 0x001A);
! Debug: list int = const $1A (used reg = )
mov	ax,*$1A
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () unsigned short = read_word+0 (used reg = )
call	_read_word
add	sp,*4
! Debug: eq unsigned short = ax+0 to unsigned short buffer_head = [S+$C-8] (used reg = )
mov	-6[bp],ax
!BCC_EOS
! 3338   buffer_tail = read_word(0x0040, 0x001C);
! Debug: list int = const $1C (used reg = )
mov	ax,*$1C
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () unsigned short = read_word+0 (used reg = )
call	_read_word
add	sp,*4
! Debug: eq unsigned short = ax+0 to unsigned short buffer_tail = [S+$C-$A] (used reg = )
mov	-8[bp],ax
!BCC_EOS
! 3339   temp_tail = buffer_tail;
! Debug: eq unsigned short buffer_tail = [S+$C-$A] to unsigned short temp_tail = [S+$C-$C] (used reg = )
mov	ax,-8[bp]
mov	-$A[bp],ax
!BCC_EOS
! 3340   buffer_tail += 2;
! Debug: addab int = const 2 to unsigned short buffer_tail = [S+$C-$A] (used reg = )
mov	ax,-8[bp]
inc	ax
inc	ax
mov	-8[bp],ax
!BCC_EOS
! 3341   if (buffer_tail >= buffer_end)
! Debug: ge unsigned short buffer_end = [S+$C-6] to unsigned short buffer_tail = [S+$C-$A] (used reg = )
mov	ax,-8[bp]
cmp	ax,-4[bp]
jb  	.482
.483:
! 3342     buffer_tail = buffer_start;
! Debug: eq unsigned short buffer_start = [S+$C-4] to unsigned short buffer_tail = [S+$C-$A] (used reg = )
mov	ax,-2[bp]
mov	-8[bp],ax
!BCC_EOS
! 3343   if (buffer_tail == buffer_head) {
.482:
! Debug: logeq unsigned short buffer_head = [S+$C-8] to unsigned short buffer_tail = [S+$C-$A] (used reg = )
mov	ax,-8[bp]
cmp	ax,-6[bp]
jne 	.484
.485:
! 3344     return(0);
xor	ax,ax
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 3345     }
! 3346    write_byte(0x0040, temp_tail, ascii_code);
.484:
! Debug: list unsigned char ascii_code = [S+$C+4] (used reg = )
mov	al,6[bp]
xor	ah,ah
push	ax
! Debug: list unsigned short temp_tail = [S+$E-$C] (used reg = )
push	-$A[bp]
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 3347    write_byte(0x0040, temp_tail+1, scan_code);
! Debug: list unsigned char scan_code = [S+$C+2] (used reg = )
mov	al,4[bp]
xor	ah,ah
push	ax
! Debug: add int = const 1 to unsigned short temp_tail = [S+$E-$C] (used reg = )
mov	ax,-$A[bp]
! Debug: list unsigned int = ax+1 (used reg = )
inc	ax
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 3348    write_word(0x0040, 0x001C, buffer_tail);
! Debug: list unsigned short buffer_tail = [S+$C-$A] (used reg = )
push	-8[bp]
! Debug: list int = const $1C (used reg = )
mov	ax,*$1C
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
add	sp,*6
!BCC_EOS
! 3349    return(1);
mov	ax,*1
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 3350 }
! 3351   void
! 3352 int74_function(make_farcall, Z, Y, X, status)
! 3353   Bit16u make_farcall, Z, Y, X, status;
export	_int74_function
_int74_function:
!BCC_EOS
! 3354 {
! 3355   Bit16u ebda_seg=read_word(0x0040,0x000E);
push	bp
mov	bp,sp
dec	sp
dec	sp
! Debug: list int = const $E (used reg = )
mov	ax,*$E
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () unsigned short = read_word+0 (used reg = )
call	_read_word
add	sp,*4
! Debug: eq unsigned short = ax+0 to unsigned short ebda_seg = [S+4-4] (used reg = )
mov	-2[bp],ax
!BCC_EOS
! 3356   Bit8u in_byte, index, package_count;
!BCC_EOS
! 3357   Bit8u mouse_flags_1, mouse_flags_2;
!BCC_EOS
! 3358 ;
add	sp,*-6
!BCC_EOS
! 3359   make_farcall = 0;
! Debug: eq int = const 0 to unsigned short make_farcall = [S+$A+2] (used reg = )
xor	ax,ax
mov	4[bp],ax
!BCC_EOS
! 3360   in_byte = inb(0x64);
! Debug: list int = const $64 (used reg = )
mov	ax,*$64
push	ax
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char in_byte = [S+$A-5] (used reg = )
mov	-3[bp],al
!BCC_EOS
! 3361   if ( (in_byte & 0x21) != 0x21 ) {
! Debug: and int = const $21 to unsigned char in_byte = [S+$A-5] (used reg = )
mov	al,-3[bp]
and	al,*$21
! Debug: ne int = const $21 to unsigned char = al+0 (used reg = )
cmp	al,*$21
je  	.486
.487:
! 3362     return;
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 3363     }
! 3364   in_byte = inb(0x60);
.486:
! Debug: list int = const $60 (used reg = )
mov	ax,*$60
push	ax
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char in_byte = [S+$A-5] (used reg = )
mov	-3[bp],al
!BCC_EOS
! 3365 ;
!BCC_EOS
! 3366   mouse_flags_1 = read_byte(ebda_seg, 0x0026);
! Debug: list int = const $26 (used reg = )
mov	ax,*$26
push	ax
! Debug: list unsigned short ebda_seg = [S+$C-4] (used reg = )
push	-2[bp]
! Debug: func () unsigned char = read_byte+0 (used reg = )
call	_read_byte
add	sp,*4
! Debug: eq unsigned char = al+0 to unsigned char mouse_flags_1 = [S+$A-8] (used reg = )
mov	-6[bp],al
!BCC_EOS
! 3367   mouse_flags_2 = 
! 3367 read_byte(ebda_seg, 0x0027);
! Debug: list int = const $27 (used reg = )
mov	ax,*$27
push	ax
! Debug: list unsigned short ebda_seg = [S+$C-4] (used reg = )
push	-2[bp]
! Debug: func () unsigned char = read_byte+0 (used reg = )
call	_read_byte
add	sp,*4
! Debug: eq unsigned char = al+0 to unsigned char mouse_flags_2 = [S+$A-9] (used reg = )
mov	-7[bp],al
!BCC_EOS
! 3368   if ( (mouse_flags_2 & 0x80) != 0x80 ) {
! Debug: and int = const $80 to unsigned char mouse_flags_2 = [S+$A-9] (used reg = )
mov	al,-7[bp]
and	al,#$80
! Debug: ne int = const $80 to unsigned char = al+0 (used reg = )
cmp	al,#$80
je  	.488
.489:
! 3369       return;
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 3370     }
! 3371   package_count = mouse_flags_2 & 0x07;
.488:
! Debug: and int = const 7 to unsigned char mouse_flags_2 = [S+$A-9] (used reg = )
mov	al,-7[bp]
and	al,*7
! Debug: eq unsigned char = al+0 to unsigned char package_count = [S+$A-7] (used reg = )
mov	-5[bp],al
!BCC_EOS
! 3372   index = mouse_flags_1 & 0x07;
! Debug: and int = const 7 to unsigned char mouse_flags_1 = [S+$A-8] (used reg = )
mov	al,-6[bp]
and	al,*7
! Debug: eq unsigned char = al+0 to unsigned char index = [S+$A-6] (used reg = )
mov	-4[bp],al
!BCC_EOS
! 3373   write_byte(ebda_seg, 0x28 + index, in_byte);
! Debug: list unsigned char in_byte = [S+$A-5] (used reg = )
mov	al,-3[bp]
xor	ah,ah
push	ax
! Debug: add unsigned char index = [S+$C-6] to int = const $28 (used reg = )
! Debug: expression subtree swapping
mov	al,-4[bp]
xor	ah,ah
! Debug: list unsigned int = ax+$28 (used reg = )
add	ax,*$28
push	ax
! Debug: list unsigned short ebda_seg = [S+$E-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 3374   if ( (index+1) >= package_count ) {
! Debug: add int = const 1 to unsigned char index = [S+$A-6] (used reg = )
mov	al,-4[bp]
xor	ah,ah
! Debug: ge unsigned char package_count = [S+$A-7] to unsigned int = ax+1 (used reg = )
inc	ax
push	ax
mov	al,-5[bp]
xor	ah,ah
cmp	ax,-$A[bp]
lea	sp,-8[bp]
ja  	.48A
.48B:
! 3375 ;
!BCC_EOS
! 3376     status = read_byte(ebda_seg, 0x0028 + 0);
! Debug: list int = const $28 (used reg = )
mov	ax,*$28
push	ax
! Debug: list unsigned short ebda_seg = [S+$C-4] (used reg = )
push	-2[bp]
! Debug: func () unsigned char = read_byte+0 (used reg = )
call	_read_byte
add	sp,*4
! Debug: eq unsigned char = al+0 to unsigned short status = [S+$A+$A] (used reg = )
xor	ah,ah
mov	$C[bp],ax
!BCC_EOS
! 3377     X = read_byte(ebda_seg, 0x0028 + 1);
! Debug: list int = const $29 (used reg = )
mov	ax,*$29
push	ax
! Debug: list unsigned short ebda_seg = [S+$C-4] (used reg = )
push	-2[bp]
! Debug: func () unsigned char = read_byte+0 (used reg = )
call	_read_byte
add	sp,*4
! Debug: eq unsigned char = al+0 to unsigned short X = [S+$A+8] (used reg = )
xor	ah,ah
mov	$A[bp],ax
!BCC_EOS
! 3378     Y = read_byte(ebda_seg, 0x0028 + 2);
! Debug: list int = const $2A (used reg = )
mov	ax,*$2A
push	ax
! Debug: list unsigned short ebda_seg = [S+$C-4] (used reg = )
push	-2[bp]
! Debug: func () unsigned char = read_byte+0 (used reg = )
call	_read_byte
add	sp,*4
! Debug: eq unsigned char = al+0 to unsigned short Y = [S+$A+6] (used reg = )
xor	ah,ah
mov	8[bp],ax
!BCC_EOS
! 3379     Z = 0;
! Debug: eq int = const 0 to unsigned short Z = [S+$A+4] (used reg = )
xor	ax,ax
mov	6[bp],ax
!BCC_EOS
! 3380     mouse_flags_1 = 0;
! Debug: eq int = const 0 to unsigned char mouse_flags_1 = [S+$A-8] (used reg = )
xor	al,al
mov	-6[bp],al
!BCC_EOS
! 3381     if (mouse_flags_2 & 0x80)
! Debug: and int = const $80 to unsigned char mouse_flags_2 = [S+$A-9] (used reg = )
mov	al,-7[bp]
and	al,#$80
test	al,al
je  	.48C
.48D:
! 3382       make_farcall = 1;
! Debug: eq int = const 1 to unsigned short make_farcall = [S+$A+2] (used reg = )
mov	ax,*1
mov	4[bp],ax
!BCC_EOS
! 3383     }
.48C:
! 3384   else {
jmp .48E
.48A:
! 3385     mouse_flags_1++;
! Debug: postinc unsigned char mouse_flags_1 = [S+$A-8] (used reg = )
mov	al,-6[bp]
inc	ax
mov	-6[bp],al
!BCC_EOS
! 3386     }
! 3387   write_byte(ebda_seg, 0x0026, mouse_flags_1);
.48E:
! Debug: list unsigned char mouse_flags_1 = [S+$A-8] (used reg = )
mov	al,-6[bp]
xor	ah,ah
push	ax
! Debug: list int = const $26 (used reg = )
mov	ax,*$26
push	ax
! Debug: list unsigned short ebda_seg = [S+$E-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 3388 }
mov	sp,bp
pop	bp
ret
! 3389   void
! 3390 int13_harddisk(DS, ES, DI, SI, BP, ELDX, BX, DX, CX, AX, IP, CS, FLAGS)
! 3391   Bit16u DS, ES, DI, SI, BP, ELDX, BX, DX, CX, AX, IP, CS, FLAGS;
export	_int13_harddisk
_int13_harddisk:
!BCC_EOS
! 3392 {
! 3393   Bit32u lba;
!BCC_EOS
! 3394   Bit16u ebda_seg=read_word(0x0040,0x000E);
push	bp
mov	bp,sp
add	sp,*-6
! Debug: list int = const $E (used reg = )
mov	ax,*$E
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () unsigned short = read_word+0 (used reg = )
call	_read_word
add	sp,*4
! Debug: eq unsigned short = ax+0 to unsigned short ebda_seg = [S+8-8] (used reg = )
mov	-6[bp],ax
!BCC_EOS
! 3395   Bit16u cylinder, head, sector;
!BCC_EOS
! 3396   Bit16u segment, offset;
!BCC_EOS
! 3397   Bit16u npc, nph, npspt, nlc, nlh, nlspt;
!BCC_EOS
! 3398   Bit16u size, count;
!BCC_EOS
! 3399   Bit8u device, status;
!BCC_EOS
! 3400   ;
add	sp,*-$1C
!BCC_EOS
! 3401   write_byte(0x0040, 0x008e, 0);
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: list int = const $8E (used reg = )
mov	ax,#$8E
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 3402   if ( (( ELDX & 0x00ff ) < 0x80) || (( ELDX & 0x00ff ) >= 0x80 + (4*2)) ) {
! Debug: and int = const $FF to unsigned short ELDX = [S+$24+$C] (used reg = )
mov	al,$E[bp]
! Debug: lt int = const $80 to unsigned char = al+0 (used reg = )
cmp	al,#$80
jb  	.490
.491:
! Debug: and int = const $FF to unsigned short ELDX = [S+$24+$C] (used reg = )
mov	al,$E[bp]
! Debug: ge int = const $88 to unsigned char = al+0 (used reg = )
cmp	al,#$88
jb  	.48F
.490:
! 3403     bios_printf(4, "int13_harddisk: function %02x, ELDL out of range %02x\n", ( AX >> 8 ), ( ELDX & 0x00ff ));
! Debug: and int = const $FF to unsigned short ELDX = [S+$24+$C] (used reg = )
mov	al,$E[bp]
! Debug: list unsigned char = al+0 (used reg = )
xor	ah,ah
push	ax
! Debug: sr int = const 8 to unsigned short AX = [S+$26+$14] (used reg = )
mov	ax,$16[bp]
mov	al,ah
xor	ah,ah
! Debug: list unsigned int = ax+0 (used reg = )
push	ax
! Debug: list * char = .492+0 (used reg = )
mov	bx,#.492
push	bx
! Debug: list int = const 4 (used reg = )
mov	ax,*4
push	ax
! Debug: func () void = bios_printf+0 (used reg = )
call	_bios_printf
add	sp,*8
!BCC_EOS
! 3404     goto int13_fail;
add	sp,#..FFF0+$24
br 	.FFF0
!BCC_EOS
! 3405     }
! 3406   device=read_byte(ebda_seg,&((ebda_data_t *) 0)->ata.hdidmap[( ELDX & 0x00ff )-0x80]);
.48F:
! Debug: and int = const $FF to unsigned short ELDX = [S+$24+$C] (used reg = )
mov	al,$E[bp]
! Debug: sub int = const $80 to unsigned char = al+0 (used reg = )
xor	ah,ah
! Debug: ptradd unsigned int = ax-$80 to [8] unsigned char = const $213 (used reg = )
add	ax,*-$80
mov	bx,ax
! Debug: address unsigned char = [bx+$213] (used reg = )
! Debug: list * unsigned char = bx+$213 (used reg = )
add	bx,#$213
push	bx
! Debug: list unsigned short ebda_seg = [S+$26-8] (used reg = )
push	-6[bp]
! Debug: func () unsigned char = read_byte+0 (used reg = )
call	_read_byte
add	sp,*4
! Debug: eq unsigned char = al+0 to unsigned char device = [S+$24-$23] (used reg = )
mov	-$21[bp],al
!BCC_EOS
! 3407   if (device >= (4*2)) {
! Debug: ge int = const 8 to unsigned char device = [S+$24-$23] (used reg = )
mov	al,-$21[bp]
cmp	al,*8
jb  	.493
.494:
! 3408     bios_printf(4, "int13_harddisk: function %02x, unmapped device for ELDL=%02x\n", ( AX >> 8 ), ( ELDX & 0x00ff ));
! Debug: and int = const $FF to unsigned short ELDX = [S+$24+$C] (used reg = )
mov	al,$E[bp]
! Debug: list unsigned char = al+0 (used reg = )
xor	ah,ah
push	ax
! Debug: sr int = const 8 to unsigned short AX = [S+$26+$14] (used reg = )
mov	ax,$16[bp]
mov	al,ah
xor	ah,ah
! Debug: list unsigned int = ax+0 (used reg = )
push	ax
! Debug: list * char = .495+0 (used reg = )
mov	bx,#.495
push	bx
! Debug: list int = const 4 (used reg = )
mov	ax,*4
push	ax
! Debug: func () void = bios_printf+0 (used reg = )
call	_bios_printf
add	sp,*8
!BCC_EOS
! 3409     goto int13_fail;
add	sp,#..FFF0+$24
br 	.FFF0
!BCC_EOS
! 3410     }
! 3411   switch (( AX >> 8 )) {
.493:
! Debug: sr int = const 8 to unsigned short AX = [S+$24+$14] (used reg = )
mov	ax,$16[bp]
mov	al,ah
xor	ah,ah
br 	.498
! 3412     case 0x00:
! 3413       ata_reset (device);
.499:
! Debug: list unsigned char device = [S+$24-$23] (used reg = )
mov	al,-$21[bp]
xor	ah,ah
push	ax
! Debug: func () void = ata_reset+0 (used reg = )
call	_ata_reset
inc	sp
inc	sp
!BCC_EOS
! 3414       goto int13_success;
add	sp,#..FFEE-..FFEF
br 	.FFEE
!BCC_EOS
! 3415       break;
br 	.496
!BCC_EOS
! 3416     case 0x01:
! 3417       status = read_byte(0x0040, 0x0074);
.49A:
! Debug: list int = const $74 (used reg = )
mov	ax,*$74
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () unsigned char = read_byte+0 (used reg = )
call	_read_byte
add	sp,*4
! Debug: eq unsigned char = al+0 to unsigned char status = [S+$24-$24] (used reg = )
mov	-$22[bp],al
!BCC_EOS
! 3418       AX = ((AX & 0x00ff) | ((status) << 8));
! Debug: sl int = const 8 to unsigned char status = [S+$24-$24] (used reg = )
mov	al,-$22[bp]
xor	ah,ah
mov	ah,al
xor	al,al
push	ax
! Debug: and int = const $FF to unsigned short AX = [S+$26+$14] (used reg = )
mov	al,$16[bp]
! Debug: or unsigned int (temp) = [S+$26-$26] to unsigned char = al+0 (used reg = )
xor	ah,ah
or	ax,0+..FFEF[bp]
inc	sp
inc	sp
! Debug: eq unsigned int = ax+0 to unsigned short AX = [S+$24+$14] (used reg = )
mov	$16[bp],ax
!BCC_EOS
! 3419       write_byte(0x0040, 0x0074, 0);
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: list int = const $74 (used reg = )
mov	ax,*$74
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 3420       if (status) goto int13_fail_nostatus;
mov	al,-$22[bp]
test	al,al
je  	.49B
.49C:
add	sp,#..FFED-..FFEF
br 	.FFED
!BCC_EOS
! 3421       else goto int13_success_noah;
jmp .49D
.49B:
add	sp,#..FFEC-..FFEF
br 	.FFEC
!BCC_EOS
! 3422       break;
.49D:
br 	.496
!BCC_EOS
! 3423     case 0x02:
! 3424     case 0x03:
.49E:
! 3425     case 0x04:
.49F:
! 3426       count = ( AX & 0x00ff );
.4A0:
! Debug: and int = const $FF to unsigned short AX = [S+$24+$14] (used reg = )
mov	al,$16[bp]
! Debug: eq unsigned char = al+0 to unsigned short count = [S+$24-$22] (used reg = )
xor	ah,ah
mov	-$20[bp],ax
!BCC_EOS
! 3427       cylinder = ( CX >> 8 );
! Debug: sr int = const 8 to unsigned short CX = [S+$24+$12] (used reg = )
mov	ax,$14[bp]
mov	al,ah
xor	ah,ah
! Debug: eq unsigned int = ax+0 to unsigned short cylinder = [S+$24-$A] (used reg = )
mov	-8[bp],ax
!BCC_EOS
! 3428       cylinder |= ( ((Bit16u) ( CX & 0x00ff )) << 2) & 0x300;
! Debug: and int = const $FF to unsigned short CX = [S+$24+$12] (used reg = )
mov	al,$14[bp]
! Debug: cast unsigned short = const 0 to unsigned char = al+0 (used reg = )
xor	ah,ah
! Debug: sl int = const 2 to unsigned short = ax+0 (used reg = )
shl	ax,*1
shl	ax,*1
! Debug: and int = const $300 to unsigned int = ax+0 (used reg = )
and	ax,#$300
! Debug: orab unsigned int = ax+0 to unsigned short cylinder = [S+$24-$A] (used reg = )
or	ax,-8[bp]
mov	-8[bp],ax
!BCC_EOS
! 3429       sector = (( CX & 0x00ff ) & 0x3f);
! Debug: and int = const $FF to unsigned short CX = [S+$24+$12] (used reg = )
mov	al,$14[bp]
! Debug: and int = const $3F to unsigned char = al+0 (used reg = )
and	al,*$3F
! Debug: eq unsigned char = al+0 to unsigned short sector = [S+$24-$E] (used reg = )
xor	ah,ah
mov	-$C[bp],ax
!BCC_EOS
! 3430       head = ( DX >> 8 );
! Debug: sr int = const 8 to unsigned short DX = [S+$24+$10] (used reg = )
mov	ax,$12[bp]
mov	al,ah
xor	ah,ah
! Debug: eq unsigned int = ax+0 to unsigned short head = [S+$24-$C] (used reg = )
mov	-$A[bp],ax
!BCC_EOS
! 3431       segment = ES;
! Debug: eq unsigned short ES = [S+$24+4] to unsigned short segment = [S+$24-$10] (used reg = )
mov	ax,6[bp]
mov	-$E[bp],ax
!BCC_EOS
! 3432       offset = BX;
! Debug: eq unsigned short BX = [S+$24+$E] to unsigned short offset = [S+$24-$12] (used reg = )
mov	ax,$10[bp]
mov	-$10[bp],ax
!BCC_EOS
! 3433       if ( (coun
! 3433 t > 128) || (count == 0) ) {
! Debug: gt int = const $80 to unsigned short count = [S+$24-$22] (used reg = )
mov	ax,-$20[bp]
cmp	ax,#$80
ja  	.4A2
.4A3:
! Debug: logeq int = const 0 to unsigned short count = [S+$24-$22] (used reg = )
mov	ax,-$20[bp]
test	ax,ax
jne 	.4A1
.4A2:
! 3434         bios_printf(4, "int13_harddisk: function %02x, count out of range!\n",( AX >> 8 ));
! Debug: sr int = const 8 to unsigned short AX = [S+$24+$14] (used reg = )
mov	ax,$16[bp]
mov	al,ah
xor	ah,ah
! Debug: list unsigned int = ax+0 (used reg = )
push	ax
! Debug: list * char = .4A4+0 (used reg = )
mov	bx,#.4A4
push	bx
! Debug: list int = const 4 (used reg = )
mov	ax,*4
push	ax
! Debug: func () void = bios_printf+0 (used reg = )
call	_bios_printf
add	sp,*6
!BCC_EOS
! 3435         goto int13_fail;
add	sp,#..FFF0-..FFEF
br 	.FFF0
!BCC_EOS
! 3436         }
! 3437       nlc = read_word(ebda_seg, &((ebda_data_t *) 0)->ata.devices[device].lchs.cylinders);
.4A1:
! Debug: ptradd unsigned char device = [S+$24-$23] to [8] struct  = const $142 (used reg = )
mov	al,-$21[bp]
xor	ah,ah
mov	cx,*$1A
imul	cx
mov	bx,ax
! Debug: address unsigned short = [bx+$14E] (used reg = )
! Debug: list * unsigned short = bx+$14E (used reg = )
add	bx,#$14E
push	bx
! Debug: list unsigned short ebda_seg = [S+$26-8] (used reg = )
push	-6[bp]
! Debug: func () unsigned short = read_word+0 (used reg = )
call	_read_word
add	sp,*4
! Debug: eq unsigned short = ax+0 to unsigned short nlc = [S+$24-$1A] (used reg = )
mov	-$18[bp],ax
!BCC_EOS
! 3438       nlh = read_word(ebda_seg, &((ebda_data_t *) 0)->ata.devices[device].lchs.heads);
! Debug: ptradd unsigned char device = [S+$24-$23] to [8] struct  = const $142 (used reg = )
mov	al,-$21[bp]
xor	ah,ah
mov	cx,*$1A
imul	cx
mov	bx,ax
! Debug: address unsigned short = [bx+$14C] (used reg = )
! Debug: list * unsigned short = bx+$14C (used reg = )
add	bx,#$14C
push	bx
! Debug: list unsigned short ebda_seg = [S+$26-8] (used reg = )
push	-6[bp]
! Debug: func () unsigned short = read_word+0 (used reg = )
call	_read_word
add	sp,*4
! Debug: eq unsigned short = ax+0 to unsigned short nlh = [S+$24-$1C] (used reg = )
mov	-$1A[bp],ax
!BCC_EOS
! 3439       nlspt = read_word(ebda_seg, &((ebda_data_t *) 0)->ata.devices[device].lchs.spt);
! Debug: ptradd unsigned char device = [S+$24-$23] to [8] struct  = const $142 (used reg = )
mov	al,-$21[bp]
xor	ah,ah
mov	cx,*$1A
imul	cx
mov	bx,ax
! Debug: address unsigned short = [bx+$150] (used reg = )
! Debug: list * unsigned short = bx+$150 (used reg = )
add	bx,#$150
push	bx
! Debug: list unsigned short ebda_seg = [S+$26-8] (used reg = )
push	-6[bp]
! Debug: func () unsigned short = read_word+0 (used reg = )
call	_read_word
add	sp,*4
! Debug: eq unsigned short = ax+0 to unsigned short nlspt = [S+$24-$1E] (used reg = )
mov	-$1C[bp],ax
!BCC_EOS
! 3440       if( (cylinder >= nlc) || (head >= nlh) || (sector > nlspt )) {
! Debug: ge unsigned short nlc = [S+$24-$1A] to unsigned short cylinder = [S+$24-$A] (used reg = )
mov	ax,-8[bp]
cmp	ax,-$18[bp]
jae 	.4A6
.4A8:
! Debug: ge unsigned short nlh = [S+$24-$1C] to unsigned short head = [S+$24-$C] (used reg = )
mov	ax,-$A[bp]
cmp	ax,-$1A[bp]
jae 	.4A6
.4A7:
! Debug: gt unsigned short nlspt = [S+$24-$1E] to unsigned short sector = [S+$24-$E] (used reg = )
mov	ax,-$C[bp]
cmp	ax,-$1C[bp]
jbe 	.4A5
.4A6:
! 3441         bios_printf(4, "int13_harddisk: function %02x, parameters out of range %04x/%04x/%04x!\n", ( AX >> 8 ), cylinder, head, sector);
! Debug: list unsigned short sector = [S+$24-$E] (used reg = )
push	-$C[bp]
! Debug: list unsigned short head = [S+$26-$C] (used reg = )
push	-$A[bp]
! Debug: list unsigned short cylinder = [S+$28-$A] (used reg = )
push	-8[bp]
! Debug: sr int = const 8 to unsigned short AX = [S+$2A+$14] (used reg = )
mov	ax,$16[bp]
mov	al,ah
xor	ah,ah
! Debug: list unsigned int = ax+0 (used reg = )
push	ax
! Debug: list * char = .4A9+0 (used reg = )
mov	bx,#.4A9
push	bx
! Debug: list int = const 4 (used reg = )
mov	ax,*4
push	ax
! Debug: func () void = bios_printf+0 (used reg = )
call	_bios_printf
add	sp,*$C
!BCC_EOS
! 3442         goto int13_fail;
add	sp,#..FFF0-..FFEF
br 	.FFF0
!BCC_EOS
! 3443         }
! 3444       if ( ( AX >> 8 ) == 0x04 ) goto int13_success;
.4A5:
! Debug: sr int = const 8 to unsigned short AX = [S+$24+$14] (used reg = )
mov	ax,$16[bp]
mov	al,ah
xor	ah,ah
! Debug: logeq int = const 4 to unsigned int = ax+0 (used reg = )
cmp	ax,*4
jne 	.4AA
.4AB:
add	sp,#..FFEE-..FFEF
br 	.FFEE
!BCC_EOS
! 3445       nph = read_word(ebda_seg, &((ebda_data_t *) 0)->ata.devices[device].pchs.heads);
.4AA:
! Debug: ptradd unsigned char device = [S+$24-$23] to [8] struct  = const $142 (used reg = )
mov	al,-$21[bp]
xor	ah,ah
mov	cx,*$1A
imul	cx
mov	bx,ax
! Debug: address unsigned short = [bx+$152] (used reg = )
! Debug: list * unsigned short = bx+$152 (used reg = )
add	bx,#$152
push	bx
! Debug: list unsigned short ebda_seg = [S+$26-8] (used reg = )
push	-6[bp]
! Debug: func () unsigned short = read_word+0 (used reg = )
call	_read_word
add	sp,*4
! Debug: eq unsigned short = ax+0 to unsigned short nph = [S+$24-$16] (used reg = )
mov	-$14[bp],ax
!BCC_EOS
! 3446       npspt = read_word(ebda_seg, &((ebda_data_t *) 0)->ata.devices[device].pchs.spt);
! Debug: ptradd unsigned char device = [S+$24-$23] to [8] struct  = const $142 (used reg = )
mov	al,-$21[bp]
xor	ah,ah
mov	cx,*$1A
imul	cx
mov	bx,ax
! Debug: address unsigned short = [bx+$156] (used reg = )
! Debug: list * unsigned short = bx+$156 (used reg = )
add	bx,#$156
push	bx
! Debug: list unsigned short ebda_seg = [S+$26-8] (used reg = )
push	-6[bp]
! Debug: func () unsigned short = read_word+0 (used reg = )
call	_read_word
add	sp,*4
! Debug: eq unsigned short = ax+0 to unsigned short npspt = [S+$24-$18] (used reg = )
mov	-$16[bp],ax
!BCC_EOS
! 3447       if ( (nph != nlh) || (npspt != nlspt)) {
! Debug: ne unsigned short nlh = [S+$24-$1C] to unsigned short nph = [S+$24-$16] (used reg = )
mov	ax,-$14[bp]
cmp	ax,-$1A[bp]
jne 	.4AD
.4AE:
! Debug: ne unsigned short nlspt = [S+$24-$1E] to unsigned short npspt = [S+$24-$18] (used reg = )
mov	ax,-$16[bp]
cmp	ax,-$1C[bp]
je  	.4AC
.4AD:
! 3448         lba = ((((Bit32u)cylinder * (Bit32u)nlh) + (Bit32u)head) * (Bit32u)nlspt) + (Bit32u)sector - 1;
! Debug: cast unsigned long = const 0 to unsigned short sector = [S+$24-$E] (used reg = )
mov	ax,-$C[bp]
xor	bx,bx
push	bx
push	ax
! Debug: cast unsigned long = const 0 to unsigned short nlspt = [S+$28-$1E] (used reg = )
mov	ax,-$1C[bp]
xor	bx,bx
push	bx
push	ax
! Debug: cast unsigned long = const 0 to unsigned short head = [S+$2C-$C] (used reg = )
mov	ax,-$A[bp]
xor	bx,bx
push	bx
push	ax
! Debug: cast unsigned long = const 0 to unsigned short nlh = [S+$30-$1C] (used reg = )
mov	ax,-$1A[bp]
xor	bx,bx
push	bx
push	ax
! Debug: cast unsigned long = const 0 to unsigned short cylinder = [S+$34-$A] (used reg = )
mov	ax,-8[bp]
xor	bx,bx
! Debug: mul unsigned long (temp) = [S+$34-$34] to unsigned long = bx+0 (used reg = )
lea	di,-$E+..FFEF[bp]
call	lmulul
add	sp,*4
! Debug: add unsigned long (temp) = [S+$30-$30] to unsigned long = bx+0 (used reg = )
lea	di,-$A+..FFEF[bp]
call	laddul
add	sp,*4
! Debug: mul unsigned long (temp) = [S+$2C-$2C] to unsigned long = bx+0 (used reg = )
lea	di,-6+..FFEF[bp]
call	lmulul
add	sp,*4
! Debug: add unsigned long (temp) = [S+$28-$28] to unsigned long = bx+0 (used reg = )
lea	di,-2+..FFEF[bp]
call	laddul
add	sp,*4
! Debug: sub unsigned long = const 1 to unsigned long = bx+0 (used reg = )
push	bx
push	ax
mov	ax,*1
xor	bx,bx
push	bx
push	ax
mov	ax,-2+..FFEF[bp]
mov	bx,0+..FFEF[bp]
lea	di,-6+..FFEF[bp]
call	lsubul
add	sp,*8
! Debug: eq unsigned long = bx+0 to unsigned long lba = [S+$24-6] (used reg = )
mov	-4[bp],ax
mov	-2[bp],bx
!BCC_EOS
! 3449         sector = 0;
! Debug: eq int = const 0 to unsigned short sector = [S+$24-$E] (used reg = )
xor	ax,ax
mov	-$C[bp],ax
!BCC_EOS
! 3450         }
! 3451       if ( ( AX >> 8 ) == 0x02 )
.4AC:
! Debug: sr int = const 8 to unsigned short AX = [S+$24+$14] (used reg = )
mov	ax,$16[bp]
mov	al,ah
xor	ah,ah
! Debug: logeq int = const 2 to unsigned int = ax+0 (used reg = )
cmp	ax,*2
jne 	.4AF
.4B0:
! 3452         status=ata_cmd_data_in(device, 0x20, count, cylinder, head, sector, lba, segment, offset);
! Debug: list unsigned short offset = [S+$24-$12] (used reg = )
push	-$10[bp]
! Debug: list unsigned short segment = [S+$26-$10] (used reg = )
push	-$E[bp]
! Debug: list unsigned long lba = [S+$28-6] (used reg = )
push	-2[bp]
push	-4[bp]
! Debug: list unsigned short sector = [S+$2C-$E] (used reg = )
push	-$C[bp]
! Debug: list unsigned short head = [S+$2E-$C] (used reg = )
push	-$A[bp]
! Debug: list unsigned short cylinder = [S+$30-$A] (used reg = )
push	-8[bp]
! Debug: list unsigned short count = [S+$32-$22] (used reg = )
push	-$20[bp]
! Debug: list int = const $20 (used reg = )
mov	ax,*$20
push	ax
! Debug: list unsigned char device = [S+$36-$23] (used reg = )
mov	al,-$21[bp]
xor	ah,ah
push	ax
! Debug: func () unsigned short = ata_cmd_data_in+0 (used reg = )
call	_ata_cmd_data_in
add	sp,*$14
! Debug: eq unsigned short = ax+0 to unsigned char status = [S+$24-$24] (used reg = )
mov	-$22[bp],al
!BCC_EOS
! 3453       else
! 3454         status=ata_cmd_data_out(device, 0x30, count, cylinder, head, sector, lba, segment, offset);
jmp .4B1
.4AF:
! Debug: list unsigned short offset = [S+$24-$12] (used reg = )
push	-$10[bp]
! Debug: list unsigned short segment = [S+$26-$10] (used reg = )
push	-$E[bp]
! Debug: list unsigned long lba = [S+$28-6] (used reg = )
push	-2[bp]
push	-4[bp]
! Debug: list unsigned short sector = [S+$2C-$E] (used reg = )
push	-$C[bp]
! Debug: list unsigned short head = [S+$2E-$C] (used reg = )
push	-$A[bp]
! Debug: list unsigned short cylinder = [S+$30-$A] (used reg = )
push	-8[bp]
! Debug: list unsigned short count = [S+$32-$22] (used reg = )
push	-$20[bp]
! Debug: list int = const $30 (used reg = )
mov	ax,*$30
push	ax
! Debug: list unsigned char device = [S+$36-$23] (used reg = )
mov	al,-$21[bp]
xor	ah,ah
push	ax
! Debug: func () unsigned short = ata_cmd_data_out+0 (used reg = )
call	_ata_cmd_data_out
add	sp,*$14
! Debug: eq unsigned short = ax+0 to unsigned char status = [S+$24-$24] (used reg = )
mov	-$22[bp],al
!BCC_EOS
! 3455       AX = ((AX & 0xff00) | (read_word(ebda_seg, &((ebda_data_t *) 0)->ata.trsfsectors)));
.4B1:
! Debug: list * unsigned short = const $234 (used reg = )
mov	ax,#$234
push	ax
! Debug: list unsigned short ebda_seg = [S+$26-8] (used reg = )
push	-6[bp]
! Debug: func () unsigned short = read_word+0 (used reg = )
call	_read_word
add	sp,*4
push	ax
! Debug: and unsigned int = const $FF00 to unsigned short AX = [S+$26+$14] (used reg = )
mov	ax,$16[bp]
xor	al,al
! Debug: or unsigned short (temp) = [S+$26-$26] to unsigned int = ax+0 (used reg = )
or	ax,0+..FFEF[bp]
inc	sp
inc	sp
! Debug: eq unsigned int = ax+0 to unsigned short AX = [S+$24+$14] (used reg = )
mov	$16[bp],ax
!BCC_EOS
! 3456       if (status != 0) {
! Debug: ne int = const 0 to unsigned char status = [S+$24-$24] (used reg = )
mov	al,-$22[bp]
test	al,al
je  	.4B2
.4B3:
! 3457         bios_printf(4, "int13_harddisk: function %02x, error %02x !\n",( AX >> 8 ),status);
! Debug: list unsigned char status = [S+$24-$24] (used reg = )
mov	al,-$22[bp]
xor	ah,ah
push	ax
! Debug: sr int = const 8 to unsigned short AX = [S+$26+$14] (used reg = )
mov	ax,$16[bp]
mov	al,ah
xor	ah,ah
! Debug: list unsigned int = ax+0 (used reg = )
push	ax
! Debug: list * char = .4B4+0 (used reg = )
mov	bx,#.4B4
push	bx
! Debug: list int = const 4 (used reg = )
mov	ax,*4
push	ax
! Debug: func () void = bios_printf+0 (used reg = )
call	_bios_printf
add	sp,*8
!BCC_EOS
! 3458         AX = ((AX & 0x00ff) | ((0x0c) << 8));
! Debug: and int = const $FF to unsigned short AX = [S+$24+$14] (used reg = )
mov	al,$16[bp]
! Debug: or int = const $C00 to unsigned char = al+0 (used reg = )
xor	ah,ah
or	ax,#$C00
! Debug: eq unsigned int = ax+0 to unsigned short AX = [S+$24+$14] (used reg = )
mov	$16[bp],ax
!BCC_EOS
! 3459         goto int13_fail_noah;
add	sp,#..FFEB-..FFEF
br 	.FFEB
!BCC_EOS
! 3460         }
! 3461       goto int13_success;
.4B2:
add	sp,#..FFEE-..FFEF
br 	.FFEE
!BCC_EOS
! 3462       break;
br 	.496
!BCC_EOS
! 3463     case 0x05:
! 3464       bios_printf(4, "format disk track called\n");
.4B5:
! Debug: list * char = .4B6+0 (used reg = )
mov	bx,#.4B6
push	bx
! Debug: list int = const 4 (used reg = )
mov	ax,*4
push	ax
! Debug: func () void = bios_printf+0 (used reg = )
call	_bios_printf
add	sp,*4
!BCC_EOS
! 3465       goto int13_success;
add	sp,#..FFEE-..FFEF
br 	.FFEE
!BCC_EOS
! 3466       return;
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 3467       break;
br 	.496
!BCC_EOS
! 3468     case 0x08:
! 3469       nlc = read_word(ebda_seg, &((ebda_data_t *) 0)->ata.devices[device].lchs.cylinders);
.4B7:
! Debug: ptradd unsigned char device = [S+$24-$23] to [8] struct  = const $142 (used reg = )
mov	al,-$21[bp]
xor	ah,ah
mov	cx,*$1A
imul	cx
mov	bx,ax
! Debug: address unsigned short = [bx+$14E] (used reg = )
! Debug: list * unsigned short = bx+$14E (used reg = )
add	bx,#$14E
push	bx
! Debug: list unsigned short ebda_seg = [S+$26-8] (used reg = )
push	-6[bp]
! Debug: func () unsigned short = read_word+0 (used reg = )
call	_read_word
add	sp,*4
! Debug: eq unsigned short = ax+0 to unsigned short nlc = [S+$24-$1A] (used reg = )
mov	-$18[bp],ax
!BCC_EOS
! 3470       nlh = read_word(ebda_seg, &((ebda_data_t *) 0)->ata.devices[device].lchs.heads);
! Debug: ptradd unsigned char device = [S+$24-$23] to [8] struct  = const $142 (used reg = )
mov	al,-$21[bp]
xor	ah,ah
mov	cx,*$1A
imul	cx
mov	bx,ax
! Debug: address unsigned short = [bx+$14C] (used reg = )
! Debug: list * unsigned short = bx+$14C (used reg = )
add	bx,#$14C
push	bx
! Debug: list unsigned short ebda_seg = [S+$26-8] (used reg = )
push	-6[bp]
! Debug: func () unsigned short = read_word+0 (used reg = )
call	_read_word
add	sp,*4
! Debug: eq unsigned short = ax+0 to unsigned short nlh = [S+$24-$1C] (used reg = )
mov	-$1A[bp],ax
!BCC_EOS
! 3471       nlspt = read_word(ebda_seg, &((ebda_data_t *) 0)->ata.devices[device].lchs.spt);
! Debug: ptradd unsigned char device = [S+$24-$23] to [8] struct  = const $142 (used reg = )
mov	al,-$21[bp]
xor	ah,ah
mov	cx,*$1A
imul	cx
mov	bx,ax
! Debug: address unsigned short = [bx+$150] (used reg = )
! Debug: list * unsigned short = bx+$150 (used reg = )
add	bx,#$150
push	bx
! Debug: list unsigned short ebda_seg = [S+$26-8] (used reg = )
push	-6[bp]
! Debug: func () unsigned short = read_word+0 (used reg = )
call	_read_word
add	sp,*4
! Debug: eq unsigned short = ax+0 to unsigned short nlspt = [S+$24-$1E] (used reg = )
mov	-$1C[bp],ax
!BCC_EOS
! 3472   
! 3472     count = read_byte(ebda_seg, &((ebda_data_t *) 0)->ata.hdcount);
! Debug: list * unsigned char = const $212 (used reg = )
mov	ax,#$212
push	ax
! Debug: list unsigned short ebda_seg = [S+$26-8] (used reg = )
push	-6[bp]
! Debug: func () unsigned char = read_byte+0 (used reg = )
call	_read_byte
add	sp,*4
! Debug: eq unsigned char = al+0 to unsigned short count = [S+$24-$22] (used reg = )
xor	ah,ah
mov	-$20[bp],ax
!BCC_EOS
! 3473       nlc = nlc - 2;
! Debug: sub int = const 2 to unsigned short nlc = [S+$24-$1A] (used reg = )
mov	ax,-$18[bp]
! Debug: eq unsigned int = ax-2 to unsigned short nlc = [S+$24-$1A] (used reg = )
dec	ax
dec	ax
mov	-$18[bp],ax
!BCC_EOS
! 3474       AX = ((AX & 0xff00) | (0));
! Debug: and unsigned int = const $FF00 to unsigned short AX = [S+$24+$14] (used reg = )
mov	ax,$16[bp]
xor	al,al
! Debug: or int = const 0 to unsigned int = ax+0 (used reg = )
or	al,*0
! Debug: eq unsigned int = ax+0 to unsigned short AX = [S+$24+$14] (used reg = )
mov	$16[bp],ax
!BCC_EOS
! 3475       CX = ((CX & 0x00ff) | ((nlc & 0xff) << 8));
! Debug: and int = const $FF to unsigned short nlc = [S+$24-$1A] (used reg = )
mov	al,-$18[bp]
! Debug: sl int = const 8 to unsigned char = al+0 (used reg = )
xor	ah,ah
mov	ah,al
xor	al,al
push	ax
! Debug: and int = const $FF to unsigned short CX = [S+$26+$12] (used reg = )
mov	al,$14[bp]
! Debug: or unsigned int (temp) = [S+$26-$26] to unsigned char = al+0 (used reg = )
xor	ah,ah
or	ax,0+..FFEF[bp]
inc	sp
inc	sp
! Debug: eq unsigned int = ax+0 to unsigned short CX = [S+$24+$12] (used reg = )
mov	$14[bp],ax
!BCC_EOS
! 3476       CX = ((CX & 0xff00) | (((nlc >> 2) & 0xc0) | (nlspt & 0x3f)));
! Debug: and int = const $3F to unsigned short nlspt = [S+$24-$1E] (used reg = )
mov	al,-$1C[bp]
and	al,*$3F
push	ax
! Debug: sr int = const 2 to unsigned short nlc = [S+$26-$1A] (used reg = )
mov	ax,-$18[bp]
shr	ax,*1
shr	ax,*1
! Debug: and int = const $C0 to unsigned int = ax+0 (used reg = )
and	al,#$C0
! Debug: or unsigned char (temp) = [S+$26-$26] to unsigned char = al+0 (used reg = )
or	al,0+..FFEF[bp]
inc	sp
inc	sp
push	ax
! Debug: and unsigned int = const $FF00 to unsigned short CX = [S+$26+$12] (used reg = )
mov	ax,$14[bp]
xor	al,al
! Debug: or unsigned char (temp) = [S+$26-$26] to unsigned int = ax+0 (used reg = )
or	al,0+..FFEF[bp]
inc	sp
inc	sp
! Debug: eq unsigned int = ax+0 to unsigned short CX = [S+$24+$12] (used reg = )
mov	$14[bp],ax
!BCC_EOS
! 3477       DX = ((DX & 0x00ff) | ((nlh - 1) << 8));
! Debug: sub int = const 1 to unsigned short nlh = [S+$24-$1C] (used reg = )
mov	ax,-$1A[bp]
! Debug: sl int = const 8 to unsigned int = ax-1 (used reg = )
dec	ax
mov	ah,al
xor	al,al
push	ax
! Debug: and int = const $FF to unsigned short DX = [S+$26+$10] (used reg = )
mov	al,$12[bp]
! Debug: or unsigned int (temp) = [S+$26-$26] to unsigned char = al+0 (used reg = )
xor	ah,ah
or	ax,0+..FFEF[bp]
inc	sp
inc	sp
! Debug: eq unsigned int = ax+0 to unsigned short DX = [S+$24+$10] (used reg = )
mov	$12[bp],ax
!BCC_EOS
! 3478       DX = ((DX & 0xff00) | (count));
! Debug: and unsigned int = const $FF00 to unsigned short DX = [S+$24+$10] (used reg = )
mov	ax,$12[bp]
xor	al,al
! Debug: or unsigned short count = [S+$24-$22] to unsigned int = ax+0 (used reg = )
or	ax,-$20[bp]
! Debug: eq unsigned int = ax+0 to unsigned short DX = [S+$24+$10] (used reg = )
mov	$12[bp],ax
!BCC_EOS
! 3479       goto int13_success;
add	sp,#..FFEE-..FFEF
br 	.FFEE
!BCC_EOS
! 3480       break;
br 	.496
!BCC_EOS
! 3481     case 0x10:
! 3482       status = inb(read_word(ebda_seg, &((ebda_data_t *) 0)->ata.channels[device/2].iobase1) + 7);
.4B8:
! Debug: div int = const 2 to unsigned char device = [S+$24-$23] (used reg = )
mov	al,-$21[bp]
xor	ah,ah
shr	ax,*1
! Debug: ptradd unsigned int = ax+0 to [4] struct  = const $122 (used reg = )
mov	cl,*3
shl	ax,cl
mov	bx,ax
! Debug: address unsigned short = [bx+$124] (used reg = )
! Debug: list * unsigned short = bx+$124 (used reg = )
add	bx,#$124
push	bx
! Debug: list unsigned short ebda_seg = [S+$26-8] (used reg = )
push	-6[bp]
! Debug: func () unsigned short = read_word+0 (used reg = )
call	_read_word
add	sp,*4
! Debug: add int = const 7 to unsigned short = ax+0 (used reg = )
! Debug: list unsigned int = ax+7 (used reg = )
add	ax,*7
push	ax
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char status = [S+$24-$24] (used reg = )
mov	-$22[bp],al
!BCC_EOS
! 3483       if ( (status & ( 0x80 | 0x40 )) == 0x40 ) {
! Debug: and int = const $C0 to unsigned char status = [S+$24-$24] (used reg = )
mov	al,-$22[bp]
and	al,#$C0
! Debug: logeq int = const $40 to unsigned char = al+0 (used reg = )
cmp	al,*$40
jne 	.4B9
.4BA:
! 3484         goto int13_success;
add	sp,#..FFEE-..FFEF
br 	.FFEE
!BCC_EOS
! 3485         }
! 3486       else {
jmp .4BB
.4B9:
! 3487         AX = ((AX & 0x00ff) | ((0xAA) << 8));
! Debug: and int = const $FF to unsigned short AX = [S+$24+$14] (used reg = )
mov	al,$16[bp]
! Debug: or int = const -$5600 to unsigned char = al+0 (used reg = )
xor	ah,ah
or	ax,#-$5600
! Debug: eq unsigned int = ax+0 to unsigned short AX = [S+$24+$14] (used reg = )
mov	$16[bp],ax
!BCC_EOS
! 3488         goto int13_fail_noah;
add	sp,#..FFEB-..FFEF
br 	.FFEB
!BCC_EOS
! 3489         }
! 3490       break;
.4BB:
br 	.496
!BCC_EOS
! 3491     case 0x15:
! 3492       npc = read_word(ebda_seg, &((ebda_data_t *) 0)->ata.devices[device].pchs.cylinders);
.4BC:
! Debug: ptradd unsigned char device = [S+$24-$23] to [8] struct  = const $142 (used reg = )
mov	al,-$21[bp]
xor	ah,ah
mov	cx,*$1A
imul	cx
mov	bx,ax
! Debug: address unsigned short = [bx+$154] (used reg = )
! Debug: list * unsigned short = bx+$154 (used reg = )
add	bx,#$154
push	bx
! Debug: list unsigned short ebda_seg = [S+$26-8] (used reg = )
push	-6[bp]
! Debug: func () unsigned short = read_word+0 (used reg = )
call	_read_word
add	sp,*4
! Debug: eq unsigned short = ax+0 to unsigned short npc = [S+$24-$14] (used reg = )
mov	-$12[bp],ax
!BCC_EOS
! 3493       nph = read_word(ebda_seg, &((ebda_data_t *) 0)->ata.devices[device].pchs.heads);
! Debug: ptradd unsigned char device = [S+$24-$23] to [8] struct  = const $142 (used reg = )
mov	al,-$21[bp]
xor	ah,ah
mov	cx,*$1A
imul	cx
mov	bx,ax
! Debug: address unsigned short = [bx+$152] (used reg = )
! Debug: list * unsigned short = bx+$152 (used reg = )
add	bx,#$152
push	bx
! Debug: list unsigned short ebda_seg = [S+$26-8] (used reg = )
push	-6[bp]
! Debug: func () unsigned short = read_word+0 (used reg = )
call	_read_word
add	sp,*4
! Debug: eq unsigned short = ax+0 to unsigned short nph = [S+$24-$16] (used reg = )
mov	-$14[bp],ax
!BCC_EOS
! 3494       npspt = read_word(ebda_seg, &((ebda_data_t *) 0)->ata.devices[device].pchs.spt);
! Debug: ptradd unsigned char device = [S+$24-$23] to [8] struct  = const $142 (used reg = )
mov	al,-$21[bp]
xor	ah,ah
mov	cx,*$1A
imul	cx
mov	bx,ax
! Debug: address unsigned short = [bx+$156] (used reg = )
! Debug: list * unsigned short = bx+$156 (used reg = )
add	bx,#$156
push	bx
! Debug: list unsigned short ebda_seg = [S+$26-8] (used reg = )
push	-6[bp]
! Debug: func () unsigned short = read_word+0 (used reg = )
call	_read_word
add	sp,*4
! Debug: eq unsigned short = ax+0 to unsigned short npspt = [S+$24-$18] (used reg = )
mov	-$16[bp],ax
!BCC_EOS
! 3495       lba = (Bit32u)(npc - 1) * (Bit32u)nph * (Bit32u)npspt;
! Debug: cast unsigned long = const 0 to unsigned short npspt = [S+$24-$18] (used reg = )
mov	ax,-$16[bp]
xor	bx,bx
push	bx
push	ax
! Debug: cast unsigned long = const 0 to unsigned short nph = [S+$28-$16] (used reg = )
mov	ax,-$14[bp]
xor	bx,bx
push	bx
push	ax
! Debug: sub int = const 1 to unsigned short npc = [S+$2C-$14] (used reg = )
mov	ax,-$12[bp]
! Debug: cast unsigned long = const 0 to unsigned int = ax-1 (used reg = )
dec	ax
xor	bx,bx
! Debug: mul unsigned long (temp) = [S+$2C-$2C] to unsigned long = bx+0 (used reg = )
lea	di,-6+..FFEF[bp]
call	lmulul
add	sp,*4
! Debug: mul unsigned long (temp) = [S+$28-$28] to unsigned long = bx+0 (used reg = )
lea	di,-2+..FFEF[bp]
call	lmulul
add	sp,*4
! Debug: eq unsigned long = bx+0 to unsigned long lba = [S+$24-6] (used reg = )
mov	-4[bp],ax
mov	-2[bp],bx
!BCC_EOS
! 3496       CX = lba >> 16;
! Debug: sr int = const $10 to unsigned long lba = [S+$24-6] (used reg = )
mov	ax,-4[bp]
mov	bx,-2[bp]
xchg	bx,ax
xor	bx,bx
! Debug: eq unsigned long = bx+0 to unsigned short CX = [S+$24+$12] (used reg = )
mov	$14[bp],ax
!BCC_EOS
! 3497       DX = lba & 0xffff;
! Debug: and unsigned long = const $FFFF to unsigned long lba = [S+$24-6] (used reg = )
! Debug: expression subtree swapping
mov	ax,#$FFFF
xor	bx,bx
lea	di,-4[bp]
call	landul
! Debug: eq unsigned long = bx+0 to unsigned short DX = [S+$24+$10] (used reg = )
mov	$12[bp],ax
!BCC_EOS
! 3498       AX = ((AX & 0x00ff) | ((3) << 8));
! Debug: and int = const $FF to unsigned short AX = [S+$24+$14] (used reg = )
mov	al,$16[bp]
! Debug: or int = const $300 to unsigned char = al+0 (used reg = )
xor	ah,ah
or	ax,#$300
! Debug: eq unsigned int = ax+0 to unsigned short AX = [S+$24+$14] (used reg = )
mov	$16[bp],ax
!BCC_EOS
! 3499       goto int13_success_noah;
add	sp,#..FFEC-..FFEF
br 	.FFEC
!BCC_EOS
! 3500       break;
br 	.496
!BCC_EOS
! 3501     case 0x41:
! 3502       BX=0xaa55;
.4BD:
! Debug: eq unsigned int = const $AA55 to unsigned short BX = [S+$24+$E] (used reg = )
mov	ax,#$AA55
mov	$10[bp],ax
!BCC_EOS
! 3503       AX = ((AX & 0x00ff) | ((0x30) << 8));
! Debug: and int = const $FF to unsigned short AX = [S+$24+$14] (used reg = )
mov	al,$16[bp]
! Debug: or int = const $3000 to unsigned char = al+0 (used reg = )
xor	ah,ah
or	ax,#$3000
! Debug: eq unsigned int = ax+0 to unsigned short AX = [S+$24+$14] (used reg = )
mov	$16[bp],ax
!BCC_EOS
! 3504       CX=0x0007;
! Debug: eq int = const 7 to unsigned short CX = [S+$24+$12] (used reg = )
mov	ax,*7
mov	$14[bp],ax
!BCC_EOS
! 3505       goto int13_success_noah;
add	sp,#..FFEC-..FFEF
br 	.FFEC
!BCC_EOS
! 3506       break;
br 	.496
!BCC_EOS
! 3507     case 0x42:
! 3508     case 0x43:
.4BE:
! 3509     case 0x44:
.4BF:
! 3510     case 0x47:
.4C0:
! 3511       count=read_word(DS, SI+(Bit16u)&((int13ext_t *) 0)->count);
.4C1:
! Debug: add unsigned short = const 2 to unsigned short SI = [S+$24+8] (used reg = )
mov	ax,$A[bp]
! Debug: list unsigned int = ax+2 (used reg = )
inc	ax
inc	ax
push	ax
! Debug: list unsigned short DS = [S+$26+2] (used reg = )
push	4[bp]
! Debug: func () unsigned short = read_word+0 (used reg = )
call	_read_word
add	sp,*4
! Debug: eq unsigned short = ax+0 to unsigned short count = [S+$24-$22] (used reg = )
mov	-$20[bp],ax
!BCC_EOS
! 3512       segment=read_word(DS, SI+(Bit16u)&((int13ext_t *) 0)->segment);
! Debug: add unsigned short = const 6 to unsigned short SI = [S+$24+8] (used reg = )
mov	ax,$A[bp]
! Debug: list unsigned int = ax+6 (used reg = )
add	ax,*6
push	ax
! Debug: list unsigned short DS = [S+$26+2] (used reg = )
push	4[bp]
! Debug: func () unsigned short = read_word+0 (used reg = )
call	_read_word
add	sp,*4
! Debug: eq unsigned short = ax+0 to unsigned short segment = [S+$24-$10] (used reg = )
mov	-$E[bp],ax
!BCC_EOS
! 3513       offset=read_word(DS, SI+(Bit16u)&((int13ext_t *) 0)->offset);
! Debug: add unsigned short = const 4 to unsigned short SI = [S+$24+8] (used reg = )
mov	ax,$A[bp]
! Debug: list unsigned int = ax+4 (used reg = )
add	ax,*4
push	ax
! Debug: list unsigned short DS = [S+$26+2] (used reg = )
push	4[bp]
! Debug: func () unsigned short = read_word+0 (used reg = )
call	_read_word
add	sp,*4
! Debug: eq unsigned short = ax+0 to unsigned short offset = [S+$24-$12] (used reg = )
mov	-$10[bp],ax
!BCC_EOS
! 3514       lba=read_dword(DS, SI+(Bit16u)&((int13ext_t *) 0)->lba2);
! Debug: add unsigned short = const $C to unsigned short SI = [S+$24+8] (used reg = )
mov	ax,$A[bp]
! Debug: list unsigned int = ax+$C (used reg = )
add	ax,*$C
push	ax
! Debug: list unsigned short DS = [S+$26+2] (used reg = )
push	4[bp]
! Debug: func () unsigned long = read_dword+0 (used reg = )
call	_read_dword
mov	bx,dx
add	sp,*4
! Debug: eq unsigned long = bx+0 to unsigned long lba = [S+$24-6] (used reg = )
mov	-4[bp],ax
mov	-2[bp],bx
!BCC_EOS
! 3515       if (lba != 0L) {
! Debug: ne long = const 0 to unsigned long lba = [S+$24-6] (used reg = )
! Debug: expression subtree swapping
xor	ax,ax
xor	bx,bx
push	bx
push	ax
mov	ax,-4[bp]
mov	bx,-2[bp]
lea	di,-2+..FFEF[bp]
call	lcmpul
lea	sp,2+..FFEF[bp]
je  	.4C2
.4C3:
! 3516         bios_printf((2 | 4 | 1), "int13_harddisk: function %02x. Can't use 64bits lba\n",( AX >> 8 ));
! Debug: sr int = const 8 to unsigned short AX = [S+$24+$14] (used reg = )
mov	ax,$16[bp]
mov	al,ah
xor	ah,ah
! Debug: list unsigned int = ax+0 (used reg = )
push	ax
! Debug: list * char = .4C4+0 (used reg = )
mov	bx,#.4C4
push	bx
! Debug: list int = const 7 (used reg = )
mov	ax,*7
push	ax
! Debug: func () void = bios_printf+0 (used reg = )
call	_bios_printf
add	sp,*6
!BCC_EOS
! 3517         goto int13_fail;
add	sp,#..FFF0-..FFEF
br 	.FFF0
!BCC_EOS
! 3518         }
! 3519       lba=read_dword(DS, SI+(Bit16u)&((int13ext_t *) 0)->lba1);
.4C2:
! Debug: add unsigned short = const 8 to unsigned short SI = [S+$24+8] (used reg = )
mov	ax,$A[bp]
! Debug: list unsigned int = ax+8 (used reg = )
add	ax,*8
push	ax
! Debug: list unsigned short DS = [S+$26+2] (used reg = )
push	4[bp]
! Debug: func () unsigned long = read_dword+0 (used reg = )
call	_read_dword
mov	bx,dx
add	sp,*4
! Debug: eq unsigned long = bx+0 to unsigned long lba = [S+$24-6] (used reg = )
mov	-4[bp],ax
mov	-2[bp],bx
!BCC_EOS
! 3520       if (lba >= read_dword(ebda_seg, &((ebda_data_t *) 0)->ata.devices[device].sectors) ) {
! Debug: ptradd unsigned char device = [S+$24-$23] to [8] struct  = const $142 (used reg = )
mov	al,-$21[bp]
xor	ah,ah
mov	cx,*$1A
imul	cx
mov	bx,ax
! Debug: address unsigned long = [bx+$158] (used reg = )
! Debug: list * unsigned long = bx+$158 (used reg = )
add	bx,#$158
push	bx
! Debug: list unsigned short ebda_seg = [S+$26-8] (used reg = )
push	-6[bp]
! Debug: func () unsigned long = read_dword+0 (used reg = )
call	_read_dword
mov	bx,dx
add	sp,*4
! Debug: ge unsigned long = bx+0 to unsigned long lba = [S+$24-6] (used reg = )
lea	di,-4[bp]
call	lcmpul
ja  	.4C5
.4C6:
! 3521         bios_printf(4, "int13_harddisk: function %02x. LBA out of range\n",( AX >> 8 ));
! Debug: sr int = const 8 to unsigned short AX = [S+$24+$14] (used reg = )
mov	ax,$16[bp]
mov	al,ah
xor	ah,ah
! Debug: list unsigned int = ax+0 (used reg = )
push	ax
! Debug: list * char = .4C7+0 (used reg = )
mov	bx,#.4C7
push	bx
! Debug: list int = const 4 (used reg = )
mov	ax,*4
push	ax
! Debug: func () void = bios_printf+0 (used reg = )
call	_bios_printf
add	sp,*6
!BCC_EOS
! 3522         goto int13_fail
! 3522 ;
add	sp,#..FFF0-..FFEF
br 	.FFF0
!BCC_EOS
! 3523         }
! 3524       if (( ( AX >> 8 ) == 0x44 ) || ( ( AX >> 8 ) == 0x47 ))
.4C5:
! Debug: sr int = const 8 to unsigned short AX = [S+$24+$14] (used reg = )
mov	ax,$16[bp]
mov	al,ah
xor	ah,ah
! Debug: logeq int = const $44 to unsigned int = ax+0 (used reg = )
cmp	ax,*$44
je  	.4C9
.4CA:
! Debug: sr int = const 8 to unsigned short AX = [S+$24+$14] (used reg = )
mov	ax,$16[bp]
mov	al,ah
xor	ah,ah
! Debug: logeq int = const $47 to unsigned int = ax+0 (used reg = )
cmp	ax,*$47
jne 	.4C8
.4C9:
! 3525         goto int13_success;
add	sp,#..FFEE-..FFEF
br 	.FFEE
!BCC_EOS
! 3526       if ( ( AX >> 8 ) == 0x42 )
.4C8:
! Debug: sr int = const 8 to unsigned short AX = [S+$24+$14] (used reg = )
mov	ax,$16[bp]
mov	al,ah
xor	ah,ah
! Debug: logeq int = const $42 to unsigned int = ax+0 (used reg = )
cmp	ax,*$42
jne 	.4CB
.4CC:
! 3527         status=ata_cmd_data_in(device, 0x20, count, 0, 0, 0, lba, segment, offset);
! Debug: list unsigned short offset = [S+$24-$12] (used reg = )
push	-$10[bp]
! Debug: list unsigned short segment = [S+$26-$10] (used reg = )
push	-$E[bp]
! Debug: list unsigned long lba = [S+$28-6] (used reg = )
push	-2[bp]
push	-4[bp]
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: list unsigned short count = [S+$32-$22] (used reg = )
push	-$20[bp]
! Debug: list int = const $20 (used reg = )
mov	ax,*$20
push	ax
! Debug: list unsigned char device = [S+$36-$23] (used reg = )
mov	al,-$21[bp]
xor	ah,ah
push	ax
! Debug: func () unsigned short = ata_cmd_data_in+0 (used reg = )
call	_ata_cmd_data_in
add	sp,*$14
! Debug: eq unsigned short = ax+0 to unsigned char status = [S+$24-$24] (used reg = )
mov	-$22[bp],al
!BCC_EOS
! 3528       else
! 3529         status=ata_cmd_data_out(device, 0x30, count, 0, 0, 0, lba, segment, offset);
jmp .4CD
.4CB:
! Debug: list unsigned short offset = [S+$24-$12] (used reg = )
push	-$10[bp]
! Debug: list unsigned short segment = [S+$26-$10] (used reg = )
push	-$E[bp]
! Debug: list unsigned long lba = [S+$28-6] (used reg = )
push	-2[bp]
push	-4[bp]
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: list unsigned short count = [S+$32-$22] (used reg = )
push	-$20[bp]
! Debug: list int = const $30 (used reg = )
mov	ax,*$30
push	ax
! Debug: list unsigned char device = [S+$36-$23] (used reg = )
mov	al,-$21[bp]
xor	ah,ah
push	ax
! Debug: func () unsigned short = ata_cmd_data_out+0 (used reg = )
call	_ata_cmd_data_out
add	sp,*$14
! Debug: eq unsigned short = ax+0 to unsigned char status = [S+$24-$24] (used reg = )
mov	-$22[bp],al
!BCC_EOS
! 3530       count=read_word(ebda_seg, &((ebda_data_t *) 0)->ata.trsfsectors);
.4CD:
! Debug: list * unsigned short = const $234 (used reg = )
mov	ax,#$234
push	ax
! Debug: list unsigned short ebda_seg = [S+$26-8] (used reg = )
push	-6[bp]
! Debug: func () unsigned short = read_word+0 (used reg = )
call	_read_word
add	sp,*4
! Debug: eq unsigned short = ax+0 to unsigned short count = [S+$24-$22] (used reg = )
mov	-$20[bp],ax
!BCC_EOS
! 3531       write_word(DS, SI+(Bit16u)&((int13ext_t *) 0)->count, count);
! Debug: list unsigned short count = [S+$24-$22] (used reg = )
push	-$20[bp]
! Debug: add unsigned short = const 2 to unsigned short SI = [S+$26+8] (used reg = )
mov	ax,$A[bp]
! Debug: list unsigned int = ax+2 (used reg = )
inc	ax
inc	ax
push	ax
! Debug: list unsigned short DS = [S+$28+2] (used reg = )
push	4[bp]
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
add	sp,*6
!BCC_EOS
! 3532       if (status != 0) {
! Debug: ne int = const 0 to unsigned char status = [S+$24-$24] (used reg = )
mov	al,-$22[bp]
test	al,al
je  	.4CE
.4CF:
! 3533         bios_printf(4, "int13_harddisk: function %02x, error %02x !\n",( AX >> 8 ),status);
! Debug: list unsigned char status = [S+$24-$24] (used reg = )
mov	al,-$22[bp]
xor	ah,ah
push	ax
! Debug: sr int = const 8 to unsigned short AX = [S+$26+$14] (used reg = )
mov	ax,$16[bp]
mov	al,ah
xor	ah,ah
! Debug: list unsigned int = ax+0 (used reg = )
push	ax
! Debug: list * char = .4D0+0 (used reg = )
mov	bx,#.4D0
push	bx
! Debug: list int = const 4 (used reg = )
mov	ax,*4
push	ax
! Debug: func () void = bios_printf+0 (used reg = )
call	_bios_printf
add	sp,*8
!BCC_EOS
! 3534         AX = ((AX & 0x00ff) | ((0x0c) << 8));
! Debug: and int = const $FF to unsigned short AX = [S+$24+$14] (used reg = )
mov	al,$16[bp]
! Debug: or int = const $C00 to unsigned char = al+0 (used reg = )
xor	ah,ah
or	ax,#$C00
! Debug: eq unsigned int = ax+0 to unsigned short AX = [S+$24+$14] (used reg = )
mov	$16[bp],ax
!BCC_EOS
! 3535         goto int13_fail_noah;
add	sp,#..FFEB-..FFEF
br 	.FFEB
!BCC_EOS
! 3536         }
! 3537       goto int13_success;
.4CE:
add	sp,#..FFEE-..FFEF
br 	.FFEE
!BCC_EOS
! 3538       break;
br 	.496
!BCC_EOS
! 3539     case 0x45:
! 3540     case 0x49:
.4D1:
! 3541       goto int13_success;
.4D2:
add	sp,#..FFEE-..FFEF
br 	.FFEE
!BCC_EOS
! 3542       break;
br 	.496
!BCC_EOS
! 3543     case 0x46:
! 3544       AX = ((AX & 0x00ff) | ((0xb2) << 8));
.4D3:
! Debug: and int = const $FF to unsigned short AX = [S+$24+$14] (used reg = )
mov	al,$16[bp]
! Debug: or int = const -$4E00 to unsigned char = al+0 (used reg = )
xor	ah,ah
or	ax,#-$4E00
! Debug: eq unsigned int = ax+0 to unsigned short AX = [S+$24+$14] (used reg = )
mov	$16[bp],ax
!BCC_EOS
! 3545       goto int13_fail_noah;
add	sp,#..FFEB-..FFEF
br 	.FFEB
!BCC_EOS
! 3546       break;
br 	.496
!BCC_EOS
! 3547     case 0x48:
! 3548       size=read_word(DS,SI+(Bit16u)&((dpt_t *) 0)->size);
.4D4:
! Debug: add unsigned short = const 0 to unsigned short SI = [S+$24+8] (used reg = )
mov	ax,$A[bp]
! Debug: list unsigned int = ax+0 (used reg = )
push	ax
! Debug: list unsigned short DS = [S+$26+2] (used reg = )
push	4[bp]
! Debug: func () unsigned short = read_word+0 (used reg = )
call	_read_word
add	sp,*4
! Debug: eq unsigned short = ax+0 to unsigned short size = [S+$24-$20] (used reg = )
mov	-$1E[bp],ax
!BCC_EOS
! 3549       if(size < 0x1a)
! Debug: lt int = const $1A to unsigned short size = [S+$24-$20] (used reg = )
mov	ax,-$1E[bp]
cmp	ax,*$1A
jae 	.4D5
.4D6:
! 3550         goto int13_fail;
add	sp,#..FFF0-..FFEF
br 	.FFF0
!BCC_EOS
! 3551       if(size >= 0x1a) {
.4D5:
! Debug: ge int = const $1A to unsigned short size = [S+$24-$20] (used reg = )
mov	ax,-$1E[bp]
cmp	ax,*$1A
blo 	.4D7
.4D8:
! 3552         Bit16u blksize;
!BCC_EOS
! 3553         npc = read_word(ebda_seg, &((ebda_data_t *) 0)->ata.devices[device].pchs.cylinders);
! Debug: ptradd unsigned char device = [S+$26-$23] to [8] struct  = const $142 (used reg = )
mov	al,-$21[bp]
xor	ah,ah
mov	cx,*$1A
imul	cx
mov	bx,ax
! Debug: address unsigned short = [bx+$154] (used reg = )
! Debug: list * unsigned short = bx+$154 (used reg = )
add	bx,#$154
push	bx
! Debug: list unsigned short ebda_seg = [S+$28-8] (used reg = )
push	-6[bp]
! Debug: func () unsigned short = read_word+0 (used reg = )
call	_read_word
add	sp,*4
! Debug: eq unsigned short = ax+0 to unsigned short npc = [S+$26-$14] (used reg = )
mov	-$12[bp],ax
!BCC_EOS
! 3554         nph = read_word(ebda_seg, &((ebda_data_t *) 0)->ata.devices[device].pchs.heads);
! Debug: ptradd unsigned char device = [S+$26-$23] to [8] struct  = const $142 (used reg = )
mov	al,-$21[bp]
xor	ah,ah
mov	cx,*$1A
imul	cx
mov	bx,ax
! Debug: address unsigned short = [bx+$152] (used reg = )
! Debug: list * unsigned short = bx+$152 (used reg = )
add	bx,#$152
push	bx
! Debug: list unsigned short ebda_seg = [S+$28-8] (used reg = )
push	-6[bp]
! Debug: func () unsigned short = read_word+0 (used reg = )
call	_read_word
add	sp,*4
! Debug: eq unsigned short = ax+0 to unsigned short nph = [S+$26-$16] (used reg = )
mov	-$14[bp],ax
!BCC_EOS
! 3555         npspt = read_word(ebda_seg, &((ebda_data_t *) 0)->ata.devices[device].pchs.spt);
! Debug: ptradd unsigned char device = [S+$26-$23] to [8] struct  = const $142 (used reg = )
mov	al,-$21[bp]
xor	ah,ah
mov	cx,*$1A
imul	cx
mov	bx,ax
! Debug: address unsigned short = [bx+$156] (used reg = )
! Debug: list * unsigned short = bx+$156 (used reg = )
add	bx,#$156
push	bx
! Debug: list unsigned short ebda_seg = [S+$28-8] (used reg = )
push	-6[bp]
! Debug: func () unsigned short = read_word+0 (used reg = )
call	_read_word
add	sp,*4
! Debug: eq unsigned short = ax+0 to unsigned short npspt = [S+$26-$18] (used reg = )
mov	-$16[bp],ax
!BCC_EOS
! 3556         lba = read_dword(ebda_seg, &((ebda_data_t *) 0)->ata.devices[device].sectors);
! Debug: ptradd unsigned char device = [S+$26-$23] to [8] struct  = const $142 (used reg = )
mov	al,-$21[bp]
xor	ah,ah
mov	cx,*$1A
imul	cx
mov	bx,ax
! Debug: address unsigned long = [bx+$158] (used reg = )
! Debug: list * unsigned long = bx+$158 (used reg = )
add	bx,#$158
push	bx
! Debug: list unsigned short ebda_seg = [S+$28-8] (used reg = )
push	-6[bp]
! Debug: func () unsigned long = read_dword+0 (used reg = )
call	_read_dword
mov	bx,dx
add	sp,*4
! Debug: eq unsigned long = bx+0 to unsigned long lba = [S+$26-6] (used reg = )
mov	-4[bp],ax
mov	-2[bp],bx
!BCC_EOS
! 3557         blksize = read_word(ebda_seg, &((ebda_data_t *) 0)->ata.devices[device].blksize);
! Debug: ptradd unsigned char device = [S+$26-$23] to [8] struct  = const $142 (used reg = )
mov	al,-$21[bp]
xor	ah,ah
mov	cx,*$1A
imul	cx
mov	bx,ax
! Debug: address unsigned short = [bx+$148] (used reg = )
! Debug: list * unsigned short = bx+$148 (used reg = )
add	bx,#$148
push	bx
! Debug: list unsigned short ebda_seg = [S+$28-8] (used reg = )
push	-6[bp]
! Debug: func () unsigned short = read_word+0 (used reg = )
call	_read_word
add	sp,*4
! Debug: eq unsigned short = ax+0 to unsigned short blksize = [S+$26-$26] (used reg = )
mov	-$24[bp],ax
!BCC_EOS
! 3558         write_word(DS, SI+(Bit16u)&((dpt_t *) 0)->size, 0x1a);
! Debug: list int = const $1A (used reg = )
mov	ax,*$1A
push	ax
! Debug: add unsigned short = const 0 to unsigned short SI = [S+$28+8] (used reg = )
mov	ax,$A[bp]
! Debug: list unsigned int = ax+0 (used reg = )
push	ax
! Debug: list unsigned short DS = [S+$2A+2] (used reg = )
push	4[bp]
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
add	sp,*6
!BCC_EOS
! 3559         write_word(DS, SI+(Bit16u)&((dpt_t *) 0)->infos, 0x02);
! Debug: list int = const 2 (used reg = )
mov	ax,*2
push	ax
! Debug: add unsigned short = const 2 to unsigned short SI = [S+$28+8] (used reg = )
mov	ax,$A[bp]
! Debug: list unsigned int = ax+2 (used reg = )
inc	ax
inc	ax
push	ax
! Debug: list unsigned short DS = [S+$2A+2] (used reg = )
push	4[bp]
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
add	sp,*6
!BCC_EOS
! 3560         write_dword(DS, SI+(Bit16u)&((dpt_t *) 0)->cylinders, (Bit32u)npc);
! Debug: cast unsigned long = const 0 to unsigned short npc = [S+$26-$14] (used reg = )
mov	ax,-$12[bp]
xor	bx,bx
! Debug: list unsigned long = bx+0 (used reg = )
push	bx
push	ax
! Debug: add unsigned short = const 4 to unsigned short SI = [S+$2A+8] (used reg = )
mov	ax,$A[bp]
! Debug: list unsigned int = ax+4 (used reg = )
add	ax,*4
push	ax
! Debug: list unsigned short DS = [S+$2C+2] (used reg = )
push	4[bp]
! Debug: func () void = write_dword+0 (used reg = )
call	_write_dword
add	sp,*8
!BCC_EOS
! 3561         write_dword(DS, SI+(Bit16u)&((dpt_t *) 0)->heads, (Bit32u)nph);
! Debug: cast unsigned long = const 0 to unsigned short nph = [S+$26-$16] (used reg = )
mov	ax,-$14[bp]
xor	bx,bx
! Debug: list unsigned long = bx+0 (used reg = )
push	bx
push	ax
! Debug: add unsigned short = const 8 to unsigned short SI = [S+$2A+8] (used reg = )
mov	ax,$A[bp]
! Debug: list unsigned int = ax+8 (used reg = )
add	ax,*8
push	ax
! Debug: list unsigned short DS = [S+$2C+2] (used reg = )
push	4[bp]
! Debug: func () void = write_dword+0 (used reg = )
call	_write_dword
add	sp,*8
!BCC_EOS
! 3562         write_dword(DS, SI+(Bit16u)&((dpt_t *) 0)->spt, (Bit32u)npspt);
! Debug: cast unsigned long = const 0 to unsigned short npspt = [S+$26-$18] (used reg = )
mov	ax,-$16[bp]
xor	bx,bx
! Debug: list unsigned long = bx+0 (used reg = )
push	bx
push	ax
! Debug: add unsigned short = const $C to unsigned short SI = [S+$2A+8] (used reg = )
mov	ax,$A[bp]
! Debug: list unsigned int = ax+$C (used reg = )
add	ax,*$C
push	ax
! Debug: list unsigned short DS = [S+$2C+2] (used reg = )
push	4[bp]
! Debug: func () void = write_dword+0 (used reg = )
call	_write_dword
add	sp,*8
!BCC_EOS
! 3563         write_dword(DS, SI+(Bit16u)&((dpt_t *) 0)->sector_count1, lba);
! Debug: list unsigned long lba = [S+$26-6] (used reg = )
push	-2[bp]
push	-4[bp]
! Debug: add unsigned short = const $10 to unsigned short SI = [S+$2A+8] (used reg = )
mov	ax,$A[bp]
! Debug: list unsigned int = ax+$10 (used reg = )
add	ax,*$10
push	ax
! Debug: list unsigned short DS = [S+$2C+2] (used reg = )
push	4[bp]
! Debug: func () void = write_dword+0 (used reg = )
call	_write_dword
add	sp,*8
!BCC_EOS
! 3564         write_dword(DS, SI+(Bit16u)&((dpt_t *) 0)->sector_count2, 0L);
! Debug: list long = const 0 (used reg = )
xor	ax,ax
xor	bx,bx
push	bx
push	ax
! Debug: add unsigned short = const $14 to unsigned short SI = [S+$2A+8] (used reg = )
mov	ax,$A[bp]
! Debug: list unsigned int = ax+$14 (used reg = )
add	ax,*$14
push	ax
! Debug: list unsigned short DS = [S+$2C+2] (used reg = )
push	4[bp]
! Debug: func () void = write_dword+0 (used reg = )
call	_write_dword
add	sp,*8
!BCC_EOS
! 3565         write_word(DS, SI+(Bit16u)&((dpt_t *) 0)->blksize, blksize);
! Debug: list unsigned short blksize = [S+$26-$26] (used reg = )
push	-$24[bp]
! Debug: add unsigned short = const $18 to unsigned short SI = [S+$28+8] (used reg = )
mov	ax,$A[bp]
! Debug: list unsigned int = ax+$18 (used reg = )
add	ax,*$18
push	ax
! Debug: list unsigned short DS = [S+$2A+2] (used reg = )
push	4[bp]
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
add	sp,*6
!BCC_EOS
! 3566       
! 3566   }
! 3567       if(size >= 0x1e) {
.4D7:
! Debug: ge int = const $1E to unsigned short size = [S+$24-$20] (used reg = )
mov	ax,-$1E[bp]
cmp	ax,*$1E
blo 	.4D9
.4DA:
! 3568         Bit8u channel, dev, irq, mode, checksum, i, translation;
!BCC_EOS
! 3569         Bit16u iobase1, iobase2, options;
!BCC_EOS
! 3570         write_word(DS, SI+(Bit16u)&((dpt_t *) 0)->size, 0x1e);
! Debug: list int = const $1E (used reg = )
mov	ax,*$1E
push	ax
! Debug: add unsigned short = const 0 to unsigned short SI = [S+$34+8] (used reg = )
mov	ax,$A[bp]
! Debug: list unsigned int = ax+0 (used reg = )
push	ax
! Debug: list unsigned short DS = [S+$36+2] (used reg = )
push	4[bp]
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
add	sp,*6
!BCC_EOS
! 3571         write_word(DS, SI+(Bit16u)&((dpt_t *) 0)->dpte_segment, ebda_seg);
! Debug: list unsigned short ebda_seg = [S+$32-8] (used reg = )
push	-6[bp]
! Debug: add unsigned short = const $1C to unsigned short SI = [S+$34+8] (used reg = )
mov	ax,$A[bp]
! Debug: list unsigned int = ax+$1C (used reg = )
add	ax,*$1C
push	ax
! Debug: list unsigned short DS = [S+$36+2] (used reg = )
push	4[bp]
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
add	sp,*6
!BCC_EOS
! 3572         write_word(DS, SI+(Bit16u)&((dpt_t *) 0)->dpte_offset, &((ebda_data_t *) 0)->ata.dpte);
! Debug: list * struct  = const $224 (used reg = )
mov	ax,#$224
push	ax
! Debug: add unsigned short = const $1A to unsigned short SI = [S+$34+8] (used reg = )
mov	ax,$A[bp]
! Debug: list unsigned int = ax+$1A (used reg = )
add	ax,*$1A
push	ax
! Debug: list unsigned short DS = [S+$36+2] (used reg = )
push	4[bp]
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
add	sp,*6
!BCC_EOS
! 3573         channel = device / 2;
! Debug: div int = const 2 to unsigned char device = [S+$32-$23] (used reg = )
mov	al,-$21[bp]
xor	ah,ah
shr	ax,*1
! Debug: eq unsigned int = ax+0 to unsigned char channel = [S+$32-$25] (used reg = )
mov	-$23[bp],al
!BCC_EOS
! 3574         iobase1 = read_word(ebda_seg, &((ebda_data_t *) 0)->ata.channels[channel].iobase1);
! Debug: ptradd unsigned char channel = [S+$32-$25] to [4] struct  = const $122 (used reg = )
mov	al,-$23[bp]
xor	ah,ah
mov	cl,*3
shl	ax,cl
mov	bx,ax
! Debug: address unsigned short = [bx+$124] (used reg = )
! Debug: list * unsigned short = bx+$124 (used reg = )
add	bx,#$124
push	bx
! Debug: list unsigned short ebda_seg = [S+$34-8] (used reg = )
push	-6[bp]
! Debug: func () unsigned short = read_word+0 (used reg = )
call	_read_word
add	sp,*4
! Debug: eq unsigned short = ax+0 to unsigned short iobase1 = [S+$32-$2E] (used reg = )
mov	-$2C[bp],ax
!BCC_EOS
! 3575         iobase2 = read_word(ebda_seg, &((ebda_data_t *) 0)->ata.channels[channel].iobase2);
! Debug: ptradd unsigned char channel = [S+$32-$25] to [4] struct  = const $122 (used reg = )
mov	al,-$23[bp]
xor	ah,ah
mov	cl,*3
shl	ax,cl
mov	bx,ax
! Debug: address unsigned short = [bx+$126] (used reg = )
! Debug: list * unsigned short = bx+$126 (used reg = )
add	bx,#$126
push	bx
! Debug: list unsigned short ebda_seg = [S+$34-8] (used reg = )
push	-6[bp]
! Debug: func () unsigned short = read_word+0 (used reg = )
call	_read_word
add	sp,*4
! Debug: eq unsigned short = ax+0 to unsigned short iobase2 = [S+$32-$30] (used reg = )
mov	-$2E[bp],ax
!BCC_EOS
! 3576         irq = read_byte(ebda_seg, &((ebda_data_t *) 0)->ata.channels[channel].irq);
! Debug: ptradd unsigned char channel = [S+$32-$25] to [4] struct  = const $122 (used reg = )
mov	al,-$23[bp]
xor	ah,ah
mov	cl,*3
shl	ax,cl
mov	bx,ax
! Debug: address unsigned char = [bx+$128] (used reg = )
! Debug: list * unsigned char = bx+$128 (used reg = )
add	bx,#$128
push	bx
! Debug: list unsigned short ebda_seg = [S+$34-8] (used reg = )
push	-6[bp]
! Debug: func () unsigned char = read_byte+0 (used reg = )
call	_read_byte
add	sp,*4
! Debug: eq unsigned char = al+0 to unsigned char irq = [S+$32-$27] (used reg = )
mov	-$25[bp],al
!BCC_EOS
! 3577         mode = read_byte(ebda_seg, &((ebda_data_t *) 0)->ata.devices[device].mode);
! Debug: ptradd unsigned char device = [S+$32-$23] to [8] struct  = const $142 (used reg = )
mov	al,-$21[bp]
xor	ah,ah
mov	cx,*$1A
imul	cx
mov	bx,ax
! Debug: address unsigned char = [bx+$146] (used reg = )
! Debug: list * unsigned char = bx+$146 (used reg = )
add	bx,#$146
push	bx
! Debug: list unsigned short ebda_seg = [S+$34-8] (used reg = )
push	-6[bp]
! Debug: func () unsigned char = read_byte+0 (used reg = )
call	_read_byte
add	sp,*4
! Debug: eq unsigned char = al+0 to unsigned char mode = [S+$32-$28] (used reg = )
mov	-$26[bp],al
!BCC_EOS
! 3578         translation = read_byte(ebda_seg, &((ebda_data_t *) 0)->ata.devices[device].translation);
! Debug: ptradd unsigned char device = [S+$32-$23] to [8] struct  = const $142 (used reg = )
mov	al,-$21[bp]
xor	ah,ah
mov	cx,*$1A
imul	cx
mov	bx,ax
! Debug: address unsigned char = [bx+$14A] (used reg = )
! Debug: list * unsigned char = bx+$14A (used reg = )
add	bx,#$14A
push	bx
! Debug: list unsigned short ebda_seg = [S+$34-8] (used reg = )
push	-6[bp]
! Debug: func () unsigned char = read_byte+0 (used reg = )
call	_read_byte
add	sp,*4
! Debug: eq unsigned char = al+0 to unsigned char translation = [S+$32-$2B] (used reg = )
mov	-$29[bp],al
!BCC_EOS
! 3579         options = (translation==0?0:1<<3);
! Debug: logeq int = const 0 to unsigned char translation = [S+$32-$2B] (used reg = )
mov	al,-$29[bp]
test	al,al
jne 	.4DB
.4DC:
xor	al,al
jmp .4DD
.4DB:
mov	al,*8
.4DD:
! Debug: eq char = al+0 to unsigned short options = [S+$32-$32] (used reg = )
xor	ah,ah
mov	-$30[bp],ax
!BCC_EOS
! 3580         options |= (1<<4);
! Debug: orab int = const $10 to unsigned short options = [S+$32-$32] (used reg = )
mov	ax,-$30[bp]
or	al,*$10
mov	-$30[bp],ax
!BCC_EOS
! 3581         options |= (mode==0x01?1:0<<7);
! Debug: logeq int = const 1 to unsigned char mode = [S+$32-$28] (used reg = )
mov	al,-$26[bp]
cmp	al,*1
jne 	.4DE
.4DF:
mov	al,*1
jmp .4E0
.4DE:
xor	al,al
.4E0:
! Debug: orab char = al+0 to unsigned short options = [S+$32-$32] (used reg = )
xor	ah,ah
or	ax,-$30[bp]
mov	-$30[bp],ax
!BCC_EOS
! 3582         options |= (translation==1?1:0<<9);
! Debug: logeq int = const 1 to unsigned char translation = [S+$32-$2B] (used reg = )
mov	al,-$29[bp]
cmp	al,*1
jne 	.4E1
.4E2:
mov	al,*1
jmp .4E3
.4E1:
xor	al,al
.4E3:
! Debug: orab char = al+0 to unsigned short options = [S+$32-$32] (used reg = )
xor	ah,ah
or	ax,-$30[bp]
mov	-$30[bp],ax
!BCC_EOS
! 3583         options |= (translation==3?3:0<<9);
! Debug: logeq int = const 3 to unsigned char translation = [S+$32-$2B] (used reg = )
mov	al,-$29[bp]
cmp	al,*3
jne 	.4E4
.4E5:
mov	al,*3
jmp .4E6
.4E4:
xor	al,al
.4E6:
! Debug: orab char = al+0 to unsigned short options = [S+$32-$32] (used reg = )
xor	ah,ah
or	ax,-$30[bp]
mov	-$30[bp],ax
!BCC_EOS
! 3584         write_word(ebda_seg, &((ebda_data_t *) 0)->ata.dpte.iobase1, iobase1);
! Debug: list unsigned short iobase1 = [S+$32-$2E] (used reg = )
push	-$2C[bp]
! Debug: list * unsigned short = const $224 (used reg = )
mov	ax,#$224
push	ax
! Debug: list unsigned short ebda_seg = [S+$36-8] (used reg = )
push	-6[bp]
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
add	sp,*6
!BCC_EOS
! 3585         write_word(ebda_seg, &((ebda_data_t *) 0)->ata.dpte.iobase2, iobase2);
! Debug: list unsigned short iobase2 = [S+$32-$30] (used reg = )
push	-$2E[bp]
! Debug: list * unsigned short = const $226 (used reg = )
mov	ax,#$226
push	ax
! Debug: list unsigned short ebda_seg = [S+$36-8] (used reg = )
push	-6[bp]
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
add	sp,*6
!BCC_EOS
! 3586         write_byte(ebda_seg, &((ebda_data_t *) 0)->ata.dpte.prefix, (0xe | (device % 2))<<4 );
! Debug: mod int = const 2 to unsigned char device = [S+$32-$23] (used reg = )
mov	al,-$21[bp]
xor	ah,ah
and	al,*1
! Debug: or unsigned char = al+0 to int = const $E (used reg = )
! Debug: expression subtree swapping
or	al,*$E
! Debug: sl int = const 4 to unsigned char = al+0 (used reg = )
xor	ah,ah
mov	cl,*4
shl	ax,cl
! Debug: list unsigned int = ax+0 (used reg = )
push	ax
! Debug: list * unsigned char = const $228 (used reg = )
mov	ax,#$228
push	ax
! Debug: list unsigned short ebda_seg = [S+$36-8] (used reg = )
push	-6[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 3587         write_byte(ebda_seg, &((ebda_data_t *) 0)->ata.dpte.unused, 0xcb );
! Debug: list int = const $CB (used reg = )
mov	ax,#$CB
push	ax
! Debug: list * unsigned char = const $229 (used reg = )
mov	ax,#$229
push	ax
! Debug: list unsigned short ebda_seg = [S+$36-8] (used reg = )
push	-6[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 3588         write_byte(ebda_seg, &((ebda_data_t *) 0)->ata.dpte.irq, irq );
! Debug: list unsigned char irq = [S+$32-$27] (used reg = )
mov	al,-$25[bp]
xor	ah,ah
push	ax
! Debug: list * unsigned char = const $22A (used reg = )
mov	ax,#$22A
push	ax
! Debug: list unsigned short ebda_seg = [S+$36-8] (used reg = )
push	-6[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 3589         write_byte(ebda_seg, &((ebda_data_t *) 0)->ata.dpte.blkcount, 1 );
! Debug: list int = const 1 (used reg = )
mov	ax,*1
push	ax
! Debug: list * unsigned char = const $22B (used reg = )
mov	ax,#$22B
push	ax
! Debug: list unsigned short ebda_seg = [S+$36-8] (used reg = )
push	-6[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 3590         write_byte(ebda_seg, &((ebda_data_t *) 0)->ata.dpte.dma, 0 );
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: list * unsigned char = const $22C (used reg = )
mov	ax,#$22C
push	ax
! Debug: list unsigned short ebda_seg = [S+$36-8] (used reg = )
push	-6[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 3591         write_byte(ebda_seg, &((ebda_data_t *) 0)->ata.dpte.pio, 0 );
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: list * unsigned char = const $22D (used reg = )
mov	ax,#$22D
push	ax
! Debug: list unsigned short ebda_seg = [S+$36-8] (used reg = )
push	-6[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 3592         write_word(ebda_seg, &((ebda_data_t *) 0)->ata.dpte.options, options);
! Debug: list unsigned short options = [S+$32-$32] (used reg = )
push	-$30[bp]
! Debug: list * unsigned short = const $22E (used reg = )
mov	ax,#$22E
push	ax
! Debug: list unsigned short ebda_seg = [S+$36-8] (used reg = )
push	-6[bp]
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
add	sp,*6
!BCC_EOS
! 3593         write_word(ebda_seg, &((ebda_data_t *) 0)->ata.dpte.reserved, 0);
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: list * unsigned short = const $230 (used reg = )
mov	ax,#$230
push	ax
! Debug: list unsigned short ebda_seg = [S+$36-8] (used reg = )
push	-6[bp]
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
add	sp,*6
!BCC_EOS
! 3594         write_byte(ebda_seg, &((ebda_data_t *) 0)->ata.dpte.revision, 0x11);
! Debug: list int = const $11 (used reg = )
mov	ax,*$11
push	ax
! Debug: list * unsigned char = const $232 (used reg = )
mov	ax,#$232
push	ax
! Debug: list unsigned short ebda_seg = [S+$36-8] (used reg = )
push	-6[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 3595         checksum=0;
! Debug: eq int = const 0 to unsigned char checksum = [S+$32-$29] (used reg = )
xor	al,al
mov	-$27[bp],al
!BCC_EOS
! 3596         for (i=0; i<15; i++) checksum+=read_byte(ebda_seg, (&((ebda_data_t *) 0)->ata.dpte) + i);
! Debug: eq int = const 0 to unsigned char i = [S+$32-$2A] (used reg = )
xor	al,al
mov	-$28[bp],al
!BCC_EOS
!BCC_EOS
jmp .4E9
.4EA:
! Debug: ptradd unsigned char i = [S+$32-$2A] to * struct  = const $224 (used reg = )
mov	al,-$28[bp]
xor	ah,ah
mov	cl,*4
shl	ax,cl
! Debug: list * struct  = ax+$224 (used reg = )
add	ax,#$224
push	ax
! Debug: list unsigned short ebda_seg = [S+$34-8] (used reg = )
push	-6[bp]
! Debug: func () unsigned char = read_byte+0 (used reg = )
call	_read_byte
add	sp,*4
! Debug: addab unsigned char = al+0 to unsigned char checksum = [S+$32-$29] (used reg = )
xor	ah,ah
add	al,-$27[bp]
adc	ah,*0
mov	-$27[bp],al
!BCC_EOS
! 3597         checksum = ~checksum;
.4E8:
! Debug: postinc unsigned char i = [S+$32-$2A] (used reg = )
mov	al,-$28[bp]
inc	ax
mov	-$28[bp],al
.4E9:
! Debug: lt int = const $F to unsigned char i = [S+$32-$2A] (used reg = )
mov	al,-$28[bp]
cmp	al,*$F
jb 	.4EA
.4EB:
.4E7:
! Debug: not unsigned char checksum = [S+$32-$29] (used reg = )
mov	al,-$27[bp]
xor	ah,ah
not	ax
! Debug: eq unsigned int = ax+0 to unsigned char checksum = [S+$32-$29] (used reg = )
mov	-$27[bp],al
!BCC_EOS
! 3598       
! 3598   write_byte(ebda_seg, &((ebda_data_t *) 0)->ata.dpte.checksum, checksum);
! Debug: list unsigned char checksum = [S+$32-$29] (used reg = )
mov	al,-$27[bp]
xor	ah,ah
push	ax
! Debug: list * unsigned char = const $233 (used reg = )
mov	ax,#$233
push	ax
! Debug: list unsigned short ebda_seg = [S+$36-8] (used reg = )
push	-6[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 3599         }
! 3600       if(size >= 0x42) {
.4D9:
! Debug: ge int = const $42 to unsigned short size = [S+$24-$20] (used reg = )
mov	ax,-$1E[bp]
cmp	ax,*$42
blo 	.4EC
.4ED:
! 3601         Bit8u channel, iface, checksum, i;
!BCC_EOS
! 3602         Bit16u iobase1;
!BCC_EOS
! 3603         channel = device / 2;
! Debug: div int = const 2 to unsigned char device = [S+$2A-$23] (used reg = )
mov	al,-$21[bp]
xor	ah,ah
shr	ax,*1
! Debug: eq unsigned int = ax+0 to unsigned char channel = [S+$2A-$25] (used reg = )
mov	-$23[bp],al
!BCC_EOS
! 3604         iface = read_byte(ebda_seg, &((ebda_data_t *) 0)->ata.channels[channel].iface);
! Debug: ptradd unsigned char channel = [S+$2A-$25] to [4] struct  = const $122 (used reg = )
mov	al,-$23[bp]
xor	ah,ah
mov	cl,*3
shl	ax,cl
mov	bx,ax
! Debug: address unsigned char = [bx+$122] (used reg = )
! Debug: list * unsigned char = bx+$122 (used reg = )
add	bx,#$122
push	bx
! Debug: list unsigned short ebda_seg = [S+$2C-8] (used reg = )
push	-6[bp]
! Debug: func () unsigned char = read_byte+0 (used reg = )
call	_read_byte
add	sp,*4
! Debug: eq unsigned char = al+0 to unsigned char iface = [S+$2A-$26] (used reg = )
mov	-$24[bp],al
!BCC_EOS
! 3605         iobase1 = read_word(ebda_seg, &((ebda_data_t *) 0)->ata.channels[channel].iobase1);
! Debug: ptradd unsigned char channel = [S+$2A-$25] to [4] struct  = const $122 (used reg = )
mov	al,-$23[bp]
xor	ah,ah
mov	cl,*3
shl	ax,cl
mov	bx,ax
! Debug: address unsigned short = [bx+$124] (used reg = )
! Debug: list * unsigned short = bx+$124 (used reg = )
add	bx,#$124
push	bx
! Debug: list unsigned short ebda_seg = [S+$2C-8] (used reg = )
push	-6[bp]
! Debug: func () unsigned short = read_word+0 (used reg = )
call	_read_word
add	sp,*4
! Debug: eq unsigned short = ax+0 to unsigned short iobase1 = [S+$2A-$2A] (used reg = )
mov	-$28[bp],ax
!BCC_EOS
! 3606         write_word(DS, SI+(Bit16u)&((dpt_t *) 0)->size, 0x42);
! Debug: list int = const $42 (used reg = )
mov	ax,*$42
push	ax
! Debug: add unsigned short = const 0 to unsigned short SI = [S+$2C+8] (used reg = )
mov	ax,$A[bp]
! Debug: list unsigned int = ax+0 (used reg = )
push	ax
! Debug: list unsigned short DS = [S+$2E+2] (used reg = )
push	4[bp]
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
add	sp,*6
!BCC_EOS
! 3607         write_word(DS, SI+(Bit16u)&((dpt_t *) 0)->key, 0xbedd);
! Debug: list unsigned int = const $BEDD (used reg = )
mov	ax,#$BEDD
push	ax
! Debug: add unsigned short = const $1E to unsigned short SI = [S+$2C+8] (used reg = )
mov	ax,$A[bp]
! Debug: list unsigned int = ax+$1E (used reg = )
add	ax,*$1E
push	ax
! Debug: list unsigned short DS = [S+$2E+2] (used reg = )
push	4[bp]
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
add	sp,*6
!BCC_EOS
! 3608         write_byte(DS, SI+(Bit16u)&((dpt_t *) 0)->dpi_length, 0x24);
! Debug: list int = const $24 (used reg = )
mov	ax,*$24
push	ax
! Debug: add unsigned short = const $20 to unsigned short SI = [S+$2C+8] (used reg = )
mov	ax,$A[bp]
! Debug: list unsigned int = ax+$20 (used reg = )
add	ax,*$20
push	ax
! Debug: list unsigned short DS = [S+$2E+2] (used reg = )
push	4[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 3609         write_byte(DS, SI+(Bit16u)&((dpt_t *) 0)->reserved1, 0);
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: add unsigned short = const $21 to unsigned short SI = [S+$2C+8] (used reg = )
mov	ax,$A[bp]
! Debug: list unsigned int = ax+$21 (used reg = )
add	ax,*$21
push	ax
! Debug: list unsigned short DS = [S+$2E+2] (used reg = )
push	4[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 3610         write_word(DS, SI+(Bit16u)&((dpt_t *) 0)->reserved2, 0);
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: add unsigned short = const $22 to unsigned short SI = [S+$2C+8] (used reg = )
mov	ax,$A[bp]
! Debug: list unsigned int = ax+$22 (used reg = )
add	ax,*$22
push	ax
! Debug: list unsigned short DS = [S+$2E+2] (used reg = )
push	4[bp]
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
add	sp,*6
!BCC_EOS
! 3611         if (iface==0x00) {
! Debug: logeq int = const 0 to unsigned char iface = [S+$2A-$26] (used reg = )
mov	al,-$24[bp]
test	al,al
jne 	.4EE
.4EF:
! 3612           write_byte(DS, SI+(Bit16u)&((dpt_t *) 0)->host_bus[0], 'I');
! Debug: list int = const $49 (used reg = )
mov	ax,*$49
push	ax
! Debug: add unsigned short = const $24 to unsigned short SI = [S+$2C+8] (used reg = )
mov	ax,$A[bp]
! Debug: list unsigned int = ax+$24 (used reg = )
add	ax,*$24
push	ax
! Debug: list unsigned short DS = [S+$2E+2] (used reg = )
push	4[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 3613           write_byte(DS, SI+(Bit16u)&((dpt_t *) 0)->host_bus[1], 'S');
! Debug: list int = const $53 (used reg = )
mov	ax,*$53
push	ax
! Debug: add unsigned short = const $25 to unsigned short SI = [S+$2C+8] (used reg = )
mov	ax,$A[bp]
! Debug: list unsigned int = ax+$25 (used reg = )
add	ax,*$25
push	ax
! Debug: list unsigned short DS = [S+$2E+2] (used reg = )
push	4[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 3614           write_byte(DS, SI+(Bit16u)&((dpt_t *) 0)->host_bus[2], 'A');
! Debug: list int = const $41 (used reg = )
mov	ax,*$41
push	ax
! Debug: add unsigned short = const $26 to unsigned short SI = [S+$2C+8] (used reg = )
mov	ax,$A[bp]
! Debug: list unsigned int = ax+$26 (used reg = )
add	ax,*$26
push	ax
! Debug: list unsigned short DS = [S+$2E+2] (used reg = )
push	4[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 3615           write_byte(DS, SI+(Bit16u)&((dpt_t *) 0)->host_bus[3], 0);
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: add unsigned short = const $27 to unsigned short SI = [S+$2C+8] (used reg = )
mov	ax,$A[bp]
! Debug: list unsigned int = ax+$27 (used reg = )
add	ax,*$27
push	ax
! Debug: list unsigned short DS = [S+$2E+2] (used reg = )
push	4[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 3616           }
! 3617         else {
jmp .4F0
.4EE:
! 3618           }
! 3619         write_byte(DS, SI+(Bit16u)&((dpt_t *) 0)->iface_type[0], 'A');
.4F0:
! Debug: list int = const $41 (used reg = )
mov	ax,*$41
push	ax
! Debug: add unsigned short = const $28 to unsigned short SI = [S+$2C+8] (used reg = )
mov	ax,$A[bp]
! Debug: list unsigned int = ax+$28 (used reg = )
add	ax,*$28
push	ax
! Debug: list unsigned short DS = [S+$2E+2] (used reg = )
push	4[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 3620         write_byte(DS, SI+(Bit16u)&((dpt_t *) 0)->iface_type[1], 'T');
! Debug: list int = const $54 (used reg = )
mov	ax,*$54
push	ax
! Debug: add unsigned short = const $29 to unsigned short SI = [S+$2C+8] (used reg = )
mov	ax,$A[bp]
! Debug: list unsigned int = ax+$29 (used reg = )
add	ax,*$29
push	ax
! Debug: list unsigned short DS = [S+$2E+2] (used reg = )
push	4[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 3621         write_byte(DS, SI+(Bit16u)&((dpt_t *) 0)->iface_type[2], 'A');
! Debug: list int = const $41 (used reg = )
mov	ax,*$41
push	ax
! Debug: add unsigned short = const $2A to unsigned short SI = [S+$2C+8] (used reg = )
mov	ax,$A[bp]
! Debug: list unsigned int = ax+$2A (used reg = )
add	ax,*$2A
push	ax
! Debug: list unsigned short DS = [S+$2E+2] (used reg = )
push	4[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 3622         write_byte(DS, SI+(Bit16u)&((dpt_t *) 0)->iface_type[3], 0);
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: add unsigned short = const $2B to unsigned short SI = [S+$2C+8] (used reg = )
mov	ax,$A[bp]
! Debug: list unsigned int = ax+$2B (used reg = )
add	ax,*$2B
push	ax
! Debug: list unsigned short DS = [S+$2E+2] (used reg = )
push	4[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 3623         if (iface==0x00) {
! Debug: logeq int = const 0 to unsigned char iface = [S+$2A-$26] (used reg = )
mov	al,-$24[bp]
test	al,al
jne 	.4F1
.4F2:
! 3624           write_word(DS, SI+(Bit16u)&((dpt_t *) 0)->iface_path[0], iobase1);
! Debug: list unsigned short iobase1 = [S+$2A-$2A] (used reg = )
push	-$28[bp]
! Debug: add unsigned short = const $30 to unsigned short SI = [S+$2C+8] (used reg = )
mov	ax,$A[bp]
! Debug: list unsigned int = ax+$30 (used reg = )
add	ax,*$30
push	ax
! Debug: list unsigned short DS = [S+$2E+2] (used reg = )
push	4[bp]
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
add	sp,*6
!BCC_EOS
! 3625           write_word(DS, SI+(Bit16u)&((dpt_t *) 0)->iface_path[2], 0);
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: add unsigned short = const $32 to unsigned short SI = [S+$2C+8] (used reg = )
mov	ax,$A[bp]
! Debug: list unsigned int = ax+$32 (used reg = )
add	ax,*$32
push	ax
! Debug: list unsigned short DS = [S+$2E+2] (used reg = )
push	4[bp]
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
add	sp,*6
!BCC_EOS
! 3626           write_dword(DS, SI+(Bit16u)&((dpt_t *) 0)->iface_path[4], 0L);
! Debug: list long = const 0 (used reg = )
xor	ax,ax
xor	bx,bx
push	bx
push	ax
! Debug: add unsigned short = const $34 to unsigned short SI = [S+$2E+8] (used reg = )
mov	ax,$A[bp]
! Debug: list unsigned int = ax+$34 (used reg = )
add	ax,*$34
push	ax
! Debug: list unsigned short DS = [S+$30+2] (used reg = )
push	4[bp]
! Debug: func () void = write_dword+0 (used reg = )
call	_write_dword
add	sp,*8
!BCC_EOS
! 3627           }
! 3628         else {
jmp .4F3
.4F1:
! 3629           }
! 3630         write_byte(DS, SI+(Bit16u)&((dpt_t *) 0)->device_path[0], device%2);
.4F3:
! Debug: mod int = const 2 to unsigned char device = [S+$2A-$23] (used reg = )
mov	al,-$21[bp]
xor	ah,ah
and	al,*1
! Debug: list unsigned char = al+0 (used reg = )
xor	ah,ah
push	ax
! Debug: add unsigned short = const $38 to unsigned short SI = [S+$2C+8] (used reg = )
mov	ax,$A[bp]
! Debug: list unsigned int = ax+$38 (used reg = )
add	ax,*$38
push	ax
! Debug: list unsigned short DS = [S+$2E+2] (used reg = )
push	4[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 3631         write_byte(DS, SI+(Bit16u)&((dpt_t *) 0)->device_path[1], 0);
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: add unsigned short = const $39 to unsigned short SI = [S+$2C+8] (used reg = )
mov	ax,$A[bp]
! Debug: list unsigned int = ax+$39 (used reg = )
add	ax,*$39
push	ax
! Debug: list unsigned short DS = [S+$2E+2] (used reg = )
push	4[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 3632         write_word(DS, SI+(Bit16u)&((dpt_t *) 0)->device_path[2], 0);
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: add unsigned short = const $3A to unsigned short SI = [S+$2C+8] (used reg = )
mov	ax,$A[bp]
! Debug: list unsigned int = ax+$3A (used reg = )
add	ax,*$3A
push	ax
! Debug: list unsigned short DS = [S+$2E+2] (used reg = )
push	4[bp]
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
add	sp,*6
!BCC_EOS
! 3633         write_dword(DS, SI+(Bit16u)&((dpt_t *) 0)->device_path[4], 0L);
! Debug: list long = const 0 (used reg = )
xor	ax,ax
xor	bx,bx
push	bx
push	ax
! Debug: add unsigned short = const $3C to unsigned short SI = [S+$2E+8] (used reg = )
mov	ax,$A[bp]
! Debug: list unsigned int = ax+$3C (used reg = )
add	ax,*$3C
push	ax
! Debug: list unsigned short DS = [S+$30+2] (used reg = )
push	4[bp]
! Debug: func () void = write_dword+0 (used reg = )
call	_write_dword
add	sp,*8
!BCC_EOS
! 3634         checksum=0;
! Debug: eq int = const 0 to unsigned char checksum = [S+$2A-$27] (used reg = )
xor	al,al
mov	-$25[bp],al
!BCC_EOS
! 3635         for (i=30; i<64; i++) checksum+=read_byte(DS, SI + i);
! Debug: eq int = const $1E to unsigned char i = [S+$2A-$28] (used reg = )
mov	al,*$1E
mov	-$26[bp],al
!BCC_EOS
!BCC_EOS
jmp .4F6
.4F7:
! Debug: add unsigned char i = [S+$2A-$28] to unsigned short SI = [S+$2A+8] (used reg = )
mov	ax,$A[bp]
add	al,-$26[bp]
adc	ah,*0
! Debug: list unsigned int = ax+0 (used reg = )
push	ax
! Debug: list unsigned short DS = [S+$2C+2] (used reg = )
push	4[bp]
! Debug: func () unsigned char = read_byte+0 (used reg = )
call	_read_byte
add	sp,*4
! Debug: addab unsigned char = al+0 to unsigned char checksum = [S+$2A-$27] (used reg = )
xor	ah,ah
add	al,-$25[bp]
adc	ah,*0
mov	-$25[bp],al
!BCC_EOS
! 3636         checksum = ~checksum;
.4F5:
! Debug: postinc unsigned char i = [S+$2A-$28] (used reg = )
mov	al,-$26[bp]
inc	ax
mov	-$26[bp],al
.4F6:
! Debug: lt int = const $40 to unsigned char i = [S+$2A-$28] (used reg = )
mov	al,-$26[bp]
cmp	al,*$40
jb 	.4F7
.4F8:
.4F4:
! Debug: not unsigned char checksum = [S+$2A-$27] (used reg = )
mov	al,-$25[bp]
xor	ah,ah
not	ax
! Debug: eq unsigned int = ax+0 to unsigned char checksum = [S+$2A-$27] (used reg = )
mov	-$25[bp],al
!BCC_EOS
! 3637         write_by
! 3637 te(DS, SI+(Bit16u)&((dpt_t *) 0)->checksum, checksum);
! Debug: list unsigned char checksum = [S+$2A-$27] (used reg = )
mov	al,-$25[bp]
xor	ah,ah
push	ax
! Debug: add unsigned short = const $41 to unsigned short SI = [S+$2C+8] (used reg = )
mov	ax,$A[bp]
! Debug: list unsigned int = ax+$41 (used reg = )
add	ax,*$41
push	ax
! Debug: list unsigned short DS = [S+$2E+2] (used reg = )
push	4[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 3638         }
! 3639       goto int13_success;
.4EC:
add	sp,#..FFEE-..FFEF
br 	.FFEE
!BCC_EOS
! 3640       break;
br 	.496
!BCC_EOS
! 3641     case 0x4e:
! 3642       switch (( AX & 0x00ff )) {
.4F9:
! Debug: and int = const $FF to unsigned short AX = [S+$24+$14] (used reg = )
mov	al,$16[bp]
jmp .4FC
! 3643         case 0x01:
! 3644         case 0x03:
.4FD:
! 3645         case 0x04:
.4FE:
! 3646         case 0x06:
.4FF:
! 3647           goto int13_success;
.500:
add	sp,#..FFEE-..FFEF
br 	.FFEE
!BCC_EOS
! 3648           break;
jmp .4FA
!BCC_EOS
! 3649         default :
! 3650           goto int13_fail;
.501:
add	sp,#..FFF0-..FFEF
br 	.FFF0
!BCC_EOS
! 3651         }
! 3652       break;
jmp .4FA
.4FC:
sub	al,*1
je 	.4FD
sub	al,*2
je 	.4FE
sub	al,*1
je 	.4FF
sub	al,*2
je 	.500
jmp	.501
.4FA:
br 	.496
!BCC_EOS
! 3653     case 0x09:
! 3654     case 0x0c:
.502:
! 3655     case 0x0d:
.503:
! 3656     case 0x11:
.504:
! 3657     case 0x14:
.505:
! 3658       bios_printf(4, "int13h_harddisk function %02xh unimplemented, returns success\n", ( AX >> 8 ));
.506:
! Debug: sr int = const 8 to unsigned short AX = [S+$24+$14] (used reg = )
mov	ax,$16[bp]
mov	al,ah
xor	ah,ah
! Debug: list unsigned int = ax+0 (used reg = )
push	ax
! Debug: list * char = .507+0 (used reg = )
mov	bx,#.507
push	bx
! Debug: list int = const 4 (used reg = )
mov	ax,*4
push	ax
! Debug: func () void = bios_printf+0 (used reg = )
call	_bios_printf
add	sp,*6
!BCC_EOS
! 3659       goto int13_success;
add	sp,#..FFEE-..FFEF
br 	.FFEE
!BCC_EOS
! 3660       break;
br 	.496
!BCC_EOS
! 3661     case 0x0a:
! 3662     case 0x0b:
.508:
! 3663     case 0x18:
.509:
! 3664     case 0x50:
.50A:
! 3665     default:
.50B:
! 3666       bios_printf(4, "int13_harddisk function %02xh unsupported, returns fail\n", ( AX >> 8 ));
.50C:
! Debug: sr int = const 8 to unsigned short AX = [S+$24+$14] (used reg = )
mov	ax,$16[bp]
mov	al,ah
xor	ah,ah
! Debug: list unsigned int = ax+0 (used reg = )
push	ax
! Debug: list * char = .50D+0 (used reg = )
mov	bx,#.50D
push	bx
! Debug: list int = const 4 (used reg = )
mov	ax,*4
push	ax
! Debug: func () void = bios_printf+0 (used reg = )
call	_bios_printf
add	sp,*6
!BCC_EOS
! 3667       goto int13_fail;
add	sp,#..FFF0-..FFEF
br 	.FFF0
!BCC_EOS
! 3668       break;
br 	.496
!BCC_EOS
! 3669     }
! 3670 int13_fail:
jmp .496
.498:
add	sp,*-$E
sub	ax,*0
jl 	.50C
cmp	ax,*$18
ja  	.50E
shl	ax,*1
mov	bx,ax
seg	cs
br	.50F[bx]
.50F:
.word	.499
.word	.49A
.word	.49E
.word	.49F
.word	.4A0
.word	.4B5
.word	.50C
.word	.50C
.word	.4B7
.word	.502
.word	.508
.word	.509
.word	.503
.word	.504
.word	.50C
.word	.50C
.word	.4B8
.word	.505
.word	.50C
.word	.50C
.word	.506
.word	.4BC
.word	.50C
.word	.50C
.word	.50A
.50E:
sub	ax,*$41
jb 	.50C
cmp	ax,*$F
ja  	.510
shl	ax,*1
mov	bx,ax
seg	cs
br	.511[bx]
.511:
.word	.4BD
.word	.4BE
.word	.4BF
.word	.4C0
.word	.4D1
.word	.4D3
.word	.4C1
.word	.4D4
.word	.4D2
.word	.50C
.word	.50C
.word	.50C
.word	.50C
.word	.4F9
.word	.50C
.word	.50B
.510:
br 	.50C
.496:
..FFEF	=	-$32
add	sp,*$E
.FFF0:
..FFF0	=	-$24
! 3671     AX = ((AX & 0x00ff) | ((0x01) << 8));
! Debug: and int = const $FF to unsigned short AX = [S+$24+$14] (used reg = )
mov	al,$16[bp]
! Debug: or int = const $100 to unsigned char = al+0 (used reg = )
xor	ah,ah
or	ax,#$100
! Debug: eq unsigned int = ax+0 to unsigned short AX = [S+$24+$14] (used reg = )
mov	$16[bp],ax
!BCC_EOS
! 3672 int13_fail_noah:
.FFEB:
..FFEB	=	-$24
! 3673     write_byte(0x0040, 0x0074, ( AX >> 8 ));
! Debug: sr int = const 8 to unsigned short AX = [S+$24+$14] (used reg = )
mov	ax,$16[bp]
mov	al,ah
xor	ah,ah
! Debug: list unsigned int = ax+0 (used reg = )
push	ax
! Debug: list int = const $74 (used reg = )
mov	ax,*$74
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 3674 int13_fail_nostatus:
.FFED:
..FFED	=	-$24
! 3675     FLAGS |= 0x0001;
! Debug: orab int = const 1 to unsigned short FLAGS = [S+$24+$1A] (used reg = )
mov	ax,$1C[bp]
or	al,*1
mov	$1C[bp],ax
!BCC_EOS
! 3676     return;
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 3677 int13_success:
.FFEE:
..FFEE	=	-$24
! 3678     AX = ((AX & 0x00ff) | ((0x00) << 8));
! Debug: and int = const $FF to unsigned short AX = [S+$24+$14] (used reg = )
mov	al,$16[bp]
! Debug: or int = const 0 to unsigned char = al+0 (used reg = )
or	al,*0
! Debug: eq unsigned char = al+0 to unsigned short AX = [S+$24+$14] (used reg = )
xor	ah,ah
mov	$16[bp],ax
!BCC_EOS
! 3679 int13_success_noah:
.FFEC:
..FFEC	=	-$24
! 3680     write_byte(0x0040, 0x0074, 0x00);
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: list int = const $74 (used reg = )
mov	ax,*$74
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 3681     FLAGS &= 0xfffe;
! Debug: andab unsigned int = const $FFFE to unsigned short FLAGS = [S+$24+$1A] (used reg = )
mov	ax,$1C[bp]
and	al,#$FE
mov	$1C[bp],ax
!BCC_EOS
! 3682     return;
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 3683 }
! 3684   void
! Register BX used in function int13_harddisk
! 3685 int13_cdrom(EHBX, DS, ES, DI, SI, BP, ELDX, BX, DX, CX, AX, IP, CS, FLAGS)
! 3686   Bit16u EHBX, DS, ES, DI, SI, BP, ELDX, BX, DX, CX, AX, IP, CS, FLAGS;
export	_int13_cdrom
_int13_cdrom:
!BCC_EOS
! 3687 {
! 3688   Bit16u ebda_seg=read_word(0x0040,0x000E);
push	bp
mov	bp,sp
dec	sp
dec	sp
! Debug: list int = const $E (used reg = )
mov	ax,*$E
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () unsigned short = read_word+0 (used reg = )
call	_read_word
add	sp,*4
! Debug: eq unsigned short = ax+0 to unsigned short ebda_seg = [S+4-4] (used reg = )
mov	-2[bp],ax
!BCC_EOS
! 3689   Bit8u device, status, locks;
!BCC_EOS
! 3690   Bit8u atacmd[12];
!BCC_EOS
! 3691   Bit32u lba;
!BCC_EOS
! 3692   Bit16u count, segment, offset, i, size;
!BCC_EOS
! 3693   ;
add	sp,*-$1E
!BCC_EOS
! 3694   write_byte(0x0040, 0x0074, 0x00);
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: list int = const $74 (used reg = )
mov	ax,*$74
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 3695   if( (( ELDX & 0x00ff ) < 0xE0) || (( ELDX & 0x00ff ) >= 0xE0+(4*2)) ) {
! Debug: and int = const $FF to unsigned short ELDX = [S+$22+$E] (used reg = )
mov	al,$10[bp]
! Debug: lt int = const $E0 to unsigned char = al+0 (used reg = )
cmp	al,#$E0
jb  	.513
.514:
! Debug: and int = const $FF to unsigned short ELDX = [S+$22+$E] (used reg = )
mov	al,$10[bp]
! Debug: ge int = const $E8 to unsigned char = al+0 (used reg = )
cmp	al,#$E8
jb  	.512
.513:
! 3696     bios_printf(4, "int13_cdrom: function %02x, ELDL out of range %02x\n", ( AX >> 8 ), ( ELDX & 0x00ff ));
! Debug: and int = const $FF to unsigned short ELDX = [S+$22+$E] (used reg = )
mov	al,$10[bp]
! Debug: list unsigned char = al+0 (used reg = )
xor	ah,ah
push	ax
! Debug: sr int = const 8 to unsigned short AX = [S+$24+$16] (used reg = )
mov	ax,$18[bp]
mov	al,ah
xor	ah,ah
! Debug: list unsigned int = ax+0 (used reg = )
push	ax
! Debug: list * char = .515+0 (used reg = )
mov	bx,#.515
push	bx
! Debug: list int = const 4 (used reg = )
mov	ax,*4
push	ax
! Debug: func () void = bios_printf+0 (used reg = )
call	_bios_printf
add	sp,*8
!BCC_EOS
! 3697     goto int13_fail;
add	sp,#..FFEA+$22
br 	.FFEA
!BCC_EOS
! 3698     }
! 3699   device=read_byte(ebda_seg,&((ebda_data_t *) 0)->ata.cdidmap[( ELDX & 0x00ff )-0xE0]);
.512:
! Debug: and int = const $FF to unsigned short ELDX = [S+$22+$E] (used reg = )
mov	al,$10[bp]
! Debug: sub int = const $E0 to unsigned char = al+0 (used reg = )
xor	ah,ah
! Debug: ptradd unsigned int = ax-$E0 to [8] unsigned char = const $21C (used reg = )
add	ax,#-$E0
mov	bx,ax
! Debug: address unsigned char = [bx+$21C] (used reg = )
! Debug: list * unsigned char = bx+$21C (used reg = )
add	bx,#$21C
push	bx
! Debug: list unsigned short ebda_seg = [S+$24-4] (used reg = )
push	-2[bp]
! Debug: func () unsigned char = read_byte+0 (used reg = )
call	_read_byte
add	sp,*4
! Debug: eq unsigned char = al+0 to unsigned char device = [S+$22-5] (used reg = )
mov	-3[bp],al
!BCC_EOS
! 3700   if (device >= (4*2)) {
! Debug: ge int = const 8 to unsigned char device = [S+$22-5] (used reg = )
mov	al,-3[bp]
cmp	al,*8
jb  	.516
.517:
! 3701     bios_printf(4, "int13_cdrom: function %02x, unmapped device for ELDL=%02x\n", ( AX >> 8 ), ( ELDX & 0x00ff ));
! Debug: and int = const $FF to unsigned short ELDX = [S+$22+$E] (used reg = )
mov	al,$10[bp]
! Debug: list unsigned char = al+0 (used reg = )
xor	ah,ah
push	ax
! Debug: sr int = const 8 to unsigned short AX = [S+$24+$16] (used reg = )
mov	ax,$18[bp]
mov	al,ah
xor	ah,ah
! Debug: list unsigned int = ax+0 (used reg = )
push	ax
! Debug: list * char = .518+0 (used reg = )
mov	bx,#.518
push	bx
! Debug: list int = const 4 (used reg = )
mov	ax,*4
push	ax
! Debug: func () void = bios_printf+0 (used reg = )
call	_bios_printf
add	sp,*8
!BCC_EOS
! 3702     goto int13_fail;
add	sp,#..FFEA+$22
br 	.FFEA
!BCC_EOS
! 3703     }
! 3704   switch (( AX >> 8 )) {
.516:
! Debug: sr int = const 8 to unsigned short AX = [S+$22+$16] (used reg = )
mov	ax,$18[bp]
mov	al,ah
xor	ah,ah
br 	.51B
! 3705     case 0x00:
! 3706     case 0x09:
.51C:
! 3707     case 0x0c:
.51D:
! 3708     case 0x0d:
.51E:
! 3709     case 0x10:
.51F:
! 3710     case 0x11:
.520:
! 3711     case 0x14:
.521:
! 3712     case 0x16:
.522:
! 3713   
! 3713     goto int13_success;
.523:
add	sp,#..FFE8-..FFE9
br 	.FFE8
!BCC_EOS
! 3714       break;
br 	.519
!BCC_EOS
! 3715     case 0x03:
! 3716     case 0x05:
.524:
! 3717     case 0x43:
.525:
! 3718       AX = ((AX & 0x00ff) | ((0x03) << 8));
.526:
! Debug: and int = const $FF to unsigned short AX = [S+$22+$16] (used reg = )
mov	al,$18[bp]
! Debug: or int = const $300 to unsigned char = al+0 (used reg = )
xor	ah,ah
or	ax,#$300
! Debug: eq unsigned int = ax+0 to unsigned short AX = [S+$22+$16] (used reg = )
mov	$18[bp],ax
!BCC_EOS
! 3719       goto int13_fail_noah;
add	sp,#..FFE7-..FFE9
br 	.FFE7
!BCC_EOS
! 3720       break;
br 	.519
!BCC_EOS
! 3721     case 0x01:
! 3722       status = read_byte(0x0040, 0x0074);
.527:
! Debug: list int = const $74 (used reg = )
mov	ax,*$74
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () unsigned char = read_byte+0 (used reg = )
call	_read_byte
add	sp,*4
! Debug: eq unsigned char = al+0 to unsigned char status = [S+$22-6] (used reg = )
mov	-4[bp],al
!BCC_EOS
! 3723       AX = ((AX & 0x00ff) | ((status) << 8));
! Debug: sl int = const 8 to unsigned char status = [S+$22-6] (used reg = )
mov	al,-4[bp]
xor	ah,ah
mov	ah,al
xor	al,al
push	ax
! Debug: and int = const $FF to unsigned short AX = [S+$24+$16] (used reg = )
mov	al,$18[bp]
! Debug: or unsigned int (temp) = [S+$24-$24] to unsigned char = al+0 (used reg = )
xor	ah,ah
or	ax,0+..FFE9[bp]
inc	sp
inc	sp
! Debug: eq unsigned int = ax+0 to unsigned short AX = [S+$22+$16] (used reg = )
mov	$18[bp],ax
!BCC_EOS
! 3724       write_byte(0x0040, 0x0074, 0);
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: list int = const $74 (used reg = )
mov	ax,*$74
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 3725       if (status) goto int13_fail_nostatus;
mov	al,-4[bp]
test	al,al
je  	.528
.529:
add	sp,#..FFE6-..FFE9
br 	.FFE6
!BCC_EOS
! 3726       else goto int13_success_noah;
jmp .52A
.528:
add	sp,#..FFE5-..FFE9
br 	.FFE5
!BCC_EOS
! 3727       break;
.52A:
br 	.519
!BCC_EOS
! 3728     case 0x15:
! 3729       AX = ((AX & 0x00ff) | ((0x02) << 8));
.52B:
! Debug: and int = const $FF to unsigned short AX = [S+$22+$16] (used reg = )
mov	al,$18[bp]
! Debug: or int = const $200 to unsigned char = al+0 (used reg = )
xor	ah,ah
or	ax,#$200
! Debug: eq unsigned int = ax+0 to unsigned short AX = [S+$22+$16] (used reg = )
mov	$18[bp],ax
!BCC_EOS
! 3730       goto int13_fail_noah;
add	sp,#..FFE7-..FFE9
br 	.FFE7
!BCC_EOS
! 3731       break;
br 	.519
!BCC_EOS
! 3732     case 0x41:
! 3733       BX=0xaa55;
.52C:
! Debug: eq unsigned int = const $AA55 to unsigned short BX = [S+$22+$10] (used reg = )
mov	ax,#$AA55
mov	$12[bp],ax
!BCC_EOS
! 3734       AX = ((AX & 0x00ff) | ((0x30) << 8));
! Debug: and int = const $FF to unsigned short AX = [S+$22+$16] (used reg = )
mov	al,$18[bp]
! Debug: or int = const $3000 to unsigned char = al+0 (used reg = )
xor	ah,ah
or	ax,#$3000
! Debug: eq unsigned int = ax+0 to unsigned short AX = [S+$22+$16] (used reg = )
mov	$18[bp],ax
!BCC_EOS
! 3735       CX=0x0007;
! Debug: eq int = const 7 to unsigned short CX = [S+$22+$14] (used reg = )
mov	ax,*7
mov	$16[bp],ax
!BCC_EOS
! 3736       goto int13_success_noah;
add	sp,#..FFE5-..FFE9
br 	.FFE5
!BCC_EOS
! 3737       break;
br 	.519
!BCC_EOS
! 3738     case 0x42:
! 3739     case 0x44:
.52D:
! 3740     case 0x47:
.52E:
! 3741       count=read_word(DS, SI+(Bit16u)&((int13ext_t *) 0)->count);
.52F:
! Debug: add unsigned short = const 2 to unsigned short SI = [S+$22+$A] (used reg = )
mov	ax,$C[bp]
! Debug: list unsigned int = ax+2 (used reg = )
inc	ax
inc	ax
push	ax
! Debug: list unsigned short DS = [S+$24+4] (used reg = )
push	6[bp]
! Debug: func () unsigned short = read_word+0 (used reg = )
call	_read_word
add	sp,*4
! Debug: eq unsigned short = ax+0 to unsigned short count = [S+$22-$1A] (used reg = )
mov	-$18[bp],ax
!BCC_EOS
! 3742       segment=read_word(DS, SI+(Bit16u)&((int13ext_t *) 0)->segment);
! Debug: add unsigned short = const 6 to unsigned short SI = [S+$22+$A] (used reg = )
mov	ax,$C[bp]
! Debug: list unsigned int = ax+6 (used reg = )
add	ax,*6
push	ax
! Debug: list unsigned short DS = [S+$24+4] (used reg = )
push	6[bp]
! Debug: func () unsigned short = read_word+0 (used reg = )
call	_read_word
add	sp,*4
! Debug: eq unsigned short = ax+0 to unsigned short segment = [S+$22-$1C] (used reg = )
mov	-$1A[bp],ax
!BCC_EOS
! 3743       offset=read_word(DS, SI+(Bit16u)&((int13ext_t *) 0)->offset);
! Debug: add unsigned short = const 4 to unsigned short SI = [S+$22+$A] (used reg = )
mov	ax,$C[bp]
! Debug: list unsigned int = ax+4 (used reg = )
add	ax,*4
push	ax
! Debug: list unsigned short DS = [S+$24+4] (used reg = )
push	6[bp]
! Debug: func () unsigned short = read_word+0 (used reg = )
call	_read_word
add	sp,*4
! Debug: eq unsigned short = ax+0 to unsigned short offset = [S+$22-$1E] (used reg = )
mov	-$1C[bp],ax
!BCC_EOS
! 3744       lba=read_dword(DS, SI+(Bit16u)&((int13ext_t *) 0)->lba2);
! Debug: add unsigned short = const $C to unsigned short SI = [S+$22+$A] (used reg = )
mov	ax,$C[bp]
! Debug: list unsigned int = ax+$C (used reg = )
add	ax,*$C
push	ax
! Debug: list unsigned short DS = [S+$24+4] (used reg = )
push	6[bp]
! Debug: func () unsigned long = read_dword+0 (used reg = )
call	_read_dword
mov	bx,dx
add	sp,*4
! Debug: eq unsigned long = bx+0 to unsigned long lba = [S+$22-$18] (used reg = )
mov	-$16[bp],ax
mov	-$14[bp],bx
!BCC_EOS
! 3745       if (lba != 0L) {
! Debug: ne long = const 0 to unsigned long lba = [S+$22-$18] (used reg = )
! Debug: expression subtree swapping
xor	ax,ax
xor	bx,bx
push	bx
push	ax
mov	ax,-$16[bp]
mov	bx,-$14[bp]
lea	di,-2+..FFE9[bp]
call	lcmpul
lea	sp,2+..FFE9[bp]
je  	.530
.531:
! 3746         bios_printf((2 | 4 | 1), "int13_cdrom: function %02x. Can't use 64bits lba\n",( AX >> 8 ));
! Debug: sr int = const 8 to unsigned short AX = [S+$22+$16] (used reg = )
mov	ax,$18[bp]
mov	al,ah
xor	ah,ah
! Debug: list unsigned int = ax+0 (used reg = )
push	ax
! Debug: list * char = .532+0 (used reg = )
mov	bx,#.532
push	bx
! Debug: list int = const 7 (used reg = )
mov	ax,*7
push	ax
! Debug: func () void = bios_printf+0 (used reg = )
call	_bios_printf
add	sp,*6
!BCC_EOS
! 3747         goto int13_fail;
add	sp,#..FFEA-..FFE9
br 	.FFEA
!BCC_EOS
! 3748         }
! 3749       lba=read_dword(DS, SI+(Bit16u)&((int13ext_t *) 0)->lba1);
.530:
! Debug: add unsigned short = const 8 to unsigned short SI = [S+$22+$A] (used reg = )
mov	ax,$C[bp]
! Debug: list unsigned int = ax+8 (used reg = )
add	ax,*8
push	ax
! Debug: list unsigned short DS = [S+$24+4] (used reg = )
push	6[bp]
! Debug: func () unsigned long = read_dword+0 (used reg = )
call	_read_dword
mov	bx,dx
add	sp,*4
! Debug: eq unsigned long = bx+0 to unsigned long lba = [S+$22-$18] (used reg = )
mov	-$16[bp],ax
mov	-$14[bp],bx
!BCC_EOS
! 3750       if (( ( AX >> 8 ) == 0x44 ) || ( ( AX >> 8 ) == 0x47 ))
! Debug: sr int = const 8 to unsigned short AX = [S+$22+$16] (used reg = )
mov	ax,$18[bp]
mov	al,ah
xor	ah,ah
! Debug: logeq int = const $44 to unsigned int = ax+0 (used reg = )
cmp	ax,*$44
je  	.534
.535:
! Debug: sr int = const 8 to unsigned short AX = [S+$22+$16] (used reg = )
mov	ax,$18[bp]
mov	al,ah
xor	ah,ah
! Debug: logeq int = const $47 to unsigned int = ax+0 (used reg = )
cmp	ax,*$47
jne 	.533
.534:
! 3751         goto int13_success;
add	sp,#..FFE8-..FFE9
br 	.FFE8
!BCC_EOS
! 3752       memsetb(get_SS(),atacmd,0,12);
.533:
! Debug: list int = const $C (used reg = )
mov	ax,*$C
push	ax
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: list * unsigned char atacmd = S+$26-$13 (used reg = )
lea	bx,-$11[bp]
push	bx
! Debug: func () unsigned short = get_SS+0 (used reg = )
call	_get_SS
! Debug: list unsigned short = ax+0 (used reg = )
push	ax
! Debug: func () void = memsetb+0 (used reg = )
call	_memsetb
add	sp,*8
!BCC_EOS
! 3753       atacmd[0]=0x28;
! Debug: eq int = const $28 to unsigned char atacmd = [S+$22-$13] (used reg = )
mov	al,*$28
mov	-$11[bp],al
!BCC_EOS
! 3754       atacmd[7]=(count & 0xff00) >> 8;
! Debug: and unsigned int = const $FF00 to unsigned short count = [S+$22-$1A] (used reg = )
mov	ax,-$18[bp]
xor	al,al
! Debug: sr int = const 8 to unsigned int = ax+0 (used reg = )
mov	al,ah
xor	ah,ah
! Debug: eq unsigned int = ax+0 to unsigned char atacmd = [S+$22-$C] (used reg = )
mov	-$A[bp],al
!BCC_EOS
! 3755       atacmd[8]=(count & 0x00ff);
! Debug: and int = const $FF to unsigned short count = [S+$22-$1A] (used reg = )
mov	al,-$18[bp]
! Debug: eq unsigned char = al+0 to unsigned char atacmd = [S+$22-$B] (used reg = )
mov	-9[bp],al
!BCC_EOS
! 3756       atacmd[2]=(lba & 0xff000000) >> 24;
! Debug: and unsigned long = const $FF000000 to unsigned long lba = [S+$22-$18] (used reg = )
! Debug: expression subtree swapping
xor	ax,ax
mov	bx,#$FF00
lea	di,-$16[bp]
call	landul
! Debug: sr int = const $18 to unsigned long = bx+0 (used reg = )
xchg	bx,ax
mov	al,ah
xor	ah,ah
xor	bx,bx
! Debug: eq unsigned long = bx+0 to unsigned char atacmd = [S+$22-$11] (used reg = )
mov	-$F[bp],al
!BCC_EOS
! 3757       atacmd[3]=(lba & 0x00ff0000) >> 16;
! Debug: and long = const $FF0000 to unsigned long lba = [S+$22-$18] (used reg = )
! Debug: expression subtree swapping
xor	ax,ax
mov	bx,#$FF
lea	di,-$16[bp]
call	landul
! Debug: sr int = const $10 to unsigned long = bx+0 (used reg = )
xchg	bx,ax
xor	bx,bx
! Debug: eq unsigned long = bx+0 to unsigned char atacmd = [S+$22-$10] (used reg = )
mov	-$E[bp],al
!BCC_EOS
! 3758       atacmd[4]=(lba & 0x0000ff00) >> 8;
! Debug: and unsigned long = const $FF00 to unsigned long lba = [S+$22-$18] (used reg = )
! Debug: expression subtree swapping
mov	ax,#$FF00
xor	bx,bx
lea	di,-$16[bp]
call	landul
! Debug: sr int = const 8 to unsigned long = bx+0 (used reg = )
mov	al,ah
mov	ah,bl
mov	bl,bh
sub	bh,bh
! Debug: eq unsigned long = bx+0 to unsigned char atacmd = [S+$22-$F] (used reg = )
mov	-$D[bp],al
!BCC_EOS
! 3759       atacmd[5]=(lba & 0x000000ff);
! Debug: and unsigned long = const $FF to unsigned long lba = [S+$22-$18] (used reg = )
! Debug: expression subtree swapping
mov	ax,#$FF
xor	bx,bx
lea	di,-$16[bp]
call	landul
! Debug: eq unsigned long = bx+0 to unsigned char atacmd = [S+$22-$E] (used reg = )
mov	-$C[bp],al
!BCC_EOS
! 3760       status = ata_cmd_packet(device, 12, get_SS(), atacmd, 0, count*2048L, 0x01, segment,offset);
! Debug: list unsigned short offset = [S+$22-$1E] (used reg = )
push	-$1C[bp]
! Debug: list unsigned short segment = [S+$24-$1C] (used reg = )
push	-$1A[bp]
! Debug: list int = const 1 (used reg = )
mov	ax,*1
push	ax
! Debug: cast unsigned long = const 0 to unsigned short count = [S+$28-$1A] (used reg = )
mov	ax,-$18[bp]
xor	bx,bx
! Debug: mul long = const $800 to unsigned long = bx+0 (used reg = )
push	bx
push	ax
mov	ax,#$800
xor	bx,bx
push	bx
push	ax
mov	ax,-8+..FFE9[bp]
mov	bx,-6+..FFE9[bp]
lea	di,-$C+..FFE9[bp]
call	lmulul
add	sp,*8
! Debug: list unsigned long = bx+0 (used reg = )
push	bx
push	ax
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: list * unsigned char atacmd = S+$2E-$13 (used reg = )
lea	bx,-$11[bp]
push	bx
! Debug: func () unsigned short = get_SS+0 (used reg = )
call	_get_SS
! Debug: list unsigned short = ax+0 (used reg = )
push	ax
! Debug: list int = const $C (used reg = )
mov	ax,*$C
push	ax
! Debug: list unsigned char device = [S+$34-5] (used reg = )
mov	al,-3[bp]
xor	ah,ah
push	ax
! Debug: func () unsigned short = ata_cmd_packet+0 (used reg = )
call	_ata_cmd_packet
add	sp,*$14
! Debug: eq unsigned short = ax+0 to unsigned char status = [S+$22-6] (used reg = )
mov	-4[bp],al
!BCC_EOS
! 3761       count = (Bit16u)(read_dword(ebda_seg, &((ebda_data_t *) 0)->ata.trsfbytes) >> 11);
! Debug: list * unsigned long = const $236 (used reg = )
mov	ax,#$236
push	ax
! Debug: list unsigned short ebda_seg = [S+$24-4] (used reg = )
push	-2[bp]
! Debug: func () unsigned long = read_dword+0 (used reg = )
call	_read_dword
mov	bx,dx
add	sp,*4
! Debug: sr int = const $B to unsigned long = bx+0 (used reg = )
mov	al,ah
mov	ah,bl
mov	bl,bh
sub	bh,bh
mov	di,*3
call	lsrul
! Debug: cast unsigned short = const 0 to unsigned long = bx+0 (used reg = )
! Debug: eq unsigned short = ax+0 to unsigned short count = [S+$22-$1A] (used reg = )
mov	-$18[bp],ax
!BCC_EOS
! 3762       write_word(DS, SI+(Bit16u)&((int13ext_t *) 0)->count, count);
! Debug: list unsigned short count = [S+$22-$1A] (used reg = )
push	-$18[bp]
! Debug: add unsigned short = const 2 to unsigned short SI = [S+$24+$A] (used reg = )
mov	ax,$C[bp]
! Debug: list unsigned int = ax+2 (used reg = )
inc	ax
inc	ax
push	ax
! Debug: list unsigned short DS = [S+$26+4] (used reg = )
push	6[bp]
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
add	sp,*6
!BCC_EOS
! 3763       if (status != 0) {
! Debug: ne int = const 0 to unsigned char status = [S+$22-6] (used reg = )
mov	al,-4[bp]
test	al,al
je  	.536
.537:
! 3764         bios_printf(4, "int13_cdrom: function %02x, status %02x !\n",( AX >> 8 ),status);
! Debug: list unsigned char status = [S+$22-6] (used reg = )
mov	al,-4[bp]
xor	ah,ah
push	ax
! Debug: sr int = const 8 to unsigned short AX = [S+$24+$16] (used reg = )
mov	ax,$18[bp]
mov	al,ah
xor	ah,ah
! Debug: list unsigned int = ax+0 (used reg = )
push	ax
! Debug: list * char = .538+0 (used reg = )
mov	bx,#.538
push	bx
! Debug: list int = const 4 (used reg = )
mov	ax,*4
push	ax
! Debug: func () void = bios_printf+0 (used reg = )
call	_bios_printf
add	sp,*8
!BCC_EOS
! 3765         AX = ((AX & 0x00ff) | ((0x0c) << 8));
! Debug: and int = const $FF to unsigned short AX = [S+$22+$16] (used reg = )
mov	al,$18[bp]
! Debug: or int = const $C00 to unsigned char = al+0 (used reg = )
xor	ah,ah
or	ax,#$C00
! Debug: eq unsigned int = ax+0 to unsigned short AX = [S+$22+$16] (used reg = )
mov	$18[bp],ax
!BCC_EOS
! 3766         goto int13_fail_noah;
add	sp,#..FFE7-..FFE9
br 	.FFE7
!BCC_EOS
! 3767         }
! 3768       goto int13_success;
.536:
add	sp,#..FFE8-..FFE9
br 	.FFE8
!BCC_EOS
! 3769       brea
! 3769 k;
br 	.519
!BCC_EOS
! 3770     case 0x45:
! 3771       if (( AX & 0x00ff ) > 2) goto int13_fail;
.539:
! Debug: and int = const $FF to unsigned short AX = [S+$22+$16] (used reg = )
mov	al,$18[bp]
! Debug: gt int = const 2 to unsigned char = al+0 (used reg = )
cmp	al,*2
jbe 	.53A
.53B:
add	sp,#..FFEA-..FFE9
br 	.FFEA
!BCC_EOS
! 3772       locks = read_byte(ebda_seg, &((ebda_data_t *) 0)->ata.devices[device].lock);
.53A:
! Debug: ptradd unsigned char device = [S+$22-5] to [8] struct  = const $142 (used reg = )
mov	al,-3[bp]
xor	ah,ah
mov	cx,*$1A
imul	cx
mov	bx,ax
! Debug: address unsigned char = [bx+$145] (used reg = )
! Debug: list * unsigned char = bx+$145 (used reg = )
add	bx,#$145
push	bx
! Debug: list unsigned short ebda_seg = [S+$24-4] (used reg = )
push	-2[bp]
! Debug: func () unsigned char = read_byte+0 (used reg = )
call	_read_byte
add	sp,*4
! Debug: eq unsigned char = al+0 to unsigned char locks = [S+$22-7] (used reg = )
mov	-5[bp],al
!BCC_EOS
! 3773       switch (( AX & 0x00ff )) {
! Debug: and int = const $FF to unsigned short AX = [S+$22+$16] (used reg = )
mov	al,$18[bp]
br 	.53E
! 3774         case 0 :
! 3775           if (locks == 0xff) {
.53F:
! Debug: logeq int = const $FF to unsigned char locks = [S+$22-7] (used reg = )
mov	al,-5[bp]
cmp	al,#$FF
jne 	.540
.541:
! 3776             AX = ((AX & 0x00ff) | ((0xb4) << 8));
! Debug: and int = const $FF to unsigned short AX = [S+$22+$16] (used reg = )
mov	al,$18[bp]
! Debug: or int = const -$4C00 to unsigned char = al+0 (used reg = )
xor	ah,ah
or	ax,#-$4C00
! Debug: eq unsigned int = ax+0 to unsigned short AX = [S+$22+$16] (used reg = )
mov	$18[bp],ax
!BCC_EOS
! 3777             AX = ((AX & 0xff00) | (1));
! Debug: and unsigned int = const $FF00 to unsigned short AX = [S+$22+$16] (used reg = )
mov	ax,$18[bp]
xor	al,al
! Debug: or int = const 1 to unsigned int = ax+0 (used reg = )
or	al,*1
! Debug: eq unsigned int = ax+0 to unsigned short AX = [S+$22+$16] (used reg = )
mov	$18[bp],ax
!BCC_EOS
! 3778             goto int13_fail_noah;
add	sp,#..FFE7-..FFE9
br 	.FFE7
!BCC_EOS
! 3779             }
! 3780           write_byte(ebda_seg, &((ebda_data_t *) 0)->ata.devices[device].lock, ++locks);
.540:
! Debug: preinc unsigned char locks = [S+$22-7] (used reg = )
mov	al,-5[bp]
inc	ax
mov	-5[bp],al
! Debug: list unsigned char = al+0 (used reg = )
xor	ah,ah
push	ax
! Debug: ptradd unsigned char device = [S+$24-5] to [8] struct  = const $142 (used reg = )
mov	al,-3[bp]
xor	ah,ah
mov	cx,*$1A
imul	cx
mov	bx,ax
! Debug: address unsigned char = [bx+$145] (used reg = )
! Debug: list * unsigned char = bx+$145 (used reg = )
add	bx,#$145
push	bx
! Debug: list unsigned short ebda_seg = [S+$26-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 3781           AX = ((AX & 0xff00) | (1));
! Debug: and unsigned int = const $FF00 to unsigned short AX = [S+$22+$16] (used reg = )
mov	ax,$18[bp]
xor	al,al
! Debug: or int = const 1 to unsigned int = ax+0 (used reg = )
or	al,*1
! Debug: eq unsigned int = ax+0 to unsigned short AX = [S+$22+$16] (used reg = )
mov	$18[bp],ax
!BCC_EOS
! 3782           break;
br 	.53C
!BCC_EOS
! 3783         case 1 :
! 3784           if (locks == 0x00) {
.542:
! Debug: logeq int = const 0 to unsigned char locks = [S+$22-7] (used reg = )
mov	al,-5[bp]
test	al,al
jne 	.543
.544:
! 3785             AX = ((AX & 0x00ff) | ((0xb0) << 8));
! Debug: and int = const $FF to unsigned short AX = [S+$22+$16] (used reg = )
mov	al,$18[bp]
! Debug: or int = const -$5000 to unsigned char = al+0 (used reg = )
xor	ah,ah
or	ax,#-$5000
! Debug: eq unsigned int = ax+0 to unsigned short AX = [S+$22+$16] (used reg = )
mov	$18[bp],ax
!BCC_EOS
! 3786             AX = ((AX & 0xff00) | (0));
! Debug: and unsigned int = const $FF00 to unsigned short AX = [S+$22+$16] (used reg = )
mov	ax,$18[bp]
xor	al,al
! Debug: or int = const 0 to unsigned int = ax+0 (used reg = )
or	al,*0
! Debug: eq unsigned int = ax+0 to unsigned short AX = [S+$22+$16] (used reg = )
mov	$18[bp],ax
!BCC_EOS
! 3787             goto int13_fail_noah;
add	sp,#..FFE7-..FFE9
br 	.FFE7
!BCC_EOS
! 3788             }
! 3789           write_byte(ebda_seg, &((ebda_data_t *) 0)->ata.devices[device].lock, --locks);
.543:
! Debug: predec unsigned char locks = [S+$22-7] (used reg = )
mov	al,-5[bp]
dec	ax
mov	-5[bp],al
! Debug: list unsigned char = al+0 (used reg = )
xor	ah,ah
push	ax
! Debug: ptradd unsigned char device = [S+$24-5] to [8] struct  = const $142 (used reg = )
mov	al,-3[bp]
xor	ah,ah
mov	cx,*$1A
imul	cx
mov	bx,ax
! Debug: address unsigned char = [bx+$145] (used reg = )
! Debug: list * unsigned char = bx+$145 (used reg = )
add	bx,#$145
push	bx
! Debug: list unsigned short ebda_seg = [S+$26-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 3790           AX = ((AX & 0xff00) | (locks==0?0:1));
! Debug: logeq int = const 0 to unsigned char locks = [S+$22-7] (used reg = )
mov	al,-5[bp]
test	al,al
jne 	.545
.546:
xor	al,al
jmp .547
.545:
mov	al,*1
.547:
push	ax
! Debug: and unsigned int = const $FF00 to unsigned short AX = [S+$24+$16] (used reg = )
mov	ax,$18[bp]
xor	al,al
! Debug: or char (temp) = [S+$24-$24] to unsigned int = ax+0 (used reg = )
or	al,0+..FFE9[bp]
inc	sp
inc	sp
! Debug: eq unsigned int = ax+0 to unsigned short AX = [S+$22+$16] (used reg = )
mov	$18[bp],ax
!BCC_EOS
! 3791           break;
jmp .53C
!BCC_EOS
! 3792         case 2 :
! 3793           AX = ((AX & 0xff00) | (locks==0?0:1));
.548:
! Debug: logeq int = const 0 to unsigned char locks = [S+$22-7] (used reg = )
mov	al,-5[bp]
test	al,al
jne 	.549
.54A:
xor	al,al
jmp .54B
.549:
mov	al,*1
.54B:
push	ax
! Debug: and unsigned int = const $FF00 to unsigned short AX = [S+$24+$16] (used reg = )
mov	ax,$18[bp]
xor	al,al
! Debug: or char (temp) = [S+$24-$24] to unsigned int = ax+0 (used reg = )
or	al,0+..FFE9[bp]
inc	sp
inc	sp
! Debug: eq unsigned int = ax+0 to unsigned short AX = [S+$22+$16] (used reg = )
mov	$18[bp],ax
!BCC_EOS
! 3794           break;
jmp .53C
!BCC_EOS
! 3795         }
! 3796       goto int13_success;
jmp .53C
.53E:
sub	al,*0
beq 	.53F
sub	al,*1
beq 	.542
sub	al,*1
je 	.548
.53C:
add	sp,#..FFE8-..FFE9
br 	.FFE8
!BCC_EOS
! 3797       break;
br 	.519
!BCC_EOS
! 3798     case 0x46:
! 3799       locks = read_byte(ebda_seg, &((ebda_data_t *) 0)->ata.devices[device].lock);
.54C:
! Debug: ptradd unsigned char device = [S+$22-5] to [8] struct  = const $142 (used reg = )
mov	al,-3[bp]
xor	ah,ah
mov	cx,*$1A
imul	cx
mov	bx,ax
! Debug: address unsigned char = [bx+$145] (used reg = )
! Debug: list * unsigned char = bx+$145 (used reg = )
add	bx,#$145
push	bx
! Debug: list unsigned short ebda_seg = [S+$24-4] (used reg = )
push	-2[bp]
! Debug: func () unsigned char = read_byte+0 (used reg = )
call	_read_byte
add	sp,*4
! Debug: eq unsigned char = al+0 to unsigned char locks = [S+$22-7] (used reg = )
mov	-5[bp],al
!BCC_EOS
! 3800       if (locks != 0) {
! Debug: ne int = const 0 to unsigned char locks = [S+$22-7] (used reg = )
mov	al,-5[bp]
test	al,al
je  	.54D
.54E:
! 3801         AX = ((AX & 0x00ff) | ((0xb1) << 8));
! Debug: and int = const $FF to unsigned short AX = [S+$22+$16] (used reg = )
mov	al,$18[bp]
! Debug: or int = const -$4F00 to unsigned char = al+0 (used reg = )
xor	ah,ah
or	ax,#-$4F00
! Debug: eq unsigned int = ax+0 to unsigned short AX = [S+$22+$16] (used reg = )
mov	$18[bp],ax
!BCC_EOS
! 3802         goto int13_fail_noah;
add	sp,#..FFE7-..FFE9
br 	.FFE7
!BCC_EOS
! 3803         }
! 3804 #asm
.54D:
!BCC_EOS
!BCC_ASM
_int13_cdrom.BP	set	$2E
.int13_cdrom.BP	set	$E
_int13_cdrom.EHBX	set	$24
.int13_cdrom.EHBX	set	4
_int13_cdrom.CS	set	$3C
.int13_cdrom.CS	set	$1C
_int13_cdrom.count	set	8
.int13_cdrom.count	set	-$18
_int13_cdrom.CX	set	$36
.int13_cdrom.CX	set	$16
_int13_cdrom.segment	set	6
.int13_cdrom.segment	set	-$1A
_int13_cdrom.DI	set	$2A
.int13_cdrom.DI	set	$A
_int13_cdrom.FLAGS	set	$3E
.int13_cdrom.FLAGS	set	$1E
_int13_cdrom.DS	set	$26
.int13_cdrom.DS	set	6
_int13_cdrom.ELDX	set	$30
.int13_cdrom.ELDX	set	$10
_int13_cdrom.DX	set	$34
.int13_cdrom.DX	set	$14
_int13_cdrom.size	set	0
.int13_cdrom.size	set	-$20
_int13_cdrom.i	set	2
.int13_cdrom.i	set	-$1E
_int13_cdrom.device	set	$1D
.int13_cdrom.device	set	-3
_int13_cdrom.ES	set	$28
.int13_cdrom.ES	set	8
_int13_cdrom.ebda_seg	set	$1E
.int13_cdrom.ebda_seg	set	-2
_int13_cdrom.SI	set	$2C
.int13_cdrom.SI	set	$C
_int13_cdrom.IP	set	$3A
.int13_cdrom.IP	set	$1A
_int13_cdrom.lba	set	$A
.int13_cdrom.lba	set	-$16
_int13_cdrom.status	set	$1C
.int13_cdrom.status	set	-4
_int13_cdrom.atacmd	set	$F
.int13_cdrom.atacmd	set	-$11
_int13_cdrom.AX	set	$38
.int13_cdrom.AX	set	$18
_int13_cdrom.offset	set	4
.int13_cdrom.offset	set	-$1C
_int13_cdrom.BX	set	$32
.int13_cdrom.BX	set	$12
_int13_cdrom.locks	set	$1B
.int13_cdrom.locks	set	-5
        push bp
        mov bp, sp
        mov ah, #0x52
        int 15
        mov _int13_cdrom.status + 2[bp], ah
        jnc int13_cdrom_rme_end
        mov _int13_cdrom.status, #1
int13_cdrom_rme_end:
        pop bp
! 3814 endasm
!BCC_ENDASM
!BCC_EOS
! 3815       if (status != 0) {
! Debug: ne int = const 0 to unsigned char status = [S+$22-6] (used reg = )
mov	al,-4[bp]
test	al,al
je  	.54F
.550:
! 3816         AX = ((AX & 0x00ff) | ((0xb1) << 8));
! Debug: and int = const $FF to unsigned short AX = [S+$22+$16] (used reg = )
mov	al,$18[bp]
! Debug: or int = const -$4F00 to unsigned char = al+0 (used reg = )
xor	ah,ah
or	ax,#-$4F00
! Debug: eq unsigned int = ax+0 to unsigned short AX = [S+$22+$16] (used reg = )
mov	$18[bp],ax
!BCC_EOS
! 3817         goto int13_fail_noah;
add	sp,#..FFE7-..FFE9
br 	.FFE7
!BCC_EOS
! 3818       }
! 3819       goto int13_success;
.54F:
add	sp,#..FFE8-..FFE9
br 	.FFE8
!BCC_EOS
! 3820       break;
br 	.519
!BCC_EOS
! 3821     case 0x48:
! 3822       size = read_word(DS,SI+(Bit16u)&((int13ext_t *) 0)->size);
.551:
! Debug: add unsigned short = const 0 to unsigned short SI = [S+$22+$A] (used reg = )
mov	ax,$C[bp]
! Debug: list unsigned int = ax+0 (used reg = )
push	ax
! Debug: list unsigned short DS = [S+$24+4] (used reg = )
push	6[bp]
! Debug: func () unsigned short = read_word+0 (used reg = )
call	_read_word
add	sp,*4
! Debug: eq unsigned short = ax+0 to unsigned short size = [S+$22-$22] (used reg = )
mov	-$20[bp],ax
!BCC_EOS
! 3823       if(size < 0x1a)
! Debug: lt int = const $1A to unsigned short size = [S+$22-$22] (used reg = )
mov	ax,-$20[bp]
cmp	ax,*$1A
jae 	.552
.553:
! 3824         goto int13_fail;
add	sp,#..FFEA-..FFE9
br 	.FFEA
!BCC_EOS
! 3825       if(size >= 0x1a) {
.552:
! Debug: ge int = const $1A to unsigned short size = [S+$22-$22] (used reg = )
mov	ax,-$20[bp]
cmp	ax,*$1A
blo 	.554
.555:
! 3826         Bit16u cylinders, heads, spt, blksize;
!BCC_EOS
! 3827         blksize = read_word(ebda_seg, &((ebda_data_t *) 0)->ata.devices[device].blksize);
! Debug: ptradd unsigned char device = [S+$2A-5] to [8] struct  = const $142 (used reg = )
mov	al,-3[bp]
xor	ah,ah
mov	cx,*$1A
imul	cx
mov	bx,ax
! Debug: address unsigned short = [bx+$148] (used reg = )
! Debug: list * unsigned short = bx+$148 (used reg = )
add	bx,#$148
push	bx
! Debug: list unsigned short ebda_seg = [S+$2C-4] (used reg = )
push	-2[bp]
! Debug: func () unsigned short = read_word+0 (used reg = )
call	_read_word
add	sp,*4
! Debug: eq unsigned short = ax+0 to unsigned short blksize = [S+$2A-$2A] (used reg = )
mov	-$28[bp],ax
!BCC_EOS
! 3828         write_word(DS, SI+(Bit16u)&((dpt_t *) 0)->size, 0x1a);
! Debug: list int = const $1A (used reg = )
mov	ax,*$1A
push	ax
! Debug: add unsigned short = const 0 to unsigned short SI = [S+$2C+$A] (used reg = )
mov	ax,$C[bp]
! Debug: list unsigned int = ax+0 (used reg = )
push	ax
! Debug: list unsigned short DS = [S+$2E+4] (used reg = )
push	6[bp]
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
add	sp,*6
!BCC_EOS
! 3829         write_word(DS, SI+(Bit16u)&((dpt_t *) 0)->infos, 0x74);
! Debug: list int = const $74 (used reg = )
mov	ax,*$74
push	ax
! Debug: add unsigned short = const 2 to unsigned short SI = [S+$2C+$A] (used reg = )
mov	ax,$C[bp]
! Debug: list unsigned int = ax+2 (used reg = )
inc	ax
inc	ax
push	ax
! Debug: list unsigned short DS = [S+$2E+4] (used reg = )
push	6[bp]
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
add	sp,*6
!BCC_EOS
! 3830         write_dword(DS, SI+(Bit16u)&((dpt_t *) 0)->cylinde
! 3830 rs, 0xffffffff);
! Debug: list unsigned long = const $FFFFFFFF (used reg = )
mov	ax,#$FFFF
mov	bx,#$FFFF
push	bx
push	ax
! Debug: add unsigned short = const 4 to unsigned short SI = [S+$2E+$A] (used reg = )
mov	ax,$C[bp]
! Debug: list unsigned int = ax+4 (used reg = )
add	ax,*4
push	ax
! Debug: list unsigned short DS = [S+$30+4] (used reg = )
push	6[bp]
! Debug: func () void = write_dword+0 (used reg = )
call	_write_dword
add	sp,*8
!BCC_EOS
! 3831         write_dword(DS, SI+(Bit16u)&((dpt_t *) 0)->heads, 0xffffffff);
! Debug: list unsigned long = const $FFFFFFFF (used reg = )
mov	ax,#$FFFF
mov	bx,#$FFFF
push	bx
push	ax
! Debug: add unsigned short = const 8 to unsigned short SI = [S+$2E+$A] (used reg = )
mov	ax,$C[bp]
! Debug: list unsigned int = ax+8 (used reg = )
add	ax,*8
push	ax
! Debug: list unsigned short DS = [S+$30+4] (used reg = )
push	6[bp]
! Debug: func () void = write_dword+0 (used reg = )
call	_write_dword
add	sp,*8
!BCC_EOS
! 3832         write_dword(DS, SI+(Bit16u)&((dpt_t *) 0)->spt, 0xffffffff);
! Debug: list unsigned long = const $FFFFFFFF (used reg = )
mov	ax,#$FFFF
mov	bx,#$FFFF
push	bx
push	ax
! Debug: add unsigned short = const $C to unsigned short SI = [S+$2E+$A] (used reg = )
mov	ax,$C[bp]
! Debug: list unsigned int = ax+$C (used reg = )
add	ax,*$C
push	ax
! Debug: list unsigned short DS = [S+$30+4] (used reg = )
push	6[bp]
! Debug: func () void = write_dword+0 (used reg = )
call	_write_dword
add	sp,*8
!BCC_EOS
! 3833         write_dword(DS, SI+(Bit16u)&((dpt_t *) 0)->sector_count1, 0xffffffff);
! Debug: list unsigned long = const $FFFFFFFF (used reg = )
mov	ax,#$FFFF
mov	bx,#$FFFF
push	bx
push	ax
! Debug: add unsigned short = const $10 to unsigned short SI = [S+$2E+$A] (used reg = )
mov	ax,$C[bp]
! Debug: list unsigned int = ax+$10 (used reg = )
add	ax,*$10
push	ax
! Debug: list unsigned short DS = [S+$30+4] (used reg = )
push	6[bp]
! Debug: func () void = write_dword+0 (used reg = )
call	_write_dword
add	sp,*8
!BCC_EOS
! 3834         write_dword(DS, SI+(Bit16u)&((dpt_t *) 0)->sector_count2, 0xffffffff);
! Debug: list unsigned long = const $FFFFFFFF (used reg = )
mov	ax,#$FFFF
mov	bx,#$FFFF
push	bx
push	ax
! Debug: add unsigned short = const $14 to unsigned short SI = [S+$2E+$A] (used reg = )
mov	ax,$C[bp]
! Debug: list unsigned int = ax+$14 (used reg = )
add	ax,*$14
push	ax
! Debug: list unsigned short DS = [S+$30+4] (used reg = )
push	6[bp]
! Debug: func () void = write_dword+0 (used reg = )
call	_write_dword
add	sp,*8
!BCC_EOS
! 3835         write_word(DS, SI+(Bit16u)&((dpt_t *) 0)->blksize, blksize);
! Debug: list unsigned short blksize = [S+$2A-$2A] (used reg = )
push	-$28[bp]
! Debug: add unsigned short = const $18 to unsigned short SI = [S+$2C+$A] (used reg = )
mov	ax,$C[bp]
! Debug: list unsigned int = ax+$18 (used reg = )
add	ax,*$18
push	ax
! Debug: list unsigned short DS = [S+$2E+4] (used reg = )
push	6[bp]
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
add	sp,*6
!BCC_EOS
! 3836         }
! 3837       if(size >= 0x1e) {
.554:
! Debug: ge int = const $1E to unsigned short size = [S+$22-$22] (used reg = )
mov	ax,-$20[bp]
cmp	ax,*$1E
blo 	.556
.557:
! 3838         Bit8u channel, dev, irq, mode, checksum, i;
!BCC_EOS
! 3839         Bit16u iobase1, iobase2, options;
!BCC_EOS
! 3840         write_word(DS, SI+(Bit16u)&((dpt_t *) 0)->size, 0x1e);
! Debug: list int = const $1E (used reg = )
mov	ax,*$1E
push	ax
! Debug: add unsigned short = const 0 to unsigned short SI = [S+$30+$A] (used reg = )
mov	ax,$C[bp]
! Debug: list unsigned int = ax+0 (used reg = )
push	ax
! Debug: list unsigned short DS = [S+$32+4] (used reg = )
push	6[bp]
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
add	sp,*6
!BCC_EOS
! 3841         write_word(DS, SI+(Bit16u)&((dpt_t *) 0)->dpte_segment, ebda_seg);
! Debug: list unsigned short ebda_seg = [S+$2E-4] (used reg = )
push	-2[bp]
! Debug: add unsigned short = const $1C to unsigned short SI = [S+$30+$A] (used reg = )
mov	ax,$C[bp]
! Debug: list unsigned int = ax+$1C (used reg = )
add	ax,*$1C
push	ax
! Debug: list unsigned short DS = [S+$32+4] (used reg = )
push	6[bp]
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
add	sp,*6
!BCC_EOS
! 3842         write_word(DS, SI+(Bit16u)&((dpt_t *) 0)->dpte_offset, &((ebda_data_t *) 0)->ata.dpte);
! Debug: list * struct  = const $224 (used reg = )
mov	ax,#$224
push	ax
! Debug: add unsigned short = const $1A to unsigned short SI = [S+$30+$A] (used reg = )
mov	ax,$C[bp]
! Debug: list unsigned int = ax+$1A (used reg = )
add	ax,*$1A
push	ax
! Debug: list unsigned short DS = [S+$32+4] (used reg = )
push	6[bp]
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
add	sp,*6
!BCC_EOS
! 3843         channel = device / 2;
! Debug: div int = const 2 to unsigned char device = [S+$2E-5] (used reg = )
mov	al,-3[bp]
xor	ah,ah
shr	ax,*1
! Debug: eq unsigned int = ax+0 to unsigned char channel = [S+$2E-$23] (used reg = )
mov	-$21[bp],al
!BCC_EOS
! 3844         iobase1 = read_word(ebda_seg, &((ebda_data_t *) 0)->ata.channels[channel].iobase1);
! Debug: ptradd unsigned char channel = [S+$2E-$23] to [4] struct  = const $122 (used reg = )
mov	al,-$21[bp]
xor	ah,ah
mov	cl,*3
shl	ax,cl
mov	bx,ax
! Debug: address unsigned short = [bx+$124] (used reg = )
! Debug: list * unsigned short = bx+$124 (used reg = )
add	bx,#$124
push	bx
! Debug: list unsigned short ebda_seg = [S+$30-4] (used reg = )
push	-2[bp]
! Debug: func () unsigned short = read_word+0 (used reg = )
call	_read_word
add	sp,*4
! Debug: eq unsigned short = ax+0 to unsigned short iobase1 = [S+$2E-$2A] (used reg = )
mov	-$28[bp],ax
!BCC_EOS
! 3845         iobase2 = read_word(ebda_seg, &((ebda_data_t *) 0)->ata.channels[channel].iobase2);
! Debug: ptradd unsigned char channel = [S+$2E-$23] to [4] struct  = const $122 (used reg = )
mov	al,-$21[bp]
xor	ah,ah
mov	cl,*3
shl	ax,cl
mov	bx,ax
! Debug: address unsigned short = [bx+$126] (used reg = )
! Debug: list * unsigned short = bx+$126 (used reg = )
add	bx,#$126
push	bx
! Debug: list unsigned short ebda_seg = [S+$30-4] (used reg = )
push	-2[bp]
! Debug: func () unsigned short = read_word+0 (used reg = )
call	_read_word
add	sp,*4
! Debug: eq unsigned short = ax+0 to unsigned short iobase2 = [S+$2E-$2C] (used reg = )
mov	-$2A[bp],ax
!BCC_EOS
! 3846         irq = read_byte(ebda_seg, &((ebda_data_t *) 0)->ata.channels[channel].irq);
! Debug: ptradd unsigned char channel = [S+$2E-$23] to [4] struct  = const $122 (used reg = )
mov	al,-$21[bp]
xor	ah,ah
mov	cl,*3
shl	ax,cl
mov	bx,ax
! Debug: address unsigned char = [bx+$128] (used reg = )
! Debug: list * unsigned char = bx+$128 (used reg = )
add	bx,#$128
push	bx
! Debug: list unsigned short ebda_seg = [S+$30-4] (used reg = )
push	-2[bp]
! Debug: func () unsigned char = read_byte+0 (used reg = )
call	_read_byte
add	sp,*4
! Debug: eq unsigned char = al+0 to unsigned char irq = [S+$2E-$25] (used reg = )
mov	-$23[bp],al
!BCC_EOS
! 3847         mode = read_byte(ebda_seg, &((ebda_data_t *) 0)->ata.devices[device].mode);
! Debug: ptradd unsigned char device = [S+$2E-5] to [8] struct  = const $142 (used reg = )
mov	al,-3[bp]
xor	ah,ah
mov	cx,*$1A
imul	cx
mov	bx,ax
! Debug: address unsigned char = [bx+$146] (used reg = )
! Debug: list * unsigned char = bx+$146 (used reg = )
add	bx,#$146
push	bx
! Debug: list unsigned short ebda_seg = [S+$30-4] (used reg = )
push	-2[bp]
! Debug: func () unsigned char = read_byte+0 (used reg = )
call	_read_byte
add	sp,*4
! Debug: eq unsigned char = al+0 to unsigned char mode = [S+$2E-$26] (used reg = )
mov	-$24[bp],al
!BCC_EOS
! 3848         options = (1<<4);
! Debug: eq int = const $10 to unsigned short options = [S+$2E-$2E] (used reg = )
mov	ax,*$10
mov	-$2C[bp],ax
!BCC_EOS
! 3849         options |= (1<<5);
! Debug: orab int = const $20 to unsigned short options = [S+$2E-$2E] (used reg = )
mov	ax,-$2C[bp]
or	al,*$20
mov	-$2C[bp],ax
!BCC_EOS
! 3850         options |= (1<<6);
! Debug: orab int = const $40 to unsigned short options = [S+$2E-$2E] (used reg = )
mov	ax,-$2C[bp]
or	al,*$40
mov	-$2C[bp],ax
!BCC_EOS
! 3851         options |= (mode==0x01?1:0<<7);
! Debug: logeq int = const 1 to unsigned char mode = [S+$2E-$26] (used reg = )
mov	al,-$24[bp]
cmp	al,*1
jne 	.558
.559:
mov	al,*1
jmp .55A
.558:
xor	al,al
.55A:
! Debug: orab char = al+0 to unsigned short options = [S+$2E-$2E] (used reg = )
xor	ah,ah
or	ax,-$2C[bp]
mov	-$2C[bp],ax
!BCC_EOS
! 3852         write_word(ebda_seg, &((ebda_data_t *) 0)->ata.dpte.iobase1, iobase1);
! Debug: list unsigned short iobase1 = [S+$2E-$2A] (used reg = )
push	-$28[bp]
! Debug: list * unsigned short = const $224 (used reg = )
mov	ax,#$224
push	ax
! Debug: list unsigned short ebda_seg = [S+$32-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
add	sp,*6
!BCC_EOS
! 3853         write_word(ebda_seg, &((ebda_data_t *) 0)->ata.dpte.iobase2, iobase2);
! Debug: list unsigned short iobase2 = [S+$2E-$2C] (used reg = )
push	-$2A[bp]
! Debug: list * unsigned short = const $226 (used reg = )
mov	ax,#$226
push	ax
! Debug: list unsigned short ebda_seg = [S+$32-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
add	sp,*6
!BCC_EOS
! 3854         write_byte(ebda_seg, &((ebda_data_t *) 0)->ata.dpte.prefix, (0xe | (device % 2))<<4 );
! Debug: mod int = const 2 to unsigned char device = [S+$2E-5] (used reg = )
mov	al,-3[bp]
xor	ah,ah
and	al,*1
! Debug: or unsigned char = al+0 to int = const $E (used reg = )
! Debug: expression subtree swapping
or	al,*$E
! Debug: sl int = const 4 to unsigned char = al+0 (used reg = )
xor	ah,ah
mov	cl,*4
shl	ax,cl
! Debug: list unsigned int = ax+0 (used reg = )
push	ax
! Debug: list * unsigned char = const $228 (used reg = )
mov	ax,#$228
push	ax
! Debug: list unsigned short ebda_seg = [S+$32-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 3855         write_byte(ebda_seg, &((ebda_data_t *) 0)->ata.dpte.unused, 0xcb );
! Debug: list int = const $CB (used reg = )
mov	ax,#$CB
push	ax
! Debug: list * unsigned char = const $229 (used reg = )
mov	ax,#$229
push	ax
! Debug: list unsigned short ebda_seg = [S+$32-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 3856         write_byte(ebda_seg, &((ebda_data_t *) 0)->ata.dpte.irq, irq );
! Debug: list unsigned char irq = [S+$2E-$25] (used reg = )
mov	al,-$23[bp]
xor	ah,ah
push	ax
! Debug: list * unsigned char = const $22A (used reg = )
mov	ax,#$22A
push	ax
! Debug: list unsigned short ebda_seg = [S+$32-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 3857         write_byte(ebda_seg, &((ebda_data_t *) 0)->ata.dpte.blkcount, 1 );
! Debug: list int = const 1 (used reg = )
mov	ax,*1
push	ax
! Debug: list * unsigned char = const $22B (used reg = )
mov	ax,#$22B
push	ax
! Debug: list unsigned short ebda_seg = [S+$32-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 3858         write_byte(ebda_seg, &((ebda_data_t *) 0)->ata.dpte.dma, 0 );
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: list * unsigned char = const $22C (used reg = )
mov	ax,#$22C
push	ax
! Debug: list unsigned short ebda_seg = [S+$32-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 3859         write_byte(ebda_seg, &((ebda_data_t *) 0)->ata.dpte.pio, 0 );
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: list * unsigned char = const $22D (used reg = )
mov	ax,#$22D
push	ax
! Debug: list unsigned short ebda_seg = [S+$32-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 3860         write_word(ebda_seg, &((ebda_data_t *) 0)->ata.dpte.options, options);
! Debug: list unsigned short options = [S+$2E-$2E] (used reg = )
push	-$2C[bp]
! Debug: list * unsigned short = const $22E (used reg = )
mov	ax,#$22E
push	ax
! Debug: list unsigned short ebda_seg = [S+$32-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
add	sp,*6
!BCC_EOS
! 3861         write_word(ebda_seg, &((ebda_data_t *) 0)->ata.dpte.reserved, 0);
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: list * unsigned short = const $230 (used reg = )
mov	ax,#$230
push	ax
! Debug: list unsigned short ebda_seg = [S+$32-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
add	sp,*6
!BCC_EOS
! 3862         write_byte(ebda_seg, &
! 3862 ((ebda_data_t *) 0)->ata.dpte.revision, 0x11);
! Debug: list int = const $11 (used reg = )
mov	ax,*$11
push	ax
! Debug: list * unsigned char = const $232 (used reg = )
mov	ax,#$232
push	ax
! Debug: list unsigned short ebda_seg = [S+$32-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 3863         checksum=0;
! Debug: eq int = const 0 to unsigned char checksum = [S+$2E-$27] (used reg = )
xor	al,al
mov	-$25[bp],al
!BCC_EOS
! 3864         for (i=0; i<15; i++) checksum+=read_byte(ebda_seg, (&((ebda_data_t *) 0)->ata.dpte) + i);
! Debug: eq int = const 0 to unsigned char i = [S+$2E-$28] (used reg = )
xor	al,al
mov	-$26[bp],al
!BCC_EOS
!BCC_EOS
jmp .55D
.55E:
! Debug: ptradd unsigned char i = [S+$2E-$28] to * struct  = const $224 (used reg = )
mov	al,-$26[bp]
xor	ah,ah
mov	cl,*4
shl	ax,cl
! Debug: list * struct  = ax+$224 (used reg = )
add	ax,#$224
push	ax
! Debug: list unsigned short ebda_seg = [S+$30-4] (used reg = )
push	-2[bp]
! Debug: func () unsigned char = read_byte+0 (used reg = )
call	_read_byte
add	sp,*4
! Debug: addab unsigned char = al+0 to unsigned char checksum = [S+$2E-$27] (used reg = )
xor	ah,ah
add	al,-$25[bp]
adc	ah,*0
mov	-$25[bp],al
!BCC_EOS
! 3865         checksum = ~checksum;
.55C:
! Debug: postinc unsigned char i = [S+$2E-$28] (used reg = )
mov	al,-$26[bp]
inc	ax
mov	-$26[bp],al
.55D:
! Debug: lt int = const $F to unsigned char i = [S+$2E-$28] (used reg = )
mov	al,-$26[bp]
cmp	al,*$F
jb 	.55E
.55F:
.55B:
! Debug: not unsigned char checksum = [S+$2E-$27] (used reg = )
mov	al,-$25[bp]
xor	ah,ah
not	ax
! Debug: eq unsigned int = ax+0 to unsigned char checksum = [S+$2E-$27] (used reg = )
mov	-$25[bp],al
!BCC_EOS
! 3866         write_byte(ebda_seg, &((ebda_data_t *) 0)->ata.dpte.checksum, checksum);
! Debug: list unsigned char checksum = [S+$2E-$27] (used reg = )
mov	al,-$25[bp]
xor	ah,ah
push	ax
! Debug: list * unsigned char = const $233 (used reg = )
mov	ax,#$233
push	ax
! Debug: list unsigned short ebda_seg = [S+$32-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 3867         }
! 3868       if(size >= 0x42) {
.556:
! Debug: ge int = const $42 to unsigned short size = [S+$22-$22] (used reg = )
mov	ax,-$20[bp]
cmp	ax,*$42
blo 	.560
.561:
! 3869         Bit8u channel, iface, checksum, i;
!BCC_EOS
! 3870         Bit16u iobase1;
!BCC_EOS
! 3871         channel = device / 2;
! Debug: div int = const 2 to unsigned char device = [S+$28-5] (used reg = )
mov	al,-3[bp]
xor	ah,ah
shr	ax,*1
! Debug: eq unsigned int = ax+0 to unsigned char channel = [S+$28-$23] (used reg = )
mov	-$21[bp],al
!BCC_EOS
! 3872         iface = read_byte(ebda_seg, &((ebda_data_t *) 0)->ata.channels[channel].iface);
! Debug: ptradd unsigned char channel = [S+$28-$23] to [4] struct  = const $122 (used reg = )
mov	al,-$21[bp]
xor	ah,ah
mov	cl,*3
shl	ax,cl
mov	bx,ax
! Debug: address unsigned char = [bx+$122] (used reg = )
! Debug: list * unsigned char = bx+$122 (used reg = )
add	bx,#$122
push	bx
! Debug: list unsigned short ebda_seg = [S+$2A-4] (used reg = )
push	-2[bp]
! Debug: func () unsigned char = read_byte+0 (used reg = )
call	_read_byte
add	sp,*4
! Debug: eq unsigned char = al+0 to unsigned char iface = [S+$28-$24] (used reg = )
mov	-$22[bp],al
!BCC_EOS
! 3873         iobase1 = read_word(ebda_seg, &((ebda_data_t *) 0)->ata.channels[channel].iobase1);
! Debug: ptradd unsigned char channel = [S+$28-$23] to [4] struct  = const $122 (used reg = )
mov	al,-$21[bp]
xor	ah,ah
mov	cl,*3
shl	ax,cl
mov	bx,ax
! Debug: address unsigned short = [bx+$124] (used reg = )
! Debug: list * unsigned short = bx+$124 (used reg = )
add	bx,#$124
push	bx
! Debug: list unsigned short ebda_seg = [S+$2A-4] (used reg = )
push	-2[bp]
! Debug: func () unsigned short = read_word+0 (used reg = )
call	_read_word
add	sp,*4
! Debug: eq unsigned short = ax+0 to unsigned short iobase1 = [S+$28-$28] (used reg = )
mov	-$26[bp],ax
!BCC_EOS
! 3874         write_word(DS, SI+(Bit16u)&((dpt_t *) 0)->size, 0x42);
! Debug: list int = const $42 (used reg = )
mov	ax,*$42
push	ax
! Debug: add unsigned short = const 0 to unsigned short SI = [S+$2A+$A] (used reg = )
mov	ax,$C[bp]
! Debug: list unsigned int = ax+0 (used reg = )
push	ax
! Debug: list unsigned short DS = [S+$2C+4] (used reg = )
push	6[bp]
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
add	sp,*6
!BCC_EOS
! 3875         write_word(DS, SI+(Bit16u)&((dpt_t *) 0)->key, 0xbedd);
! Debug: list unsigned int = const $BEDD (used reg = )
mov	ax,#$BEDD
push	ax
! Debug: add unsigned short = const $1E to unsigned short SI = [S+$2A+$A] (used reg = )
mov	ax,$C[bp]
! Debug: list unsigned int = ax+$1E (used reg = )
add	ax,*$1E
push	ax
! Debug: list unsigned short DS = [S+$2C+4] (used reg = )
push	6[bp]
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
add	sp,*6
!BCC_EOS
! 3876         write_byte(DS, SI+(Bit16u)&((dpt_t *) 0)->dpi_length, 0x24);
! Debug: list int = const $24 (used reg = )
mov	ax,*$24
push	ax
! Debug: add unsigned short = const $20 to unsigned short SI = [S+$2A+$A] (used reg = )
mov	ax,$C[bp]
! Debug: list unsigned int = ax+$20 (used reg = )
add	ax,*$20
push	ax
! Debug: list unsigned short DS = [S+$2C+4] (used reg = )
push	6[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 3877         write_byte(DS, SI+(Bit16u)&((dpt_t *) 0)->reserved1, 0);
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: add unsigned short = const $21 to unsigned short SI = [S+$2A+$A] (used reg = )
mov	ax,$C[bp]
! Debug: list unsigned int = ax+$21 (used reg = )
add	ax,*$21
push	ax
! Debug: list unsigned short DS = [S+$2C+4] (used reg = )
push	6[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 3878         write_word(DS, SI+(Bit16u)&((dpt_t *) 0)->reserved2, 0);
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: add unsigned short = const $22 to unsigned short SI = [S+$2A+$A] (used reg = )
mov	ax,$C[bp]
! Debug: list unsigned int = ax+$22 (used reg = )
add	ax,*$22
push	ax
! Debug: list unsigned short DS = [S+$2C+4] (used reg = )
push	6[bp]
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
add	sp,*6
!BCC_EOS
! 3879         if (iface==0x00) {
! Debug: logeq int = const 0 to unsigned char iface = [S+$28-$24] (used reg = )
mov	al,-$22[bp]
test	al,al
jne 	.562
.563:
! 3880           write_byte(DS, SI+(Bit16u)&((dpt_t *) 0)->host_bus[0], 'I');
! Debug: list int = const $49 (used reg = )
mov	ax,*$49
push	ax
! Debug: add unsigned short = const $24 to unsigned short SI = [S+$2A+$A] (used reg = )
mov	ax,$C[bp]
! Debug: list unsigned int = ax+$24 (used reg = )
add	ax,*$24
push	ax
! Debug: list unsigned short DS = [S+$2C+4] (used reg = )
push	6[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 3881           write_byte(DS, SI+(Bit16u)&((dpt_t *) 0)->host_bus[1], 'S');
! Debug: list int = const $53 (used reg = )
mov	ax,*$53
push	ax
! Debug: add unsigned short = const $25 to unsigned short SI = [S+$2A+$A] (used reg = )
mov	ax,$C[bp]
! Debug: list unsigned int = ax+$25 (used reg = )
add	ax,*$25
push	ax
! Debug: list unsigned short DS = [S+$2C+4] (used reg = )
push	6[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 3882           write_byte(DS, SI+(Bit16u)&((dpt_t *) 0)->host_bus[2], 'A');
! Debug: list int = const $41 (used reg = )
mov	ax,*$41
push	ax
! Debug: add unsigned short = const $26 to unsigned short SI = [S+$2A+$A] (used reg = )
mov	ax,$C[bp]
! Debug: list unsigned int = ax+$26 (used reg = )
add	ax,*$26
push	ax
! Debug: list unsigned short DS = [S+$2C+4] (used reg = )
push	6[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 3883           write_byte(DS, SI+(Bit16u)&((dpt_t *) 0)->host_bus[3], 0);
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: add unsigned short = const $27 to unsigned short SI = [S+$2A+$A] (used reg = )
mov	ax,$C[bp]
! Debug: list unsigned int = ax+$27 (used reg = )
add	ax,*$27
push	ax
! Debug: list unsigned short DS = [S+$2C+4] (used reg = )
push	6[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 3884           }
! 3885         else {
jmp .564
.562:
! 3886           }
! 3887         write_byte(DS, SI+(Bit16u)&((dpt_t *) 0)->iface_type[0], 'A');
.564:
! Debug: list int = const $41 (used reg = )
mov	ax,*$41
push	ax
! Debug: add unsigned short = const $28 to unsigned short SI = [S+$2A+$A] (used reg = )
mov	ax,$C[bp]
! Debug: list unsigned int = ax+$28 (used reg = )
add	ax,*$28
push	ax
! Debug: list unsigned short DS = [S+$2C+4] (used reg = )
push	6[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 3888         write_byte(DS, SI+(Bit16u)&((dpt_t *) 0)->iface_type[1], 'T');
! Debug: list int = const $54 (used reg = )
mov	ax,*$54
push	ax
! Debug: add unsigned short = const $29 to unsigned short SI = [S+$2A+$A] (used reg = )
mov	ax,$C[bp]
! Debug: list unsigned int = ax+$29 (used reg = )
add	ax,*$29
push	ax
! Debug: list unsigned short DS = [S+$2C+4] (used reg = )
push	6[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 3889         write_byte(DS, SI+(Bit16u)&((dpt_t *) 0)->iface_type[2], 'A');
! Debug: list int = const $41 (used reg = )
mov	ax,*$41
push	ax
! Debug: add unsigned short = const $2A to unsigned short SI = [S+$2A+$A] (used reg = )
mov	ax,$C[bp]
! Debug: list unsigned int = ax+$2A (used reg = )
add	ax,*$2A
push	ax
! Debug: list unsigned short DS = [S+$2C+4] (used reg = )
push	6[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 3890         write_byte(DS, SI+(Bit16u)&((dpt_t *) 0)->iface_type[3], 0);
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: add unsigned short = const $2B to unsigned short SI = [S+$2A+$A] (used reg = )
mov	ax,$C[bp]
! Debug: list unsigned int = ax+$2B (used reg = )
add	ax,*$2B
push	ax
! Debug: list unsigned short DS = [S+$2C+4] (used reg = )
push	6[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 3891         if (iface==0x00) {
! Debug: logeq int = const 0 to unsigned char iface = [S+$28-$24] (used reg = )
mov	al,-$22[bp]
test	al,al
jne 	.565
.566:
! 3892           write_word(DS, SI+(Bit16u)&((dpt_t *) 0)->iface_path[0], iobase1);
! Debug: list unsigned short iobase1 = [S+$28-$28] (used reg = )
push	-$26[bp]
! Debug: add unsigned short = const $30 to unsigned short SI = [S+$2A+$A] (used reg = )
mov	ax,$C[bp]
! Debug: list unsigned int = ax+$30 (used reg = )
add	ax,*$30
push	ax
! Debug: list unsigned short DS = [S+$2C+4] (used reg = )
push	6[bp]
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
add	sp,*6
!BCC_EOS
! 3893           write_word(DS, SI+(Bit16u)&((dpt_t *) 0)->iface_path[2], 0);
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: add unsigned short = const $32 to unsigned short SI = [S+$2A+$A] (used reg = )
mov	ax,$C[bp]
! Debug: list unsigned int = ax+$32 (used reg = )
add	ax,*$32
push	ax
! Debug: list unsigned short DS = [S+$2C+4] (used reg = )
push	6[bp]
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
add	sp,*6
!BCC_EOS
! 3894           write_dword(DS, SI+(Bit16u)&((dpt_t *) 0)->iface_path[4], 0L);
! Debug: list long = const 0 (used reg = )
xor	ax,ax
xor	bx,bx
push	bx
push	ax
! Debug: add unsigned short = const $34 to unsigned short SI = [S+$2C+$A] (used reg = )
mov	ax,$C[bp]
! Debug: list unsigned int = ax+$34 (used reg = )
add	ax,*$34
push	ax
! Debug: list unsigned short DS = [S+$2E+4] (used reg = )
push	6[bp]
! Debug: func () void = write_dword+0 (used reg = )
call	_write_dword
add	sp,*8
!BCC_EOS
! 3895           }
! 3896         else {
jmp .567
.565:
! 3897           }
! 3898         write_byte(DS, SI+(Bit16u)&((dpt_t *) 0)->device_path[0], device%2);
.567:
! Debug: mod int = const 2 to unsigned char device = [S+$28-5] (used reg = )
mov	al,-3[bp]
xor	ah,ah
and	al,*1
! Debug: list unsigned char = al+0 (used reg = )
xor	ah,ah
push	ax
! Debug: add unsigned short = const $38 to unsigned short SI = [S+$2A+$A] (used reg = )
mov	ax,$C[bp]
! Debug: list unsigned int = ax+$38 (used reg = )
add	ax,*$38
push	ax
! Debug: list unsigned short DS = [S+$2C+4] (used reg = )
push	6[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 3899         write_byte(DS, SI+(Bit16u)&((dpt_t *) 0)->device_path[1], 0);
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: add unsigned short = const $39 to unsigned short SI = [S+$2A+$A] (used reg = )
mov	ax,$C[bp]
! Debug: list unsigned int = ax+$39 (used reg = )
add	ax,*$39
push	ax
! Debug: list unsigned short DS = [S+$2C+4] (used reg = )
push	6[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 3900         write_word(DS, SI+(Bit16u)&((dpt_t *) 0)->device_path[2], 0);
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: add unsigned short = const $3A to unsigned short SI = [S+$2A+$A] (used reg = )
mov	ax,$C[bp]
! Debug: list unsigned int = ax+$3A (used reg = )
add	ax,*$3A
push	ax
! Debug: list unsigned short DS = [S+$2C+4] (used reg = )
push	6[bp]
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
add	sp,*6
!BCC_EOS
! 3901 
! 3901         write_dword(DS, SI+(Bit16u)&((dpt_t *) 0)->device_path[4], 0L);
! Debug: list long = const 0 (used reg = )
xor	ax,ax
xor	bx,bx
push	bx
push	ax
! Debug: add unsigned short = const $3C to unsigned short SI = [S+$2C+$A] (used reg = )
mov	ax,$C[bp]
! Debug: list unsigned int = ax+$3C (used reg = )
add	ax,*$3C
push	ax
! Debug: list unsigned short DS = [S+$2E+4] (used reg = )
push	6[bp]
! Debug: func () void = write_dword+0 (used reg = )
call	_write_dword
add	sp,*8
!BCC_EOS
! 3902         checksum=0;
! Debug: eq int = const 0 to unsigned char checksum = [S+$28-$25] (used reg = )
xor	al,al
mov	-$23[bp],al
!BCC_EOS
! 3903         for (i=30; i<64; i++) checksum+=read_byte(DS, SI + i);
! Debug: eq int = const $1E to unsigned char i = [S+$28-$26] (used reg = )
mov	al,*$1E
mov	-$24[bp],al
!BCC_EOS
!BCC_EOS
jmp .56A
.56B:
! Debug: add unsigned char i = [S+$28-$26] to unsigned short SI = [S+$28+$A] (used reg = )
mov	ax,$C[bp]
add	al,-$24[bp]
adc	ah,*0
! Debug: list unsigned int = ax+0 (used reg = )
push	ax
! Debug: list unsigned short DS = [S+$2A+4] (used reg = )
push	6[bp]
! Debug: func () unsigned char = read_byte+0 (used reg = )
call	_read_byte
add	sp,*4
! Debug: addab unsigned char = al+0 to unsigned char checksum = [S+$28-$25] (used reg = )
xor	ah,ah
add	al,-$23[bp]
adc	ah,*0
mov	-$23[bp],al
!BCC_EOS
! 3904         checksum = ~checksum;
.569:
! Debug: postinc unsigned char i = [S+$28-$26] (used reg = )
mov	al,-$24[bp]
inc	ax
mov	-$24[bp],al
.56A:
! Debug: lt int = const $40 to unsigned char i = [S+$28-$26] (used reg = )
mov	al,-$24[bp]
cmp	al,*$40
jb 	.56B
.56C:
.568:
! Debug: not unsigned char checksum = [S+$28-$25] (used reg = )
mov	al,-$23[bp]
xor	ah,ah
not	ax
! Debug: eq unsigned int = ax+0 to unsigned char checksum = [S+$28-$25] (used reg = )
mov	-$23[bp],al
!BCC_EOS
! 3905         write_byte(DS, SI+(Bit16u)&((dpt_t *) 0)->checksum, checksum);
! Debug: list unsigned char checksum = [S+$28-$25] (used reg = )
mov	al,-$23[bp]
xor	ah,ah
push	ax
! Debug: add unsigned short = const $41 to unsigned short SI = [S+$2A+$A] (used reg = )
mov	ax,$C[bp]
! Debug: list unsigned int = ax+$41 (used reg = )
add	ax,*$41
push	ax
! Debug: list unsigned short DS = [S+$2C+4] (used reg = )
push	6[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 3906         }
! 3907       goto int13_success;
.560:
add	sp,#..FFE8-..FFE9
br 	.FFE8
!BCC_EOS
! 3908       break;
br 	.519
!BCC_EOS
! 3909     case 0x49:
! 3910       AX = ((AX & 0x00ff) | ((06) << 8));
.56D:
! Debug: and int = const $FF to unsigned short AX = [S+$22+$16] (used reg = )
mov	al,$18[bp]
! Debug: or int = const $600 to unsigned char = al+0 (used reg = )
xor	ah,ah
or	ax,#$600
! Debug: eq unsigned int = ax+0 to unsigned short AX = [S+$22+$16] (used reg = )
mov	$18[bp],ax
!BCC_EOS
! 3911       goto int13_fail_nostatus;
add	sp,#..FFE6-..FFE9
br 	.FFE6
!BCC_EOS
! 3912       break;
br 	.519
!BCC_EOS
! 3913     case 0x4e:
! 3914       switch (( AX & 0x00ff )) {
.56E:
! Debug: and int = const $FF to unsigned short AX = [S+$22+$16] (used reg = )
mov	al,$18[bp]
jmp .571
! 3915         case 0x01:
! 3916         case 0x03:
.572:
! 3917         case 0x04:
.573:
! 3918         case 0x06:
.574:
! 3919           goto int13_success;
.575:
add	sp,#..FFE8-..FFE9
br 	.FFE8
!BCC_EOS
! 3920           break;
jmp .56F
!BCC_EOS
! 3921         default :
! 3922           goto int13_fail;
.576:
add	sp,#..FFEA-..FFE9
br 	.FFEA
!BCC_EOS
! 3923         }
! 3924       break;
jmp .56F
.571:
sub	al,*1
je 	.572
sub	al,*2
je 	.573
sub	al,*1
je 	.574
sub	al,*2
je 	.575
jmp	.576
.56F:
br 	.519
!BCC_EOS
! 3925     case 0x02:
! 3926     case 0x04:
.577:
! 3927     case 0x08:
.578:
! 3928     case 0x0a:
.579:
! 3929     case 0x0b:
.57A:
! 3930     case 0x18:
.57B:
! 3931     case 0x50:
.57C:
! 3932     default:
.57D:
! 3933       bios_printf(4, "int13_cdrom: unsupported AH=%02x\n", ( AX >> 8 ));
.57E:
! Debug: sr int = const 8 to unsigned short AX = [S+$22+$16] (used reg = )
mov	ax,$18[bp]
mov	al,ah
xor	ah,ah
! Debug: list unsigned int = ax+0 (used reg = )
push	ax
! Debug: list * char = .57F+0 (used reg = )
mov	bx,#.57F
push	bx
! Debug: list int = const 4 (used reg = )
mov	ax,*4
push	ax
! Debug: func () void = bios_printf+0 (used reg = )
call	_bios_printf
add	sp,*6
!BCC_EOS
! 3934       goto int13_fail;
add	sp,#..FFEA-..FFE9
br 	.FFEA
!BCC_EOS
! 3935       break;
br 	.519
!BCC_EOS
! 3936     }
! 3937 int13_fail:
jmp .519
.51B:
add	sp,*-$C
sub	ax,*0
jl 	.57E
cmp	ax,*$18
ja  	.580
shl	ax,*1
mov	bx,ax
seg	cs
br	.581[bx]
.581:
.word	.51C
.word	.527
.word	.577
.word	.524
.word	.578
.word	.525
.word	.57E
.word	.57E
.word	.579
.word	.51D
.word	.57A
.word	.57B
.word	.51E
.word	.51F
.word	.57E
.word	.57E
.word	.520
.word	.521
.word	.57E
.word	.57E
.word	.522
.word	.52B
.word	.523
.word	.57E
.word	.57C
.580:
sub	ax,*$41
jb 	.57E
cmp	ax,*$F
ja  	.582
shl	ax,*1
mov	bx,ax
seg	cs
br	.583[bx]
.583:
.word	.52C
.word	.52D
.word	.526
.word	.52E
.word	.539
.word	.54C
.word	.52F
.word	.551
.word	.56D
.word	.57E
.word	.57E
.word	.57E
.word	.57E
.word	.56E
.word	.57E
.word	.57D
.582:
br 	.57E
.519:
..FFE9	=	-$2E
add	sp,*$C
.FFEA:
..FFEA	=	-$22
! 3938     AX = ((AX & 0x00ff) | ((0x01) << 8));
! Debug: and int = const $FF to unsigned short AX = [S+$22+$16] (used reg = )
mov	al,$18[bp]
! Debug: or int = const $100 to unsigned char = al+0 (used reg = )
xor	ah,ah
or	ax,#$100
! Debug: eq unsigned int = ax+0 to unsigned short AX = [S+$22+$16] (used reg = )
mov	$18[bp],ax
!BCC_EOS
! 3939 int13_fail_noah:
.FFE7:
..FFE7	=	-$22
! 3940     write_byte(0x0040, 0x0074, ( AX >> 8 ));
! Debug: sr int = const 8 to unsigned short AX = [S+$22+$16] (used reg = )
mov	ax,$18[bp]
mov	al,ah
xor	ah,ah
! Debug: list unsigned int = ax+0 (used reg = )
push	ax
! Debug: list int = const $74 (used reg = )
mov	ax,*$74
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 3941 int13_fail_nostatus:
.FFE6:
..FFE6	=	-$22
! 3942     FLAGS |= 0x0001;
! Debug: orab int = const 1 to unsigned short FLAGS = [S+$22+$1C] (used reg = )
mov	ax,$1E[bp]
or	al,*1
mov	$1E[bp],ax
!BCC_EOS
! 3943     return;
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 3944 int13_success:
.FFE8:
..FFE8	=	-$22
! 3945     AX = ((AX & 0x00ff) | ((0x00) << 8));
! Debug: and int = const $FF to unsigned short AX = [S+$22+$16] (used reg = )
mov	al,$18[bp]
! Debug: or int = const 0 to unsigned char = al+0 (used reg = )
or	al,*0
! Debug: eq unsigned char = al+0 to unsigned short AX = [S+$22+$16] (used reg = )
xor	ah,ah
mov	$18[bp],ax
!BCC_EOS
! 3946 int13_success_noah:
.FFE5:
..FFE5	=	-$22
! 3947     write_byte(0x0040, 0x0074, 0x00);
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: list int = const $74 (used reg = )
mov	ax,*$74
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 3948     FLAGS &= 0xfffe;
! Debug: andab unsigned int = const $FFFE to unsigned short FLAGS = [S+$22+$1C] (used reg = )
mov	ax,$1E[bp]
and	al,#$FE
mov	$1E[bp],ax
!BCC_EOS
! 3949     return;
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 3950 }
! 3951   void
! Register BX used in function int13_cdrom
! 3952 int13_eltorito(DS, ES, DI, SI, BP, SP, BX, DX, CX, AX, IP, CS, FLAGS)
! 3953   Bit16u DS, ES, DI, SI, BP, SP, BX, DX, CX, AX, IP, CS, FLAGS;
export	_int13_eltorito
_int13_eltorito:
!BCC_EOS
! 3954 {
! 3955   Bit16u ebda_seg=read_word(0x0040,0x000E);
push	bp
mov	bp,sp
dec	sp
dec	sp
! Debug: list int = const $E (used reg = )
mov	ax,*$E
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () unsigned short = read_word+0 (used reg = )
call	_read_word
add	sp,*4
! Debug: eq unsigned short = ax+0 to unsigned short ebda_seg = [S+4-4] (used reg = )
mov	-2[bp],ax
!BCC_EOS
! 3956   ;
!BCC_EOS
! 3957   switch (( AX >> 8 )) {
! Debug: sr int = const 8 to unsigned short AX = [S+4+$14] (used reg = )
mov	ax,$16[bp]
mov	al,ah
xor	ah,ah
br 	.586
! 3958     case 0x4a:
! 3959     case 0x4c:
.587:
! 3960     case 0x4d:
.588:
! 3961       bios_printf((2 | 4 | 1), "Int13 eltorito call with AX=%04x. Please report\n",AX);
.589:
! Debug: list unsigned short AX = [S+4+$14] (used reg = )
push	$16[bp]
! Debug: list * char = .58A+0 (used reg = )
mov	bx,#.58A
push	bx
! Debug: list int = const 7 (used reg = )
mov	ax,*7
push	ax
! Debug: func () void = bios_printf+0 (used reg = )
call	_bios_printf
add	sp,*6
!BCC_EOS
! 3962       goto int13_fail;
add	sp,#..FFE3-..FFE4
br 	.FFE3
!BCC_EOS
! 3963       break;
br 	.584
!BCC_EOS
! 3964     case 0x4b:
! 3965       write_byte(DS,SI+0x00,0x13);
.58B:
! Debug: list int = const $13 (used reg = )
mov	ax,*$13
push	ax
! Debug: add int = const 0 to unsigned short SI = [S+6+8] (used reg = )
mov	ax,$A[bp]
! Debug: list unsigned int = ax+0 (used reg = )
push	ax
! Debug: list unsigned short DS = [S+8+2] (used reg = )
push	4[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 3966       write_byte(DS,SI+0x01,read_byte(ebda_seg,&((ebda_data_t *) 0)->cdemu.media));
! Debug: list * unsigned char = const $23B (used reg = )
mov	ax,#$23B
push	ax
! Debug: list unsigned short ebda_seg = [S+6-4] (used reg = )
push	-2[bp]
! Debug: func () unsigned char = read_byte+0 (used reg = )
call	_read_byte
add	sp,*4
! Debug: list unsigned char = al+0 (used reg = )
xor	ah,ah
push	ax
! Debug: add int = const 1 to unsigned short SI = [S+6+8] (used reg = )
mov	ax,$A[bp]
! Debug: list unsigned int = ax+1 (used reg = )
inc	ax
push	ax
! Debug: list unsigned short DS = [S+8+2] (used reg = )
push	4[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 3967       write_byte(DS,SI+0x02,read_byte(ebda_seg,&((ebda_data_t *) 0)->cdemu.emulated_drive));
! Debug: list * unsigned char = const $23C (used reg = )
mov	ax,#$23C
push	ax
! Debug: list unsigned short ebda_seg = [S+6-4] (used reg = )
push	-2[bp]
! Debug: func () unsigned char = read_byte+0 (used reg = )
call	_read_byte
add	sp,*4
! Debug: list unsigned char = al+0 (used reg = )
xor	ah,ah
push	ax
! Debug: add int = const 2 to unsigned short SI = [S+6+8] (used reg = )
mov	ax,$A[bp]
! Debug: list unsigned int = ax+2 (used reg = )
inc	ax
inc	ax
push	ax
! Debug: list unsigned short DS = [S+8+2] (used reg = )
push	4[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 3968       write_byte(DS,SI+0x03,read_byte(ebda_seg,&((ebda_data_t *) 0)->cdemu.controller_index));
! Debug: list * unsigned char = const $23D (used reg = )
mov	ax,#$23D
push	ax
! Debug: list unsigned short ebda_seg = [S+6-4] (used reg = )
push	-2[bp]
! Debug: func () unsigned char = read_byte+0 (used reg = )
call	_read_byte
add	sp,*4
! Debug: list unsigned char = al+0 (used reg = )
xor	ah,ah
push	ax
! Debug: add int = const 3 to unsigned short SI = [S+6+8] (used reg = )
mov	ax,$A[bp]
! Debug: list unsigned int = ax+3 (used reg = )
add	ax,*3
push	ax
! Debug: list unsigned short DS = [S+8+2] (used reg = )
push	4[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 3969       write_dword(DS,SI+0x04,read_dword(ebda_seg,&((ebda_data_t *) 0)->cdemu.ilba));
! Debug: list * unsigned long = const $240 (used reg = )
mov	ax,#$240
push	ax
! Debug: list unsigned short ebda_seg = [S+6-4] (used reg = )
push	-2[bp]
! Debug: func () unsigned long = read_dword+0 (used reg = )
call	_read_dword
mov	bx,dx
add	sp,*4
! Debug: list unsigned long = bx+0 (used reg = )
push	bx
push	ax
! Debug: add int = const 4 to unsigned short SI = [S+8+8] (used reg = )
mov	ax,$A[bp]
! Debug: list unsigned int = ax+4 (used reg = )
add	ax,*4
push	ax
! Debug: list unsigned short DS = [S+$A+2] (used reg = )
push	4[bp]
! Debug: func () void = write_dword+0 (used reg = )
call	_write_dword
add	sp,*8
!BCC_EOS
! 3970       write_word(DS,SI+0x08,read_word(ebda_seg,&((ebda_da
! 3970 ta_t *) 0)->cdemu.device_spec));
! Debug: list * unsigned short = const $23E (used reg = )
mov	ax,#$23E
push	ax
! Debug: list unsigned short ebda_seg = [S+6-4] (used reg = )
push	-2[bp]
! Debug: func () unsigned short = read_word+0 (used reg = )
call	_read_word
add	sp,*4
! Debug: list unsigned short = ax+0 (used reg = )
push	ax
! Debug: add int = const 8 to unsigned short SI = [S+6+8] (used reg = )
mov	ax,$A[bp]
! Debug: list unsigned int = ax+8 (used reg = )
add	ax,*8
push	ax
! Debug: list unsigned short DS = [S+8+2] (used reg = )
push	4[bp]
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
add	sp,*6
!BCC_EOS
! 3971       write_word(DS,SI+0x0a,read_word(ebda_seg,&((ebda_data_t *) 0)->cdemu.buffer_segment));
! Debug: list * unsigned short = const $244 (used reg = )
mov	ax,#$244
push	ax
! Debug: list unsigned short ebda_seg = [S+6-4] (used reg = )
push	-2[bp]
! Debug: func () unsigned short = read_word+0 (used reg = )
call	_read_word
add	sp,*4
! Debug: list unsigned short = ax+0 (used reg = )
push	ax
! Debug: add int = const $A to unsigned short SI = [S+6+8] (used reg = )
mov	ax,$A[bp]
! Debug: list unsigned int = ax+$A (used reg = )
add	ax,*$A
push	ax
! Debug: list unsigned short DS = [S+8+2] (used reg = )
push	4[bp]
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
add	sp,*6
!BCC_EOS
! 3972       write_word(DS,SI+0x0c,read_word(ebda_seg,&((ebda_data_t *) 0)->cdemu.load_segment));
! Debug: list * unsigned short = const $246 (used reg = )
mov	ax,#$246
push	ax
! Debug: list unsigned short ebda_seg = [S+6-4] (used reg = )
push	-2[bp]
! Debug: func () unsigned short = read_word+0 (used reg = )
call	_read_word
add	sp,*4
! Debug: list unsigned short = ax+0 (used reg = )
push	ax
! Debug: add int = const $C to unsigned short SI = [S+6+8] (used reg = )
mov	ax,$A[bp]
! Debug: list unsigned int = ax+$C (used reg = )
add	ax,*$C
push	ax
! Debug: list unsigned short DS = [S+8+2] (used reg = )
push	4[bp]
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
add	sp,*6
!BCC_EOS
! 3973       write_word(DS,SI+0x0e,read_word(ebda_seg,&((ebda_data_t *) 0)->cdemu.sector_count));
! Debug: list * unsigned short = const $248 (used reg = )
mov	ax,#$248
push	ax
! Debug: list unsigned short ebda_seg = [S+6-4] (used reg = )
push	-2[bp]
! Debug: func () unsigned short = read_word+0 (used reg = )
call	_read_word
add	sp,*4
! Debug: list unsigned short = ax+0 (used reg = )
push	ax
! Debug: add int = const $E to unsigned short SI = [S+6+8] (used reg = )
mov	ax,$A[bp]
! Debug: list unsigned int = ax+$E (used reg = )
add	ax,*$E
push	ax
! Debug: list unsigned short DS = [S+8+2] (used reg = )
push	4[bp]
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
add	sp,*6
!BCC_EOS
! 3974       write_byte(DS,SI+0x10,read_byte(ebda_seg,&((ebda_data_t *) 0)->cdemu.vdevice.cylinders));
! Debug: list * unsigned short = const $24C (used reg = )
mov	ax,#$24C
push	ax
! Debug: list unsigned short ebda_seg = [S+6-4] (used reg = )
push	-2[bp]
! Debug: func () unsigned char = read_byte+0 (used reg = )
call	_read_byte
add	sp,*4
! Debug: list unsigned char = al+0 (used reg = )
xor	ah,ah
push	ax
! Debug: add int = const $10 to unsigned short SI = [S+6+8] (used reg = )
mov	ax,$A[bp]
! Debug: list unsigned int = ax+$10 (used reg = )
add	ax,*$10
push	ax
! Debug: list unsigned short DS = [S+8+2] (used reg = )
push	4[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 3975       write_byte(DS,SI+0x11,read_byte(ebda_seg,&((ebda_data_t *) 0)->cdemu.vdevice.spt));
! Debug: list * unsigned short = const $24E (used reg = )
mov	ax,#$24E
push	ax
! Debug: list unsigned short ebda_seg = [S+6-4] (used reg = )
push	-2[bp]
! Debug: func () unsigned char = read_byte+0 (used reg = )
call	_read_byte
add	sp,*4
! Debug: list unsigned char = al+0 (used reg = )
xor	ah,ah
push	ax
! Debug: add int = const $11 to unsigned short SI = [S+6+8] (used reg = )
mov	ax,$A[bp]
! Debug: list unsigned int = ax+$11 (used reg = )
add	ax,*$11
push	ax
! Debug: list unsigned short DS = [S+8+2] (used reg = )
push	4[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 3976       write_byte(DS,SI+0x12,read_byte(ebda_seg,&((ebda_data_t *) 0)->cdemu.vdevice.heads));
! Debug: list * unsigned short = const $24A (used reg = )
mov	ax,#$24A
push	ax
! Debug: list unsigned short ebda_seg = [S+6-4] (used reg = )
push	-2[bp]
! Debug: func () unsigned char = read_byte+0 (used reg = )
call	_read_byte
add	sp,*4
! Debug: list unsigned char = al+0 (used reg = )
xor	ah,ah
push	ax
! Debug: add int = const $12 to unsigned short SI = [S+6+8] (used reg = )
mov	ax,$A[bp]
! Debug: list unsigned int = ax+$12 (used reg = )
add	ax,*$12
push	ax
! Debug: list unsigned short DS = [S+8+2] (used reg = )
push	4[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 3977       if(( AX & 0x00ff ) == 0x00) {
! Debug: and int = const $FF to unsigned short AX = [S+4+$14] (used reg = )
mov	al,$16[bp]
! Debug: logeq int = const 0 to unsigned char = al+0 (used reg = )
test	al,al
jne 	.58C
.58D:
! 3978         write_byte(ebda_seg,&((ebda_data_t *) 0)->cdemu.active, 0x00);
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: list * unsigned char = const $23A (used reg = )
mov	ax,#$23A
push	ax
! Debug: list unsigned short ebda_seg = [S+8-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 3979         }
! 3980       goto int13_success;
.58C:
add	sp,#..FFE2-..FFE4
jmp .FFE2
!BCC_EOS
! 3981       break;
jmp .584
!BCC_EOS
! 3982     default:
! 3983       bios_printf(4, "int13_eltorito: unsupported AH=%02x\n", ( AX >> 8 ));
.58E:
! Debug: sr int = const 8 to unsigned short AX = [S+4+$14] (used reg = )
mov	ax,$16[bp]
mov	al,ah
xor	ah,ah
! Debug: list unsigned int = ax+0 (used reg = )
push	ax
! Debug: list * char = .58F+0 (used reg = )
mov	bx,#.58F
push	bx
! Debug: list int = const 4 (used reg = )
mov	ax,*4
push	ax
! Debug: func () void = bios_printf+0 (used reg = )
call	_bios_printf
add	sp,*6
!BCC_EOS
! 3984       goto int13_fail;
add	sp,#..FFE3-..FFE4
jmp .FFE3
!BCC_EOS
! 3985       break;
jmp .584
!BCC_EOS
! 3986     }
! 3987 int13_fail:
jmp .584
.586:
sub	ax,*$4A
beq 	.587
sub	ax,*1
beq 	.58B
sub	ax,*1
beq 	.588
sub	ax,*1
beq 	.589
jmp	.58E
.584:
..FFE4	=	-4
.FFE3:
..FFE3	=	-4
! 3988     AX = ((AX & 0x00ff) | ((0x01) << 8));
! Debug: and int = const $FF to unsigned short AX = [S+4+$14] (used reg = )
mov	al,$16[bp]
! Debug: or int = const $100 to unsigned char = al+0 (used reg = )
xor	ah,ah
or	ax,#$100
! Debug: eq unsigned int = ax+0 to unsigned short AX = [S+4+$14] (used reg = )
mov	$16[bp],ax
!BCC_EOS
! 3989     write_byte(0x0040, 0x0074, ( AX >> 8 ));
! Debug: sr int = const 8 to unsigned short AX = [S+4+$14] (used reg = )
mov	ax,$16[bp]
mov	al,ah
xor	ah,ah
! Debug: list unsigned int = ax+0 (used reg = )
push	ax
! Debug: list int = const $74 (used reg = )
mov	ax,*$74
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 3990     FLAGS |= 0x0001;
! Debug: orab int = const 1 to unsigned short FLAGS = [S+4+$1A] (used reg = )
mov	ax,$1C[bp]
or	al,*1
mov	$1C[bp],ax
!BCC_EOS
! 3991     return;
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 3992 int13_success:
.FFE2:
..FFE2	=	-4
! 3993     AX = ((AX & 0x00ff) | ((0x00) << 8));
! Debug: and int = const $FF to unsigned short AX = [S+4+$14] (used reg = )
mov	al,$16[bp]
! Debug: or int = const 0 to unsigned char = al+0 (used reg = )
or	al,*0
! Debug: eq unsigned char = al+0 to unsigned short AX = [S+4+$14] (used reg = )
xor	ah,ah
mov	$16[bp],ax
!BCC_EOS
! 3994     write_byte(0x0040, 0x0074, 0x00);
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: list int = const $74 (used reg = )
mov	ax,*$74
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 3995     FLAGS &= 0xfffe;
! Debug: andab unsigned int = const $FFFE to unsigned short FLAGS = [S+4+$1A] (used reg = )
mov	ax,$1C[bp]
and	al,#$FE
mov	$1C[bp],ax
!BCC_EOS
! 3996     return;
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 3997 }
! 3998   void
! Register BX used in function int13_eltorito
! 3999 int13_cdemu(DS, ES, DI, SI, BP, SP, BX, DX, CX, AX, IP, CS, FLAGS)
! 4000   Bit16u DS, ES, DI, SI, BP, SP, BX, DX, CX, AX, IP, CS, FLAGS;
export	_int13_cdemu
_int13_cdemu:
!BCC_EOS
! 4001 {
! 4002   Bit16u ebda_seg=read_word(0x0040,0x000E);
push	bp
mov	bp,sp
dec	sp
dec	sp
! Debug: list int = const $E (used reg = )
mov	ax,*$E
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () unsigned short = read_word+0 (used reg = )
call	_read_word
add	sp,*4
! Debug: eq unsigned short = ax+0 to unsigned short ebda_seg = [S+4-4] (used reg = )
mov	-2[bp],ax
!BCC_EOS
! 4003   Bit8u device, status;
!BCC_EOS
! 4004   Bit16u vheads, vspt, vcylinders;
!BCC_EOS
! 4005   Bit16u head, sector, cylinder, nbsectors;
!BCC_EOS
! 4006   Bit32u vlba, ilba, slba, elba;
!BCC_EOS
! 4007   Bit16u before, segment, offset;
!BCC_EOS
! 4008   Bit8u atacmd[12];
!BCC_EOS
! 4009   ;
add	sp,*-$32
!BCC_EOS
! 4010   device = read_byte(ebda_seg,&((ebda_data_t *) 0)->cdemu.controller_index) * 2;
! Debug: list * unsigned char = const $23D (used reg = )
mov	ax,#$23D
push	ax
! Debug: list unsigned short ebda_seg = [S+$38-4] (used reg = )
push	-2[bp]
! Debug: func () unsigned char = read_byte+0 (used reg = )
call	_read_byte
add	sp,*4
! Debug: mul int = const 2 to unsigned char = al+0 (used reg = )
xor	ah,ah
shl	ax,*1
! Debug: eq unsigned int = ax+0 to unsigned char device = [S+$36-5] (used reg = )
mov	-3[bp],al
!BCC_EOS
! 4011   device += read_byte(ebda_seg,&((ebda_data_t *) 0)->cdemu.device_spec);
! Debug: list * unsigned short = const $23E (used reg = )
mov	ax,#$23E
push	ax
! Debug: list unsigned short ebda_seg = [S+$38-4] (used reg = )
push	-2[bp]
! Debug: func () unsigned char = read_byte+0 (used reg = )
call	_read_byte
add	sp,*4
! Debug: addab unsigned char = al+0 to unsigned char device = [S+$36-5] (used reg = )
xor	ah,ah
add	al,-3[bp]
adc	ah,*0
mov	-3[bp],al
!BCC_EOS
! 4012   write_byte(0x0040, 0x0074, 0x00);
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: list int = const $74 (used reg = )
mov	ax,*$74
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 4013   if( (read_byte(ebda_seg,&((ebda_data_t *) 0)->cdemu.active) ==0 )
! 4014    || (read_byte(ebda_seg,&((ebda_data_t *) 0)->cdemu.emulated_drive ) != ( DX & 0x00ff ))) {
! Debug: list * unsigned char = const $23A (used reg = )
mov	ax,#$23A
push	ax
! Debug: list unsigned short ebda_seg = [S+$38-4] (used reg = )
push	-2[bp]
! Debug: func () unsigned char = read_byte+0 (used reg = )
call	_read_byte
add	sp,*4
! Debug: logeq int = const 0 to unsigned char = al+0 (used reg = )
test	al,al
je  	.591
.592:
! Debug: expression subtree swapping
! Debug: list * unsigned char = const $23C (used reg = )
mov	ax,#$23C
push	ax
! Debug: list unsigned short ebda_seg = [S+$38-4] (used reg = )
push	-2[bp]
! Debug: func () unsigned char = read_byte+0 (used reg = )
call	_read_byte
add	sp,*4
push	ax
! Debug: and int = const $FF to unsigned short DX = [S+$38+$10] (used reg = )
mov	al,$12[bp]
! Debug: ne unsigned char (temp) = [S+$38-$38] to unsigned char = al+0 (used reg = )
cmp	al,-$36[bp]
lea	sp,-$34[bp]
je  	.590
.591:
! 4015     bios_printf(4, "int13_cdemu: function %02x, emulation not active for DL= %02x\n", ( AX >> 8 ), ( DX & 0x00ff ));
! Debug: and int = const $FF to unsigned short DX = [S+$36+$10] (used reg = )
mov	al,$12[bp]
! Debug: list unsigned char = al+0 (used reg = )
xor	ah,ah
push	ax
! Debug: sr int = const 8 to unsigned short AX = [S+$38+$14] (used reg = )
mov	ax,$16[bp]
mov	al,ah
xor	ah,ah
! Debug: list unsigned int = ax+0 (used reg = )
push	ax
! Debug: list * char = .593+0 (used reg = )
mov	bx,#.593
push	bx
! Debug: list int = const 4 (used reg = )
mov	ax,*4
push	ax
! Debug: func () void = bios_printf+0 (used reg = )
call	_bios_printf
add	sp,*8
!BCC_EOS
! 4016     goto int13_fail;
add	sp,#..FFE1+$36
br 	.FFE1
!BCC_EOS
! 4017     }
! 4018   switch (( AX >> 8 )) {
.590:
! Debug: sr int = const 8 to unsigned short AX = [S+$36+$14] (used reg = )
mov	ax,$16[bp]
mov	al,ah
xor	ah,ah
br 	.596
! 4019     case 0x00:
! 4019 
! 4020     case 0x09:
.597:
! 4021     case 0x0c:
.598:
! 4022     case 0x0d:
.599:
! 4023     case 0x10:
.59A:
! 4024     case 0x11:
.59B:
! 4025     case 0x14:
.59C:
! 4026     case 0x16:
.59D:
! 4027       goto int13_success;
.59E:
add	sp,#..FFDF-..FFE0
br 	.FFDF
!BCC_EOS
! 4028       break;
br 	.594
!BCC_EOS
! 4029     case 0x03:
! 4030     case 0x05:
.59F:
! 4031       AX = ((AX & 0x00ff) | ((0x03) << 8));
.5A0:
! Debug: and int = const $FF to unsigned short AX = [S+$36+$14] (used reg = )
mov	al,$16[bp]
! Debug: or int = const $300 to unsigned char = al+0 (used reg = )
xor	ah,ah
or	ax,#$300
! Debug: eq unsigned int = ax+0 to unsigned short AX = [S+$36+$14] (used reg = )
mov	$16[bp],ax
!BCC_EOS
! 4032       goto int13_fail_noah;
add	sp,#..FFDE-..FFE0
br 	.FFDE
!BCC_EOS
! 4033       break;
br 	.594
!BCC_EOS
! 4034     case 0x01:
! 4035       status=read_byte(0x0040, 0x0074);
.5A1:
! Debug: list int = const $74 (used reg = )
mov	ax,*$74
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () unsigned char = read_byte+0 (used reg = )
call	_read_byte
add	sp,*4
! Debug: eq unsigned char = al+0 to unsigned char status = [S+$36-6] (used reg = )
mov	-4[bp],al
!BCC_EOS
! 4036       AX = ((AX & 0x00ff) | ((status) << 8));
! Debug: sl int = const 8 to unsigned char status = [S+$36-6] (used reg = )
mov	al,-4[bp]
xor	ah,ah
mov	ah,al
xor	al,al
push	ax
! Debug: and int = const $FF to unsigned short AX = [S+$38+$14] (used reg = )
mov	al,$16[bp]
! Debug: or unsigned int (temp) = [S+$38-$38] to unsigned char = al+0 (used reg = )
xor	ah,ah
or	ax,0+..FFE0[bp]
inc	sp
inc	sp
! Debug: eq unsigned int = ax+0 to unsigned short AX = [S+$36+$14] (used reg = )
mov	$16[bp],ax
!BCC_EOS
! 4037       write_byte(0x0040, 0x0074, 0);
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: list int = const $74 (used reg = )
mov	ax,*$74
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 4038       if (status) goto int13_fail_nostatus;
mov	al,-4[bp]
test	al,al
je  	.5A2
.5A3:
add	sp,#..FFDD-..FFE0
br 	.FFDD
!BCC_EOS
! 4039       else goto int13_success_noah;
jmp .5A4
.5A2:
add	sp,#..FFDC-..FFE0
br 	.FFDC
!BCC_EOS
! 4040       break;
.5A4:
br 	.594
!BCC_EOS
! 4041     case 0x02:
! 4042     case 0x04:
.5A5:
! 4043       vspt = read_word(ebda_seg,&((ebda_data_t *) 0)->cdemu.vdevice.spt);
.5A6:
! Debug: list * unsigned short = const $24E (used reg = )
mov	ax,#$24E
push	ax
! Debug: list unsigned short ebda_seg = [S+$38-4] (used reg = )
push	-2[bp]
! Debug: func () unsigned short = read_word+0 (used reg = )
call	_read_word
add	sp,*4
! Debug: eq unsigned short = ax+0 to unsigned short vspt = [S+$36-$A] (used reg = )
mov	-8[bp],ax
!BCC_EOS
! 4044       vcylinders = read_word(ebda_seg,&((ebda_data_t *) 0)->cdemu.vdevice.cylinders);
! Debug: list * unsigned short = const $24C (used reg = )
mov	ax,#$24C
push	ax
! Debug: list unsigned short ebda_seg = [S+$38-4] (used reg = )
push	-2[bp]
! Debug: func () unsigned short = read_word+0 (used reg = )
call	_read_word
add	sp,*4
! Debug: eq unsigned short = ax+0 to unsigned short vcylinders = [S+$36-$C] (used reg = )
mov	-$A[bp],ax
!BCC_EOS
! 4045       vheads = read_word(ebda_seg,&((ebda_data_t *) 0)->cdemu.vdevice.heads);
! Debug: list * unsigned short = const $24A (used reg = )
mov	ax,#$24A
push	ax
! Debug: list unsigned short ebda_seg = [S+$38-4] (used reg = )
push	-2[bp]
! Debug: func () unsigned short = read_word+0 (used reg = )
call	_read_word
add	sp,*4
! Debug: eq unsigned short = ax+0 to unsigned short vheads = [S+$36-8] (used reg = )
mov	-6[bp],ax
!BCC_EOS
! 4046       ilba = read_dword(ebda_seg,&((ebda_data_t *) 0)->cdemu.ilba);
! Debug: list * unsigned long = const $240 (used reg = )
mov	ax,#$240
push	ax
! Debug: list unsigned short ebda_seg = [S+$38-4] (used reg = )
push	-2[bp]
! Debug: func () unsigned long = read_dword+0 (used reg = )
call	_read_dword
mov	bx,dx
add	sp,*4
! Debug: eq unsigned long = bx+0 to unsigned long ilba = [S+$36-$1C] (used reg = )
mov	-$1A[bp],ax
mov	-$18[bp],bx
!BCC_EOS
! 4047       sector = ( CX & 0x00ff ) & 0x003f;
! Debug: and int = const $FF to unsigned short CX = [S+$36+$12] (used reg = )
mov	al,$14[bp]
! Debug: and int = const $3F to unsigned char = al+0 (used reg = )
and	al,*$3F
! Debug: eq unsigned char = al+0 to unsigned short sector = [S+$36-$10] (used reg = )
xor	ah,ah
mov	-$E[bp],ax
!BCC_EOS
! 4048       cylinder = (( CX & 0x00ff ) & 0x00c0) << 2 | ( CX >> 8 );
! Debug: sr int = const 8 to unsigned short CX = [S+$36+$12] (used reg = )
mov	ax,$14[bp]
mov	al,ah
xor	ah,ah
push	ax
! Debug: and int = const $FF to unsigned short CX = [S+$38+$12] (used reg = )
mov	al,$14[bp]
! Debug: and int = const $C0 to unsigned char = al+0 (used reg = )
and	al,#$C0
! Debug: sl int = const 2 to unsigned char = al+0 (used reg = )
xor	ah,ah
shl	ax,*1
shl	ax,*1
! Debug: or unsigned int (temp) = [S+$38-$38] to unsigned int = ax+0 (used reg = )
or	ax,0+..FFE0[bp]
inc	sp
inc	sp
! Debug: eq unsigned int = ax+0 to unsigned short cylinder = [S+$36-$12] (used reg = )
mov	-$10[bp],ax
!BCC_EOS
! 4049       head = ( DX >> 8 );
! Debug: sr int = const 8 to unsigned short DX = [S+$36+$10] (used reg = )
mov	ax,$12[bp]
mov	al,ah
xor	ah,ah
! Debug: eq unsigned int = ax+0 to unsigned short head = [S+$36-$E] (used reg = )
mov	-$C[bp],ax
!BCC_EOS
! 4050       nbsectors = ( AX & 0x00ff );
! Debug: and int = const $FF to unsigned short AX = [S+$36+$14] (used reg = )
mov	al,$16[bp]
! Debug: eq unsigned char = al+0 to unsigned short nbsectors = [S+$36-$14] (used reg = )
xor	ah,ah
mov	-$12[bp],ax
!BCC_EOS
! 4051       segment = ES;
! Debug: eq unsigned short ES = [S+$36+4] to unsigned short segment = [S+$36-$28] (used reg = )
mov	ax,6[bp]
mov	-$26[bp],ax
!BCC_EOS
! 4052       offset = BX;
! Debug: eq unsigned short BX = [S+$36+$E] to unsigned short offset = [S+$36-$2A] (used reg = )
mov	ax,$10[bp]
mov	-$28[bp],ax
!BCC_EOS
! 4053       if(nbsectors==0) goto int13_success;
! Debug: logeq int = const 0 to unsigned short nbsectors = [S+$36-$14] (used reg = )
mov	ax,-$12[bp]
test	ax,ax
jne 	.5A7
.5A8:
add	sp,#..FFDF-..FFE0
br 	.FFDF
!BCC_EOS
! 4054       if ((sector > vspt)
.5A7:
! 4055        || (cylinder >= vcylinders)
! 4056        || (head >= vheads)) {
! Debug: gt unsigned short vspt = [S+$36-$A] to unsigned short sector = [S+$36-$10] (used reg = )
mov	ax,-$E[bp]
cmp	ax,-8[bp]
ja  	.5AA
.5AC:
! Debug: ge unsigned short vcylinders = [S+$36-$C] to unsigned short cylinder = [S+$36-$12] (used reg = )
mov	ax,-$10[bp]
cmp	ax,-$A[bp]
jae 	.5AA
.5AB:
! Debug: ge unsigned short vheads = [S+$36-8] to unsigned short head = [S+$36-$E] (used reg = )
mov	ax,-$C[bp]
cmp	ax,-6[bp]
jb  	.5A9
.5AA:
! 4057         goto int13_fail;
add	sp,#..FFE1-..FFE0
br 	.FFE1
!BCC_EOS
! 4058         }
! 4059       if (( AX >> 8 ) == 0x04) goto int13_success;
.5A9:
! Debug: sr int = const 8 to unsigned short AX = [S+$36+$14] (used reg = )
mov	ax,$16[bp]
mov	al,ah
xor	ah,ah
! Debug: logeq int = const 4 to unsigned int = ax+0 (used reg = )
cmp	ax,*4
jne 	.5AD
.5AE:
add	sp,#..FFDF-..FFE0
br 	.FFDF
!BCC_EOS
! 4060       segment = ES+(BX / 16);
.5AD:
! Debug: div int = const $10 to unsigned short BX = [S+$36+$E] (used reg = )
mov	ax,$10[bp]
mov	cl,*4
shr	ax,cl
! Debug: add unsigned int = ax+0 to unsigned short ES = [S+$36+4] (used reg = )
! Debug: expression subtree swapping
add	ax,6[bp]
! Debug: eq unsigned int = ax+0 to unsigned short segment = [S+$36-$28] (used reg = )
mov	-$26[bp],ax
!BCC_EOS
! 4061       offset = BX % 16;
! Debug: mod int = const $10 to unsigned short BX = [S+$36+$E] (used reg = )
mov	ax,$10[bp]
and	al,*$F
! Debug: eq unsigned char = al+0 to unsigned short offset = [S+$36-$2A] (used reg = )
xor	ah,ah
mov	-$28[bp],ax
!BCC_EOS
! 4062       vlba=((((Bit32u)cylinder*(Bit32u)vheads)+(Bit32u)head)*(Bit32u)vspt)+((Bit32u)(sector-1));
! Debug: sub int = const 1 to unsigned short sector = [S+$36-$10] (used reg = )
mov	ax,-$E[bp]
! Debug: cast unsigned long = const 0 to unsigned int = ax-1 (used reg = )
dec	ax
xor	bx,bx
push	bx
push	ax
! Debug: cast unsigned long = const 0 to unsigned short vspt = [S+$3A-$A] (used reg = )
mov	ax,-8[bp]
xor	bx,bx
push	bx
push	ax
! Debug: cast unsigned long = const 0 to unsigned short head = [S+$3E-$E] (used reg = )
mov	ax,-$C[bp]
xor	bx,bx
push	bx
push	ax
! Debug: cast unsigned long = const 0 to unsigned short vheads = [S+$42-8] (used reg = )
mov	ax,-6[bp]
xor	bx,bx
push	bx
push	ax
! Debug: cast unsigned long = const 0 to unsigned short cylinder = [S+$46-$12] (used reg = )
mov	ax,-$10[bp]
xor	bx,bx
! Debug: mul unsigned long (temp) = [S+$46-$46] to unsigned long = bx+0 (used reg = )
lea	di,-$E+..FFE0[bp]
call	lmulul
add	sp,*4
! Debug: add unsigned long (temp) = [S+$42-$42] to unsigned long = bx+0 (used reg = )
lea	di,-$A+..FFE0[bp]
call	laddul
add	sp,*4
! Debug: mul unsigned long (temp) = [S+$3E-$3E] to unsigned long = bx+0 (used reg = )
lea	di,-6+..FFE0[bp]
call	lmulul
add	sp,*4
! Debug: add unsigned long (temp) = [S+$3A-$3A] to unsigned long = bx+0 (used reg = )
lea	di,-2+..FFE0[bp]
call	laddul
add	sp,*4
! Debug: eq unsigned long = bx+0 to unsigned long vlba = [S+$36-$18] (used reg = )
mov	-$16[bp],ax
mov	-$14[bp],bx
!BCC_EOS
! 4063       AX = ((AX & 0xff00) | (nbsectors));
! Debug: and unsigned int = const $FF00 to unsigned short AX = [S+$36+$14] (used reg = )
mov	ax,$16[bp]
xor	al,al
! Debug: or unsigned short nbsectors = [S+$36-$14] to unsigned int = ax+0 (used reg = )
or	ax,-$12[bp]
! Debug: eq unsigned int = ax+0 to unsigned short AX = [S+$36+$14] (used reg = )
mov	$16[bp],ax
!BCC_EOS
! 4064       slba = (Bit32u)vlba/4;
! Debug: div unsigned long = const 4 to unsigned long vlba = [S+$36-$18] (used reg = )
mov	ax,*4
xor	bx,bx
push	bx
push	ax
mov	ax,-$16[bp]
mov	bx,-$14[bp]
lea	di,-2+..FFE0[bp]
call	ldivul
add	sp,*4
! Debug: eq unsigned long = bx+0 to unsigned long slba = [S+$36-$20] (used reg = )
mov	-$1E[bp],ax
mov	-$1C[bp],bx
!BCC_EOS
! 4065       before= (Bit16u)vlba%4;
! Debug: mod int = const 4 to unsigned short vlba = [S+$36-$18] (used reg = )
mov	ax,-$16[bp]
and	al,*3
! Debug: eq unsigned char = al+0 to unsigned short before = [S+$36-$26] (used reg = )
xor	ah,ah
mov	-$24[bp],ax
!BCC_EOS
! 4066       elba = (Bit32u)(vlba+nbsectors-1)/4;
! Debug: cast unsigned long = const 0 to unsigned short nbsectors = [S+$36-$14] (used reg = )
mov	ax,-$12[bp]
xor	bx,bx
! Debug: add unsigned long = bx+0 to unsigned long vlba = [S+$36-$18] (used reg = )
! Debug: expression subtree swapping
lea	di,-$16[bp]
call	laddul
! Debug: sub unsigned long = const 1 to unsigned long = bx+0 (used reg = )
push	bx
push	ax
mov	ax,*1
xor	bx,bx
push	bx
push	ax
mov	ax,-2+..FFE0[bp]
mov	bx,0+..FFE0[bp]
lea	di,-6+..FFE0[bp]
call	lsubul
add	sp,*8
! Debug: cast unsigned long = const 0 to unsigned long = bx+0 (used reg = )
! Debug: div unsigned long = const 4 to unsigned long = bx+0 (used reg = )
push	bx
push	ax
mov	ax,*4
xor	bx,bx
push	bx
push	ax
mov	ax,-2+..FFE0[bp]
mov	bx,0+..FFE0[bp]
lea	di,-6+..FFE0[bp]
call	ldivul
add	sp,*8
! Debug: eq unsigned long = bx+0 to unsigned long elba = [S+$36-$24] (used reg = )
mov	-$22[bp],ax
mov	-$20[bp],bx
!BCC_EOS
! 4067       memsetb(get_SS(),atacmd,0,12);
! Debug: list int = const $C (used reg = )
mov	ax,*$C
push	ax
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: list * unsigned char atacmd = S+$3A-$36 (used reg = )
lea	bx,-$34[bp]
push	bx
! Debug: func () unsigned short = get_SS+0 (used reg = )
call	_get_SS
! Debug: list unsigned short = ax+0 (used reg = )
push	ax
! Debug: func () void = memsetb+0 (used reg = )
call	_memsetb
add	sp,*8
!BCC_EOS
! 4068       atacmd[0]=0x28;
! Debug: eq int = const $28 to unsigned char atacmd = [S+$36-$36] (used reg = )
mov	al,*$28
mov	-$34[bp],al
!BCC_EOS
! 4069       atacmd[7]=((Bit16u)(elba-slba+1) & 0xff00) >> 8;
! Debug: sub unsigned long slba = [S+$36-$20] to unsigned long elba = [S+$36-$24] (used reg = )
mov	ax,-$22[bp]
mov	bx,-$20[bp]
lea	di,-$1E[bp]
call	lsubul
! Debug: add unsigned long = const 1 to unsigned long = bx+0 (used reg = )
push	bx
push	ax
mov	ax,*1
xor	bx,bx
push	bx
push	ax
mov	ax,-2+..FFE0[bp]
mov	bx,0+..FFE0[bp]
lea	di,-6+..FFE0[bp]
call	laddul
add	sp,*8
! Debug: cast unsigned short = const 0 to unsigned long = bx+0 (used reg = )
! Debug: and unsigned int = const $FF00 to unsigned short = ax+0 (used reg = )
xor	al,al
! Debug: sr int = const 8 to unsigned int = ax+0 (used reg = )
mov	al,ah
xor	ah,ah
! Debug: eq unsigned int = ax+0 to unsigned char atacmd = [S+$36-$2F] (used reg = )
mov	-$2D[bp],al
!BCC_EOS
! 4070       atacmd[8]=((Bit16u)(elba-slba+1) & 0x00ff);
! Debug: sub unsigned long slba = [S+$36-$20] to unsigned long elba = [S+$36-$24] (used reg = )
mov	ax,-$22[bp]
mov	bx,-$20[bp]
lea	di,-$1E[bp]
call	lsubul
! Debug: add unsigned long = const 1 to unsigned long = bx+0 (used reg = )
push	bx
push	ax
mov	ax,*1
xor	bx,bx
push	bx
push	ax
mov	ax,-2+..FFE0[bp]
mov	bx,0+..FFE0[bp]
lea	di,-6+..FFE0[bp]
call	laddul
add	sp,*8
! Debug: cast unsigned short = const 0 to unsigned long = bx+0 (used reg = )
! Debug: and int = const $FF to unsigned short = ax+0 (used reg = )
! Debug: eq unsigned char = al+0 to unsigned char atacmd = [S+$36-$2E] (used reg = )
mov	-$2C[bp],al
!BCC_EOS
! 4071       atacmd[2]=(ilba+slba & 0xff000000) >> 24;
! Debug: add unsigned long slba = [S+$36-$20] to unsigned long ilba = [S+$36-$1C] (used reg = )
mov	ax,-$1A[bp]
mov	bx,-$18[bp]
lea	di,-$1E[bp]
call	laddul
! Debug: and unsigned long = const $FF000000 to unsigned long = bx+0 (used reg = )
push	bx
push	ax
xor	ax,ax
mov	bx,#$FF00
push	bx
push	ax
mov	ax,-2+..FFE0[bp]
mov	bx,0+..FFE0[bp]
lea	di,-6+..FFE0[bp]
call	landul
add	sp,*8
! Debug: sr int = const $18 to unsigned long = bx+0 (used reg = )
xchg	bx,ax
mov	al,ah
xor	ah,ah
xor	bx,bx
! Debug: eq unsigned long = bx+0 to unsigned char atacmd = [S+$36-$34] (used reg = )
mov	-$32[bp],al
!BCC_EOS
! 4072       atacmd[3]=(ilba+slba & 0x00ff0000) >> 16;
! Debug: add unsigned long slba = [S+$36-$20] to unsigned long ilba = [S+$36-$1C] (used reg = )
mov	ax,-$1A[bp]
mov	bx,-$18[bp]
lea	di,-$1E[bp]
call	laddul
! Debug: and long = const $FF0000 to unsigned long = bx+0 (used reg = )
push	bx
push	ax
xor	ax,ax
mov	bx,#$FF
push	bx
push	ax
mov	ax,-2+..FFE0[bp]
mov	bx,0+..FFE0[bp]
lea	di,-6+..FFE0[bp]
call	landul
add	sp,*8
! Debug: sr int = const $10 to unsigned long = bx+0 (used reg = )
xchg	bx,ax
xor	bx,bx
! Debug: eq unsigned long = bx+0 to unsigned char atacmd = [S+$36-$33] (used reg = )
mov	-$31[bp],al
!BCC_EOS
! 4073       atacmd[4]=(ilba+slba & 0x0000ff00) >> 8;
! Debug: add unsigned long slba = [S+$36-$20] to unsigned long ilba = [S+$36-$1C] (used reg = )
mov	ax,-$1A[bp]
mov	bx,-$18[bp]
lea	di,-$1E[bp]
call	laddul
! Debug: and unsigned long = const $FF00 to unsigned long = bx+0 (used reg = )
push	bx
push	ax
mov	ax,#$FF00
xor	bx,bx
push	bx
push	ax
mov	ax,-2+..FFE0[bp]
mov	bx,0+..FFE0[bp]
lea	di,-6+..FFE0[bp]
call	landul
add	sp,*8
! Debug: sr int = const 8 to unsigned long = bx+0 (used reg = )
mov	al,ah
mov	ah,bl
mov	bl,bh
sub	bh,bh
! Debug: eq unsigned long = bx+0 to unsigned char atacmd = [S+$36-$32] (used reg = )
mov	-$30[bp],al
!BCC_EOS
! 4074       atacmd[5]=(ilba+slba & 0x000000ff);
! Debug: add unsigned long slba = [S+$36-$20] to unsigned long ilba = [S+$36-$1C] (used reg = )
mov	ax,-$1A[bp]
mov	bx,-$18[bp]
lea	di,-$1E[bp]
call	laddul
! Debug: and unsigned long = const $FF to unsigned long = bx+0 (used reg = )
push	bx
push	ax
mov	ax,#$FF
xor	bx,bx
push	bx
push	ax
mov	ax,-2+..FFE0[bp]
mov	bx,0+..FFE0[bp]
lea	di,-6+..FFE0[bp]
call	landul
add	sp,*8
! Debug: eq unsigned long = bx+0 to unsigned char atacmd = [S+$36-$31] (used reg = )
mov	-$2F[bp],al
!BCC_EOS
! 4075       if((status = ata_cmd_packet(device, 12, get_SS(), atacmd, before*512, nbsectors*512L, 0x01, segment,offset)) != 0) {
! Debug: list unsigned short offset = [S+$36-$2A] (used reg = )
push	-$28[bp]
! Debug: list unsigned short segment = [S+$38-$28] (used reg = )
push	-$26[bp]
! Debug: list int = const 1 (used reg = )
mov	ax,*1
push	ax
! Debug: cast unsigned long = const 0 to unsigned short nbsectors = [S+$3C-$14] (used reg = )
mov	ax,-$12[bp]
xor	bx,bx
! Debug: mul long = const $200 to unsigned long = bx+0 (used reg = )
push	bx
push	ax
mov	ax,#$200
xor	bx,bx
push	bx
push	ax
mov	ax,-8+..FFE0[bp]
mov	bx,-6+..FFE0[bp]
lea	di,-$C+..FFE0[bp]
call	lmulul
add	sp,*8
! Debug: list unsigned long = bx+0 (used reg = )
push	bx
push	ax
! Debug: mul int = const $200 to unsigned short before = [S+$40-$26] (used reg = )
mov	ax,-$24[bp]
mov	cx,#$200
imul	cx
! Debug: list unsigned int = ax+0 (used reg = )
push	ax
! Debug: list * unsigned char atacmd = S+$42-$36 (used reg = )
lea	bx,-$34[bp]
push	bx
! Debug: func () unsigned short = get_SS+0 (used reg = )
call	_get_SS
! Debug: list unsigned short = ax+0 (used reg = )
push	ax
! Debug: list int = const $C (used reg = )
mov	ax,*$C
push	ax
! Debug: list unsigned char device = [S+$48-5] (used reg = )
mov	al,-3[bp]
xor	ah,ah
push	ax
! Debug: func () unsigned short = ata_cmd_packet+0 (used reg = )
call	_ata_cmd_packet
add	sp,*$14
! Debug: eq unsigned short = ax+0 to unsigned char status = [S+$36-6] (used reg = )
mov	-4[bp],al
! Debug: ne int = const 0 to unsigned char = al+0 (used reg = )
test	al,al
je  	.5AF
.5B0:
! 4076         bios_printf(4, "int13
! 4076 _cdemu: function %02x, error %02x !\n",( AX >> 8 ),status);
! Debug: list unsigned char status = [S+$36-6] (used reg = )
mov	al,-4[bp]
xor	ah,ah
push	ax
! Debug: sr int = const 8 to unsigned short AX = [S+$38+$14] (used reg = )
mov	ax,$16[bp]
mov	al,ah
xor	ah,ah
! Debug: list unsigned int = ax+0 (used reg = )
push	ax
! Debug: list * char = .5B1+0 (used reg = )
mov	bx,#.5B1
push	bx
! Debug: list int = const 4 (used reg = )
mov	ax,*4
push	ax
! Debug: func () void = bios_printf+0 (used reg = )
call	_bios_printf
add	sp,*8
!BCC_EOS
! 4077         AX = ((AX & 0x00ff) | ((0x02) << 8));
! Debug: and int = const $FF to unsigned short AX = [S+$36+$14] (used reg = )
mov	al,$16[bp]
! Debug: or int = const $200 to unsigned char = al+0 (used reg = )
xor	ah,ah
or	ax,#$200
! Debug: eq unsigned int = ax+0 to unsigned short AX = [S+$36+$14] (used reg = )
mov	$16[bp],ax
!BCC_EOS
! 4078         AX = ((AX & 0xff00) | (0));
! Debug: and unsigned int = const $FF00 to unsigned short AX = [S+$36+$14] (used reg = )
mov	ax,$16[bp]
xor	al,al
! Debug: or int = const 0 to unsigned int = ax+0 (used reg = )
or	al,*0
! Debug: eq unsigned int = ax+0 to unsigned short AX = [S+$36+$14] (used reg = )
mov	$16[bp],ax
!BCC_EOS
! 4079         goto int13_fail_noah;
add	sp,#..FFDE-..FFE0
br 	.FFDE
!BCC_EOS
! 4080         }
! 4081       goto int13_success;
.5AF:
add	sp,#..FFDF-..FFE0
br 	.FFDF
!BCC_EOS
! 4082       break;
br 	.594
!BCC_EOS
! 4083     case 0x08:
! 4084       vspt=read_word(ebda_seg,&((ebda_data_t *) 0)->cdemu.vdevice.spt);
.5B2:
! Debug: list * unsigned short = const $24E (used reg = )
mov	ax,#$24E
push	ax
! Debug: list unsigned short ebda_seg = [S+$38-4] (used reg = )
push	-2[bp]
! Debug: func () unsigned short = read_word+0 (used reg = )
call	_read_word
add	sp,*4
! Debug: eq unsigned short = ax+0 to unsigned short vspt = [S+$36-$A] (used reg = )
mov	-8[bp],ax
!BCC_EOS
! 4085       vcylinders=read_word(ebda_seg,&((ebda_data_t *) 0)->cdemu.vdevice.cylinders) - 1;
! Debug: list * unsigned short = const $24C (used reg = )
mov	ax,#$24C
push	ax
! Debug: list unsigned short ebda_seg = [S+$38-4] (used reg = )
push	-2[bp]
! Debug: func () unsigned short = read_word+0 (used reg = )
call	_read_word
add	sp,*4
! Debug: sub int = const 1 to unsigned short = ax+0 (used reg = )
! Debug: eq unsigned int = ax-1 to unsigned short vcylinders = [S+$36-$C] (used reg = )
dec	ax
mov	-$A[bp],ax
!BCC_EOS
! 4086       vheads=read_word(ebda_seg,&((ebda_data_t *) 0)->cdemu.vdevice.heads) - 1;
! Debug: list * unsigned short = const $24A (used reg = )
mov	ax,#$24A
push	ax
! Debug: list unsigned short ebda_seg = [S+$38-4] (used reg = )
push	-2[bp]
! Debug: func () unsigned short = read_word+0 (used reg = )
call	_read_word
add	sp,*4
! Debug: sub int = const 1 to unsigned short = ax+0 (used reg = )
! Debug: eq unsigned int = ax-1 to unsigned short vheads = [S+$36-8] (used reg = )
dec	ax
mov	-6[bp],ax
!BCC_EOS
! 4087       AX = ((AX & 0xff00) | (0x00));
! Debug: and unsigned int = const $FF00 to unsigned short AX = [S+$36+$14] (used reg = )
mov	ax,$16[bp]
xor	al,al
! Debug: or int = const 0 to unsigned int = ax+0 (used reg = )
or	al,*0
! Debug: eq unsigned int = ax+0 to unsigned short AX = [S+$36+$14] (used reg = )
mov	$16[bp],ax
!BCC_EOS
! 4088       BX = ((BX & 0xff00) | (0x00));
! Debug: and unsigned int = const $FF00 to unsigned short BX = [S+$36+$E] (used reg = )
mov	ax,$10[bp]
xor	al,al
! Debug: or int = const 0 to unsigned int = ax+0 (used reg = )
or	al,*0
! Debug: eq unsigned int = ax+0 to unsigned short BX = [S+$36+$E] (used reg = )
mov	$10[bp],ax
!BCC_EOS
! 4089       CX = ((CX & 0x00ff) | ((vcylinders & 0xff) << 8));
! Debug: and int = const $FF to unsigned short vcylinders = [S+$36-$C] (used reg = )
mov	al,-$A[bp]
! Debug: sl int = const 8 to unsigned char = al+0 (used reg = )
xor	ah,ah
mov	ah,al
xor	al,al
push	ax
! Debug: and int = const $FF to unsigned short CX = [S+$38+$12] (used reg = )
mov	al,$14[bp]
! Debug: or unsigned int (temp) = [S+$38-$38] to unsigned char = al+0 (used reg = )
xor	ah,ah
or	ax,0+..FFE0[bp]
inc	sp
inc	sp
! Debug: eq unsigned int = ax+0 to unsigned short CX = [S+$36+$12] (used reg = )
mov	$14[bp],ax
!BCC_EOS
! 4090       CX = ((CX & 0xff00) | ((( vcylinders >> 2) & 0xc0) | ( vspt & 0x3f )));
! Debug: and int = const $3F to unsigned short vspt = [S+$36-$A] (used reg = )
mov	al,-8[bp]
and	al,*$3F
push	ax
! Debug: sr int = const 2 to unsigned short vcylinders = [S+$38-$C] (used reg = )
mov	ax,-$A[bp]
shr	ax,*1
shr	ax,*1
! Debug: and int = const $C0 to unsigned int = ax+0 (used reg = )
and	al,#$C0
! Debug: or unsigned char (temp) = [S+$38-$38] to unsigned char = al+0 (used reg = )
or	al,0+..FFE0[bp]
inc	sp
inc	sp
push	ax
! Debug: and unsigned int = const $FF00 to unsigned short CX = [S+$38+$12] (used reg = )
mov	ax,$14[bp]
xor	al,al
! Debug: or unsigned char (temp) = [S+$38-$38] to unsigned int = ax+0 (used reg = )
or	al,0+..FFE0[bp]
inc	sp
inc	sp
! Debug: eq unsigned int = ax+0 to unsigned short CX = [S+$36+$12] (used reg = )
mov	$14[bp],ax
!BCC_EOS
! 4091       DX = ((DX & 0x00ff) | ((vheads) << 8));
! Debug: sl int = const 8 to unsigned short vheads = [S+$36-8] (used reg = )
mov	ax,-6[bp]
mov	ah,al
xor	al,al
push	ax
! Debug: and int = const $FF to unsigned short DX = [S+$38+$10] (used reg = )
mov	al,$12[bp]
! Debug: or unsigned int (temp) = [S+$38-$38] to unsigned char = al+0 (used reg = )
xor	ah,ah
or	ax,0+..FFE0[bp]
inc	sp
inc	sp
! Debug: eq unsigned int = ax+0 to unsigned short DX = [S+$36+$10] (used reg = )
mov	$12[bp],ax
!BCC_EOS
! 4092       DX = ((DX & 0xff00) | (0x02));
! Debug: and unsigned int = const $FF00 to unsigned short DX = [S+$36+$10] (used reg = )
mov	ax,$12[bp]
xor	al,al
! Debug: or int = const 2 to unsigned int = ax+0 (used reg = )
or	al,*2
! Debug: eq unsigned int = ax+0 to unsigned short DX = [S+$36+$10] (used reg = )
mov	$12[bp],ax
!BCC_EOS
! 4093       switch(read_byte(ebda_seg,&((ebda_data_t *) 0)->cdemu.media)) {
! Debug: list * unsigned char = const $23B (used reg = )
mov	ax,#$23B
push	ax
! Debug: list unsigned short ebda_seg = [S+$38-4] (used reg = )
push	-2[bp]
! Debug: func () unsigned char = read_byte+0 (used reg = )
call	_read_byte
add	sp,*4
jmp .5B5
! 4094         case 0x01: BX = ((BX & 0xff00) | (0x02)); break;
.5B6:
! Debug: and unsigned int = const $FF00 to unsigned short BX = [S+$36+$E] (used reg = )
mov	ax,$10[bp]
xor	al,al
! Debug: or int = const 2 to unsigned int = ax+0 (used reg = )
or	al,*2
! Debug: eq unsigned int = ax+0 to unsigned short BX = [S+$36+$E] (used reg = )
mov	$10[bp],ax
!BCC_EOS
jmp .5B3
!BCC_EOS
! 4095         case 0x02: BX = ((BX & 0xff00) | (0x04)); break;
.5B7:
! Debug: and unsigned int = const $FF00 to unsigned short BX = [S+$36+$E] (used reg = )
mov	ax,$10[bp]
xor	al,al
! Debug: or int = const 4 to unsigned int = ax+0 (used reg = )
or	al,*4
! Debug: eq unsigned int = ax+0 to unsigned short BX = [S+$36+$E] (used reg = )
mov	$10[bp],ax
!BCC_EOS
jmp .5B3
!BCC_EOS
! 4096         case 0x03: BX = ((BX & 0xff00) | (0x06)); break;
.5B8:
! Debug: and unsigned int = const $FF00 to unsigned short BX = [S+$36+$E] (used reg = )
mov	ax,$10[bp]
xor	al,al
! Debug: or int = const 6 to unsigned int = ax+0 (used reg = )
or	al,*6
! Debug: eq unsigned int = ax+0 to unsigned short BX = [S+$36+$E] (used reg = )
mov	$10[bp],ax
!BCC_EOS
jmp .5B3
!BCC_EOS
! 4097         }
! 4098 #asm
jmp .5B3
.5B5:
sub	al,*1
je 	.5B6
sub	al,*1
je 	.5B7
sub	al,*1
je 	.5B8
.5B3:
!BCC_EOS
!BCC_ASM
_int13_cdemu.BP	set	$40
.int13_cdemu.BP	set	$C
_int13_cdemu.CS	set	$4E
.int13_cdemu.CS	set	$1A
_int13_cdemu.nbsectors	set	$22
.int13_cdemu.nbsectors	set	-$12
_int13_cdemu.CX	set	$48
.int13_cdemu.CX	set	$14
_int13_cdemu.elba	set	$12
.int13_cdemu.elba	set	-$22
_int13_cdemu.segment	set	$E
.int13_cdemu.segment	set	-$26
_int13_cdemu.DI	set	$3C
.int13_cdemu.DI	set	8
_int13_cdemu.FLAGS	set	$50
.int13_cdemu.FLAGS	set	$1C
_int13_cdemu.vcylinders	set	$2A
.int13_cdemu.vcylinders	set	-$A
_int13_cdemu.sector	set	$26
.int13_cdemu.sector	set	-$E
_int13_cdemu.DS	set	$38
.int13_cdemu.DS	set	4
_int13_cdemu.head	set	$28
.int13_cdemu.head	set	-$C
_int13_cdemu.cylinder	set	$24
.int13_cdemu.cylinder	set	-$10
_int13_cdemu.DX	set	$46
.int13_cdemu.DX	set	$12
_int13_cdemu.device	set	$31
.int13_cdemu.device	set	-3
_int13_cdemu.ES	set	$3A
.int13_cdemu.ES	set	6
_int13_cdemu.vspt	set	$2C
.int13_cdemu.vspt	set	-8
_int13_cdemu.vlba	set	$1E
.int13_cdemu.vlba	set	-$16
_int13_cdemu.ebda_seg	set	$32
.int13_cdemu.ebda_seg	set	-2
_int13_cdemu.SI	set	$3E
.int13_cdemu.SI	set	$A
_int13_cdemu.IP	set	$4C
.int13_cdemu.IP	set	$18
_int13_cdemu.status	set	$30
.int13_cdemu.status	set	-4
_int13_cdemu.atacmd	set	0
.int13_cdemu.atacmd	set	-$34
_int13_cdemu.AX	set	$4A
.int13_cdemu.AX	set	$16
_int13_cdemu.ilba	set	$1A
.int13_cdemu.ilba	set	-$1A
_int13_cdemu.before	set	$10
.int13_cdemu.before	set	-$24
_int13_cdemu.offset	set	$C
.int13_cdemu.offset	set	-$28
_int13_cdemu.slba	set	$16
.int13_cdemu.slba	set	-$1E
_int13_cdemu.SP	set	$42
.int13_cdemu.SP	set	$E
_int13_cdemu.vheads	set	$2E
.int13_cdemu.vheads	set	-6
_int13_cdemu.BX	set	$44
.int13_cdemu.BX	set	$10
      push bp
      mov bp, sp
      mov ax, #diskette_param_table2
      mov _int13_cdemu.DI+2[bp], ax
      mov _int13_cdemu.ES+2[bp], cs
      pop bp
! 4105 endasm
!BCC_ENDASM
!BCC_EOS
! 4106       goto int13_success;
add	sp,#..FFDF-..FFE0
br 	.FFDF
!BCC_EOS
! 4107       break;
br 	.594
!BCC_EOS
! 4108     case 0x15:
! 4109       AX = ((AX & 0x00ff) | ((0x03) << 8));
.5B9:
! Debug: and int = const $FF to unsigned short AX = [S+$36+$14] (used reg = )
mov	al,$16[bp]
! Debug: or int = const $300 to unsigned char = al+0 (used reg = )
xor	ah,ah
or	ax,#$300
! Debug: eq unsigned int = ax+0 to unsigned short AX = [S+$36+$14] (used reg = )
mov	$16[bp],ax
!BCC_EOS
! 4110       goto int13_success_noah;
add	sp,#..FFDC-..FFE0
br 	.FFDC
!BCC_EOS
! 4111       break;
br 	.594
!BCC_EOS
! 4112     case 0x0a:
! 4113     case 0x0b:
.5BA:
! 4114     case 0x18:
.5BB:
! 4115     case 0x41:
.5BC:
! 4116     case 0x42:
.5BD:
! 4117     case 0x43:
.5BE:
! 4118     case 0x44:
.5BF:
! 4119     case 0x45:
.5C0:
! 4120     case 0x46:
.5C1:
! 4121     case 0x47:
.5C2:
! 4122     case 0x48:
.5C3:
! 4123     case 0x49:
.5C4:
! 4124     case 0x4e:
.5C5:
! 4125     case 0x50:
.5C6:
! 4126     default:
.5C7:
! 4127       bios_printf(4, "int13_cdemu function AH=%02x unsupported, returns fail\n", ( AX >> 8 ));
.5C8:
! Debug: sr int = const 8 to unsigned short AX = [S+$36+$14] (used reg = )
mov	ax,$16[bp]
mov	al,ah
xor	ah,ah
! Debug: list unsigned int = ax+0 (used reg = )
push	ax
! Debug: list * char = .5C9+0 (used reg = )
mov	bx,#.5C9
push	bx
! Debug: list int = const 4 (used reg = )
mov	ax,*4
push	ax
! Debug: func () void = bios_printf+0 (used reg = )
call	_bios_printf
add	sp,*6
!BCC_EOS
! 4128       goto int13_fail;
add	sp,#..FFE1-..FFE0
jmp .FFE1
!BCC_EOS
! 4129       break;
jmp .594
!BCC_EOS
! 4130     }
! 4131 int13_fail:
jmp .594
.596:
sub	ax,*0
jl 	.5C8
cmp	ax,*$18
ja  	.5CA
shl	ax,*1
mov	bx,ax
seg	cs
br	.5CB[bx]
.5CB:
.word	.597
.word	.5A1
.word	.5A5
.word	.59F
.word	.5A6
.word	.5A0
.word	.5C8
.word	.5C8
.word	.5B2
.word	.598
.word	.5BA
.word	.5BB
.word	.599
.word	.59A
.word	.5C8
.word	.5C8
.word	.59B
.word	.59C
.word	.5C8
.word	.5C8
.word	.59D
.word	.5B9
.word	.59E
.word	.5C8
.word	.5BC
.5CA:
sub	ax,*$41
jb 	.5C8
cmp	ax,*$F
ja  	.5CC
shl	ax,*1
mov	bx,ax
seg	cs
br	.5CD[bx]
.5CD:
.word	.5BD
.word	.5BE
.word	.5BF
.word	.5C0
.word	.5C1
.word	.5C2
.word	.5C3
.word	.5C4
.word	.5C5
.word	.5C8
.word	.5C8
.word	.5C8
.word	.5C8
.word	.5C6
.word	.5C8
.word	.5C7
.5CC:
br 	.5C8
.594:
..FFE0	=	-$36
.FFE1:
..FFE1	=	-$36
! 4132     AX = ((AX & 0x00ff) | ((0x01) << 8));
! Debug: and int = const $FF to unsigned short AX = [S+$36+$14] (used reg = )
mov	al,$16[bp]
! Debug: or int = const $100 to unsigned char = al+0 (used reg = )
xor	ah,ah
or	ax,#$100
! Debug: eq unsigned int = ax+0 to unsigned short AX = [S+$36+$14] (used reg = )
mov	$16[bp],ax
!BCC_EOS
! 4133 int13_fail_noah:
.FFDE:
..FFDE	=	-$36
! 4134     write_byte(0x0040, 0x0074, ( AX >> 8 ));
! Debug: sr int = const 8 to unsigned short AX = [S+$36+$14] (used reg = )
mov	ax,$16[bp]
mov	al,ah
xor	ah,ah
! Debug: list unsigned int = ax+0 (used reg = )
push	ax
! Debug: list int = const $74 (used reg = )
mov	ax,*$74
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 4135 int13_fail_nostatus:
.FFDD:
..FFDD	=	-$36
! 4136     FLAGS |= 0x0001;
! Debug: orab int = const 1 to unsigned short FLAGS = [S+$36+$1A] (used reg = )
mov	ax,$1C[bp]
or	al,*1
mov	$1C[bp],ax
!BCC_EOS
! 4137     return;
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 4138 int13_success:
.FFDF:
..FFDF	=	-$36
! 4139     AX = ((AX & 0x00ff) | ((0x00) << 8));
! Debug: and int = const $FF to unsigned short AX = [S+$36+$14] (used reg = )
mov	al,$16[bp]
! Debug: or int = const 0 to unsigned char = al+0 (used reg = )
or	al,*0
! Debug: eq unsigned char = al+0 to unsigned short AX = [S+$36+$14] (used reg = )
xor	ah,ah
mov	$16[bp],ax
!BCC_EOS
! 4140 int13_success_noah:
.FFDC:
..FFDC	=	-$36
! 4141     write_byte(0x0040, 0x0074, 0x00);
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: list int = const $74 (used reg = )
mov	ax,*$74
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 4142     FLAGS &= 0xfffe;
! Debug: andab unsigned int = const $FFFE to unsigned short FLAGS = [S+$36+$1A] (used reg = )
mov	ax,$1C[bp]
and	al,#$FE
mov	$1C[bp],ax
!BCC_EOS
! 4143     return;
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 4144 }
! 4145   bx_bool
! Register BX used in function int13_cdemu
! 4146 floppy_media_known(drive)
! 4147   Bit
! 4147 16u drive;
export	_floppy_media_known
_floppy_media_known:
!BCC_EOS
! 4148 {
! 4149   Bit8u val8;
!BCC_EOS
! 4150   Bit16u media_state_offset;
!BCC_EOS
! 4151   val8 = read_byte(0x0040, 0x003e);
push	bp
mov	bp,sp
add	sp,*-4
! Debug: list int = const $3E (used reg = )
mov	ax,*$3E
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () unsigned char = read_byte+0 (used reg = )
call	_read_byte
add	sp,*4
! Debug: eq unsigned char = al+0 to unsigned char val8 = [S+6-3] (used reg = )
mov	-1[bp],al
!BCC_EOS
! 4152   if (drive)
mov	ax,4[bp]
test	ax,ax
je  	.5CE
.5CF:
! 4153     val8 >>= 1;
! Debug: srab int = const 1 to unsigned char val8 = [S+6-3] (used reg = )
mov	al,-1[bp]
xor	ah,ah
shr	ax,*1
mov	-1[bp],al
!BCC_EOS
! 4154   val8 &= 0x01;
.5CE:
! Debug: andab int = const 1 to unsigned char val8 = [S+6-3] (used reg = )
mov	al,-1[bp]
and	al,*1
mov	-1[bp],al
!BCC_EOS
! 4155   if (val8 == 0)
! Debug: logeq int = const 0 to unsigned char val8 = [S+6-3] (used reg = )
mov	al,-1[bp]
test	al,al
jne 	.5D0
.5D1:
! 4156     return(0);
xor	ax,ax
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 4157   media_state_offset = 0x0090;
.5D0:
! Debug: eq int = const $90 to unsigned short media_state_offset = [S+6-6] (used reg = )
mov	ax,#$90
mov	-4[bp],ax
!BCC_EOS
! 4158   if (drive)
mov	ax,4[bp]
test	ax,ax
je  	.5D2
.5D3:
! 4159     media_state_offset += 1;
! Debug: addab int = const 1 to unsigned short media_state_offset = [S+6-6] (used reg = )
mov	ax,-4[bp]
inc	ax
mov	-4[bp],ax
!BCC_EOS
! 4160   val8 = read_byte(0x0040, media_state_offset);
.5D2:
! Debug: list unsigned short media_state_offset = [S+6-6] (used reg = )
push	-4[bp]
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () unsigned char = read_byte+0 (used reg = )
call	_read_byte
add	sp,*4
! Debug: eq unsigned char = al+0 to unsigned char val8 = [S+6-3] (used reg = )
mov	-1[bp],al
!BCC_EOS
! 4161   val8 = (val8 >> 4) & 0x01;
! Debug: sr int = const 4 to unsigned char val8 = [S+6-3] (used reg = )
mov	al,-1[bp]
xor	ah,ah
mov	cl,*4
shr	ax,cl
! Debug: and int = const 1 to unsigned int = ax+0 (used reg = )
and	al,*1
! Debug: eq unsigned char = al+0 to unsigned char val8 = [S+6-3] (used reg = )
mov	-1[bp],al
!BCC_EOS
! 4162   if (val8 == 0)
! Debug: logeq int = const 0 to unsigned char val8 = [S+6-3] (used reg = )
mov	al,-1[bp]
test	al,al
jne 	.5D4
.5D5:
! 4163     return(0);
xor	ax,ax
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 4164   return(1);
.5D4:
mov	ax,*1
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 4165 }
! 4166   bx_bool
! 4167 floppy_media_sense(drive)
! 4168   Bit16u drive;
export	_floppy_media_sense
_floppy_media_sense:
!BCC_EOS
! 4169 {
! 4170   bx_bool retval;
!BCC_EOS
! 4171   Bit16u media_state_offset;
!BCC_EOS
! 4172   Bit8u drive_type, config_data, media_state;
!BCC_EOS
! 4173   if (floppy_drive_recal(drive) == 0) {
push	bp
mov	bp,sp
add	sp,*-8
! Debug: list unsigned short drive = [S+$A+2] (used reg = )
push	4[bp]
! Debug: func () unsigned short = floppy_drive_recal+0 (used reg = )
call	_floppy_drive_recal
inc	sp
inc	sp
! Debug: logeq int = const 0 to unsigned short = ax+0 (used reg = )
test	ax,ax
jne 	.5D6
.5D7:
! 4174     return(0);
xor	ax,ax
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 4175     }
! 4176   drive_type = inb_cmos(0x10);
.5D6:
! Debug: list int = const $10 (used reg = )
mov	ax,*$10
push	ax
! Debug: func () unsigned char = inb_cmos+0 (used reg = )
call	_inb_cmos
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char drive_type = [S+$A-7] (used reg = )
mov	-5[bp],al
!BCC_EOS
! 4177   if (drive == 0)
! Debug: logeq int = const 0 to unsigned short drive = [S+$A+2] (used reg = )
mov	ax,4[bp]
test	ax,ax
jne 	.5D8
.5D9:
! 4178     drive_type >>= 4;
! Debug: srab int = const 4 to unsigned char drive_type = [S+$A-7] (used reg = )
mov	al,-5[bp]
xor	ah,ah
mov	cl,*4
shr	ax,cl
mov	-5[bp],al
!BCC_EOS
! 4179   else
! 4180     drive_type &= 0x0f;
jmp .5DA
.5D8:
! Debug: andab int = const $F to unsigned char drive_type = [S+$A-7] (used reg = )
mov	al,-5[bp]
and	al,*$F
mov	-5[bp],al
!BCC_EOS
! 4181   if ( drive_type == 1 ) {
.5DA:
! Debug: logeq int = const 1 to unsigned char drive_type = [S+$A-7] (used reg = )
mov	al,-5[bp]
cmp	al,*1
jne 	.5DB
.5DC:
! 4182     config_data = 0x00;
! Debug: eq int = const 0 to unsigned char config_data = [S+$A-8] (used reg = )
xor	al,al
mov	-6[bp],al
!BCC_EOS
! 4183     media_state = 0x25;
! Debug: eq int = const $25 to unsigned char media_state = [S+$A-9] (used reg = )
mov	al,*$25
mov	-7[bp],al
!BCC_EOS
! 4184     retval = 1;
! Debug: eq int = const 1 to unsigned short retval = [S+$A-4] (used reg = )
mov	ax,*1
mov	-2[bp],ax
!BCC_EOS
! 4185     }
! 4186   else if ( drive_type == 2 ) {
br 	.5DD
.5DB:
! Debug: logeq int = const 2 to unsigned char drive_type = [S+$A-7] (used reg = )
mov	al,-5[bp]
cmp	al,*2
jne 	.5DE
.5DF:
! 4187     config_data = 0x00;
! Debug: eq int = const 0 to unsigned char config_data = [S+$A-8] (used reg = )
xor	al,al
mov	-6[bp],al
!BCC_EOS
! 4188     media_state = 0x25;
! Debug: eq int = const $25 to unsigned char media_state = [S+$A-9] (used reg = )
mov	al,*$25
mov	-7[bp],al
!BCC_EOS
! 4189     retval = 1;
! Debug: eq int = const 1 to unsigned short retval = [S+$A-4] (used reg = )
mov	ax,*1
mov	-2[bp],ax
!BCC_EOS
! 4190     }
! 4191   else if ( drive_type == 3 ) {
br 	.5E0
.5DE:
! Debug: logeq int = const 3 to unsigned char drive_type = [S+$A-7] (used reg = )
mov	al,-5[bp]
cmp	al,*3
jne 	.5E1
.5E2:
! 4192     config_data = 0x00;
! Debug: eq int = const 0 to unsigned char config_data = [S+$A-8] (used reg = )
xor	al,al
mov	-6[bp],al
!BCC_EOS
! 4193     media_state = 0x17;
! Debug: eq int = const $17 to unsigned char media_state = [S+$A-9] (used reg = )
mov	al,*$17
mov	-7[bp],al
!BCC_EOS
! 4194     retval = 1;
! Debug: eq int = const 1 to unsigned short retval = [S+$A-4] (used reg = )
mov	ax,*1
mov	-2[bp],ax
!BCC_EOS
! 4195     }
! 4196   else if ( drive_type == 4 ) {
br 	.5E3
.5E1:
! Debug: logeq int = const 4 to unsigned char drive_type = [S+$A-7] (used reg = )
mov	al,-5[bp]
cmp	al,*4
bne 	.5E4
.5E5:
! 4197     config_data = 0x00;
! Debug: eq int = const 0 to unsigned char config_data = [S+$A-8] (used reg = )
xor	al,al
mov	-6[bp],al
!BCC_EOS
! 4198     media_state = 0x17;
! Debug: eq int = const $17 to unsigned char media_state = [S+$A-9] (used reg = )
mov	al,*$17
mov	-7[bp],al
!BCC_EOS
! 4199     retval = 1;
! Debug: eq int = const 1 to unsigned short retval = [S+$A-4] (used reg = )
mov	ax,*1
mov	-2[bp],ax
!BCC_EOS
! 4200     }
! 4201   else if ( drive_type == 5 ) {
jmp .5E6
.5E4:
! Debug: logeq int = const 5 to unsigned char drive_type = [S+$A-7] (used reg = )
mov	al,-5[bp]
cmp	al,*5
jne 	.5E7
.5E8:
! 4202     config_data = 0xCC;
! Debug: eq int = const $CC to unsigned char config_data = [S+$A-8] (used reg = )
mov	al,#$CC
mov	-6[bp],al
!BCC_EOS
! 4203     media_state = 0xD7;
! Debug: eq int = const $D7 to unsigned char media_state = [S+$A-9] (used reg = )
mov	al,#$D7
mov	-7[bp],al
!BCC_EOS
! 4204     retval = 1;
! Debug: eq int = const 1 to unsigned short retval = [S+$A-4] (used reg = )
mov	ax,*1
mov	-2[bp],ax
!BCC_EOS
! 4205     }
! 4206   else if ( drive_type == 6 ) {
jmp .5E9
.5E7:
! Debug: logeq int = const 6 to unsigned char drive_type = [S+$A-7] (used reg = )
mov	al,-5[bp]
cmp	al,*6
jne 	.5EA
.5EB:
! 4207     config_data = 0x00;
! Debug: eq int = const 0 to unsigned char config_data = [S+$A-8] (used reg = )
xor	al,al
mov	-6[bp],al
!BCC_EOS
! 4208     media_state = 0x27;
! Debug: eq int = const $27 to unsigned char media_state = [S+$A-9] (used reg = )
mov	al,*$27
mov	-7[bp],al
!BCC_EOS
! 4209     retval = 1;
! Debug: eq int = const 1 to unsigned short retval = [S+$A-4] (used reg = )
mov	ax,*1
mov	-2[bp],ax
!BCC_EOS
! 4210     }
! 4211   else if ( drive_type == 7 ) {
jmp .5EC
.5EA:
! Debug: logeq int = const 7 to unsigned char drive_type = [S+$A-7] (used reg = )
mov	al,-5[bp]
cmp	al,*7
jne 	.5ED
.5EE:
! 4212     config_data = 0x00;
! Debug: eq int = const 0 to unsigned char config_data = [S+$A-8] (used reg = )
xor	al,al
mov	-6[bp],al
!BCC_EOS
! 4213     media_state = 0x27;
! Debug: eq int = const $27 to unsigned char media_state = [S+$A-9] (used reg = )
mov	al,*$27
mov	-7[bp],al
!BCC_EOS
! 4214     retval = 1;
! Debug: eq int = const 1 to unsigned short retval = [S+$A-4] (used reg = )
mov	ax,*1
mov	-2[bp],ax
!BCC_EOS
! 4215     }
! 4216   else if ( drive_type == 8 ) {
jmp .5EF
.5ED:
! Debug: logeq int = const 8 to unsigned char drive_type = [S+$A-7] (used reg = )
mov	al,-5[bp]
cmp	al,*8
jne 	.5F0
.5F1:
! 4217     config_data = 0x00;
! Debug: eq int = const 0 to unsigned char config_data = [S+$A-8] (used reg = )
xor	al,al
mov	-6[bp],al
!BCC_EOS
! 4218     media_state = 0x27;
! Debug: eq int = const $27 to unsigned char media_state = [S+$A-9] (used reg = )
mov	al,*$27
mov	-7[bp],al
!BCC_EOS
! 4219     retval = 1;
! Debug: eq int = const 1 to unsigned short retval = [S+$A-4] (used reg = )
mov	ax,*1
mov	-2[bp],ax
!BCC_EOS
! 4220     }
! 4221   else {
jmp .5F2
.5F0:
! 4222     config_data = 0x00;
! Debug: eq int = const 0 to unsigned char config_data = [S+$A-8] (used reg = )
xor	al,al
mov	-6[bp],al
!BCC_EOS
! 4223     media_state = 0x00;
! Debug: eq int = const 0 to unsigned char media_state = [S+$A-9] (used reg = )
xor	al,al
mov	-7[bp],al
!BCC_EOS
! 4224     retval = 0;
! Debug: eq int = const 0 to unsigned short retval = [S+$A-4] (used reg = )
xor	ax,ax
mov	-2[bp],ax
!BCC_EOS
! 4225     }
! 4226   if (drive == 0)
.5F2:
.5EF:
.5EC:
.5E9:
.5E6:
.5E3:
.5E0:
.5DD:
! Debug: logeq int = const 0 to unsigned short drive = [S+$A+2] (used reg = )
mov	ax,4[bp]
test	ax,ax
jne 	.5F3
.5F4:
! 4227     media_state_offset = 0x90;
! Debug: eq int = const $90 to unsigned short media_state_offset = [S+$A-6] (used reg = )
mov	ax,#$90
mov	-4[bp],ax
!BCC_EOS
! 4228   else
! 4229     media_state_offset = 0x91;
jmp .5F5
.5F3:
! Debug: eq int = const $91 to unsigned short media_state_offset = [S+$A-6] (used reg = )
mov	ax,#$91
mov	-4[bp],ax
!BCC_EOS
! 4230   write_byte(0x0040, 0x008B, config_data);
.5F5:
! Debug: list unsigned char config_data = [S+$A-8] (used reg = )
mov	al,-6[bp]
xor	ah,ah
push	ax
! Debug: list int = const $8B (used reg = )
mov	ax,#$8B
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 4231   write_byte(0x0040, media_state_offset, media_state);
! Debug: list unsigned char media_state = [S+$A-9] (used reg = )
mov	al,-7[bp]
xor	ah,ah
push	ax
! Debug: list unsigned short media_state_offset = [S+$C-6] (used reg = )
push	-4[bp]
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 4232   return(retval);
mov	ax,-2[bp]
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 4233 }
! 4234   bx_bool
! 4235 floppy_drive_recal(drive)
! 4236   Bit16u drive;
export	_floppy_drive_recal
_floppy_drive_recal:
!BCC_EOS
! 4237 {
! 4238   Bit8u val8, dor;
!BCC_EOS
! 4239   Bit16u curr_cyl_offset;
!BCC_EOS
! 4240   val8 = read_byte(0x0000, 0x043e);
push	bp
mov	bp,sp
add	sp,*-4
! Debug: list int = const $43E (used reg = )
mov	ax,#$43E
push	ax
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: func () unsigned char = read_byte+0 (used reg = )
call	_read_byte
add	sp,*4
! Debug: eq unsigned char = al+0 to unsigned char val8 = [S+6-3] (used reg = )
mov	-1[bp],al
!BCC_EOS
! 4241   val8 &= 0x7f;
! Debug: andab int = const $7F to unsigned char val8 = [S+6-3] (used reg = )
mov	al,-1[bp]
and	al,*$7F
mov	-1[bp],al
!BCC_EOS
! 4242   write_byte(0x0000, 0x043e, val8);
! Debug: list unsigned char val8 = [S+6-3] (used reg = )
mov	al,-1[bp]
xor	ah,ah
push	ax
! Debug: list int = const $43E (used reg = )
mov	ax,#$43E
push	ax
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 4243   if (drive)
mov	ax,4[bp]
test	ax,ax
je  	.5F6
.5F7:
! 4244     dor = 0x20;
! Debug: eq int = const $20 to unsigned char dor = [S+6-4] (used reg = )
mov	al,*$20
mov	-2[bp],al
!BCC_EOS
! 4245   else
! 4246     dor = 0x10;
jmp .5F8
.5F6:
! Debug: eq int = const $10 to unsigned char dor = [S+6-4] (used reg = )
mov	al,*$10
mov	-2[bp],al
!BCC_EOS
! 4247   dor |= 0x0c;
.5F8:
! Debug: orab int = const $C to unsigned char dor = [S+6-4] (used reg = )
mov	al,-2[bp]
or	al,*$C
mov	-2[bp],al
!BCC_EOS
! 4248   dor |= drive;
! Debug: orab unsigned short drive = [S+6+2] to unsigned char dor = [S+6-4] (used reg = )
mov	ax,4[bp]
or	al,-2[bp]
mov	-2[bp],al
!BCC_EOS
! 4249   outb(
! 4249 0x03f2, dor);
! Debug: list unsigned char dor = [S+6-4] (used reg = )
mov	al,-2[bp]
xor	ah,ah
push	ax
! Debug: list int = const $3F2 (used reg = )
mov	ax,#$3F2
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 4250   write_byte(0x40,0x40, 37);
! Debug: list int = const $25 (used reg = )
mov	ax,*$25
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 4251   val8 = inb(0x3f4);
! Debug: list int = const $3F4 (used reg = )
mov	ax,#$3F4
push	ax
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char val8 = [S+6-3] (used reg = )
mov	-1[bp],al
!BCC_EOS
! 4252   if ( (val8 & 0xf0) != 0x80 )
! Debug: and int = const $F0 to unsigned char val8 = [S+6-3] (used reg = )
mov	al,-1[bp]
and	al,#$F0
! Debug: ne int = const $80 to unsigned char = al+0 (used reg = )
cmp	al,#$80
je  	.5F9
.5FA:
! 4253     bios_printf((2 | 4 | 1), "floppy recal:f07: ctrl not ready\n");
! Debug: list * char = .5FB+0 (used reg = )
mov	bx,#.5FB
push	bx
! Debug: list int = const 7 (used reg = )
mov	ax,*7
push	ax
! Debug: func () void = bios_printf+0 (used reg = )
call	_bios_printf
add	sp,*4
!BCC_EOS
! 4254   outb(0x03f5, 0x07);
.5F9:
! Debug: list int = const 7 (used reg = )
mov	ax,*7
push	ax
! Debug: list int = const $3F5 (used reg = )
mov	ax,#$3F5
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 4255   outb(0x03f5, drive);
! Debug: list unsigned short drive = [S+6+2] (used reg = )
push	4[bp]
! Debug: list int = const $3F5 (used reg = )
mov	ax,#$3F5
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 4256 #asm
!BCC_EOS
!BCC_ASM
_floppy_drive_recal.dor	set	2
.floppy_drive_recal.dor	set	-2
_floppy_drive_recal.curr_cyl_offset	set	0
.floppy_drive_recal.curr_cyl_offset	set	-4
_floppy_drive_recal.val8	set	3
.floppy_drive_recal.val8	set	-1
_floppy_drive_recal.drive	set	8
.floppy_drive_recal.drive	set	4
  sti
! 4258 endasm
!BCC_ENDASM
!BCC_EOS
! 4259   val8 = (read_byte(0x0000, 0x043e) & 0x80);
! Debug: list int = const $43E (used reg = )
mov	ax,#$43E
push	ax
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: func () unsigned char = read_byte+0 (used reg = )
call	_read_byte
add	sp,*4
! Debug: and int = const $80 to unsigned char = al+0 (used reg = )
and	al,#$80
! Debug: eq unsigned char = al+0 to unsigned char val8 = [S+6-3] (used reg = )
mov	-1[bp],al
!BCC_EOS
! 4260   while ( val8 == 0 ) {
jmp .5FD
.5FE:
! 4261     val8 = (read_byte(0x0000, 0x043e) & 0x80);
! Debug: list int = const $43E (used reg = )
mov	ax,#$43E
push	ax
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: func () unsigned char = read_byte+0 (used reg = )
call	_read_byte
add	sp,*4
! Debug: and int = const $80 to unsigned char = al+0 (used reg = )
and	al,#$80
! Debug: eq unsigned char = al+0 to unsigned char val8 = [S+6-3] (used reg = )
mov	-1[bp],al
!BCC_EOS
! 4262     }
! 4263  val8 = 0;
.5FD:
! Debug: logeq int = const 0 to unsigned char val8 = [S+6-3] (used reg = )
mov	al,-1[bp]
test	al,al
je 	.5FE
.5FF:
.5FC:
! Debug: eq int = const 0 to unsigned char val8 = [S+6-3] (used reg = )
xor	al,al
mov	-1[bp],al
!BCC_EOS
! 4264 #asm
!BCC_EOS
!BCC_ASM
_floppy_drive_recal.dor	set	2
.floppy_drive_recal.dor	set	-2
_floppy_drive_recal.curr_cyl_offset	set	0
.floppy_drive_recal.curr_cyl_offset	set	-4
_floppy_drive_recal.val8	set	3
.floppy_drive_recal.val8	set	-1
_floppy_drive_recal.drive	set	8
.floppy_drive_recal.drive	set	4
  cli
! 4266 endasm
!BCC_ENDASM
!BCC_EOS
! 4267   val8 = read_byte(0x0000, 0x043e);
! Debug: list int = const $43E (used reg = )
mov	ax,#$43E
push	ax
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: func () unsigned char = read_byte+0 (used reg = )
call	_read_byte
add	sp,*4
! Debug: eq unsigned char = al+0 to unsigned char val8 = [S+6-3] (used reg = )
mov	-1[bp],al
!BCC_EOS
! 4268   val8 &= 0x7f;
! Debug: andab int = const $7F to unsigned char val8 = [S+6-3] (used reg = )
mov	al,-1[bp]
and	al,*$7F
mov	-1[bp],al
!BCC_EOS
! 4269   if (drive) {
mov	ax,4[bp]
test	ax,ax
je  	.600
.601:
! 4270     val8 |= 0x02;
! Debug: orab int = const 2 to unsigned char val8 = [S+6-3] (used reg = )
mov	al,-1[bp]
or	al,*2
mov	-1[bp],al
!BCC_EOS
! 4271     curr_cyl_offset = 0x0095;
! Debug: eq int = const $95 to unsigned short curr_cyl_offset = [S+6-6] (used reg = )
mov	ax,#$95
mov	-4[bp],ax
!BCC_EOS
! 4272     }
! 4273   else {
jmp .602
.600:
! 4274     val8 |= 0x01;
! Debug: orab int = const 1 to unsigned char val8 = [S+6-3] (used reg = )
mov	al,-1[bp]
or	al,*1
mov	-1[bp],al
!BCC_EOS
! 4275     curr_cyl_offset = 0x0094;
! Debug: eq int = const $94 to unsigned short curr_cyl_offset = [S+6-6] (used reg = )
mov	ax,#$94
mov	-4[bp],ax
!BCC_EOS
! 4276     }
! 4277   write_byte(0x0040, 0x003e, val8);
.602:
! Debug: list unsigned char val8 = [S+6-3] (used reg = )
mov	al,-1[bp]
xor	ah,ah
push	ax
! Debug: list int = const $3E (used reg = )
mov	ax,*$3E
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 4278   write_byte(0x0040, curr_cyl_offset, 0);
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: list unsigned short curr_cyl_offset = [S+8-6] (used reg = )
push	-4[bp]
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 4279   return(1);
mov	ax,*1
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 4280 }
! 4281   bx_bool
! Register BX used in function floppy_drive_recal
! 4282 floppy_drive_exists(drive)
! 4283   Bit16u drive;
export	_floppy_drive_exists
_floppy_drive_exists:
!BCC_EOS
! 4284 {
! 4285   Bit8u drive_type;
!BCC_EOS
! 4286   drive_type = inb_cmos(0x10);
push	bp
mov	bp,sp
dec	sp
dec	sp
! Debug: list int = const $10 (used reg = )
mov	ax,*$10
push	ax
! Debug: func () unsigned char = inb_cmos+0 (used reg = )
call	_inb_cmos
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char drive_type = [S+4-3] (used reg = )
mov	-1[bp],al
!BCC_EOS
! 4287   if (drive == 0)
! Debug: logeq int = const 0 to unsigned short drive = [S+4+2] (used reg = )
mov	ax,4[bp]
test	ax,ax
jne 	.603
.604:
! 4288     drive_type >>= 4;
! Debug: srab int = const 4 to unsigned char drive_type = [S+4-3] (used reg = )
mov	al,-1[bp]
xor	ah,ah
mov	cl,*4
shr	ax,cl
mov	-1[bp],al
!BCC_EOS
! 4289   else
! 4290     drive_type &= 0x0f;
jmp .605
.603:
! Debug: andab int = const $F to unsigned char drive_type = [S+4-3] (used reg = )
mov	al,-1[bp]
and	al,*$F
mov	-1[bp],al
!BCC_EOS
! 4291   if ( drive_type == 0 )
.605:
! Debug: logeq int = const 0 to unsigned char drive_type = [S+4-3] (used reg = )
mov	al,-1[bp]
test	al,al
jne 	.606
.607:
! 4292     return(0);
xor	ax,ax
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 4293   else
! 4294     return(1);
jmp .608
.606:
mov	ax,*1
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 4295 }
.608:
mov	sp,bp
pop	bp
ret
! 4296   void
! 4297 int13_diskette_function(DS, ES, DI, SI, BP, ELDX, BX, DX, CX, AX, IP, CS, FLAGS)
! 4298   Bit16u DS, ES, DI, SI, BP, ELDX, BX, DX, CX, AX, IP, CS, FLAGS;
export	_int13_diskette_function
_int13_diskette_function:
!BCC_EOS
! 4299 {
! 4300   Bit8u drive, num_sectors, track, sector, head, status;
!BCC_EOS
! 4301   Bit16u base_address, base_count, base_es;
!BCC_EOS
! 4302   Bit8u page, mode_register, val8, dor;
!BCC_EOS
! 4303   Bit8u return_status[7];
!BCC_EOS
! 4304   Bit8u drive_type, num_floppies, ah;
!BCC_EOS
! 4305   Bit16u es, last_addr;
!BCC_EOS
! 4306   ;
push	bp
mov	bp,sp
add	sp,*-$1E
!BCC_EOS
! 4307   ah = ( AX >> 8 );
! Debug: sr int = const 8 to unsigned short AX = [S+$20+$14] (used reg = )
mov	ax,$16[bp]
mov	al,ah
xor	ah,ah
! Debug: eq unsigned int = ax+0 to unsigned char ah = [S+$20-$1C] (used reg = )
mov	-$1A[bp],al
!BCC_EOS
! 4308   switch ( ah ) {
mov	al,-$1A[bp]
br 	.60B
! 4309     case 0x00:
! 4310 ;
.60C:
!BCC_EOS
! 4311       drive = ( ELDX & 0x00ff );
! Debug: and int = const $FF to unsigned short ELDX = [S+$20+$C] (used reg = )
mov	al,$E[bp]
! Debug: eq unsigned char = al+0 to unsigned char drive = [S+$20-3] (used reg = )
mov	-1[bp],al
!BCC_EOS
! 4312       if (drive > 1) {
! Debug: gt int = const 1 to unsigned char drive = [S+$20-3] (used reg = )
mov	al,-1[bp]
cmp	al,*1
jbe 	.60D
.60E:
! 4313         AX = ((AX & 0x00ff) | ((1) << 8));
! Debug: and int = const $FF to unsigned short AX = [S+$20+$14] (used reg = )
mov	al,$16[bp]
! Debug: or int = const $100 to unsigned char = al+0 (used reg = )
xor	ah,ah
or	ax,#$100
! Debug: eq unsigned int = ax+0 to unsigned short AX = [S+$20+$14] (used reg = )
mov	$16[bp],ax
!BCC_EOS
! 4314         set_diskette_ret_status(1);
! Debug: list int = const 1 (used reg = )
mov	ax,*1
push	ax
! Debug: func () void = set_diskette_ret_status+0 (used reg = )
call	_set_diskette_ret_status
inc	sp
inc	sp
!BCC_EOS
! 4315         FLAGS |= 0x0001;
! Debug: orab int = const 1 to unsigned short FLAGS = [S+$20+$1A] (used reg = )
mov	ax,$1C[bp]
or	al,*1
mov	$1C[bp],ax
!BCC_EOS
! 4316         return;
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 4317         }
! 4318       drive_type = inb_cmos(0x10);
.60D:
! Debug: list int = const $10 (used reg = )
mov	ax,*$10
push	ax
! Debug: func () unsigned char = inb_cmos+0 (used reg = )
call	_inb_cmos
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char drive_type = [S+$20-$1A] (used reg = )
mov	-$18[bp],al
!BCC_EOS
! 4319       if (drive == 0)
! Debug: logeq int = const 0 to unsigned char drive = [S+$20-3] (used reg = )
mov	al,-1[bp]
test	al,al
jne 	.60F
.610:
! 4320         drive_type >>= 4;
! Debug: srab int = const 4 to unsigned char drive_type = [S+$20-$1A] (used reg = )
mov	al,-$18[bp]
xor	ah,ah
mov	cl,*4
shr	ax,cl
mov	-$18[bp],al
!BCC_EOS
! 4321       else
! 4322         drive_type &= 0x0f;
jmp .611
.60F:
! Debug: andab int = const $F to unsigned char drive_type = [S+$20-$1A] (used reg = )
mov	al,-$18[bp]
and	al,*$F
mov	-$18[bp],al
!BCC_EOS
! 4323       if (drive_type == 0) {
.611:
! Debug: logeq int = const 0 to unsigned char drive_type = [S+$20-$1A] (used reg = )
mov	al,-$18[bp]
test	al,al
jne 	.612
.613:
! 4324         AX = ((AX & 0x00ff) | ((0x80) << 8));
! Debug: and int = const $FF to unsigned short AX = [S+$20+$14] (used reg = )
mov	al,$16[bp]
! Debug: or int = const -$8000 to unsigned char = al+0 (used reg = )
xor	ah,ah
or	ax,#-$8000
! Debug: eq unsigned int = ax+0 to unsigned short AX = [S+$20+$14] (used reg = )
mov	$16[bp],ax
!BCC_EOS
! 4325         set_diskette_ret_status(0x80);
! Debug: list int = const $80 (used reg = )
mov	ax,#$80
push	ax
! Debug: func () void = set_diskette_ret_status+0 (used reg = )
call	_set_diskette_ret_status
inc	sp
inc	sp
!BCC_EOS
! 4326         FLAGS |= 0x0001;
! Debug: orab int = const 1 to unsigned short FLAGS = [S+$20+$1A] (used reg = )
mov	ax,$1C[bp]
or	al,*1
mov	$1C[bp],ax
!BCC_EOS
! 4327         return;
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 4328         }
! 4329       AX = ((AX & 0x00ff) | ((0) << 8));
.612:
! Debug: and int = const $FF to unsigned short AX = [S+$20+$14] (used reg = )
mov	al,$16[bp]
! Debug: or int = const 0 to unsigned char = al+0 (used reg = )
or	al,*0
! Debug: eq unsigned char = al+0 to unsigned short AX = [S+$20+$14] (used reg = )
xor	ah,ah
mov	$16[bp],ax
!BCC_EOS
! 4330       set_diskette_ret_status(0);
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: func () void = set_diskette_ret_status+0 (used reg = )
call	_set_diskette_ret_status
inc	sp
inc	sp
!BCC_EOS
! 4331       FLAGS &= 0xfffe;
! Debug: andab unsigned int = const $FFFE to unsigned short FLAGS = [S+$20+$1A] (used reg = )
mov	ax,$1C[bp]
and	al,#$FE
mov	$1C[bp],ax
!BCC_EOS
! 4332       set_diskette_current_cyl(drive, 0);
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: list unsigned char drive = [S+$22-3] (used reg = )
mov	al,-1[bp]
xor	ah,ah
push	ax
! Debug: func () void = set_diskette_current_cyl+0 (used reg = )
call	_set_diskette_current_cyl
add	sp,*4
!BCC_EOS
! 4333       return;
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 4334     case 0x01:
! 4335       FLAGS &= 0xfffe;
.614:
! Debug: andab unsigned int = const $FFFE to unsigned short FLAGS = [S+$20+$1A] (used reg = )
mov	ax,$1C[bp]
and	al,#$FE
mov	$1C[bp],ax
!BCC_EOS
! 4336       val8 = read_byte(0x0000, 0x0441);
! Debug: list int = const $441 (used reg = )
mov	ax,#$441
push	ax
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: func () unsigned char = read_byte+0 (used reg = )
call	_read_byte
add	sp,*4
! Debug: eq unsigned char = al+0 to unsigned char val8 = [S+$20-$11] (used reg = )
mov	-$F[bp],al
!BCC_EOS
! 4337   
! 4337     AX = ((AX & 0x00ff) | ((val8) << 8));
! Debug: sl int = const 8 to unsigned char val8 = [S+$20-$11] (used reg = )
mov	al,-$F[bp]
xor	ah,ah
mov	ah,al
xor	al,al
push	ax
! Debug: and int = const $FF to unsigned short AX = [S+$22+$14] (used reg = )
mov	al,$16[bp]
! Debug: or unsigned int (temp) = [S+$22-$22] to unsigned char = al+0 (used reg = )
xor	ah,ah
or	ax,0+..FFDB[bp]
inc	sp
inc	sp
! Debug: eq unsigned int = ax+0 to unsigned short AX = [S+$20+$14] (used reg = )
mov	$16[bp],ax
!BCC_EOS
! 4338       if (val8) {
mov	al,-$F[bp]
test	al,al
je  	.615
.616:
! 4339         FLAGS |= 0x0001;
! Debug: orab int = const 1 to unsigned short FLAGS = [S+$20+$1A] (used reg = )
mov	ax,$1C[bp]
or	al,*1
mov	$1C[bp],ax
!BCC_EOS
! 4340         }
! 4341       return;
.615:
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 4342     case 0x02:
! 4343     case 0x03:
.617:
! 4344     case 0x04:
.618:
! 4345       num_sectors = ( AX & 0x00ff );
.619:
! Debug: and int = const $FF to unsigned short AX = [S+$20+$14] (used reg = )
mov	al,$16[bp]
! Debug: eq unsigned char = al+0 to unsigned char num_sectors = [S+$20-4] (used reg = )
mov	-2[bp],al
!BCC_EOS
! 4346       track = ( CX >> 8 );
! Debug: sr int = const 8 to unsigned short CX = [S+$20+$12] (used reg = )
mov	ax,$14[bp]
mov	al,ah
xor	ah,ah
! Debug: eq unsigned int = ax+0 to unsigned char track = [S+$20-5] (used reg = )
mov	-3[bp],al
!BCC_EOS
! 4347       sector = ( CX & 0x00ff );
! Debug: and int = const $FF to unsigned short CX = [S+$20+$12] (used reg = )
mov	al,$14[bp]
! Debug: eq unsigned char = al+0 to unsigned char sector = [S+$20-6] (used reg = )
mov	-4[bp],al
!BCC_EOS
! 4348       head = ( DX >> 8 );
! Debug: sr int = const 8 to unsigned short DX = [S+$20+$10] (used reg = )
mov	ax,$12[bp]
mov	al,ah
xor	ah,ah
! Debug: eq unsigned int = ax+0 to unsigned char head = [S+$20-7] (used reg = )
mov	-5[bp],al
!BCC_EOS
! 4349       drive = ( ELDX & 0x00ff );
! Debug: and int = const $FF to unsigned short ELDX = [S+$20+$C] (used reg = )
mov	al,$E[bp]
! Debug: eq unsigned char = al+0 to unsigned char drive = [S+$20-3] (used reg = )
mov	-1[bp],al
!BCC_EOS
! 4350       if ( (drive > 1) || (head > 1) ||
! 4351            (num_sectors == 0) || (num_sectors > 72) ) {
! Debug: gt int = const 1 to unsigned char drive = [S+$20-3] (used reg = )
mov	al,-1[bp]
cmp	al,*1
ja  	.61B
.61E:
! Debug: gt int = const 1 to unsigned char head = [S+$20-7] (used reg = )
mov	al,-5[bp]
cmp	al,*1
ja  	.61B
.61D:
! Debug: logeq int = const 0 to unsigned char num_sectors = [S+$20-4] (used reg = )
mov	al,-2[bp]
test	al,al
je  	.61B
.61C:
! Debug: gt int = const $48 to unsigned char num_sectors = [S+$20-4] (used reg = )
mov	al,-2[bp]
cmp	al,*$48
jbe 	.61A
.61B:
! 4352 bios_printf(4, "floppy: drive>1 || head>1 ...\n");
! Debug: list * char = .61F+0 (used reg = )
mov	bx,#.61F
push	bx
! Debug: list int = const 4 (used reg = )
mov	ax,*4
push	ax
! Debug: func () void = bios_printf+0 (used reg = )
call	_bios_printf
add	sp,*4
!BCC_EOS
! 4353         AX = ((AX & 0x00ff) | ((1) << 8));
! Debug: and int = const $FF to unsigned short AX = [S+$20+$14] (used reg = )
mov	al,$16[bp]
! Debug: or int = const $100 to unsigned char = al+0 (used reg = )
xor	ah,ah
or	ax,#$100
! Debug: eq unsigned int = ax+0 to unsigned short AX = [S+$20+$14] (used reg = )
mov	$16[bp],ax
!BCC_EOS
! 4354         set_diskette_ret_status(1);
! Debug: list int = const 1 (used reg = )
mov	ax,*1
push	ax
! Debug: func () void = set_diskette_ret_status+0 (used reg = )
call	_set_diskette_ret_status
inc	sp
inc	sp
!BCC_EOS
! 4355         AX = ((AX & 0xff00) | (0));
! Debug: and unsigned int = const $FF00 to unsigned short AX = [S+$20+$14] (used reg = )
mov	ax,$16[bp]
xor	al,al
! Debug: or int = const 0 to unsigned int = ax+0 (used reg = )
or	al,*0
! Debug: eq unsigned int = ax+0 to unsigned short AX = [S+$20+$14] (used reg = )
mov	$16[bp],ax
!BCC_EOS
! 4356         FLAGS |= 0x0001;
! Debug: orab int = const 1 to unsigned short FLAGS = [S+$20+$1A] (used reg = )
mov	ax,$1C[bp]
or	al,*1
mov	$1C[bp],ax
!BCC_EOS
! 4357         return;
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 4358         }
! 4359       if (floppy_drive_exists(drive) == 0) {
.61A:
! Debug: list unsigned char drive = [S+$20-3] (used reg = )
mov	al,-1[bp]
xor	ah,ah
push	ax
! Debug: func () unsigned short = floppy_drive_exists+0 (used reg = )
call	_floppy_drive_exists
inc	sp
inc	sp
! Debug: logeq int = const 0 to unsigned short = ax+0 (used reg = )
test	ax,ax
jne 	.620
.621:
! 4360         AX = ((AX & 0x00ff) | ((0x80) << 8));
! Debug: and int = const $FF to unsigned short AX = [S+$20+$14] (used reg = )
mov	al,$16[bp]
! Debug: or int = const -$8000 to unsigned char = al+0 (used reg = )
xor	ah,ah
or	ax,#-$8000
! Debug: eq unsigned int = ax+0 to unsigned short AX = [S+$20+$14] (used reg = )
mov	$16[bp],ax
!BCC_EOS
! 4361         set_diskette_ret_status(0x80);
! Debug: list int = const $80 (used reg = )
mov	ax,#$80
push	ax
! Debug: func () void = set_diskette_ret_status+0 (used reg = )
call	_set_diskette_ret_status
inc	sp
inc	sp
!BCC_EOS
! 4362         AX = ((AX & 0xff00) | (0));
! Debug: and unsigned int = const $FF00 to unsigned short AX = [S+$20+$14] (used reg = )
mov	ax,$16[bp]
xor	al,al
! Debug: or int = const 0 to unsigned int = ax+0 (used reg = )
or	al,*0
! Debug: eq unsigned int = ax+0 to unsigned short AX = [S+$20+$14] (used reg = )
mov	$16[bp],ax
!BCC_EOS
! 4363         FLAGS |= 0x0001;
! Debug: orab int = const 1 to unsigned short FLAGS = [S+$20+$1A] (used reg = )
mov	ax,$1C[bp]
or	al,*1
mov	$1C[bp],ax
!BCC_EOS
! 4364         return;
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 4365         }
! 4366       if (floppy_media_known(drive) == 0) {
.620:
! Debug: list unsigned char drive = [S+$20-3] (used reg = )
mov	al,-1[bp]
xor	ah,ah
push	ax
! Debug: func () unsigned short = floppy_media_known+0 (used reg = )
call	_floppy_media_known
inc	sp
inc	sp
! Debug: logeq int = const 0 to unsigned short = ax+0 (used reg = )
test	ax,ax
jne 	.622
.623:
! 4367         if (floppy_media_sense(drive) == 0) {
! Debug: list unsigned char drive = [S+$20-3] (used reg = )
mov	al,-1[bp]
xor	ah,ah
push	ax
! Debug: func () unsigned short = floppy_media_sense+0 (used reg = )
call	_floppy_media_sense
inc	sp
inc	sp
! Debug: logeq int = const 0 to unsigned short = ax+0 (used reg = )
test	ax,ax
jne 	.624
.625:
! 4368           AX = ((AX & 0x00ff) | ((0x0C) << 8));
! Debug: and int = const $FF to unsigned short AX = [S+$20+$14] (used reg = )
mov	al,$16[bp]
! Debug: or int = const $C00 to unsigned char = al+0 (used reg = )
xor	ah,ah
or	ax,#$C00
! Debug: eq unsigned int = ax+0 to unsigned short AX = [S+$20+$14] (used reg = )
mov	$16[bp],ax
!BCC_EOS
! 4369           set_diskette_ret_status(0x0C);
! Debug: list int = const $C (used reg = )
mov	ax,*$C
push	ax
! Debug: func () void = set_diskette_ret_status+0 (used reg = )
call	_set_diskette_ret_status
inc	sp
inc	sp
!BCC_EOS
! 4370           AX = ((AX & 0xff00) | (0));
! Debug: and unsigned int = const $FF00 to unsigned short AX = [S+$20+$14] (used reg = )
mov	ax,$16[bp]
xor	al,al
! Debug: or int = const 0 to unsigned int = ax+0 (used reg = )
or	al,*0
! Debug: eq unsigned int = ax+0 to unsigned short AX = [S+$20+$14] (used reg = )
mov	$16[bp],ax
!BCC_EOS
! 4371           FLAGS |= 0x0001;
! Debug: orab int = const 1 to unsigned short FLAGS = [S+$20+$1A] (used reg = )
mov	ax,$1C[bp]
or	al,*1
mov	$1C[bp],ax
!BCC_EOS
! 4372           return;
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 4373           }
! 4374         }
.624:
! 4375       if (ah == 0x02) {
.622:
! Debug: logeq int = const 2 to unsigned char ah = [S+$20-$1C] (used reg = )
mov	al,-$1A[bp]
cmp	al,*2
bne 	.626
.627:
! 4376         page = (ES >> 12);
! Debug: sr int = const $C to unsigned short ES = [S+$20+4] (used reg = )
mov	ax,6[bp]
mov	al,ah
xor	ah,ah
mov	cl,*4
shr	ax,cl
! Debug: eq unsigned int = ax+0 to unsigned char page = [S+$20-$F] (used reg = )
mov	-$D[bp],al
!BCC_EOS
! 4377         base_es = (ES << 4);
! Debug: sl int = const 4 to unsigned short ES = [S+$20+4] (used reg = )
mov	ax,6[bp]
mov	cl,*4
shl	ax,cl
! Debug: eq unsigned int = ax+0 to unsigned short base_es = [S+$20-$E] (used reg = )
mov	-$C[bp],ax
!BCC_EOS
! 4378         base_address = base_es + BX;
! Debug: add unsigned short BX = [S+$20+$E] to unsigned short base_es = [S+$20-$E] (used reg = )
mov	ax,-$C[bp]
add	ax,$10[bp]
! Debug: eq unsigned int = ax+0 to unsigned short base_address = [S+$20-$A] (used reg = )
mov	-8[bp],ax
!BCC_EOS
! 4379         if ( base_address < base_es ) {
! Debug: lt unsigned short base_es = [S+$20-$E] to unsigned short base_address = [S+$20-$A] (used reg = )
mov	ax,-8[bp]
cmp	ax,-$C[bp]
jae 	.628
.629:
! 4380           page++;
! Debug: postinc unsigned char page = [S+$20-$F] (used reg = )
mov	al,-$D[bp]
inc	ax
mov	-$D[bp],al
!BCC_EOS
! 4381           }
! 4382         base_count = (num_sectors * 512) - 1;
.628:
! Debug: mul int = const $200 to unsigned char num_sectors = [S+$20-4] (used reg = )
mov	al,-2[bp]
xor	ah,ah
mov	cx,#$200
imul	cx
! Debug: sub int = const 1 to unsigned int = ax+0 (used reg = )
! Debug: eq unsigned int = ax-1 to unsigned short base_count = [S+$20-$C] (used reg = )
dec	ax
mov	-$A[bp],ax
!BCC_EOS
! 4383         last_addr = base_address + base_count;
! Debug: add unsigned short base_count = [S+$20-$C] to unsigned short base_address = [S+$20-$A] (used reg = )
mov	ax,-8[bp]
add	ax,-$A[bp]
! Debug: eq unsigned int = ax+0 to unsigned short last_addr = [S+$20-$20] (used reg = )
mov	-$1E[bp],ax
!BCC_EOS
! 4384         if (last_addr < base_address) {
! Debug: lt unsigned short base_address = [S+$20-$A] to unsigned short last_addr = [S+$20-$20] (used reg = )
mov	ax,-$1E[bp]
cmp	ax,-8[bp]
jae 	.62A
.62B:
! 4385           AX = ((AX & 0x00ff) | ((0x09) << 8));
! Debug: and int = const $FF to unsigned short AX = [S+$20+$14] (used reg = )
mov	al,$16[bp]
! Debug: or int = const $900 to unsigned char = al+0 (used reg = )
xor	ah,ah
or	ax,#$900
! Debug: eq unsigned int = ax+0 to unsigned short AX = [S+$20+$14] (used reg = )
mov	$16[bp],ax
!BCC_EOS
! 4386           set_diskette_ret_status(0x09);
! Debug: list int = const 9 (used reg = )
mov	ax,*9
push	ax
! Debug: func () void = set_diskette_ret_status+0 (used reg = )
call	_set_diskette_ret_status
inc	sp
inc	sp
!BCC_EOS
! 4387           AX = ((AX & 0xff00) | (0));
! Debug: and unsigned int = const $FF00 to unsigned short AX = [S+$20+$14] (used reg = )
mov	ax,$16[bp]
xor	al,al
! Debug: or int = const 0 to unsigned int = ax+0 (used reg = )
or	al,*0
! Debug: eq unsigned int = ax+0 to unsigned short AX = [S+$20+$14] (used reg = )
mov	$16[bp],ax
!BCC_EOS
! 4388           FLAGS |= 0x0001;
! Debug: orab int = const 1 to unsigned short FLAGS = [S+$20+$1A] (used reg = )
mov	ax,$1C[bp]
or	al,*1
mov	$1C[bp],ax
!BCC_EOS
! 4389           return;
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 4390           }
! 4391         ;
.62A:
!BCC_EOS
! 4392         outb(0x000a, 0x06);
! Debug: list int = const 6 (used reg = )
mov	ax,*6
push	ax
! Debug: list int = const $A (used reg = )
mov	ax,*$A
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 4393   ;
!BCC_EOS
! 4394         outb(0x000c, 0x00);
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: list int = const $C (used reg = )
mov	ax,*$C
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 4395         outb(0x0004, base_address);
! Debug: list unsigned short base_address = [S+$20-$A] (used reg = )
push	-8[bp]
! Debug: list int = const 4 (used reg = )
mov	ax,*4
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 4396         outb(0x0004, base_address>>8);
! Debug: sr int = const 8 to unsigned short base_address = [S+$20-$A] (used reg = )
mov	ax,-8[bp]
mov	al,ah
xor	ah,ah
! Debug: list unsigned int = ax+0 (used reg = )
push	ax
! Debug: list int = const 4 (used reg = )
mov	ax,*4
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 4397   ;
!BCC_EOS
! 4398         outb(0x000c, 0x00);
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: list int = const $C (used reg = )
mov	ax,*$C
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 4399         outb(0x0005, base_count);
! Debug: list unsigned short base_count = [S+$20-$C] (used reg = )
push	-$A[bp]
! Debug: list int = const 5 (used reg = )
mov	ax,*5
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 4400         outb(0x0005, base_count>>8);
! Debug: sr int = const 8 to unsigned short base_count = [S+$20-$C] (used reg = )
mov	ax,-$A[bp]
mov	al,ah
xor	ah,ah
! Debug: list unsigned int = ax+0 (used reg = )
push	ax
! Debug: list int = const 5 (used reg = )
mov	ax,*5
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 4401         mode_register = 0x46;
! Debug: eq int = const $46 to unsigned char mode_register = [S+$20-$10] (used reg = )
mov	al,*$46
mov	-$E[bp],al
!BCC_EOS
! 4402   ;
!BCC_EOS
! 4403         outb(0x000b, mode_register);
! Debug: list unsigned char mode_register = [S+$20-$10] (used reg = )
mov	al,-$E[bp]
xor	ah,ah
push	ax
! Debug: list int = const $B (used reg = )
mov	ax,*$B
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 4404   ;
!BCC_EOS
! 4405         outb(0x0081, page);
! Debug: list unsigned char page = [S+$20-$F] (used reg = )
mov	al,-$D[bp]
xor	ah,ah
push	ax
! Debug: list int = const $81 (used reg = )
mov	ax,#$81
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 4406   ;
!BCC_EOS
! 4407         outb(0x000a, 0x02);
! Debug: list int = const 2 (used reg = )
mov	ax,*2
push	ax
! Debug: list int = const $A (used reg = )
mov	ax,*$A
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 4408         ;
!BCC_EOS
! 4409         outb(0x000a, 0x02);
! Debug: list int = const 2 (used reg = )
mov	ax,*2
push	ax
! Debug: list int = const $A (used reg = )
mov	ax,*$A
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 4410 
! 4410         val8 = read_byte(0x0000, 0x043e);
! Debug: list int = const $43E (used reg = )
mov	ax,#$43E
push	ax
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: func () unsigned char = read_byte+0 (used reg = )
call	_read_byte
add	sp,*4
! Debug: eq unsigned char = al+0 to unsigned char val8 = [S+$20-$11] (used reg = )
mov	-$F[bp],al
!BCC_EOS
! 4411         val8 &= 0x7f;
! Debug: andab int = const $7F to unsigned char val8 = [S+$20-$11] (used reg = )
mov	al,-$F[bp]
and	al,*$7F
mov	-$F[bp],al
!BCC_EOS
! 4412         write_byte(0x0000, 0x043e, val8);
! Debug: list unsigned char val8 = [S+$20-$11] (used reg = )
mov	al,-$F[bp]
xor	ah,ah
push	ax
! Debug: list int = const $43E (used reg = )
mov	ax,#$43E
push	ax
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 4413         if (drive)
mov	al,-1[bp]
test	al,al
je  	.62C
.62D:
! 4414           dor = 0x20;
! Debug: eq int = const $20 to unsigned char dor = [S+$20-$12] (used reg = )
mov	al,*$20
mov	-$10[bp],al
!BCC_EOS
! 4415         else
! 4416           dor = 0x10;
jmp .62E
.62C:
! Debug: eq int = const $10 to unsigned char dor = [S+$20-$12] (used reg = )
mov	al,*$10
mov	-$10[bp],al
!BCC_EOS
! 4417         dor |= 0x0c;
.62E:
! Debug: orab int = const $C to unsigned char dor = [S+$20-$12] (used reg = )
mov	al,-$10[bp]
or	al,*$C
mov	-$10[bp],al
!BCC_EOS
! 4418         dor |= drive;
! Debug: orab unsigned char drive = [S+$20-3] to unsigned char dor = [S+$20-$12] (used reg = )
mov	al,-$10[bp]
or	al,-1[bp]
mov	-$10[bp],al
!BCC_EOS
! 4419         outb(0x03f2, dor);
! Debug: list unsigned char dor = [S+$20-$12] (used reg = )
mov	al,-$10[bp]
xor	ah,ah
push	ax
! Debug: list int = const $3F2 (used reg = )
mov	ax,#$3F2
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 4420         write_byte(0x40,0x40, 37);
! Debug: list int = const $25 (used reg = )
mov	ax,*$25
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 4421         val8 = inb(0x3f4);
! Debug: list int = const $3F4 (used reg = )
mov	ax,#$3F4
push	ax
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char val8 = [S+$20-$11] (used reg = )
mov	-$F[bp],al
!BCC_EOS
! 4422         if ( (val8 & 0xf0) != 0x80 )
! Debug: and int = const $F0 to unsigned char val8 = [S+$20-$11] (used reg = )
mov	al,-$F[bp]
and	al,#$F0
! Debug: ne int = const $80 to unsigned char = al+0 (used reg = )
cmp	al,#$80
je  	.62F
.630:
! 4423           bios_printf((2 | 4 | 1), "int13_diskette:f02: ctrl not ready\n");
! Debug: list * char = .631+0 (used reg = )
mov	bx,#.631
push	bx
! Debug: list int = const 7 (used reg = )
mov	ax,*7
push	ax
! Debug: func () void = bios_printf+0 (used reg = )
call	_bios_printf
add	sp,*4
!BCC_EOS
! 4424         outb(0x03f5, 0xe6);
.62F:
! Debug: list int = const $E6 (used reg = )
mov	ax,#$E6
push	ax
! Debug: list int = const $3F5 (used reg = )
mov	ax,#$3F5
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 4425         outb(0x03f5, (head << 2) | drive);
! Debug: sl int = const 2 to unsigned char head = [S+$20-7] (used reg = )
mov	al,-5[bp]
xor	ah,ah
shl	ax,*1
shl	ax,*1
! Debug: or unsigned char drive = [S+$20-3] to unsigned int = ax+0 (used reg = )
or	al,-1[bp]
! Debug: list unsigned int = ax+0 (used reg = )
push	ax
! Debug: list int = const $3F5 (used reg = )
mov	ax,#$3F5
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 4426         outb(0x03f5, track);
! Debug: list unsigned char track = [S+$20-5] (used reg = )
mov	al,-3[bp]
xor	ah,ah
push	ax
! Debug: list int = const $3F5 (used reg = )
mov	ax,#$3F5
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 4427         outb(0x03f5, head);
! Debug: list unsigned char head = [S+$20-7] (used reg = )
mov	al,-5[bp]
xor	ah,ah
push	ax
! Debug: list int = const $3F5 (used reg = )
mov	ax,#$3F5
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 4428         outb(0x03f5, sector);
! Debug: list unsigned char sector = [S+$20-6] (used reg = )
mov	al,-4[bp]
xor	ah,ah
push	ax
! Debug: list int = const $3F5 (used reg = )
mov	ax,#$3F5
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 4429         outb(0x03f5, 2);
! Debug: list int = const 2 (used reg = )
mov	ax,*2
push	ax
! Debug: list int = const $3F5 (used reg = )
mov	ax,#$3F5
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 4430         outb(0x03f5, sector + num_sectors - 1);
! Debug: add unsigned char num_sectors = [S+$20-4] to unsigned char sector = [S+$20-6] (used reg = )
mov	al,-4[bp]
xor	ah,ah
add	al,-2[bp]
adc	ah,*0
! Debug: sub int = const 1 to unsigned int = ax+0 (used reg = )
! Debug: list unsigned int = ax-1 (used reg = )
dec	ax
push	ax
! Debug: list int = const $3F5 (used reg = )
mov	ax,#$3F5
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 4431         outb(0x03f5, 0);
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: list int = const $3F5 (used reg = )
mov	ax,#$3F5
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 4432         outb(0x03f5, 0xff);
! Debug: list int = const $FF (used reg = )
mov	ax,#$FF
push	ax
! Debug: list int = const $3F5 (used reg = )
mov	ax,#$3F5
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 4433 #asm
!BCC_EOS
!BCC_ASM
_int13_diskette_function.BP	set	$2A
.int13_diskette_function.BP	set	$C
_int13_diskette_function.CS	set	$38
.int13_diskette_function.CS	set	$1A
_int13_diskette_function.CX	set	$32
.int13_diskette_function.CX	set	$14
_int13_diskette_function.base_address	set	$16
.int13_diskette_function.base_address	set	-8
_int13_diskette_function.DI	set	$26
.int13_diskette_function.DI	set	8
_int13_diskette_function.FLAGS	set	$3A
.int13_diskette_function.FLAGS	set	$1C
_int13_diskette_function.base_count	set	$14
.int13_diskette_function.base_count	set	-$A
_int13_diskette_function.sector	set	$1A
.int13_diskette_function.sector	set	-4
_int13_diskette_function.DS	set	$22
.int13_diskette_function.DS	set	4
_int13_diskette_function.head	set	$19
.int13_diskette_function.head	set	-5
_int13_diskette_function.ELDX	set	$2C
.int13_diskette_function.ELDX	set	$E
_int13_diskette_function.dor	set	$E
.int13_diskette_function.dor	set	-$10
_int13_diskette_function.DX	set	$30
.int13_diskette_function.DX	set	$12
_int13_diskette_function.return_status	set	7
.int13_diskette_function.return_status	set	-$17
_int13_diskette_function.es	set	2
.int13_diskette_function.es	set	-$1C
_int13_diskette_function.mode_register	set	$10
.int13_diskette_function.mode_register	set	-$E
_int13_diskette_function.ES	set	$24
.int13_diskette_function.ES	set	6
_int13_diskette_function.base_es	set	$12
.int13_diskette_function.base_es	set	-$C
_int13_diskette_function.track	set	$1B
.int13_diskette_function.track	set	-3
_int13_diskette_function.SI	set	$28
.int13_diskette_function.SI	set	$A
_int13_diskette_function.drive_type	set	6
.int13_diskette_function.drive_type	set	-$18
_int13_diskette_function.num_sectors	set	$1C
.int13_diskette_function.num_sectors	set	-2
_int13_diskette_function.IP	set	$36
.int13_diskette_function.IP	set	$18
_int13_diskette_function.status	set	$18
.int13_diskette_function.status	set	-6
_int13_diskette_function.AX	set	$34
.int13_diskette_function.AX	set	$16
_int13_diskette_function.val8	set	$F
.int13_diskette_function.val8	set	-$F
_int13_diskette_function.last_addr	set	0
.int13_diskette_function.last_addr	set	-$1E
_int13_diskette_function.page	set	$11
.int13_diskette_function.page	set	-$D
_int13_diskette_function.ah	set	4
.int13_diskette_function.ah	set	-$1A
_int13_diskette_function.drive	set	$1D
.int13_diskette_function.drive	set	-1
_int13_diskette_function.num_floppies	set	5
.int13_diskette_function.num_floppies	set	-$19
_int13_diskette_function.BX	set	$2E
.int13_diskette_function.BX	set	$10
        sti
! 4435 endasm
!BCC_ENDASM
!BCC_EOS
! 4436         val8 = (read_byte(0x0000, 0x043e) & 0x80);
! Debug: list int = const $43E (used reg = )
mov	ax,#$43E
push	ax
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: func () unsigned char = read_byte+0 (used reg = )
call	_read_byte
add	sp,*4
! Debug: and int = const $80 to unsigned char = al+0 (used reg = )
and	al,#$80
! Debug: eq unsigned char = al+0 to unsigned char val8 = [S+$20-$11] (used reg = )
mov	-$F[bp],al
!BCC_EOS
! 4437         while ( val8 == 0 ) {
jmp .633
.634:
! 4438           val8 = (read_byte(0x0000, 0x043e) & 0x80);
! Debug: list int = const $43E (used reg = )
mov	ax,#$43E
push	ax
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: func () unsigned char = read_byte+0 (used reg = )
call	_read_byte
add	sp,*4
! Debug: and int = const $80 to unsigned char = al+0 (used reg = )
and	al,#$80
! Debug: eq unsigned char = al+0 to unsigned char val8 = [S+$20-$11] (used reg = )
mov	-$F[bp],al
!BCC_EOS
! 4439           }
! 4440        val8 = 0;
.633:
! Debug: logeq int = const 0 to unsigned char val8 = [S+$20-$11] (used reg = )
mov	al,-$F[bp]
test	al,al
je 	.634
.635:
.632:
! Debug: eq int = const 0 to unsigned char val8 = [S+$20-$11] (used reg = )
xor	al,al
mov	-$F[bp],al
!BCC_EOS
! 4441 #asm
!BCC_EOS
!BCC_ASM
_int13_diskette_function.BP	set	$2A
.int13_diskette_function.BP	set	$C
_int13_diskette_function.CS	set	$38
.int13_diskette_function.CS	set	$1A
_int13_diskette_function.CX	set	$32
.int13_diskette_function.CX	set	$14
_int13_diskette_function.base_address	set	$16
.int13_diskette_function.base_address	set	-8
_int13_diskette_function.DI	set	$26
.int13_diskette_function.DI	set	8
_int13_diskette_function.FLAGS	set	$3A
.int13_diskette_function.FLAGS	set	$1C
_int13_diskette_function.base_count	set	$14
.int13_diskette_function.base_count	set	-$A
_int13_diskette_function.sector	set	$1A
.int13_diskette_function.sector	set	-4
_int13_diskette_function.DS	set	$22
.int13_diskette_function.DS	set	4
_int13_diskette_function.head	set	$19
.int13_diskette_function.head	set	-5
_int13_diskette_function.ELDX	set	$2C
.int13_diskette_function.ELDX	set	$E
_int13_diskette_function.dor	set	$E
.int13_diskette_function.dor	set	-$10
_int13_diskette_function.DX	set	$30
.int13_diskette_function.DX	set	$12
_int13_diskette_function.return_status	set	7
.int13_diskette_function.return_status	set	-$17
_int13_diskette_function.es	set	2
.int13_diskette_function.es	set	-$1C
_int13_diskette_function.mode_register	set	$10
.int13_diskette_function.mode_register	set	-$E
_int13_diskette_function.ES	set	$24
.int13_diskette_function.ES	set	6
_int13_diskette_function.base_es	set	$12
.int13_diskette_function.base_es	set	-$C
_int13_diskette_function.track	set	$1B
.int13_diskette_function.track	set	-3
_int13_diskette_function.SI	set	$28
.int13_diskette_function.SI	set	$A
_int13_diskette_function.drive_type	set	6
.int13_diskette_function.drive_type	set	-$18
_int13_diskette_function.num_sectors	set	$1C
.int13_diskette_function.num_sectors	set	-2
_int13_diskette_function.IP	set	$36
.int13_diskette_function.IP	set	$18
_int13_diskette_function.status	set	$18
.int13_diskette_function.status	set	-6
_int13_diskette_function.AX	set	$34
.int13_diskette_function.AX	set	$16
_int13_diskette_function.val8	set	$F
.int13_diskette_function.val8	set	-$F
_int13_diskette_function.last_addr	set	0
.int13_diskette_function.last_addr	set	-$1E
_int13_diskette_function.page	set	$11
.int13_diskette_function.page	set	-$D
_int13_diskette_function.ah	set	4
.int13_diskette_function.ah	set	-$1A
_int13_diskette_function.drive	set	$1D
.int13_diskette_function.drive	set	-1
_int13_diskette_function.num_floppies	set	5
.int13_diskette_function.num_floppies	set	-$19
_int13_diskette_function.BX	set	$2E
.int13_diskette_function.BX	set	$10
        cli
! 4443 endasm
!BCC_ENDASM
!BCC_EOS
! 4444         val8 = read_byte(0x0000, 0x043e);
! Debug: list int = const $43E (used reg = )
mov	ax,#$43E
push	ax
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: func () unsigned char = read_byte+0 (used reg = )
call	_read_byte
add	sp,*4
! Debug: eq unsigned char = al+0 to unsigned char val8 = [S+$20-$11] (used reg = )
mov	-$F[bp],al
!BCC_EOS
! 4445         val8 &= 0x7f;
! Debug: andab int = const $7F to unsigned char val8 = [S+$20-$11] (used reg = )
mov	al,-$F[bp]
and	al,*$7F
mov	-$F[bp],al
!BCC_EOS
! 4446         write_byte(0x0000, 0x043e, val8);
! Debug: list unsigned char val8 = [S+$20-$11] (used reg = )
mov	al,-$F[bp]
xor	ah,ah
push	ax
! Debug: list int = const $43E (used reg = )
mov	ax,#$43E
push	ax
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 4447         val8 = inb(0x3f4);
! Debug: list int = const $3F4 (used reg = )
mov	ax,#$3F4
push	ax
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char val8 = [S+$20-$11] (used reg = )
mov	-$F[bp],al
!BCC_EOS
! 4448         if ( (val8 & 0xc0) != 0xc0 )
! Debug: and int = const $C0 to unsigned char val8 = [S+$20-$11] (used reg = )
mov	al,-$F[bp]
and	al,#$C0
! Debug: ne int = const $C0 to unsigned char = al+0 (used reg = )
cmp	al,#$C0
je  	.636
.637:
! 4449           bios_printf((2 | 4 | 1), "int13_diskette: ctrl not ready\n");
! Debug: list * char = .638+0 (used reg = )
mov	bx,#.638
push	bx
! Debug: list int = const 7 (used reg = )
mov	ax,*7
push	ax
! Debug: func () void = bios_printf+0 (used reg = )
call	_bios_printf
add	sp,*4
!BCC_EOS
! 4450         return_status[0] = inb(0x3f5);
.636:
! Debug: list int = const $3F5 (used reg = )
mov	ax,#$3F5
push	ax
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char return_status = [S+$20-$19] (used reg = )
mov	-$17[bp],al
!BCC_EOS
! 4451         return_status[1] = inb(0x3f5);
! Debug: list int = const $3F5 (used reg = )
mov	ax,#$3F5
push	ax
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char return_status = [S+$20-$18] (used reg = )
mov	-$16[bp],al
!BCC_EOS
! 4452         return_status[2] = inb(0x3f5);
! Debug: list int = const $3F5 (used reg = )
mov	ax,#$3F5
push	ax
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char return_status = [S+$20-$17] (used reg = )
mov	-$15[bp],al
!BCC_EOS
! 4453         return_status[3] = inb(0x3f5);
! Debug: list int = const $3F5 (used reg = )
mov	ax,#$3F5
push	ax
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char return_status = [S+$20-$16] (used reg = )
mov	-$14[bp],al
!BCC_EOS
! 4454         return_status[4] = inb(0x3f5);
! Debug: list int = const $3F5 (used reg = )
mov	ax,#$3F5
push	ax
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char return_status = [S+$20-$15] (used reg = )
mov	-$13[bp],al
!BCC_EOS
! 4455         return_status[5] = inb(0x3f5);
! Debug: list int = const $3F5 (used reg = )
mov	ax,#$3F5
push	ax
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char return_status = [S+$20-$14] (used reg = )
mov	-$12[bp],al
!BCC_EOS
! 4456         return_status[6] = inb(0x3f5);
! Debug: list int = const $3F5 (used reg = )
mov	ax,#$3F5
push	ax
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char return_status = [S+$20-$13] (used reg = )
mov	-$11[bp],al
!BCC_EOS
! 4457         write_byte(0x0040, 0x0042, return_status[0]);
! Debug: list unsigned char return_status = [S+$20-$19] (used reg = )
mov	al,-$17[bp]
xor	ah,ah
push	ax
! Debug: list int = const $42 (used reg = )
mov	ax,*$42
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 4458         write_byte(0x0040, 0x0043, return_status[1]);
! Debug: list unsigned char return_status = [S+$20-$18] (used reg = )
mov	al,-$16[bp]
xor	ah,ah
push	ax
! Debug: list int = const $43 (used reg = )
mov	ax,*$43
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 4459         write_byte(0x0040, 0x0044, return_status[2]);
! Debug: list unsigned char return_status = [S+$20-$17] (used reg = )
mov	al,-$15[bp]
xor	ah,ah
push	ax
! Debug: list int = const $44 (used reg = )
mov	ax,*$44
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 4460         write_byte(0x0040, 0x0045, return_status[3]);
! Debug: list unsigned char return_status = [S+$20-$16] (used reg = )
mov	al,-$14[bp]
xor	ah,ah
push	ax
! Debug: list int = const $45 (used reg = )
mov	ax,*$45
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 4461         write_byte(0x0040, 0x0046, return_status[4]);
! Debug: list unsigned char return_status = [S+$20-$15] (used reg = )
mov	al,-$13[bp]
xor	ah,ah
push	ax
! Debug: list int = const $46 (used reg = )
mov	ax,*$46
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 4462         write_byte(0x0040, 0x0047, return_status[5]);
! Debug: list unsigned char return_status = [S+$20-$14] (used reg = )
mov	al,-$12[bp]
xor	ah,ah
push	ax
! Debug: list int = const $47 (used reg = )
mov	ax,*$47
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 4463         write_byte(0x0040, 0x0048, return_status[6]);
! Debug: list unsigned char return_status = [S+$20-$13] (used reg = )
mov	al,-$11[bp]
xor	ah,ah
push	ax
! Debug: list int = const $48 (used reg = )
mov	ax,*$48
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 4464         if ( (return_status[0] & 0xc0) != 0 ) {
! Debug: and int = const $C0 to unsigned char return_status = [S+$20-$19] (used reg = )
mov	al,-$17[bp]
and	al,#$C0
! Debug: ne int = const 0 to unsigned char = al+0 (used reg = )
test	al,al
je  	.639
.63A:
! 4465           AX = ((AX & 0x00ff) | ((0x20) << 8));
! Debug: and int = const $FF to unsigned short AX = [S+$20+$14] (used reg = )
mov	al,$16[bp]
! Debug: or int = const $2000 to unsigned char = al+0 (used reg = )
xor	ah,ah
or	ax,#$2000
! Debug: eq unsigned int = ax+0 to unsigned short AX = [S+$20+$14] (used reg = )
mov	$16[bp],ax
!BCC_EOS
! 4466           set_diskette_ret_status(0x20);
! Debug: list int = const $20 (used reg = )
mov	ax,*$20
push	ax
! Debug: func () void = set_diskette_ret_status+0 (used reg = )
call	_set_diskette_ret_status
inc	sp
inc	sp
!BCC_EOS
! 4467           AX = ((AX & 0xff00) | (0));
! Debug: and unsigned int = const $FF00 to unsigned short AX = [S+$20+$14] (used reg = )
mov	ax,$16[bp]
xor	al,al
! Debug: or int = const 0 to unsigned int = ax+0 (used reg = )
or	al,*0
! Debug: eq unsigned int = ax+0 to unsigned short AX = [S+$20+$14] (used reg = )
mov	$16[bp],ax
!BCC_EOS
! 4468           FLAGS |= 0x0001;
! Debug: orab int = const 1 to unsigned short FLAGS = [S+$20+$1A] (used reg = )
mov	ax,$1C[bp]
or	al,*1
mov	$1C[bp],ax
!BCC_EOS
! 4469           return;
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 4470           }
! 4470 
! 4471         set_diskette_current_cyl(drive, track);
.639:
! Debug: list unsigned char track = [S+$20-5] (used reg = )
mov	al,-3[bp]
xor	ah,ah
push	ax
! Debug: list unsigned char drive = [S+$22-3] (used reg = )
mov	al,-1[bp]
xor	ah,ah
push	ax
! Debug: func () void = set_diskette_current_cyl+0 (used reg = )
call	_set_diskette_current_cyl
add	sp,*4
!BCC_EOS
! 4472         AX = ((AX & 0x00ff) | ((0x00) << 8));
! Debug: and int = const $FF to unsigned short AX = [S+$20+$14] (used reg = )
mov	al,$16[bp]
! Debug: or int = const 0 to unsigned char = al+0 (used reg = )
or	al,*0
! Debug: eq unsigned char = al+0 to unsigned short AX = [S+$20+$14] (used reg = )
xor	ah,ah
mov	$16[bp],ax
!BCC_EOS
! 4473         FLAGS &= 0xfffe;
! Debug: andab unsigned int = const $FFFE to unsigned short FLAGS = [S+$20+$1A] (used reg = )
mov	ax,$1C[bp]
and	al,#$FE
mov	$1C[bp],ax
!BCC_EOS
! 4474         return;
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 4475         }
! 4476       else if (ah == 0x03) {
br 	.63B
.626:
! Debug: logeq int = const 3 to unsigned char ah = [S+$20-$1C] (used reg = )
mov	al,-$1A[bp]
cmp	al,*3
bne 	.63C
.63D:
! 4477         page = (ES >> 12);
! Debug: sr int = const $C to unsigned short ES = [S+$20+4] (used reg = )
mov	ax,6[bp]
mov	al,ah
xor	ah,ah
mov	cl,*4
shr	ax,cl
! Debug: eq unsigned int = ax+0 to unsigned char page = [S+$20-$F] (used reg = )
mov	-$D[bp],al
!BCC_EOS
! 4478         base_es = (ES << 4);
! Debug: sl int = const 4 to unsigned short ES = [S+$20+4] (used reg = )
mov	ax,6[bp]
mov	cl,*4
shl	ax,cl
! Debug: eq unsigned int = ax+0 to unsigned short base_es = [S+$20-$E] (used reg = )
mov	-$C[bp],ax
!BCC_EOS
! 4479         base_address = base_es + BX;
! Debug: add unsigned short BX = [S+$20+$E] to unsigned short base_es = [S+$20-$E] (used reg = )
mov	ax,-$C[bp]
add	ax,$10[bp]
! Debug: eq unsigned int = ax+0 to unsigned short base_address = [S+$20-$A] (used reg = )
mov	-8[bp],ax
!BCC_EOS
! 4480         if ( base_address < base_es ) {
! Debug: lt unsigned short base_es = [S+$20-$E] to unsigned short base_address = [S+$20-$A] (used reg = )
mov	ax,-8[bp]
cmp	ax,-$C[bp]
jae 	.63E
.63F:
! 4481           page++;
! Debug: postinc unsigned char page = [S+$20-$F] (used reg = )
mov	al,-$D[bp]
inc	ax
mov	-$D[bp],al
!BCC_EOS
! 4482           }
! 4483         base_count = (num_sectors * 512) - 1;
.63E:
! Debug: mul int = const $200 to unsigned char num_sectors = [S+$20-4] (used reg = )
mov	al,-2[bp]
xor	ah,ah
mov	cx,#$200
imul	cx
! Debug: sub int = const 1 to unsigned int = ax+0 (used reg = )
! Debug: eq unsigned int = ax-1 to unsigned short base_count = [S+$20-$C] (used reg = )
dec	ax
mov	-$A[bp],ax
!BCC_EOS
! 4484         last_addr = base_address + base_count;
! Debug: add unsigned short base_count = [S+$20-$C] to unsigned short base_address = [S+$20-$A] (used reg = )
mov	ax,-8[bp]
add	ax,-$A[bp]
! Debug: eq unsigned int = ax+0 to unsigned short last_addr = [S+$20-$20] (used reg = )
mov	-$1E[bp],ax
!BCC_EOS
! 4485         if (last_addr < base_address) {
! Debug: lt unsigned short base_address = [S+$20-$A] to unsigned short last_addr = [S+$20-$20] (used reg = )
mov	ax,-$1E[bp]
cmp	ax,-8[bp]
jae 	.640
.641:
! 4486           AX = ((AX & 0x00ff) | ((0x09) << 8));
! Debug: and int = const $FF to unsigned short AX = [S+$20+$14] (used reg = )
mov	al,$16[bp]
! Debug: or int = const $900 to unsigned char = al+0 (used reg = )
xor	ah,ah
or	ax,#$900
! Debug: eq unsigned int = ax+0 to unsigned short AX = [S+$20+$14] (used reg = )
mov	$16[bp],ax
!BCC_EOS
! 4487           set_diskette_ret_status(0x09);
! Debug: list int = const 9 (used reg = )
mov	ax,*9
push	ax
! Debug: func () void = set_diskette_ret_status+0 (used reg = )
call	_set_diskette_ret_status
inc	sp
inc	sp
!BCC_EOS
! 4488           AX = ((AX & 0xff00) | (0));
! Debug: and unsigned int = const $FF00 to unsigned short AX = [S+$20+$14] (used reg = )
mov	ax,$16[bp]
xor	al,al
! Debug: or int = const 0 to unsigned int = ax+0 (used reg = )
or	al,*0
! Debug: eq unsigned int = ax+0 to unsigned short AX = [S+$20+$14] (used reg = )
mov	$16[bp],ax
!BCC_EOS
! 4489           FLAGS |= 0x0001;
! Debug: orab int = const 1 to unsigned short FLAGS = [S+$20+$1A] (used reg = )
mov	ax,$1C[bp]
or	al,*1
mov	$1C[bp],ax
!BCC_EOS
! 4490           return;
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 4491           }
! 4492         ;
.640:
!BCC_EOS
! 4493         outb(0x000a, 0x06);
! Debug: list int = const 6 (used reg = )
mov	ax,*6
push	ax
! Debug: list int = const $A (used reg = )
mov	ax,*$A
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 4494         outb(0x000c, 0x00);
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: list int = const $C (used reg = )
mov	ax,*$C
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 4495         outb(0x0004, base_address);
! Debug: list unsigned short base_address = [S+$20-$A] (used reg = )
push	-8[bp]
! Debug: list int = const 4 (used reg = )
mov	ax,*4
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 4496         outb(0x0004, base_address>>8);
! Debug: sr int = const 8 to unsigned short base_address = [S+$20-$A] (used reg = )
mov	ax,-8[bp]
mov	al,ah
xor	ah,ah
! Debug: list unsigned int = ax+0 (used reg = )
push	ax
! Debug: list int = const 4 (used reg = )
mov	ax,*4
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 4497         outb(0x000c, 0x00);
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: list int = const $C (used reg = )
mov	ax,*$C
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 4498         outb(0x0005, base_count);
! Debug: list unsigned short base_count = [S+$20-$C] (used reg = )
push	-$A[bp]
! Debug: list int = const 5 (used reg = )
mov	ax,*5
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 4499         outb(0x0005, base_count>>8);
! Debug: sr int = const 8 to unsigned short base_count = [S+$20-$C] (used reg = )
mov	ax,-$A[bp]
mov	al,ah
xor	ah,ah
! Debug: list unsigned int = ax+0 (used reg = )
push	ax
! Debug: list int = const 5 (used reg = )
mov	ax,*5
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 4500         mode_register = 0x4a;
! Debug: eq int = const $4A to unsigned char mode_register = [S+$20-$10] (used reg = )
mov	al,*$4A
mov	-$E[bp],al
!BCC_EOS
! 4501         outb(0x000b, mode_register);
! Debug: list unsigned char mode_register = [S+$20-$10] (used reg = )
mov	al,-$E[bp]
xor	ah,ah
push	ax
! Debug: list int = const $B (used reg = )
mov	ax,*$B
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 4502         outb(0x0081, page);
! Debug: list unsigned char page = [S+$20-$F] (used reg = )
mov	al,-$D[bp]
xor	ah,ah
push	ax
! Debug: list int = const $81 (used reg = )
mov	ax,#$81
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 4503         ;
!BCC_EOS
! 4504         outb(0x000a, 0x02);
! Debug: list int = const 2 (used reg = )
mov	ax,*2
push	ax
! Debug: list int = const $A (used reg = )
mov	ax,*$A
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 4505         val8 = read_byte(0x0000, 0x043e);
! Debug: list int = const $43E (used reg = )
mov	ax,#$43E
push	ax
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: func () unsigned char = read_byte+0 (used reg = )
call	_read_byte
add	sp,*4
! Debug: eq unsigned char = al+0 to unsigned char val8 = [S+$20-$11] (used reg = )
mov	-$F[bp],al
!BCC_EOS
! 4506         val8 &= 0x7f;
! Debug: andab int = const $7F to unsigned char val8 = [S+$20-$11] (used reg = )
mov	al,-$F[bp]
and	al,*$7F
mov	-$F[bp],al
!BCC_EOS
! 4507         write_byte(0x0000, 0x043e, val8);
! Debug: list unsigned char val8 = [S+$20-$11] (used reg = )
mov	al,-$F[bp]
xor	ah,ah
push	ax
! Debug: list int = const $43E (used reg = )
mov	ax,#$43E
push	ax
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 4508         if (drive)
mov	al,-1[bp]
test	al,al
je  	.642
.643:
! 4509           dor = 0x20;
! Debug: eq int = const $20 to unsigned char dor = [S+$20-$12] (used reg = )
mov	al,*$20
mov	-$10[bp],al
!BCC_EOS
! 4510         else
! 4511           dor = 0x10;
jmp .644
.642:
! Debug: eq int = const $10 to unsigned char dor = [S+$20-$12] (used reg = )
mov	al,*$10
mov	-$10[bp],al
!BCC_EOS
! 4512         dor |= 0x0c;
.644:
! Debug: orab int = const $C to unsigned char dor = [S+$20-$12] (used reg = )
mov	al,-$10[bp]
or	al,*$C
mov	-$10[bp],al
!BCC_EOS
! 4513         dor |= drive;
! Debug: orab unsigned char drive = [S+$20-3] to unsigned char dor = [S+$20-$12] (used reg = )
mov	al,-$10[bp]
or	al,-1[bp]
mov	-$10[bp],al
!BCC_EOS
! 4514         outb(0x03f2, dor);
! Debug: list unsigned char dor = [S+$20-$12] (used reg = )
mov	al,-$10[bp]
xor	ah,ah
push	ax
! Debug: list int = const $3F2 (used reg = )
mov	ax,#$3F2
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 4515         write_byte(0x40,0x40, 37);
! Debug: list int = const $25 (used reg = )
mov	ax,*$25
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 4516         val8 = inb(0x3f4);
! Debug: list int = const $3F4 (used reg = )
mov	ax,#$3F4
push	ax
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char val8 = [S+$20-$11] (used reg = )
mov	-$F[bp],al
!BCC_EOS
! 4517         if ( (val8 & 0xf0) != 0x80 )
! Debug: and int = const $F0 to unsigned char val8 = [S+$20-$11] (used reg = )
mov	al,-$F[bp]
and	al,#$F0
! Debug: ne int = const $80 to unsigned char = al+0 (used reg = )
cmp	al,#$80
je  	.645
.646:
! 4518           bios_printf((2 | 4 | 1), "int13_diskette:f03: ctrl not ready\n");
! Debug: list * char = .647+0 (used reg = )
mov	bx,#.647
push	bx
! Debug: list int = const 7 (used reg = )
mov	ax,*7
push	ax
! Debug: func () void = bios_printf+0 (used reg = )
call	_bios_printf
add	sp,*4
!BCC_EOS
! 4519         outb(0x03f5, 0xc5);
.645:
! Debug: list int = const $C5 (used reg = )
mov	ax,#$C5
push	ax
! Debug: list int = const $3F5 (used reg = )
mov	ax,#$3F5
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 4520         outb(0x03f5, (head << 2) | drive);
! Debug: sl int = const 2 to unsigned char head = [S+$20-7] (used reg = )
mov	al,-5[bp]
xor	ah,ah
shl	ax,*1
shl	ax,*1
! Debug: or unsigned char drive = [S+$20-3] to unsigned int = ax+0 (used reg = )
or	al,-1[bp]
! Debug: list unsigned int = ax+0 (used reg = )
push	ax
! Debug: list int = const $3F5 (used reg = )
mov	ax,#$3F5
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 4521         outb(0x03f5, track);
! Debug: list unsigned char track = [S+$20-5] (used reg = )
mov	al,-3[bp]
xor	ah,ah
push	ax
! Debug: list int = const $3F5 (used reg = )
mov	ax,#$3F5
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 4522         outb(0x03f5, head);
! Debug: list unsigned char head = [S+$20-7] (used reg = )
mov	al,-5[bp]
xor	ah,ah
push	ax
! Debug: list int = const $3F5 (used reg = )
mov	ax,#$3F5
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 4523         outb(0x03f5, sector);
! Debug: list unsigned char sector = [S+$20-6] (used reg = )
mov	al,-4[bp]
xor	ah,ah
push	ax
! Debug: list int = const $3F5 (used reg = )
mov	ax,#$3F5
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 4524         outb(0x03f5, 2);
! Debug: list int = const 2 (used reg = )
mov	ax,*2
push	ax
! Debug: list int = const $3F5 (used reg = )
mov	ax,#$3F5
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 4525         outb(0x03f5, sector + num_sectors - 1);
! Debug: add unsigned char num_sectors = [S+$20-4] to unsigned char sector = [S+$20-6] (used reg = )
mov	al,-4[bp]
xor	ah,ah
add	al,-2[bp]
adc	ah,*0
! Debug: sub int = const 1 to unsigned int = ax+0 (used reg = )
! Debug: list unsigned int = ax-1 (used reg = )
dec	ax
push	ax
! Debug: list int = const $3F5 (used reg = )
mov	ax,#$3F5
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 4526         outb(0x03f5, 0);
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: list int = const $3F5 (used reg = )
mov	ax,#$3F5
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 4527         outb(0x03f5, 0xff);
! Debug: list int = const $FF (used reg = )
mov	ax,#$FF
push	ax
! Debug: list int = const $3F5 (used reg = )
mov	ax,#$3F5
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 4528 #asm
!BCC_EOS
!BCC_ASM
_int13_diskette_function.BP	set	$2A
.int13_diskette_function.BP	set	$C
_int13_diskette_function.CS	set	$38
.int13_diskette_function.CS	set	$1A
_int13_diskette_function.CX	set	$32
.int13_diskette_function.CX	set	$14
_int13_diskette_function.base_address	set	$16
.int13_diskette_function.base_address	set	-8
_int13_diskette_function.DI	set	$26
.int13_diskette_function.DI	set	8
_int13_diskette_function.FLAGS	set	$3A
.int13_diskette_function.FLAGS	set	$1C
_int13_diskette_function.base_count	set	$14
.int13_diskette_function.base_count	set	-$A
_int13_diskette_function.sector	set	$1A
.int13_diskette_function.sector	set	-4
_int13_diskette_function.DS	set	$22
.int13_diskette_function.DS	set	4
_int13_diskette_function.head	set	$19
.int13_diskette_function.head	set	-5
_int13_diskette_function.ELDX	set	$2C
.int13_diskette_function.ELDX	set	$E
_int13_diskette_function.dor	set	$E
.int13_diskette_function.dor	set	-$10
_int13_diskette_function.DX	set	$30
.int13_diskette_function.DX	set	$12
_int13_diskette_function.return_status	set	7
.int13_diskette_function.return_status	set	-$17
_int13_diskette_function.es	set	2
.int13_diskette_function.es	set	-$1C
_int13_diskette_function.mode_register	set	$10
.int13_diskette_function.mode_register	set	-$E
_int13_diskette_function.ES	set	$24
.int13_diskette_function.ES	set	6
_int13_diskette_function.base_es	set	$12
.int13_diskette_function.base_es	set	-$C
_int13_diskette_function.track	set	$1B
.int13_diskette_function.track	set	-3
_int13_diskette_function.SI	set	$28
.int13_diskette_function.SI	set	$A
_int13_diskette_function.drive_type	set	6
.int13_diskette_function.drive_type	set	-$18
_int13_diskette_function.num_sectors	set	$1C
.int13_diskette_function.num_sectors	set	-2
_int13_diskette_function.IP	set	$36
.int13_diskette_function.IP	set	$18
_int13_diskette_function.status	set	$18
.int13_diskette_function.status	set	-6
_int13_diskette_function.AX	set	$34
.int13_diskette_function.AX	set	$16
_int13_diskette_function.val8	set	$F
.int13_diskette_function.val8	set	-$F
_int13_diskette_function.last_addr	set	0
.int13_diskette_function.last_addr	set	-$1E
_int13_diskette_function.page	set	$11
.int13_diskette_function.page	set	-$D
_int13_diskette_function.ah	set	4
.int13_diskette_function.ah	set	-$1A
_int13_diskette_function.drive	set	$1D
.int13_diskette_function.drive	set	-1
_int13_diskette_function.num_floppies	set	5
.int13_diskette_function.num_floppies	set	-$19
_int13_diskette_function.BX	set	$2E
.int13_diskette_function.BX	set	$10
        sti
! 4530 endasm
!BCC_ENDASM
!BCC_EOS
! 4531         val8 = (read_byte(0x0000, 0x043e) & 0x80);
! Debug: list int = const $43E (used reg = )
mov	ax,#$43E
push	ax
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: func () unsigned char = read_byte+0 (used reg = )
call	_read_byte
add	sp,*4
! Debug: and int = const $80 to unsigned char = al+0 (used reg = )
and	al,#$80
! Debug: eq unsigned char = al+0 to unsigned char val8 = [S+$20-$11] (used reg = )
mov	-$F[bp],al
!BCC_EOS
! 4532         while ( val8 == 0 ) {
jmp .649
.64A:
! 4533           val8 = (read_byte(0x0000, 0x043e) & 0x80);
! Debug: list int = const $43E (used reg = )
mov	ax,#$43E
push	ax
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: func () unsigned char = read_byte+0 (used reg = )
call	_read_byte
add	sp,*4
! Debug: and int = const $80 to unsigned char = al+0 (used reg = )
and	al,#$80
! Debug: eq unsigned char = al+0 to unsigned char val8 = [S+$20-$11] (used reg = )
mov	-$F[bp],al
!BCC_EOS
! 4534           }
! 4535        val8 = 0;
.649:
! Debug: logeq int = const 0 to unsigned char val8 = [S+$20-$11] (used reg = )
mov	al,-$F[bp]
test	al,al
je 	.64A
.64B:
.648:
! Debug: eq int = const 0 to unsigned char val8 = [S+$20-$11] (used reg = )
xor	al,al
mov	-$F[bp],al
!BCC_EOS
! 4536 #asm
!BCC_EOS
!BCC_ASM
_int13_diskette_function.BP	set	$2A
.int13_diskette_function.BP	set	$C
_int13_diskette_function.CS	set	$38
.int13_diskette_function.CS	set	$1A
_int13_diskette_function.CX	set	$32
.int13_diskette_function.CX	set	$14
_int13_diskette_function.base_address	set	$16
.int13_diskette_function.base_address	set	-8
_int13_diskette_function.DI	set	$26
.int13_diskette_function.DI	set	8
_int13_diskette_function.FLAGS	set	$3A
.int13_diskette_function.FLAGS	set	$1C
_int13_diskette_function.base_count	set	$14
.int13_diskette_function.base_count	set	-$A
_int13_diskette_function.sector	set	$1A
.int13_diskette_function.sector	set	-4
_int13_diskette_function.DS	set	$22
.int13_diskette_function.DS	set	4
_int13_diskette_function.head	set	$19
.int13_diskette_function.head	set	-5
_int13_diskette_function.ELDX	set	$2C
.int13_diskette_function.ELDX	set	$E
_int13_diskette_function.dor	set	$E
.int13_diskette_function.dor	set	-$10
_int13_diskette_function.DX	set	$30
.int13_diskette_function.DX	set	$12
_int13_diskette_function.return_status	set	7
.int13_diskette_function.return_status	set	-$17
_int13_diskette_function.es	set	2
.int13_diskette_function.es	set	-$1C
_int13_diskette_function.mode_register	set	$10
.int13_diskette_function.mode_register	set	-$E
_int13_diskette_function.ES	set	$24
.int13_diskette_function.ES	set	6
_int13_diskette_function.base_es	set	$12
.int13_diskette_function.base_es	set	-$C
_int13_diskette_function.track	set	$1B
.int13_diskette_function.track	set	-3
_int13_diskette_function.SI	set	$28
.int13_diskette_function.SI	set	$A
_int13_diskette_function.drive_type	set	6
.int13_diskette_function.drive_type	set	-$18
_int13_diskette_function.num_sectors	set	$1C
.int13_diskette_function.num_sectors	set	-2
_int13_diskette_function.IP	set	$36
.int13_diskette_function.IP	set	$18
_int13_diskette_function.status	set	$18
.int13_diskette_function.status	set	-6
_int13_diskette_function.AX	set	$34
.int13_diskette_function.AX	set	$16
_int13_diskette_function.val8	set	$F
.int13_diskette_function.val8	set	-$F
_int13_diskette_function.last_addr	set	0
.int13_diskette_function.last_addr	set	-$1E
_int13_diskette_function.page	set	$11
.int13_diskette_function.page	set	-$D
_int13_diskette_function.ah	set	4
.int13_diskette_function.ah	set	-$1A
_int13_diskette_function.drive	set	$1D
.int13_diskette_function.drive	set	-1
_int13_diskette_function.num_floppies	set	5
.int13_diskette_function.num_floppies	set	-$19
_int13_diskette_function.BX	set	$2E
.int13_diskette_function.BX	set	$10
        cli
! 4538 endasm
!BCC_ENDASM
!BCC_EOS
! 4539         val8 = read_byte(0x0000, 0x043e);
! Debug: list int = const $43E (used reg = )
mov	ax,#$43E
push	ax
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: func () unsigned char = read_byte+0 (used reg = )
call	_read_byte
add	sp,*4
! Debug: eq unsigned char = al+0 to unsigned char val8 = [S+$20-$11] (used reg = )
mov	-$F[bp],al
!BCC_EOS
! 4540         val8 &= 0x7f;
! Debug: andab int = const $7F to unsigned char val8 = [S+$20-$11] (used reg = )
mov	al,-$F[bp]
and	al,*$7F
mov	-$F[bp],al
!BCC_EOS
! 4541         write_byte(0x0000, 0x043
! 4541 e, val8);
! Debug: list unsigned char val8 = [S+$20-$11] (used reg = )
mov	al,-$F[bp]
xor	ah,ah
push	ax
! Debug: list int = const $43E (used reg = )
mov	ax,#$43E
push	ax
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 4542         val8 = inb(0x3f4);
! Debug: list int = const $3F4 (used reg = )
mov	ax,#$3F4
push	ax
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char val8 = [S+$20-$11] (used reg = )
mov	-$F[bp],al
!BCC_EOS
! 4543         if ( (val8 & 0xc0) != 0xc0 )
! Debug: and int = const $C0 to unsigned char val8 = [S+$20-$11] (used reg = )
mov	al,-$F[bp]
and	al,#$C0
! Debug: ne int = const $C0 to unsigned char = al+0 (used reg = )
cmp	al,#$C0
je  	.64C
.64D:
! 4544           bios_printf((2 | 4 | 1), "int13_diskette: ctrl not ready\n");
! Debug: list * char = .64E+0 (used reg = )
mov	bx,#.64E
push	bx
! Debug: list int = const 7 (used reg = )
mov	ax,*7
push	ax
! Debug: func () void = bios_printf+0 (used reg = )
call	_bios_printf
add	sp,*4
!BCC_EOS
! 4545         return_status[0] = inb(0x3f5);
.64C:
! Debug: list int = const $3F5 (used reg = )
mov	ax,#$3F5
push	ax
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char return_status = [S+$20-$19] (used reg = )
mov	-$17[bp],al
!BCC_EOS
! 4546         return_status[1] = inb(0x3f5);
! Debug: list int = const $3F5 (used reg = )
mov	ax,#$3F5
push	ax
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char return_status = [S+$20-$18] (used reg = )
mov	-$16[bp],al
!BCC_EOS
! 4547         return_status[2] = inb(0x3f5);
! Debug: list int = const $3F5 (used reg = )
mov	ax,#$3F5
push	ax
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char return_status = [S+$20-$17] (used reg = )
mov	-$15[bp],al
!BCC_EOS
! 4548         return_status[3] = inb(0x3f5);
! Debug: list int = const $3F5 (used reg = )
mov	ax,#$3F5
push	ax
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char return_status = [S+$20-$16] (used reg = )
mov	-$14[bp],al
!BCC_EOS
! 4549         return_status[4] = inb(0x3f5);
! Debug: list int = const $3F5 (used reg = )
mov	ax,#$3F5
push	ax
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char return_status = [S+$20-$15] (used reg = )
mov	-$13[bp],al
!BCC_EOS
! 4550         return_status[5] = inb(0x3f5);
! Debug: list int = const $3F5 (used reg = )
mov	ax,#$3F5
push	ax
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char return_status = [S+$20-$14] (used reg = )
mov	-$12[bp],al
!BCC_EOS
! 4551         return_status[6] = inb(0x3f5);
! Debug: list int = const $3F5 (used reg = )
mov	ax,#$3F5
push	ax
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char return_status = [S+$20-$13] (used reg = )
mov	-$11[bp],al
!BCC_EOS
! 4552         write_byte(0x0040, 0x0042, return_status[0]);
! Debug: list unsigned char return_status = [S+$20-$19] (used reg = )
mov	al,-$17[bp]
xor	ah,ah
push	ax
! Debug: list int = const $42 (used reg = )
mov	ax,*$42
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 4553         write_byte(0x0040, 0x0043, return_status[1]);
! Debug: list unsigned char return_status = [S+$20-$18] (used reg = )
mov	al,-$16[bp]
xor	ah,ah
push	ax
! Debug: list int = const $43 (used reg = )
mov	ax,*$43
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 4554         write_byte(0x0040, 0x0044, return_status[2]);
! Debug: list unsigned char return_status = [S+$20-$17] (used reg = )
mov	al,-$15[bp]
xor	ah,ah
push	ax
! Debug: list int = const $44 (used reg = )
mov	ax,*$44
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 4555         write_byte(0x0040, 0x0045, return_status[3]);
! Debug: list unsigned char return_status = [S+$20-$16] (used reg = )
mov	al,-$14[bp]
xor	ah,ah
push	ax
! Debug: list int = const $45 (used reg = )
mov	ax,*$45
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 4556         write_byte(0x0040, 0x0046, return_status[4]);
! Debug: list unsigned char return_status = [S+$20-$15] (used reg = )
mov	al,-$13[bp]
xor	ah,ah
push	ax
! Debug: list int = const $46 (used reg = )
mov	ax,*$46
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 4557         write_byte(0x0040, 0x0047, return_status[5]);
! Debug: list unsigned char return_status = [S+$20-$14] (used reg = )
mov	al,-$12[bp]
xor	ah,ah
push	ax
! Debug: list int = const $47 (used reg = )
mov	ax,*$47
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 4558         write_byte(0x0040, 0x0048, return_status[6]);
! Debug: list unsigned char return_status = [S+$20-$13] (used reg = )
mov	al,-$11[bp]
xor	ah,ah
push	ax
! Debug: list int = const $48 (used reg = )
mov	ax,*$48
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 4559         if ( (return_status[0] & 0xc0) != 0 ) {
! Debug: and int = const $C0 to unsigned char return_status = [S+$20-$19] (used reg = )
mov	al,-$17[bp]
and	al,#$C0
! Debug: ne int = const 0 to unsigned char = al+0 (used reg = )
test	al,al
je  	.64F
.650:
! 4560           if ( (return_status[1] & 0x02) != 0 ) {
! Debug: and int = const 2 to unsigned char return_status = [S+$20-$18] (used reg = )
mov	al,-$16[bp]
and	al,*2
! Debug: ne int = const 0 to unsigned char = al+0 (used reg = )
test	al,al
je  	.651
.652:
! 4561             AX = 0x0300;
! Debug: eq int = const $300 to unsigned short AX = [S+$20+$14] (used reg = )
mov	ax,#$300
mov	$16[bp],ax
!BCC_EOS
! 4562             FLAGS |= 0x0001;
! Debug: orab int = const 1 to unsigned short FLAGS = [S+$20+$1A] (used reg = )
mov	ax,$1C[bp]
or	al,*1
mov	$1C[bp],ax
!BCC_EOS
! 4563             return;
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 4564           } else {
jmp .653
.651:
! 4565             bios_printf((2 | 4 | 1), "int13_diskette_function: read error\n");
! Debug: list * char = .654+0 (used reg = )
mov	bx,#.654
push	bx
! Debug: list int = const 7 (used reg = )
mov	ax,*7
push	ax
! Debug: func () void = bios_printf+0 (used reg = )
call	_bios_printf
add	sp,*4
!BCC_EOS
! 4566           }
! 4567         }
.653:
! 4568         set_diskette_current_cyl(drive, track);
.64F:
! Debug: list unsigned char track = [S+$20-5] (used reg = )
mov	al,-3[bp]
xor	ah,ah
push	ax
! Debug: list unsigned char drive = [S+$22-3] (used reg = )
mov	al,-1[bp]
xor	ah,ah
push	ax
! Debug: func () void = set_diskette_current_cyl+0 (used reg = )
call	_set_diskette_current_cyl
add	sp,*4
!BCC_EOS
! 4569         AX = ((AX & 0x00ff) | ((0x00) << 8));
! Debug: and int = const $FF to unsigned short AX = [S+$20+$14] (used reg = )
mov	al,$16[bp]
! Debug: or int = const 0 to unsigned char = al+0 (used reg = )
or	al,*0
! Debug: eq unsigned char = al+0 to unsigned short AX = [S+$20+$14] (used reg = )
xor	ah,ah
mov	$16[bp],ax
!BCC_EOS
! 4570         FLAGS &= 0xfffe;
! Debug: andab unsigned int = const $FFFE to unsigned short FLAGS = [S+$20+$1A] (used reg = )
mov	ax,$1C[bp]
and	al,#$FE
mov	$1C[bp],ax
!BCC_EOS
! 4571         return;
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 4572         }
! 4573       else {
jmp .655
.63C:
! 4574         set_diskette_current_cyl(drive, track);
! Debug: list unsigned char track = [S+$20-5] (used reg = )
mov	al,-3[bp]
xor	ah,ah
push	ax
! Debug: list unsigned char drive = [S+$22-3] (used reg = )
mov	al,-1[bp]
xor	ah,ah
push	ax
! Debug: func () void = set_diskette_current_cyl+0 (used reg = )
call	_set_diskette_current_cyl
add	sp,*4
!BCC_EOS
! 4575         FLAGS &= 0xfffe;
! Debug: andab unsigned int = const $FFFE to unsigned short FLAGS = [S+$20+$1A] (used reg = )
mov	ax,$1C[bp]
and	al,#$FE
mov	$1C[bp],ax
!BCC_EOS
! 4576         AX = ((AX & 0x00ff) | ((0x00) << 8));
! Debug: and int = const $FF to unsigned short AX = [S+$20+$14] (used reg = )
mov	al,$16[bp]
! Debug: or int = const 0 to unsigned char = al+0 (used reg = )
or	al,*0
! Debug: eq unsigned char = al+0 to unsigned short AX = [S+$20+$14] (used reg = )
xor	ah,ah
mov	$16[bp],ax
!BCC_EOS
! 4577         return;
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 4578         }
! 4579     case 0x05:
.655:
.63B:
! 4580 ;
.656:
!BCC_EOS
! 4581       num_sectors = ( AX & 0x00ff );
! Debug: and int = const $FF to unsigned short AX = [S+$20+$14] (used reg = )
mov	al,$16[bp]
! Debug: eq unsigned char = al+0 to unsigned char num_sectors = [S+$20-4] (used reg = )
mov	-2[bp],al
!BCC_EOS
! 4582       track = ( CX >> 8 );
! Debug: sr int = const 8 to unsigned short CX = [S+$20+$12] (used reg = )
mov	ax,$14[bp]
mov	al,ah
xor	ah,ah
! Debug: eq unsigned int = ax+0 to unsigned char track = [S+$20-5] (used reg = )
mov	-3[bp],al
!BCC_EOS
! 4583       head = ( DX >> 8 );
! Debug: sr int = const 8 to unsigned short DX = [S+$20+$10] (used reg = )
mov	ax,$12[bp]
mov	al,ah
xor	ah,ah
! Debug: eq unsigned int = ax+0 to unsigned char head = [S+$20-7] (used reg = )
mov	-5[bp],al
!BCC_EOS
! 4584       drive = ( ELDX & 0x00ff );
! Debug: and int = const $FF to unsigned short ELDX = [S+$20+$C] (used reg = )
mov	al,$E[bp]
! Debug: eq unsigned char = al+0 to unsigned char drive = [S+$20-3] (used reg = )
mov	-1[bp],al
!BCC_EOS
! 4585       if ((drive > 1) || (head > 1) || (track > 79) ||
! 4586           (num_sectors == 0) || (num_sectors > 18)) {
! Debug: gt int = const 1 to unsigned char drive = [S+$20-3] (used reg = )
mov	al,-1[bp]
cmp	al,*1
ja  	.658
.65C:
! Debug: gt int = const 1 to unsigned char head = [S+$20-7] (used reg = )
mov	al,-5[bp]
cmp	al,*1
ja  	.658
.65B:
! Debug: gt int = const $4F to unsigned char track = [S+$20-5] (used reg = )
mov	al,-3[bp]
cmp	al,*$4F
ja  	.658
.65A:
! Debug: logeq int = const 0 to unsigned char num_sectors = [S+$20-4] (used reg = )
mov	al,-2[bp]
test	al,al
je  	.658
.659:
! Debug: gt int = const $12 to unsigned char num_sectors = [S+$20-4] (used reg = )
mov	al,-2[bp]
cmp	al,*$12
jbe 	.657
.658:
! 4587         AX = ((AX & 0x00ff) | ((1) << 8));
! Debug: and int = const $FF to unsigned short AX = [S+$20+$14] (used reg = )
mov	al,$16[bp]
! Debug: or int = const $100 to unsigned char = al+0 (used reg = )
xor	ah,ah
or	ax,#$100
! Debug: eq unsigned int = ax+0 to unsigned short AX = [S+$20+$14] (used reg = )
mov	$16[bp],ax
!BCC_EOS
! 4588         set_diskette_ret_status(1);
! Debug: list int = const 1 (used reg = )
mov	ax,*1
push	ax
! Debug: func () void = set_diskette_ret_status+0 (used reg = )
call	_set_diskette_ret_status
inc	sp
inc	sp
!BCC_EOS
! 4589         FLAGS |= 0x0001;
! Debug: orab int = const 1 to unsigned short FLAGS = [S+$20+$1A] (used reg = )
mov	ax,$1C[bp]
or	al,*1
mov	$1C[bp],ax
!BCC_EOS
! 4590         }
! 4591       if (floppy_drive_exists(drive) == 0) {
.657:
! Debug: list unsigned char drive = [S+$20-3] (used reg = )
mov	al,-1[bp]
xor	ah,ah
push	ax
! Debug: func () unsigned short = floppy_drive_exists+0 (used reg = )
call	_floppy_drive_exists
inc	sp
inc	sp
! Debug: logeq int = const 0 to unsigned short = ax+0 (used reg = )
test	ax,ax
jne 	.65D
.65E:
! 4592         AX = ((AX & 0x00ff) | ((0x80) << 8));
! Debug: and int = const $FF to unsigned short AX = [S+$20+$14] (used reg = )
mov	al,$16[bp]
! Debug: or int = const -$8000 to unsigned char = al+0 (used reg = )
xor	ah,ah
or	ax,#-$8000
! Debug: eq unsigned int = ax+0 to unsigned short AX = [S+$20+$14] (used reg = )
mov	$16[bp],ax
!BCC_EOS
! 4593         set_diskette_ret_status(0x80);
! Debug: list int = const $80 (used reg = )
mov	ax,#$80
push	ax
! Debug: func () void = set_diskette_ret_status+0 (used reg = )
call	_set_diskette_ret_status
inc	sp
inc	sp
!BCC_EOS
! 4594         FLAGS |= 0x0001;
! Debug: orab int = const 1 to unsigned short FLAGS = [S+$20+$1A] (used reg = )
mov	ax,$1C[bp]
or	al,*1
mov	$1C[bp],ax
!BCC_EOS
! 4595         return;
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 4596         }
! 4597       if (floppy_media_known(drive) == 0) {
.65D:
! Debug: list unsigned char drive = [S+$20-3] (used reg = )
mov	al,-1[bp]
xor	ah,ah
push	ax
! Debug: func () unsigned short = floppy_media_known+0 (used reg = )
call	_floppy_media_known
inc	sp
inc	sp
! Debug: logeq int = const 0 to unsigned short = ax+0 (used reg = )
test	ax,ax
jne 	.65F
.660:
! 4598         if (floppy_media_sense(drive) == 0) {
! Debug: list unsigned char drive = [S+$20-3] (used reg = )
mov	al,-1[bp]
xor	ah,ah
push	ax
! Debug: func () unsigned short = floppy_media_sense+0 (used reg = )
call	_floppy_media_sense
inc	sp
inc	sp
! Debug: logeq int = const 0 to unsigned short = ax+0 (used reg = )
test	ax,ax
jne 	.661
.662:
! 4599           AX = ((AX & 
! 4599 0x00ff) | ((0x0C) << 8));
! Debug: and int = const $FF to unsigned short AX = [S+$20+$14] (used reg = )
mov	al,$16[bp]
! Debug: or int = const $C00 to unsigned char = al+0 (used reg = )
xor	ah,ah
or	ax,#$C00
! Debug: eq unsigned int = ax+0 to unsigned short AX = [S+$20+$14] (used reg = )
mov	$16[bp],ax
!BCC_EOS
! 4600           set_diskette_ret_status(0x0C);
! Debug: list int = const $C (used reg = )
mov	ax,*$C
push	ax
! Debug: func () void = set_diskette_ret_status+0 (used reg = )
call	_set_diskette_ret_status
inc	sp
inc	sp
!BCC_EOS
! 4601           AX = ((AX & 0xff00) | (0));
! Debug: and unsigned int = const $FF00 to unsigned short AX = [S+$20+$14] (used reg = )
mov	ax,$16[bp]
xor	al,al
! Debug: or int = const 0 to unsigned int = ax+0 (used reg = )
or	al,*0
! Debug: eq unsigned int = ax+0 to unsigned short AX = [S+$20+$14] (used reg = )
mov	$16[bp],ax
!BCC_EOS
! 4602           FLAGS |= 0x0001;
! Debug: orab int = const 1 to unsigned short FLAGS = [S+$20+$1A] (used reg = )
mov	ax,$1C[bp]
or	al,*1
mov	$1C[bp],ax
!BCC_EOS
! 4603           return;
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 4604           }
! 4605         }
.661:
! 4606       page = (ES >> 12);
.65F:
! Debug: sr int = const $C to unsigned short ES = [S+$20+4] (used reg = )
mov	ax,6[bp]
mov	al,ah
xor	ah,ah
mov	cl,*4
shr	ax,cl
! Debug: eq unsigned int = ax+0 to unsigned char page = [S+$20-$F] (used reg = )
mov	-$D[bp],al
!BCC_EOS
! 4607       base_es = (ES << 4);
! Debug: sl int = const 4 to unsigned short ES = [S+$20+4] (used reg = )
mov	ax,6[bp]
mov	cl,*4
shl	ax,cl
! Debug: eq unsigned int = ax+0 to unsigned short base_es = [S+$20-$E] (used reg = )
mov	-$C[bp],ax
!BCC_EOS
! 4608       base_address = base_es + BX;
! Debug: add unsigned short BX = [S+$20+$E] to unsigned short base_es = [S+$20-$E] (used reg = )
mov	ax,-$C[bp]
add	ax,$10[bp]
! Debug: eq unsigned int = ax+0 to unsigned short base_address = [S+$20-$A] (used reg = )
mov	-8[bp],ax
!BCC_EOS
! 4609       if ( base_address < base_es ) {
! Debug: lt unsigned short base_es = [S+$20-$E] to unsigned short base_address = [S+$20-$A] (used reg = )
mov	ax,-8[bp]
cmp	ax,-$C[bp]
jae 	.663
.664:
! 4610         page++;
! Debug: postinc unsigned char page = [S+$20-$F] (used reg = )
mov	al,-$D[bp]
inc	ax
mov	-$D[bp],al
!BCC_EOS
! 4611         }
! 4612       base_count = (num_sectors * 4) - 1;
.663:
! Debug: mul int = const 4 to unsigned char num_sectors = [S+$20-4] (used reg = )
mov	al,-2[bp]
xor	ah,ah
shl	ax,*1
shl	ax,*1
! Debug: sub int = const 1 to unsigned int = ax+0 (used reg = )
! Debug: eq unsigned int = ax-1 to unsigned short base_count = [S+$20-$C] (used reg = )
dec	ax
mov	-$A[bp],ax
!BCC_EOS
! 4613       last_addr = base_address + base_count;
! Debug: add unsigned short base_count = [S+$20-$C] to unsigned short base_address = [S+$20-$A] (used reg = )
mov	ax,-8[bp]
add	ax,-$A[bp]
! Debug: eq unsigned int = ax+0 to unsigned short last_addr = [S+$20-$20] (used reg = )
mov	-$1E[bp],ax
!BCC_EOS
! 4614       if (last_addr < base_address) {
! Debug: lt unsigned short base_address = [S+$20-$A] to unsigned short last_addr = [S+$20-$20] (used reg = )
mov	ax,-$1E[bp]
cmp	ax,-8[bp]
jae 	.665
.666:
! 4615         AX = ((AX & 0x00ff) | ((0x09) << 8));
! Debug: and int = const $FF to unsigned short AX = [S+$20+$14] (used reg = )
mov	al,$16[bp]
! Debug: or int = const $900 to unsigned char = al+0 (used reg = )
xor	ah,ah
or	ax,#$900
! Debug: eq unsigned int = ax+0 to unsigned short AX = [S+$20+$14] (used reg = )
mov	$16[bp],ax
!BCC_EOS
! 4616         set_diskette_ret_status(0x09);
! Debug: list int = const 9 (used reg = )
mov	ax,*9
push	ax
! Debug: func () void = set_diskette_ret_status+0 (used reg = )
call	_set_diskette_ret_status
inc	sp
inc	sp
!BCC_EOS
! 4617         AX = ((AX & 0xff00) | (0));
! Debug: and unsigned int = const $FF00 to unsigned short AX = [S+$20+$14] (used reg = )
mov	ax,$16[bp]
xor	al,al
! Debug: or int = const 0 to unsigned int = ax+0 (used reg = )
or	al,*0
! Debug: eq unsigned int = ax+0 to unsigned short AX = [S+$20+$14] (used reg = )
mov	$16[bp],ax
!BCC_EOS
! 4618         FLAGS |= 0x0001;
! Debug: orab int = const 1 to unsigned short FLAGS = [S+$20+$1A] (used reg = )
mov	ax,$1C[bp]
or	al,*1
mov	$1C[bp],ax
!BCC_EOS
! 4619         return;
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 4620         }
! 4621       outb(0x000a, 0x06);
.665:
! Debug: list int = const 6 (used reg = )
mov	ax,*6
push	ax
! Debug: list int = const $A (used reg = )
mov	ax,*$A
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 4622       outb(0x000c, 0x00);
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: list int = const $C (used reg = )
mov	ax,*$C
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 4623       outb(0x0004, base_address);
! Debug: list unsigned short base_address = [S+$20-$A] (used reg = )
push	-8[bp]
! Debug: list int = const 4 (used reg = )
mov	ax,*4
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 4624       outb(0x0004, base_address>>8);
! Debug: sr int = const 8 to unsigned short base_address = [S+$20-$A] (used reg = )
mov	ax,-8[bp]
mov	al,ah
xor	ah,ah
! Debug: list unsigned int = ax+0 (used reg = )
push	ax
! Debug: list int = const 4 (used reg = )
mov	ax,*4
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 4625       outb(0x000c, 0x00);
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: list int = const $C (used reg = )
mov	ax,*$C
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 4626       outb(0x0005, base_count);
! Debug: list unsigned short base_count = [S+$20-$C] (used reg = )
push	-$A[bp]
! Debug: list int = const 5 (used reg = )
mov	ax,*5
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 4627       outb(0x0005, base_count>>8);
! Debug: sr int = const 8 to unsigned short base_count = [S+$20-$C] (used reg = )
mov	ax,-$A[bp]
mov	al,ah
xor	ah,ah
! Debug: list unsigned int = ax+0 (used reg = )
push	ax
! Debug: list int = const 5 (used reg = )
mov	ax,*5
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 4628       mode_register = 0x4a;
! Debug: eq int = const $4A to unsigned char mode_register = [S+$20-$10] (used reg = )
mov	al,*$4A
mov	-$E[bp],al
!BCC_EOS
! 4629       outb(0x000b, mode_register);
! Debug: list unsigned char mode_register = [S+$20-$10] (used reg = )
mov	al,-$E[bp]
xor	ah,ah
push	ax
! Debug: list int = const $B (used reg = )
mov	ax,*$B
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 4630       outb(0x0081, page);
! Debug: list unsigned char page = [S+$20-$F] (used reg = )
mov	al,-$D[bp]
xor	ah,ah
push	ax
! Debug: list int = const $81 (used reg = )
mov	ax,#$81
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 4631       outb(0x000a, 0x02);
! Debug: list int = const 2 (used reg = )
mov	ax,*2
push	ax
! Debug: list int = const $A (used reg = )
mov	ax,*$A
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 4632       val8 = read_byte(0x0000, 0x043e);
! Debug: list int = const $43E (used reg = )
mov	ax,#$43E
push	ax
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: func () unsigned char = read_byte+0 (used reg = )
call	_read_byte
add	sp,*4
! Debug: eq unsigned char = al+0 to unsigned char val8 = [S+$20-$11] (used reg = )
mov	-$F[bp],al
!BCC_EOS
! 4633       val8 &= 0x7f;
! Debug: andab int = const $7F to unsigned char val8 = [S+$20-$11] (used reg = )
mov	al,-$F[bp]
and	al,*$7F
mov	-$F[bp],al
!BCC_EOS
! 4634       write_byte(0x0000, 0x043e, val8);
! Debug: list unsigned char val8 = [S+$20-$11] (used reg = )
mov	al,-$F[bp]
xor	ah,ah
push	ax
! Debug: list int = const $43E (used reg = )
mov	ax,#$43E
push	ax
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 4635       if (drive)
mov	al,-1[bp]
test	al,al
je  	.667
.668:
! 4636         dor = 0x20;
! Debug: eq int = const $20 to unsigned char dor = [S+$20-$12] (used reg = )
mov	al,*$20
mov	-$10[bp],al
!BCC_EOS
! 4637       else
! 4638         dor = 0x10;
jmp .669
.667:
! Debug: eq int = const $10 to unsigned char dor = [S+$20-$12] (used reg = )
mov	al,*$10
mov	-$10[bp],al
!BCC_EOS
! 4639       dor |= 0x0c;
.669:
! Debug: orab int = const $C to unsigned char dor = [S+$20-$12] (used reg = )
mov	al,-$10[bp]
or	al,*$C
mov	-$10[bp],al
!BCC_EOS
! 4640       dor |= drive;
! Debug: orab unsigned char drive = [S+$20-3] to unsigned char dor = [S+$20-$12] (used reg = )
mov	al,-$10[bp]
or	al,-1[bp]
mov	-$10[bp],al
!BCC_EOS
! 4641       outb(0x03f2, dor);
! Debug: list unsigned char dor = [S+$20-$12] (used reg = )
mov	al,-$10[bp]
xor	ah,ah
push	ax
! Debug: list int = const $3F2 (used reg = )
mov	ax,#$3F2
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 4642       write_byte(0x40,0x40, 37);
! Debug: list int = const $25 (used reg = )
mov	ax,*$25
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 4643       val8 = inb(0x3f4);
! Debug: list int = const $3F4 (used reg = )
mov	ax,#$3F4
push	ax
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char val8 = [S+$20-$11] (used reg = )
mov	-$F[bp],al
!BCC_EOS
! 4644       if ( (val8 & 0xf0) != 0x80 )
! Debug: and int = const $F0 to unsigned char val8 = [S+$20-$11] (used reg = )
mov	al,-$F[bp]
and	al,#$F0
! Debug: ne int = const $80 to unsigned char = al+0 (used reg = )
cmp	al,#$80
je  	.66A
.66B:
! 4645         bios_printf((2 | 4 | 1), "int13_diskette:f05: ctrl not ready\n");
! Debug: list * char = .66C+0 (used reg = )
mov	bx,#.66C
push	bx
! Debug: list int = const 7 (used reg = )
mov	ax,*7
push	ax
! Debug: func () void = bios_printf+0 (used reg = )
call	_bios_printf
add	sp,*4
!BCC_EOS
! 4646       outb(0x03f5, 0x4d);
.66A:
! Debug: list int = const $4D (used reg = )
mov	ax,*$4D
push	ax
! Debug: list int = const $3F5 (used reg = )
mov	ax,#$3F5
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 4647       outb(0x03f5, (head << 2) | drive);
! Debug: sl int = const 2 to unsigned char head = [S+$20-7] (used reg = )
mov	al,-5[bp]
xor	ah,ah
shl	ax,*1
shl	ax,*1
! Debug: or unsigned char drive = [S+$20-3] to unsigned int = ax+0 (used reg = )
or	al,-1[bp]
! Debug: list unsigned int = ax+0 (used reg = )
push	ax
! Debug: list int = const $3F5 (used reg = )
mov	ax,#$3F5
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 4648       outb(0x03f5, 2);
! Debug: list int = const 2 (used reg = )
mov	ax,*2
push	ax
! Debug: list int = const $3F5 (used reg = )
mov	ax,#$3F5
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 4649       outb(0x03f5, num_sectors);
! Debug: list unsigned char num_sectors = [S+$20-4] (used reg = )
mov	al,-2[bp]
xor	ah,ah
push	ax
! Debug: list int = const $3F5 (used reg = )
mov	ax,#$3F5
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 4650       outb(0x03f5, 0);
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: list int = const $3F5 (used reg = )
mov	ax,#$3F5
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 4651       outb(0x03f5, 0xf6);
! Debug: list int = const $F6 (used reg = )
mov	ax,#$F6
push	ax
! Debug: list int = const $3F5 (used reg = )
mov	ax,#$3F5
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 4652 #asm
!BCC_EOS
!BCC_ASM
_int13_diskette_function.BP	set	$2A
.int13_diskette_function.BP	set	$C
_int13_diskette_function.CS	set	$38
.int13_diskette_function.CS	set	$1A
_int13_diskette_function.CX	set	$32
.int13_diskette_function.CX	set	$14
_int13_diskette_function.base_address	set	$16
.int13_diskette_function.base_address	set	-8
_int13_diskette_function.DI	set	$26
.int13_diskette_function.DI	set	8
_int13_diskette_function.FLAGS	set	$3A
.int13_diskette_function.FLAGS	set	$1C
_int13_diskette_function.base_count	set	$14
.int13_diskette_function.base_count	set	-$A
_int13_diskette_function.sector	set	$1A
.int13_diskette_function.sector	set	-4
_int13_diskette_function.DS	set	$22
.int13_diskette_function.DS	set	4
_int13_diskette_function.head	set	$19
.int13_diskette_function.head	set	-5
_int13_diskette_function.ELDX	set	$2C
.int13_diskette_function.ELDX	set	$E
_int13_diskette_function.dor	set	$E
.int13_diskette_function.dor	set	-$10
_int13_diskette_function.DX	set	$30
.int13_diskette_function.DX	set	$12
_int13_diskette_function.return_status	set	7
.int13_diskette_function.return_status	set	-$17
_int13_diskette_function.es	set	2
.int13_diskette_function.es	set	-$1C
_int13_diskette_function.mode_register	set	$10
.int13_diskette_function.mode_register	set	-$E
_int13_diskette_function.ES	set	$24
.int13_diskette_function.ES	set	6
_int13_diskette_function.base_es	set	$12
.int13_diskette_function.base_es	set	-$C
_int13_diskette_function.track	set	$1B
.int13_diskette_function.track	set	-3
_int13_diskette_function.SI	set	$28
.int13_diskette_function.SI	set	$A
_int13_diskette_function.drive_type	set	6
.int13_diskette_function.drive_type	set	-$18
_int13_diskette_function.num_sectors	set	$1C
.int13_diskette_function.num_sectors	set	-2
_int13_diskette_function.IP	set	$36
.int13_diskette_function.IP	set	$18
_int13_diskette_function.status	set	$18
.int13_diskette_function.status	set	-6
_int13_diskette_function.AX	set	$34
.int13_diskette_function.AX	set	$16
_int13_diskette_function.val8	set	$F
.int13_diskette_function.val8	set	-$F
_int13_diskette_function.last_addr	set	0
.int13_diskette_function.last_addr	set	-$1E
_int13_diskette_function.page	set	$11
.int13_diskette_function.page	set	-$D
_int13_diskette_function.ah	set	4
.int13_diskette_function.ah	set	-$1A
_int13_diskette_function.drive	set	$1D
.int13_diskette_function.drive	set	-1
_int13_diskette_function.num_floppies	set	5
.int13_diskette_function.num_floppies	set	-$19
_int13_diskette_function.BX	set	$2E
.int13_diskette_function.BX	set	$10
      sti
! 4654 endasm
!BCC_ENDASM
!BCC_EOS
! 4655       val8 = (read_byte(0x0000, 0x043e) & 0x80);
! Debug: list int = const $43E (used reg = )
mov	ax,#$43E
push	ax
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: func () unsigned char = read_byte+0 (used reg = )
call	_read_byte
add	sp,*4
! Debug: and int = const $80 to unsigned char = al+0 (used reg = )
and	al,#$80
! Debug: eq unsigned char = al+0 to unsigned char val8 = [S+$20-$11] (used reg = )
mov	-$F[bp],al
!BCC_EOS
! 4656       while ( val8 == 0 ) {
jmp .66E
.66F:
! 4657         val8 = (read_byte(0x0000, 0x043e) & 0x80);
! Debug: list int = const $43E (used reg = )
mov	ax,#$43E
push	ax
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: func () unsigned char = read_byte+0 (used reg = )
call	_read_byte
add	sp,*4
! Debug: and int = const $80 to unsigned char = al+0 (used reg = )
and	al,#$80
! Debug: eq unsigned char = al+0 to unsigned char val8 = [S+$20-$11] (used reg = )
mov	-$F[bp],al
!BCC_EOS
! 4658         }
! 4659      val8 = 0;
.66E:
! Debug: logeq int = const 0 to unsigned char val8 = [S+$20-$11] (used reg = )
mov	al,-$F[bp]
test	al,al
je 	.66F
.670:
.66D:
! Debug: eq int = const 0 to unsigned char val8 = [S+$20-$11] (used reg = )
xor	al,al
mov	-$F[bp],al
!BCC_EOS
! 4660 #asm
!BCC_EOS
!BCC_ASM
_int13_diskette_function.BP	set	$2A
.int13_diskette_function.BP	set	$C
_int13_diskette_function.CS	set	$38
.int13_diskette_function.CS	set	$1A
_int13_diskette_function.CX	set	$32
.int13_diskette_function.CX	set	$14
_int13_diskette_function.base_address	set	$16
.int13_diskette_function.base_address	set	-8
_int13_diskette_function.DI	set	$26
.int13_diskette_function.DI	set	8
_int13_diskette_function.FLAGS	set	$3A
.int13_diskette_function.FLAGS	set	$1C
_int13_diskette_function.base_count	set	$14
.int13_diskette_function.base_count	set	-$A
_int13_diskette_function.sector	set	$1A
.int13_diskette_function.sector	set	-4
_int13_diskette_function.DS	set	$22
.int13_diskette_function.DS	set	4
_int13_diskette_function.head	set	$19
.int13_diskette_function.head	set	-5
_int13_diskette_function.ELDX	set	$2C
.int13_diskette_function.ELDX	set	$E
_int13_diskette_function.dor	set	$E
.int13_diskette_function.dor	set	-$10
_int13_diskette_function.DX	set	$30
.int13_diskette_function.DX	set	$12
_int13_diskette_function.return_status	set	7
.int13_diskette_function.return_status	set	-$17
_int13_diskette_function.es	set	2
.int13_diskette_function.es	set	-$1C
_int13_diskette_function.mode_register	set	$10
.int13_diskette_function.mode_register	set	-$E
_int13_diskette_function.ES	set	$24
.int13_diskette_function.ES	set	6
_int13_diskette_function.base_es	set	$12
.int13_diskette_function.base_es	set	-$C
_int13_diskette_function.track	set	$1B
.int13_diskette_function.track	set	-3
_int13_diskette_function.SI	set	$28
.int13_diskette_function.SI	set	$A
_int13_diskette_function.drive_type	set	6
.int13_diskette_function.drive_type	set	-$18
_int13_diskette_function.num_sectors	set	$1C
.int13_diskette_function.num_sectors	set	-2
_int13_diskette_function.IP	set	$36
.int13_diskette_function.IP	set	$18
_int13_diskette_function.status	set	$18
.int13_diskette_function.status	set	-6
_int13_diskette_function.AX	set	$34
.int13_diskette_function.AX	set	$16
_int13_diskette_function.val8	set	$F
.int13_diskette_function.val8	set	-$F
_int13_diskette_function.last_addr	set	0
.int13_diskette_function.last_addr	set	-$1E
_int13_diskette_function.page	set	$11
.int13_diskette_function.page	set	-$D
_int13_diskette_function.ah	set	4
.int13_diskette_function.ah	set	-$1A
_int13_diskette_function.drive	set	$1D
.int13_diskette_function.drive	set	-1
_int13_diskette_function.num_floppies	set	5
.int13_diskette_function.num_floppies	set	-$19
_int13_diskette_function.BX	set	$2E
.int13_diskette_function.BX	set	$10
      cli
! 4662 endasm
!BCC_ENDASM
!BCC_EOS
! 4663       val8 = read_byte(0x0000, 0x043e);
! Debug: list int = const $43E (used reg = )
mov	ax,#$43E
push	ax
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: func () unsigned char = read_byte+0 (used reg = )
call	_read_byte
add	sp,*4
! Debug: eq unsigned char = al+0 to unsigned char val8 = [S+$20-$11] (used reg = )
mov	-$F[bp],al
!BCC_EOS
! 4664       val8 &= 0x7f;
! Debug: andab int = const $7F to unsigned char val8 = [S+$20-$11] (used reg = )
mov	al,-$F[bp]
and	al,*$7F
mov	-$F[bp],al
!BCC_EOS
! 4665       write_byte(0x0000, 0x043e, val8);
! Debug: list unsigned char val8 = [S+$20-$11] (used reg = )
mov	al,-$F[bp]
xor	ah,ah
push	ax
! Debug: list int = const $43E (used reg = )
mov	ax,#$43E
push	ax
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 4666       val8 = inb(0x3f4);
! Debug: list int = const $3F4 (used reg = )
mov	ax,#$3F4
push	ax
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char val8 = [S+$20-$11] (used reg = )
mov	-$F[bp],al
!BCC_EOS
! 4667       if ( (val8 & 0xc0) != 0xc0 )
! Debug: and int = const $C0 to unsigned char val8 = [S+$20-$11] (used reg = )
mov	al,-$F[bp]
and	al,#$C0
! Debug: ne int = const $C0 to unsigned char = al+0 (used reg = )
cmp	al,#$C0
je  	.671
.672:
! 4668         bios_printf((2 | 4 | 1), "int13_diskette: ctrl not ready\n");
! Debug: list * char = .673+0 (used reg = )
mov	bx,#.673
push	bx
! Debug: list int = const 7 (used reg = )
mov	ax,*7
push	ax
! Debug: func () void = bios_printf+0 (used reg = )
call	_bios_printf
add	sp,*4
!BCC_EOS
! 4669       return_status[0] = inb(0x3f5);
.671:
! Debug: list int = const $3F5 (used reg = )
mov	ax,#$3F5
push	ax
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char return_status = [S+$20-$19] (used reg = )
mov	-$17[bp],al
!BCC_EOS
! 4670       return_status[1] = inb(0x3f5);
! Debug: list int = const $3F5 (used reg = )
mov	ax,#$3F5
push	ax
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char return_status = [S+$20-$18] (used reg = )
mov	-$16[bp],al
!BCC_EOS
! 4671       return_status[2] 
! 4671 = inb(0x3f5);
! Debug: list int = const $3F5 (used reg = )
mov	ax,#$3F5
push	ax
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char return_status = [S+$20-$17] (used reg = )
mov	-$15[bp],al
!BCC_EOS
! 4672       return_status[3] = inb(0x3f5);
! Debug: list int = const $3F5 (used reg = )
mov	ax,#$3F5
push	ax
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char return_status = [S+$20-$16] (used reg = )
mov	-$14[bp],al
!BCC_EOS
! 4673       return_status[4] = inb(0x3f5);
! Debug: list int = const $3F5 (used reg = )
mov	ax,#$3F5
push	ax
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char return_status = [S+$20-$15] (used reg = )
mov	-$13[bp],al
!BCC_EOS
! 4674       return_status[5] = inb(0x3f5);
! Debug: list int = const $3F5 (used reg = )
mov	ax,#$3F5
push	ax
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char return_status = [S+$20-$14] (used reg = )
mov	-$12[bp],al
!BCC_EOS
! 4675       return_status[6] = inb(0x3f5);
! Debug: list int = const $3F5 (used reg = )
mov	ax,#$3F5
push	ax
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char return_status = [S+$20-$13] (used reg = )
mov	-$11[bp],al
!BCC_EOS
! 4676       write_byte(0x0040, 0x0042, return_status[0]);
! Debug: list unsigned char return_status = [S+$20-$19] (used reg = )
mov	al,-$17[bp]
xor	ah,ah
push	ax
! Debug: list int = const $42 (used reg = )
mov	ax,*$42
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 4677       write_byte(0x0040, 0x0043, return_status[1]);
! Debug: list unsigned char return_status = [S+$20-$18] (used reg = )
mov	al,-$16[bp]
xor	ah,ah
push	ax
! Debug: list int = const $43 (used reg = )
mov	ax,*$43
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 4678       write_byte(0x0040, 0x0044, return_status[2]);
! Debug: list unsigned char return_status = [S+$20-$17] (used reg = )
mov	al,-$15[bp]
xor	ah,ah
push	ax
! Debug: list int = const $44 (used reg = )
mov	ax,*$44
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 4679       write_byte(0x0040, 0x0045, return_status[3]);
! Debug: list unsigned char return_status = [S+$20-$16] (used reg = )
mov	al,-$14[bp]
xor	ah,ah
push	ax
! Debug: list int = const $45 (used reg = )
mov	ax,*$45
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 4680       write_byte(0x0040, 0x0046, return_status[4]);
! Debug: list unsigned char return_status = [S+$20-$15] (used reg = )
mov	al,-$13[bp]
xor	ah,ah
push	ax
! Debug: list int = const $46 (used reg = )
mov	ax,*$46
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 4681       write_byte(0x0040, 0x0047, return_status[5]);
! Debug: list unsigned char return_status = [S+$20-$14] (used reg = )
mov	al,-$12[bp]
xor	ah,ah
push	ax
! Debug: list int = const $47 (used reg = )
mov	ax,*$47
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 4682       write_byte(0x0040, 0x0048, return_status[6]);
! Debug: list unsigned char return_status = [S+$20-$13] (used reg = )
mov	al,-$11[bp]
xor	ah,ah
push	ax
! Debug: list int = const $48 (used reg = )
mov	ax,*$48
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 4683       if ( (return_status[0] & 0xc0) != 0 ) {
! Debug: and int = const $C0 to unsigned char return_status = [S+$20-$19] (used reg = )
mov	al,-$17[bp]
and	al,#$C0
! Debug: ne int = const 0 to unsigned char = al+0 (used reg = )
test	al,al
je  	.674
.675:
! 4684         if ( (return_status[1] & 0x02) != 0 ) {
! Debug: and int = const 2 to unsigned char return_status = [S+$20-$18] (used reg = )
mov	al,-$16[bp]
and	al,*2
! Debug: ne int = const 0 to unsigned char = al+0 (used reg = )
test	al,al
je  	.676
.677:
! 4685           AX = 0x0300;
! Debug: eq int = const $300 to unsigned short AX = [S+$20+$14] (used reg = )
mov	ax,#$300
mov	$16[bp],ax
!BCC_EOS
! 4686           FLAGS |= 0x0001;
! Debug: orab int = const 1 to unsigned short FLAGS = [S+$20+$1A] (used reg = )
mov	ax,$1C[bp]
or	al,*1
mov	$1C[bp],ax
!BCC_EOS
! 4687           return;
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 4688         } else {
jmp .678
.676:
! 4689           bios_printf((2 | 4 | 1), "int13_diskette_function: write error\n");
! Debug: list * char = .679+0 (used reg = )
mov	bx,#.679
push	bx
! Debug: list int = const 7 (used reg = )
mov	ax,*7
push	ax
! Debug: func () void = bios_printf+0 (used reg = )
call	_bios_printf
add	sp,*4
!BCC_EOS
! 4690         }
! 4691       }
.678:
! 4692       AX = ((AX & 0x00ff) | ((0) << 8));
.674:
! Debug: and int = const $FF to unsigned short AX = [S+$20+$14] (used reg = )
mov	al,$16[bp]
! Debug: or int = const 0 to unsigned char = al+0 (used reg = )
or	al,*0
! Debug: eq unsigned char = al+0 to unsigned short AX = [S+$20+$14] (used reg = )
xor	ah,ah
mov	$16[bp],ax
!BCC_EOS
! 4693       set_diskette_ret_status(0);
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: func () void = set_diskette_ret_status+0 (used reg = )
call	_set_diskette_ret_status
inc	sp
inc	sp
!BCC_EOS
! 4694       set_diskette_current_cyl(drive, 0);
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: list unsigned char drive = [S+$22-3] (used reg = )
mov	al,-1[bp]
xor	ah,ah
push	ax
! Debug: func () void = set_diskette_current_cyl+0 (used reg = )
call	_set_diskette_current_cyl
add	sp,*4
!BCC_EOS
! 4695       FLAGS &= 0xfffe;
! Debug: andab unsigned int = const $FFFE to unsigned short FLAGS = [S+$20+$1A] (used reg = )
mov	ax,$1C[bp]
and	al,#$FE
mov	$1C[bp],ax
!BCC_EOS
! 4696       return;
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 4697     case 0x08:
! 4698 ;
.67A:
!BCC_EOS
! 4699       drive = ( ELDX & 0x00ff );
! Debug: and int = const $FF to unsigned short ELDX = [S+$20+$C] (used reg = )
mov	al,$E[bp]
! Debug: eq unsigned char = al+0 to unsigned char drive = [S+$20-3] (used reg = )
mov	-1[bp],al
!BCC_EOS
! 4700       if (drive > 1) {
! Debug: gt int = const 1 to unsigned char drive = [S+$20-3] (used reg = )
mov	al,-1[bp]
cmp	al,*1
jbe 	.67B
.67C:
! 4701         AX = 0;
! Debug: eq int = const 0 to unsigned short AX = [S+$20+$14] (used reg = )
xor	ax,ax
mov	$16[bp],ax
!BCC_EOS
! 4702         BX = 0;
! Debug: eq int = const 0 to unsigned short BX = [S+$20+$E] (used reg = )
xor	ax,ax
mov	$10[bp],ax
!BCC_EOS
! 4703         CX = 0;
! Debug: eq int = const 0 to unsigned short CX = [S+$20+$12] (used reg = )
xor	ax,ax
mov	$14[bp],ax
!BCC_EOS
! 4704         DX = 0;
! Debug: eq int = const 0 to unsigned short DX = [S+$20+$10] (used reg = )
xor	ax,ax
mov	$12[bp],ax
!BCC_EOS
! 4705         ES = 0;
! Debug: eq int = const 0 to unsigned short ES = [S+$20+4] (used reg = )
xor	ax,ax
mov	6[bp],ax
!BCC_EOS
! 4706         DI = 0;
! Debug: eq int = const 0 to unsigned short DI = [S+$20+6] (used reg = )
xor	ax,ax
mov	8[bp],ax
!BCC_EOS
! 4707         DX = ((DX & 0xff00) | (num_floppies));
! Debug: and unsigned int = const $FF00 to unsigned short DX = [S+$20+$10] (used reg = )
mov	ax,$12[bp]
xor	al,al
! Debug: or unsigned char num_floppies = [S+$20-$1B] to unsigned int = ax+0 (used reg = )
or	al,-$19[bp]
! Debug: eq unsigned int = ax+0 to unsigned short DX = [S+$20+$10] (used reg = )
mov	$12[bp],ax
!BCC_EOS
! 4708         FLAGS |= 0x0001;
! Debug: orab int = const 1 to unsigned short FLAGS = [S+$20+$1A] (used reg = )
mov	ax,$1C[bp]
or	al,*1
mov	$1C[bp],ax
!BCC_EOS
! 4709         return;
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 4710         }
! 4711       drive_type = inb_cmos(0x10);
.67B:
! Debug: list int = const $10 (used reg = )
mov	ax,*$10
push	ax
! Debug: func () unsigned char = inb_cmos+0 (used reg = )
call	_inb_cmos
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char drive_type = [S+$20-$1A] (used reg = )
mov	-$18[bp],al
!BCC_EOS
! 4712       num_floppies = 0;
! Debug: eq int = const 0 to unsigned char num_floppies = [S+$20-$1B] (used reg = )
xor	al,al
mov	-$19[bp],al
!BCC_EOS
! 4713       if (drive_type & 0xf0)
! Debug: and int = const $F0 to unsigned char drive_type = [S+$20-$1A] (used reg = )
mov	al,-$18[bp]
and	al,#$F0
test	al,al
je  	.67D
.67E:
! 4714         num_floppies++;
! Debug: postinc unsigned char num_floppies = [S+$20-$1B] (used reg = )
mov	al,-$19[bp]
inc	ax
mov	-$19[bp],al
!BCC_EOS
! 4715       if (drive_type & 0x0f)
.67D:
! Debug: and int = const $F to unsigned char drive_type = [S+$20-$1A] (used reg = )
mov	al,-$18[bp]
and	al,*$F
test	al,al
je  	.67F
.680:
! 4716         num_floppies++;
! Debug: postinc unsigned char num_floppies = [S+$20-$1B] (used reg = )
mov	al,-$19[bp]
inc	ax
mov	-$19[bp],al
!BCC_EOS
! 4717       if (drive == 0)
.67F:
! Debug: logeq int = const 0 to unsigned char drive = [S+$20-3] (used reg = )
mov	al,-1[bp]
test	al,al
jne 	.681
.682:
! 4718         drive_type >>= 4;
! Debug: srab int = const 4 to unsigned char drive_type = [S+$20-$1A] (used reg = )
mov	al,-$18[bp]
xor	ah,ah
mov	cl,*4
shr	ax,cl
mov	-$18[bp],al
!BCC_EOS
! 4719       else
! 4720         drive_type &= 0x0f;
jmp .683
.681:
! Debug: andab int = const $F to unsigned char drive_type = [S+$20-$1A] (used reg = )
mov	al,-$18[bp]
and	al,*$F
mov	-$18[bp],al
!BCC_EOS
! 4721       BX = ((BX & 0x00ff) | ((0) << 8));
.683:
! Debug: and int = const $FF to unsigned short BX = [S+$20+$E] (used reg = )
mov	al,$10[bp]
! Debug: or int = const 0 to unsigned char = al+0 (used reg = )
or	al,*0
! Debug: eq unsigned char = al+0 to unsigned short BX = [S+$20+$E] (used reg = )
xor	ah,ah
mov	$10[bp],ax
!BCC_EOS
! 4722       BX = ((BX & 0xff00) | (drive_type));
! Debug: and unsigned int = const $FF00 to unsigned short BX = [S+$20+$E] (used reg = )
mov	ax,$10[bp]
xor	al,al
! Debug: or unsigned char drive_type = [S+$20-$1A] to unsigned int = ax+0 (used reg = )
or	al,-$18[bp]
! Debug: eq unsigned int = ax+0 to unsigned short BX = [S+$20+$E] (used reg = )
mov	$10[bp],ax
!BCC_EOS
! 4723       AX = ((AX & 0x00ff) | ((0) << 8));
! Debug: and int = const $FF to unsigned short AX = [S+$20+$14] (used reg = )
mov	al,$16[bp]
! Debug: or int = const 0 to unsigned char = al+0 (used reg = )
or	al,*0
! Debug: eq unsigned char = al+0 to unsigned short AX = [S+$20+$14] (used reg = )
xor	ah,ah
mov	$16[bp],ax
!BCC_EOS
! 4724       AX = ((AX & 0xff00) | (0));
! Debug: and unsigned int = const $FF00 to unsigned short AX = [S+$20+$14] (used reg = )
mov	ax,$16[bp]
xor	al,al
! Debug: or int = const 0 to unsigned int = ax+0 (used reg = )
or	al,*0
! Debug: eq unsigned int = ax+0 to unsigned short AX = [S+$20+$14] (used reg = )
mov	$16[bp],ax
!BCC_EOS
! 4725       DX = ((DX & 0xff00) | (num_floppies));
! Debug: and unsigned int = const $FF00 to unsigned short DX = [S+$20+$10] (used reg = )
mov	ax,$12[bp]
xor	al,al
! Debug: or unsigned char num_floppies = [S+$20-$1B] to unsigned int = ax+0 (used reg = )
or	al,-$19[bp]
! Debug: eq unsigned int = ax+0 to unsigned short DX = [S+$20+$10] (used reg = )
mov	$12[bp],ax
!BCC_EOS
! 4726       switch (drive_type) {
mov	al,-$18[bp]
br 	.686
! 4727         case 0:
! 4728           CX = 0;
.687:
! Debug: eq int = const 0 to unsigned short CX = [S+$20+$12] (used reg = )
xor	ax,ax
mov	$14[bp],ax
!BCC_EOS
! 4729           DX = ((DX & 0x00ff) | ((0) << 8));
! Debug: and int = const $FF to unsigned short DX = [S+$20+$10] (used reg = )
mov	al,$12[bp]
! Debug: or int = const 0 to unsigned char = al+0 (used reg = )
or	al,*0
! Debug: eq unsigned char = al+0 to unsigned short DX = [S+$20+$10] (used reg = )
xor	ah,ah
mov	$12[bp],ax
!BCC_EOS
! 4730           break;
br 	.684
!BCC_EOS
! 4731         case 1:
! 4732           CX = 0x2709;
.688:
! Debug: eq int = const $2709 to unsigned short CX = [S+$20+$12] (used reg = )
mov	ax,#$2709
mov	$14[bp],ax
!BCC_EOS
! 4733           DX = ((DX & 0x00ff) | ((1) << 8));
! Debug: and int = const $FF to unsigned short DX = [S+$20+$10] (used reg = )
mov	al,$12[bp]
! Debug: or int = const $100 to unsigned char = al+0 (used reg = )
xor	ah,ah
or	ax,#$100
! Debug: eq unsigned int = ax+0 to unsigned short DX = [S+$20+$10] (used reg = )
mov	$12[bp],ax
!BCC_EOS
! 4734           break;
br 	.684
!BCC_EOS
! 4735         case 2:
! 4736           CX = 0x4f0f;
.689:
! Debug: eq int = const $4F0F to unsigned short CX = [S+$20+$12] (used reg = )
mov	ax,#$4F0F
mov	$14[bp],ax
!BCC_EOS
! 4737           DX = ((DX & 0x00ff) | ((1) << 8));
! Debug: and int = const $FF to unsigned short DX = [S+$20+$10] (used reg = )
mov	al,$12[bp]
! Debug: or int = const $100 to unsigned char = al+0 (used reg = )
xor	ah,ah
or	ax,#$100
! Debug: eq unsigned int = ax+0 to unsigned short DX = [S+$20+$10] (used reg = )
mov	$12[bp],ax
!BCC_EOS
! 4738           break;
br 	.684
!BCC_EOS
! 4739         case 3:
! 4740           CX = 0x4f09;
.68A:
! Debug: eq int = const $4F09 to unsigned short CX = [S+$20+$12] (used reg = )
mov	ax,#$4F09
mov	$14[bp],ax
!BCC_EOS
! 4741      
! 4741      DX = ((DX & 0x00ff) | ((1) << 8));
! Debug: and int = const $FF to unsigned short DX = [S+$20+$10] (used reg = )
mov	al,$12[bp]
! Debug: or int = const $100 to unsigned char = al+0 (used reg = )
xor	ah,ah
or	ax,#$100
! Debug: eq unsigned int = ax+0 to unsigned short DX = [S+$20+$10] (used reg = )
mov	$12[bp],ax
!BCC_EOS
! 4742           break;
br 	.684
!BCC_EOS
! 4743         case 4:
! 4744           CX = 0x4f12;
.68B:
! Debug: eq int = const $4F12 to unsigned short CX = [S+$20+$12] (used reg = )
mov	ax,#$4F12
mov	$14[bp],ax
!BCC_EOS
! 4745           DX = ((DX & 0x00ff) | ((1) << 8));
! Debug: and int = const $FF to unsigned short DX = [S+$20+$10] (used reg = )
mov	al,$12[bp]
! Debug: or int = const $100 to unsigned char = al+0 (used reg = )
xor	ah,ah
or	ax,#$100
! Debug: eq unsigned int = ax+0 to unsigned short DX = [S+$20+$10] (used reg = )
mov	$12[bp],ax
!BCC_EOS
! 4746           break;
br 	.684
!BCC_EOS
! 4747         case 5:
! 4748           CX = 0x4f24;
.68C:
! Debug: eq int = const $4F24 to unsigned short CX = [S+$20+$12] (used reg = )
mov	ax,#$4F24
mov	$14[bp],ax
!BCC_EOS
! 4749           DX = ((DX & 0x00ff) | ((1) << 8));
! Debug: and int = const $FF to unsigned short DX = [S+$20+$10] (used reg = )
mov	al,$12[bp]
! Debug: or int = const $100 to unsigned char = al+0 (used reg = )
xor	ah,ah
or	ax,#$100
! Debug: eq unsigned int = ax+0 to unsigned short DX = [S+$20+$10] (used reg = )
mov	$12[bp],ax
!BCC_EOS
! 4750           break;
jmp .684
!BCC_EOS
! 4751         case 6:
! 4752           CX = 0x2708;
.68D:
! Debug: eq int = const $2708 to unsigned short CX = [S+$20+$12] (used reg = )
mov	ax,#$2708
mov	$14[bp],ax
!BCC_EOS
! 4753           DX = ((DX & 0x00ff) | ((0) << 8));
! Debug: and int = const $FF to unsigned short DX = [S+$20+$10] (used reg = )
mov	al,$12[bp]
! Debug: or int = const 0 to unsigned char = al+0 (used reg = )
or	al,*0
! Debug: eq unsigned char = al+0 to unsigned short DX = [S+$20+$10] (used reg = )
xor	ah,ah
mov	$12[bp],ax
!BCC_EOS
! 4754           break;
jmp .684
!BCC_EOS
! 4755         case 7:
! 4756           CX = 0x2709;
.68E:
! Debug: eq int = const $2709 to unsigned short CX = [S+$20+$12] (used reg = )
mov	ax,#$2709
mov	$14[bp],ax
!BCC_EOS
! 4757           DX = ((DX & 0x00ff) | ((0) << 8));
! Debug: and int = const $FF to unsigned short DX = [S+$20+$10] (used reg = )
mov	al,$12[bp]
! Debug: or int = const 0 to unsigned char = al+0 (used reg = )
or	al,*0
! Debug: eq unsigned char = al+0 to unsigned short DX = [S+$20+$10] (used reg = )
xor	ah,ah
mov	$12[bp],ax
!BCC_EOS
! 4758           break;
jmp .684
!BCC_EOS
! 4759         case 8:
! 4760           CX = 0x2708;
.68F:
! Debug: eq int = const $2708 to unsigned short CX = [S+$20+$12] (used reg = )
mov	ax,#$2708
mov	$14[bp],ax
!BCC_EOS
! 4761           DX = ((DX & 0x00ff) | ((1) << 8));
! Debug: and int = const $FF to unsigned short DX = [S+$20+$10] (used reg = )
mov	al,$12[bp]
! Debug: or int = const $100 to unsigned char = al+0 (used reg = )
xor	ah,ah
or	ax,#$100
! Debug: eq unsigned int = ax+0 to unsigned short DX = [S+$20+$10] (used reg = )
mov	$12[bp],ax
!BCC_EOS
! 4762           break;
jmp .684
!BCC_EOS
! 4763         default:
! 4764           bios_printf((2 | 4 | 1), "floppy: int13: bad floppy type\n");
.690:
! Debug: list * char = .691+0 (used reg = )
mov	bx,#.691
push	bx
! Debug: list int = const 7 (used reg = )
mov	ax,*7
push	ax
! Debug: func () void = bios_printf+0 (used reg = )
call	_bios_printf
add	sp,*4
!BCC_EOS
! 4765         }
! 4766 #asm
jmp .684
.686:
sub	al,*0
jb 	.690
cmp	al,*8
ja  	.692
xor	ah,ah
shl	ax,*1
mov	bx,ax
seg	cs
br	.693[bx]
.693:
.word	.687
.word	.688
.word	.689
.word	.68A
.word	.68B
.word	.68C
.word	.68D
.word	.68E
.word	.68F
.692:
jmp	.690
.684:
!BCC_EOS
!BCC_ASM
_int13_diskette_function.BP	set	$2A
.int13_diskette_function.BP	set	$C
_int13_diskette_function.CS	set	$38
.int13_diskette_function.CS	set	$1A
_int13_diskette_function.CX	set	$32
.int13_diskette_function.CX	set	$14
_int13_diskette_function.base_address	set	$16
.int13_diskette_function.base_address	set	-8
_int13_diskette_function.DI	set	$26
.int13_diskette_function.DI	set	8
_int13_diskette_function.FLAGS	set	$3A
.int13_diskette_function.FLAGS	set	$1C
_int13_diskette_function.base_count	set	$14
.int13_diskette_function.base_count	set	-$A
_int13_diskette_function.sector	set	$1A
.int13_diskette_function.sector	set	-4
_int13_diskette_function.DS	set	$22
.int13_diskette_function.DS	set	4
_int13_diskette_function.head	set	$19
.int13_diskette_function.head	set	-5
_int13_diskette_function.ELDX	set	$2C
.int13_diskette_function.ELDX	set	$E
_int13_diskette_function.dor	set	$E
.int13_diskette_function.dor	set	-$10
_int13_diskette_function.DX	set	$30
.int13_diskette_function.DX	set	$12
_int13_diskette_function.return_status	set	7
.int13_diskette_function.return_status	set	-$17
_int13_diskette_function.es	set	2
.int13_diskette_function.es	set	-$1C
_int13_diskette_function.mode_register	set	$10
.int13_diskette_function.mode_register	set	-$E
_int13_diskette_function.ES	set	$24
.int13_diskette_function.ES	set	6
_int13_diskette_function.base_es	set	$12
.int13_diskette_function.base_es	set	-$C
_int13_diskette_function.track	set	$1B
.int13_diskette_function.track	set	-3
_int13_diskette_function.SI	set	$28
.int13_diskette_function.SI	set	$A
_int13_diskette_function.drive_type	set	6
.int13_diskette_function.drive_type	set	-$18
_int13_diskette_function.num_sectors	set	$1C
.int13_diskette_function.num_sectors	set	-2
_int13_diskette_function.IP	set	$36
.int13_diskette_function.IP	set	$18
_int13_diskette_function.status	set	$18
.int13_diskette_function.status	set	-6
_int13_diskette_function.AX	set	$34
.int13_diskette_function.AX	set	$16
_int13_diskette_function.val8	set	$F
.int13_diskette_function.val8	set	-$F
_int13_diskette_function.last_addr	set	0
.int13_diskette_function.last_addr	set	-$1E
_int13_diskette_function.page	set	$11
.int13_diskette_function.page	set	-$D
_int13_diskette_function.ah	set	4
.int13_diskette_function.ah	set	-$1A
_int13_diskette_function.drive	set	$1D
.int13_diskette_function.drive	set	-1
_int13_diskette_function.num_floppies	set	5
.int13_diskette_function.num_floppies	set	-$19
_int13_diskette_function.BX	set	$2E
.int13_diskette_function.BX	set	$10
      push bp
      mov bp, sp
      mov ax, #diskette_param_table2
      mov _int13_diskette_function.DI+2[bp], ax
      mov _int13_diskette_function.ES+2[bp], cs
      pop bp
! 4773 endasm
!BCC_ENDASM
!BCC_EOS
! 4774       FLAGS &= 0xfffe;
! Debug: andab unsigned int = const $FFFE to unsigned short FLAGS = [S+$20+$1A] (used reg = )
mov	ax,$1C[bp]
and	al,#$FE
mov	$1C[bp],ax
!BCC_EOS
! 4775       return;
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 4776     case 0x15:
! 4777 ;
.694:
!BCC_EOS
! 4778       drive = ( ELDX & 0x00ff );
! Debug: and int = const $FF to unsigned short ELDX = [S+$20+$C] (used reg = )
mov	al,$E[bp]
! Debug: eq unsigned char = al+0 to unsigned char drive = [S+$20-3] (used reg = )
mov	-1[bp],al
!BCC_EOS
! 4779       if (drive > 1) {
! Debug: gt int = const 1 to unsigned char drive = [S+$20-3] (used reg = )
mov	al,-1[bp]
cmp	al,*1
jbe 	.695
.696:
! 4780         AX = ((AX & 0x00ff) | ((0) << 8));
! Debug: and int = const $FF to unsigned short AX = [S+$20+$14] (used reg = )
mov	al,$16[bp]
! Debug: or int = const 0 to unsigned char = al+0 (used reg = )
or	al,*0
! Debug: eq unsigned char = al+0 to unsigned short AX = [S+$20+$14] (used reg = )
xor	ah,ah
mov	$16[bp],ax
!BCC_EOS
! 4781         FLAGS |= 0x0001;
! Debug: orab int = const 1 to unsigned short FLAGS = [S+$20+$1A] (used reg = )
mov	ax,$1C[bp]
or	al,*1
mov	$1C[bp],ax
!BCC_EOS
! 4782         return;
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 4783         }
! 4784       drive_type = inb_cmos(0x10);
.695:
! Debug: list int = const $10 (used reg = )
mov	ax,*$10
push	ax
! Debug: func () unsigned char = inb_cmos+0 (used reg = )
call	_inb_cmos
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char drive_type = [S+$20-$1A] (used reg = )
mov	-$18[bp],al
!BCC_EOS
! 4785       if (drive == 0)
! Debug: logeq int = const 0 to unsigned char drive = [S+$20-3] (used reg = )
mov	al,-1[bp]
test	al,al
jne 	.697
.698:
! 4786         drive_type >>= 4;
! Debug: srab int = const 4 to unsigned char drive_type = [S+$20-$1A] (used reg = )
mov	al,-$18[bp]
xor	ah,ah
mov	cl,*4
shr	ax,cl
mov	-$18[bp],al
!BCC_EOS
! 4787       else
! 4788         drive_type &= 0x0f;
jmp .699
.697:
! Debug: andab int = const $F to unsigned char drive_type = [S+$20-$1A] (used reg = )
mov	al,-$18[bp]
and	al,*$F
mov	-$18[bp],al
!BCC_EOS
! 4789       FLAGS &= 0xfffe;
.699:
! Debug: andab unsigned int = const $FFFE to unsigned short FLAGS = [S+$20+$1A] (used reg = )
mov	ax,$1C[bp]
and	al,#$FE
mov	$1C[bp],ax
!BCC_EOS
! 4790       if (drive_type==0) {
! Debug: logeq int = const 0 to unsigned char drive_type = [S+$20-$1A] (used reg = )
mov	al,-$18[bp]
test	al,al
jne 	.69A
.69B:
! 4791         AX = ((AX & 0x00ff) | ((0) << 8));
! Debug: and int = const $FF to unsigned short AX = [S+$20+$14] (used reg = )
mov	al,$16[bp]
! Debug: or int = const 0 to unsigned char = al+0 (used reg = )
or	al,*0
! Debug: eq unsigned char = al+0 to unsigned short AX = [S+$20+$14] (used reg = )
xor	ah,ah
mov	$16[bp],ax
!BCC_EOS
! 4792         }
! 4793       else {
jmp .69C
.69A:
! 4794         AX = ((AX & 0x00ff) | ((1) << 8));
! Debug: and int = const $FF to unsigned short AX = [S+$20+$14] (used reg = )
mov	al,$16[bp]
! Debug: or int = const $100 to unsigned char = al+0 (used reg = )
xor	ah,ah
or	ax,#$100
! Debug: eq unsigned int = ax+0 to unsigned short AX = [S+$20+$14] (used reg = )
mov	$16[bp],ax
!BCC_EOS
! 4795         }
! 4796       return;
.69C:
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 4797     case 0x16:
! 4798 ;
.69D:
!BCC_EOS
! 4799       drive = ( ELDX & 0x00ff );
! Debug: and int = const $FF to unsigned short ELDX = [S+$20+$C] (used reg = )
mov	al,$E[bp]
! Debug: eq unsigned char = al+0 to unsigned char drive = [S+$20-3] (used reg = )
mov	-1[bp],al
!BCC_EOS
! 4800       if (drive > 1) {
! Debug: gt int = const 1 to unsigned char drive = [S+$20-3] (used reg = )
mov	al,-1[bp]
cmp	al,*1
jbe 	.69E
.69F:
! 4801         AX = ((AX & 0x00ff) | ((0x01) << 8));
! Debug: and int = const $FF to unsigned short AX = [S+$20+$14] (used reg = )
mov	al,$16[bp]
! Debug: or int = const $100 to unsigned char = al+0 (used reg = )
xor	ah,ah
or	ax,#$100
! Debug: eq unsigned int = ax+0 to unsigned short AX = [S+$20+$14] (used reg = )
mov	$16[bp],ax
!BCC_EOS
! 4802         set_diskette_ret_status(0x01);
! Debug: list int = const 1 (used reg = )
mov	ax,*1
push	ax
! Debug: func () void = set_diskette_ret_status+0 (used reg = )
call	_set_diskette_ret_status
inc	sp
inc	sp
!BCC_EOS
! 4803         FLAGS |= 0x0001;
! Debug: orab int = const 1 to unsigned short FLAGS = [S+$20+$1A] (used reg = )
mov	ax,$1C[bp]
or	al,*1
mov	$1C[bp],ax
!BCC_EOS
! 4804         return;
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 4805         }
! 4806       AX = ((AX & 0x00ff) | ((0x06) << 8));
.69E:
! Debug: and int = const $FF to unsigned short AX = [S+$20+$14] (used reg = )
mov	al,$16[bp]
! Debug: or int = const $600 to unsigned char = al+0 (used reg = )
xor	ah,ah
or	ax,#$600
! Debug: eq unsigned int = ax+0 to unsigned short AX = [S+$20+$14] (used reg = )
mov	$16[bp],ax
!BCC_EOS
! 4807       set_diskette_ret_status(0x06);
! Debug: list int = const 6 (used reg = )
mov	ax,*6
push	ax
! Debug: func () void = set_diskette_ret_status+0 (used reg = )
call	_set_diskette_ret_status
inc	sp
inc	sp
!BCC_EOS
! 4808       FLAGS |= 0x0001;
! Debug: orab int = const 1 to unsigned short FLAGS = [S+$20+$1A] (used reg = )
mov	ax,$1C[bp]
or	al,*1
mov	$1C[bp],ax
!BCC_EOS
! 4809       return;
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 4810     case 0x17:
! 4811 ;
.6A0:
!BCC_EOS
! 4812       AX = ((AX & 0x00ff) | ((0x01) << 8));
! Debug: and int = const $FF to unsigned short AX = [S+$20+$14] (used reg = )
mov	al,$16[bp]
! Debug: or int = const $100 to unsigned char = al+0 (used reg = )
xor	ah,ah
or	ax,#$100
! Debug: eq unsigned int = ax+0 to unsigned short AX = [S+$20+$14] (used reg = )
mov	$16[bp],ax
!BCC_EOS
! 4813       set_diskette_ret_status(1);
! Debug: list int = const 1 (used reg = )
mov	ax,*1
push	ax
! Debug: func () void = set_diskette_ret_status+0 (used reg = )
call	_set_diskette_ret_status
inc	sp
inc	sp
!BCC_EOS
! 4814       FLAGS |= 0x0001;
! Debug: orab int = const 1 to unsigned short FLAGS = [S+$20+$1A] (used reg = )
mov	ax,$1C[bp]
or	al,*1
mov	$1C[bp],ax
!BCC_EOS
! 4815       return;
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 4816     case 0x18:
! 4817 ;
.6A1:
!BCC_EOS
! 4818       AX = ((AX & 0x00ff) | ((0x01) << 8));
! Debug: and int = const $FF to unsigned short AX = [S+$20+$14] (used reg = )
mov	al,$16[bp]
! Debug: or int = const $100 to unsigned char = al+0 (used reg = )
xor	ah,ah
or	ax,#$100
! Debug: eq unsigned int = ax+0 to unsigned short AX = [S+$20+$14] (used reg = )
mov	$16[bp],ax
!BCC_EOS
! 4819       set_diskette_ret_status(1);
! Debug: list int = const 1 (used reg = )
mov	ax,*1
push	ax
! Debug: func () void = set_diskette_ret_status+0 (used reg = )
call	_set_diskette_ret_status
inc	sp
inc	sp
!BCC_EOS
! 4820       FLAGS |= 0x0001;
! Debug: orab int = const 1 to unsigned short FLAGS = [S+$20+$1A] (used reg = )
mov	ax,$1C[bp]
or	al,*1
mov	$1C[bp],ax
!BCC_EOS
! 4821       return;
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 4822     default:
! 4823         bios_printf(4, "int13_diskette: unsupported AH=%02x\n", ( AX >> 8 ));
.6A2:
! Debug: sr int = const 8 to unsigned short AX = [S+$20+$14] (used reg = )
mov	ax,$16[bp]
mov	al,ah
xor	ah,ah
! Debug: list unsigned int = ax+0 (used reg = )
push	ax
! Debug: list * char = .6A3+0 (used reg = )
mov	bx,#.6A3
push	bx
! Debug: list int = const 4 (used reg = )
mov	ax,*4
push	ax
! Debug: func () void = bios_printf+0 (used reg = )
call	_bios_printf
add	sp,*6
!BCC_EOS
! 4824       
! 4824   AX = ((AX & 0x00ff) | ((0x01) << 8));
! Debug: and int = const $FF to unsigned short AX = [S+$20+$14] (used reg = )
mov	al,$16[bp]
! Debug: or int = const $100 to unsigned char = al+0 (used reg = )
xor	ah,ah
or	ax,#$100
! Debug: eq unsigned int = ax+0 to unsigned short AX = [S+$20+$14] (used reg = )
mov	$16[bp],ax
!BCC_EOS
! 4825         set_diskette_ret_status(1);
! Debug: list int = const 1 (used reg = )
mov	ax,*1
push	ax
! Debug: func () void = set_diskette_ret_status+0 (used reg = )
call	_set_diskette_ret_status
inc	sp
inc	sp
!BCC_EOS
! 4826         FLAGS |= 0x0001;
! Debug: orab int = const 1 to unsigned short FLAGS = [S+$20+$1A] (used reg = )
mov	ax,$1C[bp]
or	al,*1
mov	$1C[bp],ax
!BCC_EOS
! 4827         return;
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 4828     }
! 4829 }
jmp .609
.60B:
sub	al,*0
jb 	.6A2
cmp	al,*8
ja  	.6A4
xor	ah,ah
shl	ax,*1
mov	bx,ax
seg	cs
br	.6A5[bx]
.6A5:
.word	.60C
.word	.614
.word	.617
.word	.618
.word	.619
.word	.656
.word	.6A2
.word	.6A2
.word	.67A
.6A4:
sub	al,*$15
beq 	.694
sub	al,*1
beq 	.69D
sub	al,*1
beq 	.6A0
sub	al,*1
beq 	.6A1
jmp	.6A2
.609:
..FFDB	=	-$20
mov	sp,bp
pop	bp
ret
! 4830  void
! Register BX used in function int13_diskette_function
! 4831 set_diskette_ret_status(value)
! 4832   Bit8u value;
export	_set_diskette_ret_status
_set_diskette_ret_status:
!BCC_EOS
! 4833 {
! 4834   write_byte(0x0040, 0x0041, value);
push	bp
mov	bp,sp
! Debug: list unsigned char value = [S+2+2] (used reg = )
mov	al,4[bp]
xor	ah,ah
push	ax
! Debug: list int = const $41 (used reg = )
mov	ax,*$41
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
mov	sp,bp
!BCC_EOS
! 4835 }
pop	bp
ret
! 4836   void
! 4837 set_diskette_current_cyl(drive, cyl)
! 4838   Bit8u drive;
export	_set_diskette_current_cyl
_set_diskette_current_cyl:
!BCC_EOS
! 4839   Bit8u cyl;
!BCC_EOS
! 4840 {
! 4841   if (drive > 1)
push	bp
mov	bp,sp
! Debug: gt int = const 1 to unsigned char drive = [S+2+2] (used reg = )
mov	al,4[bp]
cmp	al,*1
jbe 	.6A6
.6A7:
! 4842     bios_printf((2 | 4 | 1), "set_diskette_current_cyl(): drive > 1\n");
! Debug: list * char = .6A8+0 (used reg = )
mov	bx,#.6A8
push	bx
! Debug: list int = const 7 (used reg = )
mov	ax,*7
push	ax
! Debug: func () void = bios_printf+0 (used reg = )
call	_bios_printf
mov	sp,bp
!BCC_EOS
! 4843   write_byte(0x0040, 0x0094+drive, cyl);
.6A6:
! Debug: list unsigned char cyl = [S+2+4] (used reg = )
mov	al,6[bp]
xor	ah,ah
push	ax
! Debug: add unsigned char drive = [S+4+2] to int = const $94 (used reg = )
! Debug: expression subtree swapping
mov	al,4[bp]
xor	ah,ah
! Debug: list unsigned int = ax+$94 (used reg = )
add	ax,#$94
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
mov	sp,bp
!BCC_EOS
! 4844 }
pop	bp
ret
! 4845   void
! Register BX used in function set_diskette_current_cyl
! 4846 determine_floppy_media(drive)
! 4847   Bit16u drive;
export	_determine_floppy_media
_determine_floppy_media:
!BCC_EOS
! 4848 {
! 4849 }
ret
! 4850   void
! 4851 int17_function(regs, ds, iret_addr)
! 4852   pusha_regs_t regs;
export	_int17_function
_int17_function:
!BCC_EOS
! 4853   Bit16u ds;
!BCC_EOS
! 4854   iret_addr_t iret_addr;
!BCC_EOS
! 4855 {
! 4856   Bit16u addr,timeout;
!BCC_EOS
! 4857   Bit8u val8;
!BCC_EOS
! 4858 #asm
push	bp
mov	bp,sp
add	sp,*-6
!BCC_EOS
!BCC_ASM
_int17_function.ds	set	$1A
.int17_function.ds	set	$14
_int17_function.timeout	set	2
.int17_function.timeout	set	-4
_int17_function.val8	set	1
.int17_function.val8	set	-5
_int17_function.iret_addr	set	$1C
.int17_function.iret_addr	set	$16
_int17_function.addr	set	4
.int17_function.addr	set	-2
_int17_function.regs	set	$A
.int17_function.regs	set	4
  sti
! 4860 endasm
!BCC_ENDASM
!BCC_EOS
! 4861   addr = read_word(0x0040, (regs.u.r16.dx << 1) + 8);
! Debug: sl int = const 1 to unsigned short regs = [S+8+$C] (used reg = )
mov	ax,$E[bp]
shl	ax,*1
! Debug: add int = const 8 to unsigned int = ax+0 (used reg = )
! Debug: list unsigned int = ax+8 (used reg = )
add	ax,*8
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () unsigned short = read_word+0 (used reg = )
call	_read_word
add	sp,*4
! Debug: eq unsigned short = ax+0 to unsigned short addr = [S+8-4] (used reg = )
mov	-2[bp],ax
!BCC_EOS
! 4862   if ((regs.u.r8.ah < 3) && (regs.u.r16.dx < 3) && (addr > 0)) {
! Debug: lt int = const 3 to unsigned char regs = [S+8+$11] (used reg = )
mov	al,$13[bp]
cmp	al,*3
bhis	.6A9
.6AC:
! Debug: lt int = const 3 to unsigned short regs = [S+8+$C] (used reg = )
mov	ax,$E[bp]
cmp	ax,*3
bhis	.6A9
.6AB:
! Debug: gt int = const 0 to unsigned short addr = [S+8-4] (used reg = )
mov	ax,-2[bp]
test	ax,ax
beq 	.6A9
.6AA:
! 4863     timeout = read_byte(0x0040, 0x0078 + regs.u.r16.dx) << 8;
! Debug: add unsigned short regs = [S+8+$C] to int = const $78 (used reg = )
! Debug: expression subtree swapping
mov	ax,$E[bp]
! Debug: list unsigned int = ax+$78 (used reg = )
add	ax,*$78
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () unsigned char = read_byte+0 (used reg = )
call	_read_byte
add	sp,*4
! Debug: sl int = const 8 to unsigned char = al+0 (used reg = )
xor	ah,ah
mov	ah,al
xor	al,al
! Debug: eq unsigned int = ax+0 to unsigned short timeout = [S+8-6] (used reg = )
mov	-4[bp],ax
!BCC_EOS
! 4864     if (regs.u.r8.ah == 0) {
! Debug: logeq int = const 0 to unsigned char regs = [S+8+$11] (used reg = )
mov	al,$13[bp]
test	al,al
jne 	.6AD
.6AE:
! 4865       outb(addr, regs.u.r8.al);
! Debug: list unsigned char regs = [S+8+$10] (used reg = )
mov	al,$12[bp]
xor	ah,ah
push	ax
! Debug: list unsigned short addr = [S+$A-4] (used reg = )
push	-2[bp]
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 4866       val8 = inb(addr+2);
! Debug: add int = const 2 to unsigned short addr = [S+8-4] (used reg = )
mov	ax,-2[bp]
! Debug: list unsigned int = ax+2 (used reg = )
inc	ax
inc	ax
push	ax
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char val8 = [S+8-7] (used reg = )
mov	-5[bp],al
!BCC_EOS
! 4867       outb(addr+2, val8 | 0x01);
! Debug: or int = const 1 to unsigned char val8 = [S+8-7] (used reg = )
mov	al,-5[bp]
or	al,*1
! Debug: list unsigned char = al+0 (used reg = )
xor	ah,ah
push	ax
! Debug: add int = const 2 to unsigned short addr = [S+$A-4] (used reg = )
mov	ax,-2[bp]
! Debug: list unsigned int = ax+2 (used reg = )
inc	ax
inc	ax
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 4868 #asm
!BCC_EOS
!BCC_ASM
_int17_function.ds	set	$1A
.int17_function.ds	set	$14
_int17_function.timeout	set	2
.int17_function.timeout	set	-4
_int17_function.val8	set	1
.int17_function.val8	set	-5
_int17_function.iret_addr	set	$1C
.int17_function.iret_addr	set	$16
_int17_function.addr	set	4
.int17_function.addr	set	-2
_int17_function.regs	set	$A
.int17_function.regs	set	4
      nop
! 4870 endasm
!BCC_ENDASM
!BCC_EOS
! 4871       outb(addr+2, val8 & ~0x01);
! Debug: and int = const -2 to unsigned char val8 = [S+8-7] (used reg = )
mov	al,-5[bp]
and	al,#$FE
! Debug: list unsigned char = al+0 (used reg = )
xor	ah,ah
push	ax
! Debug: add int = const 2 to unsigned short addr = [S+$A-4] (used reg = )
mov	ax,-2[bp]
! Debug: list unsigned int = ax+2 (used reg = )
inc	ax
inc	ax
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 4872       while (((inb(addr+1) & 0x40) == 0x40) && (timeout)) {
jmp .6B0
.6B1:
! 4873         timeout--;
! Debug: postdec unsigned short timeout = [S+8-6] (used reg = )
mov	ax,-4[bp]
dec	ax
mov	-4[bp],ax
!BCC_EOS
! 4874       }
! 4875     }
.6B0:
! Debug: add int = const 1 to unsigned short addr = [S+8-4] (used reg = )
mov	ax,-2[bp]
! Debug: list unsigned int = ax+1 (used reg = )
inc	ax
push	ax
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
inc	sp
inc	sp
! Debug: and int = const $40 to unsigned char = al+0 (used reg = )
and	al,*$40
! Debug: logeq int = const $40 to unsigned char = al+0 (used reg = )
cmp	al,*$40
jne 	.6B2
.6B3:
mov	ax,-4[bp]
test	ax,ax
jne	.6B1
.6B2:
.6AF:
! 4876     if (regs.u.r8.ah == 1) {
.6AD:
! Debug: logeq int = const 1 to unsigned char regs = [S+8+$11] (used reg = )
mov	al,$13[bp]
cmp	al,*1
jne 	.6B4
.6B5:
! 4877       val8 = inb(addr+2);
! Debug: add int = const 2 to unsigned short addr = [S+8-4] (used reg = )
mov	ax,-2[bp]
! Debug: list unsigned int = ax+2 (used reg = )
inc	ax
inc	ax
push	ax
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char val8 = [S+8-7] (used reg = )
mov	-5[bp],al
!BCC_EOS
! 4878       outb(addr+2, val8 & ~0x04);
! Debug: and int = const -5 to unsigned char val8 = [S+8-7] (used reg = )
mov	al,-5[bp]
and	al,#$FB
! Debug: list unsigned char = al+0 (used reg = )
xor	ah,ah
push	ax
! Debug: add int = const 2 to unsigned short addr = [S+$A-4] (used reg = )
mov	ax,-2[bp]
! Debug: list unsigned int = ax+2 (used reg = )
inc	ax
inc	ax
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 4879 #asm
!BCC_EOS
!BCC_ASM
_int17_function.ds	set	$1A
.int17_function.ds	set	$14
_int17_function.timeout	set	2
.int17_function.timeout	set	-4
_int17_function.val8	set	1
.int17_function.val8	set	-5
_int17_function.iret_addr	set	$1C
.int17_function.iret_addr	set	$16
_int17_function.addr	set	4
.int17_function.addr	set	-2
_int17_function.regs	set	$A
.int17_function.regs	set	4
      nop
! 4881 endasm
!BCC_ENDASM
!BCC_EOS
! 4882       outb(addr+2, val8 | 0x04);
! Debug: or int = const 4 to unsigned char val8 = [S+8-7] (used reg = )
mov	al,-5[bp]
or	al,*4
! Debug: list unsigned char = al+0 (used reg = )
xor	ah,ah
push	ax
! Debug: add int = const 2 to unsigned short addr = [S+$A-4] (used reg = )
mov	ax,-2[bp]
! Debug: list unsigned int = ax+2 (used reg = )
inc	ax
inc	ax
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 4883     }
! 4884     val8 = inb(addr+1);
.6B4:
! Debug: add int = const 1 to unsigned short addr = [S+8-4] (used reg = )
mov	ax,-2[bp]
! Debug: list unsigned int = ax+1 (used reg = )
inc	ax
push	ax
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char val8 = [S+8-7] (used reg = )
mov	-5[bp],al
!BCC_EOS
! 4885     regs.u.r8.ah = (val8 ^ 0x48);
! Debug: eor int = const $48 to unsigned char val8 = [S+8-7] (used reg = )
mov	al,-5[bp]
xor	al,*$48
! Debug: eq unsigned char = al+0 to unsigned char regs = [S+8+$11] (used reg = )
mov	$13[bp],al
!BCC_EOS
! 4886     if (!timeout) regs.u.r8.ah |= 0x01;
mov	ax,-4[bp]
test	ax,ax
jne 	.6B6
.6B7:
! Debug: orab int = const 1 to unsigned char regs = [S+8+$11] (used reg = )
mov	al,$13[bp]
or	al,*1
mov	$13[bp],al
!BCC_EOS
! 4887     iret_addr.flags.u.r8.flagsl &= 0xfe;
.6B6:
! Debug: andab int = const $FE to unsigned char iret_addr = [S+8+$18] (used reg = )
mov	al,$1A[bp]
and	al,#$FE
mov	$1A[bp],al
!BCC_EOS
! 4888   } else {
jmp .6B8
.6A9:
! 4889     iret_addr.flags.u.r8.flagsl |= 0x01;
! Debug: orab int = const 1 to unsigned char iret_addr = [S+8+$18] (used reg = )
mov	al,$1A[bp]
or	al,*1
mov	$1A[bp],al
!BCC_EOS
! 4890   }
! 4891 }
.6B8:
mov	sp,bp
pop	bp
ret
! 4892 void
! 4893 int18_function(seq_nr)
! 4894 Bit16u seq_nr;
export	_int18_function
_int18_function:
!BCC_EOS
! 4895 {
! 4896   Bit16u ebda_seg=read_word(0x0040,0x000E);
push	bp
mov	bp,sp
dec	sp
dec	sp
! Debug: list int = const $E (used reg = )
mov	ax,*$E
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () unsigned short = read_word+0 (used reg = )
call	_read_word
add	sp,*4
! Debug: eq unsigned short = ax+0 to unsigned short ebda_seg = [S+4-4] (used reg = )
mov	-2[bp],ax
!BCC_EOS
! 4897   Bit16u bootdev;
!BCC_EOS
! 4898   Bit8u bootdrv;
!BCC_EOS
! 4899   Bit8u bootchk;
!BCC_EOS
! 4900   Bit16u bootseg;
!BCC_EOS
! 4901   Bit16u bootip;
!BCC_EOS
! 4902   Bit16u status;
!BCC_EOS
! 4903   struct ipl_entry e;
!BCC_EOS
! 4904   bootdev = inb_cmos(0x3d);
add	sp,*-$1A
! Debug: list int = const $3D (used reg = )
mov	ax,*$3D
push	ax
! Debug: func () unsigned char = inb_cmos+0 (used reg = )
call	_inb_cmos
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned short bootdev = [S+$1E-6] (used reg = )
xor	ah,ah
mov	-4[bp],ax
!BCC_EOS
! 4905   bootdev |= ((inb_cmos(0x38) & 0xf0) << 4);
! Debug: list int = const $38 (used reg = )
mov	ax,*$38
push	ax
! Debug: func () unsigned char = inb_cmos+0 (used reg = )
call	_inb_cmos
inc	sp
inc	sp
! Debug: and int = const $F0 to unsigned char = al+0 (used reg = )
and	al,#$F0
! Debug: sl int = const 4 to unsigned char = al+0 (used reg = )
xor	ah,ah
mov	cl,*4
shl	ax,cl
! Debug: orab unsigned int = ax+0 to unsigned short bootdev = [S+$1E-6] (used reg = )
or	ax,-4[bp]
mov	-4[bp],ax
!BCC_EOS
! 4906   bootdev >>= 4 * seq_nr;
! Debug: mul unsigned short seq_nr = [S+$1E+2] to int = const 4 (used reg = )
! Debug: expression subtree swapping
mov	ax,4[bp]
shl	ax,*1
shl	ax,*1
! Debug: srab unsigned int = ax+0 to unsigned short bootdev = [S+$1E-6] (used reg = )
mov	bx,ax
mov	ax,-4[bp]
mov	cx,bx
shr	ax,cl
mov	-4[bp],ax
!BCC_EOS
! 4907   bootdev &= 0xf;
! Debug: andab int = const $F to unsigned short bootdev = [S+$1E-6] (used reg = )
mov	al,-4[bp]
and	al,*$F
xor	ah,ah
mov	-4[bp],ax
!BCC_EOS
! 4908   if (bootdev == 0) bios_printf((2 | 4 | 1), "No bootable device.\n");
! Debug: logeq int = const 0 to unsigned short bootdev = [S+$1E-6] (used reg = )
mov	ax,-4[bp]
test	ax,ax
jne 	.6B9
.6BA:
! Debug: list * char = .6BB+0 (used reg = )
mov	bx,#.6BB
push	bx
! Debug: list int = const 7 (used reg = )
mov	ax,*7
push	ax
! Debug: func () void = bios_printf+0 (used reg = )
call	_bios_printf
add	sp,*4
!BCC_EOS
! 4909   bootdev -= 1;
.6B9:
! Debug: subab int = const 1 to unsigned short bootdev = [S+$1E-6] (used reg = )
mov	ax,-4[bp]
dec	ax
mov	-4[bp],ax
!BCC_EOS
! 4910   if (get_boot_vector(bootdev, &e) == 0) {
! Debug: list * struct ipl_entry e = S+$1E-$1E (used reg = )
lea	bx,-$1C[bp]
push	bx
! Debug: list unsigned short bootdev = [S+$20-6] (used reg = )
push	-4[bp]
! Debug: func () unsigned char = get_boot_vector+0 (used reg = )
call	_get_boot_vector
add	sp,*4
! Debug: logeq int = const 0 to unsigned char = al+0 (used reg = )
test	al,al
jne 	.6BC
.6BD:
! 4911     bios_printf(4, "Invalid boot device (0x%x)\n", bootdev);
! Debug: list unsigned short bootdev = [S+$1E-6] (used reg = )
push	-4[bp]
! Debug: list * char = .6BE+0 (used reg = )
mov	bx,#.6BE
push	bx
! Debug: list int = const 4 (used reg = )
mov	ax,*4
push	ax
! Debug: func () void = bios_printf+0 (used reg = )
call	_bios_printf
add	sp,*6
!BCC_EOS
! 4912     return;
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 4913   }
! 4914   print_boot_device(e.type);
.6BC:
! Debug: list unsigned short e = [S+$1E-$1E] (used reg = )
push	-$1C[bp]
! Debug: func () void = print_boot_device+0 (used reg = )
call	_print_boot_device
inc	sp
inc	sp
!BCC_EOS
! 4915   switch(e.type) {
mov	ax,-$1C[bp]
br 	.6C1
! 4916   case 0x01:
! 4917   case 
.6C2:
! 4917 0x02:
! 4918     bootdrv = (e.type == 0x02) ? 0x80 : 0x00;
.6C3:
! Debug: logeq int = const 2 to unsigned short e = [S+$1E-$1E] (used reg = )
mov	ax,-$1C[bp]
cmp	ax,*2
jne 	.6C4
.6C5:
mov	al,#$80
jmp .6C6
.6C4:
xor	al,al
.6C6:
! Debug: eq char = al+0 to unsigned char bootdrv = [S+$1E-7] (used reg = )
mov	-5[bp],al
!BCC_EOS
! 4919     bootseg = 0x07c0;
! Debug: eq int = const $7C0 to unsigned short bootseg = [S+$1E-$A] (used reg = )
mov	ax,#$7C0
mov	-8[bp],ax
!BCC_EOS
! 4920     status = 0;
! Debug: eq int = const 0 to unsigned short status = [S+$1E-$E] (used reg = )
xor	ax,ax
mov	-$C[bp],ax
!BCC_EOS
! 4921 #asm
!BCC_EOS
!BCC_ASM
_int18_function.bootip	set	$12
.int18_function.bootip	set	-$A
_int18_function.seq_nr	set	$20
.int18_function.seq_nr	set	4
_int18_function.bootchk	set	$16
.int18_function.bootchk	set	-6
_int18_function.bootseg	set	$14
.int18_function.bootseg	set	-8
_int18_function.ebda_seg	set	$1A
.int18_function.ebda_seg	set	-2
_int18_function.status	set	$10
.int18_function.status	set	-$C
_int18_function.bootdrv	set	$17
.int18_function.bootdrv	set	-5
_int18_function.bootdev	set	$18
.int18_function.bootdev	set	-4
_int18_function.e	set	0
.int18_function.e	set	-$1C
    push bp
    mov bp, sp
    push ax
    push bx
    push cx
    push dx
    mov dl, _int18_function.bootdrv + 2[bp]
    mov ax, _int18_function.bootseg + 2[bp]
    mov es, ax ;; segment
    mov bx, #0x0000 ;; offset
    mov ah, #0x02 ;; function 2, read diskette sector
    mov al, #0x01 ;; read 1 sector
    mov ch, #0x00 ;; track 0
    mov cl, #0x01 ;; sector 1
    mov dh, #0x00 ;; head 0
    int #0x13 ;; read sector
    jnc int19_load_done
    mov ax, #0x0001
    mov _int18_function.status + 2[bp], ax
int19_load_done:
    pop dx
    pop cx
    pop bx
    pop ax
    pop bp
! 4947 endasm
!BCC_ENDASM
!BCC_EOS
! 4948     if (status != 0) {
! Debug: ne int = const 0 to unsigned short status = [S+$1E-$E] (used reg = )
mov	ax,-$C[bp]
test	ax,ax
je  	.6C7
.6C8:
! 4949       print_boot_failure(e.type, 1);
! Debug: list int = const 1 (used reg = )
mov	ax,*1
push	ax
! Debug: list unsigned short e = [S+$20-$1E] (used reg = )
push	-$1C[bp]
! Debug: func () void = print_boot_failure+0 (used reg = )
call	_print_boot_failure
add	sp,*4
!BCC_EOS
! 4950       return;
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 4951     }
! 4952     if (e.type != 0x00 || !((inb_cmos(0x38) & 0x01))) {
.6C7:
! Debug: ne int = const 0 to unsigned short e = [S+$1E-$1E] (used reg = )
mov	ax,-$1C[bp]
test	ax,ax
jne 	.6CA
.6CB:
! Debug: list int = const $38 (used reg = )
mov	ax,*$38
push	ax
! Debug: func () unsigned char = inb_cmos+0 (used reg = )
call	_inb_cmos
inc	sp
inc	sp
! Debug: and int = const 1 to unsigned char = al+0 (used reg = )
and	al,*1
test	al,al
jne 	.6C9
.6CA:
! 4953       if (read_word(bootseg,0x1fe) != 0xaa55) {
! Debug: list int = const $1FE (used reg = )
mov	ax,#$1FE
push	ax
! Debug: list unsigned short bootseg = [S+$20-$A] (used reg = )
push	-8[bp]
! Debug: func () unsigned short = read_word+0 (used reg = )
call	_read_word
add	sp,*4
! Debug: ne unsigned int = const $AA55 to unsigned short = ax+0 (used reg = )
cmp	ax,#$AA55
je  	.6CC
.6CD:
! 4954         print_boot_failure(e.type, 0);
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: list unsigned short e = [S+$20-$1E] (used reg = )
push	-$1C[bp]
! Debug: func () void = print_boot_failure+0 (used reg = )
call	_print_boot_failure
add	sp,*4
!BCC_EOS
! 4955         return;
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 4956       }
! 4957     }
.6CC:
! 4958     tcpa_add_bootdevice((Bit32u)0L, (Bit32u)bootdrv);
.6C9:
! Debug: cast unsigned long = const 0 to unsigned char bootdrv = [S+$1E-7] (used reg = )
mov	al,-5[bp]
xor	ah,ah
xor	bx,bx
! Debug: list unsigned long = bx+0 (used reg = )
push	bx
push	ax
! Debug: list unsigned long = const 0 (used reg = )
xor	ax,ax
xor	bx,bx
push	bx
push	ax
! Debug: func () void = tcpa_add_bootdevice+0 (used reg = )
call	_tcpa_add_bootdevice
add	sp,*8
!BCC_EOS
! 4959     tcpa_ipl((Bit32u)0L,(Bit32u)bootseg,(Bit32u)0L,(Bit32u)512L);
! Debug: list unsigned long = const $200 (used reg = )
mov	ax,#$200
xor	bx,bx
push	bx
push	ax
! Debug: list unsigned long = const 0 (used reg = )
xor	ax,ax
xor	bx,bx
push	bx
push	ax
! Debug: cast unsigned long = const 0 to unsigned short bootseg = [S+$26-$A] (used reg = )
mov	ax,-8[bp]
xor	bx,bx
! Debug: list unsigned long = bx+0 (used reg = )
push	bx
push	ax
! Debug: list unsigned long = const 0 (used reg = )
xor	ax,ax
xor	bx,bx
push	bx
push	ax
! Debug: func () void = tcpa_ipl+0 (used reg = )
call	_tcpa_ipl
add	sp,*$10
!BCC_EOS
! 4960     bootip = (bootseg & 0x0fff) << 4;
! Debug: and int = const $FFF to unsigned short bootseg = [S+$1E-$A] (used reg = )
mov	ax,-8[bp]
and	ax,#$FFF
! Debug: sl int = const 4 to unsigned int = ax+0 (used reg = )
mov	cl,*4
shl	ax,cl
! Debug: eq unsigned int = ax+0 to unsigned short bootip = [S+$1E-$C] (used reg = )
mov	-$A[bp],ax
!BCC_EOS
! 4961     bootseg &= 0xf000;
! Debug: andab unsigned int = const $F000 to unsigned short bootseg = [S+$1E-$A] (used reg = )
mov	ax,-8[bp]
and	ax,#$F000
mov	-8[bp],ax
!BCC_EOS
! 4962   break;
br 	.6BF
!BCC_EOS
! 4963   case 0x03:
! 4964     status = cdrom_boot();
.6CE:
! Debug: func () unsigned short = cdrom_boot+0 (used reg = )
call	_cdrom_boot
! Debug: eq unsigned short = ax+0 to unsigned short status = [S+$1E-$E] (used reg = )
mov	-$C[bp],ax
!BCC_EOS
! 4965     if ( (status & 0x00ff) !=0 ) {
! Debug: and int = const $FF to unsigned short status = [S+$1E-$E] (used reg = )
mov	al,-$C[bp]
! Debug: ne int = const 0 to unsigned char = al+0 (used reg = )
test	al,al
je  	.6CF
.6D0:
! 4966       print_cdromboot_failure(status);
! Debug: list unsigned short status = [S+$1E-$E] (used reg = )
push	-$C[bp]
! Debug: func () void = print_cdromboot_failure+0 (used reg = )
call	_print_cdromboot_failure
inc	sp
inc	sp
!BCC_EOS
! 4967       print_boot_failure(e.type, 1);
! Debug: list int = const 1 (used reg = )
mov	ax,*1
push	ax
! Debug: list unsigned short e = [S+$20-$1E] (used reg = )
push	-$1C[bp]
! Debug: func () void = print_boot_failure+0 (used reg = )
call	_print_boot_failure
add	sp,*4
!BCC_EOS
! 4968       return;
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 4969     }
! 4970     bootdrv = (Bit8u)(status>>8);
.6CF:
! Debug: sr int = const 8 to unsigned short status = [S+$1E-$E] (used reg = )
mov	ax,-$C[bp]
mov	al,ah
xor	ah,ah
! Debug: cast unsigned char = const 0 to unsigned int = ax+0 (used reg = )
! Debug: eq unsigned char = al+0 to unsigned char bootdrv = [S+$1E-7] (used reg = )
mov	-5[bp],al
!BCC_EOS
! 4971     bootseg = read_word(ebda_seg,&((ebda_data_t *) 0)->cdemu.load_segment);
! Debug: list * unsigned short = const $246 (used reg = )
mov	ax,#$246
push	ax
! Debug: list unsigned short ebda_seg = [S+$20-4] (used reg = )
push	-2[bp]
! Debug: func () unsigned short = read_word+0 (used reg = )
call	_read_word
add	sp,*4
! Debug: eq unsigned short = ax+0 to unsigned short bootseg = [S+$1E-$A] (used reg = )
mov	-8[bp],ax
!BCC_EOS
! 4972     bootip = (bootseg & 0x0fff) << 4;
! Debug: and int = const $FFF to unsigned short bootseg = [S+$1E-$A] (used reg = )
mov	ax,-8[bp]
and	ax,#$FFF
! Debug: sl int = const 4 to unsigned int = ax+0 (used reg = )
mov	cl,*4
shl	ax,cl
! Debug: eq unsigned int = ax+0 to unsigned short bootip = [S+$1E-$C] (used reg = )
mov	-$A[bp],ax
!BCC_EOS
! 4973     bootseg &= 0xf000;
! Debug: andab unsigned int = const $F000 to unsigned short bootseg = [S+$1E-$A] (used reg = )
mov	ax,-8[bp]
and	ax,#$F000
mov	-8[bp],ax
!BCC_EOS
! 4974     break;
jmp .6BF
!BCC_EOS
! 4975   case 0x80:
! 4976     bootseg = e.vector >> 16;
.6D1:
! Debug: sr int = const $10 to unsigned long e = [S+$1E-$1A] (used reg = )
mov	ax,-$18[bp]
mov	bx,-$16[bp]
xchg	bx,ax
xor	bx,bx
! Debug: eq unsigned long = bx+0 to unsigned short bootseg = [S+$1E-$A] (used reg = )
mov	-8[bp],ax
!BCC_EOS
! 4977     bootip = e.vector & 0xffff;
! Debug: and unsigned long = const $FFFF to unsigned long e = [S+$1E-$1A] (used reg = )
! Debug: expression subtree swapping
mov	ax,#$FFFF
xor	bx,bx
lea	di,-$18[bp]
call	landul
! Debug: eq unsigned long = bx+0 to unsigned short bootip = [S+$1E-$C] (used reg = )
mov	-$A[bp],ax
!BCC_EOS
! 4978     break;
jmp .6BF
!BCC_EOS
! 4979   default: return;
.6D2:
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 4980   }
! 4981 #asm
jmp .6BF
.6C1:
sub	ax,*1
beq 	.6C2
sub	ax,*1
beq 	.6C3
sub	ax,*1
beq 	.6CE
sub	ax,*$7D
je 	.6D1
jmp	.6D2
.6BF:
..FFDA	=	-$1E
!BCC_EOS
!BCC_ASM
_int18_function.bootip	set	$12
.int18_function.bootip	set	-$A
_int18_function.seq_nr	set	$20
.int18_function.seq_nr	set	4
_int18_function.bootchk	set	$16
.int18_function.bootchk	set	-6
_int18_function.bootseg	set	$14
.int18_function.bootseg	set	-8
_int18_function.ebda_seg	set	$1A
.int18_function.ebda_seg	set	-2
_int18_function.status	set	$10
.int18_function.status	set	-$C
_int18_function.bootdrv	set	$17
.int18_function.bootdrv	set	-5
_int18_function.bootdev	set	$18
.int18_function.bootdev	set	-4
_int18_function.e	set	0
.int18_function.e	set	-$1C
    mov bp, sp
    ;; Build an iret stack frame that will take us to the boot vector.
    ;; iret pops ip, then cs, then flags, so push them in the opposite order.
    pushf
    mov ax, _int18_function.bootseg + 0[bp]
    push ax
    mov ax, _int18_function.bootip + 0[bp]
    push ax
    ;; Set the magic number in ax and the boot drive in dl.
    mov ax, #0xaa55
    mov dl, _int18_function.bootdrv + 0[bp]
    ;; Zero some of the other registers.
    xor bx, bx
    mov ds, bx
    mov es, bx
    mov bp, bx
    ;; Go!
    iret
! 5000 endasm
!BCC_ENDASM
!BCC_EOS
! 5001 }
mov	sp,bp
pop	bp
ret
! 5002   void
! Register BX used in function int18_function
! 5003 int1a_function(regs, ds, iret_addr)
! 5004   pusha_regs_t regs;
export	_int1a_function
_int1a_function:
!BCC_EOS
! 5005   Bit16u ds;
!BCC_EOS
! 5006   iret_addr_t iret_addr;
!BCC_EOS
! 5007 {
! 5008   Bit8u val8;
!BCC_EOS
! 5009   ;
push	bp
mov	bp,sp
dec	sp
dec	sp
!BCC_EOS
! 5010 #asm
!BCC_EOS
!BCC_ASM
_int1a_function.ds	set	$16
.int1a_function.ds	set	$14
_int1a_function.val8	set	1
.int1a_function.val8	set	-1
_int1a_function.iret_addr	set	$18
.int1a_function.iret_addr	set	$16
_int1a_function.regs	set	6
.int1a_function.regs	set	4
  sti
! 5012 endasm
!BCC_ENDASM
!BCC_EOS
! 5013   switch (regs.u.r8.ah) {
mov	al,$13[bp]
br 	.6D5
! 5014     case 0:
! 5015 #asm
.6D6:
!BCC_EOS
!BCC_ASM
_int1a_function.ds	set	$16
.int1a_function.ds	set	$14
_int1a_function.val8	set	1
.int1a_function.val8	set	-1
_int1a_function.iret_addr	set	$18
.int1a_function.iret_addr	set	$16
_int1a_function.regs	set	6
.int1a_function.regs	set	4
      cli
! 5017 endasm
!BCC_ENDASM
!BCC_EOS
! 5018       regs.u.r16.cx = ((bios_data_t *) 0)->ticks_high;
! Debug: eq unsigned short = [+$46E] to unsigned short regs = [S+4+$E] (used reg = )
mov	ax,[$46E]
mov	$10[bp],ax
!BCC_EOS
! 5019       regs.u.r16.dx = ((bios_data_t *) 0)->ticks_low;
! Debug: eq unsigned short = [+$46C] to unsigned short regs = [S+4+$C] (used reg = )
mov	ax,[$46C]
mov	$E[bp],ax
!BCC_EOS
! 5020       regs.u.r8.al = ((bios_data_t *) 0)->midnight_flag;
! Debug: eq unsigned char = [+$470] to unsigned char regs = [S+4+$10] (used reg = )
mov	al,[$470]
mov	$12[bp],al
!BCC_EOS
! 5021       ((bios_data_t *) 0)->midnight_flag = 0;
! Debug: eq int = const 0 to unsigned char = [+$470] (used reg = )
xor	al,al
mov	[$470],al
!BCC_EOS
! 5022 #asm
!BCC_EOS
!BCC_ASM
_int1a_function.ds	set	$16
.int1a_function.ds	set	$14
_int1a_function.val8	set	1
.int1a_function.val8	set	-1
_int1a_function.iret_addr	set	$18
.int1a_function.iret_addr	set	$16
_int1a_function.regs	set	6
.int1a_function.regs	set	4
      sti
! 5024 endasm
!BCC_ENDASM
!BCC_EOS
! 5025       iret_addr.flags.u.r8.flagsl &= 0xfe;
! Debug: andab int = const $FE to unsigned char iret_addr = [S+4+$18] (used reg = )
mov	al,$1A[bp]
and	al,#$FE
mov	$1A[bp],al
!BCC_EOS
! 5026       break;
br 	.6D3
!BCC_EOS
! 5027     case 1:
! 5028 #asm
.6D7:
!BCC_EOS
!BCC_ASM
_int1a_function.ds	set	$16
.int1a_function.ds	set	$14
_int1a_function.val8	set	1
.int1a_function.val8	set	-1
_int1a_function.iret_addr	set	$18
.int1a_function.iret_addr	set	$16
_int1a_function.regs	set	6
.int1a_function.regs	set	4
      cli
! 5030 endasm
!BCC_ENDASM
!BCC_EOS
! 5031       ((bios_data_t *) 0)->ticks_high = regs.u.r16.cx;
! Debug: eq unsigned short regs = [S+4+$E] to unsigned short = [+$46E] (used reg = )
mov	ax,$10[bp]
mov	[$46E],ax
!BCC_EOS
! 5032       ((bios_data_t *) 0)->ticks_low = regs.u.r16.dx;
! Debug: eq unsigned short regs = [S+4+$C] to unsigned short = [+$46C] (used reg = )
mov	ax,$E[bp]
mov	[$46C],ax
!BCC_EOS
! 5033       ((bios_data_t *) 0)->midnight_flag = 0;
! Debug: eq int = const 0 to unsigned char = [+$470] (used reg = )
xor	al,al
mov	[$470],al
!BCC_EOS
! 5034 #asm
!BCC_EOS
!BCC_ASM
_int1a_function.ds	set	$16
.int1a_function.ds	set	$14
_int1a_function.val8	set	1
.int1a_function.val8	set	-1
_int1a_function.iret_addr	set	$18
.int1a_function.iret_addr	set	$16
_int1a_function.regs	set	6
.int1a_function.regs	set	4
      sti
! 5036 endasm
!BCC_ENDASM
!BCC_EOS
! 5037       regs.u.r8.ah = 0;
! Debug: eq int = const 0 to unsigned char regs = [S+4+$11] (used reg = )
xor	al,al
mov	$13[bp],al
!BCC_EOS
! 5038       iret_addr.flags.u.r8.flagsl &= 0xfe;
! Debug: andab int = const $FE to unsigned char iret_addr = [S+4+$18] (used reg = )
mov	al,$1A[bp]
and	al,#$FE
mov	$1A[bp],al
!BCC_EOS
! 5039       break;
br 	.6D3
!BCC_EOS
! 5040     case 2:
! 5041       if (rtc_updating()) {
.6D8:
! Debug: func () unsigned short = rtc_updating+0 (used reg = )
call	_rtc_updating
test	ax,ax
je  	.6D9
.6DA:
! 5042         iret_addr.flags.u.r8.flagsl |= 0x01;
! Debug: orab int = const 1 to unsigned char iret_addr = [S+4+$18] (used reg = )
mov	al,$1A[bp]
or	al,*1
mov	$1A[bp],al
!BCC_EOS
! 5043         break;
br 	.6D3
!BCC_EOS
! 5044         }
! 5045       regs.u.r8.dh = inb_cmos(0x00);
.6D9:
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: func () unsigned char = inb_cmos+0 (used reg = )
call	_inb_cmos
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char regs = [S+4+$D] (used reg = )
mov	$F[bp],al
!BCC_EOS
! 5046       regs.u.r8.cl = inb_cmos(0x02);
! Debug: list int = const 2 (used reg = )
mov	ax,*2
push	ax
! Debug: func () unsigned char = inb_cmos+0 (used reg = )
call	_inb_cmos
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char regs = [S+4+$E] (used reg = )
mov	$10[bp],al
!BCC_EOS
! 5047       regs.u.r8.ch = inb_cmos(0x04);
! Debug: list int = const 4 (used reg = )
mov	ax,*4
push	ax
! Debug: func () unsigned char = inb_cmos+0 (used reg = )
call	_inb_cmos
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char regs = [S+4+$F] (used reg = )
mov	$11[bp],al
!BCC_EOS
! 5048       regs.u.r8.dl = inb_cmos(0x0b) & 0x01;
! Debug: list int = const $B (used reg = )
mov	ax,*$B
push	ax
! Debug: func () unsigned char = inb_cmos+0 (used reg = )
call	_inb_cmos
inc	sp
inc	sp
! Debug: and int = const 1 to unsigned char = al+0 (used reg = )
and	al,*1
! Debug: eq unsigned char = al+0 to unsigned char regs = [S+4+$C] (used reg = )
mov	$E[bp],al
!BCC_EOS
! 5049       regs.u.r8.ah = 0;
! Debug: eq int = const 0 to unsigned char regs = [S+4+$11] (used reg = )
xor	al,al
mov	$13[bp],al
!BCC_EOS
! 5050       regs.u.r8.al = regs.u.r8.ch;
! Debug: eq unsigned char regs = [S+4+$F] to unsigned char regs = [S+4+$10] (used reg = )
mov	al,$11[bp]
mov	$12[bp],al
!BCC_EOS
! 5051       iret_addr.flags.u.r8.flagsl &= 0xfe;
! Debug: andab int = const $FE to unsigned char iret_addr = [S+4+$18] (used reg = )
mov	al,$1A[bp]
and	al,#$FE
mov	$1A[bp],al
!BCC_EOS
! 5052       break;
br 	.6D3
!BCC_EOS
! 5053     case 3:
! 5054       if (rtc_updating()) {
.6DB:
! Debug: func () unsigned short = rtc_updating+0 (used reg = )
call	_rtc_updating
test	ax,ax
je  	.6DC
.6DD:
! 5055         init_rtc();
! Debug: func () void = init_rtc+0 (used reg = )
call	_init_rtc
!BCC_EOS
! 5056         }
! 5057       outb_cmos(0x00, regs.u.r8.dh);
.6DC:
! Debug: list unsigned char regs = [S+4+$D] (used reg = )
mov	al,$F[bp]
xor	ah,ah
push	ax
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: func () void = outb_cmos+0 (used reg = )
call	_outb_cmos
add	sp,*4
!BCC_EOS
! 5058       outb_cmos(0x02, regs.u.r8.cl);
! Debug: list unsigned char regs = [S+4+$E] (used reg = )
mov	al,$10[bp]
xor	ah,ah
push	ax
! Debug: list int = const 2 (used reg = )
mov	ax,*2
push	ax
! Debug: func () void = outb_cmos+0 (used reg = )
call	_outb_cmos
add	sp,*4
!BCC_EOS
! 5059       outb_cmos(0x04, regs.u.r8.ch);
! Debug: list unsigned char regs = [S+4+$F] (used reg = )
mov	al,$11[bp]
xor	ah,ah
push	ax
! Debug: list int = const 4 (used reg = )
mov	ax,*4
push	ax
! Debug: func () void = outb_cmos+0 (used reg = )
call	_outb_cmos
add	sp,*4
!BCC_EOS
! 5060       val8 = (inb_cmos(0x0b) & 0x60) | 0x02 | (regs.u.r8.dl & 0x01);
! Debug: expression subtree swapping
! Debug: list int = const $B (used reg = )
mov	ax,*$B
push	ax
! Debug: func () unsigned char = inb_cmos+0 (used reg = )
call	_inb_cmos
inc	sp
inc	sp
! Debug: and int = const $60 to unsigned char = al+0 (used reg = )
and	al,*$60
! Debug: or int = const 2 to unsigned char = al+0 (used reg = )
or	al,*2
push	ax
! Debug: and int = const 1 to unsigned char regs = [S+6+$C] (used reg = )
mov	al,$E[bp]
and	al,*1
! Debug: or unsigned char (temp) = [S+6-6] to unsigned char = al+0 (used reg = )
or	al,0+..FFD9[bp]
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char val8 = [S+4-3] (used reg = )
mov	-1[bp],al
!BCC_EOS
! 5061       outb_cmos(0x0b, val8);
! Debug: list unsigned char val8 = [S+4-3] (used reg = )
mov	al,-1[bp]
xor	ah,ah
push	ax
! Debug: list int = const $B (used reg = )
mov	ax,*$B
push	ax
! Debug: func () void = outb_cmos+0 (used reg = )
call	_outb_cmos
add	sp,*4
!BCC_EOS
! 5062       regs.u.r8.ah = 0;
! Debug: eq int = const 0 to unsigned char regs = [S+4+$11] (used reg = )
xor	al,al
mov	$13[bp],al
!BCC_EOS
! 5063       regs.u.r8.al = val8;
! Debug: eq unsigned char val8 = [S+4-3] to unsigned char regs = [S+4+$10] (used reg = )
mov	al,-1[bp]
mov	$12[bp],al
!BCC_EOS
! 5064       iret_addr.flags.u.r8.flagsl &= 0xfe;
! Debug: andab int = const $FE to unsigned char iret_addr = [S+4+$18] (used reg = )
mov	al,$1A[bp]
and	al,#$FE
mov	$1A[bp],al
!BCC_EOS
! 5065       break;
br 	.6D3
!BCC_EOS
! 5066     case 4:
! 5067       regs.u.r8.ah = 0;
.6DE:
! Debug: eq int = const 0 to unsigned char regs = [S+4+$11] (used reg = )
xor	al,al
mov	$13[bp],al
!BCC_EOS
! 5068       if (rtc_updating()) {
! Debug: func () unsigned short = rtc_updating+0 (used reg = )
call	_rtc_updating
test	ax,ax
je  	.6DF
.6E0:
! 5069         iret_addr.flags.u.r8.flagsl |= 0x01;
! Debug: orab int = const 1 to unsigned char iret_addr = [S+4+$18] (used reg = )
mov	al,$1A[bp]
or	al,*1
mov	$1A[bp],al
!BCC_EOS
! 5070         break;
br 	.6D3
!BCC_EOS
! 5071         }
! 5072       regs.u.r8.cl = inb_cmos(0x09);
.6DF:
! Debug: list int = const 9 (used reg = )
mov	ax,*9
push	ax
! Debug: func () unsigned char = inb_cmos+0 (used reg = )
call	_inb_cmos
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char regs = [S+4+$E] (used reg = )
mov	$10[bp],al
!BCC_EOS
! 5073       regs.u.r8.dh = inb_cmos(0x08);
! Debug: list int = const 8 (used reg = )
mov	ax,*8
push	ax
! Debug: func () unsigned char = inb_cmos+0 (used reg = )
call	_inb_cmos
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char regs = [S+4+$D] (used reg = )
mov	$F[bp],al
!BCC_EOS
! 5074       regs.u.r8.dl = inb_cmos(0x07);
! Debug: list int = const 7 (used reg = )
mov	ax,*7
push	ax
! Debug: func () unsigned char = inb_cmos+0 (used reg = )
call	_inb_cmos
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char regs = [S+4+$C] (used reg = )
mov	$E[bp],al
!BCC_EOS
! 5075       regs.u.r8.ch = inb_cmos(0x32);
! Debug: list int = const $32 (used reg = )
mov	ax,*$32
push	ax
! Debug: func () unsigned char = inb_cmos+0 (used reg = )
call	_inb_cmos
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char regs = [S+4+$F] (used reg = )
mov	$11[bp],al
!BCC_EOS
! 5076       regs.u.r8.al = regs.u.r8.ch;
! Debug: eq unsigned char regs = [S+4+$F] to unsigned char regs = [S+4+$10] (used reg = )
mov	al,$11[bp]
mov	$12[bp],al
!BCC_EOS
! 5077       iret_addr.flags.u.r8.flagsl &= 0xfe;
! Debug: andab int = const $FE to unsigned char iret_addr = [S+4+$18] (used reg = )
mov	al,$1A[bp]
and	al,#$FE
mov	$1A[bp],al
!BCC_EOS
! 5078       break;
br 	.6D3
!BCC_EOS
! 5079     case 5:
! 5080       if (rtc_updating(
.6E1:
! 5080 )) {
! Debug: func () unsigned short = rtc_updating+0 (used reg = )
call	_rtc_updating
test	ax,ax
je  	.6E2
.6E3:
! 5081         init_rtc();
! Debug: func () void = init_rtc+0 (used reg = )
call	_init_rtc
!BCC_EOS
! 5082         iret_addr.flags.u.r8.flagsl |= 0x01;
! Debug: orab int = const 1 to unsigned char iret_addr = [S+4+$18] (used reg = )
mov	al,$1A[bp]
or	al,*1
mov	$1A[bp],al
!BCC_EOS
! 5083         break;
br 	.6D3
!BCC_EOS
! 5084         }
! 5085       outb_cmos(0x09, regs.u.r8.cl);
.6E2:
! Debug: list unsigned char regs = [S+4+$E] (used reg = )
mov	al,$10[bp]
xor	ah,ah
push	ax
! Debug: list int = const 9 (used reg = )
mov	ax,*9
push	ax
! Debug: func () void = outb_cmos+0 (used reg = )
call	_outb_cmos
add	sp,*4
!BCC_EOS
! 5086       outb_cmos(0x08, regs.u.r8.dh);
! Debug: list unsigned char regs = [S+4+$D] (used reg = )
mov	al,$F[bp]
xor	ah,ah
push	ax
! Debug: list int = const 8 (used reg = )
mov	ax,*8
push	ax
! Debug: func () void = outb_cmos+0 (used reg = )
call	_outb_cmos
add	sp,*4
!BCC_EOS
! 5087       outb_cmos(0x07, regs.u.r8.dl);
! Debug: list unsigned char regs = [S+4+$C] (used reg = )
mov	al,$E[bp]
xor	ah,ah
push	ax
! Debug: list int = const 7 (used reg = )
mov	ax,*7
push	ax
! Debug: func () void = outb_cmos+0 (used reg = )
call	_outb_cmos
add	sp,*4
!BCC_EOS
! 5088       outb_cmos(0x32, regs.u.r8.ch);
! Debug: list unsigned char regs = [S+4+$F] (used reg = )
mov	al,$11[bp]
xor	ah,ah
push	ax
! Debug: list int = const $32 (used reg = )
mov	ax,*$32
push	ax
! Debug: func () void = outb_cmos+0 (used reg = )
call	_outb_cmos
add	sp,*4
!BCC_EOS
! 5089       val8 = inb_cmos(0x0b) & 0x7f;
! Debug: list int = const $B (used reg = )
mov	ax,*$B
push	ax
! Debug: func () unsigned char = inb_cmos+0 (used reg = )
call	_inb_cmos
inc	sp
inc	sp
! Debug: and int = const $7F to unsigned char = al+0 (used reg = )
and	al,*$7F
! Debug: eq unsigned char = al+0 to unsigned char val8 = [S+4-3] (used reg = )
mov	-1[bp],al
!BCC_EOS
! 5090       outb_cmos(0x0b, val8);
! Debug: list unsigned char val8 = [S+4-3] (used reg = )
mov	al,-1[bp]
xor	ah,ah
push	ax
! Debug: list int = const $B (used reg = )
mov	ax,*$B
push	ax
! Debug: func () void = outb_cmos+0 (used reg = )
call	_outb_cmos
add	sp,*4
!BCC_EOS
! 5091       regs.u.r8.ah = 0;
! Debug: eq int = const 0 to unsigned char regs = [S+4+$11] (used reg = )
xor	al,al
mov	$13[bp],al
!BCC_EOS
! 5092       regs.u.r8.al = val8;
! Debug: eq unsigned char val8 = [S+4-3] to unsigned char regs = [S+4+$10] (used reg = )
mov	al,-1[bp]
mov	$12[bp],al
!BCC_EOS
! 5093       iret_addr.flags.u.r8.flagsl &= 0xfe;
! Debug: andab int = const $FE to unsigned char iret_addr = [S+4+$18] (used reg = )
mov	al,$1A[bp]
and	al,#$FE
mov	$1A[bp],al
!BCC_EOS
! 5094       break;
br 	.6D3
!BCC_EOS
! 5095     case 6:
! 5096       val8 = inb_cmos(0x0b);
.6E4:
! Debug: list int = const $B (used reg = )
mov	ax,*$B
push	ax
! Debug: func () unsigned char = inb_cmos+0 (used reg = )
call	_inb_cmos
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char val8 = [S+4-3] (used reg = )
mov	-1[bp],al
!BCC_EOS
! 5097       regs.u.r16.ax = 0;
! Debug: eq int = const 0 to unsigned short regs = [S+4+$10] (used reg = )
xor	ax,ax
mov	$12[bp],ax
!BCC_EOS
! 5098       if (val8 & 0x20) {
! Debug: and int = const $20 to unsigned char val8 = [S+4-3] (used reg = )
mov	al,-1[bp]
and	al,*$20
test	al,al
je  	.6E5
.6E6:
! 5099         iret_addr.flags.u.r8.flagsl |= 0x01;
! Debug: orab int = const 1 to unsigned char iret_addr = [S+4+$18] (used reg = )
mov	al,$1A[bp]
or	al,*1
mov	$1A[bp],al
!BCC_EOS
! 5100         break;
br 	.6D3
!BCC_EOS
! 5101         }
! 5102       if (rtc_updating()) {
.6E5:
! Debug: func () unsigned short = rtc_updating+0 (used reg = )
call	_rtc_updating
test	ax,ax
je  	.6E7
.6E8:
! 5103         init_rtc();
! Debug: func () void = init_rtc+0 (used reg = )
call	_init_rtc
!BCC_EOS
! 5104         }
! 5105       outb_cmos(0x01, regs.u.r8.dh);
.6E7:
! Debug: list unsigned char regs = [S+4+$D] (used reg = )
mov	al,$F[bp]
xor	ah,ah
push	ax
! Debug: list int = const 1 (used reg = )
mov	ax,*1
push	ax
! Debug: func () void = outb_cmos+0 (used reg = )
call	_outb_cmos
add	sp,*4
!BCC_EOS
! 5106       outb_cmos(0x03, regs.u.r8.cl);
! Debug: list unsigned char regs = [S+4+$E] (used reg = )
mov	al,$10[bp]
xor	ah,ah
push	ax
! Debug: list int = const 3 (used reg = )
mov	ax,*3
push	ax
! Debug: func () void = outb_cmos+0 (used reg = )
call	_outb_cmos
add	sp,*4
!BCC_EOS
! 5107       outb_cmos(0x05, regs.u.r8.ch);
! Debug: list unsigned char regs = [S+4+$F] (used reg = )
mov	al,$11[bp]
xor	ah,ah
push	ax
! Debug: list int = const 5 (used reg = )
mov	ax,*5
push	ax
! Debug: func () void = outb_cmos+0 (used reg = )
call	_outb_cmos
add	sp,*4
!BCC_EOS
! 5108       outb(0xa1, inb(0xa1) & 0xfe);
! Debug: list int = const $A1 (used reg = )
mov	ax,#$A1
push	ax
! Debug: func () unsigned char = inb+0 (used reg = )
call	_inb
inc	sp
inc	sp
! Debug: and int = const $FE to unsigned char = al+0 (used reg = )
and	al,#$FE
! Debug: list unsigned char = al+0 (used reg = )
xor	ah,ah
push	ax
! Debug: list int = const $A1 (used reg = )
mov	ax,#$A1
push	ax
! Debug: func () void = outb+0 (used reg = )
call	_outb
add	sp,*4
!BCC_EOS
! 5109       outb_cmos(0x0b, (val8 & 0x7f) | 0x20);
! Debug: and int = const $7F to unsigned char val8 = [S+4-3] (used reg = )
mov	al,-1[bp]
and	al,*$7F
! Debug: or int = const $20 to unsigned char = al+0 (used reg = )
or	al,*$20
! Debug: list unsigned char = al+0 (used reg = )
xor	ah,ah
push	ax
! Debug: list int = const $B (used reg = )
mov	ax,*$B
push	ax
! Debug: func () void = outb_cmos+0 (used reg = )
call	_outb_cmos
add	sp,*4
!BCC_EOS
! 5110       iret_addr.flags.u.r8.flagsl &= 0xfe;
! Debug: andab int = const $FE to unsigned char iret_addr = [S+4+$18] (used reg = )
mov	al,$1A[bp]
and	al,#$FE
mov	$1A[bp],al
!BCC_EOS
! 5111       break;
br 	.6D3
!BCC_EOS
! 5112     case 7:
! 5113       val8 = inb_cmos(0x0b);
.6E9:
! Debug: list int = const $B (used reg = )
mov	ax,*$B
push	ax
! Debug: func () unsigned char = inb_cmos+0 (used reg = )
call	_inb_cmos
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char val8 = [S+4-3] (used reg = )
mov	-1[bp],al
!BCC_EOS
! 5114       outb_cmos(0x0b, val8 & 0x57);
! Debug: and int = const $57 to unsigned char val8 = [S+4-3] (used reg = )
mov	al,-1[bp]
and	al,*$57
! Debug: list unsigned char = al+0 (used reg = )
xor	ah,ah
push	ax
! Debug: list int = const $B (used reg = )
mov	ax,*$B
push	ax
! Debug: func () void = outb_cmos+0 (used reg = )
call	_outb_cmos
add	sp,*4
!BCC_EOS
! 5115       regs.u.r8.ah = 0;
! Debug: eq int = const 0 to unsigned char regs = [S+4+$11] (used reg = )
xor	al,al
mov	$13[bp],al
!BCC_EOS
! 5116       regs.u.r8.al = val8;
! Debug: eq unsigned char val8 = [S+4-3] to unsigned char regs = [S+4+$10] (used reg = )
mov	al,-1[bp]
mov	$12[bp],al
!BCC_EOS
! 5117       iret_addr.flags.u.r8.flagsl &= 0xfe;
! Debug: andab int = const $FE to unsigned char iret_addr = [S+4+$18] (used reg = )
mov	al,$1A[bp]
and	al,#$FE
mov	$1A[bp],al
!BCC_EOS
! 5118       break;
br 	.6D3
!BCC_EOS
! 5119     case 0xb1:
! 5120       if (regs.u.r8.bl == 0xff) {
.6EA:
! Debug: logeq int = const $FF to unsigned char regs = [S+4+$A] (used reg = )
mov	al,$C[bp]
cmp	al,#$FF
jne 	.6EB
.6EC:
! 5121         bios_printf(4, "PCI BIOS: PCI not present\n");
! Debug: list * char = .6ED+0 (used reg = )
mov	bx,#.6ED
push	bx
! Debug: list int = const 4 (used reg = )
mov	ax,*4
push	ax
! Debug: func () void = bios_printf+0 (used reg = )
call	_bios_printf
add	sp,*4
!BCC_EOS
! 5122       } else if (regs.u.r8.bl == 0x81) {
jmp .6EE
.6EB:
! Debug: logeq int = const $81 to unsigned char regs = [S+4+$A] (used reg = )
mov	al,$C[bp]
cmp	al,#$81
jne 	.6EF
.6F0:
! 5123         bios_printf(4, "unsupported PCI BIOS function 0x%02x\n", regs.u.r8.al);
! Debug: list unsigned char regs = [S+4+$10] (used reg = )
mov	al,$12[bp]
xor	ah,ah
push	ax
! Debug: list * char = .6F1+0 (used reg = )
mov	bx,#.6F1
push	bx
! Debug: list int = const 4 (used reg = )
mov	ax,*4
push	ax
! Debug: func () void = bios_printf+0 (used reg = )
call	_bios_printf
add	sp,*6
!BCC_EOS
! 5124       } else if (regs.u.r8.bl == 0x83) {
jmp .6F2
.6EF:
! Debug: logeq int = const $83 to unsigned char regs = [S+4+$A] (used reg = )
mov	al,$C[bp]
cmp	al,#$83
jne 	.6F3
.6F4:
! 5125         bios_printf(4, "bad PCI vendor ID %04x\n", regs.u.r16.dx);
! Debug: list unsigned short regs = [S+4+$C] (used reg = )
push	$E[bp]
! Debug: list * char = .6F5+0 (used reg = )
mov	bx,#.6F5
push	bx
! Debug: list int = const 4 (used reg = )
mov	ax,*4
push	ax
! Debug: func () void = bios_printf+0 (used reg = )
call	_bios_printf
add	sp,*6
!BCC_EOS
! 5126       } else if (regs.u.r8.bl == 0x86) {
jmp .6F6
.6F3:
! Debug: logeq int = const $86 to unsigned char regs = [S+4+$A] (used reg = )
mov	al,$C[bp]
cmp	al,#$86
jne 	.6F7
.6F8:
! 5127         bios_printf(4, "PCI device %04x:%04x not found\n", regs.u.r16.dx, regs.u.r16.cx);
! Debug: list unsigned short regs = [S+4+$E] (used reg = )
push	$10[bp]
! Debug: list unsigned short regs = [S+6+$C] (used reg = )
push	$E[bp]
! Debug: list * char = .6F9+0 (used reg = )
mov	bx,#.6F9
push	bx
! Debug: list int = const 4 (used reg = )
mov	ax,*4
push	ax
! Debug: func () void = bios_printf+0 (used reg = )
call	_bios_printf
add	sp,*8
!BCC_EOS
! 5128       }
! 5129       regs.u.r8.ah = regs.u.r8.bl;
.6F7:
.6F6:
.6F2:
.6EE:
! Debug: eq unsigned char regs = [S+4+$A] to unsigned char regs = [S+4+$11] (used reg = )
mov	al,$C[bp]
mov	$13[bp],al
!BCC_EOS
! 5130       iret_addr.flags.u.r8.flagsl |= 0x01;
! Debug: orab int = const 1 to unsigned char iret_addr = [S+4+$18] (used reg = )
mov	al,$1A[bp]
or	al,*1
mov	$1A[bp],al
!BCC_EOS
! 5131       break;
jmp .6D3
!BCC_EOS
! 5132     default:
! 5133       iret_addr.flags.u.r8.flagsl |= 0x01;
.6FA:
! Debug: orab int = const 1 to unsigned char iret_addr = [S+4+$18] (used reg = )
mov	al,$1A[bp]
or	al,*1
mov	$1A[bp],al
!BCC_EOS
! 5134     }
! 5135 }
jmp .6D3
.6D5:
sub	al,*0
jb 	.6FA
cmp	al,*7
ja  	.6FB
xor	ah,ah
shl	ax,*1
mov	bx,ax
seg	cs
br	.6FC[bx]
.6FC:
.word	.6D6
.word	.6D7
.word	.6D8
.word	.6DB
.word	.6DE
.word	.6E1
.word	.6E4
.word	.6E9
.6FB:
sub	al,#$B1
beq 	.6EA
jmp	.6FA
.6D3:
..FFD9	=	-4
mov	sp,bp
pop	bp
ret
! 5136   void
! Register BX used in function int1a_function
! 5137 int70_function(regs, ds, iret_addr)
! 5138   pusha_regs_t regs;
export	_int70_function
_int70_function:
!BCC_EOS
! 5139   Bit16u ds;
!BCC_EOS
! 5140   iret_addr_t iret_addr;
!BCC_EOS
! 5141 {
! 5142   Bit8u registerB = 0, registerC = 0;
push	bp
mov	bp,sp
dec	sp
! Debug: eq int = const 0 to unsigned char registerB = [S+3-3] (used reg = )
xor	al,al
mov	-1[bp],al
dec	sp
! Debug: eq int = const 0 to unsigned char registerC = [S+4-4] (used reg = )
xor	al,al
mov	-2[bp],al
!BCC_EOS
! 5143   registerB = inb_cmos( 0xB );
! Debug: list int = const $B (used reg = )
mov	ax,*$B
push	ax
! Debug: func () unsigned char = inb_cmos+0 (used reg = )
call	_inb_cmos
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char registerB = [S+4-3] (used reg = )
mov	-1[bp],al
!BCC_EOS
! 5144   registerC = inb_cmos( 0xC );
! Debug: list int = const $C (used reg = )
mov	ax,*$C
push	ax
! Debug: func () unsigned char = inb_cmos+0 (used reg = )
call	_inb_cmos
inc	sp
inc	sp
! Debug: eq unsigned char = al+0 to unsigned char registerC = [S+4-4] (used reg = )
mov	-2[bp],al
!BCC_EOS
! 5145   if( ( registerB & 0x60 ) != 0 ) {
! Debug: and int = const $60 to unsigned char registerB = [S+4-3] (used reg = )
mov	al,-1[bp]
and	al,*$60
! Debug: ne int = const 0 to unsigned char = al+0 (used reg = )
test	al,al
beq 	.6FD
.6FE:
! 5146     if( ( registerC & 0x20 ) != 0 ) {
! Debug: and int = const $20 to unsigned char registerC = [S+4-4] (used reg = )
mov	al,-2[bp]
and	al,*$20
! Debug: ne int = const 0 to unsigned char = al+0 (used reg = )
test	al,al
je  	.6FF
.700:
! 5147 #asm
!BCC_EOS
!BCC_ASM
_int70_function.registerC	set	0
.int70_function.registerC	set	-2
_int70_function.ds	set	$16
.int70_function.ds	set	$14
_int70_function.registerB	set	1
.int70_function.registerB	set	-1
_int70_function.iret_addr	set	$18
.int70_function.iret_addr	set	$16
_int70_function.regs	set	6
.int70_function.regs	set	4
      sti
      int #0x4a
      cli
! 5151 endasm
!BCC_ENDASM
!BCC_EOS
! 5152     }
! 5153     if( ( registerC & 
.6FF:
! 5153 0x40 ) != 0 ) {
! Debug: and int = const $40 to unsigned char registerC = [S+4-4] (used reg = )
mov	al,-2[bp]
and	al,*$40
! Debug: ne int = const 0 to unsigned char = al+0 (used reg = )
test	al,al
beq 	.701
.702:
! 5154       if( read_byte( 0x40, 0xA0 ) != 0 ) {
! Debug: list int = const $A0 (used reg = )
mov	ax,#$A0
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () unsigned char = read_byte+0 (used reg = )
call	_read_byte
add	sp,*4
! Debug: ne int = const 0 to unsigned char = al+0 (used reg = )
test	al,al
beq 	.703
.704:
! 5155         Bit32u time, toggle;
!BCC_EOS
! 5156         time = read_dword( 0x40, 0x9C );
add	sp,*-8
! Debug: list int = const $9C (used reg = )
mov	ax,#$9C
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () unsigned long = read_dword+0 (used reg = )
call	_read_dword
mov	bx,dx
add	sp,*4
! Debug: eq unsigned long = bx+0 to unsigned long time = [S+$C-8] (used reg = )
mov	-6[bp],ax
mov	-4[bp],bx
!BCC_EOS
! 5157         if( time < 0x3D1 ) {
! Debug: lt unsigned long = const $3D1 to unsigned long time = [S+$C-8] (used reg = )
mov	ax,#$3D1
xor	bx,bx
lea	di,-6[bp]
call	lcmpul
jbe 	.705
.706:
! 5158           Bit16u segment, offset;
!BCC_EOS
! 5159           offset = read_word( 0x40, 0x98 );
add	sp,*-4
! Debug: list int = const $98 (used reg = )
mov	ax,#$98
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () unsigned short = read_word+0 (used reg = )
call	_read_word
add	sp,*4
! Debug: eq unsigned short = ax+0 to unsigned short offset = [S+$10-$10] (used reg = )
mov	-$E[bp],ax
!BCC_EOS
! 5160           segment = read_word( 0x40, 0x9A );
! Debug: list int = const $9A (used reg = )
mov	ax,#$9A
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () unsigned short = read_word+0 (used reg = )
call	_read_word
add	sp,*4
! Debug: eq unsigned short = ax+0 to unsigned short segment = [S+$10-$E] (used reg = )
mov	-$C[bp],ax
!BCC_EOS
! 5161           write_byte( 0x40, 0xA0, 0 );
! Debug: list int = const 0 (used reg = )
xor	ax,ax
push	ax
! Debug: list int = const $A0 (used reg = )
mov	ax,#$A0
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 5162           outb_cmos( 0xB, registerB & 0x37 );
! Debug: and int = const $37 to unsigned char registerB = [S+$10-3] (used reg = )
mov	al,-1[bp]
and	al,*$37
! Debug: list unsigned char = al+0 (used reg = )
xor	ah,ah
push	ax
! Debug: list int = const $B (used reg = )
mov	ax,*$B
push	ax
! Debug: func () void = outb_cmos+0 (used reg = )
call	_outb_cmos
add	sp,*4
!BCC_EOS
! 5163           write_byte( segment, offset, 0x80 );
! Debug: list int = const $80 (used reg = )
mov	ax,#$80
push	ax
! Debug: list unsigned short offset = [S+$12-$10] (used reg = )
push	-$E[bp]
! Debug: list unsigned short segment = [S+$14-$E] (used reg = )
push	-$C[bp]
! Debug: func () void = write_byte+0 (used reg = )
call	_write_byte
add	sp,*6
!BCC_EOS
! 5164         } else {
add	sp,*4
jmp .707
.705:
! 5165           time -= 0x3D1;
! Debug: subab unsigned long = const $3D1 to unsigned long time = [S+$C-8] (used reg = )
mov	ax,#$3D1
xor	bx,bx
push	bx
push	ax
mov	ax,-6[bp]
mov	bx,-4[bp]
lea	di,-$E[bp]
call	lsubul
mov	-6[bp],ax
mov	-4[bp],bx
add	sp,*4
!BCC_EOS
! 5166           write_dword( 0x40, 0x9C, time );
! Debug: list unsigned long time = [S+$C-8] (used reg = )
push	-4[bp]
push	-6[bp]
! Debug: list int = const $9C (used reg = )
mov	ax,#$9C
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () void = write_dword+0 (used reg = )
call	_write_dword
add	sp,*8
!BCC_EOS
! 5167         }
! 5168       }
.707:
add	sp,*8
! 5169     }
.703:
! 5170   }
.701:
! 5171 #asm
.6FD:
!BCC_EOS
!BCC_ASM
_int70_function.registerC	set	0
.int70_function.registerC	set	-2
_int70_function.ds	set	$16
.int70_function.ds	set	$14
_int70_function.registerB	set	1
.int70_function.registerB	set	-1
_int70_function.iret_addr	set	$18
.int70_function.iret_addr	set	$16
_int70_function.regs	set	6
.int70_function.regs	set	4
  call eoi_both_pics
! 5173 endasm
!BCC_ENDASM
!BCC_EOS
! 5174 }
mov	sp,bp
pop	bp
ret
! 5175 #asm
!BCC_ASM
_int70_function.ds	set	$12
_int70_function.iret_addr	set	$14
_int70_function.regs	set	2
;------------------------------------------
;- INT74h : PS/2 mouse hardware interrupt -
;------------------------------------------
int74_handler:
  sti
  pusha
  push ds ;; save DS
  push #0x00 ;; placeholder for status
  push #0x00 ;; placeholder for X
  push #0x00 ;; placeholder for Y
  push #0x00 ;; placeholder for Z
  push #0x00 ;; placeholder for make_far_call boolean
  call _int74_function
  pop cx ;; remove make_far_call from stack
  jcxz int74_done
  ;; make far call to EBDA:0022
  push #0x00
  pop ds
  push 0x040E ;; push 0000:040E (opcodes 0xff, 0x36, 0x0E, 0x04)
  pop ds
  call far ptr[0x22]
int74_done:
  cli
  call eoi_both_pics
  add sp, #8 ;; pop status, x, y, z
  pop ds ;; restore DS
  popa
  iret
;; This will perform an IRET, but will retain value of current CF
;; by altering flags on stack. Better than RETF #02.
iret_modify_cf:
  jc carry_set
  push bp
  mov bp, sp
  and BYTE [bp + 0x06], #0xfe
  pop bp
  iret
carry_set:
  push bp
  mov bp, sp
  or BYTE [bp + 0x06], #0x01
  pop bp
  iret
;----------------------
;- INT13h (relocated) -
;----------------------
;
; int13_relocated is a little bit messed up since I played with it
; I have to rewrite it:
; - call a function that detect which function to call
; - make all called C function get the same parameters list
;
int13_relocated:
  ;; check for an eltorito function
  cmp ah,#0x4a
  jb int13_not_eltorito
  cmp ah,#0x4d
  ja int13_not_eltorito
  pusha
  push es
  push ds
  push ss
  pop ds
  push #int13_out
  jmp _int13_eltorito ;; ELDX not used
int13_not_eltorito:
  push ax
  push bx
  push cx
  push dx
  ;; check if emulation active
  call _cdemu_isactive
  cmp al,#0x00
  je int13_cdemu_inactive
  ;; check if access to the emulated drive
  call _cdemu_emulated_drive
  pop dx
  push dx
  cmp al,dl ;; int13 on emulated drive
  jne int13_nocdemu
  pop dx
  pop cx
  pop bx
  pop ax
  pusha
  push es
  push ds
  push ss
  pop ds
  push #int13_out
  jmp _int13_cdemu ;; ELDX not used
int13_nocdemu:
  and dl,#0xE0 ;; mask to get device class, including cdroms
  cmp al,dl ;; al is 0x00 or 0x80
  jne int13_cdemu_inactive ;; inactive for device class
  pop dx
  pop cx
  pop bx
  pop ax
  push ax
  push cx
  push dx
  push bx
  dec dl ;; real drive is dl - 1
  jmp int13_legacy
int13_cdemu_inactive:
  pop dx
  pop cx
  pop bx
  pop ax
int13_noeltorito:
  push ax
  push cx
  push dx
  push bx
int13_legacy:
  push dx ;; push eltorito value of dx instead of sp
  push bp
  push si
  push di
  push es
  push ds
  push ss
  pop ds
  ;; now the 16-bit registers can be restored with:
  ;; pop ds; pop es; popa; iret
  ;; arguments passed to functions should be
  ;; DS, ES, DI, SI, BP, ELDX, BX, DX, CX, AX, IP, CS, FLAGS
  test dl, #0x80
  jnz int13_notfloppy
  push #int13_out
  jmp _int13_diskette_function
int13_notfloppy:
  cmp dl, #0xE0
  jb int13_notcdrom
  shr ebx, #16
  push bx
  call _int13_cdrom
  pop bx
  shl ebx, #16
  jmp int13_out
int13_notcdrom:
int13_disk:
  call _int13_harddisk
int13_out:
  pop ds
  pop es
  popa
  iret
;----------
;- INT18h -
;----------
int18_handler: ;; Boot Failure recovery: try the next device.
  ;; Reset SP and SS
  mov ax, #0xfffe
  mov sp, ax
  xor ax, ax
  mov ss, ax
  ;; Get the boot sequence number out of the IPL memory
  ;; The first time we do this it will have been set to -1 so
  ;; we will start from device 0.
  mov bx, #0x9ff0
  mov ds, bx ;; Set segment
  mov bx, 0x0082 ;; BX is now the sequence number
  inc bx ;; ++
  mov 0x0082, bx ;; Write it back
  mov ds, ax ;; and reset the segment to zero.
  ;; Call the C code for the next boot device
  push bx
  call _int18_function
  ;; Boot failed: invoke the boot recovery function...
  int #0x18
;----------
;- INT19h -
;----------
int19_relocated: ;; Boot function, relocated
  ;;
  ;; *** Warning: INT 19h resets the whole machine ***
  ;;
  ;; Because PV drivers in HVM guests detach some of the emulated devices,
  ;; it is not safe to do a soft reboot by just dropping to real mode and
  ;; invoking INT 19h -- the boot drives might have disappeared!
  ;; If the user asks for a soft reboot, the only thing we can do is
  ;; reset the whole machine. When it comes back up, the normal BIOS
  ;; boot sequence will start, which is more or less the required behaviour.
  ;;
  ;; Reset SP and SS
  mov ax, #0xfffe
  mov sp, ax
  xor ax, ax
  mov ss, ax
  call _machine_reset
;----------
;- INT1Ch -
;----------
int1c_handler: ;; User Timer Tick
  iret
;----------------------
;- POST: Floppy Drive -
;----------------------
floppy_drive_post:
  mov ax, #0x0000
  mov ds, ax
  mov al, #0x00
  mov 0x043e, al ;; drive 0 & 1 uncalibrated, no interrupt has occurred
  mov 0x043f, al ;; diskette motor status: read op, drive0, motors off
  mov 0x0440, al ;; diskette motor timeout counter: not active
  mov 0x0441, al ;; diskette controller status return code
  mov 0x0442, al ;; disk & diskette controller status register 0
  mov 0x0443, al ;; diskette controller status register 1
  mov 0x0444, al ;; diskette controller status register 2
  mov 0x0445, al ;; diskette controller cylinder number
  mov 0x0446, al ;; diskette controller head number
  mov 0x0447, al ;; diskette controller sector number
  mov 0x0448, al ;; diskette controller bytes written
  mov 0x048b, al ;; diskette configuration data
  ;; -----------------------------------------------------------------
  ;; (048F) diskette controller information
  ;;
  mov al, #0x10 ;; get CMOS diskette drive type
  out 0x70, AL
  in AL, 0x71
  mov ah, al ;; save byte to AH
look_drive0:
  shr al, #4 ;; look at top 4 bits for drive 0
  jz f0_missing ;; jump if no drive0
  mov bl, #0x07 ;; drive0 determined, multi-rate, has changed line
  jmp look_drive1
f0_missing:
  mov bl, #0x00 ;; no drive0
look_drive1:
  mov al, ah ;; restore from AH
  and al, #0x0f ;; look at bottom 4 bits for drive 1
  jz f1_missing ;; jump if no drive1
  or bl, #0x70 ;; drive1 determined, multi-rate, has changed line
f1_missing:
                   ;; leave high bits in BL zerod
  mov 0x048f, bl ;; put new val in BDA (diskette controller information)
  ;; -----------------------------------------------------------------
  mov al, #0x00
  mov 0x0490, al ;; diskette 0 media state
  mov 0x0491, al ;; diskette 1 media state
                   ;; diskette 0,1 operational starting state
                   ;; drive type has not been determined,
                   ;; has no changed detection line
  mov 0x0492, al
  mov 0x0493, al
  mov 0x0494, al ;; diskette 0 current cylinder
  mov 0x0495, al ;; diskette 1 current cylinder
  mov al, #0x02
  out #0x0a, al ;; clear DMA-1 channel 2 mask bit
  SET_INT_VECTOR(0x1E, #0xF000, #diskette_param_table2)
  SET_INT_VECTOR(0x40, #0xF000, #int13_diskette)
  SET_INT_VECTOR(0x0E, #0xF000, #int0e_handler) ;; IRQ 6
  ret
;--------------------
;- POST: HARD DRIVE -
;--------------------
; relocated here because the primary POST area isnt big enough.
hard_drive_post:
  mov al, #0x0a ; 0000 1010 = reserved, disable IRQ 14
  mov dx, #0x03f6
  out dx, al
  mov ax, #0x0000
  mov ds, ax
  mov 0x0474, al
  mov 0x0477, al
  mov 0x048c, al
  mov 0x048d, al
  mov 0x048e, al
  mov al, #0x01
  mov 0x0475, al
  mov al, #0xc0
  mov 0x0476, al
  SET_INT_VECTOR(0x13, #0xF000, #int13_handler)
  SET_INT_VECTOR(0x76, #0xF000, #int76_handler)
  ;; INT 41h: hard disk 0 configuration pointer
  ;; INT 46h: hard disk 1 configuration pointer
  SET_INT_VECTOR(0x41, #0x9FC0, #0x003D)
  SET_INT_VECTOR(0x46, #0x9FC0, #0x004D)
  ;; move disk geometry data from CMOS to EBDA disk parameter table(s)
  mov al, #0x12
  out #0x70, al
  in al, #0x71
  and al, #0xf0
  cmp al, #0xf0
  je post_d0_extended
  jmp check_for_hd1
post_d0_extended:
  mov al, #0x19
  out #0x70, al
  in al, #0x71
  cmp al, #47 ;; decimal 47 - user definable
  je post_d0_type47
  HALT(8707)
post_d0_type47:
  ;; CMOS purpose param table offset
  ;; 1b cylinders low 0
  ;; 1c cylinders high 1
  ;; 1d heads 2
  ;; 1e write pre-comp low 5
  ;; 1f write pre-comp high 6
  ;; 20 retries/bad map/heads>8 8
  ;; 21 landing zone low C
  ;; 22 landing zone high D
  ;; 23 sectors/track E
  mov ax, #0x9FC0
  mov ds, ax
  ;;; Filling EBDA table for hard disk 0.
  mov al, #0x1f
  out #0x70, al
  in al, #0x71
  mov ah, al
  mov al, #0x1e
  out #0x70, al
  in al, #0x71
  mov (0x003d + 0x05), ax ;; write precomp word
  mov al, #0x20
  out #0x70, al
  in al, #0x71
  mov (0x003d + 0x08), al ;; drive control byte
  mov al, #0x22
  out #0x70, al
  in al, #0x71
  mov ah, al
  mov al, #0x21
  out #0x70, al
  in al, #0x71
  mov (0x003d + 0x0C), ax ;; landing zone word
  mov al, #0x1c ;; get cylinders word in AX
  out #0x70, al
  in al, #0x71 ;; high byte
  mov ah, al
  mov al, #0x1b
  out #0x70, al
  in al, #0x71 ;; low byte
  mov bx, ax ;; BX = cylinders
  mov al, #0x1d
  out #0x70, al
  in al, #0x71
  mov cl, al ;; CL = heads
  mov al, #0x23
  out #0x70, al
  in al, #0x71
  mov dl, al ;; DL = sectors
  cmp bx, #1024
  jnbe hd0_post_logical_chs ;; if cylinders > 1024, use translated style CHS
hd0_post_physical_chs:
  ;; no logical CHS mapping used, just physical CHS
  ;; use Standard Fixed Disk Parameter Table (FDPT)
  mov (0x003d + 0x00), bx ;; number of physical cylinders
  mov (0x003d + 0x02), cl ;; number of physical heads
  mov (0x003d + 0x0E), dl ;; number of physical sectors
  jmp check_for_hd1
hd0_post_logical_chs:
  ;; complies with Phoenix style Translated Fixed Disk Parameter Table (FDPT)
  mov (0x003d + 0x09), bx ;; number of physical cylinders
  mov (0x003d + 0x0b), cl ;; number of physical heads
  mov (0x003d + 0x04), dl ;; number of physical sectors
  mov (0x003d + 0x0e), dl ;; number of logical sectors (same)
  mov al, #0xa0
  mov (0x003d + 0x03), al ;; A0h signature, indicates translated table
  cmp bx, #2048
  jnbe hd0_post_above_2048
  ;; 1024 < c <= 2048 cylinders
  shr bx, #0x01
  shl cl, #0x01
  jmp hd0_post_store_logical
hd0_post_above_2048:
  cmp bx, #4096
  jnbe hd0_post_above_4096
  ;; 2048 < c <= 4096 cylinders
  shr bx, #0x02
  shl cl, #0x02
  jmp hd0_post_store_logical
hd0_post_above_4096:
  cmp bx, #8192
  jnbe hd0_post_above_8192
  ;; 4096 < c <= 8192 cylinders
  shr bx, #0x03
  shl cl, #0x03
  jmp hd0_post_store_logical
hd0_post_above_8192:
  ;; 8192 < c <= 16384 cylinders
  shr bx, #0x04
  shl cl, #0x04
hd0_post_store_logical:
  mov (0x003d + 0x00), bx ;; number of physical cylinders
  mov (0x003d + 0x02), cl ;; number of physical heads
  ;; checksum
  mov cl, #0x0f ;; repeat count
  mov si, #0x003d ;; offset to disk0 FDPT
  mov al, #0x00 ;; sum
hd0_post_checksum_loop:
  add al, [si]
  inc si
  dec cl
  jnz hd0_post_checksum_loop
  not al ;; now take 2s complement
  inc al
  mov [si], al
;;; Done filling EBDA table for hard disk 0.
check_for_hd1:
  ;; is there really a second hard disk? if not, return now
  mov al, #0x12
  out #0x70, al
  in al, #0x71
  and al, #0x0f
  jnz post_d1_exists
  ret
post_d1_exists:
  ;; check that the hd type is really 0x0f.
  cmp al, #0x0f
  jz post_d1_extended
  HALT(8844)
post_d1_extended:
  ;; check that the extended type is 47 - user definable
  mov al, #0x1a
  out #0x70, al
  in al, #0x71
  cmp al, #47 ;; decimal 47 - user definable
  je post_d1_type47
  HALT(8852)
post_d1_type47:
  ;; Table for disk1.
  ;; CMOS purpose param table offset
  ;; 0x24 cylinders low 0
  ;; 0x25 cylinders high 1
  ;; 0x26 heads 2
  ;; 0x27 write pre-comp low 5
  ;; 0x28 write pre-comp high 6
  ;; 0x29 heads>8 8
  ;; 0x2a landing zone low C
  ;; 0x2b landing zone high D
  ;; 0x2c sectors/track E
;;; Fill EBDA table for hard disk 1.
  mov ax, #0x9FC0
  mov ds, ax
  mov al, #0x28
  out #0x70, al
  in al, #0x71
  mov ah, al
  mov al, #0x27
  out #0x70, al
  in al, #0x71
  mov (0x004d + 0x05), ax ;; write precomp word
  mov al, #0x29
  out #0x70, al
  in al, #0x71
  mov (0x004d + 0x08), al ;; drive control byte
  mov al, #0x2b
  out #0x70, al
  in al, #0x71
  mov ah, al
  mov al, #0x2a
  out #0x70, al
  in al, #0x71
  mov (0x004d + 0x0C), ax ;; landing zone word
  mov al, #0x25 ;; get cylinders word in AX
  out #0x70, al
  in al, #0x71 ;; high byte
  mov ah, al
  mov al, #0x24
  out #0x70, al
  in al, #0x71 ;; low byte
  mov bx, ax ;; BX = cylinders
  mov al, #0x26
  out #0x70, al
  in al, #0x71
  mov cl, al ;; CL = heads
  mov al, #0x2c
  out #0x70, al
  in al, #0x71
  mov dl, al ;; DL = sectors
  cmp bx, #1024
  jnbe hd1_post_logical_chs ;; if cylinders > 1024, use translated style CHS
hd1_post_physical_chs:
  ;; no logical CHS mapping used, just physical CHS
  ;; use Standard Fixed Disk Parameter Table (FDPT)
  mov (0x004d + 0x00), bx ;; number of physical cylinders
  mov (0x004d + 0x02), cl ;; number of physical heads
  mov (0x004d + 0x0E), dl ;; number of physical sectors
  ret
hd1_post_logical_chs:
  ;; complies with Phoenix style Translated Fixed Disk Parameter Table (FDPT)
  mov (0x004d + 0x09), bx ;; number of physical cylinders
  mov (0x004d + 0x0b), cl ;; number of physical heads
  mov (0x004d + 0x04), dl ;; number of physical sectors
  mov (0x004d + 0x0e), dl ;; number of logical sectors (same)
  mov al, #0xa0
  mov (0x004d + 0x03), al ;; A0h signature, indicates translated table
  cmp bx, #2048
  jnbe hd1_post_above_2048
  ;; 1024 < c <= 2048 cylinders
  shr bx, #0x01
  shl cl, #0x01
  jmp hd1_post_store_logical
hd1_post_above_2048:
  cmp bx, #4096
  jnbe hd1_post_above_4096
  ;; 2048 < c <= 4096 cylinders
  shr bx, #0x02
  shl cl, #0x02
  jmp hd1_post_store_logical
hd1_post_above_4096:
  cmp bx, #8192
  jnbe hd1_post_above_8192
  ;; 4096 < c <= 8192 cylinders
  shr bx, #0x03
  shl cl, #0x03
  jmp hd1_post_store_logical
hd1_post_above_8192:
  ;; 8192 < c <= 16384 cylinders
  shr bx, #0x04
  shl cl, #0x04
hd1_post_store_logical:
  mov (0x004d + 0x00), bx ;; number of physical cylinders
  mov (0x004d + 0x02), cl ;; number of physical heads
  ;; checksum
  mov cl, #0x0f ;; repeat count
  mov si, #0x004d ;; offset to disk0 FDPT
  mov al, #0x00 ;; sum
hd1_post_checksum_loop:
  add al, [si]
  inc si
  dec cl
  jnz hd1_post_checksum_loop
  not al ;; now take 2s complement
  inc al
  mov [si], al
;;; Done filling EBDA table for hard disk 1.
  ret
;--------------------
;- POST: EBDA segment
;--------------------
; relocated here because the primary POST area isnt big enough.
ebda_post:
  mov ax, #0x9FC0
  mov ds, ax
  mov byte ptr [0x0], #1
  xor ax, ax ; mov EBDA seg into 40E
  mov ds, ax
  mov word ptr [0x40E], #0x9FC0
  ret;;
;--------------------
;- POST: EOI + jmp via [0x40:67)
;--------------------
; relocated here because the primary POST area isnt big enough.
eoi_jmp_post:
  call eoi_both_pics
  xor ax, ax
  mov ds, ax
  jmp far ptr [0x467]
;--------------------
eoi_both_pics:
  mov al, #0x20
  out #0xA0, al ;; slave PIC EOI
eoi_master_pic:
  mov al, #0x20
  out #0x20, al ;; master PIC EOI
  ret
;--------------------
BcdToBin:
  ;; in: AL in BCD format
  ;; out: AL in binary format, AH will always be 0
  ;; trashes BX
  mov bl, al
  and bl, #0x0f ;; bl has low digit
  shr al, #4 ;; al has high digit
  mov bh, #10
  mul al, bh ;; multiply high digit by 10 (result in AX)
  add al, bl ;; then add low digit
  ret
;--------------------
timer_tick_post:
  ;; Setup the Timer Ticks Count (0x46C:dword) and
  ;; Timer Ticks Roller Flag (0x470:byte)
  ;; The Timer Ticks Count needs to be set according to
  ;; the current CMOS time, as if ticks have been occurring
  ;; at 18.2hz since midnight up to this point. Calculating
  ;; this is a little complicated. Here are the factors I gather
  ;; regarding this. 14,318,180 hz was the original clock speed,
  ;; chosen so it could be divided by either 3 to drive the 5Mhz CPU
  ;; at the time, or 4 to drive the CGA video adapter. The div3
  ;; source was divided again by 4 to feed a 1.193Mhz signal to
  ;; the timer. With a maximum 16bit timer count, this is again
  ;; divided down by 65536 to 18.2hz.
  ;;
  ;; 14,318,180 Hz clock
  ;; /3 = 4,772,726 Hz fed to orginal 5Mhz CPU
  ;; /4 = 1,193,181 Hz fed to timer
  ;; /65536 (maximum timer count) = 18.20650736 ticks/second
  ;; 1 second = 18.20650736 ticks
  ;; 1 minute = 1092.390442 ticks
  ;; 1 hour = 65543.42651 ticks
  ;;
  ;; Given the values in the CMOS clock, one could calculate
  ;; the number of ticks by the following:
  ;; ticks = (BcdToBin(seconds) * 18.206507) +
  ;; (BcdToBin(minutes) * 1092.3904)
  ;; (BcdToBin(hours) * 65543.427)
  ;; To get a little more accuracy, since Im using integer
  ;; arithmatic, I use:
  ;; ticks = (BcdToBin(seconds) * 18206507) / 1000000 +
  ;; (BcdToBin(minutes) * 10923904) / 10000 +
  ;; (BcdToBin(hours) * 65543427) / 1000
  ;; assuming DS=0000
  ;; get CMOS seconds
  xor eax, eax ;; clear EAX
  mov al, #0x00
  out #0x70, al
  in al, #0x71 ;; AL has CMOS seconds in BCD
  call BcdToBin ;; EAX now has seconds in binary
  mov edx, #18206507
  mul eax, edx
  mov ebx, #1000000
  xor edx, edx
  div eax, ebx
  mov ecx, eax ;; ECX will accumulate total ticks
  ;; get CMOS minutes
  xor eax, eax ;; clear EAX
  mov al, #0x02
  out #0x70, al
  in al, #0x71 ;; AL has CMOS minutes in BCD
  call BcdToBin ;; EAX now has minutes in binary
  mov edx, #10923904
  mul eax, edx
  mov ebx, #10000
  xor edx, edx
  div eax, ebx
  add ecx, eax ;; add to total ticks
  ;; get CMOS hours
  xor eax, eax ;; clear EAX
  mov al, #0x04
  out #0x70, al
  in al, #0x71 ;; AL has CMOS hours in BCD
  call BcdToBin ;; EAX now has hours in binary
  mov edx, #65543427
  mul eax, edx
  mov ebx, #1000
  xor edx, edx
  div eax, ebx
  add ecx, eax ;; add to total ticks
  mov 0x46C, ecx ;; Timer Ticks Count
  xor al, al
  mov 0x470, al ;; Timer Ticks Rollover Flag
  ret
;--------------------
int76_handler:
  ;; record completion in BIOS task complete flag
  push ax
  push ds
  mov ax, #0x0040
  mov ds, ax
  mov 0x008E, #0xff
  call eoi_both_pics
  pop ds
  pop ax
  iret
;--------------------
use32 386
apm32_out_str:
  push eax
  push ebx
  mov ebx, eax
apm32_out_str1:
  SEG CS
  mov al, byte ptr [bx]
  cmp al, #0
  je apm32_out_str2
  outb dx, al
  inc ebx
  jmp apm32_out_str1
apm32_out_str2:
  pop ebx
  pop eax
  ret
apm32_07_poweroff_str:
  .ascii "Shutdown"
  db 0
apm32_07_suspend_str:
  .ascii "Suspend"
  db 0
apm32_07_standby_str:
  .ascii "Standby"
  db 0
_apm32_entry:
  pushf
;-----------------
; APM interface disconnect
apm32_04:
  cmp al, #0x04
  jne apm32_05
  jmp apm32_ok
;-----------------
; APM cpu idle
apm32_05:
  cmp al, #0x05
  jne apm32_07
  pushf ; XEN
  sti ; XEN: OS calls us with ints disabled -- better re-enable here!
  hlt
  popf ; XEN
  jmp apm32_ok
;-----------------
; APM Set Power State
apm32_07:
  cmp al, #0x07
  jne apm32_08
  cmp bx, #1
  jne apm32_ok
  cmp cx, #3
  je apm32_07_poweroff
  cmp cx, #2
  je apm32_07_suspend
  cmp cx, #1
  je apm32_07_standby
  jne apm32_ok
apm32_07_poweroff:
  cli
  mov dx, #0x8900
  mov ax, #apm32_07_poweroff_str
  call apm32_out_str
apm32_07_1:
  hlt
  jmp apm32_07_1
apm32_07_suspend:
  push edx
  mov dx, #0x8900
  mov ax, #apm32_07_suspend_str
  call apm32_out_str
  pop edx
  jmp apm32_ok
apm32_07_standby:
  push edx
  mov dx, #0x8900
  mov ax, #apm32_07_standby_str
  call apm32_out_str
  pop edx
  jmp apm32_ok
;-----------------
; APM Enable / Disable
apm32_08:
  cmp al, #0x08
  jne apm32_0a
  jmp apm32_ok
;-----------------
; Get Power Status
apm32_0a:
  cmp al, #0x0a
  jne apm32_0b
  mov bh, #0x01
  mov bl, #0xff
  mov ch, #0x80
  mov cl, #0xff
  mov dx, #0xffff
  mov si, #0
  jmp apm32_ok
;-----------------
; Get PM Event
apm32_0b:
  cmp al, #0x0b
  jne apm32_0e
  mov ah, #0x80
  jmp apm32_error
;-----------------
; APM Driver Version
apm32_0e:
  cmp al, #0x0e
  jne apm32_0f
  mov ah, #1
  mov al, #2
  jmp apm32_ok
;-----------------
; APM Engage / Disengage
apm32_0f:
  cmp al, #0x0f
  jne apm32_10
  jmp apm32_ok
;-----------------
; APM Get Capabilities
apm32_10:
  cmp al, #0x10
  jne apm32_unimplemented
  mov bl, #0
  mov cx, #0
  jmp apm32_ok
;-----------------
apm32_ok:
  popf
  clc
  retf
apm32_unimplemented:
apm32_error:
  popf
  stc
  retf
use16 386
apm16_out_str:
  push eax
  push ebx
  mov ebx, eax
apm16_out_str1:
  SEG CS
  mov al, byte ptr [bx]
  cmp al, #0
  je apm16_out_str2
  outb dx, al
  inc ebx
  jmp apm16_out_str1
apm16_out_str2:
  pop ebx
  pop eax
  ret
apm16_07_poweroff_str:
  .ascii "Shutdown"
  db 0
apm16_07_suspend_str:
  .ascii "Suspend"
  db 0
apm16_07_standby_str:
  .ascii "Standby"
  db 0
_apm16_entry:
  pushf
;-----------------
; APM interface disconnect
apm16_04:
  cmp al, #0x04
  jne apm16_05
  jmp apm16_ok
;-----------------
; APM cpu idle
apm16_05:
  cmp al, #0x05
  jne apm16_07
  pushf ; XEN
  sti ; XEN: OS calls us with ints disabled -- better re-enable here!
  hlt
  popf ; XEN
  jmp apm16_ok
;-----------------
; APM Set Power State
apm16_07:
  cmp al, #0x07
  jne apm16_08
  cmp bx, #1
  jne apm16_ok
  cmp cx, #3
  je apm16_07_poweroff
  cmp cx, #2
  je apm16_07_suspend
  cmp cx, #1
  je apm16_07_standby
  jne apm16_ok
apm16_07_poweroff:
  cli
  mov dx, #0x8900
  mov ax, #apm16_07_poweroff_str
  call apm16_out_str
apm16_07_1:
  hlt
  jmp apm16_07_1
apm16_07_suspend:
  push edx
  mov dx, #0x8900
  mov ax, #apm16_07_suspend_str
  call apm16_out_str
  pop edx
  jmp apm16_ok
apm16_07_standby:
  push edx
  mov dx, #0x8900
  mov ax, #apm16_07_standby_str
  call apm16_out_str
  pop edx
  jmp apm16_ok
;-----------------
; APM Enable / Disable
apm16_08:
  cmp al, #0x08
  jne apm16_0a
  jmp apm16_ok
;-----------------
; Get Power Status
apm16_0a:
  cmp al, #0x0a
  jne apm16_0b
  mov bh, #0x01
  mov bl, #0xff
  mov ch, #0x80
  mov cl, #0xff
  mov dx, #0xffff
  mov si, #0
  jmp apm16_ok
;-----------------
; Get PM Event
apm16_0b:
  cmp al, #0x0b
  jne apm16_0e
  mov ah, #0x80
  jmp apm16_error
;-----------------
; APM Driver Version
apm16_0e:
  cmp al, #0x0e
  jne apm16_0f
  mov ah, #1
  mov al, #2
  jmp apm16_ok
;-----------------
; APM Engage / Disengage
apm16_0f:
  cmp al, #0x0f
  jne apm16_10
  jmp apm16_ok
;-----------------
; APM Get Capabilities
apm16_10:
  cmp al, #0x10
  jne apm16_unimplemented
  mov bl, #0
  mov cx, #0
  jmp apm16_ok
;-----------------
apm16_ok:
  popf
  clc
  retf
apm16_unimplemented:
apm16_error:
  popf
  stc
  retf
apmreal_out_str:
  push eax
  push ebx
  mov ebx, eax
apmreal_out_str1:
  SEG CS
  mov al, byte ptr [bx]
  cmp al, #0
  je apmreal_out_str2
  outb dx, al
  inc ebx
  jmp apmreal_out_str1
apmreal_out_str2:
  pop ebx
  pop eax
  ret
apmreal_07_poweroff_str:
  .ascii "Shutdown"
  db 0
apmreal_07_suspend_str:
  .ascii "Suspend"
  db 0
apmreal_07_standby_str:
  .ascii "Standby"
  db 0
  pushf
_apmreal_entry:
;-----------------
; APM installation check
apmreal_00:
  cmp al, #0x00
  jne apmreal_01
  mov ah, #1
  mov al, #2
  mov bh, #0x50
  mov bl, #0x4d
  mov cx, #0x3
  jmp apmreal_ok
;-----------------
; APM real mode interface connect
apmreal_01:
  cmp al, #0x01
  jne apmreal_02
  jmp apmreal_ok
;-----------------
; APM 16 bit protected mode interface connect
apmreal_02:
  cmp al, #0x02
  jne apmreal_03
  mov bx, #_apm16_entry
  mov ax, #0xf000
  mov si, #0xfff0
  mov cx, #0xf000
  mov di, #0xfff0
  jmp apmreal_ok
;-----------------
; APM 32 bit protected mode interface connect
apmreal_03:
  cmp al, #0x03
  jne apmreal_04
  mov ax, #0xf000
  mov ebx, #_apm32_entry
  mov cx, #0xf000
  mov esi, #0xfff0fff0
  mov dx, #0xf000
  mov di, #0xfff0
  jmp apmreal_ok
;-----------------
; APM interface disconnect
apmreal_04:
  cmp al, #0x04
  jne apmreal_05
  jmp apmreal_ok
;-----------------
; APM cpu idle
apmreal_05:
  cmp al, #0x05
  jne apmreal_07
  pushf ; XEN
  sti ; XEN: OS calls us with ints disabled -- better re-enable here!
  hlt
  popf ; XEN
  jmp apmreal_ok
;-----------------
; APM Set Power State
apmreal_07:
  cmp al, #0x07
  jne apmreal_08
  cmp bx, #1
  jne apmreal_ok
  cmp cx, #3
  je apmreal_07_poweroff
  cmp cx, #2
  je apmreal_07_suspend
  cmp cx, #1
  je apmreal_07_standby
  jne apmreal_ok
apmreal_07_poweroff:
  cli
  mov dx, #0x8900
  mov ax, #apmreal_07_poweroff_str
  call apmreal_out_str
apmreal_07_1:
  hlt
  jmp apmreal_07_1
apmreal_07_suspend:
  push edx
  mov dx, #0x8900
  mov ax, #apmreal_07_suspend_str
  call apmreal_out_str
  pop edx
  jmp apmreal_ok
apmreal_07_standby:
  push edx
  mov dx, #0x8900
  mov ax, #apmreal_07_standby_str
  call apmreal_out_str
  pop edx
  jmp apmreal_ok
;-----------------
; APM Enable / Disable
apmreal_08:
  cmp al, #0x08
  jne apmreal_0a
  jmp apmreal_ok
;-----------------
; Get Power Status
apmreal_0a:
  cmp al, #0x0a
  jne apmreal_0b
  mov bh, #0x01
  mov bl, #0xff
  mov ch, #0x80
  mov cl, #0xff
  mov dx, #0xffff
  mov si, #0
  jmp apmreal_ok
;-----------------
; Get PM Event
apmreal_0b:
  cmp al, #0x0b
  jne apmreal_0e
  mov ah, #0x80
  jmp apmreal_error
;-----------------
; APM Driver Version
apmreal_0e:
  cmp al, #0x0e
  jne apmreal_0f
  mov ah, #1
  mov al, #2
  jmp apmreal_ok
;-----------------
; APM Engage / Disengage
apmreal_0f:
  cmp al, #0x0f
  jne apmreal_10
  jmp apmreal_ok
;-----------------
; APM Get Capabilities
apmreal_10:
  cmp al, #0x10
  jne apmreal_unimplemented
  mov bl, #0
  mov cx, #0
  jmp apmreal_ok
;-----------------
apmreal_ok:
  popf
  clc
  jmp iret_modify_cf
apmreal_unimplemented:
apmreal_error:
  popf
  stc
  jmp iret_modify_cf
! 6286 endasm
!BCC_ENDASM
! 6287 #asm
!BCC_ASM
_int70_function.ds	set	$12
_int70_function.iret_addr	set	$14
_int70_function.regs	set	2
    ; Switch into protected mode to allow access to 32 bit addresses.
    ; This function allows switching into protected mode.
    ; (the specs says big real mode, but that will not work)
    ;
    ; preserves all registers and prepares cs, ds, es, ss for usage
    ; in protected mode; while in prot.mode interrupts remain disabled
switch_to_protmode:
    cli
    ; have to fix the stack for proper return address in 32 bit mode
    push WORD #(0xf000>>12) ;extended return address
    push bp ;pop@A1
    mov bp, sp
    push eax ;pop@A2
    mov eax, 2[bp] ; fix return address
    rol eax, #16
    mov 2[bp], eax
    mov eax, esp
    ror eax, #16 ; hi(esp)
    push bx ; preserve before function call
    push cx
    push dx
    push ax ; prepare stack for
    push es ; call
    push ds
    push cs
    push ss
    call _store_segment_registers
    add sp, #10 ; pop ax,es-ss
    pop dx ; restore after function call
    pop cx
    pop bx
    ; calculate protected-mode esp from ss:sp
    and esp, #0xffff
    xor eax, eax
    mov ax, ss
    rol eax, #4
    add eax, esp
    mov esp, eax
    seg cs
    lgdt my_gdtdesc ; switch to own table
    mov eax, cr0
    or al, #0x1 ; protected mode 'on'
    mov cr0, eax
    jmpf DWORD (0xf0000 | switch_to_protmode_goon_1), #(gdt_entry_pm_cs - gdt_base)
    USE32
switch_to_protmode_goon_1:
    mov ax, #(gdt_entry_pm_32bit_ds - gdt_base) ; 32 bit segment that allows
    mov ds, ax ; to reach all 32 bit
    mov es, ax ; addresses
    mov ss, ax
    pop eax ;@A2
    pop bp ;@A1
    ret
    USE16
    .align 16
gdt_base:
    ; see Intel SW Dev. Manuals section 3.4.5, Volume 3 for meaning of bits
    .word 0,0
    .byte 0,0,0,0
gdt_entry_pm_cs:
    ; 32 bit code segment for protected mode
    .word 0xffff, 0x0000
    .byte 0x00, 0x9a, 0xcf, 0x00
gdt_entry_pm_16bit_cs:
    ; temp. 16 bit code segment used while in protected mode
    .word 0xffff, 0x0000
    .byte 0xf0000 >> 16, 0x9a, 0x0, 0x0
gdt_entry_pm_32bit_ds:
    ; (32 bit) data segment (r/w) reaching all possible areas in 32bit memory
    ; 4kb granularity
    .word 0xffff, 0x0000
    .byte 0x0, 0x92, 0xcf, 0x0
gdt_entry_end:
my_gdtdesc:
    .word (gdt_entry_end - gdt_base) - 1
    .long gdt_base | 0xf0000
realmode_gdtdesc: ;to be used in real mode
    .word 0xffff
    .long 0x0
switch_to_realmode:
    ; Implementation of switching from protected mode to real mode
    ; prepares cs, es, ds, ss to be used in real mode
    ; spills eax
    USE32
    ; need to fix up the stack to return in 16 bit mode
    ; currently the 32 bit return address is on the stack
    pop eax
    push ax
    push bx ;pop@1
    push si ;pop@2
    call _ebda_ss_offset32 ; get the offset of the ss
    mov bx, ax ; entry within the ebda.
    jmpf switch_to_realmode_goon_1, #(gdt_entry_pm_16bit_cs - gdt_base)
    USE16
switch_to_realmode_goon_1:
    mov eax, cr0
    and al, #0xfe ; protected mode 'off'
    mov cr0, eax
    jmpf switch_to_realmode_goon_2, #0xf000
switch_to_realmode_goon_2:
    ; get orig. 'ss' without using the stack (no 'call'!)
    xor eax, eax ; clear upper 16 bits (and lower)
    mov ax, #0x40 ; where is the ebda located?
    mov ds, ax
    mov si, #0xe
    seg ds
    mov ax, [si] ; ax = segment of ebda
    mov ds, ax ; segment of ebda
    seg ds
    mov ax, [bx] ; stack segment - bx has been set above
    mov ss, ax
    ; from esp and ss calculate real-mode sp
    rol eax, #4
    sub esp, eax
    push dx ;preserve before call(s)
    push cx
    push bx
    call _get_register_ds ; get orig. 'ds'
    mov ds, ax
    call _get_register_es ; get orig. 'es'
    mov es, ax
    call _get_register_esp_hi ; fix the upper 16 bits of esp
    ror esp, #16
    mov sp, ax
    rol esp, #16
    pop bx
    pop cx
    pop dx
    seg cs
    lgdt realmode_gdtdesc
    sti ; allow interrupts
    pop si ;@2
    pop bx ;@1
    ret
! 6422 endasm
!BCC_ENDASM
! 6423 Bit16u
! Register BX used in function int70_function
! 6424 ebda_ss_offset32()
! 6425 {
export	_ebda_ss_offset32
_ebda_ss_offset32:
! 6426 #asm
!BCC_ASM
    USE32
! 6428 endasm
!BCC_ENDASM
! 6429     return &((ebda_data_t *) 0)->upcall.reg_ss;
push	bp
mov	bp,sp
mov	ax,#$250
pop	bp
ret
!BCC_EOS
! 6430 #asm
!BCC_EOS
!BCC_ASM
    USE16
! 6432 endasm
!BCC_ENDASM
!BCC_EOS
! 6433 }
! 6434 Bit16u
! 6435 read_word_from_ebda(offset)
! 6436     Bit16u offset;
export	_read_word_from_ebda
_read_word_from_ebda:
!BCC_EOS
! 6437 {
! 6438  Bit16u ebda_seg = read_word(0x0040, 0x000E);
push	bp
mov	bp,sp
dec	sp
dec	sp
! Debug: list int = const $E (used reg = )
mov	ax,*$E
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () unsigned short = read_word+0 (used reg = )
call	_read_word
add	sp,*4
! Debug: eq unsigned short = ax+0 to unsigned short ebda_seg = [S+4-4] (used reg = )
mov	-2[bp],ax
!BCC_EOS
! 6439  return read_word(ebda_seg, offset);
! Debug: list unsigned short offset = [S+4+2] (used reg = )
push	4[bp]
! Debug: list unsigned short ebda_seg = [S+6-4] (used reg = )
push	-2[bp]
! Debug: func () unsigned short = read_word+0 (used reg = )
call	_read_word
add	sp,*4
! Debug: cast unsigned short = const 0 to unsigned short = ax+0 (used reg = )
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 6440 }
! 6441 Bit32u
! 6442 read_dword_from_ebda(offset)
! 6443     Bit16u offset;
export	_read_dword_from_ebda
_read_dword_from_ebda:
!BCC_EOS
! 6444 {
! 6445  Bit16u ebda_seg = read_word(0x0040, 0x000E);
push	bp
mov	bp,sp
dec	sp
dec	sp
! Debug: list int = const $E (used reg = )
mov	ax,*$E
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () unsigned short = read_word+0 (used reg = )
call	_read_word
add	sp,*4
! Debug: eq unsigned short = ax+0 to unsigned short ebda_seg = [S+4-4] (used reg = )
mov	-2[bp],ax
!BCC_EOS
! 6446  return read_dword(ebda_seg, offset);
! Debug: list unsigned short offset = [S+4+2] (used reg = )
push	4[bp]
! Debug: list unsigned short ebda_seg = [S+6-4] (used reg = )
push	-2[bp]
! Debug: func () unsigned long = read_dword+0 (used reg = )
call	_read_dword
mov	bx,dx
add	sp,*4
! Debug: cast unsigned long = const 0 to unsigned long = bx+0 (used reg = )
mov	dx,bx
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 6447 }
! 6448   void
! Register BX used in function read_dword_from_ebda
! 6449 store_segment_registers(ss, cs, ds, es, esp_hi)
! 6450   Bit16u ss, cs, ds, es, esp_hi;
export	_store_segment_registers
_store_segment_registers:
!BCC_EOS
! 6451 {
! 6452  Bit16u ebda_seg = read_word(0x0040, 0x000E);
push	bp
mov	bp,sp
dec	sp
dec	sp
! Debug: list int = const $E (used reg = )
mov	ax,*$E
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () unsigned short = read_word+0 (used reg = )
call	_read_word
add	sp,*4
! Debug: eq unsigned short = ax+0 to unsigned short ebda_seg = [S+4-4] (used reg = )
mov	-2[bp],ax
!BCC_EOS
! 6453  write_word(ebda_seg, &((ebda_data_t *) 0)->upcall.reg_ss, ss);
! Debug: list unsigned short ss = [S+4+2] (used reg = )
push	4[bp]
! Debug: list * unsigned short = const $250 (used reg = )
mov	ax,#$250
push	ax
! Debug: list unsigned short ebda_seg = [S+8-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
add	sp,*6
!BCC_EOS
! 6454  write_word(ebda_seg, &((ebda_data_t *) 0)->upcall.reg_cs, cs);
! Debug: list unsigned short cs = [S+4+4] (used reg = )
push	6[bp]
! Debug: list * unsigned short = const $252 (used reg = )
mov	ax,#$252
push	ax
! Debug: list unsigned short ebda_seg = [S+8-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
add	sp,*6
!BCC_EOS
! 6455  write_word(ebda_seg, &((ebda_data_t *) 0)->upcall.reg_ds, ds);
! Debug: list unsigned short ds = [S+4+6] (used reg = )
push	8[bp]
! Debug: list * unsigned short = const $254 (used reg = )
mov	ax,#$254
push	ax
! Debug: list unsigned short ebda_seg = [S+8-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
add	sp,*6
!BCC_EOS
! 6456  write_word(ebda_seg, &((ebda_data_t *) 0)->upcall.reg_es, es);
! Debug: list unsigned short es = [S+4+8] (used reg = )
push	$A[bp]
! Debug: list * unsigned short = const $256 (used reg = )
mov	ax,#$256
push	ax
! Debug: list unsigned short ebda_seg = [S+8-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
add	sp,*6
!BCC_EOS
! 6457  write_word(ebda_seg, &((ebda_data_t *) 0)->upcall.esp_hi, esp_hi);
! Debug: list unsigned short esp_hi = [S+4+$A] (used reg = )
push	$C[bp]
! Debug: list * unsigned short = const $258 (used reg = )
mov	ax,#$258
push	ax
! Debug: list unsigned short ebda_seg = [S+8-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
add	sp,*6
!BCC_EOS
! 6458 }
mov	sp,bp
pop	bp
ret
! 6459   void
! 6460 store_returnaddress(retaddr)
! 6461    Bit16u retaddr;
export	_store_returnaddress
_store_returnaddress:
!BCC_EOS
! 6462 {
! 6463  Bit16u ebda_seg = read_word(0x0040, 0x000E);
push	bp
mov	bp,sp
dec	sp
dec	sp
! Debug: list int = const $E (used reg = )
mov	ax,*$E
push	ax
! Debug: list int = const $40 (used reg = )
mov	ax,*$40
push	ax
! Debug: func () unsigned short = read_word+0 (used reg = )
call	_read_word
add	sp,*4
! Debug: eq unsigned short = ax+0 to unsigned short ebda_seg = [S+4-4] (used reg = )
mov	-2[bp],ax
!BCC_EOS
! 6464  write_word(ebda_seg, &((ebda_data_t *) 0)->upcall.retaddr, retaddr);
! Debug: list unsigned short retaddr = [S+4+2] (used reg = )
push	4[bp]
! Debug: list * unsigned short = const $25A (used reg = )
mov	ax,#$25A
push	ax
! Debug: list unsigned short ebda_seg = [S+8-4] (used reg = )
push	-2[bp]
! Debug: func () void = write_word+0 (used reg = )
call	_write_word
add	sp,*6
!BCC_EOS
! 6465 }
mov	sp,bp
pop	bp
ret
! 6466 Bit16u
! 6467 get_returnaddress()
! 6468 {
export	_get_returnaddress
_get_returnaddress:
! 6469  
! 6469 return read_word_from_ebda(&((ebda_data_t *) 0)->upcall.retaddr);
push	bp
mov	bp,sp
! Debug: list * unsigned short = const $25A (used reg = )
mov	ax,#$25A
push	ax
! Debug: func () unsigned short = read_word_from_ebda+0 (used reg = )
call	_read_word_from_ebda
mov	sp,bp
! Debug: cast unsigned short = const 0 to unsigned short = ax+0 (used reg = )
pop	bp
ret
!BCC_EOS
! 6470 }
! 6471 Bit16u
! 6472 get_register_cs()
! 6473 {
export	_get_register_cs
_get_register_cs:
! 6474  return read_word_from_ebda(&((ebda_data_t *) 0)->upcall.reg_cs);
push	bp
mov	bp,sp
! Debug: list * unsigned short = const $252 (used reg = )
mov	ax,#$252
push	ax
! Debug: func () unsigned short = read_word_from_ebda+0 (used reg = )
call	_read_word_from_ebda
mov	sp,bp
! Debug: cast unsigned short = const 0 to unsigned short = ax+0 (used reg = )
pop	bp
ret
!BCC_EOS
! 6475 }
! 6476 Bit16u
! 6477 get_register_ds()
! 6478 {
export	_get_register_ds
_get_register_ds:
! 6479  return read_word_from_ebda(&((ebda_data_t *) 0)->upcall.reg_ds);
push	bp
mov	bp,sp
! Debug: list * unsigned short = const $254 (used reg = )
mov	ax,#$254
push	ax
! Debug: func () unsigned short = read_word_from_ebda+0 (used reg = )
call	_read_word_from_ebda
mov	sp,bp
! Debug: cast unsigned short = const 0 to unsigned short = ax+0 (used reg = )
pop	bp
ret
!BCC_EOS
! 6480 }
! 6481 Bit16u
! 6482 get_register_es()
! 6483 {
export	_get_register_es
_get_register_es:
! 6484  return read_word_from_ebda(&((ebda_data_t *) 0)->upcall.reg_es);
push	bp
mov	bp,sp
! Debug: list * unsigned short = const $256 (used reg = )
mov	ax,#$256
push	ax
! Debug: func () unsigned short = read_word_from_ebda+0 (used reg = )
call	_read_word_from_ebda
mov	sp,bp
! Debug: cast unsigned short = const 0 to unsigned short = ax+0 (used reg = )
pop	bp
ret
!BCC_EOS
! 6485 }
! 6486 Bit16u
! 6487 get_register_esp_hi()
! 6488 {
export	_get_register_esp_hi
_get_register_esp_hi:
! 6489  return read_word_from_ebda(&((ebda_data_t *) 0)->upcall.esp_hi);
push	bp
mov	bp,sp
! Debug: list * unsigned short = const $258 (used reg = )
mov	ax,#$258
push	ax
! Debug: func () unsigned short = read_word_from_ebda+0 (used reg = )
call	_read_word_from_ebda
mov	sp,bp
! Debug: cast unsigned short = const 0 to unsigned short = ax+0 (used reg = )
pop	bp
ret
!BCC_EOS
! 6490 }
! 6491 #asm
!BCC_ASM
Upcall:
 ; do the upcall into 32 bit space
 ; clear the stack frame so that 32 bit space sees all the parameters
 ; on the stack as if they were prepared for it
 ; ---> take the 16 bit return address off the stack and remember it
 ;
 ; Input:
 ; bx: index of function to call
 ; Ouput:
 ; dx, ax: 32 bit result of call (even if 'void' is expected)
 push bp ;pop @1
 mov bp, sp
 push si ;pop @2
 mov ax, 2[bp] ; 16 bit return address
 push ax
 call _store_returnaddress ; store away
 pop ax
 ; XXX GDT munging requires ROM to be writable!
 call _enable_rom_write_access
 rol bx, #2
 mov si, #jmptable
 seg cs
 mov eax, dword ptr [si+bx] ; address to call from table
 pop si ;@2
 pop bp ;@1
 add sp, #2 ; remove 16bit return address from stack
 call switch_to_protmode
 USE32
 call eax ; call 32bit function
 push eax ; preserve result
 call switch_to_realmode ; back to realmode
 USE16
 pop eax ; get result
 push word 0x0000 ; placeholder for 16 bit return address
 push bp
 mov bp,sp
 push eax ; preserve work register
 call _disable_rom_write_access
 call _get_returnaddress
 mov 2[bp], ax ; 16bit return address onto stack
 pop eax
 pop bp
 ror eax, #16 ; result into dx/ax
 mov dx, ax ; hi(res) -> dx
 ror eax, #16
 ret
MACRO DoUpcall
 mov bx, #?1
 jmp Upcall
MEND
! 6542 endasm
!BCC_ENDASM
! 6543 Bit32u tcpa_extend_acpi_log(entry_ptr)
! 6544     Bit32u entry_ptr;
export	_tcpa_extend_acpi_log
_tcpa_extend_acpi_log:
!BCC_EOS
! 6545 {
! 6546 #asm
!BCC_ASM
_tcpa_extend_acpi_log.entry_ptr	set	2
 DoUpcall(2)
! 6548 endasm
!BCC_ENDASM
! 6549 }
ret
! 6550  void
! 6551 tcpa_acpi_init()
! 6552 {
export	_tcpa_acpi_init
_tcpa_acpi_init:
! 6553 #asm
!BCC_ASM
 DoUpcall(1)
! 6555 endasm
!BCC_ENDASM
! 6556 }
ret
! 6557  void
! 6558 tcpa_calling_int19h()
! 6559 {
export	_tcpa_calling_int19h
_tcpa_calling_int19h:
! 6560 #asm
!BCC_ASM
 DoUpcall(3)
! 6562 endasm
!BCC_ENDASM
! 6563 }
ret
! 6564  void
! 6565 tcpa_returned_int19h()
! 6566 {
export	_tcpa_returned_int19h
_tcpa_returned_int19h:
! 6567 #asm
!BCC_ASM
 DoUpcall(4)
! 6569 endasm
!BCC_ENDASM
! 6570 }
ret
! 6571  void
! 6572 tcpa_add_event_separators()
! 6573 {
export	_tcpa_add_event_separators
_tcpa_add_event_separators:
! 6574 #asm
!BCC_ASM
 DoUpcall(5)
! 6576 endasm
!BCC_ENDASM
! 6577 }
ret
! 6578  void
! 6579 tcpa_wake_event()
! 6580 {
export	_tcpa_wake_event
_tcpa_wake_event:
! 6581 #asm
!BCC_ASM
 DoUpcall(6)
! 6583 endasm
!BCC_ENDASM
! 6584 }
ret
! 6585  void
! 6586 tcpa_start_option_rom_scan()
! 6587 {
export	_tcpa_start_option_rom_scan
_tcpa_start_option_rom_scan:
! 6588 #asm
!BCC_ASM
 DoUpcall(8)
! 6590 endasm
!BCC_ENDASM
! 6591 }
ret
! 6592  void
! 6593 tcpa_option_rom(seg)
! 6594     Bit32u seg;
export	_tcpa_option_rom
_tcpa_option_rom:
!BCC_EOS
! 6595 {
! 6596 #asm
!BCC_ASM
_tcpa_option_rom.seg	set	2
 DoUpcall(9)
! 6598 endasm
!BCC_ENDASM
! 6599 }
ret
! 6600 void
! 6601  tcpa_add_bootdevice(bootcd, bootdrv)
! 6602   Bit32u bootcd;
export	_tcpa_add_bootdevice
_tcpa_add_bootdevice:
!BCC_EOS
! 6603   Bit32u bootdrv;
!BCC_EOS
! 6604 {
! 6605 #asm
!BCC_ASM
_tcpa_add_bootdevice.bootcd	set	2
_tcpa_add_bootdevice.bootdrv	set	6
 DoUpcall(7)
! 6607 endasm
!BCC_ENDASM
! 6608 }
ret
! 6609  void
! 6610  tcpa_ipl(bootcd,seg,off,count)
! 6611     Bit32u bootcd;
export	_tcpa_ipl
_tcpa_ipl:
!BCC_EOS
! 6612     Bit32u seg;
!BCC_EOS
! 6613     Bit32u off;
!BCC_EOS
! 6614     Bit32u count;
!BCC_EOS
! 6615 {
! 6616 #asm
!BCC_ASM
_tcpa_ipl.count	set	$E
_tcpa_ipl.seg	set	6
_tcpa_ipl.bootcd	set	2
_tcpa_ipl.off	set	$A
 DoUpcall(10)
! 6618 endasm
!BCC_ENDASM
! 6619 }
ret
! 6620 Bit32u
! 6621 tcpa_initialize_tpm(physpres)
! 6622   Bit32u physpres;
export	_tcpa_initialize_tpm
_tcpa_initialize_tpm:
!BCC_EOS
! 6623 {
! 6624 #asm
!BCC_ASM
_tcpa_initialize_tpm.physpres	set	2
 DoUpcall(11)
! 6626 endasm
!BCC_ENDASM
! 6627 }
ret
! 6628 void
! 6629 tcpa_measure_post(from, to)
! 6630    Bit32u from;
export	_tcpa_measure_post
_tcpa_measure_post:
!BCC_EOS
! 6631    Bit32u to;
!BCC_EOS
! 6632 {
! 6633 #asm
!BCC_ASM
_tcpa_measure_post.from	set	2
_tcpa_measure_post.to	set	6
 DoUpcall(12)
! 6635 endasm
!BCC_ENDASM
! 6636 }
ret
! 6637 #asm
!BCC_ASM
_tcpa_measure_post.from	set	2
_tcpa_measure_post.to	set	6
MACRO POST_MEASURE
 push word #0x000f
 push #?2
 push word #0x000f
 push #?1
 call _tcpa_measure_post
 add sp, #8
MEND
! 6646 endasm
!BCC_ENDASM
! 6647 void
! 6648 tcpa_do_measure_POSTs()
! 6649 {
export	_tcpa_do_measure_POSTs
_tcpa_do_measure_POSTs:
! 6650 #asm
!BCC_ASM
 POST_MEASURE(post, nmi)
 POST_MEASURE(floppy_drive_post, hard_drive_post)
 POST_MEASURE(hard_drive_post, ebda_post)
 POST_MEASURE(ebda_post, eoi_jmp_post)
 POST_MEASURE(eoi_jmp_post, timer_tick_post)
 POST_MEASURE(timer_tick_post, int76_handler)
 ret
! 6658 endasm
!BCC_ENDASM
! 6659 }
ret
! 6660 Bit32u
! 6661 TCGInterruptHandler(regs_ptr, es, ds, flags_ptr)
! 6662    Bit32u regs_ptr;
export	_TCGInterruptHandler
_TCGInterruptHandler:
!BCC_EOS
! 6663    Bit16u es;
!BCC_EOS
! 6664    Bit16u ds;
!BCC_EOS
! 6665    Bit32u flags_ptr;
!BCC_EOS
! 6666 {
! 6667 #asm
!BCC_ASM
_TCGInterruptHandler.flags_ptr	set	$A
_TCGInterruptHandler.regs_ptr	set	2
_TCGInterruptHandler.ds	set	8
_TCGInterruptHandler.es	set	6
 DoUpcall(0)
! 6669 endasm
!BCC_ENDASM
! 6670 }
ret
! 6671   void
! 6672 int1a_function32(regs, ES, DS, FLAGS)
! 6673   pushad_regs_t regs;
export	_int1a_function32
_int1a_function32:
!BCC_EOS
! 6674   Bit16u ES, DS, FLAGS;
!BCC_EOS
! 6675 {
! 6676  Bit16u rc;
!BCC_EOS
! 6677  ;
push	bp
mov	bp,sp
dec	sp
dec	sp
!BCC_EOS
! 6678  switch (regs.u.r8.ah) {
mov	al,$21[bp]
br 	.70A
! 6679  case 0xbb:
! 6680   if (regs.u.r8.al != 0 &&
.70B:
! 6681       regs.u.r32.ebx != 0x41504354L) {
! Debug: ne int = const 0 to unsigned char regs = [S+4+$1E] (used reg = )
mov	al,$20[bp]
test	al,al
je  	.70C
.70E:
! Debug: ne long = const $41504354 to unsigned long regs = [S+4+$12] (used reg = )
! Debug: expression subtree swapping
mov	ax,#$4354
mov	bx,#$4150
push	bx
push	ax
mov	ax,$14[bp]
mov	bx,$16[bp]
lea	di,-2+..FFD8[bp]
call	lcmpul
lea	sp,2+..FFD8[bp]
je  	.70C
.70D:
! 6682       FLAGS |= 0x0001;
! Debug: orab int = const 1 to unsigned short FLAGS = [S+4+$26] (used reg = )
mov	ax,$28[bp]
or	al,*1
mov	$28[bp],ax
!BCC_EOS
! 6683       return;
mov	sp,bp
pop	bp
ret
!BCC_EOS
! 6684   }
! 6685   switch(regs.u.r8.al) {
.70C:
mov	al,$20[bp]
jmp .711
! 6686   case 0x00:
! 6687   case 0x01:
.712:
! 6688   case 0x02:
.713:
! 6689   case 0x03:
.714:
! 6690   case 0x04:
.715:
! 6691   case 0x05:
.716:
! 6692   case 0x06:
.717:
! 6693   case 0x07:
.718:
! 6694    TCGInterruptHandler(((Bit32u)get_SS() << 4) + (Bit32u)&regs,
.719:
! 6695                        ES, DS,
! 6696                        ((Bit32u)get_SS() << 4) + (Bit32u)&FLAGS);
! Debug: expression subtree swapping
! Debug: func () unsigned short = get_SS+0 (used reg = )
call	_get_SS
! Debug: cast unsigned long = const 0 to unsigned short = ax+0 (used reg = )
xor	bx,bx
! Debug: sl int = const 4 to unsigned long = bx+0 (used reg = )
mov	di,*4
call	lslul
push	bx
push	ax
! Debug: cast unsigned long = const 0 to * unsigned short FLAGS = S+8+$26 (used reg = )
mov	ax,bp
add	ax,*$28
xor	bx,bx
! Debug: add unsigned long (temp) = [S+8-8] to unsigned long = bx+0 (used reg = )
lea	di,-2+..FFD8[bp]
call	laddul
add	sp,*4
! Debug: list unsigned long = bx+0 (used reg = )
push	bx
push	ax
! Debug: list unsigned short DS = [S+8+$24] (used reg = )
push	$26[bp]
! Debug: list unsigned short ES = [S+$A+$22] (used reg = )
push	$24[bp]
! Debug: expression subtree swapping
! Debug: func () unsigned short = get_SS+0 (used reg = )
call	_get_SS
! Debug: cast unsigned long = const 0 to unsigned short = ax+0 (used reg = )
xor	bx,bx
! Debug: sl int = const 4 to unsigned long = bx+0 (used reg = )
mov	di,*4
call	lslul
push	bx
push	ax
! Debug: cast unsigned long = const 0 to * struct  regs = S+$10+2 (used reg = )
mov	ax,bp
add	ax,*4
xor	bx,bx
! Debug: add unsigned long (temp) = [S+$10-$10] to unsigned long = bx+0 (used reg = )
lea	di,-$A+..FFD8[bp]
call	laddul
add	sp,*4
! Debug: list unsigned long = bx+0 (used reg = )
push	bx
push	ax
! Debug: func () unsigned long = TCGInterruptHandler+0 (used reg = )
call	_TCGInterruptHandler
mov	bx,dx
add	sp,*$C
!BCC_EOS
! 6697    break;
jmp .70F
!BCC_EOS
! 6698   default:
! 6699    FLAGS |= 0x0001;
.71A:
! Debug: orab int = const 1 to unsigned short FLAGS = [S+4+$26] (used reg = )
mov	ax,$28[bp]
or	al,*1
mov	$28[bp],ax
!BCC_EOS
! 6700   }
! 6701   break;
jmp .70F
.711:
sub	al,*0
jb 	.71A
cmp	al,*7
ja  	.71B
xor	ah,ah
shl	ax,*1
mov	bx,ax
seg	cs
br	.71C[bx]
.71C:
.word	.712
.word	.713
.word	.714
.word	.715
.word	.716
.word	.717
.word	.718
.word	.719
.71B:
jmp	.71A
.70F:
jmp .708
!BCC_EOS
! 6702  default:
! 6703   FLAGS |= 0x0001;
.71D:
! Debug: orab int = const 1 to unsigned short FLAGS = [S+4+$26] (used reg = )
mov	ax,$28[bp]
or	al,*1
mov	$28[bp],ax
!BCC_EOS
! 6704   break;
jmp .708
!BCC_EOS
! 6705  }
! 6706  ;
jmp .708
.70A:
sub	al,#$BB
beq 	.70B
jmp	.71D
.708:
..FFD8	=	-4
!BCC_EOS
! 6707 }
mov	sp,bp
pop	bp
ret
! 6708 Bit32u get_s3_waking_vector()
! Register BX used in function int1a_function32
! 6709 {
export	_get_s3_waking_vector
_get_s3_waking_vector:
! 6710 #asm
!BCC_ASM
 DoUpcall(13)
! 6712 endasm
!BCC_ENDASM
! 6713 }
ret
! 6714 #asm
!BCC_ASM
;--------------------
use32 386
.align 16
bios32_structure:
  db 0x5f, 0x33, 0x32, 0x5f ;; "_32_" signature
  dw bios32_entry_point, 0xf ;; 32 bit physical address
  db 0 ;; revision level
  ;; length in paragraphs and checksum stored in a word to prevent errors
  dw (~(((bios32_entry_point >> 8) + (bios32_entry_point & 0xff) + 0x32) & 0xff) << 8) + 0x01
  db 0,0,0,0,0 ;; reserved
.align 16
bios32_entry_point:
  pushf
  cmp eax, #0x49435024
  jne unknown_service
  mov eax, #0x80000000
  mov dx, #0x0cf8
  out dx, eax
  mov dx, #0x0cfc
  in eax, dx
  cmp eax, #0x12378086
  jne unknown_service
  mov ebx, #0x000f0000
  mov ecx, #0
  mov edx, #pcibios_protected
  xor al, al
  jmp bios32_end
unknown_service:
  mov al, #0x80
bios32_end:
  popf
  retf
.align 16
pcibios_protected:
  pushf
  cli
  push esi
  push edi
  cmp al, #0x01 ;; installation check
  jne pci_pro_f02
  mov bx, #0x0210
  mov cx, #0
  mov edx, #0x20494350
  mov al, #0x01
  jmp pci_pro_ok
pci_pro_f02: ;; find pci device
  cmp al, #0x02
  jne pci_pro_f08
  shl ecx, #16
  mov cx, dx
  mov bx, #0x0000
  mov di, #0x00
pci_pro_devloop:
  call pci_pro_select_reg
  mov dx, #0x0cfc
  in eax, dx
  cmp eax, ecx
  jne pci_pro_nextdev
  cmp si, #0
  je pci_pro_ok
  dec si
pci_pro_nextdev:
  inc bx
  cmp bx, #0x0100
  jne pci_pro_devloop
  mov ah, #0x86
  jmp pci_pro_fail
pci_pro_f08: ;; read configuration byte
  cmp al, #0x08
  jne pci_pro_f09
  call pci_pro_select_reg
  push edx
  mov dx, di
  and dx, #0x03
  add dx, #0x0cfc
  in al, dx
  pop edx
  mov cl, al
  jmp pci_pro_ok
pci_pro_f09: ;; read configuration word
  cmp al, #0x09
  jne pci_pro_f0a
  call pci_pro_select_reg
  push edx
  mov dx, di
  and dx, #0x02
  add dx, #0x0cfc
  in ax, dx
  pop edx
  mov cx, ax
  jmp pci_pro_ok
pci_pro_f0a: ;; read configuration dword
  cmp al, #0x0a
  jne pci_pro_f0b
  call pci_pro_select_reg
  push edx
  mov dx, #0x0cfc
  in eax, dx
  pop edx
  mov ecx, eax
  jmp pci_pro_ok
pci_pro_f0b: ;; write configuration byte
  cmp al, #0x0b
  jne pci_pro_f0c
  call pci_pro_select_reg
  push edx
  mov dx, di
  and dx, #0x03
  add dx, #0x0cfc
  mov al, cl
  out dx, al
  pop edx
  jmp pci_pro_ok
pci_pro_f0c: ;; write configuration word
  cmp al, #0x0c
  jne pci_pro_f0d
  call pci_pro_select_reg
  push edx
  mov dx, di
  and dx, #0x02
  add dx, #0x0cfc
  mov ax, cx
  out dx, ax
  pop edx
  jmp pci_pro_ok
pci_pro_f0d: ;; write configuration dword
  cmp al, #0x0d
  jne pci_pro_unknown
  call pci_pro_select_reg
  push edx
  mov dx, #0x0cfc
  mov eax, ecx
  out dx, eax
  pop edx
  jmp pci_pro_ok
pci_pro_unknown:
  mov ah, #0x81
pci_pro_fail:
  pop edi
  pop esi
  sti
  popf
  stc
  retf
pci_pro_ok:
  xor ah, ah
  pop edi
  pop esi
  sti
  popf
  clc
  retf
pci_pro_select_reg:
  push edx
  mov eax, #0x800000
  mov ax, bx
  shl eax, #8
  and di, #0xff
  or ax, di
  and al, #0xfc
  mov dx, #0x0cf8
  out dx, eax
  pop edx
  ret
use16 386
pcibios_real:
  push eax
  push dx
  mov eax, #0x80000000
  mov dx, #0x0cf8
  out dx, eax
  mov dx, #0x0cfc
  in eax, dx
  cmp eax, #0x12378086
  je pci_present
  pop dx
  pop eax
  mov ah, #0xff
  stc
  ret
pci_present:
  pop dx
  pop eax
  cmp al, #0x01 ;; installation check
  jne pci_real_f02
  mov ax, #0x0001
  mov bx, #0x0210
  mov cx, #0
  mov edx, #0x20494350
  mov edi, #0xf0000
  mov di, #pcibios_protected
  clc
  ret
pci_real_f02: ;; find pci device
  push esi
  push edi
  cmp al, #0x02
  jne pci_real_f08
  shl ecx, #16
  mov cx, dx
  mov bx, #0x0000
  mov di, #0x00
pci_real_devloop:
  call pci_real_select_reg
  mov dx, #0x0cfc
  in eax, dx
  cmp eax, ecx
  jne pci_real_nextdev
  cmp si, #0
  je pci_real_ok
  dec si
pci_real_nextdev:
  inc bx
  cmp bx, #0x0100
  jne pci_real_devloop
  mov dx, cx
  shr ecx, #16
  mov ah, #0x86
  jmp pci_real_fail
pci_real_f08: ;; read configuration byte
  cmp al, #0x08
  jne pci_real_f09
  call pci_real_select_reg
  push dx
  mov dx, di
  and dx, #0x03
  add dx, #0x0cfc
  in al, dx
  pop dx
  mov cl, al
  jmp pci_real_ok
pci_real_f09: ;; read configuration word
  cmp al, #0x09
  jne pci_real_f0a
  call pci_real_select_reg
  push dx
  mov dx, di
  and dx, #0x02
  add dx, #0x0cfc
  in ax, dx
  pop dx
  mov cx, ax
  jmp pci_real_ok
pci_real_f0a: ;; read configuration dword
  cmp al, #0x0a
  jne pci_real_f0b
  call pci_real_select_reg
  push dx
  mov dx, #0x0cfc
  in eax, dx
  pop dx
  mov ecx, eax
  jmp pci_real_ok
pci_real_f0b: ;; write configuration byte
  cmp al, #0x0b
  jne pci_real_f0c
  call pci_real_select_reg
  push dx
  mov dx, di
  and dx, #0x03
  add dx, #0x0cfc
  mov al, cl
  out dx, al
  pop dx
  jmp pci_real_ok
pci_real_f0c: ;; write configuration word
  cmp al, #0x0c
  jne pci_real_f0d
  call pci_real_select_reg
  push dx
  mov dx, di
  and dx, #0x02
  add dx, #0x0cfc
  mov ax, cx
  out dx, ax
  pop dx
  jmp pci_real_ok
pci_real_f0d: ;; write configuration dword
  cmp al, #0x0d
  jne pci_real_unknown
  call pci_real_select_reg
  push dx
  mov dx, #0x0cfc
  mov eax, ecx
  out dx, eax
  pop dx
  jmp pci_real_ok
pci_real_unknown:
  mov ah, #0x81
pci_real_fail:
  pop edi
  pop esi
  stc
  ret
pci_real_ok:
  xor ah, ah
  pop edi
  pop esi
  clc
  ret
pci_real_select_reg:
  push dx
  mov eax, #0x800000
  mov ax, bx
  shl eax, #8
  and di, #0xff
  or ax, di
  and al, #0xfc
  mov dx, #0x0cf8
  out dx, eax
  pop dx
  ret
.align 16
pci_routing_table_structure:
  db 0x24, 0x50, 0x49, 0x52 ;; "$PIR" signature
  db 0, 1 ;; version
  dw 32 + (6 * 16) ;; table size
  db 0 ;; PCI interrupt router bus
  db 0x08 ;; PCI interrupt router DevFunc
  dw 0x0000 ;; PCI exclusive IRQs
  dw 0x8086 ;; compatible PCI interrupt router vendor ID
  dw 0x7000 ;; compatible PCI interrupt router device ID
  dw 0,0 ;; Miniport data
  db 0,0,0,0,0,0,0,0,0,0,0 ;; reserved
  db 0x07 ;; checksum
  ;; first slot entry PCI-to-ISA (embedded)
  db 0 ;; pci bus number
  db 0x08 ;; pci device number (bit 7-3)
  db 0x61 ;; link value INTA#: pointer into PCI2ISA config space
  dw 0x0c20 ;; IRQ bitmap INTA#
  db 0x62 ;; link value INTB#
  dw 0x0c20 ;; IRQ bitmap INTB#
  db 0x63 ;; link value INTC#
  dw 0x0c20 ;; IRQ bitmap INTC#
  db 0x60 ;; link value INTD#
  dw 0x0c20 ;; IRQ bitmap INTD#
  db 0 ;; physical slot (0 = embedded)
  db 0 ;; reserved
  ;; second slot entry: 1st PCI slot
  db 0 ;; pci bus number
  db 0x10 ;; pci device number (bit 7-3)
  db 0x62 ;; link value INTA#
  dw 0x0c20 ;; IRQ bitmap INTA#
  db 0x63 ;; link value INTB#
  dw 0x0c20 ;; IRQ bitmap INTB#
  db 0x60 ;; link value INTC#
  dw 0x0c20 ;; IRQ bitmap INTC#
  db 0x61 ;; link value INTD#
  dw 0x0c20 ;; IRQ bitmap INTD#
  db 1 ;; physical slot (0 = embedded)
  db 0 ;; reserved
  ;; third slot entry: 2nd PCI slot
  db 0 ;; pci bus number
  db 0x18 ;; pci device number (bit 7-3)
  db 0x63 ;; link value INTA#
  dw 0x0c20 ;; IRQ bitmap INTA#
  db 0x60 ;; link value INTB#
  dw 0x0c20 ;; IRQ bitmap INTB#
  db 0x61 ;; link value INTC#
  dw 0x0c20 ;; IRQ bitmap INTC#
  db 0x62 ;; link value INTD#
  dw 0x0c20 ;; IRQ bitmap INTD#
  db 2 ;; physical slot (0 = embedded)
  db 0 ;; reserved
  ;; 4th slot entry: 3rd PCI slot
  db 0 ;; pci bus number
  db 0x20 ;; pci device number (bit 7-3)
  db 0x60 ;; link value INTA#
  dw 0x0c20 ;; IRQ bitmap INTA#
  db 0x61 ;; link value INTB#
  dw 0x0c20 ;; IRQ bitmap INTB#
  db 0x62 ;; link value INTC#
  dw 0x0c20 ;; IRQ bitmap INTC#
  db 0x63 ;; link value INTD#
  dw 0x0c20 ;; IRQ bitmap INTD#
  db 3 ;; physical slot (0 = embedded)
  db 0 ;; reserved
  ;; 5th slot entry: 4rd PCI slot
  db 0 ;; pci bus number
  db 0x28 ;; pci device number (bit 7-3)
  db 0x61 ;; link value INTA#
  dw 0x0c20 ;; IRQ bitmap INTA#
  db 0x62 ;; link value INTB#
  dw 0x0c20 ;; IRQ bitmap INTB#
  db 0x63 ;; link value INTC#
  dw 0x0c20 ;; IRQ bitmap INTC#
  db 0x60 ;; link value INTD#
  dw 0x0c20 ;; IRQ bitmap INTD#
  db 4 ;; physical slot (0 = embedded)
  db 0 ;; reserved
  ;; 6th slot entry: 5rd PCI slot
  db 0 ;; pci bus number
  db 0x30 ;; pci device number (bit 7-3)
  db 0x62 ;; link value INTA#
  dw 0x0c20 ;; IRQ bitmap INTA#
  db 0x63 ;; link value INTB#
  dw 0x0c20 ;; IRQ bitmap INTB#
  db 0x60 ;; link value INTC#
  dw 0x0c20 ;; IRQ bitmap INTC#
  db 0x61 ;; link value INTD#
  dw 0x0c20 ;; IRQ bitmap INTD#
  db 5 ;; physical slot (0 = embedded)
  db 0 ;; reserved
; parallel port detection: base address in DX, index in BX, timeout in CL
detect_parport:
  push dx
  add dx, #2
  in al, dx
  and al, #0xdf ; clear input mode
  out dx, al
  pop dx
  mov al, #0xaa
  out dx, al
  in al, dx
  cmp al, #0xaa
  jne no_parport
  push bx
  shl bx, #1
  mov [bx+0x408], dx ; Parallel I/O address
  pop bx
  mov [bx+0x478], cl ; Parallel printer timeout
  inc bx
no_parport:
  ret
; serial port detection: base address in DX, index in BX, timeout in CL
detect_serial:
  push dx
  inc dx
  mov al, #0x02
  out dx, al
  in al, dx
  cmp al, #0x02
  jne no_serial
  inc dx
  in al, dx
  cmp al, #0x02
  jne no_serial
  dec dx
  xor al, al
  out dx, al
  pop dx
  push bx
  shl bx, #1
  mov [bx+0x400], dx ; Serial I/O address
  pop bx
  mov [bx+0x47c], cl ; Serial timeout
  inc bx
  ret
no_serial:
  pop dx
  ret
rom_checksum:
  push ax
  push bx
  push cx
  xor ax, ax
  xor bx, bx
  xor cx, cx
  mov ch, [2]
  shl cx, #1
checksum_loop:
  add al, [bx]
  inc bx
  loop checksum_loop
  and al, #0xff
  pop cx
  pop bx
  pop ax
  ret
;; We need a copy of this string, but we are not actually a PnP BIOS,
;; so make sure it is *not* aligned, so OSes will not see it if they scan.
.align 16
  db 0
pnp_string:
  .ascii "$PnP"
rom_scan:
  ;; Scan for existence of valid expansion ROMS.
  ;; Video ROM: from 0xC0000..0xC7FFF in 2k increments
  ;; General ROM: from 0xC8000..0xDFFFF in 2k increments
  ;; System ROM: only 0xE0000
  ;;
  ;; Header:
  ;; Offset Value
  ;; 0 0x55
  ;; 1 0xAA
  ;; 2 ROM length in 512-byte blocks
  ;; 3 ROM initialization entry point (FAR CALL)
  call _tcpa_start_option_rom_scan
  mov cx, #0xc000
rom_scan_loop:
  mov ds, cx
  mov ax, #0x0004 ;; start with increment of 4 (512-byte) blocks = 2k
  cmp [0], #0xAA55 ;; look for signature
  jne rom_scan_increment
  call rom_checksum
  jnz rom_scan_increment
  mov al, [2] ;; change increment to ROM length in 512-byte blocks
  ;; We want our increment in 512-byte quantities, rounded to
  ;; the nearest 2k quantity, since we only scan at 2k intervals.
  test al, #0x03
  jz block_count_rounded
  and al, #0xfc ;; needs rounding up
  add al, #0x04
block_count_rounded:
  push ax
  push ds
  push ecx
  xor ax, ax
  mov ds, ax
  and ecx, #0xffff
  push ecx ;; segment where option rom is located at
  call _tcpa_option_rom
  add sp, #4 ;; pop segment
  pop ecx ;; original ecx
  pop ds
  pop ax
  xor bx, bx ;; Restore DS back to 0000:
  mov ds, bx
  push ax ;; Save AX
  push di ;; Save DI
  ;; Push addr of ROM entry point
  push cx ;; Push seg
  push #0x0003 ;; Push offset
  ;; Point ES:DI at "$PnP", which tells the ROM that we are a PnP BIOS.
  ;; That should stop it grabbing INT 19h; we will use its BEV instead.
  mov ax, #0xf000
  mov es, ax
  lea di, pnp_string
  mov bp, sp ;; Call ROM init routine using seg:off on stack
  db 0xff ;; call_far ss:[bp+0]
  db 0x5e
  db 0
  cli ;; In case expansion ROM BIOS turns IF on
  add sp, #2 ;; Pop offset value
  pop cx ;; Pop seg value (restore CX)
  ;; Look at the ROM's PnP Expansion header.  Properly, we're supposed
  ;; to init all the ROMs and then go back and build an IPL table of
  ;; all the bootable devices, but we can get away with one pass.
  mov ds, cx ;; ROM base
  mov bx, 0x001a ;; 0x1A is the offset into ROM header that contains...
  mov ax, [bx] ;; the offset of PnP expansion header, where...
  cmp ax, #0x5024 ;; we look for signature "$PnP"
  jne no_bev
  mov ax, 2[bx]
  cmp ax, #0x506e
  jne no_bev
  mov ax, 0x1a[bx] ;; 0x1A is also the offset into the expansion header of...
  cmp ax, #0x0000 ;; the Bootstrap Entry Vector, or zero if there is 0 .
  je no_bev
  ;; Found a device that thinks it can boot the system. Record its BEV.
  mov bx, #0x9ff0 ;; Go to the segment where the IPL table lives
  mov ds, bx
  mov bx, 0x0080 ;; Read the number of entries so far
  cmp bx, #8
  je no_bev ;; Get out if the table is full
  shl bx, #0x4 ;; Turn count into offset (entries are 16 bytes)
  mov 0[bx], #0x80 ;; This entry is a BEV device
  mov 6[bx], cx ;; Build a far pointer from the segment...
  mov 4[bx], ax ;; and the offset
  shr bx, #0x4 ;; Turn the offset back into a count
  inc bx ;; We have one more entry now
  mov 0x0080, bx ;; Remember that.
no_bev:
  pop di ;; Restore DI
  pop ax ;; Restore AX
rom_scan_increment:
  shl ax, #5 ;; convert 512-bytes blocks to 16-byte increments
                ;; because the segment selector is shifted left 4 bits.
  add cx, ax
  cmp cx, #0xe000
  jbe rom_scan_loop
  xor ax, ax ;; Restore DS back to 0000:
  mov ds, ax
  ret
; Copy the SMBIOS entry point from where hvmloader left it.
; The entry point must be somewhere in 0xf0000-0xfffff on a 16-byte boundary,
; but the tables themselves can be elsewhere.
smbios_init:
  push ax
  push cx
  push es
  push ds
  push di
  push si
  mov cx, #0x001f ; 0x1f bytes to copy
  mov ax, #0xf000
  mov es, ax ; destination segment is 0xf0000
  mov di, #smbios_entry_point ; destination offset
  mov ax, #(0x000E9000>>4)
  mov ds, ax
  mov si, #(0x000E9000&15)
  cld
  rep
    movsb
  pop si
  pop di
  pop ds
  pop es
  pop cx
  pop ax
  ret
; The section between the POST entry and the NMI entry is filling up
; and causes crashes if this code was directly there
tcpa_post_part1:
  call _tcpa_acpi_init
  push dword #0
  call _tcpa_initialize_tpm
  add sp, #4
  call _tcpa_do_measure_POSTs
  call _tcpa_wake_event
  ret
tcpa_post_part2:
  call _tcpa_calling_int19h
  call _tcpa_add_event_separators
  call _tcpa_returned_int19h
  ret
;; for 'C' strings and other data, insert them here with
;; a the following hack:
;; DATA_SEG_DEFS_HERE
;--------
;- POST -
;--------
.org 0xe05b ; POST Entry Point
post:
  xor ax, ax
  ;; first reset the DMA controllers
  out 0x0d,al
  out 0xda,al
  ;; then initialize the DMA controllers
  mov al, #0xC0
  out 0xD6, al ; cascade mode of channel 4 enabled
  mov al, #0x00
  out 0xD4, al ; unmask channel 4
  ;; Examine CMOS shutdown status.
  mov AL, #0x0f
  out 0x70, AL
  in AL, 0x71
  ;; backup status
  mov bl, al
  ;; Reset CMOS shutdown status.
  mov AL, #0x0f
  out 0x70, AL ; select CMOS register Fh
  mov AL, #0x00
  out 0x71, AL ; set shutdown action to normal
  ;; Examine CMOS shutdown status.
  mov al, bl
  mov dx, #0x9FC0
  mov ds, dx
  mov [1], AL
  cli
  mov ax, #0xfffe
  mov sp, ax
  mov ax, #0x0000
  mov ds, ax
  mov ss, ax
  ;; zero out BIOS data area (40:00..40:ff)
  mov es, ax
  mov cx, #0x0080 ;; 128 words
  mov di, #0x0400
  cld
  rep
    stosw
  call _log_bios_start
  ;; set all interrupts to default handler
  mov bx, #0x0000 ;; offset index
  mov cx, #0x0100 ;; counter (256 interrupts)
  mov ax, #dummy_iret_handler
  mov dx, #0xF000
post_default_ints:
  mov [bx], ax
  inc bx
  inc bx
  mov [bx], dx
  inc bx
  inc bx
  loop post_default_ints
  ;; set vector 0x79 to zero
  ;; this is used by 'gardian angel' protection system
  SET_INT_VECTOR(0x79, #0, #0)
  ;; base memory in K 40:13 (word)
  mov ax, #(640 - 1)
  mov 0x0413, ax
  ;; Manufacturing Test 40:12
  ;; zerod out above
  ;; Warm Boot Flag 0040:0072
  ;; value of 1234h = skip memory checks
  ;; zerod out above
  ;; Printer Services vector
  SET_INT_VECTOR(0x17, #0xF000, #int17_handler)
  ;; Bootstrap failure vector
  SET_INT_VECTOR(0x18, #0xF000, #int18_handler)
  ;; Bootstrap Loader vector
  SET_INT_VECTOR(0x19, #0xF000, #int19_handler)
  ;; User Timer Tick vector
  SET_INT_VECTOR(0x1c, #0xF000, #int1c_handler)
  ;; Memory Size Check vector
  SET_INT_VECTOR(0x12, #0xF000, #int12_handler)
  ;; Equipment Configuration Check vector
  SET_INT_VECTOR(0x11, #0xF000, #int11_handler)
  ;; System Services
  SET_INT_VECTOR(0x15, #0xF000, #int15_handler)
  ;; EBDA setup
  call ebda_post
  ;; PIT setup
  SET_INT_VECTOR(0x08, #0xF000, #int08_handler)
  ;; int 1C already points at dummy_iret_handler (above)
  mov al, #0x34 ; timer0: binary count, 16bit count, mode 2
  out 0x43, al
  mov al, #0x0b ; #0xe90b = 20 Hz (temporary, until we fix xen/vmx support)
  out 0x40, al ; lsb
  mov al, #0xe9
  out 0x40, al ; msb
  ;; Keyboard
  SET_INT_VECTOR(0x09, #0xF000, #int09_handler)
  SET_INT_VECTOR(0x16, #0xF000, #int16_handler)
  xor ax, ax
  mov ds, ax
  mov 0x0417, al
  mov 0x0418, al
  mov 0x0419, al
  mov 0x0471, al
  mov 0x0497, al
  mov al, #0x10
  mov 0x0496, al
  mov bx, #0x001E
  mov 0x041A, bx
  mov 0x041C, bx
  mov bx, #0x001E
  mov 0x0480, bx
  mov bx, #0x003E
  mov 0x0482, bx
  call _keyboard_init
  ;; mov CMOS Equipment Byte to BDA Equipment Word
  mov ax, 0x0410
  mov al, #0x14
  out 0x70, al
  in al, 0x71
  mov 0x0410, ax
  call tcpa_post_part1
  ;; Parallel setup
  SET_INT_VECTOR(0x0F, #0xF000, #dummy_iret_handler)
  xor ax, ax
  mov ds, ax
  xor bx, bx
  mov cl, #0x14 ; timeout value
  mov dx, #0x378 ; Parallel I/O address, port 1
  call detect_parport
  mov dx, #0x278 ; Parallel I/O address, port 2
  call detect_parport
  shl bx, #0x0e
  mov ax, 0x410 ; Equipment word bits 14..15 determing # parallel ports
  and ax, #0x3fff
  or ax, bx ; set number of parallel ports
  mov 0x410, ax
  ;; Serial setup
  SET_INT_VECTOR(0x0C, #0xF000, #dummy_iret_handler)
  SET_INT_VECTOR(0x14, #0xF000, #int14_handler)
  xor bx, bx
  mov cl, #0x0a ; timeout value
  mov dx, #0x03f8 ; Serial I/O address, port 1
  call detect_serial
  mov dx, #0x02f8 ; Serial I/O address, port 2
  call detect_serial
  mov dx, #0x03e8 ; Serial I/O address, port 3
  call detect_serial
  mov dx, #0x02e8 ; Serial I/O address, port 4
  call detect_serial
  shl bx, #0x09
  mov ax, 0x410 ; Equipment word bits 9..11 determing # serial ports
  and ax, #0xf1ff
  or ax, bx ; set number of serial port
  mov 0x410, ax
  ;; CMOS RTC
  SET_INT_VECTOR(0x1A, #0xF000, #int1a_handler)
  SET_INT_VECTOR(0x4A, #0xF000, #dummy_iret_handler)
  SET_INT_VECTOR(0x70, #0xF000, #int70_handler)
  ;; BIOS DATA AREA 0x4CE ???
  call timer_tick_post
  ;; PS/2 mouse setup
  SET_INT_VECTOR(0x74, #0xF000, #int74_handler)
  ;; IRQ13 (FPU exception) setup
  SET_INT_VECTOR(0x75, #0xF000, #int75_handler)
  ;; Video setup
  SET_INT_VECTOR(0x10, #0xF000, #int10_handler)
  ;; PIC
  mov al, #0x11 ; send initialisation commands
  out 0x20, al
  out 0xa0, al
  mov al, #0x08
  out 0x21, al
  mov al, #0x70
  out 0xa1, al
  mov al, #0x04
  out 0x21, al
  mov al, #0x02
  out 0xa1, al
  mov al, #0x01
  out 0x21, al
  out 0xa1, al
  mov al, #0xb8
  out 0x21, AL ;master pic: unmask IRQ 0, 1, 2, 6
  mov al, #0x8f
  out 0xa1, AL ;slave pic: unmask IRQ 12, 13, 14
  call _enable_rom_write_access
  call _clobber_entry_point
  call _copy_e820_table
  call smbios_init
  call _disable_rom_write_access
  call _init_boot_vectors
  call rom_scan
  call _print_bios_banner
  ;;
  ;; Floppy setup
  ;;
  call floppy_drive_post
  ;;
  ;; Hard Drive setup
  ;;
  call hard_drive_post
  ;;
  ;; ATA/ATAPI driver setup
  ;;
  call _ata_init
  call _ata_detect
  ;;
  ;;
  ;; eltorito floppy/harddisk emulation from cd
  ;;
  call _cdemu_init
  ;;
  call _s3_resume
  call _interactive_bootkey
  call tcpa_post_part2
  ;; Start the boot sequence. See the comments in int19_relocated
  ;; for why we use INT 18h instead of INT 19h here.
  int #0x18
.org 0xe2c3 ; NMI Handler Entry Point
nmi:
  ;; FIXME the NMI handler should not panic
  ;; but iret when called from int75 (fpu exception)
  call _nmi_handler_msg
  iret
int75_handler:
  out 0xf0, al
  call eoi_both_pics
  int 2
  iret
;-------------------------------------------
;- INT 13h Fixed Disk Services Entry Point -
;-------------------------------------------
.org 0xe3fe ; INT 13h Fixed Disk Services Entry Point
int13_handler:
  jmp int13_relocated
.org 0xe401 ; Fixed Disk Parameter Table
;----------
;- INT19h -
;----------
.org 0xe6f2 ; INT 19h Boot Load Service Entry Point
int19_handler:
  jmp int19_relocated
;-------------------------------------------
;- System BIOS Configuration Data Table
;-------------------------------------------
.org 0xe6f5
db 0x08 ; Table size (bytes) -Lo
db 0x00 ; Table size (bytes) -Hi
db 0xFC
db 0x00
db 1
; Feature byte 1
; b7: 1=DMA channel 3 used by hard disk
; b6: 1=2 interrupt controllers present
; b5: 1=RTC present
; b4: 1=BIOS calls int 15h/4Fh every key
; b3: 1=wait for extern event supported (Int 15h/41h)
; b2: 1=extended BIOS data area used
; b1: 0=AT or ESDI bus, 1=MicroChannel
; b0: 1=Dual bus (MicroChannel + ISA)
db (0 << 7) | (1 << 6) | (1 << 5) | (1 << 4) | (0 << 3) | (1 << 2) | (0 << 1) | (0 << 0)
; Feature byte 2
; b7: 1=32-bit DMA supported
; b6: 1=int16h, function 9 supported
; b5: 1=int15h/C6h (get POS data) supported
; b4: 1=int15h/C7h (get mem map info) supported
; b3: 1=int15h/C8h (en/dis CPU) supported
; b2: 1=non-8042 kb controller
; b1: 1=data streaming supported
; b0: reserved
db (0 << 7) | (1 << 6) | (0 << 5) | (0 << 4) | (0 << 3) | (0 << 2) | (0 << 1) | (0 << 0)
; Feature byte 3
; b7: not used
; b6: reserved
; b5: reserved
; b4: POST supports ROM-to-RAM enable/disable
; b3: SCSI on system board
; b2: info panel installed
; b1: Initial Machine Load (IML) system - BIOS on disk
; b0: SCSI supported in IML
db 0x00
; Feature byte 4
; b7: IBM private
; b6: EEPROM present
; b5-3: ABIOS presence (011 = not supported)
; b2: private
; b1: memory split above 16Mb supported
; b0: POSTEXT directly supported by POST
db 0x00
; Feature byte 5 (IBM)
; b1: enhanced mouse
; b0: flash EPROM
db 0x00
.org 0xe729 ; Baud Rate Generator Table
;----------
;- INT14h -
;----------
.org 0xe739 ; INT 14h Serial Communications Service Entry Point
int14_handler:
  push ds
  pusha
  mov ax, #0x0000
  mov ds, ax
  call _int14_function
  popa
  pop ds
  iret
;----------------------------------------
;- INT 16h Keyboard Service Entry Point -
;----------------------------------------
.org 0xe82e
int16_handler:
  sti
  push ds
  pushf
  pusha
  cmp ah, #0x00
  je int16_F00
  cmp ah, #0x10
  je int16_F00
  mov bx, #0xf000
  mov ds, bx
  call _int16_function
  popa
  popf
  pop ds
  jz int16_zero_set
int16_zero_clear:
  push bp
  mov bp, sp
  and BYTE [bp + 0x06], #0xbf
  pop bp
  iret
int16_zero_set:
  push bp
  mov bp, sp
  or BYTE [bp + 0x06], #0x40
  pop bp
  iret
int16_F00:
  mov bx, #0x0040
  mov ds, bx
int16_wait_for_key:
  cli
  mov bx, 0x001a
  cmp bx, 0x001c
  jne int16_key_found
  sti
  nop
  jmp int16_wait_for_key
int16_key_found:
  mov bx, #0xf000
  mov ds, bx
  call _int16_function
  popa
  popf
  pop ds
  iret
;-------------------------------------------------
;- INT09h : Keyboard Hardware Service Entry Point -
;-------------------------------------------------
.org 0xe987
int09_handler:
  cli
  push ax
  mov al, #0xAD ;;disable keyboard
  out #0x64, al
  mov al, #0x0B
  out #0x20, al
  in al, #0x20
  and al, #0x02
  jz int09_finish
  in al, #0x60 ;;read key from keyboard controller
  ;; check for extended key
  cmp al, #0xe0
  jne int09_call_int15_4f
  push ds
  xor ax, ax
  mov ds, ax
  mov al, BYTE [0x496] ;; mf2_state |= 0x01
  or al, #0x01
  mov BYTE [0x496], al
  pop ds
  in al, #0x60 ;;read another key from keyboard controller
  sti
int09_call_int15_4f:
  push ds
  pusha
  mov ah, #0x4f ;; allow for keyboard intercept
  stc
  int #0x15
  jnc int09_done
  mov bx, #0xf000
  mov ds, bx
  call _int09_function
int09_done:
  popa
  pop ds
  cli
  call eoi_master_pic
int09_finish:
  mov al, #0xAE ;;enable keyboard
  out #0x64, al
  pop ax
  iret
;----------------------------------------
;- INT 13h Diskette Service Entry Point -
;----------------------------------------
.org 0xec59
int13_diskette:
  jmp int13_noeltorito
;---------------------------------------------
;- INT 0Eh Diskette Hardware ISR Entry Point -
;---------------------------------------------
.org 0xef57 ; INT 0Eh Diskette Hardware ISR Entry Point
int0e_handler:
  push ax
  push dx
  mov dx, #0x03f4
  in al, dx
  and al, #0xc0
  cmp al, #0xc0
  je int0e_normal
  mov dx, #0x03f5
  mov al, #0x08 ; sense interrupt status
  out dx, al
int0e_loop1:
  mov dx, #0x03f4
  in al, dx
  and al, #0xc0
  cmp al, #0xc0
  jne int0e_loop1
int0e_loop2:
  mov dx, #0x03f5
  in al, dx
  mov dx, #0x03f4
  in al, dx
  and al, #0xc0
  cmp al, #0xc0
  je int0e_loop2
int0e_normal:
  push ds
  mov ax, #0x0000 ;; segment 0000
  mov ds, ax
  call eoi_master_pic
  mov al, 0x043e
  or al, #0x80 ;; diskette interrupt has occurred
  mov 0x043e, al
  pop ds
  pop dx
  pop ax
  iret
.org 0xefc7 ; Diskette Controller Parameter Table
diskette_param_table:
;; Since no provisions are made for multiple drive types, most
;; values in this table are ignored. I set parameters for 1.44M
;; floppy here
db 0xAF
db 0x02 ;; head load time 0000001, DMA used
db 0x25
db 0x02
db 18
db 0x1B
db 0xFF
db 0x6C
db 0xF6
db 0x0F
db 0x08
;----------------------------------------
;- INT17h : Printer Service Entry Point -
;----------------------------------------
.org 0xefd2
int17_handler:
  push ds
  pusha
  mov ax, #0x0000
  mov ds, ax
  call _int17_function
  popa
  pop ds
  iret
diskette_param_table2:
;; New diskette parameter table adding 3 parameters from IBM
;; Since no provisions are made for multiple drive types, most
;; values in this table are ignored. I set parameters for 1.44M
;; floppy here
db 0xAF
db 0x02 ;; head load time 0000001, DMA used
db 0x25
db 0x02
db 18
db 0x1B
db 0xFF
db 0x6C
db 0xF6
db 0x0F
db 0x08
db 79 ;; maximum track
db 0 ;; data transfer rate
db 4 ;; drive type in cmos
.org 0xf045 ; INT 10 Functions 0-Fh Entry Point
  HALT(10480)
  iret
;----------
;- INT10h -
;----------
.org 0xf065 ; INT 10h Video Support Service Entry Point
int10_handler:
  ;; dont do anything, since the VGA BIOS handles int10h requests
  iret
.org 0xf0a4 ; MDA/CGA Video Parameter Table (INT 1Dh)
;----------
;- INT12h -
;----------
.org 0xf841 ; INT 12h Memory Size Service Entry Point
; ??? different for Pentium (machine check)?
int12_handler:
  push ds
  mov ax, #0x0040
  mov ds, ax
  mov ax, 0x0013
  pop ds
  iret
;----------
;- INT11h -
;----------
.org 0xf84d ; INT 11h Equipment List Service Entry Point
int11_handler:
  push ds
  mov ax, #0x0040
  mov ds, ax
  mov ax, 0x0010
  pop ds
  iret
;----------
;- INT15h -
;----------
.org 0xf859 ; INT 15h System Services Entry Point
int15_handler:
  pushf
  cmp ah, #0x53
  je apm_call
  push ds
  push es
  cmp ah, #0x86
  je int15_handler32
  cmp ah, #0xE8
  je int15_handler32
  pusha
  cmp ah, #0xC2
  je int15_handler_mouse
  call _int15_function
int15_handler_mouse_ret:
  popa
int15_handler32_ret:
  pop es
  pop ds
  popf
  jmp iret_modify_cf
apm_call:
  jmp _apmreal_entry
int15_handler_mouse:
  call _int15_function_mouse
  jmp int15_handler_mouse_ret
int15_handler32:
  pushad
  call _int15_function32
  popad
  jmp int15_handler32_ret
;; Protected mode IDT descriptor
;;
;; I just make the limit 0, so the machine will shutdown
;; if an exception occurs during protected mode memory
;; transfers.
;;
;; Set base to f0000 to correspond to beginning of BIOS,
;; in case I actually define an IDT later
;; Set limit to 0
pmode_IDT_info:
dw 0x0000 ;; limit 15:00
dw 0x0000 ;; base 15:00
db 0x0f ;; base 23:16
;; Real mode IDT descriptor
;;
;; Set to typical real-mode values.
;; base = 000000
;; limit = 03ff
rmode_IDT_info:
dw 0x03ff ;; limit 15:00
dw 0x0000 ;; base 15:00
db 0x00 ;; base 23:16
;----------
;- INT1Ah -
;----------
.org 0xfe6e ; INT 1Ah Time-of-day Service Entry Point
int1a_handler:
  cmp ah, #0xbb
  jne no_tcg
  pushf
  push ds
  push es
  pushad
  call _int1a_function32
  popad
  pop es
  pop ds
  popf
  iret
no_tcg:
  cmp ah, #0xb1
  jne int1a_normal
  call pcibios_real
  jc pcibios_error
  retf 2
pcibios_error:
  mov bl, ah
  mov ah, #0xb1
  push ds
  pusha
  mov ax, ss ; set readable descriptor to ds, for calling pcibios
  mov ds, ax ; on 16bit protected mode.
  jmp int1a_callfunction
int1a_normal:
  push ds
  pusha
  xor ax, ax
  mov ds, ax
int1a_callfunction:
  call _int1a_function
  popa
  pop ds
  iret
;;
;; int70h: IRQ8 - CMOS RTC
;;
int70_handler:
  push ds
  pusha
  xor ax, ax
  mov ds, ax
  call _int70_function
  popa
  pop ds
  iret
;---------
;- INT08 -
;---------
.org 0xfea5 ; INT 08h System Timer ISR Entry Point
int08_handler:
  sti
  push eax
  push ds
  xor ax, ax
  mov ds, ax
  ;; time to turn off drive(s)?
  mov al,0x0440
  or al,al
  jz int08_floppy_off
  dec al
  mov 0x0440,al
  jnz int08_floppy_off
  ;; turn motor(s) off
  push dx
  mov dx,#0x03f2
  in al,dx
  and al,#0xcf
  out dx,al
  pop dx
int08_floppy_off:
  mov eax, 0x046c ;; get ticks dword
  inc eax
  ;; compare eax to one days worth of timer ticks at 18.2 hz
  cmp eax, #0x001800B0
  jb int08_store_ticks
  ;; there has been a midnight rollover at this point
  xor eax, eax ;; zero out counter
  inc BYTE 0x0470 ;; increment rollover flag
int08_store_ticks:
  mov 0x046c, eax ;; store new ticks dword
  ;; chain to user timer tick INT #0x1c
  int #0x1c
  cli
  call eoi_master_pic
  pop ds
  pop eax
  iret
.org 0xfef3 ; Initial Interrupt Vector Offsets Loaded by POST
.org 0xff00
.ascii "(c) 2002 MandrakeSoft S.A. Written by Kevin Lawton & the Bochs team."
;------------------------------------------------
;- IRET Instruction for Dummy Interrupt Handler -
;------------------------------------------------
.org 0xff53 ; IRET Instruction for Dummy Interrupt Handler
dummy_iret_handler:
  iret
.org 0xff54 ; INT 05h Print Screen Service Entry Point
  HALT(10714)
  iret
.org 0xfff0 ; Power-up Entry Point
  jmp 0xf000:post
.org 0xfff5 ; ASCII Date ROM was built - 8 characters in MM/DD/YY
.ascii "06/23/99"
.org 0xfffe ; System Model ID
db 0xFC
db 0x00 ; filler
.org 0xfa6e ;; Character Font for 320x200 & 640x200 Graphics (lower 128 characters)
! 8039 endasm
!BCC_ENDASM
! 8040 static Bit8u vgafont8[128*8]=
! 8041 {
.data
_vgafont8:
! 8042  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
.byte	0
.byte	0
.byte	0
.byte	0
.byte	0
.byte	0
.byte	0
.byte	0
! 8043  0x7e, 0x81, 0xa5, 0x81, 0xbd, 0x99, 0x81, 0x7e,
.byte	$7E
.byte	$81
.byte	$A5
.byte	$81
.byte	$BD
.byte	$99
.byte	$81
.byte	$7E
! 8044  0x7e, 0xff, 0xdb, 0xff, 0xc3, 0xe7, 0xff, 0x7e,
.byte	$7E
.byte	$FF
.byte	$DB
.byte	$FF
.byte	$C3
.byte	$E7
.byte	$FF
.byte	$7E
! 8045  0x6c, 0xfe, 0xfe, 0xfe, 0x7c, 0x38, 0x10, 0x00,
.byte	$6C
.byte	$FE
.byte	$FE
.byte	$FE
.byte	$7C
.byte	$38
.byte	$10
.byte	0
! 8046  0x10, 0x38, 0x7c, 0xfe, 0x7c, 0x38, 0x10, 0x00,
.byte	$10
.byte	$38
.byte	$7C
.byte	$FE
.byte	$7C
.byte	$38
.byte	$10
.byte	0
! 8047  0x38, 0x7c, 0x38, 0xfe, 0xfe, 0x7c, 0x38, 0x7c,
.byte	$38
.byte	$7C
.byte	$38
.byte	$FE
.byte	$FE
.byte	$7C
.byte	$38
.byte	$7C
! 8048  0x10, 0x10, 0x38, 0x7c, 0xfe, 0x7c, 0x38, 0x7c,
.byte	$10
.byte	$10
.byte	$38
.byte	$7C
.byte	$FE
.byte	$7C
.byte	$38
.byte	$7C
! 8049  0x00, 0x00, 0x18, 0x3c, 0x3c, 0x18, 0x00, 0x00,
.byte	0
.byte	0
.byte	$18
.byte	$3C
.byte	$3C
.byte	$18
.byte	0
.byte	0
! 8050  0xff, 0xff, 0xe7, 0xc3, 0xc3, 0xe7, 0xff, 0xff,
.byte	$FF
.byte	$FF
.byte	$E7
.byte	$C3
.byte	$C3
.byte	$E7
.byte	$FF
.byte	$FF
! 8051  0x00, 0x3c, 0x66, 0x42, 0x42, 0x66, 0x3c, 0x00,
.byte	0
.byte	$3C
.byte	$66
.byte	$42
.byte	$42
.byte	$66
.byte	$3C
.byte	0
! 8052  0xff, 0xc3, 0x99, 0xbd, 0xbd, 0x99, 0xc3, 0xff,
.byte	$FF
.byte	$C3
.byte	$99
.byte	$BD
.byte	$BD
.byte	$99
.byte	$C3
.byte	$FF
! 8053  0x0f, 0x07, 0x0f, 0x7d, 0xcc, 0xcc, 0xcc, 0x78,
.byte	$F
.byte	7
.byte	$F
.byte	$7D
.byte	$CC
.byte	$CC
.byte	$CC
.byte	$78
! 8054  0x3c, 0x66, 0x66, 0x66, 0x3c, 0x18, 0x7e, 0x18,
.byte	$3C
.byte	$66
.byte	$66
.byte	$66
.byte	$3C
.byte	$18
.byte	$7E
.byte	$18
! 8055  0x3f, 0x33, 0x3f, 0x30, 0x30, 0x70, 0xf0, 0xe0,
.byte	$3F
.byte	$33
.byte	$3F
.byte	$30
.byte	$30
.byte	$70
.byte	$F0
.byte	$E0
! 8056  0x7f, 0x63, 0x7f, 0x63, 0x63, 0x67, 0xe6, 0xc0,
.byte	$7F
.byte	$63
.byte	$7F
.byte	$63
.byte	$63
.byte	$67
.byte	$E6
.byte	$C0
! 8057  0x99, 0x5a, 0x3c, 0xe7, 0xe7, 0x3c, 0x5a, 0x99,
.byte	$99
.byte	$5A
.byte	$3C
.byte	$E7
.byte	$E7
.byte	$3C
.byte	$5A
.byte	$99
! 8058  0x80, 0xe0, 0xf8, 0xfe, 0xf8, 0xe0, 0x80, 0x00,
.byte	$80
.byte	$E0
.byte	$F8
.byte	$FE
.byte	$F8
.byte	$E0
.byte	$80
.byte	0
! 8059  0x02, 0x0e, 0x3e, 0xfe, 0x3e, 0x0e, 0x02, 0x00,
.byte	2
.byte	$E
.byte	$3E
.byte	$FE
.byte	$3E
.byte	$E
.byte	2
.byte	0
! 8060  0x18, 0x3c, 0x7e, 0x18, 0x18, 0x7e, 0x3c, 0x18,
.byte	$18
.byte	$3C
.byte	$7E
.byte	$18
.byte	$18
.byte	$7E
.byte	$3C
.byte	$18
! 8061  0x66, 0x66, 0x66, 0x66, 0x66, 0x00, 0x66, 0x00,
.byte	$66
.byte	$66
.byte	$66
.byte	$66
.byte	$66
.byte	0
.byte	$66
.byte	0
! 8062  0x7f, 0xdb, 0xdb, 0x7b, 0x1b, 0x1b, 0x1b, 0x00,
.byte	$7F
.byte	$DB
.byte	$DB
.byte	$7B
.byte	$1B
.byte	$1B
.byte	$1B
.byte	0
! 8063  0x3e, 0x63, 0x38
.byte	$3E
.byte	$63
! 8063 , 0x6c, 0x6c, 0x38, 0xcc, 0x78,
.byte	$38
.byte	$6C
.byte	$6C
.byte	$38
.byte	$CC
.byte	$78
! 8064  0x00, 0x00, 0x00, 0x00, 0x7e, 0x7e, 0x7e, 0x00,
.byte	0
.byte	0
.byte	0
.byte	0
.byte	$7E
.byte	$7E
.byte	$7E
.byte	0
! 8065  0x18, 0x3c, 0x7e, 0x18, 0x7e, 0x3c, 0x18, 0xff,
.byte	$18
.byte	$3C
.byte	$7E
.byte	$18
.byte	$7E
.byte	$3C
.byte	$18
.byte	$FF
! 8066  0x18, 0x3c, 0x7e, 0x18, 0x18, 0x18, 0x18, 0x00,
.byte	$18
.byte	$3C
.byte	$7E
.byte	$18
.byte	$18
.byte	$18
.byte	$18
.byte	0
! 8067  0x18, 0x18, 0x18, 0x18, 0x7e, 0x3c, 0x18, 0x00,
.byte	$18
.byte	$18
.byte	$18
.byte	$18
.byte	$7E
.byte	$3C
.byte	$18
.byte	0
! 8068  0x00, 0x18, 0x0c, 0xfe, 0x0c, 0x18, 0x00, 0x00,
.byte	0
.byte	$18
.byte	$C
.byte	$FE
.byte	$C
.byte	$18
.byte	0
.byte	0
! 8069  0x00, 0x30, 0x60, 0xfe, 0x60, 0x30, 0x00, 0x00,
.byte	0
.byte	$30
.byte	$60
.byte	$FE
.byte	$60
.byte	$30
.byte	0
.byte	0
! 8070  0x00, 0x00, 0xc0, 0xc0, 0xc0, 0xfe, 0x00, 0x00,
.byte	0
.byte	0
.byte	$C0
.byte	$C0
.byte	$C0
.byte	$FE
.byte	0
.byte	0
! 8071  0x00, 0x24, 0x66, 0xff, 0x66, 0x24, 0x00, 0x00,
.byte	0
.byte	$24
.byte	$66
.byte	$FF
.byte	$66
.byte	$24
.byte	0
.byte	0
! 8072  0x00, 0x18, 0x3c, 0x7e, 0xff, 0xff, 0x00, 0x00,
.byte	0
.byte	$18
.byte	$3C
.byte	$7E
.byte	$FF
.byte	$FF
.byte	0
.byte	0
! 8073  0x00, 0xff, 0xff, 0x7e, 0x3c, 0x18, 0x00, 0x00,
.byte	0
.byte	$FF
.byte	$FF
.byte	$7E
.byte	$3C
.byte	$18
.byte	0
.byte	0
! 8074  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
.byte	0
.byte	0
.byte	0
.byte	0
.byte	0
.byte	0
.byte	0
.byte	0
! 8075  0x30, 0x78, 0x78, 0x30, 0x30, 0x00, 0x30, 0x00,
.byte	$30
.byte	$78
.byte	$78
.byte	$30
.byte	$30
.byte	0
.byte	$30
.byte	0
! 8076  0x6c, 0x6c, 0x6c, 0x00, 0x00, 0x00, 0x00, 0x00,
.byte	$6C
.byte	$6C
.byte	$6C
.byte	0
.byte	0
.byte	0
.byte	0
.byte	0
! 8077  0x6c, 0x6c, 0xfe, 0x6c, 0xfe, 0x6c, 0x6c, 0x00,
.byte	$6C
.byte	$6C
.byte	$FE
.byte	$6C
.byte	$FE
.byte	$6C
.byte	$6C
.byte	0
! 8078  0x30, 0x7c, 0xc0, 0x78, 0x0c, 0xf8, 0x30, 0x00,
.byte	$30
.byte	$7C
.byte	$C0
.byte	$78
.byte	$C
.byte	$F8
.byte	$30
.byte	0
! 8079  0x00, 0xc6, 0xcc, 0x18, 0x30, 0x66, 0xc6, 0x00,
.byte	0
.byte	$C6
.byte	$CC
.byte	$18
.byte	$30
.byte	$66
.byte	$C6
.byte	0
! 8080  0x38, 0x6c, 0x38, 0x76, 0xdc, 0xcc, 0x76, 0x00,
.byte	$38
.byte	$6C
.byte	$38
.byte	$76
.byte	$DC
.byte	$CC
.byte	$76
.byte	0
! 8081  0x60, 0x60, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00,
.byte	$60
.byte	$60
.byte	$C0
.byte	0
.byte	0
.byte	0
.byte	0
.byte	0
! 8082  0x18, 0x30, 0x60, 0x60, 0x60, 0x30, 0x18, 0x00,
.byte	$18
.byte	$30
.byte	$60
.byte	$60
.byte	$60
.byte	$30
.byte	$18
.byte	0
! 8083  0x60, 0x30, 0x18, 0x18, 0x18, 0x30, 0x60, 0x00,
.byte	$60
.byte	$30
.byte	$18
.byte	$18
.byte	$18
.byte	$30
.byte	$60
.byte	0
! 8084  0x00, 0x66, 0x3c, 0xff, 0x3c, 0x66, 0x00, 0x00,
.byte	0
.byte	$66
.byte	$3C
.byte	$FF
.byte	$3C
.byte	$66
.byte	0
.byte	0
! 8085  0x00, 0x30, 0x30, 0xfc, 0x30, 0x30, 0x00, 0x00,
.byte	0
.byte	$30
.byte	$30
.byte	$FC
.byte	$30
.byte	$30
.byte	0
.byte	0
! 8086  0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x30, 0x60,
.byte	0
.byte	0
.byte	0
.byte	0
.byte	0
.byte	$30
.byte	$30
.byte	$60
! 8087  0x00, 0x00, 0x00, 0xfc, 0x00, 0x00, 0x00, 0x00,
.byte	0
.byte	0
.byte	0
.byte	$FC
.byte	0
.byte	0
.byte	0
.byte	0
! 8088  0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x30, 0x00,
.byte	0
.byte	0
.byte	0
.byte	0
.byte	0
.byte	$30
.byte	$30
.byte	0
! 8089  0x06, 0x0c, 0x18, 0x30, 0x60, 0xc0, 0x80, 0x00,
.byte	6
.byte	$C
.byte	$18
.byte	$30
.byte	$60
.byte	$C0
.byte	$80
.byte	0
! 8090  0x7c, 0xc6, 0xce, 0xde, 0xf6, 0xe6, 0x7c, 0x00,
.byte	$7C
.byte	$C6
.byte	$CE
.byte	$DE
.byte	$F6
.byte	$E6
.byte	$7C
.byte	0
! 8091  0x30, 0x70, 0x30, 0x30, 0x30, 0x30, 0xfc, 0x00,
.byte	$30
.byte	$70
.byte	$30
.byte	$30
.byte	$30
.byte	$30
.byte	$FC
.byte	0
! 8092  0x78, 0xcc, 0x0c, 0x38, 0x60, 0xcc, 0xfc, 0x00,
.byte	$78
.byte	$CC
.byte	$C
.byte	$38
.byte	$60
.byte	$CC
.byte	$FC
.byte	0
! 8093  0x78, 0xcc, 0x0c, 0x38, 0x0c, 0xcc, 0x78, 0x00,
.byte	$78
.byte	$CC
.byte	$C
.byte	$38
.byte	$C
.byte	$CC
.byte	$78
.byte	0
! 8094  0x1c, 0x3c, 0x6c, 0xcc, 0xfe, 0x0c, 0x1e, 0x00,
.byte	$1C
.byte	$3C
.byte	$6C
.byte	$CC
.byte	$FE
.byte	$C
.byte	$1E
.byte	0
! 8095  0xfc, 0xc0, 0xf8, 0x0c, 0x0c, 0xcc, 0x78, 0x00,
.byte	$FC
.byte	$C0
.byte	$F8
.byte	$C
.byte	$C
.byte	$CC
.byte	$78
.byte	0
! 8096  0x38, 0x60, 0xc0, 0xf8, 0xcc, 0xcc, 0x78, 0x00,
.byte	$38
.byte	$60
.byte	$C0
.byte	$F8
.byte	$CC
.byte	$CC
.byte	$78
.byte	0
! 8097  0xfc, 0xcc, 0x0c, 0x18, 0x30, 0x30, 0x30, 0x00,
.byte	$FC
.byte	$CC
.byte	$C
.byte	$18
.byte	$30
.byte	$30
.byte	$30
.byte	0
! 8098  0x78, 0xcc, 0xcc, 0x78, 0xcc, 0xcc, 0x78, 0x00,
.byte	$78
.byte	$CC
.byte	$CC
.byte	$78
.byte	$CC
.byte	$CC
.byte	$78
.byte	0
! 8099  0x78, 0xcc, 0xcc, 0x7c, 0x0c, 0x18, 0x70, 0x00,
.byte	$78
.byte	$CC
.byte	$CC
.byte	$7C
.byte	$C
.byte	$18
.byte	$70
.byte	0
! 8100  0x00, 0x30, 0x30, 0x00, 0x00, 0x30, 0x30, 0x00,
.byte	0
.byte	$30
.byte	$30
.byte	0
.byte	0
.byte	$30
.byte	$30
.byte	0
! 8101  0x00, 0x30, 0x30, 0x00, 0x00, 0x30, 0x30, 0x60,
.byte	0
.byte	$30
.byte	$30
.byte	0
.byte	0
.byte	$30
.byte	$30
.byte	$60
! 8102  0x18, 0x30, 0x60, 0xc0, 0x60, 0x30, 0x18, 0x00,
.byte	$18
.byte	$30
.byte	$60
.byte	$C0
.byte	$60
.byte	$30
.byte	$18
.byte	0
! 8103  0x00, 0x00, 0xfc, 0x00, 0x00, 0xfc, 0x00, 0x00,
.byte	0
.byte	0
.byte	$FC
.byte	0
.byte	0
.byte	$FC
.byte	0
.byte	0
! 8104  0x60, 0x30, 0x18, 0x0c, 0x18, 0x30, 0x60, 0x00,
.byte	$60
.byte	$30
.byte	$18
.byte	$C
.byte	$18
.byte	$30
.byte	$60
.byte	0
! 8105  0x78, 
.byte	$78
! 8105 0xcc, 0x0c, 0x18, 0x30, 0x00, 0x30, 0x00,
.byte	$CC
.byte	$C
.byte	$18
.byte	$30
.byte	0
.byte	$30
.byte	0
! 8106  0x7c, 0xc6, 0xde, 0xde, 0xde, 0xc0, 0x78, 0x00,
.byte	$7C
.byte	$C6
.byte	$DE
.byte	$DE
.byte	$DE
.byte	$C0
.byte	$78
.byte	0
! 8107  0x30, 0x78, 0xcc, 0xcc, 0xfc, 0xcc, 0xcc, 0x00,
.byte	$30
.byte	$78
.byte	$CC
.byte	$CC
.byte	$FC
.byte	$CC
.byte	$CC
.byte	0
! 8108  0xfc, 0x66, 0x66, 0x7c, 0x66, 0x66, 0xfc, 0x00,
.byte	$FC
.byte	$66
.byte	$66
.byte	$7C
.byte	$66
.byte	$66
.byte	$FC
.byte	0
! 8109  0x3c, 0x66, 0xc0, 0xc0, 0xc0, 0x66, 0x3c, 0x00,
.byte	$3C
.byte	$66
.byte	$C0
.byte	$C0
.byte	$C0
.byte	$66
.byte	$3C
.byte	0
! 8110  0xf8, 0x6c, 0x66, 0x66, 0x66, 0x6c, 0xf8, 0x00,
.byte	$F8
.byte	$6C
.byte	$66
.byte	$66
.byte	$66
.byte	$6C
.byte	$F8
.byte	0
! 8111  0xfe, 0x62, 0x68, 0x78, 0x68, 0x62, 0xfe, 0x00,
.byte	$FE
.byte	$62
.byte	$68
.byte	$78
.byte	$68
.byte	$62
.byte	$FE
.byte	0
! 8112  0xfe, 0x62, 0x68, 0x78, 0x68, 0x60, 0xf0, 0x00,
.byte	$FE
.byte	$62
.byte	$68
.byte	$78
.byte	$68
.byte	$60
.byte	$F0
.byte	0
! 8113  0x3c, 0x66, 0xc0, 0xc0, 0xce, 0x66, 0x3e, 0x00,
.byte	$3C
.byte	$66
.byte	$C0
.byte	$C0
.byte	$CE
.byte	$66
.byte	$3E
.byte	0
! 8114  0xcc, 0xcc, 0xcc, 0xfc, 0xcc, 0xcc, 0xcc, 0x00,
.byte	$CC
.byte	$CC
.byte	$CC
.byte	$FC
.byte	$CC
.byte	$CC
.byte	$CC
.byte	0
! 8115  0x78, 0x30, 0x30, 0x30, 0x30, 0x30, 0x78, 0x00,
.byte	$78
.byte	$30
.byte	$30
.byte	$30
.byte	$30
.byte	$30
.byte	$78
.byte	0
! 8116  0x1e, 0x0c, 0x0c, 0x0c, 0xcc, 0xcc, 0x78, 0x00,
.byte	$1E
.byte	$C
.byte	$C
.byte	$C
.byte	$CC
.byte	$CC
.byte	$78
.byte	0
! 8117  0xe6, 0x66, 0x6c, 0x78, 0x6c, 0x66, 0xe6, 0x00,
.byte	$E6
.byte	$66
.byte	$6C
.byte	$78
.byte	$6C
.byte	$66
.byte	$E6
.byte	0
! 8118  0xf0, 0x60, 0x60, 0x60, 0x62, 0x66, 0xfe, 0x00,
.byte	$F0
.byte	$60
.byte	$60
.byte	$60
.byte	$62
.byte	$66
.byte	$FE
.byte	0
! 8119  0xc6, 0xee, 0xfe, 0xfe, 0xd6, 0xc6, 0xc6, 0x00,
.byte	$C6
.byte	$EE
.byte	$FE
.byte	$FE
.byte	$D6
.byte	$C6
.byte	$C6
.byte	0
! 8120  0xc6, 0xe6, 0xf6, 0xde, 0xce, 0xc6, 0xc6, 0x00,
.byte	$C6
.byte	$E6
.byte	$F6
.byte	$DE
.byte	$CE
.byte	$C6
.byte	$C6
.byte	0
! 8121  0x38, 0x6c, 0xc6, 0xc6, 0xc6, 0x6c, 0x38, 0x00,
.byte	$38
.byte	$6C
.byte	$C6
.byte	$C6
.byte	$C6
.byte	$6C
.byte	$38
.byte	0
! 8122  0xfc, 0x66, 0x66, 0x7c, 0x60, 0x60, 0xf0, 0x00,
.byte	$FC
.byte	$66
.byte	$66
.byte	$7C
.byte	$60
.byte	$60
.byte	$F0
.byte	0
! 8123  0x78, 0xcc, 0xcc, 0xcc, 0xdc, 0x78, 0x1c, 0x00,
.byte	$78
.byte	$CC
.byte	$CC
.byte	$CC
.byte	$DC
.byte	$78
.byte	$1C
.byte	0
! 8124  0xfc, 0x66, 0x66, 0x7c, 0x6c, 0x66, 0xe6, 0x00,
.byte	$FC
.byte	$66
.byte	$66
.byte	$7C
.byte	$6C
.byte	$66
.byte	$E6
.byte	0
! 8125  0x78, 0xcc, 0xe0, 0x70, 0x1c, 0xcc, 0x78, 0x00,
.byte	$78
.byte	$CC
.byte	$E0
.byte	$70
.byte	$1C
.byte	$CC
.byte	$78
.byte	0
! 8126  0xfc, 0xb4, 0x30, 0x30, 0x30, 0x30, 0x78, 0x00,
.byte	$FC
.byte	$B4
.byte	$30
.byte	$30
.byte	$30
.byte	$30
.byte	$78
.byte	0
! 8127  0xcc, 0xcc, 0xcc, 0xcc, 0xcc, 0xcc, 0xfc, 0x00,
.byte	$CC
.byte	$CC
.byte	$CC
.byte	$CC
.byte	$CC
.byte	$CC
.byte	$FC
.byte	0
! 8128  0xcc, 0xcc, 0xcc, 0xcc, 0xcc, 0x78, 0x30, 0x00,
.byte	$CC
.byte	$CC
.byte	$CC
.byte	$CC
.byte	$CC
.byte	$78
.byte	$30
.byte	0
! 8129  0xc6, 0xc6, 0xc6, 0xd6, 0xfe, 0xee, 0xc6, 0x00,
.byte	$C6
.byte	$C6
.byte	$C6
.byte	$D6
.byte	$FE
.byte	$EE
.byte	$C6
.byte	0
! 8130  0xc6, 0xc6, 0x6c, 0x38, 0x38, 0x6c, 0xc6, 0x00,
.byte	$C6
.byte	$C6
.byte	$6C
.byte	$38
.byte	$38
.byte	$6C
.byte	$C6
.byte	0
! 8131  0xcc, 0xcc, 0xcc, 0x78, 0x30, 0x30, 0x78, 0x00,
.byte	$CC
.byte	$CC
.byte	$CC
.byte	$78
.byte	$30
.byte	$30
.byte	$78
.byte	0
! 8132  0xfe, 0xc6, 0x8c, 0x18, 0x32, 0x66, 0xfe, 0x00,
.byte	$FE
.byte	$C6
.byte	$8C
.byte	$18
.byte	$32
.byte	$66
.byte	$FE
.byte	0
! 8133  0x78, 0x60, 0x60, 0x60, 0x60, 0x60, 0x78, 0x00,
.byte	$78
.byte	$60
.byte	$60
.byte	$60
.byte	$60
.byte	$60
.byte	$78
.byte	0
! 8134  0xc0, 0x60, 0x30, 0x18, 0x0c, 0x06, 0x02, 0x00,
.byte	$C0
.byte	$60
.byte	$30
.byte	$18
.byte	$C
.byte	6
.byte	2
.byte	0
! 8135  0x78, 0x18, 0x18, 0x18, 0x18, 0x18, 0x78, 0x00,
.byte	$78
.byte	$18
.byte	$18
.byte	$18
.byte	$18
.byte	$18
.byte	$78
.byte	0
! 8136  0x10, 0x38, 0x6c, 0xc6, 0x00, 0x00, 0x00, 0x00,
.byte	$10
.byte	$38
.byte	$6C
.byte	$C6
.byte	0
.byte	0
.byte	0
.byte	0
! 8137  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff,
.byte	0
.byte	0
.byte	0
.byte	0
.byte	0
.byte	0
.byte	0
.byte	$FF
! 8138  0x30, 0x30, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00,
.byte	$30
.byte	$30
.byte	$18
.byte	0
.byte	0
.byte	0
.byte	0
.byte	0
! 8139  0x00, 0x00, 0x78, 0x0c, 0x7c, 0xcc, 0x76, 0x00,
.byte	0
.byte	0
.byte	$78
.byte	$C
.byte	$7C
.byte	$CC
.byte	$76
.byte	0
! 8140  0xe0, 0x60, 0x60, 0x7c, 0x66, 0x66, 0xdc, 0x00,
.byte	$E0
.byte	$60
.byte	$60
.byte	$7C
.byte	$66
.byte	$66
.byte	$DC
.byte	0
! 8141  0x00, 0x00, 0x78, 0xcc, 0xc0, 0xcc, 0x78, 0x00,
.byte	0
.byte	0
.byte	$78
.byte	$CC
.byte	$C0
.byte	$CC
.byte	$78
.byte	0
! 8142  0x1c, 0x0c, 0x0c, 0x7c, 0xcc, 0xcc, 0x76, 0x00,
.byte	$1C
.byte	$C
.byte	$C
.byte	$7C
.byte	$CC
.byte	$CC
.byte	$76
.byte	0
! 8143  0x00, 0x00, 0x78, 0xcc, 0xfc, 0xc0, 0x78, 0x00,
.byte	0
.byte	0
.byte	$78
.byte	$CC
.byte	$FC
.byte	$C0
.byte	$78
.byte	0
! 8144  0x38, 0x6c, 0x60, 0xf0, 0x60, 0x60, 0xf0, 0x00,
.byte	$38
.byte	$6C
.byte	$60
.byte	$F0
.byte	$60
.byte	$60
.byte	$F0
.byte	0
! 8145  0x00, 0x00, 0x76, 0xcc, 0xcc, 0x7c, 0x0c, 0xf8,
.byte	0
.byte	0
.byte	$76
.byte	$CC
.byte	$CC
.byte	$7C
.byte	$C
.byte	$F8
! 8146  0xe0, 0x60, 0x6c, 0x76, 0x66, 0x66, 0xe6, 0x0
.byte	$E0
.byte	$60
.byte	$6C
.byte	$76
.byte	$66
.byte	$66
.byte	$E6
! 8146 0,
.byte	0
! 8147  0x30, 0x00, 0x70, 0x30, 0x30, 0x30, 0x78, 0x00,
.byte	$30
.byte	0
.byte	$70
.byte	$30
.byte	$30
.byte	$30
.byte	$78
.byte	0
! 8148  0x0c, 0x00, 0x0c, 0x0c, 0x0c, 0xcc, 0xcc, 0x78,
.byte	$C
.byte	0
.byte	$C
.byte	$C
.byte	$C
.byte	$CC
.byte	$CC
.byte	$78
! 8149  0xe0, 0x60, 0x66, 0x6c, 0x78, 0x6c, 0xe6, 0x00,
.byte	$E0
.byte	$60
.byte	$66
.byte	$6C
.byte	$78
.byte	$6C
.byte	$E6
.byte	0
! 8150  0x70, 0x30, 0x30, 0x30, 0x30, 0x30, 0x78, 0x00,
.byte	$70
.byte	$30
.byte	$30
.byte	$30
.byte	$30
.byte	$30
.byte	$78
.byte	0
! 8151  0x00, 0x00, 0xcc, 0xfe, 0xfe, 0xd6, 0xc6, 0x00,
.byte	0
.byte	0
.byte	$CC
.byte	$FE
.byte	$FE
.byte	$D6
.byte	$C6
.byte	0
! 8152  0x00, 0x00, 0xf8, 0xcc, 0xcc, 0xcc, 0xcc, 0x00,
.byte	0
.byte	0
.byte	$F8
.byte	$CC
.byte	$CC
.byte	$CC
.byte	$CC
.byte	0
! 8153  0x00, 0x00, 0x78, 0xcc, 0xcc, 0xcc, 0x78, 0x00,
.byte	0
.byte	0
.byte	$78
.byte	$CC
.byte	$CC
.byte	$CC
.byte	$78
.byte	0
! 8154  0x00, 0x00, 0xdc, 0x66, 0x66, 0x7c, 0x60, 0xf0,
.byte	0
.byte	0
.byte	$DC
.byte	$66
.byte	$66
.byte	$7C
.byte	$60
.byte	$F0
! 8155  0x00, 0x00, 0x76, 0xcc, 0xcc, 0x7c, 0x0c, 0x1e,
.byte	0
.byte	0
.byte	$76
.byte	$CC
.byte	$CC
.byte	$7C
.byte	$C
.byte	$1E
! 8156  0x00, 0x00, 0xdc, 0x76, 0x66, 0x60, 0xf0, 0x00,
.byte	0
.byte	0
.byte	$DC
.byte	$76
.byte	$66
.byte	$60
.byte	$F0
.byte	0
! 8157  0x00, 0x00, 0x7c, 0xc0, 0x78, 0x0c, 0xf8, 0x00,
.byte	0
.byte	0
.byte	$7C
.byte	$C0
.byte	$78
.byte	$C
.byte	$F8
.byte	0
! 8158  0x10, 0x30, 0x7c, 0x30, 0x30, 0x34, 0x18, 0x00,
.byte	$10
.byte	$30
.byte	$7C
.byte	$30
.byte	$30
.byte	$34
.byte	$18
.byte	0
! 8159  0x00, 0x00, 0xcc, 0xcc, 0xcc, 0xcc, 0x76, 0x00,
.byte	0
.byte	0
.byte	$CC
.byte	$CC
.byte	$CC
.byte	$CC
.byte	$76
.byte	0
! 8160  0x00, 0x00, 0xcc, 0xcc, 0xcc, 0x78, 0x30, 0x00,
.byte	0
.byte	0
.byte	$CC
.byte	$CC
.byte	$CC
.byte	$78
.byte	$30
.byte	0
! 8161  0x00, 0x00, 0xc6, 0xd6, 0xfe, 0xfe, 0x6c, 0x00,
.byte	0
.byte	0
.byte	$C6
.byte	$D6
.byte	$FE
.byte	$FE
.byte	$6C
.byte	0
! 8162  0x00, 0x00, 0xc6, 0x6c, 0x38, 0x6c, 0xc6, 0x00,
.byte	0
.byte	0
.byte	$C6
.byte	$6C
.byte	$38
.byte	$6C
.byte	$C6
.byte	0
! 8163  0x00, 0x00, 0xcc, 0xcc, 0xcc, 0x7c, 0x0c, 0xf8,
.byte	0
.byte	0
.byte	$CC
.byte	$CC
.byte	$CC
.byte	$7C
.byte	$C
.byte	$F8
! 8164  0x00, 0x00, 0xfc, 0x98, 0x30, 0x64, 0xfc, 0x00,
.byte	0
.byte	0
.byte	$FC
.byte	$98
.byte	$30
.byte	$64
.byte	$FC
.byte	0
! 8165  0x1c, 0x30, 0x30, 0xe0, 0x30, 0x30, 0x1c, 0x00,
.byte	$1C
.byte	$30
.byte	$30
.byte	$E0
.byte	$30
.byte	$30
.byte	$1C
.byte	0
! 8166  0x18, 0x18, 0x18, 0x00, 0x18, 0x18, 0x18, 0x00,
.byte	$18
.byte	$18
.byte	$18
.byte	0
.byte	$18
.byte	$18
.byte	$18
.byte	0
! 8167  0xe0, 0x30, 0x30, 0x1c, 0x30, 0x30, 0xe0, 0x00,
.byte	$E0
.byte	$30
.byte	$30
.byte	$1C
.byte	$30
.byte	$30
.byte	$E0
.byte	0
! 8168  0x76, 0xdc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
.byte	$76
.byte	$DC
.byte	0
.byte	0
.byte	0
.byte	0
.byte	0
.byte	0
! 8169  0x00, 0x10, 0x38, 0x6c, 0xc6, 0xc6, 0xfe, 0x00,
.byte	0
.byte	$10
.byte	$38
.byte	$6C
.byte	$C6
.byte	$C6
.byte	$FE
.byte	0
! 8170 };
!BCC_EOS
! 8171 #asm
!BCC_ASM
.org 0xcb00
jmptable:
db 0x5F, 0x5F, 0x5F, 0x4A, 0x4D, 0x50, 0x54 ;; ___JMPT
dw 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 ;; 64 bytes
dw 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 ;; 128 bytes
dw 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 ;; 192 bytes
.org 0xcc00
db 0x5F, 0x5F, 0x5F, 0x48, 0x56, 0x4D, 0x4D, 0x50 ;; ___HVMMP
dw 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 ;; 64 bytes
dw 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 ;; 128 bytes
dw 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 ;; 192 bytes
dw 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 ;; 256 bytes
dw 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 ;; 320 bytes
dw 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 ;; 384 bytes
dw 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 ;; 448 bytes
dw 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 ;; 512 bytes
dw 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 ;; 576 bytes
dw 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 ;; 640 bytes
dw 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 ;; 704 bytes
dw 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 ;; 768 bytes
dw 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 ;; 832 bytes
dw 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 ;; 896 bytes
.align 16
smbios_entry_point:
db 0,0,0,0,0,0,0,0 ; 8 bytes
db 0,0,0,0,0,0,0,0 ; 16 bytes
db 0,0,0,0,0,0,0,0 ; 24 bytes
db 0,0,0,0,0,0,0 ; 31 bytes
! 8200 endasm
!BCC_ENDASM
! 8201 
.6F9:
.71E:
.ascii	"PCI device %04x:%04x not found"
.byte	$A
.byte	0
.6F5:
.71F:
.ascii	"bad PCI vendor ID %04x"
.byte	$A
.byte	0
.6F1:
.720:
.ascii	"unsupported PCI BIOS function 0x%02x"
.byte	$A
.byte	0
.6ED:
.721:
.ascii	"PCI BIOS: PCI not present"
.byte	$A
.byte	0
.6BE:
.722:
.ascii	"Invalid boot device (0x%x)"
.byte	$A
.byte	0
.6BB:
.723:
.ascii	"No bootable device."
.byte	$A
.byte	0
.6A8:
.724:
.ascii	"set_diskette_current_cyl(): drive > 1"
.byte	$A
.byte	0
.6A3:
.725:
.ascii	"int13_diskette: unsupported AH=%02x"
.byte	$A
.byte	0
.691:
.726:
.ascii	"floppy: int13: bad floppy type"
.byte	$A
.byte	0
.679:
.727:
.ascii	"int13_diskette_function: write error"
.byte	$A
.byte	0
.673:
.728:
.ascii	"int13_diskette: ctrl not ready"
.byte	$A
.byte	0
.66C:
.729:
.ascii	"int13_diskette:f05: ctrl not ready"
.byte	$A
.byte	0
.654:
.72A:
.ascii	"int13_diskette_function: read error"
.byte	$A
.byte	0
.64E:
.72B:
.ascii	"int13_diskette: ctrl not ready"
.byte	$A
.byte	0
.647:
.72C:
.ascii	"int13_diskette:f03: ctrl not ready"
.byte	$A
.byte	0
.638:
.72D:
.ascii	"int13_diskette: ctrl not ready"
.byte	$A
.byte	0
.631:
.72E:
.ascii	"int13_diskette:f02: ctrl not ready"
.byte	$A
.byte	0
.61F:
.72F:
.ascii	"floppy: drive>1 || head>1 ..."
.byte	$A
.byte	0
.5FB:
.730:
.ascii	"floppy recal:f07: ctrl not ready"
.byte	$A
.byte	0
.5C9:
.731:
.ascii	"int13_cdemu function AH=%02x unsupported"
.ascii	", returns fail"
.byte	$A
.byte	0
.5B1:
.732:
.ascii	"int13_cdemu: function %02x, error %02x !"
.byte	$A
.byte	0
.593:
.733:
.ascii	"int13_cdemu: function %02x, emulation no"
.ascii	"t active for DL= %02x"
.byte	$A
.byte	0
.58F:
.734:
.ascii	"int13_eltorito: unsupported AH=%02x"
.byte	$A
.byte	0
.58A:
.735:
.ascii	"Int13 eltorito call with AX=%04x. Please"
.ascii	" report"
.byte	$A
.byte	0
.57F:
.736:
.ascii	"int13_cdrom: unsupported AH=%02x"
.byte	$A
.byte	0
.538:
.737:
.ascii	"int13_cdrom: function %02x, status %02x "
.ascii	"!"
.byte	$A
.byte	0
.532:
.738:
.ascii	"int13_cdrom: function %02x. Can't use 64"
.ascii	"bits lba"
.byte	$A
.byte	0
.518:
.739:
.ascii	"int13_cdrom: function %02x, unmapped dev"
.ascii	"ice for ELDL=%02x"
.byte	$A
.byte	0
.515:
.73A:
.ascii	"int13_cdrom: function %02x, ELDL out of "
.ascii	"range %02x"
.byte	$A
.byte	0
.50D:
.73B:
.ascii	"int13_harddisk function %02xh unsupporte"
.ascii	"d, returns fail"
.byte	$A
.byte	0
.507:
.73C:
.ascii	"int13h_harddisk function %02xh unimpleme"
.ascii	"nted, returns success"
.byte	$A
.byte	0
.4D0:
.73D:
.ascii	"int13_harddisk: function %02x, error %02"
.ascii	"x !"
.byte	$A
.byte	0
.4C7:
.73E:
.ascii	"int13_harddisk: function %02x. LBA out o"
.ascii	"f range"
.byte	$A
.byte	0
.4C4:
.73F:
.ascii	"int13_harddisk: function %02x. Can't use"
.ascii	" 64bits lba"
.byte	$A
.byte	0
.4B6:
.740:
.ascii	"format disk track called"
.byte	$A
.byte	0
.4B4:
.741:
.ascii	"int13_harddisk: function %02x, error %02"
.ascii	"x !"
.byte	$A
.byte	0
.4A9:
.742:
.ascii	"int13_harddisk: function %02x, parameter"
.ascii	"s out of range %04x/%04x/%04x!"
.byte	$A
.byte	0
.4A4:
.743:
.ascii	"int13_harddisk: function %02x, count out"
.ascii	" of range!"
.byte	$A
.byte	0
.495:
.744:
.ascii	"int13_harddisk: function %02x, unmapped "
.ascii	"device for ELDL=%02x"
.byte	$A
.byte	0
.492:
.745:
.ascii	"int13_harddisk: function %02x, ELDL out "
.ascii	"of range %02x"
.byte	$A
.byte	0
.481:
.746:
.ascii	"KBD: int09h_handler(): scancode & asciic"
.ascii	"ode are zero?"
.byte	$A
.byte	0
.46E:
.747:
.ascii	"KBD: int09h_handler(): unknown scancode "
.ascii	"(%x) read!"
.byte	$A
.byte	0
.444:
.748:
.ascii	"KBD: int09 handler: AL=0"
.byte	$A
.byte	0
.441:
.749:
.ascii	"setkbdcomm"
.byte	0
.43A:
.74A:
.ascii	"sendmouse"
.byte	0
.437:
.74B:
.ascii	"enabmouse"
.byte	0
.430:
.74C:
.ascii	"enabmouse"
.byte	0
.42D:
.74D:
.ascii	"inhibmouse"
.byte	0
.426:
.74E:
.ascii	"inhibmouse"
.byte	0
.419:
.74F:
.ascii	"KBD: unsupported int 16h function %02x"
.byte	$A
.byte	0
.408:
.750:
.ascii	"KBD: int16h: out of keyboard input"
.byte	$A
.byte	0
.3DA:
.751:
.ascii	"KBD: int16h: out of keyboard input"
.byte	$A
.byte	0
.3D3:
.752:
.ascii	"*** int 15h function AX=%04x, BX=%04x no"
.ascii	"t yet supported!"
.byte	$A
.byte	0
.3A2:
.753:
.ascii	"*** int 15h function AX=%04x, BX=%04x no"
.ascii	"t yet supported!"
.byte	$A
.byte	0
.396:
.754:
.ascii	"INT 15h C2 AL=6, BH=%02x"
.byte	$A
.byte	0
.380:
.755:
.ascii	"Mouse status returned %02x (should be ac"
.ascii	"k)"
.byte	$A
.byte	0
.357:
.756:
.ascii	"Mouse reset returned %02x (should be ack"
.ascii	")"
.byte	$A
.byte	0
.330:
.757:
.ascii	"*** int 15h function AX=%04x, BX=%04x no"
.ascii	"t yet supported!"
.byte	$A
.byte	0
.32E:
.758:
.ascii	"EISA BIOS not present"
.byte	$A
.byte	0
.32A:
.759:
.ascii	"*** int 15h function AH=bf not yet suppo"
.ascii	"rted!"
.byte	$A
.byte	0
.313:
.75A:
.ascii	"int15: Func 24h, subfunc %02xh, A20 gate"
.ascii	" control not supported"
.byte	$A
.byte	0
.258:
.75B:
.ascii	"ata_cmd_packet: DATA_OUT not supported y"
.ascii	"et"
.byte	$A
.byte	0
.1F5:
.75C:
.byte	$A
.byte	0
.1F0:
.75D:
.ascii	"master"
.byte	0
.1EF:
.75E:
.ascii	" slave"
.byte	0
.1EE:
.75F:
.ascii	"ata%d %s: Unknown device"
.byte	$A
.byte	0
.1EC:
.760:
.ascii	" ATAPI-%d Device"
.byte	$A
.byte	0
.1EA:
.761:
.ascii	" ATAPI-%d CD-Rom/DVD-Rom"
.byte	$A
.byte	0
.1E6:
.762:
.ascii	"%c"
.byte	0
.1DF:
.763:
.ascii	"master"
.byte	0
.1DE:
.764:
.ascii	" slave"
.byte	0
.1DD:
.765:
.ascii	"ata%d %s: "
.byte	0
.1DB:
.766:
.ascii	" ATA-%d Hard-Disk (%04u GBytes)"
.byte	$A
.byte	0
.1D9:
.767:
.ascii	" ATA-%d Hard-Disk (%04u MBytes)"
.byte	$A
.byte	0
.1D5:
.768:
.ascii	"%c"
.byte	0
.1CE:
.769:
.ascii	"master"
.byte	0
.1CD:
.76A:
.ascii	" slave"
.byte	0
.1CC:
.76B:
.ascii	"ata%d %s: "
.byte	0
.1A8:
.76C:
.ascii	"ata-detect: Failed to detect ATAPI devic"
.ascii	"e"
.byte	$A
.byte	0
.1A3:
.76D:
.ascii	" LCHS=%d/%d/%d"
.byte	$A
.byte	0
.183:
.76E:
.ascii	"r-echs"
.byte	0
.181:
.76F:
.ascii	"large"
.byte	0
.17F:
.770:
.ascii	"lba"
.byte	0
.17D:
.771:
.ascii	"none"
.byte	0
.173:
.772:
.ascii	"ata%d-%d: PCHS=%u/%d/%d translation="
.byte	0
.16C:
.773:
.ascii	"ata-detect: Failed to detect ATA device"
.byte	$A
.byte	0
.137:
.774:
.ascii	"%s"
.byte	$A
.byte	0
.136:
.775:
.ascii	"INT18: BOOT FAILURE"
.byte	$A
.byte	0
.135:
.776:
.ascii	"NMI Handler called"
.byte	$A
.byte	0
.12A:
.777:
.byte	$A,$A
.ascii	"Press F10 to select boot device."
.byte	$A
.byte	0
.109:
.778:
.byte	$A,$A
.ascii	"          Currently selected: %d"
.byte	$A
.byte	0
.108:
.779:
.ascii	"            4. Network"
.byte	$A
.byte	0
.105:
.77A:
.ascii	"            3. CD-ROM"
.byte	$A
.byte	0
.104:
.77B:
.ascii	"            2. Hard drive"
.byte	$A
.byte	0
.103:
.77C:
.ascii	"            1. Floppy"
.byte	$A
.byte	0
.102:
.77D:
.ascii	"          Select boot device"
.byte	$A,$A
.byte	0
.101:
.77E:
.byte	$A,$A,$A,$A,$A,$A,$A
.byte	0
.F7:
.77F:
.ascii	"Key pressed: %x"
.byte	$A
.byte	0
.EC:
.780:
.ascii	"CDROM boot failure code : %04x"
.byte	$A
.byte	0
.EB:
.781:
.byte	$A
.byte	0
.EA:
.782:
.ascii	": could not read the boot disk"
.byte	0
.E8:
.783:
.ascii	": not a bootable disk"
.byte	0
.E3:
.784:
.ascii	"Boot from %s failed"
.byte	0
.E2:
.785:
.ascii	"Bad drive type"
.byte	$A
.byte	0
.DE:
.786:
.ascii	"Booting from %s..."
.byte	$A
.byte	0
.DD:
.787:
.ascii	"Bad drive type"
.byte	$A
.byte	0
.D0:
.788:
.byte	$A
.byte	0
.CF:
.789:
.ascii	"TCG-enabled BIOS."
.byte	$A
.byte	0
.CE:
.78A:
.ascii	"%s %s"
.byte	$A
.byte	0
.CD:
.78B:
.byte	0
.CC:
.78C:
.ascii	"s"
.byte	0
.CB:
.78D:
.ascii	"HVMAssist BIOS, %d cpu%s, "
.byte	0
.CA:
.78E:
.ascii	"Unimplemented shutdown status: %02x"
.byte	$A
.byte	0
.C9:
.78F:
.ascii	"Couldn't reset the machine"
.byte	$A
.byte	0
.C8:
.790:
.ascii	"Keyboard error:%u"
.byte	$A
.byte	0
.51:
.791:
.ascii	"bios_printf: unknown format"
.byte	$A
.byte	0
.28:
.792:
.ascii	"FATAL: "
.byte	0
.bss

! 0 errors detected
