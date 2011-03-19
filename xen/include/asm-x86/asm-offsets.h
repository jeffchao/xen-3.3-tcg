/*
 * DO NOT MODIFY.
 *
 * This file was auto-generated from arch/x86/asm-offsets.s
 *
 */

#ifndef __ASM_OFFSETS_H__
#define __ASM_OFFSETS_H__

#define UREGS_eax 24 /* offsetof(struct cpu_user_regs, eax) */
#define UREGS_ebx 0 /* offsetof(struct cpu_user_regs, ebx) */
#define UREGS_ecx 4 /* offsetof(struct cpu_user_regs, ecx) */
#define UREGS_edx 8 /* offsetof(struct cpu_user_regs, edx) */
#define UREGS_esi 12 /* offsetof(struct cpu_user_regs, esi) */
#define UREGS_edi 16 /* offsetof(struct cpu_user_regs, edi) */
#define UREGS_esp 44 /* offsetof(struct cpu_user_regs, esp) */
#define UREGS_ebp 20 /* offsetof(struct cpu_user_regs, ebp) */
#define UREGS_eip 32 /* offsetof(struct cpu_user_regs, eip) */
#define UREGS_cs 36 /* offsetof(struct cpu_user_regs, cs) */
#define UREGS_ds 56 /* offsetof(struct cpu_user_regs, ds) */
#define UREGS_es 52 /* offsetof(struct cpu_user_regs, es) */
#define UREGS_fs 60 /* offsetof(struct cpu_user_regs, fs) */
#define UREGS_gs 64 /* offsetof(struct cpu_user_regs, gs) */
#define UREGS_ss 48 /* offsetof(struct cpu_user_regs, ss) */
#define UREGS_eflags 40 /* offsetof(struct cpu_user_regs, eflags) */
#define UREGS_error_code 28 /* offsetof(struct cpu_user_regs, error_code) */
#define UREGS_entry_vector 30 /* offsetof(struct cpu_user_regs, entry_vector) */
#define UREGS_saved_upcall_mask 38 /* offsetof(struct cpu_user_regs, saved_upcall_mask) */
#define UREGS_kernel_sizeof 44 /* offsetof(struct cpu_user_regs, esp) */
#define UREGS_user_sizeof 68 /* sizeof(struct cpu_user_regs) */

#define VCPU_processor 4 /* offsetof(struct vcpu, processor) */
#define VCPU_vcpu_info 8 /* offsetof(struct vcpu, vcpu_info) */
#define VCPU_trap_bounce 3188 /* offsetof(struct vcpu, arch.trap_bounce) */
#define VCPU_thread_flags 3168 /* offsetof(struct vcpu, arch.flags) */
#define VCPU_event_sel 3036 /* offsetof(struct vcpu, arch.guest_context.event_callback_cs) */
#define VCPU_event_addr 3040 /* offsetof(struct vcpu, arch.guest_context.event_callback_eip) */
#define VCPU_failsafe_sel 3044 /* offsetof(struct vcpu, arch.guest_context.failsafe_callback_cs) */
#define VCPU_failsafe_addr 3048 /* offsetof(struct vcpu, arch.guest_context.failsafe_callback_eip) */
#define VCPU_kernel_ss 2964 /* offsetof(struct vcpu, arch.guest_context.kernel_ss) */
#define VCPU_kernel_sp 2968 /* offsetof(struct vcpu, arch.guest_context.kernel_sp) */
#define VCPU_guest_context_flags 768 /* offsetof(struct vcpu, arch.guest_context.flags) */
#define VCPU_nmi_pending 166 /* offsetof(struct vcpu, nmi_pending) */
#define VCPU_mce_pending 165 /* offsetof(struct vcpu, mce_pending) */
#define VCPU_old_trap_priority 168 /* offsetof(struct vcpu, old_trap_priority) */
#define VCPU_trap_priority 170 /* offsetof(struct vcpu, trap_priority) */
#define VCPU_TRAP_NMI 1 /* VCPU_TRAP_NMI */
#define VCPU_TRAP_MCE 2 /* VCPU_TRAP_MCE */
#define _VGCF_failsafe_disables_events 3 /* _VGCF_failsafe_disables_events */

#define TSS_ss0 8 /* offsetof(struct tss_struct, ss0) */
#define TSS_esp0 4 /* offsetof(struct tss_struct, esp0) */
#define TSS_ss1 16 /* offsetof(struct tss_struct, ss1) */
#define TSS_esp1 12 /* offsetof(struct tss_struct, esp1) */
#define TSS_sizeof 128 /* sizeof(struct tss_struct) */

#define VCPU_svm_vmcb_pa 3444 /* offsetof(struct vcpu, arch.hvm_vcpu.u.svm.vmcb_pa) */
#define VCPU_svm_vmcb 3440 /* offsetof(struct vcpu, arch.hvm_vcpu.u.svm.vmcb) */
#define VCPU_svm_vmcb_in_sync 3468 /* offsetof(struct vcpu, arch.hvm_vcpu.u.svm.vmcb_in_sync) */

#define VCPU_vmx_launched 3460 /* offsetof(struct vcpu, arch.hvm_vcpu.u.vmx.launched) */
#define VCPU_vmx_emul 3512 /* offsetof(struct vcpu, arch.hvm_vcpu.u.vmx.vmxemul) */
#define VCPU_hvm_guest_cr2 3228 /* offsetof(struct vcpu, arch.hvm_vcpu.guest_cr[2]) */

#define VMCB_rax 1528 /* offsetof(struct vmcb_struct, rax) */
#define VMCB_rip 1400 /* offsetof(struct vmcb_struct, rip) */
#define VMCB_rsp 1496 /* offsetof(struct vmcb_struct, rsp) */
#define VMCB_rflags 1392 /* offsetof(struct vmcb_struct, rflags) */

#define VCPUINFO_upcall_pending 0 /* offsetof(vcpu_info_t, evtchn_upcall_pending) */
#define VCPUINFO_upcall_mask 1 /* offsetof(vcpu_info_t, evtchn_upcall_mask) */

#define CPUINFO_sizeof 76 /* sizeof(struct cpu_info) */

#define TRAPBOUNCE_error_code 0 /* offsetof(struct trap_bounce, error_code) */
#define TRAPBOUNCE_flags 4 /* offsetof(struct trap_bounce, flags) */
#define TRAPBOUNCE_cs 6 /* offsetof(struct trap_bounce, cs) */
#define TRAPBOUNCE_eip 8 /* offsetof(struct trap_bounce, eip) */

#define FIXMAP_apic_base -135168 /* fix_to_virt(FIX_APIC_BASE) */

#define IRQSTAT_shift 7 /* LOG_2(sizeof(irq_cpustat_t)) */

#define CPUINFO_ext_features 12 /* offsetof(struct cpuinfo_x86, x86_capability[1]) */

#endif
