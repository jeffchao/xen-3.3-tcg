
/*
 * public xen defines and struct for ia64
 * generated by mkheader.py -- DO NOT EDIT
 */

#ifndef __FOREIGN_IA64_H__
#define __FOREIGN_IA64_H__ 1


#define __align8__ __attribute__((aligned (8)))
#define __align16__ __attribute__((aligned (16)))
typedef unsigned char ldouble_t[16];

#define MAX_VIRT_CPUS_IA64 64
#define VGCF_EXTRA_REGS_IA64 (1UL << 1)	/* Set extra regs.  */
#define VGCF_online_IA64     (1UL << 3)  /* make this vcpu online */
#define MAX_GUEST_CMDLINE_IA64 1024

union vcpu_cr_regs_ia64 {
    __align8__ uint64_t cr[128];
    struct {
        __align8__ uint64_t dcr;  // CR0
        __align8__ uint64_t itm;
        __align8__ uint64_t iva;
        __align8__ uint64_t rsv1[5];
        __align8__ uint64_t pta;  // CR8
        __align8__ uint64_t rsv2[7];
        __align8__ uint64_t ipsr;  // CR16
        __align8__ uint64_t isr;
        __align8__ uint64_t rsv3;
        __align8__ uint64_t iip;
        __align8__ uint64_t ifa;
        __align8__ uint64_t itir;
        __align8__ uint64_t iipa;
        __align8__ uint64_t ifs;
        __align8__ uint64_t iim;  // CR24
        __align8__ uint64_t iha;
        __align8__ uint64_t rsv4[38];
        __align8__ uint64_t lid;  // CR64
        __align8__ uint64_t ivr;
        __align8__ uint64_t tpr;
        __align8__ uint64_t eoi;
        __align8__ uint64_t irr[4];
        __align8__ uint64_t itv;  // CR72
        __align8__ uint64_t pmv;
        __align8__ uint64_t cmcv;
        __align8__ uint64_t rsv5[5];
        __align8__ uint64_t lrr0;  // CR80
        __align8__ uint64_t lrr1;
        __align8__ uint64_t rsv6[46];
    };
};

union vcpu_ar_regs_ia64 {
    __align8__ uint64_t ar[128];
    struct {
        __align8__ uint64_t kr[8];
        __align8__ uint64_t rsv1[8];
        __align8__ uint64_t rsc;
        __align8__ uint64_t bsp;
        __align8__ uint64_t bspstore;
        __align8__ uint64_t rnat;
        __align8__ uint64_t rsv2;
        __align8__ uint64_t fcr;
        __align8__ uint64_t rsv3[2];
        __align8__ uint64_t eflag;
        __align8__ uint64_t csd;
        __align8__ uint64_t ssd;
        __align8__ uint64_t cflg;
        __align8__ uint64_t fsr;
        __align8__ uint64_t fir;
        __align8__ uint64_t fdr;
        __align8__ uint64_t rsv4;
        __align8__ uint64_t ccv; 
        __align8__ uint64_t rsv5[3];
        __align8__ uint64_t unat;
        __align8__ uint64_t rsv6[3];
        __align8__ uint64_t fpsr;
        __align8__ uint64_t rsv7[3];
        __align8__ uint64_t itc;
        __align8__ uint64_t rsv8[3];
        __align8__ uint64_t ign1[16];
        __align8__ uint64_t pfs; 
        __align8__ uint64_t lc;
        __align8__ uint64_t ec;
        __align8__ uint64_t rsv9[45];
        __align8__ uint64_t ign2[16];
    };
};

struct start_info_ia64 {
    char magic[32];             
    __align8__ uint64_t nr_pages;     
    __align8__ uint64_t shared_info;  
    uint32_t flags;             
    __align8__ uint64_t store_mfn;        
    uint32_t store_evtchn;      
    union {
        struct {
            __align8__ uint64_t mfn;      
            uint32_t  evtchn;   
        } domU;
        struct {
            uint32_t info_off;  
            uint32_t info_size; 
        } dom0;
    } console;
    __align8__ uint64_t pt_base;      
    __align8__ uint64_t nr_pt_frames; 
    __align8__ uint64_t mfn_list;     
    __align8__ uint64_t mod_start;    
    __align8__ uint64_t mod_len;      
    int8_t cmd_line[MAX_GUEST_CMDLINE_IA64];
};
typedef struct start_info_ia64 start_info_ia64_t;

#define ia64_has_no_trap_info 1

struct pt_fpreg_ia64 {
    union {
        __align8__ uint64_t bits[2];
        __align16__ ldouble_t __dummy;    
    } u;
};
typedef struct pt_fpreg_ia64 pt_fpreg_ia64_t;

#define ia64_has_no_cpu_user_regs 1

struct xen_ia64_boot_param_ia64 {
	__align8__ uint64_t command_line;	
	__align8__ uint64_t efi_systab;	
	__align8__ uint64_t efi_memmap;	
	__align8__ uint64_t efi_memmap_size;	
	__align8__ uint64_t efi_memdesc_size;	
	unsigned int  efi_memdesc_version;	
	struct {
		unsigned short num_cols;	
		unsigned short num_rows;	
		unsigned short orig_x;	
		unsigned short orig_y;	
	} console_info;
	__align8__ uint64_t fpswa;		
	__align8__ uint64_t initrd_start;
	__align8__ uint64_t initrd_size;
	__align8__ uint64_t domain_start;	
	__align8__ uint64_t domain_size;	
};
typedef struct xen_ia64_boot_param_ia64 xen_ia64_boot_param_ia64_t;

struct ia64_tr_entry_ia64 {
    __align8__ uint64_t pte;
    __align8__ uint64_t itir;
    __align8__ uint64_t vadr;
    __align8__ uint64_t rid;
};
typedef struct ia64_tr_entry_ia64 ia64_tr_entry_ia64_t;

struct vcpu_tr_regs_ia64 {
    struct ia64_tr_entry_ia64 itrs[12];
    struct ia64_tr_entry_ia64 dtrs[12];
};
typedef struct vcpu_tr_regs_ia64 vcpu_tr_regs_ia64_t;

struct vcpu_guest_context_regs_ia64 {
        __align8__ uint64_t r[32];
        __align8__ uint64_t b[8];
        __align8__ uint64_t bank[16];
        __align8__ uint64_t ip;
        __align8__ uint64_t psr;
        __align8__ uint64_t cfm;
        __align8__ uint64_t pr;
        unsigned int nats; 
        unsigned int bnats; 
        union vcpu_ar_regs_ia64 ar;
        union vcpu_cr_regs_ia64 cr;
        struct pt_fpreg_ia64 f[128];
        __align8__ uint64_t dbr[8];
        __align8__ uint64_t ibr[8];
        __align8__ uint64_t rr[8];
        __align8__ uint64_t pkr[16];
        __align8__ uint64_t xip;
        __align8__ uint64_t xpsr;
        __align8__ uint64_t xfs;
        __align8__ uint64_t xr[4];
        struct vcpu_tr_regs_ia64 tr;
        __align8__ uint64_t excp_iipa;
        __align8__ uint64_t excp_ifa;
        __align8__ uint64_t excp_isr;
        unsigned int excp_vector;
        unsigned int rbs_voff;
        __align8__ uint64_t rbs[2048];
        __align8__ uint64_t rbs_rnat;
        __align8__ uint64_t num_phys_stacked;
};
typedef struct vcpu_guest_context_regs_ia64 vcpu_guest_context_regs_ia64_t;

struct vcpu_guest_context_ia64 {
    __align8__ uint64_t flags;       
    struct vcpu_guest_context_regs_ia64 regs;
    __align8__ uint64_t event_callback_ip;
    __align8__ uint64_t privregs_pfn;
};
typedef struct vcpu_guest_context_ia64 vcpu_guest_context_ia64_t;

struct arch_vcpu_info_ia64 {
};
typedef struct arch_vcpu_info_ia64 arch_vcpu_info_ia64_t;

struct vcpu_time_info_ia64 {
    uint32_t version;
    uint32_t pad0;
    uint64_t tsc_timestamp;   
    uint64_t system_time;     
    uint32_t tsc_to_system_mul;
    int8_t   tsc_shift;
    int8_t   pad1[3];
};
typedef struct vcpu_time_info_ia64 vcpu_time_info_ia64_t;

struct vcpu_info_ia64 {
    uint8_t evtchn_upcall_pending;
    uint8_t evtchn_upcall_mask;
    __align8__ uint64_t evtchn_pending_sel;
    struct arch_vcpu_info_ia64 arch;
    struct vcpu_time_info_ia64 time;
};
typedef struct vcpu_info_ia64 vcpu_info_ia64_t;

struct arch_shared_info_ia64 {
    __align8__ uint64_t start_info_pfn;
    int evtchn_vector;
    unsigned int memmap_info_num_pages;
    __align8__ uint64_t memmap_info_pfn;
    uint64_t pad[31];
};
typedef struct arch_shared_info_ia64 arch_shared_info_ia64_t;

struct shared_info_ia64 {
    struct vcpu_info_ia64 vcpu_info[MAX_VIRT_CPUS_IA64];
    __align8__ uint64_t evtchn_pending[sizeof(__align8__ uint64_t) * 8];
    __align8__ uint64_t evtchn_mask[sizeof(__align8__ uint64_t) * 8];
    uint32_t wc_version;      
    uint32_t wc_sec;          
    uint32_t wc_nsec;         
    struct arch_shared_info_ia64 arch;
};
typedef struct shared_info_ia64 shared_info_ia64_t;

#endif /* __FOREIGN_IA64_H__ */
