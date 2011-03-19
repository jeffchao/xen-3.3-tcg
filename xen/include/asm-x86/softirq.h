#ifndef __ASM_SOFTIRQ_H__
#define __ASM_SOFTIRQ_H__

#define NMI_MCE_SOFTIRQ        (NR_COMMON_SOFTIRQS + 0)
#define TIME_CALIBRATE_SOFTIRQ (NR_COMMON_SOFTIRQS + 1)
#define VCPU_KICK_SOFTIRQ      (NR_COMMON_SOFTIRQS + 2)

#define NR_ARCH_SOFTIRQS       3

#endif /* __ASM_SOFTIRQ_H__ */