/*
 * psr.c: Platform Shared Resource related service for guest.
 *
 * Copyright (c) 2014, Intel Corporation
 * Author: Dongxiao Xu <dongxiao.xu@intel.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */
#include <xen/init.h>
#include <xen/cpu.h>
#include <xen/err.h>
#include <xen/sched.h>
#include <xen/list.h>
#include <asm/psr.h>

/*
 * Terminology:
 * - CAT         Cache Allocation Technology
 * - CBM         Capacity BitMasks
 * - CDP         Code and Data Prioritization
 * - COS/CLOS    Class of Service. Also mean COS registers.
 * - COS_MAX     Max number of COS for the feature (minus 1)
 * - MSRs        Machine Specific Registers
 * - PSR         Intel Platform Shared Resource
 */

#define PSR_CMT        (1<<0)
#define PSR_CAT        (1<<1)
#define PSR_CDP        (1<<2)

/*
 * Per SDM chapter 'Cache Allocation Technology: Cache Mask Configuration',
 * the MSRs ranging from 0C90H through 0D0FH (inclusive), enables support for
 * up to 128 L3 CAT Classes of Service. The COS_ID=[0,127].
 *
 * The MSRs ranging from 0D10H through 0D4FH (inclusive), enables support for
 * up to 64 L2 CAT COS. The COS_ID=[0,63].
 *
 * So, the maximum COS register count of one feature is 128.
 */
#define MAX_COS_REG_CNT  128

/*
 * PSR features are managed per socket. Below structure defines the members
 * used to manage these features.
 * feat_mask - Mask used to record features enabled on socket. There may be
 *             some features enabled at same time.
 * nr_feat   - Record how many features enabled.
 * feat_list - A list used to manage all features enabled.
 * cos_ref   - A reference count array to record how many domains are using the
 *             COS_ID.
 *             Every entry of cos_ref corresponds to one COS ID.
 * ref_lock  - A lock to protect cos_ref.
 */
struct psr_socket_info {
    /*
     * It maps to values defined in 'enum psr_feat_type' below. Value in 'enum
     * psr_feat_type' means the bit position.
     * bit 0:   L3 CAT
     * bit 1:   L3 CDP
     * bit 2:   L2 CAT
     */
    unsigned int feat_mask;
    unsigned int nr_feat;
    struct list_head feat_list;
    unsigned int cos_ref[MAX_COS_REG_CNT];
    spinlock_t ref_lock;
};

enum psr_feat_type {
    PSR_SOCKET_L3_CAT = 0,
    PSR_SOCKET_L3_CDP,
    PSR_SOCKET_L2_CAT,
};

/* CAT/CDP HW info data structure. */
struct psr_cat_hw_info {
    unsigned int cbm_len;
    unsigned int cos_max;
};

/* Encapsulate feature specific HW info here. */
struct feat_hw_info {
    union {
        struct psr_cat_hw_info l3_cat_info;
    };
};

struct feat_node;

/*
 * This structure defines feature operation callback functions. Every feature
 * enabled MUST implement such callback functions and register them to ops.
 *
 * Feature specific behaviors will be encapsulated into these callback
 * functions. Then, the main flows will not be changed when introducing a new
 * feature.
 */
struct feat_ops {
    /* get_cos_max is used to get feature's cos_max. */
    unsigned int (*get_cos_max)(const struct feat_node *feat);
};

/*
 * This structure represents one feature.
 * feature     - Which feature it is.
 * feat_ops    - Feature operation callback functions.
 * info        - Feature HW info.
 * cos_reg_val - Array to store the values of COS registers. One entry stores
 *               the value of one COS register.
 *               For L3 CAT and L2 CAT, one entry corresponds to one COS_ID.
 *               For CDP, two entries correspond to one COS_ID. E.g.
 *               COS_ID=0 corresponds to cos_reg_val[0] (Data) and
 *               cos_reg_val[1] (Code).
 * list        - Feature list.
 */
struct feat_node {
    enum psr_feat_type feature;
    struct feat_ops ops;
    struct feat_hw_info info;
    uint64_t cos_reg_val[MAX_COS_REG_CNT];
    struct list_head list;
};

struct psr_assoc {
    uint64_t val;
    uint64_t cos_mask;
};

struct psr_cmt *__read_mostly psr_cmt;

static unsigned int opt_psr;
static unsigned int __initdata opt_rmid_max = 255;
static uint64_t rmid_mask;
static DEFINE_PER_CPU(struct psr_assoc, psr_assoc);

static void __init parse_psr_bool(char *s, char *value, char *feature,
                                  unsigned int mask)
{
    if ( !strcmp(s, feature) )
    {
        if ( !value )
            opt_psr |= mask;
        else
        {
            int val_int = parse_bool(value);

            if ( val_int == 0 )
                opt_psr &= ~mask;
            else if ( val_int == 1 )
                opt_psr |= mask;
        }
    }
}

static void __init parse_psr_param(char *s)
{
    char *ss, *val_str;

    do {
        ss = strchr(s, ',');
        if ( ss )
            *ss = '\0';

        val_str = strchr(s, ':');
        if ( val_str )
            *val_str++ = '\0';

        parse_psr_bool(s, val_str, "cmt", PSR_CMT);
        parse_psr_bool(s, val_str, "cat", PSR_CAT);
        parse_psr_bool(s, val_str, "cdp", PSR_CDP);

        if ( val_str && !strcmp(s, "rmid_max") )
            opt_rmid_max = simple_strtoul(val_str, NULL, 0);

        s = ss + 1;
    } while ( ss );
}
custom_param("psr", parse_psr_param);

static void __init init_psr_cmt(unsigned int rmid_max)
{
    unsigned int eax, ebx, ecx, edx;
    unsigned int rmid;

    if ( !boot_cpu_has(X86_FEATURE_PQM) )
        return;

    cpuid_count(0xf, 0, &eax, &ebx, &ecx, &edx);
    if ( !edx )
        return;

    psr_cmt = xzalloc(struct psr_cmt);
    if ( !psr_cmt )
        return;

    psr_cmt->features = edx;
    psr_cmt->rmid_max = min(rmid_max, ebx);
    rmid_mask = ~(~0ull << get_count_order(ebx));

    if ( psr_cmt->features & PSR_RESOURCE_TYPE_L3 )
    {
        cpuid_count(0xf, 1, &eax, &ebx, &ecx, &edx);
        psr_cmt->l3.upscaling_factor = ebx;
        psr_cmt->l3.rmid_max = ecx;
        psr_cmt->l3.features = edx;
    }

    psr_cmt->rmid_max = min(psr_cmt->rmid_max, psr_cmt->l3.rmid_max);
    psr_cmt->rmid_to_dom = xmalloc_array(domid_t, psr_cmt->rmid_max + 1UL);
    if ( !psr_cmt->rmid_to_dom )
    {
        xfree(psr_cmt);
        psr_cmt = NULL;
        return;
    }

    /*
     * Once CMT is enabled each CPU will always require a RMID to associate
     * with it. To reduce the waste of RMID, reserve RMID 0 for all CPUs that
     * have no domain being monitored.
     */
    psr_cmt->rmid_to_dom[0] = DOMID_XEN;
    for ( rmid = 1; rmid <= psr_cmt->rmid_max; rmid++ )
        psr_cmt->rmid_to_dom[rmid] = DOMID_INVALID;

    printk(XENLOG_INFO "Cache Monitoring Technology enabled\n");
}

/* Called with domain lock held, no psr specific lock needed */
int psr_alloc_rmid(struct domain *d)
{
    unsigned int rmid;

    ASSERT(psr_cmt_enabled());

    if ( d->arch.psr_rmid > 0 )
        return -EEXIST;

    for ( rmid = 1; rmid <= psr_cmt->rmid_max; rmid++ )
    {
        if ( psr_cmt->rmid_to_dom[rmid] != DOMID_INVALID )
            continue;

        psr_cmt->rmid_to_dom[rmid] = d->domain_id;
        break;
    }

    /* No RMID available, assign RMID=0 by default. */
    if ( rmid > psr_cmt->rmid_max )
    {
        d->arch.psr_rmid = 0;
        return -EOVERFLOW;
    }

    d->arch.psr_rmid = rmid;

    return 0;
}

/* Called with domain lock held, no psr specific lock needed */
void psr_free_rmid(struct domain *d)
{
    unsigned int rmid;

    rmid = d->arch.psr_rmid;
    /* We do not free system reserved "RMID=0". */
    if ( rmid == 0 )
        return;

    psr_cmt->rmid_to_dom[rmid] = DOMID_INVALID;
    d->arch.psr_rmid = 0;
}

static inline void psr_assoc_init(void)
{
    struct psr_assoc *psra = &this_cpu(psr_assoc);

    if ( psr_cmt_enabled() )
        rdmsrl(MSR_IA32_PSR_ASSOC, psra->val);
}

static inline void psr_assoc_rmid(uint64_t *reg, unsigned int rmid)
{
    *reg = (*reg & ~rmid_mask) | (rmid & rmid_mask);
}

void psr_ctxt_switch_to(struct domain *d)
{
    struct psr_assoc *psra = &this_cpu(psr_assoc);
    uint64_t reg = psra->val;

    if ( psr_cmt_enabled() )
        psr_assoc_rmid(&reg, d->arch.psr_rmid);

    if ( reg != psra->val )
    {
        wrmsrl(MSR_IA32_PSR_ASSOC, reg);
        psra->val = reg;
    }
}

int psr_get_cat_l3_info(unsigned int socket, uint32_t *cbm_len,
                        uint32_t *cos_max, uint32_t *flags)
{
    return 0;
}

int psr_get_l3_cbm(struct domain *d, unsigned int socket,
                   uint64_t *cbm, enum cbm_type type)
{
    return 0;
}

int psr_set_l3_cbm(struct domain *d, unsigned int socket,
                   uint64_t cbm, enum cbm_type type)
{
    return 0;
}

int psr_domain_init(struct domain *d)
{
    return 0;
}

void psr_domain_free(struct domain *d)
{
    psr_free_rmid(d);
}

static int psr_cpu_prepare(unsigned int cpu)
{
    return 0;
}

static void psr_cpu_init(void)
{
    psr_assoc_init();
}

static void psr_cpu_fini(unsigned int cpu)
{
    return;
}

static int cpu_callback(
    struct notifier_block *nfb, unsigned long action, void *hcpu)
{
    int rc = 0;
    unsigned int cpu = (unsigned long)hcpu;

    switch ( action )
    {
    case CPU_UP_PREPARE:
        rc = psr_cpu_prepare(cpu);
        break;
    case CPU_STARTING:
        psr_cpu_init();
        break;
    case CPU_UP_CANCELED:
    case CPU_DEAD:
        psr_cpu_fini(cpu);
        break;
    }

    return !rc ? NOTIFY_DONE : notifier_from_errno(rc);
}

static struct notifier_block cpu_nfb = {
    .notifier_call = cpu_callback,
    /*
     * Ensure socket_cpumask is still valid in CPU_DEAD notification
     * (E.g. our CPU_DEAD notification should be called ahead of
     * cpu_smpboot_free).
     */
    .priority = -1
};

static int __init psr_presmp_init(void)
{
    if ( (opt_psr & PSR_CMT) && opt_rmid_max )
        init_psr_cmt(opt_rmid_max);

    psr_cpu_prepare(0);

    psr_cpu_init();
    if ( psr_cmt_enabled() )
        register_cpu_notifier(&cpu_nfb);

    return 0;
}
presmp_initcall(psr_presmp_init);

/*
 * Local variables:
 * mode: C
 * c-file-style: "BSD"
 * c-basic-offset: 4
 * tab-width: 4
 * indent-tabs-mode: nil
 * End:
 */
