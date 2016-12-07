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
#include <asm/x86_emulate.h>

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

#define CAT_CBM_LEN_MASK 0x1f
#define CAT_COS_MAX_MASK 0xffff

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

#define PSR_ASSOC_REG_SHIFT 32

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
    PSR_SOCKET_UNKNOWN = 0xFFFF,
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
    /* get_feat_info is used to get feature HW info. */
    bool (*get_feat_info)(const struct feat_node *feat,
                          uint32_t data[], unsigned int array_len);
    /* get_val is used to get feature COS register value. */
    bool (*get_val)(const struct feat_node *feat, unsigned int cos,
                    enum cbm_type type, uint64_t *val);
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

static struct psr_socket_info *__read_mostly socket_info;

static unsigned int opt_psr;
static unsigned int __initdata opt_rmid_max = 255;
static unsigned int __read_mostly opt_cos_max = MAX_COS_REG_CNT;
static uint64_t rmid_mask;
static DEFINE_PER_CPU(struct psr_assoc, psr_assoc);

/*
 * Declare global feature list entry for every feature to facilitate the
 * feature list creation. It will be allocated in psr_cpu_prepare() and
 * inserted into feature list in cpu_init_work(). It is protected by
 * cpu_add_remove_lock spinlock.
 */
static struct feat_node *feat_l3_cat;

/* Common functions. */
static void free_feature(struct psr_socket_info *info)
{
    struct feat_node *feat, *next;

    if ( !info )
        return;

    /*
     * Free resources of features. But we do not free global feature list
     * entry, like feat_l3_cat. Although it may cause a few memory leak,
     * it is OK simplify things.
     */
    list_for_each_entry_safe(feat, next, &info->feat_list, list)
    {
        __clear_bit(feat->feature, &info->feat_mask);
        list_del(&feat->list);
        xfree(feat);
    }
}

static enum psr_feat_type psr_cbm_type_to_feat_type(enum cbm_type type)
{
    enum psr_feat_type feat_type;

    /* Judge if feature is enabled. */
    switch ( type )
    {
    case PSR_CBM_TYPE_L3:
        feat_type = PSR_SOCKET_L3_CAT;
        break;
    default:
        feat_type = PSR_SOCKET_UNKNOWN;
        break;
    }

    return feat_type;
}

/* L3 CAT functions implementation. */
static void l3_cat_init_feature(struct cpuid_leaf regs,
                                struct feat_node *feat,
                                struct psr_socket_info *info)
{
    struct psr_cat_hw_info l3_cat;
    unsigned int socket;

    /* No valid value so do not enable feature. */
    if ( !regs.a || !regs.b )
        return;

    l3_cat.cbm_len = (regs.a & CAT_CBM_LEN_MASK) + 1;
    l3_cat.cos_max = min(opt_cos_max, regs.d & CAT_COS_MAX_MASK);

    /* cos=0 is reserved as default cbm(all bits within cbm_len are 1). */
    feat->cos_reg_val[0] = (1ull << l3_cat.cbm_len) - 1;

    feat->feature = PSR_SOCKET_L3_CAT;
    ASSERT(!test_bit(PSR_SOCKET_L3_CAT, &info->feat_mask));
    __set_bit(PSR_SOCKET_L3_CAT, &info->feat_mask);

    feat->info.l3_cat_info = l3_cat;

    info->nr_feat++;

    /* Add this feature into list. */
    list_add_tail(&feat->list, &info->feat_list);

    socket = cpu_to_socket(smp_processor_id());
    if ( !opt_cpu_info )
        return;

    printk(XENLOG_INFO "L3 CAT: enabled on socket %u, cos_max:%u, cbm_len:%u\n",
           socket, feat->info.l3_cat_info.cos_max,
           feat->info.l3_cat_info.cbm_len);
}

static unsigned int l3_cat_get_cos_max(const struct feat_node *feat)
{
    return feat->info.l3_cat_info.cos_max;
}

static bool l3_cat_get_feat_info(const struct feat_node *feat,
                                 uint32_t data[], unsigned int array_len)
{
    if ( !data || 3 > array_len )
        return false;

    data[CBM_LEN] = feat->info.l3_cat_info.cbm_len;
    data[COS_MAX] = feat->info.l3_cat_info.cos_max;
    data[PSR_FLAG] = 0;

    return true;
}

static bool l3_cat_get_val(const struct feat_node *feat, unsigned int cos,
                           enum cbm_type type, uint64_t *val)
{
    if ( cos > feat->info.l3_cat_info.cos_max )
        /* Use default value. */
        cos = 0;

    *val = feat->cos_reg_val[cos];

    return true;
}

static const struct feat_ops l3_cat_ops = {
    .get_cos_max = l3_cat_get_cos_max,
    .get_feat_info = l3_cat_get_feat_info,
    .get_val = l3_cat_get_val,
};

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

        if ( val_str && !strcmp(s, "cos_max") )
            opt_cos_max = simple_strtoul(val_str, NULL, 0);

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

static inline unsigned int get_max_cos_max(const struct psr_socket_info *info)
{
    const struct feat_node *feat;
    unsigned int cos_max = 0;

    list_for_each_entry(feat, &info->feat_list, list)
        cos_max = max(feat->ops.get_cos_max(feat), cos_max);

    return cos_max;
}

static inline void psr_assoc_init(void)
{
    struct psr_assoc *psra = &this_cpu(psr_assoc);

    if ( socket_info )
    {
        unsigned int socket = cpu_to_socket(smp_processor_id());
        const struct psr_socket_info *info = socket_info + socket;
        unsigned int cos_max = get_max_cos_max(info);

        if ( info->feat_mask )
            psra->cos_mask = ((1ull << get_count_order(cos_max)) - 1) <<
                              PSR_ASSOC_REG_SHIFT;
    }

    if ( psr_cmt_enabled() || psra->cos_mask )
        rdmsrl(MSR_IA32_PSR_ASSOC, psra->val);
}

static inline void psr_assoc_rmid(uint64_t *reg, unsigned int rmid)
{
    *reg = (*reg & ~rmid_mask) | (rmid & rmid_mask);
}

static inline void psr_assoc_cos(uint64_t *reg, unsigned int cos,
                                 uint64_t cos_mask)
{
    *reg = (*reg & ~cos_mask) |
            (((uint64_t)cos << PSR_ASSOC_REG_SHIFT) & cos_mask);
}

void psr_ctxt_switch_to(struct domain *d)
{
    struct psr_assoc *psra = &this_cpu(psr_assoc);
    uint64_t reg = psra->val;

    if ( psr_cmt_enabled() )
        psr_assoc_rmid(&reg, d->arch.psr_rmid);

    if ( psra->cos_mask )
        psr_assoc_cos(&reg, d->arch.psr_cos_ids ?
                      d->arch.psr_cos_ids[cpu_to_socket(smp_processor_id())] :
                      0, psra->cos_mask);

    if ( reg != psra->val )
    {
        wrmsrl(MSR_IA32_PSR_ASSOC, reg);
        psra->val = reg;
    }
}

static struct psr_socket_info *get_socket_info(unsigned int socket)
{
    if ( !socket_info )
        return ERR_PTR(-ENODEV);

    if ( socket >= nr_sockets )
        return ERR_PTR(-ERANGE);

    if ( !socket_info[socket].feat_mask )
        return ERR_PTR(-ENOENT);

    return socket_info + socket;
}

static int __psr_get(unsigned int socket, enum cbm_type type,
                     uint32_t data[], unsigned int array_len,
                     struct domain *d, uint64_t *val)
{
    const struct psr_socket_info *info = get_socket_info(socket);
    const struct feat_node *feat;
    enum psr_feat_type feat_type;
    unsigned int cos;

    if ( IS_ERR(info) )
        return PTR_ERR(info);

    feat_type = psr_cbm_type_to_feat_type(type);
    list_for_each_entry(feat, &info->feat_list, list)
    {
        if ( feat->feature != feat_type )
            continue;

        if ( d )
        {
            cos = d->arch.psr_cos_ids[socket];
            if ( feat->ops.get_val(feat, cos, type, val) )
                return 0;
            else
                break;
        }

        if ( feat->ops.get_feat_info(feat, data, array_len) )
            return 0;
        else
            return -EINVAL;
    }

    return -ENOENT;
}

int psr_get_info(unsigned int socket, enum cbm_type type,
                 uint32_t data[], unsigned int array_len)
{
    return __psr_get(socket, type, data, array_len, NULL, NULL);
}

int psr_get_val(struct domain *d, unsigned int socket,
                uint64_t *val, enum cbm_type type)
{
    return __psr_get(socket, type, NULL, 0, d, val);
}

/* Set value functions */
static unsigned int get_cos_num(const struct psr_socket_info *info)
{
    return 0;
}

static int assemble_val_array(uint64_t *val,
                              uint32_t array_len,
                              const struct psr_socket_info *info,
                              unsigned int old_cos)
{
    return -EINVAL;
}

static int set_new_val_to_array(uint64_t *val,
                                uint32_t array_len,
                                const struct psr_socket_info *info,
                                enum psr_feat_type feat_type,
                                enum cbm_type type,
                                uint64_t m)
{
    return -EINVAL;
}

static int find_cos(const uint64_t *val, uint32_t array_len,
                    enum psr_feat_type feat_type,
                    const struct psr_socket_info *info)
{
    return -ENOENT;
}

static int pick_avail_cos(const struct psr_socket_info *info,
                          const uint64_t *val, uint32_t array_len,
                          unsigned int old_cos,
                          enum psr_feat_type feat_type)
{
    return -ENOENT;
}

static int write_psr_msr(unsigned int socket, unsigned int cos,
                         const uint64_t *val)
{
    return -ENOENT;
}

int psr_set_val(struct domain *d, unsigned int socket,
                uint64_t val, enum cbm_type type)
{
    unsigned int old_cos;
    int cos, ret;
    unsigned int *ref;
    uint64_t *val_array;
    struct psr_socket_info *info = get_socket_info(socket);
    uint32_t array_len;
    enum psr_feat_type feat_type;

    if ( IS_ERR(info) )
        return PTR_ERR(info);

    feat_type = psr_cbm_type_to_feat_type(type);
    if ( !test_bit(feat_type, &info->feat_mask) )
        return -ENOENT;

    /*
     * Step 0:
     * old_cos means the COS ID current domain is using. By default, it is 0.
     *
     * For every COS ID, there is a reference count to record how many domains
     * are using the COS register corresponding to this COS ID.
     * - If ref[old_cos] is 0, that means this COS is not used by any domain.
     * - If ref[old_cos] is 1, that means this COS is only used by current
     *   domain.
     * - If ref[old_cos] is more than 1, that mean multiple domains are using
     *   this COS.
     */
    old_cos = d->arch.psr_cos_ids[socket];
    if ( old_cos > MAX_COS_REG_CNT )
        return -EOVERFLOW;

    ref = info->cos_ref;

    /*
     * Step 1:
     * Assemle a value array to store all featues cos_reg_val[old_cos].
     * And, set the input val into array according to the feature's
     * position in array.
     */
    array_len = get_cos_num(info);
    val_array = xzalloc_array(uint64_t, array_len);
    if ( !val_array )
        return -ENOMEM;

    if ( (ret = assemble_val_array(val_array, array_len, info, old_cos)) != 0 )
    {
        xfree(val_array);
        return ret;
    }

    if ( (ret = set_new_val_to_array(val_array, array_len, info,
                                     feat_type, type, val)) != 0 )
    {
        xfree(val_array);
        return ret;
    }

    /*
     * Lock here to make sure the ref is not changed during find and
     * write process.
     */
    spin_lock(&info->ref_lock);

    /*
     * Step 2:
     * Try to find if there is already a COS ID on which all features' values
     * are same as the array. Then, we can reuse this COS ID.
     */
    cos = find_cos(val_array, array_len, feat_type, info);
    if ( cos >= 0 )
    {
        if ( cos == old_cos )
        {
            spin_unlock(&info->ref_lock);
            xfree(val_array);
            return 0;
        }
    }
    else
    {
        /*
         * Step 3:
         * If fail to find, we need allocate a new COS ID.
         * If multiple domains are using same COS ID, its ref is more
         * than 1. That means we cannot free this COS to make current domain
         * use it. Because other domains are using the value saved in the COS.
         * Unless the ref is changed to 1 (mean only current domain is using
         * it), we cannot allocate the COS ID to current domain.
         * So, only the COS ID which ref is 1 or 0 can be allocated.
         */
        cos = pick_avail_cos(info, val_array, array_len, old_cos, feat_type);
        if ( cos < 0 )
        {
            spin_unlock(&info->ref_lock);
            xfree(val_array);
            return cos;
        }

        /*
         * Step 4:
         * Write all features MSRs according to the COS ID.
         */
        ret = write_psr_msr(socket, cos, val_array);
        if ( ret )
        {
            spin_unlock(&info->ref_lock);
            xfree(val_array);
            return ret;
        }
    }

    /*
     * Step 5:
     * Update ref according to COS ID.
     */
    ref[cos]++;
    ASSERT(ref[cos] || cos == 0);
    ref[old_cos]--;
    spin_unlock(&info->ref_lock);

    /*
     * Step 6:
     * Save the COS ID into current domain's psr_cos_ids[] so that we can know
     * which COS the domain is using on the socket. One domain can only use
     * one COS ID at same time on each socket.
     */
    d->arch.psr_cos_ids[socket] = cos;
    xfree(val_array);

    return 0;
}

/* Called with domain lock held, no extra lock needed for 'psr_cos_ids' */
static void psr_free_cos(struct domain *d)
{
    unsigned int socket, cos;

    if ( !d->arch.psr_cos_ids )
        return;

    /* Domain is free so its cos_ref should be decreased. */
    for ( socket = 0; socket < nr_sockets; socket++ )
    {
        struct psr_socket_info *info;

        /* cos 0 is default one which does not need be handled. */
        if ( (cos = d->arch.psr_cos_ids[socket]) == 0 )
            continue;

        /*
         * If domain uses other cos ids, all corresponding refs must have been
         * increased 1 for this domain. So, we need decrease them.
         */
        info = socket_info + socket;
        ASSERT(info->cos_ref[cos] || cos == 0);
        spin_lock(&info->ref_lock);
        info->cos_ref[cos]--;
        spin_unlock(&info->ref_lock);
    }

    xfree(d->arch.psr_cos_ids);
    d->arch.psr_cos_ids = NULL;
}

int psr_domain_init(struct domain *d)
{
    if ( socket_info )
    {
        d->arch.psr_cos_ids = xzalloc_array(unsigned int, nr_sockets);
        if ( !d->arch.psr_cos_ids )
            return -ENOMEM;
    }

    return 0;
}

void psr_domain_free(struct domain *d)
{
    psr_free_rmid(d);
    psr_free_cos(d);
}

static void cpu_init_work(void)
{
    struct psr_socket_info *info;
    unsigned int socket;
    unsigned int cpu = smp_processor_id();
    struct feat_node *feat;
    struct cpuid_leaf regs = {.a = 0, .b = 0, .c = 0, .d = 0};

    if ( !cpu_has(&current_cpu_data, X86_FEATURE_PQE) )
        return;
    else if ( current_cpu_data.cpuid_level < PSR_CPUID_LEVEL_CAT )
    {
        __clear_bit(X86_FEATURE_PQE, current_cpu_data.x86_capability);
        return;
    }

    socket = cpu_to_socket(cpu);
    info = socket_info + socket;
    if ( info->feat_mask )
        return;

    INIT_LIST_HEAD(&info->feat_list);
    spin_lock_init(&info->ref_lock);

    cpuid_count_leaf(PSR_CPUID_LEVEL_CAT, 0, &regs);
    if ( regs.b & PSR_RESOURCE_TYPE_L3 )
    {
        cpuid_count_leaf(PSR_CPUID_LEVEL_CAT, 1, &regs);

        feat = feat_l3_cat;
        /* psr_cpu_prepare will allocate it on subsequent CPU onlining. */
        feat_l3_cat = NULL;
        feat->ops = l3_cat_ops;

        l3_cat_init_feature(regs, feat, info);
    }
}

static void cpu_fini_work(unsigned int cpu)
{
    unsigned int socket = cpu_to_socket(cpu);

    if ( !socket_cpumask[socket] || cpumask_empty(socket_cpumask[socket]) )
    {
        free_feature(socket_info + socket);
    }
}

static void __init init_psr(void)
{
    if ( opt_cos_max < 1 )
    {
        printk(XENLOG_INFO "CAT: disabled, cos_max is too small\n");
        return;
    }

    socket_info = xzalloc_array(struct psr_socket_info, nr_sockets);

    if ( !socket_info )
    {
        printk(XENLOG_INFO "Failed to alloc socket_info!\n");
        return;
    }
}

static void __init psr_free(void)
{
    unsigned int i;

    for ( i = 0; i < nr_sockets; i++ )
        free_feature(&socket_info[i]);

    xfree(socket_info);
    socket_info = NULL;
}

static int psr_cpu_prepare(unsigned int cpu)
{
    if ( !socket_info )
        return 0;

    /* Malloc memory for the global feature head here. */
    if ( feat_l3_cat == NULL &&
         (feat_l3_cat = xzalloc(struct feat_node)) == NULL )
        return -ENOMEM;

    return 0;
}

static void psr_cpu_init(void)
{
    if ( socket_info )
        cpu_init_work();

    psr_assoc_init();
}

static void psr_cpu_fini(unsigned int cpu)
{
    if ( socket_info )
        cpu_fini_work(cpu);
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

    if ( opt_psr & PSR_CAT )
        init_psr();

    if ( psr_cpu_prepare(0) )
        psr_free();

    psr_cpu_init();
    if ( psr_cmt_enabled() || socket_info )
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
