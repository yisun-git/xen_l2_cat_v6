#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <limits.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <unistd.h>
#include <xen/xen.h>

#include "x86_emulate.h"

#define MSR_INDEX_MAX 16

#define SEG_NUM x86_seg_none

struct input_struct {
    unsigned long cr[5];
    uint64_t msr[MSR_INDEX_MAX];
    struct cpu_user_regs regs;
    struct segment_register segments[SEG_NUM];
    unsigned long options;
    unsigned char data[4096];
} input;
#define DATA_OFFSET offsetof(struct input_struct, data)
static unsigned int data_index;
static unsigned int data_num;

/*
 * Randomly return success or failure when processing data.  If
 * `exception` is false, this function turns _EXCEPTION to _OKAY.
 */
static int maybe_fail(const char *why, bool exception)
{
    int rc;

    if ( data_index >= data_num )
        rc = X86EMUL_EXCEPTION;
    else
    {
        /* Randomly returns value:
         * 50% okay
         * 25% unhandlable
         * 25% exception
         */
        if ( input.data[data_index] > 0xc0 )
            rc = X86EMUL_EXCEPTION;
        else if ( input.data[data_index] > 0x80 )
            rc = X86EMUL_UNHANDLEABLE;
        else
            rc = X86EMUL_OKAY;
        data_index++;
    }

    if ( rc == X86EMUL_EXCEPTION && !exception )
        rc = X86EMUL_OKAY;

    printf("maybe_fail %s: %d\n", why, rc);

    return rc;
}

static int data_read(const char *why, void *dst, unsigned int bytes)
{
    unsigned int i;
    int rc;

    if ( data_index + bytes > data_num )
        rc = X86EMUL_EXCEPTION;
    else
        rc = maybe_fail(why, true);

    if ( rc == X86EMUL_OKAY )
    {
        memcpy(dst,  input.data + data_index, bytes);
        data_index += bytes;

        printf("%s: ", why);
        for ( i = 0; i < bytes; i++ )
            printf(" %02x", *(unsigned char *)(dst + i));
        printf("\n");
    }

    return rc;
}

static int fuzz_read(
    unsigned int seg,
    unsigned long offset,
    void *p_data,
    unsigned int bytes,
    struct x86_emulate_ctxt *ctxt)
{
    return data_read("read", p_data, bytes);
}

static int fuzz_read_io(
    unsigned int port,
    unsigned int bytes,
    unsigned long *val,
    struct x86_emulate_ctxt *ctxt)
{
    return data_read("read_io", val, bytes);
}

static int fuzz_insn_fetch(
    unsigned int seg,
    unsigned long offset,
    void *p_data,
    unsigned int bytes,
    struct x86_emulate_ctxt *ctxt)
{
    return data_read("insn_fetch", p_data, bytes);
}

static int _fuzz_rep_read(const char *why, unsigned long *reps)
{
    int rc;
    unsigned long bytes_read = 0;

    rc = data_read(why, &bytes_read, sizeof(bytes_read));

    if ( bytes_read <= *reps )
        *reps = bytes_read;

    switch ( rc )
    {
    case X86EMUL_UNHANDLEABLE:
        /* No work is done in this case */
        *reps = 0;
        break;
    case X86EMUL_EXCEPTION:
    case X86EMUL_RETRY:
        /* Halve the amount in this case */
        *reps /= 2;
        break;
    }

    return rc;
}

static int _fuzz_rep_write(const char *why, unsigned long *reps)
{
    int rc = maybe_fail(why, true);

    switch ( rc )
    {
    case X86EMUL_UNHANDLEABLE:
        /* No work is done in this case */
        *reps = 0;
        break;
    case X86EMUL_EXCEPTION:
    case X86EMUL_RETRY:
        /* Halve the amount in this case */
        *reps /= 2;
        break;
    }

    return rc;
}

static int fuzz_rep_ins(
    uint16_t src_port,
    enum x86_segment dst_seg,
    unsigned long dst_offset,
    unsigned int bytes_per_rep,
    unsigned long *reps,
    struct x86_emulate_ctxt *ctxt)
{
    return _fuzz_rep_read("rep_ins", reps);
}

static int fuzz_rep_movs(
    enum x86_segment src_seg,
    unsigned long src_offset,
    enum x86_segment dst_seg,
    unsigned long dst_offset,
    unsigned int bytes_per_rep,
    unsigned long *reps,
    struct x86_emulate_ctxt *ctxt)
{
    return _fuzz_rep_read("rep_movs", reps);
}

static int fuzz_rep_outs(
    enum x86_segment src_seg,
    unsigned long src_offset,
    uint16_t dst_port,
    unsigned int bytes_per_rep,
    unsigned long *reps,
    struct x86_emulate_ctxt *ctxt)
{
    return _fuzz_rep_write("rep_outs", reps);
}

static int fuzz_rep_stos(
    void *p_data,
    enum x86_segment seg,
    unsigned long offset,
    unsigned int bytes_per_rep,
    unsigned long *reps,
    struct x86_emulate_ctxt *ctxt)
{
    return _fuzz_rep_write("rep_stos", reps);
}

static int fuzz_write(
    unsigned int seg,
    unsigned long offset,
    void *p_data,
    unsigned int bytes,
    struct x86_emulate_ctxt *ctxt)
{
    return maybe_fail("write", true);
}

static int fuzz_cmpxchg(
    unsigned int seg,
    unsigned long offset,
    void *old,
    void *new,
    unsigned int bytes,
    struct x86_emulate_ctxt *ctxt)
{
    return maybe_fail("cmpxchg", true);
}

static int fuzz_invlpg(
    enum x86_segment seg,
    unsigned long offset,
    struct x86_emulate_ctxt *ctxt)
{
    return maybe_fail("invlpg", false);
}

static int fuzz_wbinvd(
    struct x86_emulate_ctxt *ctxt)
{
    return maybe_fail("wbinvd", true);
}

static int fuzz_write_io(
    unsigned int port,
    unsigned int bytes,
    unsigned long val,
    struct x86_emulate_ctxt *ctxt)
{
    return maybe_fail("write_io", true);
}

static int fuzz_read_segment(
    enum x86_segment seg,
    struct segment_register *reg,
    struct x86_emulate_ctxt *ctxt)
{
    if ( seg >= SEG_NUM )
        return X86EMUL_UNHANDLEABLE;

    *reg = input.segments[seg];

    return X86EMUL_OKAY;
}

static int fuzz_write_segment(
    enum x86_segment seg,
    const struct segment_register *reg,
    struct x86_emulate_ctxt *ctxt)
{
    int rc;

    if ( seg >= SEG_NUM )
        return X86EMUL_UNHANDLEABLE;

    rc = maybe_fail("write_segment", true);

    if ( rc == X86EMUL_OKAY )
        input.segments[seg] = *reg;

    return rc;
}

static int fuzz_read_cr(
    unsigned int reg,
    unsigned long *val,
    struct x86_emulate_ctxt *ctxt)
{
    if ( reg >= ARRAY_SIZE(input.cr) )
        return X86EMUL_UNHANDLEABLE;

    *val = input.cr[reg];

    return X86EMUL_OKAY;
}

static int fuzz_write_cr(
    unsigned int reg,
    unsigned long val,
    struct x86_emulate_ctxt *ctxt)
{
    int rc;

    if ( reg >= ARRAY_SIZE(input.cr) )
        return X86EMUL_UNHANDLEABLE;

    rc = maybe_fail("write_cr", true);
    if ( rc != X86EMUL_OKAY )
        return rc;

    input.cr[reg] = val;

    return X86EMUL_OKAY;
}

enum {
    MSRI_IA32_SYSENTER_CS,
    MSRI_IA32_SYSENTER_ESP,
    MSRI_IA32_SYSENTER_EIP,
    MSRI_EFER,
    MSRI_STAR,
    MSRI_LSTAR,
    MSRI_CSTAR,
    MSRI_SYSCALL_MASK
};

static const unsigned int msr_index[MSR_INDEX_MAX] = {
    [MSRI_IA32_SYSENTER_CS]  = MSR_IA32_SYSENTER_CS,
    [MSRI_IA32_SYSENTER_ESP] = MSR_IA32_SYSENTER_ESP,
    [MSRI_IA32_SYSENTER_EIP] = MSR_IA32_SYSENTER_EIP,
    [MSRI_EFER]              = MSR_EFER,
    [MSRI_STAR]              = MSR_STAR,
    [MSRI_LSTAR]             = MSR_LSTAR,
    [MSRI_CSTAR]             = MSR_CSTAR,
    [MSRI_SYSCALL_MASK]      = MSR_SYSCALL_MASK
};

static int fuzz_read_msr(
    unsigned int reg,
    uint64_t *val,
    struct x86_emulate_ctxt *ctxt)
{
    unsigned int idx;

    if ( reg >= MSR_INDEX_MAX )
        return X86EMUL_UNHANDLEABLE;

    switch ( reg )
    {
    case MSR_TSC_AUX:
    case MSR_IA32_TSC:
        /*
         * TSC should return monotonically increasing values, TSC_AUX
         * should preferably return consistent values, but returning
         * random values is fine in fuzzer.
         */
        return data_read("read_msr", val, sizeof(*val));
    case MSR_EFER:
        *val = input.msr[MSRI_EFER];
        *val &= ~EFER_LMA;
        if ( (*val & EFER_LME) && (input.cr[4] & X86_CR4_PAE) &&
             (input.cr[0] & X86_CR0_PG) )
        {
            printf("Setting EFER_LMA\n");
            *val |= EFER_LMA;
        }
        return X86EMUL_OKAY;
    }

    for ( idx = 0; idx < MSR_INDEX_MAX; idx++ )
    {
        if ( msr_index[idx] == reg )
        {
            *val = input.msr[idx];
            return X86EMUL_OKAY;
        }
    }

    return X86EMUL_EXCEPTION;
}

static int fuzz_write_msr(
    unsigned int reg,
    uint64_t val,
    struct x86_emulate_ctxt *ctxt)
{
    unsigned int idx;
    int rc;

    rc = maybe_fail("write_msr", true);
    if ( rc != X86EMUL_OKAY )
        return rc;

    switch ( reg )
    {
    case MSR_TSC_AUX:
    case MSR_IA32_TSC:
        return X86EMUL_OKAY;
    }

    for ( idx = 0; idx < MSR_INDEX_MAX; idx++ )
    {
        if ( msr_index[idx] == reg )
        {
            input.msr[idx] = val;
            return X86EMUL_OKAY;
        }
    }

    return X86EMUL_EXCEPTION;
}

#define SET(h) .h = fuzz_##h
static struct x86_emulate_ops fuzz_emulops = {
    SET(read),
    SET(insn_fetch),
    SET(write),
    SET(cmpxchg),
    SET(rep_ins),
    SET(rep_outs),
    SET(rep_movs),
    SET(rep_stos),
    SET(read_segment),
    SET(write_segment),
    SET(read_io),
    SET(write_io),
    SET(read_cr),
    SET(write_cr),
    SET(read_msr),
    SET(write_msr),
    SET(wbinvd),
    SET(invlpg),
    .get_fpu    = emul_test_get_fpu,
    .cpuid      = emul_test_cpuid,
};
#undef SET

static void setup_fpu_exception_handler(void)
{
    /* FIXME - just disable exceptions for now */
    unsigned long a;

    asm volatile ( "fnclex");
    a = 0x37f; /* FCW_DEFAULT in Xen */
    asm volatile ( "fldcw %0" :: "m" (a));
    a = 0x1f80; /* MXCSR_DEFAULT in Xen */
    asm volatile ( "ldmxcsr %0" :: "m" (a) );
}

static void dump_state(struct x86_emulate_ctxt *ctxt)
{
    struct cpu_user_regs *regs = ctxt->regs;
    uint64_t val = 0;

    printf(" -- State -- \n");
    printf("addr / sp size: %d / %d\n", ctxt->addr_size, ctxt->sp_size);
    printf(" cr0: %lx\n", input.cr[0]);
    printf(" cr3: %lx\n", input.cr[3]);
    printf(" cr4: %lx\n", input.cr[4]);

    printf(" rip: %"PRIx64"\n", regs->rip);

    fuzz_read_msr(MSR_EFER, &val, ctxt);
    printf("EFER: %"PRIx64"\n", val);
}

static bool long_mode_active(struct x86_emulate_ctxt *ctxt)
{
    uint64_t val;

    if ( fuzz_read_msr(MSR_EFER, &val, ctxt) != X86EMUL_OKAY )
        return false;

    return val & EFER_LMA;
}

static bool in_longmode(struct x86_emulate_ctxt *ctxt)
{
    return long_mode_active(ctxt) && input.segments[x86_seg_cs].attr.fields.l;
}

static void set_sizes(struct x86_emulate_ctxt *ctxt)
{
    if ( in_longmode(ctxt) )
        ctxt->addr_size = ctxt->sp_size = 64;
    else
    {
        ctxt->addr_size = input.segments[x86_seg_cs].attr.fields.db ? 32 : 16;
        ctxt->sp_size   = input.segments[x86_seg_ss].attr.fields.db ? 32 : 16;
    }
}

#define CANONICALIZE(x)                                   \
    do {                                                  \
        uint64_t _y = (x);                                \
        if ( _y & (1ULL << 47) )                          \
            _y |= (~0ULL) << 48;                          \
        else                                              \
            _y &= (1ULL << 48)-1;                         \
        printf("Canonicalized %" PRIx64 " to %" PRIx64 "\n", x, _y);    \
        (x) = _y;                                       \
    } while( 0 )

/* Expects bitmap and regs to be defined */
#define CANONICALIZE_MAYBE(reg)                       \
    if ( !(bitmap & (1 << CANONICALIZE_##reg)) )      \
        CANONICALIZE(regs->reg);                      \

enum {
    HOOK_read,
    HOOK_insn_fetch,
    HOOK_write,
    HOOK_cmpxchg,
    HOOK_rep_ins,
    HOOK_rep_outs,
    HOOK_rep_movs,
    HOOK_rep_stos,
    HOOK_read_segment,
    HOOK_write_segment,
    HOOK_read_io,
    HOOK_write_io,
    HOOK_read_cr,
    HOOK_write_cr,
    HOOK_read_dr,
    HOOK_write_dr,
    HOOK_read_msr,
    HOOK_write_msr,
    HOOK_wbinvd,
    HOOK_cpuid,
    HOOK_inject_hw_exception,
    HOOK_inject_sw_interrupt,
    HOOK_get_fpu,
    HOOK_put_fpu,
    HOOK_invlpg,
    HOOK_vmfunc,
    OPTION_swint_emulation, /* Two bits */
    CANONICALIZE_rip = OPTION_swint_emulation + 2,
    CANONICALIZE_rsp,
    CANONICALIZE_rbp
};

/* Expects bitmap to be defined */
#define MAYBE_DISABLE_HOOK(h)                          \
    if ( bitmap & (1 << HOOK_##h) )                    \
    {                                                  \
        fuzz_emulops.h = NULL;                         \
        printf("Disabling hook "#h"\n");               \
    }

static void disable_hooks(void)
{
    unsigned long bitmap = input.options;

    /* See also sanitize_input, some hooks can't be disabled. */
    MAYBE_DISABLE_HOOK(read);
    MAYBE_DISABLE_HOOK(insn_fetch);
    MAYBE_DISABLE_HOOK(write);
    MAYBE_DISABLE_HOOK(cmpxchg);
    MAYBE_DISABLE_HOOK(rep_ins);
    MAYBE_DISABLE_HOOK(rep_outs);
    MAYBE_DISABLE_HOOK(rep_movs);
    MAYBE_DISABLE_HOOK(rep_stos);
    MAYBE_DISABLE_HOOK(read_segment);
    MAYBE_DISABLE_HOOK(write_segment);
    MAYBE_DISABLE_HOOK(read_io);
    MAYBE_DISABLE_HOOK(write_io);
    MAYBE_DISABLE_HOOK(read_cr);
    MAYBE_DISABLE_HOOK(write_cr);
    MAYBE_DISABLE_HOOK(read_msr);
    MAYBE_DISABLE_HOOK(write_msr);
    MAYBE_DISABLE_HOOK(wbinvd);
    MAYBE_DISABLE_HOOK(cpuid);
    MAYBE_DISABLE_HOOK(get_fpu);
    MAYBE_DISABLE_HOOK(invlpg);
}

static void set_swint_support(struct x86_emulate_ctxt *ctxt)
{
    unsigned int swint_opt = (input.options >> OPTION_swint_emulation) & 3;
    static const enum x86_swint_emulation map[4] = {
        x86_swint_emulate_none,
        x86_swint_emulate_none,
        x86_swint_emulate_icebp,
        x86_swint_emulate_all
    };

    ctxt->swint_emulate = map[swint_opt];
}

/*
 * Constrain input to architecturally-possible states where
 * the emulator relies on these
 *
 * In general we want the emulator to be as absolutely robust as
 * possible; which means that we want to minimize the number of things
 * it assumes about the input state.  Tesing this means minimizing and
 * removing as much of the input constraints as possible.
 *
 * So we only add constraints that (in general) have been proven to
 * cause crashes in the emulator.
 *
 * For future reference: other constraints which might be necessary at
 * some point:
 *
 * - EFER.LMA => !EFLAGS.NT
 * - In VM86 mode, force segment...
 *  - ...access rights to 0xf3
 *  - ...limits to 0xffff
 *  - ...bases to below 1Mb, 16-byte aligned
 *  - ...selectors to (base >> 4)
 */
static void sanitize_input(struct x86_emulate_ctxt *ctxt)
{
    struct cpu_user_regs *regs = &input.regs;
    unsigned long bitmap = input.options;

    /* Some hooks can't be disabled. */
    input.options &= ~((1<<HOOK_read)|(1<<HOOK_insn_fetch));

    /* Zero 'private' entries */
    regs->error_code = 0;
    regs->entry_vector = 0;

    CANONICALIZE_MAYBE(rip);
    CANONICALIZE_MAYBE(rsp);
    CANONICALIZE_MAYBE(rbp);

    /*
     * CR0.PG can't be set if CR0.PE isn't set.  Set is more interesting, so
     * set PE if PG is set.
     */
    if ( input.cr[0] & X86_CR0_PG )
        input.cr[0] |= X86_CR0_PE;

    /* EFLAGS.VM not available in long mode */
    if ( long_mode_active(ctxt) )
        regs->rflags &= ~X86_EFLAGS_VM;

    /* EFLAGS.VM implies 16-bit mode */
    if ( regs->rflags & X86_EFLAGS_VM )
    {
        input.segments[x86_seg_cs].attr.fields.db = 0;
        input.segments[x86_seg_ss].attr.fields.db = 0;
    }
}

int LLVMFuzzerTestOneInput(const uint8_t *data_p, size_t size)
{
    bool stack_exec;
    struct cpu_user_regs regs = {};
    struct x86_emulate_ctxt ctxt = {
        .regs = &regs,
        .addr_size = 8 * sizeof(void *),
        .sp_size = 8 * sizeof(void *),
    };
    int rc;

    stack_exec = emul_test_make_stack_executable();
    if ( !stack_exec )
    {
        printf("Warning: Stack could not be made executable (%d).\n", errno);
        return 1;
    }

    /* Reset all global state variables */
    memset(&input, 0, sizeof(input));
    data_index = 0;
    data_num = 0;

    if ( size <= DATA_OFFSET )
    {
        printf("Input too small\n");
        return 1;
    }

    if ( size > sizeof(input) )
    {
        printf("Input too large\n");
        return 1;
    }

    memcpy(&input, data_p, size);

    data_num = size - DATA_OFFSET;

    sanitize_input(&ctxt);

    disable_hooks();

    set_swint_support(&ctxt);

    do {
        /* FIXME: Until we actually implement SIGFPE handling properly */
        setup_fpu_exception_handler();

        set_sizes(&ctxt);
        dump_state(&ctxt);

        rc = x86_emulate(&ctxt, &fuzz_emulops);
        printf("Emulation result: %d\n", rc);
    } while ( rc == X86EMUL_OKAY );

    return 0;
}

unsigned int fuzz_minimal_input_size(void)
{
    return DATA_OFFSET + 1;
}

/*
 * Local variables:
 * mode: C
 * c-file-style: "BSD"
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * End:
 */
