#ifndef ROADFUSION_HANDSHAKE_REFERENCE_H
#define ROADFUSION_HANDSHAKE_REFERENCE_H
/* Protocol reference, not a Linux PCI driver. Call every helper under the
 * same driver lock. Async consumers must finish before rf_hs_release().
 * The adapter owns DMA allocation, byte order, READ_ONCE, barriers and MMIO.
 * This header targets a C99 userspace/mock adapter; adapt types for kernel use.
 */
#include <stdint.h>
#include <stddef.h>

#define RF_IMAGE_BYTES UINT32_C(0x3f4800)
#define RF_BUFFER_BYTES UINT32_C(0x3f4840)
#define RF_STATUS_SLOT 4u

struct rf_hs_ops {
    /* Return a logical LE32-decoded marker word. slot 4 means STOP status.
     * Coherent kernel adapter: le32_to_cpu(READ_ONCE(marker[dw])). */
    uint32_t (*read_word)(void *ctx, unsigned slot, unsigned dw);
    /* Clear exactly 64 bytes, only while software owns this marker area. */
    void (*clear_marker)(void *ctx, unsigned slot);
    /* Kernel adapter uses dma_rmb()/dma_wmb(); these are NOT no-op hooks. */
    void (*read_barrier)(void *ctx);
    void (*write_barrier)(void *ctx);
    /* Ordered, one-DW BAR1 command using logical values from the guide.
     * Verify the IP's byte mapping before choosing writel(value) vs swapping. */
    void (*bar_write32)(void *ctx, unsigned offset, uint32_t value);
};

struct rf_hs {
    const struct rf_hs_ops *ops;
    void *ctx;
    uint32_t last_token[4];
    unsigned held[4];
    unsigned running, stopping, configured;
    uint32_t stop_cookie, previous_cookie;
};

/* Use zero-initialized storage, only after known reset or confirmed STOP.
 * Do not initialize away bookkeeping while hardware may still be writing. */
static inline int rf_hs_init(struct rf_hs *s, const struct rf_hs_ops *ops, void *ctx)
{
    unsigned i;
    if (!s || !ops || !ops->read_word || !ops->clear_marker ||
        !ops->read_barrier || !ops->write_barrier || !ops->bar_write32) return -1;
    s->ops=ops; s->ctx=ctx;
    s->running=s->stopping=s->configured=0;
    s->stop_cookie=s->previous_cookie=0;
    for (i=0;i<4;i++) { s->held[i]=0; s->last_token[i]=0; }
    return 0;
}

static inline int rf_hs_consumers_done(const struct rf_hs *s)
{
    unsigned i;
    for (i=0;i<4;i++) if (s->held[i]) return 0;
    return 1;
}

/* addr[] are DMA/IOVA addresses, never CPU pointers. Validate full ranges
 * in 64 bits BEFORE truncating to the 32-bit hardware register. */
static inline int rf_hs_configure(struct rf_hs *s, const uint64_t addr[4], uint64_t status)
{
    unsigned i,j;
    if (s->running || s->stopping || !rf_hs_consumers_done(s)) return -1;
    if (!status || (status&63) || status>UINT64_C(0x100000000)-64) return -1;
    for (i=0;i<4;i++) {
        if (!addr[i] || (addr[i]&63) || addr[i]>UINT64_C(0x100000000)-RF_BUFFER_BYTES) return -1;
        if (!(addr[i]+RF_BUFFER_BYTES<=status || status+64<=addr[i])) return -1;
        for (j=0;j<i;j++)
            if (!(addr[i]+RF_BUFFER_BYTES<=addr[j] || addr[j]+RF_BUFFER_BYTES<=addr[i])) return -1;
    }
    s->ops->bar_write32(s->ctx,0x1a0,1);
    for (i=0;i<4;i++) s->ops->bar_write32(s->ctx,0x110,(uint32_t)addr[i]);
    s->ops->bar_write32(s->ctx,0x180,(uint32_t)status);
    /* Locally submitted, not hardware-confirmed. An idle STOP echo can
     * validate the configuration before starting capture. */
    s->configured=1;
    return 0;
}

static inline int rf_hs_start(struct rf_hs *s)
{
    unsigned i;
    if (!s->configured || s->running || s->stopping || !rf_hs_consumers_done(s)) return -1;
    for (i=0;i<5;i++) s->ops->clear_marker(s->ctx,i);
    s->ops->write_barrier(s->ctx);
    s->ops->bar_write32(s->ctx,0x190,1);
    s->running=1;
    return 0;
}

/* Returns 1 and grants a software-held frame, 0 if not ready, -1 on misuse.
 * Pixels may be consumed only after this helper returns 1. */
static inline int rf_hs_poll_frame(struct rf_hs *s, unsigned slot, uint32_t *token)
{
    unsigned i;
    uint32_t v;
    if (slot>=4 || !token) return -1;
    if (!s->running || s->stopping || s->held[slot]) return 0;
    v=s->ops->read_word(s->ctx,slot,0);
    if (!(v>>2) || (v&3)!=slot || v==s->last_token[slot]) return 0;
    for (i=1;i<16;i++) if (s->ops->read_word(s->ctx,slot,i)!=v) return 0;
    s->ops->read_barrier(s->ctx);
    if (s->ops->read_word(s->ctx,slot,0)!=v) return 0;
    s->last_token[slot]=v; s->held[slot]=1; *token=v;
    return 1;
}

/* Call once the LAST CPU/RGA/NPU/scanout consumer has finished. After the
 * RELEASE write, do not touch pixels or marker again until a new completion. */
static inline int rf_hs_release(struct rf_hs *s, unsigned slot, uint32_t token)
{
    if (slot>=4 || !s->held[slot] || token!=s->last_token[slot] || s->stopping) return -1;
    if (s->running) {
        s->ops->clear_marker(s->ctx,slot);
        s->ops->write_barrier(s->ctx);
        s->ops->bar_write32(s->ctx,0x170,token);
    }
    /* Once STOP is acknowledged, hardware needs no RELEASE. We still wait
     * for consumers before dropping the software hold or permitting START. */
    s->held[slot]=0;
    return 0;
}

static inline int rf_hs_request_stop(struct rf_hs *s, uint32_t cookie)
{
    if (!s->configured || s->stopping || !cookie || cookie==s->previous_cookie) return -1;
    s->ops->clear_marker(s->ctx,RF_STATUS_SLOT);
    s->ops->write_barrier(s->ctx);
    s->stop_cookie=cookie; s->previous_cookie=cookie;
    s->stopping=1; s->running=0;
    s->ops->bar_write32(s->ctx,0x130,cookie);
    return 0;
}

static inline int rf_hs_poll_stop(struct rf_hs *s)
{
    unsigned i;
    if (!s->stopping) return -1;
    for (i=0;i<16;i++)
        if (s->ops->read_word(s->ctx,RF_STATUS_SLOT,i)!=s->stop_cookie) return 0;
    s->ops->read_barrier(s->ctx);
    s->stopping=0;
    return 1;
}
/* A timeout is NOT equivalent to rf_hs_poll_stop()==1. Keep DMA mappings
 * alive until a successful STOP or a platform-verified reset/drain recovery. */
#endif
