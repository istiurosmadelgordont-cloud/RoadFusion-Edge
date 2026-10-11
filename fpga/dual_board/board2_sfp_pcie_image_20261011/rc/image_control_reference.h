#ifndef FPGA_IMAGE_CONTROL_REFERENCE_H
#define FPGA_IMAGE_CONTROL_REFERENCE_H

/* C99 protocol adapter, not a PCIe driver. Use the same driver lock for all
 * commands. DMA allocation, lifetime, MMIO ordering and barriers belong to
 * the platform driver. Values here are logical LE32-decoded values.
 */
#include <stdint.h>
#include <stddef.h>

#define RF_IMG_MAGIC         UINT32_C(0x494d4743)
#define RF_IMG_BUFFER_BYTES  UINT32_C(0x003f4840)
#define RF_IMG_ACK_BYTES     64u

enum rf_img_status {
    RF_IMG_RESET = 0,
    RF_IMG_ACCEPTED = 1,
    RF_IMG_APPLIED = 2,
    RF_IMG_BUSY = 3,
    RF_IMG_BAD_COMMAND = 4,
    RF_IMG_BAD_ADDRESS = 5,
    RF_IMG_OVERFLOW = 6,
    RF_IMG_WAITING_INPUT = 7,
    RF_IMG_RESTORING = 8
};

struct rf_img_ops {
    /* Read coherent ACK memory using READ_ONCE + le32_to_cpu in a kernel
     * adapter. This function must perform a new load on every invocation. */
    uint32_t (*read_ack_word)(void *ctx, unsigned dw);
    void (*read_barrier)(void *ctx);       /* dma_rmb(), not a no-op */
    void (*bar_write32)(void *ctx, unsigned offset, uint32_t value);
};

struct rf_img_reply {
    uint32_t sequence;
    uint32_t cookie;
    uint32_t status;
    uint32_t flags;
    uint32_t active[5];
    uint32_t accepted_cookie;
    uint32_t applied_cookie;
    uint32_t rejected_count;
    uint32_t overflow_count;
    uint32_t last_rejected_cookie;
};

static inline uint32_t rf_img_clip(uint32_t value, uint32_t limit)
{
    return value > limit ? limit : value;
}

/* Mirrors image_runtime_control.sanitize_config, including reserved bits. */
static inline void rf_img_sanitize(const uint32_t in[5], uint32_t out[5])
{
    uint32_t floor = in[1] & 255u;
    uint32_t sky = (in[1] >> 8) & 255u;
    uint32_t sigma = (in[2] >> 9) & 63u;
    if (!floor) floor = 1;
    if (sky < floor) sky = floor;
    if (sigma != 16 && sigma != 48) sigma = 32;
    out[0] = in[0] & UINT32_C(0x000019ff);
    out[1] = floor | (sky << 8) | (in[1] & UINT32_C(0x00ff0000));
    out[2] = rf_img_clip(in[2] & 511u, 256u) | (sigma << 9) |
             (in[2] & UINT32_C(0xffff0000));
    out[3] = (in[3] & 65535u ? in[3] & 65535u : 1u) |
             (rf_img_clip((in[3] >> 16) & 7u, 4u) << 16) |
             (rf_img_clip((in[3] >> 19) & 127u, 64u) << 19);
    out[4] = (in[4] & 65535u) |
             (rf_img_clip((in[4] >> 16) & 511u, 256u) << 16);
}

/* Validate before truncating DMA/IOVA to the current 32-bit MWr address.
 * The caller must supply all currently registered image/STOP ranges.
 */
static inline int rf_img_ack_address_ok(uint64_t ack,
                                       const uint64_t image[4], uint64_t stop)
{
    unsigned i;
    if (!image || !ack || (ack & 63u) ||
        ack > UINT64_C(0x100000000) - RF_IMG_ACK_BYTES) return 0;
    for (i = 0; i < 4; ++i) {
        if (image[i] && (image[i] > UINT64_C(0x100000000) - RF_IMG_BUFFER_BYTES ||
            !(ack + RF_IMG_ACK_BYTES <= image[i] ||
              image[i] + RF_IMG_BUFFER_BYTES <= ack))) return 0;
    }
    if (stop && (stop > UINT64_C(0x100000000) - RF_IMG_ACK_BYTES ||
        !(ack + RF_IMG_ACK_BYTES <= stop || stop + RF_IMG_ACK_BYTES <= ack))) return 0;
    return 1;
}

/* Call once after reset with the initialized coherent ACK page. Confirm the
 * registration using a QUERY reply before committing. Never clear that page
 * while the endpoint may be writing it. Keep the page mapped across STOP;
 * image-DMA STOP does not stop configuration replies.
 */
static inline int rf_img_register_ack(const struct rf_img_ops *ops, void *ctx,
                                      uint64_t ack, const uint64_t image[4], uint64_t stop)
{
    if (!ops || !ops->bar_write32 || !rf_img_ack_address_ok(ack, image, stop)) return -1;
    ops->bar_write32(ctx, 0x220u, (uint32_t)ack);
    return 0;
}

/* Cookie is nonzero and unique throughout this PCIe reset session. Keep at
 * most one uncompleted commit. Ordered BAR1 MWr calls must not be flushed
 * using BAR reads: this endpoint does not implement MRd completions.
 */
static inline int rf_img_submit(const struct rf_img_ops *ops, void *ctx,
                                const uint32_t words[5], uint32_t cookie)
{
    uint32_t clean[5];
    unsigned i;
    if (!ops || !ops->bar_write32 || !words || !cookie) return -1;
    rf_img_sanitize(words, clean);
    for (i = 0; i < 5; ++i) ops->bar_write32(ctx, 0x200u + 4u * i, clean[i]);
    ops->bar_write32(ctx, 0x214u, cookie);
    return 0;
}

static inline int rf_img_query(const struct rf_img_ops *ops, void *ctx, uint32_t cookie)
{
    if (!ops || !ops->bar_write32 || !cookie) return -1;
    ops->bar_write32(ctx, 0x224u, cookie);
    return 0;
}

/* Return 1 for a complete new record, 0 for unchanged/incomplete data,
 * -1 for a bad adapter. sequence_seen is initialized to zero after reset.
 * A new sequence at the front invalidates the old tail until all four MWr
 * payload beats have arrived; repeated reads reject DMA updates in flight.
 */
static inline int rf_img_poll(const struct rf_img_ops *ops, void *ctx,
                              uint32_t *sequence_seen, struct rf_img_reply *reply)
{
    uint32_t record[16];
    uint32_t first;
    unsigned i;
    if (!ops || !ops->read_ack_word || !ops->read_barrier ||
        !sequence_seen || !reply) return -1;
    first = ops->read_ack_word(ctx, 1);
    if (first == *sequence_seen) return 0;
    ops->read_barrier(ctx);
    for (i = 0; i < 16; ++i) record[i] = ops->read_ack_word(ctx, i);
    ops->read_barrier(ctx);
    if (ops->read_ack_word(ctx, 1) != first || record[1] != first ||
        record[0] != RF_IMG_MAGIC || record[15] != ~first) return 0;
    reply->sequence = first;
    reply->cookie = record[2];
    reply->status = record[3];
    reply->flags = record[4];
    for (i = 0; i < 5; ++i) reply->active[i] = record[5 + i];
    reply->accepted_cookie = record[10];
    reply->applied_cookie = record[11];
    reply->rejected_count = record[12];
    reply->overflow_count = record[13];
    reply->last_rejected_cookie = record[14];
    *sequence_seen = first;
    return 1;
}

/* A QUERY may recover an APPLIED reply overwritten in the single ACK page.
 * Compare all sanitized active words as well, especially after reset.
 */
static inline int rf_img_is_applied(const struct rf_img_reply *reply,
                                    uint32_t cookie, const uint32_t words[5])
{
    uint32_t clean[5];
    unsigned i;
    if (!reply || !words || !cookie || reply->applied_cookie != cookie ||
        (reply->flags & UINT32_C(0x8c)) || (reply->flags & 3u) != 3u) return 0;
    rf_img_sanitize(words, clean);
    for (i = 0; i < 5; ++i) if (reply->active[i] != clean[i]) return 0;
    return 1;
}

#endif
