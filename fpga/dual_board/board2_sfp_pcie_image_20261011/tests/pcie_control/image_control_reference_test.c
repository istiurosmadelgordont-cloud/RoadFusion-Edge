#include "../../rc/image_control_reference.h"

struct mock {
    uint32_t ack[16];
    uint32_t writes[16];
    unsigned offsets[16];
    unsigned count;
    unsigned mutate_at_read;
    unsigned reads;
    unsigned barriers;
};

static uint32_t mock_read(void *ctx, unsigned dw)
{
    struct mock *m = ctx;
    ++m->reads;
    if (m->mutate_at_read == m->reads) ++m->ack[1];
    return m->ack[dw];
}

static void mock_barrier(void *ctx)
{
    struct mock *m = ctx;
    ++m->barriers;
}

static void mock_write(void *ctx, unsigned offset, uint32_t value)
{
    struct mock *m = ctx;
    m->offsets[m->count] = offset;
    m->writes[m->count++] = value;
}

#define CHECK(expression) do { if (!(expression)) return __LINE__; } while (0)

/* A driver test adapter may call this and require zero. No system PCIe or
 * DMA memory is accessed. Compile this C99 translation unit with warnings.
 */
int rf_img_reference_self_test(void)
{
    const struct rf_img_ops ops = {mock_read, mock_barrier, mock_write};
    struct mock m = {0};
    struct rf_img_reply reply = {0};
    uint32_t seen = 0;
    uint32_t words[5] = {0xffffffffu, 0xff000000u, 0xffff81ffu,
                         0xffffffffu, 0xffffffffu};
    uint32_t clean[5];
    const uint64_t frames[4] = {0x1000000u, 0x2000000u, 0x3000000u, 0x4000000u};
    unsigned i;
    rf_img_sanitize(words, clean);
    CHECK(clean[0] == 0x19ffu);
    CHECK(clean[1] == 0x101u);
    CHECK(clean[2] == 0xffff4100u);
    CHECK(clean[3] == 0x0204ffffu);
    CHECK(clean[4] == 0x0100ffffu);
    CHECK(rf_img_register_ack(&ops, &m, 0x800000u, frames, 0x900000u) == 0);
    CHECK(m.count == 1 && m.offsets[0] == 0x220u && m.writes[0] == 0x800000u);
    CHECK(!rf_img_ack_address_ok(0x1000000u, frames, 0x900000u));
    CHECK(!rf_img_ack_address_ok(0x13f4800u, frames, 0x900000u));
    CHECK(!rf_img_ack_address_ok(0x900000u, frames, 0x900000u));
    CHECK(!rf_img_ack_address_ok(UINT64_C(0x100000000), frames, 0x900000u));
    CHECK(rf_img_ack_address_ok(0x13f4840u, frames, 0x900000u));
    CHECK(rf_img_submit(&ops, &m, words, 7) == 0);
    for (i = 0; i < 5; ++i) {
        CHECK(m.offsets[1 + i] == 0x200u + 4u * i);
        CHECK(m.writes[1 + i] == clean[i]);
    }
    CHECK(m.offsets[6] == 0x214u && m.writes[6] == 7);
    CHECK(rf_img_query(&ops, &m, 8) == 0);
    CHECK(m.offsets[7] == 0x224u && m.writes[7] == 8);
    CHECK(rf_img_poll(&ops, &m, &seen, &reply) == 0);
    m.ack[0] = RF_IMG_MAGIC;
    m.ack[1] = 1;
    m.ack[2] = 7;
    m.ack[3] = RF_IMG_ACCEPTED;
    m.ack[4] = 0x5bu;
    for (i = 0; i < 5; ++i) m.ack[5 + i] = clean[i];
    m.ack[10] = 7;
    CHECK(rf_img_poll(&ops, &m, &seen, &reply) == 0); /* old/incomplete tail */
    m.ack[15] = ~m.ack[1];
    CHECK(rf_img_poll(&ops, &m, &seen, &reply) == 1);
    CHECK(!rf_img_is_applied(&reply, 7, words));
    CHECK(rf_img_poll(&ops, &m, &seen, &reply) == 0);
    m.ack[1] = 2;
    m.ack[2] = 8; /* QUERY can recover overwritten APPLIED */
    m.ack[3] = RF_IMG_APPLIED;
    m.ack[4] = 0x53u;
    m.ack[11] = 7;
    CHECK(rf_img_poll(&ops, &m, &seen, &reply) == 0);
    m.ack[15] = ~m.ack[1];
    m.mutate_at_read = m.reads + 10;
    CHECK(rf_img_poll(&ops, &m, &seen, &reply) == 0); /* write during load */
    m.mutate_at_read = 0;
    m.ack[15] = ~m.ack[1];
    CHECK(rf_img_poll(&ops, &m, &seen, &reply) == 1);
    CHECK(rf_img_is_applied(&reply, 7, words));
    reply.flags |= 0x80u;
    CHECK(!rf_img_is_applied(&reply, 7, words));
    CHECK(rf_img_query(&ops, &m, 0) == -1);
    CHECK(rf_img_submit(&ops, &m, words, 0) == -1);
    return 0;
}
