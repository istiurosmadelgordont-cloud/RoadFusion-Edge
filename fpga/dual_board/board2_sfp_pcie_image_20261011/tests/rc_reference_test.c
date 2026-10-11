#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "../rc/handshake_reference.h"

struct mock {
    uint32_t words[5][16];
    unsigned reads,writes,bar_count,last_offset;
    uint32_t last_value;
    unsigned stage;
};
static uint32_t read_word(void *ctx,unsigned slot,unsigned dw)
{ return ((struct mock *)ctx)->words[slot][dw]; }
static void clear_marker(void *ctx,unsigned slot)
{
    struct mock *m=ctx;
    memset(m->words[slot],0,sizeof(m->words[slot]));m->stage=1;
}
static void rb(void *ctx) { ((struct mock *)ctx)->reads++; }
static void wb(void *ctx)
{
    struct mock *m=ctx;
    assert(m->stage==1);m->stage=2;m->writes++;
}
static void bar(void *ctx,unsigned offset,uint32_t value)
{
    struct mock *m=ctx;
    if(offset==0x130 || offset==0x170 || offset==0x190) {
        assert(m->stage==2);m->stage=0;
    }
    m->bar_count++;m->last_offset=offset;m->last_value=value;
}
static void fill(struct mock *m,unsigned slot,uint32_t value)
{ unsigned i;for(i=0;i<16;i++)m->words[slot][i]=value; }
int main(void)
{
    const struct rf_hs_ops ops={read_word,clear_marker,rb,wb,bar};
    struct mock m;struct rf_hs s;uint32_t token=0;
    uint64_t addr[4]={0x1000000,0x2000000,0x3000000,0x4000000};
    memset(&m,0,sizeof(m));memset(&s,0,sizeof(s));
    assert(rf_hs_init(&s,&ops,&m)==0);
    assert(rf_hs_start(&s)==-1);
    assert(rf_hs_configure(&s,addr,UINT64_C(0xffffffffffffffc0))==-1);
    assert(rf_hs_configure(&s,addr,addr[0])==-1);
    addr[3]=addr[2]+64;assert(rf_hs_configure(&s,addr,0x8000000)==-1);
    addr[3]=0x4000000;
    assert(rf_hs_configure(&s,addr,0x8000000)==0);
    assert(m.bar_count==6 && m.last_offset==0x180);
    assert(rf_hs_start(&s)==0);
    assert(rf_hs_poll_frame(&s,0,&token)==0);
    fill(&m,0,4);m.words[0][15]=0;
    assert(rf_hs_poll_frame(&s,0,&token)==0); /* partially written marker */
    m.words[0][15]=4;
    assert(rf_hs_poll_frame(&s,0,&token)==1 && token==4 && m.reads==1);
    assert(rf_hs_poll_frame(&s,0,&token)==0); /* already held */
    assert(rf_hs_release(&s,0,8)==-1);
    assert(rf_hs_release(&s,0,4)==0 && m.last_offset==0x170 && m.last_value==4);
    assert(rf_hs_release(&s,0,4)==-1);
    fill(&m,0,4);assert(rf_hs_poll_frame(&s,0,&token)==0); /* stale */
    fill(&m,0,8);assert(rf_hs_poll_frame(&s,0,&token)==1 && token==8);
    assert(rf_hs_request_stop(&s,0x12345678)==0);
    assert(rf_hs_poll_stop(&s)==0);
    assert(rf_hs_release(&s,0,8)==-1); /* defer during STOP */
    fill(&m,4,0x12345678);assert(rf_hs_poll_stop(&s)==1);
    assert(rf_hs_start(&s)==-1); /* old display consumer still holds slot */
    assert(rf_hs_release(&s,0,8)==0);
    assert(m.last_offset==0x130); /* stopped release is software-only */
    assert(rf_hs_start(&s)==0);
    assert(rf_hs_request_stop(&s,0x12345678)==-1);
    assert(rf_hs_request_stop(&s,0x12345679)==0);
    puts("RC REFERENCE TESTS PASSED: ranges, partial/stale marker, ownership, barriers, STOP/restart");
    return 0;
}
