/**
 * @file test_bridge_core.c
 * @brief Host tests that exercise the REAL firmware logic (bridge_core.c).
 *
 * Includes a randomized differential test against an independent, naive
 * reference model, plus boundary / corruption / wrap-around cases.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "bridge_core.h"

static int g_checks = 0;
static int g_fail   = 0;

#define CHECK(cond) do { \
    g_checks++; \
    if (!(cond)) { g_fail++; printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); } \
} while (0)

/* ───────────────────────────── DLC ─────────────────────────────────────── */
static void test_dlc(void) {
    for (uint32_t d = 0; d <= 8; d++) CHECK(bridge_clamp_dlc(d) == d);
    for (uint32_t d = 9; d <= 15; d++) CHECK(bridge_clamp_dlc(d) == 8);
    CHECK(bridge_clamp_dlc(0xFFFFFFFFu) == 8);
}

/* ─────────────────────────── Reassembly ────────────────────────────────── */
static void fill(uint8_t *p, size_t n, uint8_t seed) {
    for (size_t i = 0; i < n; i++) p[i] = (uint8_t)(seed + i * 7);
}

static void test_reasm_init_limits(void) {
    bridge_reasm_t r;
    CHECK(!bridge_reasm_init(NULL, 0, 8, 100));
    CHECK(!bridge_reasm_init(&r, 0x100, 0, 100));
    CHECK(!bridge_reasm_init(&r, 0x100, BRIDGE_REASM_MAX_MSG_SIZE + 1, 100));
    uint8_t d[8] = {0};
    CHECK(bridge_reasm_feed(&r, 0x100, d, 8, 0) == BRIDGE_REASM_IGNORED); /* unusable slot */
    CHECK(bridge_reasm_init(&r, 0x100, BRIDGE_REASM_MAX_MSG_SIZE, 100));
    CHECK(r.num_frames == 16 && r.expected_mask == 0xFFFF);
    CHECK(bridge_reasm_init(&r, 0x100, 1, 100));
    CHECK(r.num_frames == 1 && r.expected_mask == 0x1);
    CHECK(bridge_reasm_init(&r, 0x100, 9, 100));
    CHECK(r.num_frames == 2 && r.expected_mask == 0x3);
}

static void test_reasm_roundtrip_all_sizes(void) {
    for (size_t size = 1; size <= BRIDGE_REASM_MAX_MSG_SIZE; size++) {
        bridge_reasm_t r;
        uint8_t msg[BRIDGE_REASM_MAX_MSG_SIZE];
        fill(msg, size, (uint8_t)size);
        CHECK(bridge_reasm_init(&r, 0x200, size, 100));
        bridge_reasm_result_t res = BRIDGE_REASM_PENDING;
        for (uint8_t f = 0; f < r.num_frames; f++) {
            size_t off = (size_t)f * 8;
            uint32_t chunk = (size - off >= 8) ? 8 : (uint32_t)(size - off);
            CHECK(res != BRIDGE_REASM_COMPLETE);
            res = bridge_reasm_feed(&r, 0x200 + f, msg + off, chunk, 10 + f);
        }
        CHECK(res == BRIDGE_REASM_COMPLETE);
        CHECK(memcmp(r.buffer, msg, size) == 0);
        CHECK(r.received_mask == 0 && r.rx_msg_count == 1);
    }
}

static void test_reasm_out_of_order(void) {
    bridge_reasm_t r;
    uint8_t msg[24];
    fill(msg, sizeof msg, 3);
    CHECK(bridge_reasm_init(&r, 0x300, 24, 100));
    /* frame 0 must come first (it resets); then any order for the rest */
    CHECK(bridge_reasm_feed(&r, 0x300, msg, 8, 0) == BRIDGE_REASM_PENDING);
    CHECK(bridge_reasm_feed(&r, 0x302, msg + 16, 8, 1) == BRIDGE_REASM_PENDING);
    CHECK(bridge_reasm_feed(&r, 0x301, msg + 8, 8, 2) == BRIDGE_REASM_COMPLETE);
    CHECK(memcmp(r.buffer, msg, 24) == 0);
}

static void test_reasm_loss_and_resync(void) {
    bridge_reasm_t r;
    uint8_t a[24], b[24];
    fill(a, 24, 1);
    fill(b, 24, 100);
    CHECK(bridge_reasm_init(&r, 0x400, 24, 100));
    /* message A loses frame 1 */
    CHECK(bridge_reasm_feed(&r, 0x400, a, 8, 0) == BRIDGE_REASM_PENDING);
    CHECK(bridge_reasm_feed(&r, 0x402, a + 16, 8, 1) == BRIDGE_REASM_PENDING);
    /* message B starts: frame 0 resets, B must come out intact, never A+B mix */
    CHECK(bridge_reasm_feed(&r, 0x400, b, 8, 5) == BRIDGE_REASM_PENDING);
    CHECK(r.stat_restarts == 1);
    CHECK(bridge_reasm_feed(&r, 0x401, b + 8, 8, 6) == BRIDGE_REASM_PENDING);
    CHECK(bridge_reasm_feed(&r, 0x402, b + 16, 8, 7) == BRIDGE_REASM_COMPLETE);
    CHECK(memcmp(r.buffer, b, 24) == 0);
}

static void test_reasm_timeout_and_wrap(void) {
    bridge_reasm_t r;
    uint8_t m[16];
    fill(m, 16, 9);
    CHECK(bridge_reasm_init(&r, 0x500, 16, 100));
    CHECK(bridge_reasm_feed(&r, 0x500, m, 8, 1000) == BRIDGE_REASM_PENDING);
    /* second half arrives too late: stale -> discarded, not completed */
    CHECK(bridge_reasm_feed(&r, 0x501, m + 8, 8, 1101) == BRIDGE_REASM_PENDING);
    CHECK(r.stat_timeouts == 1 && r.rx_msg_count == 0);
    /* exactly at the limit is still valid */
    CHECK(bridge_reasm_feed(&r, 0x500, m, 8, 2000) == BRIDGE_REASM_PENDING);
    CHECK(bridge_reasm_feed(&r, 0x501, m + 8, 8, 2100) == BRIDGE_REASM_COMPLETE);
    /* uint32 tick wrap-around must not fake a timeout */
    CHECK(bridge_reasm_feed(&r, 0x500, m, 8, 0xFFFFFFF0u) == BRIDGE_REASM_PENDING);
    CHECK(bridge_reasm_feed(&r, 0x501, m + 8, 8, 0x00000010u) == BRIDGE_REASM_COMPLETE);
    CHECK(memcmp(r.buffer, m, 16) == 0);
}

static void test_reasm_corrupt_input(void) {
    bridge_reasm_t r;
    uint8_t m[16];
    fill(m, 16, 5);
    CHECK(bridge_reasm_init(&r, 0x600, 16, 100));
    /* short frame: remainder is zero filled, stale bytes never survive */
    memset(r.buffer, 0xEE, sizeof r.buffer);
    CHECK(bridge_reasm_feed(&r, 0x600, m, 3, 0) == BRIDGE_REASM_PENDING);
    CHECK(memcmp(r.buffer, m, 3) == 0);
    for (int i = 3; i < 8; i++) CHECK(r.buffer[i] == 0);
    CHECK(r.stat_short_frames == 1);
    /* DLC 15 (corrupt) must never read past data[8] or write past chunk */
    uint8_t guard[8 + 8];
    memset(guard, 0xAA, sizeof guard);
    memcpy(guard, m + 8, 8);
    CHECK(bridge_reasm_feed(&r, 0x601, guard, 15, 1) == BRIDGE_REASM_COMPLETE);
    /* ids outside the window */
    CHECK(bridge_reasm_feed(&r, 0x5FF, m, 8, 2) == BRIDGE_REASM_IGNORED);
    CHECK(bridge_reasm_feed(&r, 0x602, m, 8, 2) == BRIDGE_REASM_IGNORED);
    CHECK(bridge_reasm_feed(&r, 0xFFFFFFFFu, m, 8, 2) == BRIDGE_REASM_IGNORED);
    CHECK(bridge_reasm_feed(&r, 0x600, NULL, 8, 2) == BRIDGE_REASM_IGNORED);
    CHECK(bridge_reasm_feed(NULL, 0x600, m, 8, 2) == BRIDGE_REASM_IGNORED);
    /* base id near the top of the u32 range must not overflow */
    CHECK(bridge_reasm_init(&r, 0xFFFFFFFEu, 16, 100));
    CHECK(bridge_reasm_feed(&r, 0xFFFFFFFEu, m, 8, 0) == BRIDGE_REASM_PENDING);
    CHECK(bridge_reasm_feed(&r, 0xFFFFFFFFu, m + 8, 8, 0) == BRIDGE_REASM_COMPLETE);
    CHECK(bridge_reasm_feed(&r, 0x00000000u, m, 8, 0) == BRIDGE_REASM_IGNORED);
}

/* Randomized differential test against a naive reference implementation. */
typedef struct {
    uint8_t  buf[BRIDGE_REASM_MAX_MSG_SIZE];
    int      have[BRIDGE_REASM_MAX_FRAMES];
    uint32_t last;
    int      any;
} ref_t;

static uint32_t lcg(uint32_t *s) { *s = *s * 1664525u + 1013904223u; return *s >> 8; }

static void test_reasm_fuzz_vs_reference(void) {
    uint32_t seed = 12345;
    for (int round = 0; round < 300; round++) {
        size_t size = 1 + lcg(&seed) % BRIDGE_REASM_MAX_MSG_SIZE;
        uint32_t base = lcg(&seed) % 0x700;
        uint32_t tmo = 50 + lcg(&seed) % 200;
        bridge_reasm_t r;
        CHECK(bridge_reasm_init(&r, base, size, tmo));
        ref_t ref;
        memset(&ref, 0, sizeof ref);
        size_t nf = (size + 7) / 8;
        uint32_t now = lcg(&seed);
        for (int i = 0; i < 400; i++) {
            now += lcg(&seed) % 120;
            uint32_t id = base + lcg(&seed) % (nf + 2) - (lcg(&seed) % 4 == 0 ? 1 : 0);
            uint8_t d[8];
            for (int k = 0; k < 8; k++) d[k] = (uint8_t)lcg(&seed);
            uint32_t dlc = lcg(&seed) % 16;

            /* reference model */
            int expect_complete = 0;
            int in_window = (id >= base && id - base < nf);
            if (in_window) {
                size_t idx = id - base;
                if (ref.any && (uint32_t)(now - ref.last) > tmo) { memset(ref.have, 0, sizeof ref.have); ref.any = 0; }
                if (idx == 0) { memset(ref.have, 0, sizeof ref.have); ref.any = 0; }
                ref.last = now;
                size_t off = idx * 8, chunk = (size - off >= 8) ? 8 : size - off;
                size_t cp = dlc > 8 ? 8 : dlc; if (cp > chunk) cp = chunk;
                memset(ref.buf + off, 0, chunk);
                memcpy(ref.buf + off, d, cp);
                ref.have[idx] = 1; ref.any = 1;
                int all = 1; for (size_t k = 0; k < nf; k++) all &= ref.have[k];
                if (all) { expect_complete = 1; memset(ref.have, 0, sizeof ref.have); ref.any = 0; }
            }
            bridge_reasm_result_t res = bridge_reasm_feed(&r, id, d, dlc, now);
            if (!in_window) { CHECK(res == BRIDGE_REASM_IGNORED); continue; }
            CHECK((res == BRIDGE_REASM_COMPLETE) == expect_complete);
            if (expect_complete) CHECK(memcmp(r.buffer, ref.buf, size) == 0);
        }
    }
}

/* ───────────────────────── Filter planning ─────────────────────────────── */
static void test_filters(void) {
    uint16_t slots[56];
    size_t n = 0;
    n = bridge_filter_slots_add(slots, n, 56, 0x123, 3);
    CHECK(n == 3);
    CHECK(slots[0] == (0x123 << 5) && slots[1] == (0x124 << 5) && slots[2] == (0x125 << 5));
    /* ids are masked to 11 bits */
    size_t m = bridge_filter_slots_add(slots, 0, 56, 0x7FF, 2);
    CHECK(m == 2 && slots[0] == (0x7FF << 5) && slots[1] == (0x000 << 5));
    /* capacity is honoured */
    size_t full = bridge_filter_slots_add(slots, 54, 56, 0x100, 16);
    CHECK(full == 56);

    uint16_t banks[14][BRIDGE_FILTER_IDS_PER_BANK];
    n = bridge_filter_slots_add(slots, 0, 56, 0x10, 6);
    size_t nb = bridge_filter_pack_banks(slots, n, banks, 14);
    CHECK(nb == 2);
    CHECK(banks[1][0] == slots[4] && banks[1][1] == slots[5]);
    CHECK(banks[1][2] == slots[5] && banks[1][3] == slots[5]); /* padded with an existing id */
    CHECK(bridge_filter_pack_banks(slots, 0, banks, 14) == 0);
    n = bridge_filter_slots_add(slots, 0, 56, 0, 56 > 255 ? 255 : 56 + 0);
    CHECK(bridge_filter_pack_banks(slots, 56, banks, 14) == 14);
    CHECK(bridge_filter_pack_banks(slots, 56, banks, 5) == 5);  /* bank limit honoured */
}

/* ───────────────────────────── Backoff ─────────────────────────────────── */
static void test_backoff(void) {
    CHECK(bridge_backoff_ms(0, 200, 5000) == 200);
    CHECK(bridge_backoff_ms(1, 200, 5000) == 200);
    CHECK(bridge_backoff_ms(2, 200, 5000) == 400);
    CHECK(bridge_backoff_ms(3, 200, 5000) == 800);
    CHECK(bridge_backoff_ms(5, 200, 5000) == 3200);
    CHECK(bridge_backoff_ms(6, 200, 5000) == 5000);
    CHECK(bridge_backoff_ms(1000000, 200, 5000) == 5000);
    CHECK(bridge_backoff_ms(0xFFFFFFFFu, 200, 5000) == 5000);
    CHECK(bridge_backoff_ms(3, 9000, 5000) == 5000); /* misconfig is clamped */
    uint32_t prev = 0;
    for (uint32_t a = 1; a < 100; a++) {  /* monotonic, bounded, never 0 */
        uint32_t d = bridge_backoff_ms(a, 200, 5000);
        CHECK(d >= prev && d >= 200 && d <= 5000);
        prev = d;
    }
}

/* ───────────────────────────── Health ──────────────────────────────────── */
static void test_health(void) {
    bridge_health_task_t t[2] = {
        {"engine", 1000, 30000, true},
        {"bridge", 1000, 1000, true},
    };
    CHECK(bridge_health_find_stalled(t, 2, 1000) == -1);
    CHECK(bridge_health_find_stalled(t, 2, 2000) == -1);   /* at the limit: alive */
    CHECK(bridge_health_find_stalled(t, 2, 2001) == 1);    /* bridge silent */
    t[1].enabled = false;
    CHECK(bridge_health_find_stalled(t, 2, 2001) == -1);   /* disabled ignored */
    CHECK(bridge_health_find_stalled(t, 2, 31001) == 0);   /* engine stalled */
    /* tick wrap-around */
    t[0].last_checkin_ms = 0xFFFFFF00u;
    CHECK(bridge_health_find_stalled(t, 2, 0x00000100u) == -1);
    CHECK(bridge_health_find_stalled(NULL, 2, 0) == -1);
}

int main(void) {
    test_dlc();
    test_reasm_init_limits();
    test_reasm_roundtrip_all_sizes();
    test_reasm_out_of_order();
    test_reasm_loss_and_resync();
    test_reasm_timeout_and_wrap();
    test_reasm_corrupt_input();
    test_reasm_fuzz_vs_reference();
    test_filters();
    test_backoff();
    test_health();
    printf("%d checks, %d failed\n", g_checks, g_fail);
    return g_fail ? 1 : 0;
}
