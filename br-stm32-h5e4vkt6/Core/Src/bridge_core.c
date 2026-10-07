/**
 * @file bridge_core.c
 * @brief Pure, host-testable bridge logic. See bridge_core.h.
 */
#include "bridge_core.h"

#include <string.h>

bool bridge_reasm_init(bridge_reasm_t *r, uint32_t can_base_id,
                       size_t msg_size, uint32_t timeout_ms) {
    if (r == NULL) {
        return false;
    }
    memset(r, 0, sizeof(*r));
    r->can_base_id = can_base_id;
    r->timeout_ms  = timeout_ms;

    if (msg_size == 0 || msg_size > BRIDGE_REASM_MAX_MSG_SIZE) {
        return false; /* num_frames stays 0 -> feed() ignores everything */
    }
    size_t frames = (msg_size + 7U) / 8U;
    if (frames > BRIDGE_REASM_MAX_FRAMES) {
        return false;
    }
    r->msg_size      = msg_size;
    r->num_frames    = (uint8_t)frames;
    r->expected_mask = (uint16_t)((1UL << frames) - 1UL);
    return true;
}

bridge_reasm_result_t bridge_reasm_feed(bridge_reasm_t *r, uint32_t id,
                                        const uint8_t *data, uint32_t dlc,
                                        uint32_t now_ms) {
    if (r == NULL || data == NULL || r->num_frames == 0) {
        return BRIDGE_REASM_IGNORED;
    }
    if (id < r->can_base_id || (id - r->can_base_id) >= r->num_frames) {
        return BRIDGE_REASM_IGNORED;
    }
    const uint8_t idx = (uint8_t)(id - r->can_base_id);

    /* Guard 1: a partially received message that went stale is discarded. */
    if (r->received_mask != 0 && (uint32_t)(now_ms - r->last_recv_tick) > r->timeout_ms) {
        r->received_mask = 0;
        r->stat_timeouts++;
    }
    /* Guard 2: frame 0 always starts a new message (lost-frame resync). */
    if (idx == 0 && r->received_mask != 0) {
        r->received_mask = 0;
        r->stat_restarts++;
    }
    r->last_recv_tick = now_ms;

    const size_t offset = (size_t)idx * 8U;
    const size_t chunk  = (r->msg_size - offset >= 8U) ? 8U : (r->msg_size - offset);
    size_t copy = bridge_clamp_dlc(dlc);
    if (copy > chunk) {
        copy = chunk;
    }
    memcpy(&r->buffer[offset], data, copy);
    if (copy < chunk) {
        /* Never leave bytes of a previous message behind. */
        memset(&r->buffer[offset + copy], 0, chunk - copy);
        r->stat_short_frames++;
    }

    r->received_mask |= (uint16_t)(1U << idx);
    if (r->received_mask == r->expected_mask) {
        r->received_mask = 0;
        r->rx_msg_count++;
        return BRIDGE_REASM_COMPLETE;
    }
    return BRIDGE_REASM_PENDING;
}

size_t bridge_filter_slots_add(uint16_t *slots, size_t count, size_t max,
                               uint32_t base_id, uint8_t n_frames) {
    for (uint8_t f = 0; f < n_frames && count < max; f++) {
        slots[count++] = (uint16_t)(((base_id + f) & 0x7FFU) << 5);
    }
    return count;
}

size_t bridge_filter_pack_banks(const uint16_t *slots, size_t count,
                                uint16_t banks[][BRIDGE_FILTER_IDS_PER_BANK],
                                size_t max_banks) {
    size_t bank = 0;
    size_t cur  = 0;
    while (cur < count && bank < max_banks) {
        for (size_t j = 0; j < BRIDGE_FILTER_IDS_PER_BANK; j++) {
            banks[bank][j] = (cur < count) ? slots[cur++] : banks[bank][j - 1];
        }
        bank++;
    }
    return bank;
}

uint32_t bridge_backoff_ms(uint32_t attempt, uint32_t min_ms, uint32_t max_ms) {
    if (min_ms > max_ms) {
        return max_ms;
    }
    uint64_t delay = min_ms;
    for (uint32_t i = 1; i < attempt && delay < max_ms; i++) {
        delay *= 2U;
    }
    return (uint32_t)(delay > max_ms ? max_ms : delay);
}

int bridge_health_find_stalled(const bridge_health_task_t *tasks, size_t n,
                               uint32_t now_ms) {
    if (tasks == NULL) {
        return -1;
    }
    for (size_t i = 0; i < n; i++) {
        if (tasks[i].enabled &&
            (uint32_t)(now_ms - tasks[i].last_checkin_ms) > tasks[i].limit_ms) {
            return (int)i;
        }
    }
    return -1;
}
