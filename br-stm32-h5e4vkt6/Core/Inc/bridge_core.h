/**
 * @file bridge_core.h
 * @brief Hardware-independent core logic of the Zenoh <-> CAN bridge.
 *
 * Everything in here is pure C (no HAL / FreeRTOS / LwIP). It is compiled both
 * into the firmware and into the host unit tests (tests/test_bridge_core.c),
 * so the code that runs on the robot is exactly the code that is tested.
 */
#ifndef BRIDGE_CORE_H
#define BRIDGE_CORE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ───────────────────────── CAN frame helpers ───────────────────────────── */

/** Classic CAN carries at most 8 data bytes; bxCAN can report DLC up to 15. */
#define BRIDGE_CAN_MAX_DLC 8U

/** Clamp a raw (possibly corrupt) DLC to the 0..8 range. */
static inline uint8_t bridge_clamp_dlc(uint32_t dlc) {
    return (uint8_t)(dlc > BRIDGE_CAN_MAX_DLC ? BRIDGE_CAN_MAX_DLC : dlc);
}

/* ───────────────────── Multi-frame reassembly ──────────────────────────── */

#define BRIDGE_REASM_MAX_MSG_SIZE 128U /**< must match BRIDGE_MAX_MSG_SIZE   */
#define BRIDGE_REASM_MAX_FRAMES   16U  /**< width of the received_mask (u16) */

typedef enum {
    BRIDGE_REASM_IGNORED = 0, /**< CAN id does not belong to this message   */
    BRIDGE_REASM_PENDING,     /**< accepted, message still incomplete       */
    BRIDGE_REASM_COMPLETE,    /**< message complete; buffer is valid        */
} bridge_reasm_result_t;

typedef struct {
    uint8_t  buffer[BRIDGE_REASM_MAX_MSG_SIZE];
    uint32_t can_base_id;
    uint32_t timeout_ms;
    uint32_t last_recv_tick;
    uint32_t rx_msg_count;       /**< completed messages                    */
    uint32_t stat_timeouts;      /**< partial message dropped by timeout    */
    uint32_t stat_restarts;      /**< partial message dropped by frame-0    */
    uint32_t stat_short_frames;  /**< frame with DLC smaller than expected  */
    size_t   msg_size;
    uint16_t received_mask;
    uint16_t expected_mask;
    uint8_t  num_frames;
} bridge_reasm_t;

/**
 * Prepare a reassembly slot.
 * @return false if msg_size is 0 or does not fit the buffer / mask width
 *         (the slot is then left unusable: feed() returns IGNORED).
 */
bool bridge_reasm_init(bridge_reasm_t *r, uint32_t can_base_id,
                       size_t msg_size, uint32_t timeout_ms);

/** Feed one CAN frame. Self-healing: stale partial data never leaks out. */
bridge_reasm_result_t bridge_reasm_feed(bridge_reasm_t *r, uint32_t id,
                                        const uint8_t *data, uint32_t dlc,
                                        uint32_t now_ms);

/* ───────────────────── Hardware filter planning ────────────────────────── */

#define BRIDGE_FILTER_IDS_PER_BANK 4U /* 16-bit list mode: 4 ids per bank */

/** Append `n_frames` consecutive standard ids to the slot list. */
size_t bridge_filter_slots_add(uint16_t *slots, size_t count, size_t max,
                               uint32_t base_id, uint8_t n_frames);

/**
 * Pack id slots (already shifted <<5) into 16-bit list-mode banks of 4,
 * padding the last bank with a duplicate of an id that is already present.
 * @return number of banks used (<= max_banks)
 */
size_t bridge_filter_pack_banks(const uint16_t *slots, size_t count,
                                uint16_t banks[][BRIDGE_FILTER_IDS_PER_BANK],
                                size_t max_banks);

/* ───────────────────── Reconnect back-off ──────────────────────────────── */

/**
 * Exponential back-off: min, 2*min, 4*min ... capped at max.
 * attempt is 1-based (attempt 0 is treated as 1). Never returns 0 for min>0.
 */
uint32_t bridge_backoff_ms(uint32_t attempt, uint32_t min_ms, uint32_t max_ms);

/* ───────────────────── Task liveness (watchdog policy) ─────────────────── */

typedef struct {
    const char *name;
    uint32_t    last_checkin_ms; /**< tick of the last heartbeat           */
    uint32_t    limit_ms;        /**< max allowed silence                  */
    bool        enabled;
} bridge_health_task_t;

/**
 * @return index of the first enabled task that has been silent longer than
 *         its limit (wrap-safe), or -1 if everybody is alive.
 */
int bridge_health_find_stalled(const bridge_health_task_t *tasks, size_t n,
                               uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* BRIDGE_CORE_H */
