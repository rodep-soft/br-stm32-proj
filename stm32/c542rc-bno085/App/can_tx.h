#ifndef CAN_TX_H
#define CAN_TX_H
#include <stdbool.h>
#include <stdint.h>
/* Keep one outstanding hardware request; age is measured from the older sample. */
void can_tx_service(uint32_t now_ms, bool invalidate);
bool can_tx_send(const uint8_t payload[8], uint32_t oldest_sample_ms);
extern volatile uint32_t can_queued_frames, can_tx_failures, can_expired_frames, can_sent_frames;
#endif
