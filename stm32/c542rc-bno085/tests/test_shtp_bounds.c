#include <assert.h>
#include <stdio.h>
#include "../Drivers/sh2/Src/shtp.c"
static unsigned delivered;
static void receive(void *cookie, uint8_t *data, uint16_t length, uint32_t time)
{
    (void)cookie; (void)time;
    assert(length == 2 && data[0] == 0x12 && data[1] == 0x34);
    delivered++;
}
int main(void)
{
    shtp_t transport = {0};
    for (unsigned channel = SHTP_MAX_CHANS; channel <= UINT8_MAX; ++channel) {
        uint8_t header[] = {4, 0, (uint8_t)channel, 0};
        rxAssemble(&transport, header, sizeof(header), 0);
    }
    assert(transport.rxBadChan == 256 - SHTP_MAX_CHANS);
    uint8_t packet[] = {6, 0, 3, 0, 0x12, 0x34};
    transport.chan[3].callback = receive;
    for (unsigned length = 0; length < 4; ++length) rxAssemble(&transport, packet, length, 0);
    assert(transport.rxShortFragments == 4 && delivered == 0);
    rxAssemble(&transport, packet, sizeof(packet), 0);
    assert(delivered == 1);
    puts("PASS: all invalid SHTP channels and short headers rejected; valid packet delivered");
}
