#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include "../Drivers/sh2/Src/sh2.c"
static unsigned delivered;
static void receive(void *cookie, sh2_SensorEvent_t *event)
{
    (void)cookie;
    assert(event->reportId == SH2_ACCELEROMETER && event->len == 10);
    delivered++;
}
int main(void)
{
    sh2_t hub = {0};
    hub.sensorCallback = receive;
    /* Each allocation ends at the declared length: sanitizers catch any read
       past a truncated record, even if the production RX buffer is larger. */
    for (unsigned id = 0; id <= 255; ++id) {
        unsigned length = getReportLen(id);
        for (unsigned short_len = 1; short_len < length; ++short_len) {
            uint8_t *packet = calloc(short_len, 1);
            assert(packet);
            packet[0] = id;
            sensorhubInputHdlr(&hub, packet, short_len, 0);
            sensorhubControlHdlr(&hub, packet, short_len, 0);
            free(packet);
        }
    }
    for (unsigned n = 1; n < getReportLen(SH2_GYRO_INTEGRATED_RV); ++n) {
        uint8_t *packet = calloc(n, 1);
        assert(packet);
        sensorhubInputGyroRvHdlr(&hub, packet, n, 0);
        free(packet);
    }
    assert(delivered == 0);
    uint8_t valid[20] = {SH2_ACCELEROMETER};
    valid[10] = SH2_ACCELEROMETER;
    sensorhubInputHdlr(&hub, valid, sizeof(valid), 0);
    assert(delivered == 2);
    delivered = 0;
    sensorhubInputHdlr(&hub, valid, sizeof(valid) - 1, 0);
    assert(delivered == 1);
    /* Read/modify/write must not silently turn off on-table calibration.
       Exercise all five calibration enable bits, including the old missing R5. */
    for (unsigned mask = 0; mask < 32; ++mask) {
        CommandResp_t response = {0};
        response.reportId = SENSORHUB_COMMAND_RESP;
        response.command = hub.lastCmdId = SH2_CMD_ME_CAL;
        response.commandSeq = hub.cmdSeq;
        for (unsigned bit = 0; bit < 5; ++bit) response.r[bit + 1] = !!(mask & (1U << bit));
        uint8_t result = 0xff;
        hub.opData.getCalConfig.pSensors = &result;
        getCalConfigRx(&hub, (const uint8_t *)&response, sizeof(response));
        assert(hub.opStatus == SH2_OK && result == mask);
    }
    puts("PASS: truncated SH-2 records rejected, complete records preserved, all calibration enable bits decoded");
}
