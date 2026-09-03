// Band finder: is the desk station transmitting on 868 MHz (EU) rather than
// the 902-928 MHz (US) table we've been scanning?
//
// We hear a distant ID-0 station at ~-115 dBm on the US table, but nothing
// from the unit sitting 30cm away -- and at that range free-space loss is
// only ~21 dB, so it should arrive around -40 dBm. It is therefore almost
// certainly not transmitting where we're listening.
//
// The EU set is only 5 channels, so a full hop cycle is ~13s: if the station
// is an EU unit we should hear it within seconds, and very loudly.
// Accepts every transmitter ID and prints RSSI, so ours is unmistakable.

#include <RadioLib.h>

#define RFM95_CS   16
#define RFM95_INT  21
#define RFM95_RST  17
#define RFM95_DIO1 22

SX1276 radio = new Module(RFM95_CS, RFM95_INT, RFM95_RST, RFM95_DIO1);

#define DAVIS_PACKET_LEN 10
#define DWELL_MS 4000

// Davis EU channel set (868 MHz), from the DavisRFM69 reference table.
static const float euFreq[5] = {
    868.066711f, 868.297119f, 868.527466f, 868.181885f, 868.412292f,
};

volatile bool packetFlag = false;
void onPacketReceived() { packetFlag = true; }
uint8_t idx = 0;
uint32_t hits = 0, noise = 0;

uint16_t crc16ccitt(const uint8_t *buf, uint8_t len) {
    uint16_t crc = 0;
    while (len--) {
        crc ^= (uint16_t)(*buf++) << 8;
        for (uint8_t i = 0; i < 8; i++)
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
    }
    return crc;
}

uint8_t reverseBits(uint8_t b) {
    b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
    b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
    b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
    return b;
}

void tuneTo(uint8_t i) {
    idx = i % 5;
    radio.standby();
    radio.setFrequency(euFreq[idx]);
    radio.startReceive();
}

void setup() {
    Serial.begin(115200);
    uint32_t t0 = millis();
    while (!Serial && millis() - t0 < 3000) {}

    int st = radio.beginFSK(euFreq[0], 19.2, 5.0, 25.0, 10, 4);
    if (st != RADIOLIB_ERR_NONE) {
        while (true) { Serial.print(F("Radio init failed, code ")); Serial.println(st); delay(2000); }
    }
    radio.setEncoding(RADIOLIB_ENCODING_NRZ);
    radio.setDataShaping(RADIOLIB_SHAPING_0_5);
    radio.setCRC(false);
    uint8_t syncWord[] = {0xCB, 0x89};
    radio.setSyncWord(syncWord, 2);
    radio.fixedPacketLengthMode(DAVIS_PACKET_LEN);
    radio.setPacketReceivedAction(onPacketReceived);
    tuneTo(0);

    Serial.println(F("[band] scanning Davis EU 868MHz channels, all transmitter IDs"));
}

void loop() {
    if (packetFlag) {
        packetFlag = false;
        uint8_t data[DAVIS_PACKET_LEN];
        int st = radio.readData(data, DAVIS_PACKET_LEN);
        float rssi = radio.getRSSI(true, true);

        if (st == RADIOLIB_ERR_NONE) {
            for (uint8_t i = 0; i < DAVIS_PACKET_LEN; i++) data[i] = reverseBits(data[i]);
            uint16_t crcRx = ((uint16_t)data[6] << 8) | data[7];
            if (crcRx == crc16ccitt(data, 6)) {
                hits++;
                Serial.print(F("[HIT] 868 ch")); Serial.print(idx);
                Serial.print(F(" ")); Serial.print(euFreq[idx], 6);
                Serial.print(F("MHz id=")); Serial.print(data[0] & 0x07);
                Serial.print(F(" rssi=")); Serial.print(rssi, 1);
                Serial.print(F("dBm type=0x")); Serial.println(data[0] >> 4, HEX);
            } else noise++;
        } else noise++;
        radio.startReceive();
    }

    static uint32_t lastHop = 0;
    if (millis() - lastHop > DWELL_MS) {
        lastHop = millis();
        tuneTo(idx + 1);
    }

    static uint32_t lastReport = 0;
    if (millis() - lastReport > 20000) {
        lastReport = millis();
        Serial.print(F("[band] hits=")); Serial.print(hits);
        Serial.print(F(" noise=")); Serial.println(noise);
    }
}
