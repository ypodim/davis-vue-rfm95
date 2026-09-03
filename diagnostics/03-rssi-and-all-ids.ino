// Image-response diagnostic.
//
// Question: when we hear the station on a given table channel, is that the
// frequency it actually transmitted on, or an image response from the
// receiver's low-IF architecture landing on a neighbouring table entry?
// Davis channels here are spaced ~500.9 kHz, which is close to twice a
// typical SX127x IF -- so an image can land almost exactly on the adjacent
// channel and masquerade as a real hit.
//
// Test: park on one channel and log RSSI for every CRC-verified packet.
// A true-frequency reception should be markedly stronger than an image.
// If hits split into a strong cluster and weak cluster(s), images are real,
// and the "3 hits per cycle" that the 17-channel theory was built on is an
// artefact rather than the station visiting this frequency 3 times.

#include <RadioLib.h>

#define RFM95_CS   16
#define RFM95_INT  21
#define RFM95_RST  17
#define RFM95_DIO1 22

SX1276 radio = new Module(RFM95_CS, RFM95_INT, RFM95_RST, RFM95_DIO1);

#define TRANSMITTER_ID   0
#define DAVIS_PACKET_LEN 10
#define PARK_CHANNEL     26

static const float channelFreqMHz[51] = {
    911.413818f, 902.381897f, 911.915161f, 922.953186f, 914.926575f, 906.395874f,
    925.964600f, 918.438354f, 908.904663f, 920.445374f, 913.420410f, 903.888062f,
    916.933533f, 924.458923f, 910.409912f, 904.890625f, 915.929138f, 921.448364f,
    907.399414f, 926.967651f, 912.919067f, 903.385376f, 917.434387f, 923.456299f,
    909.406860f, 926.466370f, 905.894592f, 914.424316f, 919.441406f, 924.960205f,
    902.884521f, 910.912109f, 921.949707f, 915.427856f, 906.898071f, 917.935669f,
    927.469849f, 920.947083f, 908.402893f, 912.417358f, 918.940125f, 904.389343f,
    923.957153f, 916.431824f, 909.909058f, 919.943604f, 905.391968f, 922.451904f,
    907.901123f, 913.922607f, 925.462402f,
};

volatile bool packetFlag = false;
void onPacketReceived() { packetFlag = true; }
uint32_t goodPackets = 0, falseTriggers = 0, lastHit = 0;

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

void setup() {
    Serial.begin(115200);
    uint32_t t0 = millis();
    while (!Serial && millis() - t0 < 3000) {}

    int st = radio.beginFSK(channelFreqMHz[PARK_CHANNEL], 19.2, 5.0, 25.0, 10, 4);
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
    radio.startReceive();

    Serial.print(F("[rssi] parked on ch ")); Serial.print(PARK_CHANNEL);
    Serial.print(F(" = ")); Serial.print(channelFreqMHz[PARK_CHANNEL], 6);
    Serial.println(F(" MHz"));
}

void loop() {
    if (packetFlag) {
        packetFlag = false;
        uint8_t data[DAVIS_PACKET_LEN];
        int st = radio.readData(data, DAVIS_PACKET_LEN);
        float rssi = radio.getRSSI(true, true);   // packet RSSI, don't re-enter RX

        if (st == RADIOLIB_ERR_NONE) {
            for (uint8_t i = 0; i < DAVIS_PACKET_LEN; i++) data[i] = reverseBits(data[i]);
            uint16_t crcRx = ((uint16_t)data[6] << 8) | data[7];
            // Accept EVERY transmitter ID -- we're identifying which one is
            // ours. A station on the same desk should be enormously stronger
            // than a distant neighbour's.
            if (crcRx == crc16ccitt(data, 6)) {
                goodPackets++;
                uint32_t now = millis();
                Serial.print(F("[hit] id=")); Serial.print(data[0] & 0x07);
                Serial.print(F(" rssi=")); Serial.print(rssi, 1);
                Serial.print(F("dBm type=0x")); Serial.print(data[0] >> 4, HEX);
                Serial.print(F(" gap=")); Serial.print(lastHit ? now - lastHit : 0);
                Serial.print(F("ms  temp/raw=")); Serial.print(data[3]);
                Serial.print(','); Serial.println(data[4]);
                lastHit = now;
            } else {
                falseTriggers++;
            }
        } else {
            falseTriggers++;
        }
        radio.startReceive();
    }

    static uint32_t lastReport = 0;
    if (millis() - lastReport > 30000) {
        lastReport = millis();
        Serial.print(F("[rssi] good=")); Serial.print(goodPackets);
        Serial.print(F(" noise=")); Serial.println(falseTriggers);
    }
}
