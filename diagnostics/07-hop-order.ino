// Determine the ISS hop ORDER, cheaply.
//
// Known (measured): transmissions sit on an exact 2750ms grid for wire ID 3,
// and the station returns to any given channel exactly every 51 slots. So
// each channel has a fixed "slot residue" within the 51-slot cycle.
//
// Unknown: the order channels are visited in. Stepping +1 through the
// DavisRFM69 table -- what the reference implementation does -- was tested
// and does NOT work here.
//
// Rather than brute-force a 51-element permutation (~2 hours of probing),
// measure the residue of a handful of channels and look for the rule. If
// residue is linear in channel index, a few samples give the whole map.
//
// Method: anchor a slot clock on the first strong packet, then park on each
// target channel for a bit over one full cycle -- which guarantees the
// station passes through it exactly once -- and record the slot it landed on.
//
// Only strong packets count: with the ISS nearby it bleeds through the front
// end on channels it isn't using, ~40dB down. Those decode fine but are not
// real visits, and would produce a garbage map.

#include <RadioLib.h>

#define RFM95_CS   16
#define RFM95_INT  21
#define RFM95_RST  17
#define RFM95_DIO1 22

SX1276 radio = new Module(RFM95_CS, RFM95_INT, RFM95_RST, RFM95_DIO1);

#define TRANSMITTER_ID   3
#define DAVIS_PACKET_LEN 10
#define NUM_CHANNELS     51
#define SLOTS_PER_CYCLE  51
#define SLOT_MS          2750UL     // (41 + 3)/16 s
#define RSSI_FLOOR       -95.0f
#define ANCHOR_CHANNEL   26
#define DWELL_SLOTS      56         // > one full cycle, so a visit is guaranteed

// Channels to measure, chosen to expose a linear rule quickly if one exists.
static const uint8_t targets[] = {27, 25, 28, 24, 30, 0, 40, 13};
#define NTARGETS (sizeof(targets)/sizeof(targets[0]))

static const float channelFreqMHz[NUM_CHANNELS] = {
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

uint8_t  channel = ANCHOR_CHANNEL;
bool     anchored = false;
uint32_t epochMs = 0;          // time of slot 0
uint8_t  tIdx = 0;             // which target we're measuring
uint32_t dwellStart = 0;
bool     gotThisTarget = false;
uint32_t good = 0, bleed = 0;

volatile bool packetFlag = false;
void onPacketReceived() { packetFlag = true; }

void setChannel(uint8_t ch) {
    channel = ch % NUM_CHANNELS;
    radio.standby();
    radio.setFrequency(channelFreqMHz[channel]);
    radio.startReceive();
}

uint32_t slotOf(uint32_t t) { return (t - epochMs) / SLOT_MS; }

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

bool decodeOurs(uint8_t *data) {
    for (uint8_t i = 0; i < DAVIS_PACKET_LEN; i++) data[i] = reverseBits(data[i]);
    uint16_t crcRx = ((uint16_t)data[6] << 8) | data[7];
    if (crcRx != crc16ccitt(data, 6)) return false;
    return (data[0] & 0x07) == TRANSMITTER_ID;
}

void startTarget() {
    if (tIdx >= NTARGETS) {
        Serial.println(F("[order] all targets measured."));
        return;
    }
    gotThisTarget = false;
    dwellStart = millis();
    setChannel(targets[tIdx]);
    Serial.print(F("[order] measuring ch ")); Serial.print(targets[tIdx]);
    Serial.print(F(" (")); Serial.print(tIdx + 1); Serial.print('/');
    Serial.print(NTARGETS); Serial.println(F(") ..."));
}

void setup() {
    Serial.begin(115200);
    uint32_t t0 = millis();
    while (!Serial && millis() - t0 < 3000) {}

    int st = radio.beginFSK(channelFreqMHz[ANCHOR_CHANNEL], 19.2, 5.0, 25.0, 10, 4);
    if (st != RADIOLIB_ERR_NONE) {
        while (true) { Serial.print(F("Radio init failed, code ")); Serial.println(st); delay(2000); }
    }
    radio.setEncoding(RADIOLIB_ENCODING_NRZ);
    radio.setDataShaping(RADIOLIB_SHAPING_0_5);
    radio.setCRC(false);
    uint8_t syncWord[] = {0xCB, 0x89};
    radio.setSyncWord(syncWord, 2);
    radio.fixedPacketLengthMode(DAVIS_PACKET_LEN);
    radio.setAFC(true);
    radio.setAFCBandwidth(50.0);
    radio.setAFCAGCTrigger(RADIOLIB_SX127X_RX_TRIGGER_PREAMBLE_DETECT);
    radio.setPacketReceivedAction(onPacketReceived);

    setChannel(ANCHOR_CHANNEL);
    Serial.println(F("[order] anchoring on ch 26, waiting for a strong packet..."));
}

void loop() {
    uint32_t now = millis();

    if (packetFlag) {
        packetFlag = false;
        uint8_t data[DAVIS_PACKET_LEN];
        int st = radio.readData(data, DAVIS_PACKET_LEN);
        float rssi = radio.getRSSI(true, true);

        if (st == RADIOLIB_ERR_NONE && decodeOurs(data)) {
            if (rssi >= RSSI_FLOOR) {
                good++;
                if (!anchored) {
                    epochMs = now;
                    anchored = true;
                    Serial.print(F("[order] anchored: ch 26 = slot 0, rssi "));
                    Serial.println(rssi, 0);
                    tIdx = 0;
                    startTarget();
                } else if (!gotThisTarget) {
                    gotThisTarget = true;
                    uint32_t s = slotOf(now);
                    Serial.print(F("[RESULT] ch ")); Serial.print(channel);
                    Serial.print(F(" -> slot ")); Serial.print(s % SLOTS_PER_CYCLE);
                    Serial.print(F("  (abs ")); Serial.print(s);
                    Serial.print(F(", rssi ")); Serial.print(rssi, 0);
                    Serial.println(F(")"));
                    tIdx++;
                    startTarget();
                }
            } else bleed++;
        }
        radio.startReceive();
    }

    // Give up on a target after more than a full cycle and move on, so one
    // bad channel can't stall the whole measurement.
    if (anchored && !gotThisTarget && tIdx < NTARGETS &&
        (now - dwellStart) > (uint32_t)DWELL_SLOTS * SLOT_MS) {
        Serial.print(F("[RESULT] ch ")); Serial.print(targets[tIdx]);
        Serial.println(F(" -> NO VISIT (not in hop set?)"));
        tIdx++;
        startTarget();
    }

    static uint32_t lastReport = 0;
    if (now - lastReport > 30000) {
        lastReport = now;
        Serial.print(F("[order] good=")); Serial.print(good);
        Serial.print(F(" bleed=")); Serial.println(bleed);
    }
}
