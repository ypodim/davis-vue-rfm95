// Davis ISS hop-sequence survey for the Adafruit Feather RP2040 RFM95.
//
// Purpose: recover the station's actual transmit schedule, so the receiver
// can follow it and catch every packet (~1 per 2.6s) instead of parking on
// one frequency (~1 per 44s).
//
// What we already measured on this hardware:
//   * Transmissions sit on an exact 2562.5ms grid ((41 + ID)/16 seconds).
//   * The whole pattern repeats every exactly 51 slots (~130.7s).
//   * Parked on one frequency, the station shows up 3 times per cycle.
//
// 51 slots hold 51 transmissions, so if each used frequency takes 3 of them
// the station is cycling ~17 distinct frequencies, not 51. The 51-entry
// table below is therefore a frequency POOL, not a hop order.
//
// So the schedule is fully described by a map slot -> frequency, for the 51
// slots in one cycle. This sketch fills that map in.
//
// Method: rather than parking per channel (51 x ~2min = 2 hours), test one
// candidate per SLOT -- 51 experiments per cycle instead of 1. A CRC-verified
// hit positively identifies that slot's frequency; a miss just moves on.
// Two constraints make it converge fast:
//   * A channel found in 3 slots is saturated and gets eliminated.
//   * Channels already seen in use are tried before untested ones.

#include <RadioLib.h>

#define RFM95_CS   16
#define RFM95_INT  21   // DIO0
#define RFM95_RST  17
#define RFM95_DIO1 22

SX1276 radio = new Module(RFM95_CS, RFM95_INT, RFM95_RST, RFM95_DIO1);

#define TRANSMITTER_ID   0
#define DAVIS_PACKET_LEN 10
#define NUM_CHANNELS     51
#define SLOTS_PER_CYCLE  51
#define SLOTS_PER_CHAN   3     // each used frequency occupies 3 slots/cycle

// Slot grid: 2562.5ms. Kept in halves-of-a-ms so it stays integer math.
#define SLOT_HALFMS      5125ULL

// Hop to the next slot's channel this early, so we're already listening
// when the transmission lands.
#define PRE_SLOT_MS      150

// Anchor channel: confirmed in use, used to establish the slot phase.
#define ANCHOR_CHANNEL   26

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

// Channels that have already produced confirmed catches in earlier runs --
// very likely 16 of the ~17 actually in use, so try these first.
static const uint8_t candidateOrder[NUM_CHANNELS] = {
    0, 1, 2, 5, 7, 12, 18, 20, 21, 26, 30, 33, 34, 35, 45, 46,
    3, 4, 6, 8, 9, 10, 11, 13, 14, 15, 16, 17, 19, 22, 23, 24, 25,
    27, 28, 29, 31, 32, 36, 37, 38, 39, 40, 41, 42, 43, 44, 47, 48, 49, 50
};

int8_t  slotFreq[SLOTS_PER_CYCLE];        // -1 = unknown
uint8_t useCount[NUM_CHANNELS];           // slots assigned to each channel
uint8_t tried[SLOTS_PER_CYCLE][7];        // per-slot bitmap of tested channels
uint8_t knownSlots = 0;

uint32_t epochMs = 0;
bool     haveAnchor = false;
uint64_t curSlot = 0;
uint8_t  channel = 0;

volatile bool packetFlag = false;
void onPacketReceived() { packetFlag = true; }

uint32_t goodPackets = 0, falseTriggers = 0;

static inline bool wasTried(uint8_t res, uint8_t ch) {
    return tried[res][ch >> 3] & (1 << (ch & 7));
}
static inline void markTried(uint8_t res, uint8_t ch) {
    tried[res][ch >> 3] |= (1 << (ch & 7));
}
static inline void clearTried(uint8_t res) {
    for (uint8_t i = 0; i < 7; i++) tried[res][i] = 0;
}

void setChannel(uint8_t ch) {
    channel = ch % NUM_CHANNELS;
    radio.standby();
    radio.setFrequency(channelFreqMHz[channel]);
    radio.startReceive();
}

uint64_t slotOf(uint32_t t)      { return ((uint64_t)(t - epochMs) * 2ULL) / SLOT_HALFMS; }
uint32_t slotStart(uint64_t n)   { return epochMs + (uint32_t)((n * SLOT_HALFMS) / 2ULL); }

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

// True only for a CRC-valid packet from our own transmitter ID.
bool decodeOurs(uint8_t *data) {
    for (uint8_t i = 0; i < DAVIS_PACKET_LEN; i++) data[i] = reverseBits(data[i]);
    uint16_t crcRx = ((uint16_t)data[6] << 8) | data[7];
    if (crcRx != crc16ccitt(data, 6)) return false;
    return (data[0] & 0x07) == TRANSMITTER_ID;
}

// Pick the next channel worth testing for an unidentified slot.
// Priority: channels already seen in use but not yet saturated, then
// untested ones in candidateOrder. Saturated channels (3 slots) are skipped
// entirely -- they cannot appear again.
uint8_t pickCandidate(uint8_t res) {
    for (uint8_t pass = 0; pass < 3; pass++) {
        for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
            uint8_t c = candidateOrder[i];
            if (useCount[c] >= SLOTS_PER_CHAN) continue;
            if (wasTried(res, c)) continue;
            if (pass == 0 && useCount[c] == 0) continue;   // prefer known-in-use
            return c;
        }
        if (pass == 1) clearTried(res);   // exhausted: start a fresh sweep
    }
    return ANCHOR_CHANNEL;
}

void dumpMap() {
    Serial.println(F("=== SLOT MAP ==="));
    for (uint8_t s = 0; s < SLOTS_PER_CYCLE; s++) {
        Serial.print(F("slot ")); Serial.print(s);
        Serial.print(F(" -> ch "));
        if (slotFreq[s] < 0) Serial.println(F("?"));
        else { Serial.print(slotFreq[s]); Serial.print(F("  ")); Serial.println(channelFreqMHz[slotFreq[s]], 6); }
    }
    Serial.print(F("channels in use:"));
    for (uint8_t c = 0; c < NUM_CHANNELS; c++)
        if (useCount[c]) { Serial.print(' '); Serial.print(c); Serial.print('x'); Serial.print(useCount[c]); }
    Serial.println();
    Serial.print(F("=== identified ")); Serial.print(knownSlots);
    Serial.println(F("/51 ==="));
}

void setup() {
    Serial.begin(115200);
    uint32_t t0 = millis();
    while (!Serial && millis() - t0 < 3000) {}

    for (uint8_t i = 0; i < SLOTS_PER_CYCLE; i++) { slotFreq[i] = -1; clearTried(i); }
    for (uint8_t i = 0; i < NUM_CHANNELS; i++) useCount[i] = 0;

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
    radio.setPacketReceivedAction(onPacketReceived);

    setChannel(ANCHOR_CHANNEL);
    Serial.println(F("[survey] waiting on anchor channel to establish slot phase..."));
}

void loop() {
    uint32_t now = millis();

    if (packetFlag) {
        packetFlag = false;
        uint8_t data[DAVIS_PACKET_LEN];
        int st = radio.readData(data, DAVIS_PACKET_LEN);

        if (st == RADIOLIB_ERR_NONE && decodeOurs(data)) {
            goodPackets++;

            if (!haveAnchor) {
                // First confirmed packet defines slot 0 of the grid.
                epochMs = now;
                curSlot = 0;
                haveAnchor = true;
                slotFreq[0] = ANCHOR_CHANNEL;
                useCount[ANCHOR_CHANNEL]++;
                knownSlots = 1;
                Serial.println(F("[survey] anchored. slot 0 -> ch 26. starting sweep."));
            } else {
                uint8_t res = (uint8_t)(curSlot % SLOTS_PER_CYCLE);
                if (slotFreq[res] < 0) {
                    slotFreq[res] = channel;
                    useCount[channel]++;
                    knownSlots++;
                    Serial.print(F("[found] slot ")); Serial.print(res);
                    Serial.print(F(" -> ch ")); Serial.print(channel);
                    Serial.print(F("   (")); Serial.print(knownSlots); Serial.println(F("/51)"));
                    if (knownSlots == SLOTS_PER_CYCLE) dumpMap();
                }
                // Re-anchor gently: keep our grid aligned with the station's
                // despite clock drift, without letting one noisy sample jerk it.
                int32_t drift = (int32_t)(now - slotStart(curSlot));
                if (drift > -500 && drift < 500) epochMs += drift / 4;
            }
        } else {
            falseTriggers++;
            radio.startReceive();
        }
    }

    if (!haveAnchor) return;

    // Prepare the next slot slightly before it begins.
    if ((int32_t)(now - (slotStart(curSlot + 1) - PRE_SLOT_MS)) >= 0) {
        curSlot++;
        uint8_t res = (uint8_t)(curSlot % SLOTS_PER_CYCLE);
        uint8_t ch;
        if (slotFreq[res] >= 0) {
            ch = slotFreq[res];          // known: listen to keep sync
        } else {
            ch = pickCandidate(res);
            markTried(res, ch);
        }
        setChannel(ch);
    }

    static uint32_t lastReport = 0;
    if (now - lastReport > 30000) {
        lastReport = now;
        Serial.print(F("[survey] identified=")); Serial.print(knownSlots);
        Serial.print(F("/51 cycle=")); Serial.print((uint32_t)(curSlot / SLOTS_PER_CYCLE));
        Serial.print(F(" good=")); Serial.print(goodPackets);
        Serial.print(F(" noise=")); Serial.println(falseTriggers);
    }
}
