// Davis Vantage Vue (ISS 6357) receiver for the Adafruit Feather RP2040 RFM95.
//
// The Vantage Vue ISS transmits continuously and independently of any
// console -- it needs no pairing step, it just needs power and to be within
// range. This sketch puts the RFM95's SX1276 radio into raw GFSK mode
// (not LoRa) and replicates the reverse-engineered Davis protocol: a
// 51-channel frequency-hopping sequence across 902-928MHz (North America),
// 19200bps GFSK, 10-byte fixed-length packets, CRC-16-CCITT.
//
// Protocol reference: https://github.com/dekay/DavisRFM69
//
// ---- Measured facts about this station (empirical, from this hardware) ----
//   * Transmissions sit on an exact 2562.5ms grid. Six independently measured
//     inter-packet gaps were all integer multiples of it to within 1ms, which
//     matches the documented Davis formula (41 + ID) / 16 seconds.
//   * The pattern repeats with a period of exactly 51 slots (~130.7s).
//   * Parked on ONE frequency, the station appears 3 times per 51-slot cycle
//     (at slots congruent to 0, 11 and 33 mod 51).
//
// That last point matters: 51 slots hold only 51 transmissions, so if one
// frequency takes 3 of them the station is cycling through just 17 distinct
// frequencies, not 51. The 51-entry table below is therefore NOT a hop
// sequence visited one-per-transmission -- which is why following it
// sequentially never holds a lock, no matter how the timing is tuned.
// Hence PARK_MODE. Deriving the real 17-frequency sequence would need a
// per-channel survey (~2 hours of unattended runtime).
//
// Radio: RadioLib (install via Arduino Library Manager)
// Board: Arduino-Pico core, "Adafruit Feather RP2040 RFM95"

#include <RadioLib.h>

// ---- Feather RP2040 RFM95 wiring (fixed on-board, from Adafruit's pinout) ----
#define RFM95_CS   16
#define RFM95_INT  21   // DIO0
#define RFM95_RST  17
#define RFM95_DIO1 22

SX1276 radio = new Module(RFM95_CS, RFM95_INT, RFM95_RST, RFM95_DIO1);

// Declared up front (not just above their first use) because the Arduino
// build system auto-generates function prototypes above these types --
// if they're declared later in the file, those prototypes fail to compile.
struct Reading {
    float windSpeedMph;
    int windDirDeg;
    uint8_t sensorType;
    bool batteryLow;
};

enum DecodeResult { CRC_FAIL, WRONG_ID, OK };
enum RxState { SEARCHING, TRACKING };

// ---- Your ISS's transmitter ID ----
// Set by the 3-bit DIP switch inside the ISS (switches 1-3; off=0, on=1,
// value = binary sum, range 0-7). Read the physical switch position and
// match it here.
#define TRANSMITTER_ID 0

#define DAVIS_PACKET_LEN 10
#define NUM_CHANNELS     51

// PARK_MODE: sit on one channel and listen continuously, rather than trying
// to follow the station's frequency hopping.
//
// This is deliberate, and measured. Parking on one channel was observed to
// receive this station 3 times per 51-slot cycle -- i.e. ~1 packet every
// 44s -- which is BETTER than chasing the hop sequence was managing, and it
// has no sync to lose. See the "measured facts" note at the top of the file
// for why following the hop sequence doesn't work with the assumed table.
//
// Any channel the station actually uses will work here. Channel 26
// (905.894592 MHz) is confirmed good; if you get nothing for ~3 minutes,
// try another index that has produced catches (0, 1, 2, 5, 7, 12, 18, 20,
// 21, 26, 30, 33, 34, 35, 45, 46 are all confirmed in use).
#define PARK_MODE     1
#define PARK_CHANNEL  26

// Channel frequencies (MHz), precomputed from the DavisRFM69 project's
// North America FRF register table using Fstep = 32MHz / 2^19 (the
// standard SX12xx frequency synthesizer step -- identical formula on the
// RFM69's SX1231 and this board's SX1276).
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

static uint8_t channel = 0;

// Retune and immediately go back to listening. The radio must spend
// essentially all its time in RX -- the ISS packet is only ~7ms long, so
// any window where we're in standby is a packet lost.
void setChannel(uint8_t ch) {
    channel = ch % NUM_CHANNELS;
    radio.standby();
    radio.setFrequency(channelFreqMHz[channel]);
    radio.startReceive();
}

void hop() {
    setChannel(channel + 1);
}

// CRC-16-CCITT (poly 0x1021, init 0), as used by Davis. Computed over the
// first 6 bytes; result compared against the CRC carried in bytes 6-7.
uint16_t crc16ccitt(const uint8_t *buf, uint8_t len) {
    uint16_t crc = 0;
    while (len--) {
        crc ^= (uint16_t)(*buf++) << 8;
        for (uint8_t i = 0; i < 8; i++) {
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
        }
    }
    return crc;
}

// Davis transmits each byte LSB-first; the radio's FIFO shifts bits out
// MSB-first, so every received byte must be bit-reversed before use.
uint8_t reverseBits(uint8_t b) {
    b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
    b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
    b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
    return b;
}

DecodeResult decodePacket(const uint8_t *data, Reading &r, uint8_t &seenId) {
    uint16_t crcRx = ((uint16_t)data[6] << 8) | data[7];
    uint16_t crcCalc = crc16ccitt(data, 6);
    if (crcRx != crcCalc) return CRC_FAIL;

    r.sensorType = data[0] >> 4;
    seenId = data[0] & 0x07;
    r.batteryLow = (data[0] & 0x08) != 0;
    if (seenId != TRANSMITTER_ID) return WRONG_ID;

    r.windSpeedMph = data[1];
    uint8_t dirRaw = data[2];
    r.windDirDeg = (dirRaw == 0) ? 0 : (int)((dirRaw - 1) * 360.0f / 255.0f);
    return OK;
}

void printReading(const uint8_t *data, const Reading &r) {
    Serial.print(F("ch=")); Serial.print(channel);
    Serial.print(F(" type=0x")); Serial.print(r.sensorType, HEX);
    Serial.print(F(" batt=")); Serial.print(r.batteryLow ? F("LOW") : F("OK"));
    Serial.print(F(" wind=")); Serial.print(r.windSpeedMph, 0);
    Serial.print(F("mph@")); Serial.print(r.windDirDeg);

    switch (r.sensorType) {
        case 0x8: { // temperature
            int16_t raw = (int16_t)(((uint16_t)data[3] << 8) | data[4]);
            Serial.print(F(" temp=")); Serial.print(raw / 160.0f, 1); Serial.print(F("F"));
            break;
        }
        case 0xA: { // humidity
            uint16_t raw = (((uint16_t)data[4] >> 4) << 8) | data[3];
            Serial.print(F(" rh=")); Serial.print(raw / 10.0f, 1); Serial.print(F("%"));
            break;
        }
        case 0x4: { // UV index
            uint16_t raw = (((uint16_t)data[3] << 8) | data[4]) >> 6;
            Serial.print(F(" uv=")); Serial.print(raw / 50.0f, 1);
            break;
        }
        case 0x6: { // solar radiation
            uint16_t raw = (((uint16_t)data[3] << 8) | data[4]) >> 6;
            Serial.print(F(" solar=")); Serial.print(raw * 1.757936f, 0); Serial.print(F("W/m2"));
            break;
        }
        case 0xE: // rain: cumulative bucket-tip counter (mod 256); derive
                  // rate/accumulation yourself by tracking deltas over time
            Serial.print(F(" rainCount=")); Serial.print(data[3]);
            break;
        case 0x5: // rain rate -- scaling not confidently verified, raw byte only
            Serial.print(F(" rainRateRaw=")); Serial.print(data[3]);
            break;
        case 0x9: // wind gust -- scaling not confidently verified, raw byte only
            Serial.print(F(" gustRaw=")); Serial.print(data[3]);
            break;
        case 0x2: // supercap voltage (solar panel/ISS health)
            Serial.print(F(" supercapRaw=")); Serial.print(data[3]);
            break;
        default:
            break;
    }
    Serial.println();
}

// Time between transmissions. The DavisRFM69 reference hardcodes 2555ms,
// but this station measured ~2584ms -- and being even 30ms short makes us
// arrive early on every hop, missing until the accumulated slack catches
// up. So this is only a starting estimate; it's refined at runtime from
// observed packet timing (see calibrateInterval).
uint32_t packetInterval = 2584 + (uint32_t)TRANSMITTER_ID * 62;

// Slop allowed past the expected arrival before we give up on a slot.
const uint32_t HOP_SLOP_MS = 400;

// Previous confirmed packet, used to measure the real interval.
uint32_t lastGoodRxTime = 0;
uint8_t lastGoodChannel = 0;

// hopCount == 0 means "lost": stop chasing and park on one channel until
// the station's rotation comes back around to us. Otherwise it's the number
// of consecutive slots we've been waiting, which widens the deadline so we
// stay lined up with the station's schedule while missing packets.
uint8_t hopCount = 0;
uint32_t lastRxTime = 0;
const uint8_t MAX_MISSES = 25;

// Set by the DIO0 interrupt when a full packet has landed in the FIFO.
volatile bool packetFlag = false;
void onPacketReceived() { packetFlag = true; }

// Diagnostics: noise regularly fakes the sync word, so track how much of
// what we pick up is junk vs. real.
uint32_t falseTriggers = 0;
uint32_t otherStation = 0;
uint32_t goodPackets = 0;

// Derive the true transmit interval from two confirmed packets: the channel
// delta says how many slots elapsed, the clock says how long that took.
// Self-correcting, so we don't stay hostage to a hardcoded constant.
void calibrateInterval(uint8_t rxChannel) {
    uint32_t now = millis();
    if (lastGoodRxTime != 0) {
        uint8_t dch = (rxChannel + NUM_CHANNELS - lastGoodChannel) % NUM_CHANNELS;
        uint32_t dt = now - lastGoodRxTime;
        if (dch > 0) {
            uint32_t est = dt / dch;
            // Reject samples that span a full wrap of the hop table, which
            // would make the slot count ambiguous.
            if (est > 2200 && est < 3000) {
                packetInterval = (packetInterval * 3 + est) / 4;   // smooth
                Serial.print(F("[cal] interval="));
                Serial.print(packetInterval);
                Serial.print(F("ms (sample "));
                Serial.print(est);
                Serial.println(F("ms)"));
            }
        }
    }
    lastGoodRxTime = now;
    lastGoodChannel = rxChannel;
}

void setup() {
    Serial.begin(115200);
    uint32_t t0 = millis();
    while (!Serial && millis() - t0 < 3000) {}

    Serial.println(F("[boot] Serial up, initializing radio..."));

    // beginFSK(freq, bitrate_kbps, freqDev_kHz, rxBandwidth_kHz, power_dBm, preambleLen)
    int st = radio.beginFSK(channelFreqMHz[0], 19.2, 5.0, 25.0, 10, 4);
    if (st != RADIOLIB_ERR_NONE) {
        while (true) {
            Serial.print(F("Radio init failed, code "));
            Serial.println(st);
            delay(2000);
        }
    }

    radio.setEncoding(RADIOLIB_ENCODING_NRZ);
    radio.setDataShaping(RADIOLIB_SHAPING_0_5);   // Gaussian BT=0.5, matches Davis GFSK
    radio.setCRC(false);                           // Davis CRC is checked in software
    uint8_t syncWord[] = {0xCB, 0x89};
    radio.setSyncWord(syncWord, 2);
    radio.fixedPacketLengthMode(DAVIS_PACKET_LEN);
    radio.setPacketReceivedAction(onPacketReceived);

#if PARK_MODE
    setChannel(PARK_CHANNEL);
    Serial.print(F("[park] measurement mode, parked on ch="));
    Serial.println(PARK_CHANNEL);
#else
    setChannel(0);   // also arms continuous receive
#endif
    lastRxTime = millis();
    Serial.println(F("Davis Vantage Vue ISS receiver starting, waiting for signal..."));
}

void loop() {
    // The radio is listening continuously; this just checks whether the
    // DIO0 interrupt has handed us a packet since we last looked.
    if (packetFlag) {
        packetFlag = false;

        uint8_t data[DAVIS_PACKET_LEN];
        int st = radio.readData(data, DAVIS_PACKET_LEN);
        bool ours = false;

        if (st == RADIOLIB_ERR_NONE) {
            for (uint8_t i = 0; i < DAVIS_PACKET_LEN; i++) data[i] = reverseBits(data[i]);

            Reading r;
            uint8_t seenId;
            DecodeResult res = decodePacket(data, r, seenId);

            if (res == OK) {
                ours = true;
                goodPackets++;
                hopCount = 1;      // locked on
                calibrateInterval(channel);
                printReading(data, r);
            } else if (res == WRONG_ID) {
                otherStation++;
            } else {
                falseTriggers++;
            }
        } else {
            falseTriggers++;
        }

#if PARK_MODE
        if (ours) {
            static uint32_t prevPark = 0;
            uint32_t now = millis();
            Serial.print(F("[park] hit at t="));
            Serial.print(now);
            if (prevPark) { Serial.print(F("ms  delta=")); Serial.print(now - prevPark); Serial.print(F("ms")); }
            Serial.println();
            prevPark = now;
        }
        radio.startReceive();
#else
        if (ours) {
            // Only a CRC-verified packet from OUR station is a trustworthy
            // timing anchor. Random noise fakes the 2-byte sync word roughly
            // every few seconds at this bit rate; anchoring on those would
            // corrupt the schedule and destroy the lock (which is exactly
            // what was happening before).
            lastRxTime = millis();
            hop();
        } else {
            radio.startReceive();   // re-arm, stay put
        }
#endif
    }

    // Expected packet never showed up. Hop anyway to stay lined up with the
    // station's schedule, widening the deadline by one slot each time so we
    // track where it *should* be. After MAX_MISSES, give up chasing and park
    // (hopCount = 0) -- the station's rotation will come back to us.
#if !PARK_MODE
    if (hopCount > 0 && (millis() - lastRxTime) > (hopCount * packetInterval + HOP_SLOP_MS)) {
        if (++hopCount > MAX_MISSES) {
            hopCount = 0;
            Serial.print(F("[lost] sync lost, parking on ch="));
            Serial.println(channel);
        }
        hop();
    }
#endif

    static uint32_t lastHeartbeat = 0;
    if (millis() - lastHeartbeat > 10000) {
        lastHeartbeat = millis();
        Serial.print(F("[alive] hopCount="));
        Serial.print(hopCount);
        Serial.print(F(" ch="));
        Serial.print(channel);
        Serial.print(F(" sinceRx="));
        Serial.print(millis() - lastRxTime);
        Serial.print(F("ms good="));
        Serial.print(goodPackets);
        Serial.print(F(" noise="));
        Serial.print(falseTriggers);
        Serial.print(F(" otherStn="));
        Serial.println(otherStation);
    }
}
