// Davis Vantage Vue ISS receiver -- schedule-following.
//
// Catches a packet every 2.75s instead of every ~24s parked.
//
// Three things had to be right, and each was wrong at some point:
//   1. AFC must be ON. The station's crystal offset puts it outside a
//      25kHz window; without AFC it simply never demodulates. (The
//      DavisRFM69 reference config enables it; an earlier port dropped it.)
//   2. The slot interval is ID-dependent: (41 + wireID)/16 seconds.
//      For wire ID 3 that's 2750ms, not the 2562.5ms of ID 0.
//   3. Only STRONG receptions are real. With the ISS a short distance away
//      it bleeds through the front end on channels it isn't using, showing
//      up as packets 40dB down. Those are artifacts -- anchoring the
//      schedule on them corrupts it. Hence RSSI_FLOOR.
//
// Structure (measured): the station steps one channel per slot through the
// 51-entry table, visiting each once per 51-slot cycle. So absolute slot
// numbering is unnecessary -- hearing it on channel c means the next
// transmission is on c+1.

#include <RadioLib.h>
#include <Adafruit_NeoPixel.h>

#define RFM95_CS   16
#define RFM95_INT  21
#define RFM95_RST  17
#define RFM95_DIO1 22

SX1276 radio = new Module(RFM95_CS, RFM95_INT, RFM95_RST, RFM95_DIO1);

// Wire-level transmitter ID = (ID shown on the ISS LED) - 1.
#define TRANSMITTER_ID   3
#define DAVIS_PACKET_LEN 10
#define NUM_CHANNELS     51

// Slot interval for this ID: (41 + 3)/16 s = 2750ms. Kept in halves of a ms.
#define SLOT_HALFMS      5500ULL

#define PRE_SLOT_MS      120     // retune this early so we're already listening
#define RSSI_FLOOR      -120.0f  // below this it is front-end bleed, not a real visit
#define ANCHOR_CHANNEL   26
#define MAX_MISSES       30      // consecutive misses before re-acquiring

// Status indication. This board has two LEDs: a plain red one on GPIO13
// (PIN_LED) and a NeoPixel RGB on GPIO4 (PIN_NEOPIXEL). The NeoPixel is used
// as the primary indicator because colour can encode STATE, not just
// activity, which is what you actually want on a headless Pi:
//
//     green flash  = packet received and decoded (one per ~2.75s when healthy)
//     dim blue     = running but not locked -- searching for the station
//     red          = radio init failed
//
// Both are driven from timestamps, never delay(). Blocking here would push
// the slot timer late and break the hop tracking everything depends on.
// NeoPixel show() briefly disables interrupts, but for a single pixel that
// is ~30us -- far too short to disturb a 2750ms slot.
#define LED_BLINK_MS     60
#define PIXEL_BRIGHTNESS 30      // out of 255; these are very bright

struct Reading {
    float windSpeedMph;
    int windDirDeg;
    uint8_t sensorType;
    bool batteryLow;
};

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
bool     locked = false;
uint32_t nextSlotMs = 0;
uint16_t missStreak = 0;
uint32_t good = 0, weak = 0, junk = 0;

volatile bool packetFlag = false;
void onPacketReceived() { packetFlag = true; }

uint32_t ledOffAtMs = 0;   // 0 = LED idle
Adafruit_NeoPixel pixel(1, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

void pixelSet(uint8_t r, uint8_t g, uint8_t b) {
    pixel.setPixelColor(0, pixel.Color(r, g, b));
    pixel.show();
}

void setChannel(uint8_t ch) {
    channel = ch % NUM_CHANNELS;
    radio.standby();
    radio.setFrequency(channelFreqMHz[channel]);
    radio.startReceive();
}

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

bool decodeOurs(uint8_t *data, Reading &r) {
    for (uint8_t i = 0; i < DAVIS_PACKET_LEN; i++) data[i] = reverseBits(data[i]);
    uint16_t crcRx = ((uint16_t)data[6] << 8) | data[7];
    if (crcRx != crc16ccitt(data, 6)) return false;
    if ((data[0] & 0x07) != TRANSMITTER_ID) return false;
    r.sensorType = data[0] >> 4;
    r.batteryLow = (data[0] & 0x08) != 0;
    r.windSpeedMph = data[1];
    uint8_t dirRaw = data[2];
    r.windDirDeg = (dirRaw == 0) ? 0 : (int)((dirRaw - 1) * 360.0f / 255.0f);
    return true;
}

// One JSON object per line, so a host-side driver can parse without regex.
// Wind speed/direction ride along in every packet; the remaining field
// depends on the sensor type in this transmission.
//
// Unverified scalings (rain rate, gust) are emitted as raw bytes under
// explicitly "_raw" names, so nothing downstream mistakes them for
// calibrated values and publishes them.
void printReading(const uint8_t *data, const Reading &r, float rssi) {
    Serial.print(F("{\"type\":")); Serial.print(r.sensorType);
    Serial.print(F(",\"ch\":"));   Serial.print(channel);
    Serial.print(F(",\"rssi\":")); Serial.print(rssi, 0);
    Serial.print(F(",\"battLow\":")); Serial.print(r.batteryLow ? 1 : 0);
    Serial.print(F(",\"windSpeedMph\":")); Serial.print(r.windSpeedMph, 0);
    Serial.print(F(",\"windDirDeg\":"));   Serial.print(r.windDirDeg);

    switch (r.sensorType) {
        case 0x8: { int16_t raw = (int16_t)(((uint16_t)data[3] << 8) | data[4]);
                    Serial.print(F(",\"tempF\":")); Serial.print(raw / 160.0f, 1); break; }
        case 0xA: { uint16_t raw = (((uint16_t)data[4] >> 4) << 8) | data[3];
                    Serial.print(F(",\"humidity\":")); Serial.print(raw / 10.0f, 1); break; }
        case 0x4: { uint16_t raw = (((uint16_t)data[3] << 8) | data[4]) >> 6;
                    Serial.print(F(",\"uv\":")); Serial.print(raw / 50.0f, 1); break; }
        case 0x6: { uint16_t raw = (((uint16_t)data[3] << 8) | data[4]) >> 6;
                    Serial.print(F(",\"solarWm2\":")); Serial.print(raw * 1.757936f, 0); break; }
        case 0xE:   Serial.print(F(",\"rainCount\":"));    Serial.print(data[3]); break;
        case 0x5:   Serial.print(F(",\"rainRate_raw\":")); Serial.print(data[3]); break;
        case 0x9:   Serial.print(F(",\"gust_raw\":"));     Serial.print(data[3]); break;
        case 0x2:   Serial.print(F(",\"supercap_raw\":")); Serial.print(data[3]); break;
        default: break;
    }
    Serial.println('}');
}

void setup() {
    Serial.begin(115200);
    pixel.begin();
    pixel.setBrightness(PIXEL_BRIGHTNESS);
    pixelSet(0, 0, 20);            // dim blue: alive, not yet locked

    uint32_t t0 = millis();
    while (!Serial && millis() - t0 < 3000) {}

    int st = radio.beginFSK(channelFreqMHz[ANCHOR_CHANNEL], 19.2, 5.0, 25.0, 10, 4);
    if (st != RADIOLIB_ERR_NONE) {
        pixelSet(60, 0, 0);        // red: radio init failed
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

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    setChannel(ANCHOR_CHANNEL);
    Serial.println(F("# waiting for a strong packet to lock onto..."));
}

void loop() {
    uint32_t now = millis();

    // Non-blocking blink timeout. Signed comparison so it survives the
    // millis() rollover at ~49 days.
    if (ledOffAtMs != 0 && (int32_t)(now - ledOffAtMs) >= 0) {
        digitalWrite(LED_BUILTIN, LOW);
        // Back to the resting colour for whatever state we are in.
        if (locked) pixelSet(0, 0, 0);      // locked: dark between packets
        else        pixelSet(0, 0, 20);     // searching: dim blue
        ledOffAtMs = 0;
    }

    if (packetFlag) {
        packetFlag = false;
        uint8_t data[DAVIS_PACKET_LEN];
        Reading r;
        int st = radio.readData(data, DAVIS_PACKET_LEN);
        float rssi = radio.getRSSI(true, true);

        if (st == RADIOLIB_ERR_NONE && decodeOurs(data, r)) {
            if (rssi >= RSSI_FLOOR) {
                // A real visit: the station is on this channel right now, so
                // the next transmission is one channel along, one slot later.
                // Only re-time here -- do NOT hop. The slot timer below owns
                // hopping; doing it in both places advances two channels per
                // slot and leaves us permanently one ahead of the station.
                good++;
                missStreak = 0;
                nextSlotMs = now + (uint32_t)(SLOT_HALFMS / 2ULL);
                if (!locked) { locked = true; Serial.println(F("# locked on")); }
                digitalWrite(LED_BUILTIN, HIGH);
                pixelSet(0, 60, 0);        // green: got a packet
                ledOffAtMs = now + LED_BLINK_MS;
                printReading(data, r, rssi);
            }
            weak++;      // front-end bleed -- decodes fine but isn't a real visit
        } else {
            junk++;
        }
        radio.startReceive();
    }

    // Heartbeat FIRST, and unconditionally: when unlocked this loop used to
    // return early and print nothing at all, making a dead radio look
    // identical to a station that is simply out of range.
    static uint32_t lastReport = 0;
    if (now - lastReport > 15000) {
        lastReport = now;
        Serial.print(F("# good=")); Serial.print(good);
        Serial.print(F(" weak=")); Serial.print(weak);
        Serial.print(F(" junk=")); Serial.print(junk);
        Serial.print(F(" locked=")); Serial.print(locked ? 1 : 0);
        Serial.print(F(" ch=")); Serial.println(channel);
    }

    if (!locked) return;

    // Advance one channel per slot whether or not we heard anything, so we
    // stay lined up with the station's schedule through dropouts.
    if ((int32_t)(now - (nextSlotMs - PRE_SLOT_MS)) >= 0) {
        nextSlotMs += (uint32_t)(SLOT_HALFMS / 2ULL);
        setChannel(channel + 1);
        if (++missStreak > MAX_MISSES) {
            locked = false;
            missStreak = 0;
            setChannel(ANCHOR_CHANNEL);
            pixelSet(0, 0, 20);        // dim blue: searching again
            Serial.println(F("# lost lock, re-acquiring..."));
        }
    }

}
