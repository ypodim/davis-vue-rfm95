// Portable site-survey logger for the Davis receiver.
//
// Purpose: carry the Feather (on battery) to a candidate location, power it
// on, let it record, power it off, bring it back and download the results.
//
// Workflow
//   1. Power on at the location. NeoPixel goes BLUE while it hunts for the
//      station -- this can take up to ~140s, since acquisition means waiting
//      for the station to hop onto our anchor channel.
//   2. Once locked it records for RECORD_SECONDS. NeoPixel rests DIM GREEN
//      and flashes BRIGHT GREEN on each packet.
//   3. PURPLE = finished and written to flash. Safe to power off.
//      ORANGE  = never acquired; nothing recorded. Move closer or retry.
//   4. Back at the PC, plug in USB and run tools/survey_download.py.
//
// Sessions accumulate in LittleFS, so several locations can be surveyed
// before downloading. Send 'c' over serial to erase.
//
// Packets are buffered in RAM and written to flash once, at the end of the
// session. A LittleFS write can block for milliseconds, which would push the
// slot timer late and cost packets -- exactly the measurement we are trying
// to make. Nothing touches flash while recording.
//
// Build with a filesystem partition:
//   arduino-cli compile --fqbn rp2040:rp2040:adafruit_feather_rfm:flash=8388608_65536

#include <RadioLib.h>
#include <Adafruit_NeoPixel.h>
#include <LittleFS.h>

#define RFM95_CS   16
#define RFM95_INT  21
#define RFM95_RST  17
#define RFM95_DIO1 22

SX1276 radio = new Module(RFM95_CS, RFM95_INT, RFM95_RST, RFM95_DIO1);

#define TRANSMITTER_ID   3
#define DAVIS_PACKET_LEN 10
#define NUM_CHANNELS     51
#define SLOT_MS          2750UL     // (41 + wireID)/16 s
#define PRE_SLOT_MS      120
#define RSSI_FLOOR       -120.0f
#define ANCHOR_CHANNEL   26
#define MAX_MISSES       30

#define RECORD_SECONDS   180UL      // 3 minutes of LOCKED recording
#define ACQUIRE_TIMEOUT  200UL      // give up acquiring after this
#define MAX_SAMPLES      260        // 180/2.75 = ~66; generous headroom
#define LOG_PATH         "/survey.jsonl"

#define PIXEL_BRIGHTNESS 30
#define LED_BLINK_MS     60

Adafruit_NeoPixel pixel(1, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);
void pixelSet(uint8_t r, uint8_t g, uint8_t b) {
    pixel.setPixelColor(0, pixel.Color(r, g, b));
    pixel.show();
}

struct Sample {
    uint32_t ms;      // millis() at reception
    int16_t  rssi;
    uint8_t  ch;
    uint8_t  type;
};
Sample samples[MAX_SAMPLES];
uint16_t nSamples = 0;

enum Phase { ACQUIRING, RECORDING, DONE, FAILED };
Phase phase = ACQUIRING;

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
uint32_t nextSlotMs = 0, recordStartMs = 0, ledOffAtMs = 0;
uint16_t missStreak = 0;

volatile bool packetFlag = false;
void onPacketReceived() { packetFlag = true; }

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

bool decodeOurs(uint8_t *data, uint8_t &type) {
    for (uint8_t i = 0; i < DAVIS_PACKET_LEN; i++) data[i] = reverseBits(data[i]);
    uint16_t crcRx = ((uint16_t)data[6] << 8) | data[7];
    if (crcRx != crc16ccitt(data, 6)) return false;
    if ((data[0] & 0x07) != TRANSMITTER_ID) return false;
    type = data[0] >> 4;
    return true;
}

void dumpLog() {
    File f = LittleFS.open(LOG_PATH, "r");
    if (!f) { Serial.println(F("#EMPTY")); return; }
    Serial.println(F("#BEGIN"));
    while (f.available()) Serial.write(f.read());
    f.close();
    Serial.println(F("#END"));
}

void writeSession() {
    File f = LittleFS.open(LOG_PATH, "a");
    if (!f) { pixelSet(60, 0, 0); return; }   // red: could not write
    f.printf("#SESSION packets=%u duration_ms=%lu\n",
             nSamples, (unsigned long)(millis() - recordStartMs));
    for (uint16_t i = 0; i < nSamples; i++) {
        f.printf("{\"ms\":%lu,\"ch\":%u,\"rssi\":%d,\"type\":%u}\n",
                 (unsigned long)samples[i].ms, samples[i].ch,
                 samples[i].rssi, samples[i].type);
    }
    f.close();
}

void setup() {
    Serial.begin(115200);
    pixel.begin();
    pixel.setBrightness(PIXEL_BRIGHTNESS);
    pixelSet(0, 0, 20);                    // blue: acquiring

    LittleFS.begin();

    // Give a host a moment to attach, then dump whatever is stored. On
    // battery nobody is listening and this is harmless.
    uint32_t t0 = millis();
    while (!Serial && millis() - t0 < 2000) {}
    delay(200);
    dumpLog();

    int st = radio.beginFSK(channelFreqMHz[ANCHOR_CHANNEL], 19.2, 5.0, 25.0, 10, 4);
    if (st != RADIOLIB_ERR_NONE) {
        pixelSet(60, 0, 0);
        while (true) { Serial.print(F("# radio init failed ")); Serial.println(st); delay(2000); }
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
    Serial.println(F("# acquiring..."));
}

void loop() {
    uint32_t now = millis();

    // Serial commands work in any phase.
    if (Serial.available()) {
        char c = Serial.read();
        if (c == 'd') dumpLog();
        else if (c == 'c') { LittleFS.remove(LOG_PATH); Serial.println(F("#CLEARED")); }
    }

    if (ledOffAtMs != 0 && (int32_t)(now - ledOffAtMs) >= 0) {
        pixelSet(0, phase == RECORDING ? 4 : 0, phase == RECORDING ? 0 : 20);
        ledOffAtMs = 0;
    }

    if (phase == DONE || phase == FAILED) return;

    // Never acquired -- report and stop, rather than sitting blue forever.
    if (phase == ACQUIRING && now > ACQUIRE_TIMEOUT * 1000UL) {
        phase = FAILED;
        pixelSet(60, 25, 0);                 // orange: no station heard
        Serial.println(F("# FAILED: never acquired"));
        return;
    }

    if (packetFlag) {
        packetFlag = false;
        uint8_t data[DAVIS_PACKET_LEN], type;
        int st = radio.readData(data, DAVIS_PACKET_LEN);
        float rssi = radio.getRSSI(true, true);

        if (st == RADIOLIB_ERR_NONE && decodeOurs(data, type) && rssi >= RSSI_FLOOR) {
            missStreak = 0;
            nextSlotMs = now + SLOT_MS;
            if (!locked) {
                locked = true;
                phase = RECORDING;
                recordStartMs = now;
                Serial.println(F("# locked, recording"));
            }
            if (nSamples < MAX_SAMPLES) {
                samples[nSamples].ms   = now;
                samples[nSamples].rssi = (int16_t)rssi;
                samples[nSamples].ch   = channel;
                samples[nSamples].type = type;
                nSamples++;
            }
            pixelSet(0, 60, 0);              // bright green: packet
            ledOffAtMs = now + LED_BLINK_MS;
        } else {
            radio.startReceive();
        }
    }

    if (phase == RECORDING && (now - recordStartMs) >= RECORD_SECONDS * 1000UL) {
        writeSession();                      // single flash write, after recording
        phase = DONE;
        pixelSet(40, 0, 60);                 // purple: done, safe to power off
        Serial.print(F("# DONE packets=")); Serial.println(nSamples);
        return;
    }

    if (!locked) return;

    if ((int32_t)(now - (nextSlotMs - PRE_SLOT_MS)) >= 0) {
        nextSlotMs += SLOT_MS;
        setChannel(channel + 1);
        if (++missStreak > MAX_MISSES) {
            // Lost the station mid-survey. Keep what we have rather than
            // discarding a partial measurement.
            writeSession();
            phase = DONE;
            pixelSet(40, 0, 60);
            Serial.println(F("# DONE (lost lock, partial session saved)"));
        }
    }
}
