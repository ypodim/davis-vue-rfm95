// Band sweeper: find where the desk station is actually transmitting.
//
// Established so far: the ISS is transmitting (green LED flashes every
// ~2.5s), our demodulator settings are correct (we decode a distant station
// fine), but nothing strong appears on either the US 902-928 table or the
// EU 868 set. So it's transmitting somewhere we haven't looked.
//
// This makes no assumption about any channel table. It sweeps the whole
// 902-928 MHz band measuring raw RSSI and reports anything well above the
// noise floor. A transmitter 30cm away should land around -40 dBm, against
// a measured noise floor near -115 dBm -- impossible to miss once we're
// pointed at the right frequency at the right moment.
//
// The catch is duty cycle: the packet is only ~7ms long every 2562ms, so
// any single pass will usually miss it. That's fine -- we sweep fast and
// continuously, so detections accumulate over a few minutes.

#include <RadioLib.h>

#define RFM95_CS   16
#define RFM95_INT  21
#define RFM95_RST  17
#define RFM95_DIO1 22

SX1276 radio = new Module(RFM95_CS, RFM95_INT, RFM95_RST, RFM95_DIO1);

#define F_START      902.0f
#define F_STOP       928.0f
#define F_STEP       0.200f    // MHz
#define DWELL_MS     30
#define REPORT_DBM   -95.0f    // well above the ~-115 dBm floor

float freq;
float passMax = -200.0f;
float passMaxFreq = 0.0f;
uint32_t passes = 0, detections = 0;

void setup() {
    Serial.begin(115200);
    uint32_t t0 = millis();
    while (!Serial && millis() - t0 < 3000) {}

    // Wide RX bandwidth: we're measuring energy here, not demodulating.
    int st = radio.beginFSK(F_START, 19.2, 5.0, 250.0, 10, 4);
    if (st != RADIOLIB_ERR_NONE) {
        while (true) { Serial.print(F("Radio init failed, code ")); Serial.println(st); delay(2000); }
    }
    radio.startReceive();
    freq = F_START;

    Serial.println(F("[sweep] 902-928 MHz, 200kHz steps, reporting anything > -95 dBm"));
    Serial.println(F("[sweep] station should read around -40 dBm at 30cm"));
}

void loop() {
    radio.standby();
    radio.setFrequency(freq);
    radio.startReceive();

    // Peak-hold across the dwell so a brief burst isn't averaged away.
    float peak = -200.0f;
    uint32_t t0 = millis();
    while (millis() - t0 < DWELL_MS) {
        float r = radio.getRSSI(false, true);
        if (r > peak) peak = r;
    }

    if (peak > REPORT_DBM) {
        detections++;
        Serial.print(F("[DETECT] ")); Serial.print(freq, 3);
        Serial.print(F(" MHz  rssi=")); Serial.print(peak, 1);
        Serial.println(F(" dBm"));
    }
    if (peak > passMax) { passMax = peak; passMaxFreq = freq; }

    freq += F_STEP;
    if (freq > F_STOP) {
        freq = F_START;
        passes++;
        Serial.print(F("[sweep] pass ")); Serial.print(passes);
        Serial.print(F(" strongest=")); Serial.print(passMax, 1);
        Serial.print(F("dBm @ ")); Serial.print(passMaxFreq, 3);
        Serial.print(F("MHz  detections=")); Serial.println(detections);
        passMax = -200.0f;
    }
}
