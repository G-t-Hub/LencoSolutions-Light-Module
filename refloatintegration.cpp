#pragma once
#include "esc.cpp"

static const uint8_t FOOT_NONE = 0;
static const uint8_t FOOT_LEFT = 1;
static const uint8_t FOOT_RIGHT = 2;
static const uint8_t FOOT_BOTH = 3;
static const uint8_t REFLOAT_STATE_UNKNOWN = 0xFF;
static const unsigned long REFLOAT_POLL_TIMEOUT_MS = 2000;
static const unsigned long REFLOAT_REALTIME_TIMEOUT_MS = 700;
static const unsigned long LIFT_FADE_TIME_MS = 500;
static const unsigned long REFLOAT_DISABLE_GRACE_MS = 5000;
static const uint8_t REFLOAT_DISABLE_SHORT_RESPONSES = 6;
static const uint8_t REFLOAT_ACTIVE_DISABLE_SHORT_RESPONSES = 2;
static const uint8_t REFLOAT_REALTIME_MISSES_BEFORE_LEGACY = 5;

// Refloat LCM poll response layout.
#define LCM_POLL_STATE_IDX 3
#define LCM_POLL_DUTY_PITCH_IDX 5
#define LCM_POLL_BRIGHTNESS_IDX 12

// Legacy compact ALLDATA response layout.
#define ALLDATA_STATE_IDX 10
#define ALLDATA_ADC1_IDX 12
#define ALLDATA_ADC2_IDX 13
#define ALLDATA_VOLTAGE_IDX 23
#define ALLDATA_ERPM_IDX 25
#define ALLDATA_CURRENT_IDX 31
#define ALLDATA_DUTY_IDX 33
#define ALLDATA_ROLL_IDX 8
#define ALLDATA_FET_TEMP_IDX 39
#define ALLDATA_MOTOR_TEMP_IDX 40
#define BATTERY_LEVEL_IDX 3

// Refloat preview/v1.3 public REALTIME_DATA mask1 fields.
#define RT_MASK1_STATE_FLAGS     (1UL << 1)
#define RT_MASK1_SPEED           (1UL << 6)
#define RT_MASK1_ERPM            (1UL << 7)
#define RT_MASK1_DUTY_CYCLE      (1UL << 11)
#define RT_MASK1_BATTERY_VOLTAGE (1UL << 12)
#define RT_MASK1_BATTERY_CURRENT (1UL << 13)
#define RT_MASK1_BATTERY_SOC     (1UL << 14)
#define RT_MASK1_MOSFET_TEMP     (1UL << 15)
#define RT_MASK1_MOTOR_TEMP      (1UL << 16)
#define RT_MASK1_PITCH           (1UL << 17)
#define RT_MASK1_ROLL            (1UL << 19)
#define RT_MASK1_ADC_LEFT        (1UL << 20)
#define RT_MASK1_ADC_RIGHT       (1UL << 21)

class RefloatIntegration {
public:
    enum Protocol {
        PROTOCOL_UNKNOWN,
        PROTOCOL_REALTIME,
        PROTOCOL_LEGACY_ALLDATA
    };

    bool active = false;
    bool disabledByConfig = false;
    bool handtest = false;
    bool batteryValid = false;
    bool boardLifted = false;
    bool lightsOnState = true;
    bool realtimeDataValid = false;
    bool legacyAllDataValid = false;

    uint8_t brightness = 100;
    uint8_t state = REFLOAT_STATE_UNKNOWN;
    uint8_t lcmState = REFLOAT_STATE_UNKNOWN;
    uint8_t allDataState = REFLOAT_STATE_UNKNOWN;
    uint8_t footpad = FOOT_NONE;
    uint8_t pitchDeg = 0;

    float erpm = 0.0f;
    float dutyCycle = 0.0f;
    float voltage = 0.0f;
    float currentIn = 0.0f;
    float adcLeft = 0.0f;
    float adcRight = 0.0f;
    float roll = 0.0f;
    float fetTemp = 0.0f;
    float motorTemp = 0.0f;
    float batteryPct = 0.0f;
    float speedKmh = 0.0f;
    float liftFadeScale = 1.0f;

    unsigned long lastPollMs = 0;
    unsigned long lastRealtimeMs = 0;
    unsigned long lastAllDataMs = 0;

    Protocol protocol = PROTOCOL_UNKNOWN;

    void handleLcmPoll(uint8_t* data, uint8_t len) {
        if (len <= LCM_POLL_BRIGHTNESS_IDX) {
            if (len >= 3 && data[1] == 101 && data[2] == 24) {
                handleShortLcmResponse();
            }
            return;
        }

        disabledByConfig = false;
        shortResponseCount = 0;
        firstShortResponseMs = 0;
        active = true;
        brightness = data[LCM_POLL_BRIGHTNESS_IDX] > 100 ? 100 : data[LCM_POLL_BRIGHTNESS_IDX];
        uint8_t stateByte = data[LCM_POLL_STATE_IDX];
        lcmState = stateByte & 0x0F;
        if (!hasRecentRealtimeData() && !hasRecentAllData()) {
            state = lcmState;
        }
        handtest = (stateByte & 0x80) != 0;
        if (!isRunningCompatState(lcmState) && !hasRecentRealtimeData()) {
            pitchDeg = data[LCM_POLL_DUTY_PITCH_IDX];
        }
        footpad = (stateByte >> 4) & 0x03;
        lastPollMs = millis();
    }

    void handleLegacyAllData(uint8_t* data, uint8_t len) {
        if (len <= ALLDATA_MOTOR_TEMP_IDX || data[1] != 101 || data[2] != 10) return;
        if (disabledByConfig || !active) return;
        protocol = PROTOCOL_LEGACY_ALLDATA;
        lastPollMs = millis();
        lastAllDataMs = lastPollMs;
        legacyAllDataValid = true;
        allDataState = data[ALLDATA_STATE_IDX] & 0x0F;
        state = allDataState;
        adcLeft = data[ALLDATA_ADC1_IDX] / 50.0f;
        adcRight = data[ALLDATA_ADC2_IDX] / 50.0f;
        voltage = decodeInt16Scaled(data[ALLDATA_VOLTAGE_IDX], data[ALLDATA_VOLTAGE_IDX + 1], 10.0f);
        erpm = int16_t((uint16_t(data[ALLDATA_ERPM_IDX]) << 8) | data[ALLDATA_ERPM_IDX + 1]);
        currentIn = decodeInt16Scaled(data[ALLDATA_CURRENT_IDX], data[ALLDATA_CURRENT_IDX + 1], 10.0f);
        dutyCycle = constrain(abs(int(data[ALLDATA_DUTY_IDX]) - 128) / 100.0f, 0.0f, 1.0f);
        roll = decodeInt16Scaled(data[ALLDATA_ROLL_IDX], data[ALLDATA_ROLL_IDX + 1], 10.0f);
        fetTemp = data[ALLDATA_FET_TEMP_IDX] / 2.0f;
        motorTemp = data[ALLDATA_MOTOR_TEMP_IDX] / 2.0f;
    }

    void handleRealtimeData(uint8_t* data, uint8_t len) {
        if (len < 44 || data[1] != 101 || data[2] != 33) return;
        if (disabledByConfig || !active) return;

        uint8_t idx = 3;
        uint8_t controlFlags = data[idx++];
        if ((controlFlags & 0x01) != 0) return; // This firmware requests float16 only.

        uint32_t mask1 = readU32(data, idx); idx += 4;
        idx += 4; // mask2, intentionally unused until upstream implementation is stable.
        idx += 4; // timestamp

        if (mask1 & RT_MASK1_STATE_FLAGS) {
            uint32_t stateFlags = readU32(data, idx); idx += 4;
            uint8_t packageMode = (stateFlags >> 28) & 0x03;
            uint8_t packageState = (stateFlags >> 24) & 0x03;
            state = stateFromPackageState(packageState, packageMode, (stateFlags >> 16) & 0x01, (stateFlags >> 17) & 0x01);
            footpad = (stateFlags >> 22) & 0x03;
            handtest = packageMode == 1;
        }

        if (mask1 & RT_MASK1_SPEED) { speedKmh = abs(readFloat16(data, idx)); idx += 2; }
        if (mask1 & RT_MASK1_ERPM) { erpm = readFloat16(data, idx); idx += 2; }
        if (mask1 & RT_MASK1_DUTY_CYCLE) { dutyCycle = constrain(abs(readFloat16(data, idx)), 0.0f, 1.0f); idx += 2; }
        if (mask1 & RT_MASK1_BATTERY_VOLTAGE) { voltage = readFloat16(data, idx); idx += 2; }
        if (mask1 & RT_MASK1_BATTERY_CURRENT) { currentIn = readFloat16(data, idx); idx += 2; }
        if (mask1 & RT_MASK1_BATTERY_SOC) { batteryPct = constrain(readFloat16(data, idx), 0.0f, 1.0f); batteryValid = true; idx += 2; }
        if (mask1 & RT_MASK1_MOSFET_TEMP) { fetTemp = readFloat16(data, idx); idx += 2; }
        if (mask1 & RT_MASK1_MOTOR_TEMP) { motorTemp = readFloat16(data, idx); idx += 2; }
        if (mask1 & RT_MASK1_PITCH) { pitchDeg = (uint8_t)constrain((int)abs(readFloat16(data, idx)), 0, 180); idx += 2; }
        if (mask1 & RT_MASK1_ROLL) { roll = readFloat16(data, idx); idx += 2; }
        if (mask1 & RT_MASK1_ADC_LEFT) { adcLeft = readFloat16(data, idx); idx += 2; }
        if (mask1 & RT_MASK1_ADC_RIGHT) { adcRight = readFloat16(data, idx); idx += 2; }

        protocol = PROTOCOL_REALTIME;
        realtimeMisses = 0;
        realtimeDataValid = true;
        lastRealtimeMs = millis();
        lastPollMs = lastRealtimeMs;
    }

    void handleLegacyBattery(uint8_t* data, uint8_t len) {
        if (len < BATTERY_LEVEL_IDX + 4 || data[1] != 101 || data[2] != 29) return;
        if (disabledByConfig || !active || protocol == PROTOCOL_REALTIME) return;
        batteryPct = constrain(decodeFloat32Auto(data + BATTERY_LEVEL_IDX), 0.0f, 1.0f);
        batteryValid = true;
    }

    void requestTelemetry(ESC& esc) {
        if (protocol != PROTOCOL_LEGACY_ALLDATA) {
            esc.sendRefloatRealtimeRequest(realtimeMask1());
            lastRealtimeRequestMs = millis();
            if (protocol == PROTOCOL_UNKNOWN) {
                if (realtimeMisses < 255) realtimeMisses++;
                if (realtimeMisses >= REFLOAT_REALTIME_MISSES_BEFORE_LEGACY) {
                    protocol = PROTOCOL_LEGACY_ALLDATA;
                }
            }
            return;
        }
        esc.sendRefloatAllDataRequest();
    }

    void update(bool ledEnabled) {
        if (protocol == PROTOCOL_REALTIME && millis() - lastRealtimeRequestMs > REFLOAT_REALTIME_TIMEOUT_MS &&
            millis() - lastRealtimeMs > REFLOAT_REALTIME_TIMEOUT_MS) {
            protocol = PROTOCOL_LEGACY_ALLDATA;
        }
        updateLiftState();
        updateLiftFade();
        lightsOnState = ledEnabled && (!active || brightness > 0);
    }

    bool integrationEnabled() const {
        return !disabledByConfig;
    }

    bool lightsOn() const {
        return lightsOnState;
    }

    bool hasRealtimeBattery() const {
        return protocol == PROTOCOL_REALTIME && batteryValid;
    }

    bool needsLegacyBatteryRequest() const {
        return protocol != PROTOCOL_REALTIME;
    }

    bool leftFootpadPressed(const ESC& esc, bool swapFootpadAdc) const {
        float leftValue = active ? adcLeft : esc.adc1;
        float rightValue = active ? adcRight : esc.adc2;
        return (swapFootpadAdc ? rightValue : leftValue) > esc.footpadThreshold;
    }

    bool rightFootpadPressed(const ESC& esc, bool swapFootpadAdc) const {
        float leftValue = active ? adcLeft : esc.adc1;
        float rightValue = active ? adcRight : esc.adc2;
        return (swapFootpadAdc ? leftValue : rightValue) > esc.footpadThreshold;
    }

    float idleBrightnessScale(uint8_t idleBrightness) const {
        return (idleBrightness / 100.0f) * liftFadeScale;
    }

    float ridingBrightnessScale(uint8_t ridingBrightness) const {
        return ridingBrightness / 100.0f;
    }

    float statusBrightnessScale(uint8_t statusBrightness) const {
        return statusBrightness / 100.0f;
    }

private:
    uint8_t shortResponseCount = 0;
    uint8_t realtimeMisses = 0;
    unsigned long firstShortResponseMs = 0;
    unsigned long lastLiftFadeMs = 0;
    unsigned long lastRealtimeRequestMs = 0;

    static uint32_t realtimeMask1() {
        return RT_MASK1_STATE_FLAGS |
               RT_MASK1_SPEED |
               RT_MASK1_ERPM |
               RT_MASK1_DUTY_CYCLE |
               RT_MASK1_BATTERY_VOLTAGE |
               RT_MASK1_BATTERY_CURRENT |
               RT_MASK1_BATTERY_SOC |
               RT_MASK1_MOSFET_TEMP |
               RT_MASK1_MOTOR_TEMP |
               RT_MASK1_PITCH |
               RT_MASK1_ROLL |
               RT_MASK1_ADC_LEFT |
               RT_MASK1_ADC_RIGHT;
    }

    void handleShortLcmResponse() {
        unsigned long now = millis();
        bool wasActive = active;
        if (shortResponseCount == 0) firstShortResponseMs = now;
        if (shortResponseCount < 255) shortResponseCount++;
        uint8_t requiredResponses = wasActive ? REFLOAT_ACTIVE_DISABLE_SHORT_RESPONSES : REFLOAT_DISABLE_SHORT_RESPONSES;
        unsigned long requiredGrace = wasActive ? 0 : REFLOAT_DISABLE_GRACE_MS;
        if (shortResponseCount >= requiredResponses && now - firstShortResponseMs >= requiredGrace) {
            disableFromConfig();
        }
    }

    void disableFromConfig() {
        disabledByConfig = true;
        active = false;
        state = REFLOAT_STATE_UNKNOWN;
        lcmState = REFLOAT_STATE_UNKNOWN;
        allDataState = REFLOAT_STATE_UNKNOWN;
        realtimeDataValid = false;
        legacyAllDataValid = false;
        batteryValid = false;
        handtest = false;
        adcLeft = 0.0f;
        adcRight = 0.0f;
        roll = 0.0f;
        currentIn = 0.0f;
        fetTemp = 0.0f;
        motorTemp = 0.0f;
        speedKmh = 0.0f;
    }

    static bool isRunningCompatState(uint8_t value) {
        return value >= 1 && value <= 5;
    }

    bool hasRecentRealtimeData() const {
        return realtimeDataValid && (millis() - lastRealtimeMs <= REFLOAT_POLL_TIMEOUT_MS);
    }

    bool hasRecentAllData() const {
        return legacyAllDataValid && (millis() - lastAllDataMs <= REFLOAT_POLL_TIMEOUT_MS);
    }

    void updateLiftState() {
        if (!active || state == REFLOAT_STATE_UNKNOWN || isRunningCompatState(state)) {
            boardLifted = false;
            return;
        }
        if (pitchDeg > 60) {
            boardLifted = true;
        } else if (pitchDeg < 50) {
            boardLifted = false;
        }
    }

    void updateLiftFade() {
        unsigned long now = millis();
        if (lastLiftFadeMs == 0) {
            lastLiftFadeMs = now;
            return;
        }
        unsigned long delta = now - lastLiftFadeMs;
        lastLiftFadeMs = now;
        float step = delta / (float)LIFT_FADE_TIME_MS;
        liftFadeScale += boardLifted ? -step : step;
        liftFadeScale = constrain(liftFadeScale, 0.0f, 1.0f);
    }

    static float decodeInt16Scaled(uint8_t hi, uint8_t lo, float scale) {
        int16_t raw = (int16_t)((uint16_t(hi) << 8) | lo);
        return raw / scale;
    }

    static float decodeFloat32Auto(uint8_t* data) {
        uint32_t raw = ((uint32_t)data[0] << 24) |
                       ((uint32_t)data[1] << 16) |
                       ((uint32_t)data[2] << 8) |
                       (uint32_t)data[3];
        union { uint32_t u; float f; } value;
        value.u = raw;
        return value.f;
    }

    static uint32_t readU32(uint8_t* data, uint8_t idx) {
        return ((uint32_t)data[idx] << 24) |
               ((uint32_t)data[idx + 1] << 16) |
               ((uint32_t)data[idx + 2] << 8) |
               data[idx + 3];
    }

    static float readFloat16(uint8_t* data, uint8_t idx) {
        uint16_t h = ((uint16_t)data[idx] << 8) | data[idx + 1];
        uint16_t sign = (h >> 15) & 0x1;
        int16_t exp = (h >> 10) & 0x1F;
        uint16_t mant = h & 0x03FF;

        float value;
        if (exp == 0) {
            value = 0.0f;
        } else {
            value = 1.0f + mant / 1024.0f;
            int8_t shift = exp - 15;
            while (shift > 0) {
                value *= 2.0f;
                shift--;
            }
            while (shift < 0) {
                value *= 0.5f;
                shift++;
            }
        }
        return sign ? -value : value;
    }

    static uint8_t stateFromPackageState(uint8_t packageState, uint8_t packageMode, uint8_t wheelslip, uint8_t darkride) {
        if (packageState == 0) return 15;
        if (packageState == 1) return 0;
        if (packageState == 3) {
            if (packageMode == 2) return 5;
            if (wheelslip) return 3;
            if (darkride) return 4;
            return 1;
        }
        return 11;
    }
};
