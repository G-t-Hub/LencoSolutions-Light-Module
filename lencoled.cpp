#include "esc.cpp"
#include <EEPROM.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>

const char LENCOLED_FIRMWARE_MARKER[] PROGMEM __attribute__((used)) = "LCOTA1";
const char LENCOLED_FIRMWARE_VERSION[] PROGMEM __attribute__((used)) = "LencoLED Arduino v3.4";

// CAN packet type used by VESC Express to send LencoLED config commands to this node
#define CAN_PACKET_LENCOLED_CONFIG 240

// EEPROM layout (86 bytes total)
#define EEPROM_SIZE            87
#define EEPROM_MAGIC           0xABD6
#define EEPROM_MAGIC_ADDR      0  // uint16: EEPROM_MAGIC if initialized
#define EEPROM_COLORS_ADDR     2  // 51 bytes: 17 groups x 3 (RGB)
#define EEPROM_THRESH_ADDR    53  // uint16: footpadThreshold x 100
#define EEPROM_FULL_V_ADDR    55  // uint16: fullVoltage x 10
#define EEPROM_LOW_V_ADDR     57  // uint16: lowVoltage x 10
#define EEPROM_LED_STATE_ADDR 59  // uint8: startup LED state (0/1)
#define EEPROM_FP_WIDTH_ADDR  60  // uint8: footpad knight rider blob width
#define EEPROM_FP_DELAY_ADDR  61  // uint16: footpad knight rider step delay, big-endian
#define EEPROM_FP_FADE_ADDR   63  // uint8: footpad knight rider tail fade
#define EEPROM_LED_FORWARD_ADDR 64 // uint8: active forward LED count
#define EEPROM_LED_REVERSE_ADDR 65 // uint8: active reverse LED count
#define EEPROM_LED_FOOTPAD_ADDR 66 // uint8: active footpad LED count
#define EEPROM_RIDING_FOOTPAD_MODE_ADDR 67 // uint8: low nibble riding mode, high nibble idle mode
#define EEPROM_IDLE_BRIGHTNESS_ADDR 68      // uint8: idle brightness percent
#define EEPROM_RIDING_BRIGHTNESS_ADDR 69    // uint8: riding brightness percent
#define EEPROM_STATUS_BRIGHTNESS_ADDR 70    // uint8: riding footpad brightness percent
#define EEPROM_CURRENT_MIN_ADDR 71           // int16: current draw minimum, amps
#define EEPROM_CURRENT_MAX_ADDR 73           // int16: current draw maximum, amps
#define EEPROM_ROLL_MAX_ADDR 75              // uint8: symmetric roll angle maximum, degrees
#define EEPROM_FET_TEMP_MIN_ADDR 76          // int16: FET temp minimum, C
#define EEPROM_FET_TEMP_MAX_ADDR 78          // int16: FET temp maximum, C
#define EEPROM_MOTOR_TEMP_MIN_ADDR 80        // int16: motor temp minimum, C
#define EEPROM_MOTOR_TEMP_MAX_ADDR 82        // int16: motor temp maximum, C
#define EEPROM_DUTY_MID_ADDR 84              // uint8: duty mid threshold percent
#define EEPROM_DUTY_HIGH_ADDR 85             // uint8: duty high threshold percent
#define EEPROM_ADC_SWAP_ADDR  86             // uint8: swap footpad ADC sides
#define EEPROM_OTA_REQUEST_ADDR 87           // uint8: 0xA5 requests bootloader OTA mode
#define OTA_REQUEST_MAGIC      0xA5

class LencoLED {
public:
    enum {
        MAX_LED_COUNT = 20,
        DEFAULT_FORWARD_LED_COUNT = 17,
        DEFAULT_REVERSE_LED_COUNT = 17,
        DEFAULT_FOOTPAD_LED_COUNT = 10,
        DEFAULT_FP_RIDING_WIDTH = 2,
        DEFAULT_FP_ANIMATION_DELAY = 85,
        DEFAULT_FP_FADE_AMOUNT = 3,
        RIDING_FOOTPAD_DUTY = 0,
        RIDING_FOOTPAD_BATTERY = 1,
        RIDING_FOOTPAD_NONE = 2,
        RIDING_FOOTPAD_KNIGHTRIDER = 3,
        RIDING_FOOTPAD_CURRENT = 4,
        RIDING_FOOTPAD_ROLL = 5,
        RIDING_FOOTPAD_FET_TEMP = 6,
        RIDING_FOOTPAD_MOTOR_TEMP = 7,
        RIDING_FOOTPAD_SPEED = 8,
        DEFAULT_IDLE_FOOTPAD_MODE = RIDING_FOOTPAD_KNIGHTRIDER,
        LED_COLOR_COUNT = 17
    };

    bool    ledEnabled      = true;
    uint8_t startupLedState = 1;
    uint8_t fpRidingWidth = DEFAULT_FP_RIDING_WIDTH;
    uint16_t fpAnimationDelay = DEFAULT_FP_ANIMATION_DELAY;
    uint8_t fpFadeAmount = DEFAULT_FP_FADE_AMOUNT;
    uint8_t numLedsForward = DEFAULT_FORWARD_LED_COUNT;
    uint8_t numLedsReverse = DEFAULT_REVERSE_LED_COUNT;
    uint8_t numLedsFootpad = DEFAULT_FOOTPAD_LED_COUNT;
    uint8_t ridingFootpadMode = RIDING_FOOTPAD_DUTY;
    uint8_t idleFootpadMode = DEFAULT_IDLE_FOOTPAD_MODE;
    bool swapFootpadAdc = false;
    int16_t currentMin = -50;
    int16_t currentMax = 50;
    uint8_t rollMax = 45;
    int16_t fetTempMin = 0;
    int16_t fetTempMax = 80;
    int16_t motorTempMin = 0;
    int16_t motorTempMax = 80;
    uint8_t dutyMidThreshold = 70;
    uint8_t dutyHighThreshold = 80;
    uint8_t idleBrightness = 30;
    uint8_t ridingBrightness = 100;
    uint8_t statusBrightness = 100;
    double  lowVoltage      = 58.9;
    double  fullVoltage     = 79.8;
    uint8_t ledColors[LED_COLOR_COUNT][3] = {
        {228, 158,   0},  // 0: Forward LEDs
        {255,   0,   0},  // 1: Reverse LEDs
        { 50, 205,  50},  // 2: Startup Animation
        {  0, 255,   0},  // 3: Battery High (> 40%)
        {255, 180,   0},  // 4: Battery Mid (20-40%)
        {255,   0,   0},  // 5: Battery Low (< 20%)
        {  0,   0,  30},  // 6: Battery Empty
        {  0,   0, 255},  // 7: Footpad Indicator
        {  0,   0, 255},  // 8: Footpad Knight Rider
        {  0, 255,   0},  // 9: Duty 0-70%
        {255,  80,   0},  // 10: Duty 70-80%
        {255,   0,   0},  // 11: Duty > 80%
        {  0, 180, 255},  // 12: Current Draw
        {  0, 255,   0},  // 13: Current Regen
        {180,   0, 255},  // 14: Roll
        {255, 120,   0},  // 15: FET Temp
        {255,   0,  80},  // 16: Motor Temp
    };

    // Call from setup() after ESC is initialised
    void init(ESC& esc) {
        volatile uint8_t markerGuard = pgm_read_byte(&LENCOLED_FIRMWARE_MARKER[0]);
        markerGuard ^= pgm_read_byte(&LENCOLED_FIRMWARE_VERSION[0]);
        (void)markerGuard;
        setDefaultLedCounts();
        setDefaultFootpadSettings();
        setDefaultRidingFootpadMode();
        setDefaultBrightnessSettings();
        setDefaultTelemetryRanges();
        uint16_t magic;
        EEPROM.get(EEPROM_MAGIC_ADDR, magic);
        if (magic != EEPROM_MAGIC) {
            saveAll(esc.footpadThreshold);
            uint16_t m = EEPROM_MAGIC;
            EEPROM.put(EEPROM_MAGIC_ADDR, m);
        } else {
            loadFromEEPROM(esc);
        }
    }

    // Call from loop() when esc.appFrameAvailable is true
    void handleCommand(uint8_t* data, uint8_t len, ESC& esc) {
        if (len < 1) return;
        switch (data[0]) {
            case 0x01: // CMD_SET_COLOR
                if (len >= 5 && data[1] < LED_COLOR_COUNT) {
                    ledColors[data[1]][0] = data[2];
                    ledColors[data[1]][1] = data[3];
                    ledColors[data[1]][2] = data[4];
                    saveColor(data[1]);
                }
                break;
            case 0x02: // CMD_SET_THRESHOLD
                if (len >= 3) {
                    esc.footpadThreshold = ((uint16_t(data[1]) << 8) | data[2]) / 100.0;
                    saveSettings(esc.footpadThreshold);
                }
                break;
            case 0x03: // CMD_SET_BATTERY
                if (len >= 5) {
                    fullVoltage = ((uint16_t(data[1]) << 8) | data[2]) / 10.0;
                    lowVoltage  = ((uint16_t(data[3]) << 8) | data[4]) / 10.0;
                    saveSettings(esc.footpadThreshold);
                }
                break;
            case 0x04: // CMD_SET_LED_STATE
                // persist=0: apply to current runtime state only (do not touch EEPROM)
                // persist=1: save boot state to EEPROM only (do not change current runtime state)
                if (len >= 2) {
                    if (len >= 3 && data[2] == 1) {
                        startupLedState = data[1];
                        EEPROM.update(EEPROM_LED_STATE_ADDR, startupLedState);
                    } else {
                        ledEnabled = (data[1] == 1);
                    }
                }
                break;
            case 112: // CMD_SET_FOOTPAD_ANIMATION
                if (len >= 5) {
                    uint8_t width = data[1];
                    uint16_t delay = decodeEncodedDelay(data[2], data[3]);
                    uint8_t fade = data[4];
                    if (validFootpadSettings(width, delay, fade)) {
                        fpRidingWidth = width;
                        fpAnimationDelay = delay;
                        fpFadeAmount = fade;
                        saveFootpadSettings();
                    }
                }
                break;
            case 113: // CMD_SET_LED_COUNTS
                if (len >= 4) {
                    uint8_t forwardCount = data[1];
                    uint8_t reverseCount = data[2];
                    uint8_t footpadCount = data[3];
                    if (validLedCounts(forwardCount, reverseCount, footpadCount)) {
                        numLedsForward = forwardCount;
                        numLedsReverse = reverseCount;
                        numLedsFootpad = footpadCount;
                        bool needsSave = false;
                        if (fpRidingWidth > numLedsFootpad) {
                            fpRidingWidth = numLedsFootpad;
                            needsSave = true;
                        }
                        if (fpFadeAmount > numLedsFootpad) {
                            fpFadeAmount = numLedsFootpad;
                            needsSave = true;
                        }
                        if (needsSave) {
                            saveFootpadSettings();
                        }
                        saveLedCounts();
                    }
                }
                break;
            case 114: // CMD_SET_FOOTPAD_MODES
                if (len >= 2) {
                    uint8_t ridingMode = data[1] & 0x0F;
                    uint8_t idleMode = (data[1] >> 4) & 0x0F;
                    if (validFootpadMode(ridingMode) && validFootpadMode(idleMode)) {
                        ridingFootpadMode = ridingMode;
                        idleFootpadMode = idleMode;
                        if (len >= 3) {
                            swapFootpadAdc = data[2] != 0;
                        }
                        saveFootpadModes();
                    }
                }
                break;
            case 115: // CMD_SET_BRIGHTNESS_SETTINGS
                if (len >= 4 && validBrightnessSettings(data[1], data[2], data[3])) {
                    idleBrightness = data[1];
                    ridingBrightness = data[2];
                    statusBrightness = data[3];
                    saveBrightnessSettings();
                }
                break;
            case 116: // CMD_SET_TELEMETRY_RANGE
                handleTelemetryRangeCommand(data, len);
                break;
            case 120: // CMD_ENTER_OTA_BOOTLOADER
                EEPROM.update(EEPROM_OTA_REQUEST_ADDR, OTA_REQUEST_MAGIC);
                wdt_enable(WDTO_15MS);
                while (true) {}
                break;
        }
    }

private:
    static uint16_t decodeEncodedDelay(uint8_t hiEncoded, uint8_t loEncoded) {
        if (hiEncoded == 0 || loEncoded == 0) return 0;
        return (uint16_t(uint8_t(hiEncoded - 1)) << 8) | uint8_t(loEncoded - 1);
    }

    bool validFootpadSettings(uint8_t width, uint16_t delay, uint8_t fade) const {
        return width >= 1 && width <= numLedsFootpad &&
               delay >= 20 && delay <= 500 &&
               fade <= numLedsFootpad;
    }

    static bool validLedCount(uint8_t count) {
        return count >= 1 && count <= MAX_LED_COUNT;
    }

    static bool validLedCounts(uint8_t forwardCount, uint8_t reverseCount, uint8_t footpadCount) {
        return validLedCount(forwardCount) &&
               validLedCount(reverseCount) &&
               validLedCount(footpadCount);
    }

    static bool validFootpadMode(uint8_t mode) {
        return mode <= RIDING_FOOTPAD_SPEED;
    }

    static uint8_t packFootpadModes(uint8_t ridingMode, uint8_t idleMode) {
        return (idleMode << 4) | (ridingMode & 0x0F);
    }

    static bool validBrightness(uint8_t value) {
        return value <= 100;
    }

    static bool validBrightnessSettings(uint8_t idle, uint8_t riding, uint8_t status) {
        return validBrightness(idle) && validBrightness(riding) && validBrightness(status);
    }

    static bool validCurrentRange(int16_t minValue, int16_t maxValue) {
        return minValue >= -100 && maxValue <= 100 && minValue < maxValue;
    }

    static bool validTempRange(int16_t minValue, int16_t maxValue) {
        return minValue >= 0 && maxValue <= 150 && minValue < maxValue;
    }

    static bool validRollMax(uint8_t maxValue) {
        return maxValue >= 1 && maxValue <= 90;
    }

    static bool validDutyThresholds(uint8_t midValue, uint8_t highValue) {
        return midValue <= 100 && highValue <= 100 && midValue < highValue;
    }

    void handleTelemetryRangeCommand(uint8_t* data, uint8_t len) {
        if (len < 2) return;
        uint8_t mode = data[1];
        if (mode == RIDING_FOOTPAD_DUTY) {
            if (len >= 4 && validDutyThresholds(data[2], data[3])) {
                dutyMidThreshold = data[2];
                dutyHighThreshold = data[3];
                saveDutyThresholds();
            }
            return;
        }

        if (mode == RIDING_FOOTPAD_ROLL) {
            if (len >= 3 && validRollMax(data[2])) {
                rollMax = data[2];
                saveTelemetryRanges();
            }
            return;
        }

        if (len < 4) return;
        int16_t minValue = decodeTelemetryRangeValue(mode, data[2]);
        int16_t maxValue = decodeTelemetryRangeValue(mode, data[3]);
        if (mode == RIDING_FOOTPAD_CURRENT && validCurrentRange(minValue, maxValue)) {
            currentMin = minValue;
            currentMax = maxValue;
            saveTelemetryRanges();
        } else if (mode == RIDING_FOOTPAD_FET_TEMP && validTempRange(minValue, maxValue)) {
            fetTempMin = minValue;
            fetTempMax = maxValue;
            saveTelemetryRanges();
        } else if (mode == RIDING_FOOTPAD_MOTOR_TEMP && validTempRange(minValue, maxValue)) {
            motorTempMin = minValue;
            motorTempMax = maxValue;
            saveTelemetryRanges();
        }
    }

    static int16_t decodeTelemetryRangeValue(uint8_t mode, uint8_t value) {
        return mode == RIDING_FOOTPAD_CURRENT ? int16_t(value) - 100 : value;
    }

    void setDefaultFootpadSettings() {
        fpRidingWidth = DEFAULT_FP_RIDING_WIDTH;
        fpAnimationDelay = DEFAULT_FP_ANIMATION_DELAY;
        fpFadeAmount = DEFAULT_FP_FADE_AMOUNT;
    }

    void setDefaultLedCounts() {
        numLedsForward = DEFAULT_FORWARD_LED_COUNT;
        numLedsReverse = DEFAULT_REVERSE_LED_COUNT;
        numLedsFootpad = DEFAULT_FOOTPAD_LED_COUNT;
    }

    void setDefaultRidingFootpadMode() {
        ridingFootpadMode = RIDING_FOOTPAD_DUTY;
        idleFootpadMode = DEFAULT_IDLE_FOOTPAD_MODE;
    }

    void setDefaultBrightnessSettings() {
        idleBrightness = 30;
        ridingBrightness = 100;
        statusBrightness = 100;
    }

    void setDefaultTelemetryRanges() {
        currentMin = -50;
        currentMax = 50;
        rollMax = 45;
        fetTempMin = 0;
        fetTempMax = 80;
        motorTempMin = 0;
        motorTempMax = 80;
        dutyMidThreshold = 70;
        dutyHighThreshold = 80;
    }

    void loadFromEEPROM(ESC& esc) {
        for (uint8_t i = 0; i < LED_COLOR_COUNT; i++) {
            ledColors[i][0] = EEPROM.read(EEPROM_COLORS_ADDR + i * 3);
            ledColors[i][1] = EEPROM.read(EEPROM_COLORS_ADDR + i * 3 + 1);
            ledColors[i][2] = EEPROM.read(EEPROM_COLORS_ADDR + i * 3 + 2);
        }
        uint16_t threshRaw; EEPROM.get(EEPROM_THRESH_ADDR, threshRaw);
        esc.footpadThreshold = threshRaw / 100.0;
        uint16_t fullRaw;   EEPROM.get(EEPROM_FULL_V_ADDR, fullRaw);
        fullVoltage = fullRaw / 10.0;
        uint16_t lowRaw;    EEPROM.get(EEPROM_LOW_V_ADDR, lowRaw);
        lowVoltage = lowRaw / 10.0;
        startupLedState = EEPROM.read(EEPROM_LED_STATE_ADDR);
        ledEnabled = (startupLedState == 1);

        numLedsForward = EEPROM.read(EEPROM_LED_FORWARD_ADDR);
        numLedsReverse = EEPROM.read(EEPROM_LED_REVERSE_ADDR);
        numLedsFootpad = EEPROM.read(EEPROM_LED_FOOTPAD_ADDR);
        if (!validLedCounts(numLedsForward, numLedsReverse, numLedsFootpad)) {
            setDefaultLedCounts();
            saveLedCounts();
        }

        fpRidingWidth = EEPROM.read(EEPROM_FP_WIDTH_ADDR);
        fpAnimationDelay = (uint16_t(EEPROM.read(EEPROM_FP_DELAY_ADDR)) << 8) |
                           EEPROM.read(EEPROM_FP_DELAY_ADDR + 1);
        fpFadeAmount = EEPROM.read(EEPROM_FP_FADE_ADDR);
        if (!validFootpadSettings(fpRidingWidth, fpAnimationDelay, fpFadeAmount)) {
            setDefaultFootpadSettings();
            saveFootpadSettings();
        }

        uint8_t packedFootpadModes = EEPROM.read(EEPROM_RIDING_FOOTPAD_MODE_ADDR);
        if (packedFootpadModes <= RIDING_FOOTPAD_SPEED) {
            ridingFootpadMode = packedFootpadModes;
            idleFootpadMode = DEFAULT_IDLE_FOOTPAD_MODE;
            saveFootpadModes();
        } else {
            ridingFootpadMode = packedFootpadModes & 0x0F;
            idleFootpadMode = (packedFootpadModes >> 4) & 0x0F;
        }
        if (!validFootpadMode(ridingFootpadMode) || !validFootpadMode(idleFootpadMode)) {
            setDefaultRidingFootpadMode();
            saveFootpadModes();
        }
        uint8_t storedAdcSwap = EEPROM.read(EEPROM_ADC_SWAP_ADDR);
        if (storedAdcSwap <= 1) {
            swapFootpadAdc = storedAdcSwap == 1;
        } else {
            swapFootpadAdc = false;
            saveFootpadModes();
        }

        idleBrightness = EEPROM.read(EEPROM_IDLE_BRIGHTNESS_ADDR);
        ridingBrightness = EEPROM.read(EEPROM_RIDING_BRIGHTNESS_ADDR);
        statusBrightness = EEPROM.read(EEPROM_STATUS_BRIGHTNESS_ADDR);
        if (!validBrightnessSettings(idleBrightness, ridingBrightness, statusBrightness)) {
            setDefaultBrightnessSettings();
            saveBrightnessSettings();
        }

        EEPROM.get(EEPROM_CURRENT_MIN_ADDR, currentMin);
        EEPROM.get(EEPROM_CURRENT_MAX_ADDR, currentMax);
        rollMax = EEPROM.read(EEPROM_ROLL_MAX_ADDR);
        EEPROM.get(EEPROM_FET_TEMP_MIN_ADDR, fetTempMin);
        EEPROM.get(EEPROM_FET_TEMP_MAX_ADDR, fetTempMax);
        EEPROM.get(EEPROM_MOTOR_TEMP_MIN_ADDR, motorTempMin);
        EEPROM.get(EEPROM_MOTOR_TEMP_MAX_ADDR, motorTempMax);
        dutyMidThreshold = EEPROM.read(EEPROM_DUTY_MID_ADDR);
        dutyHighThreshold = EEPROM.read(EEPROM_DUTY_HIGH_ADDR);
        if (!validCurrentRange(currentMin, currentMax) ||
            !validRollMax(rollMax) ||
            !validTempRange(fetTempMin, fetTempMax) ||
            !validTempRange(motorTempMin, motorTempMax) ||
            !validDutyThresholds(dutyMidThreshold, dutyHighThreshold)) {
            setDefaultTelemetryRanges();
            saveTelemetryRanges();
        }
    }

    void saveAll(double footpadThreshold) {
        for (uint8_t i = 0; i < LED_COLOR_COUNT; i++) {
            EEPROM.update(EEPROM_COLORS_ADDR + i * 3,     ledColors[i][0]);
            EEPROM.update(EEPROM_COLORS_ADDR + i * 3 + 1, ledColors[i][1]);
            EEPROM.update(EEPROM_COLORS_ADDR + i * 3 + 2, ledColors[i][2]);
        }
        saveSettings(footpadThreshold);
        EEPROM.update(EEPROM_LED_STATE_ADDR, startupLedState);
        saveLedCounts();
        saveFootpadSettings();
        saveFootpadModes();
        saveBrightnessSettings();
        saveTelemetryRanges();
    }

    void saveColor(uint8_t id) {
        EEPROM.update(EEPROM_COLORS_ADDR + id * 3,     ledColors[id][0]);
        EEPROM.update(EEPROM_COLORS_ADDR + id * 3 + 1, ledColors[id][1]);
        EEPROM.update(EEPROM_COLORS_ADDR + id * 3 + 2, ledColors[id][2]);
    }

    void saveSettings(double footpadThreshold) {
        uint16_t threshRaw = (uint16_t)(footpadThreshold * 100);
        EEPROM.put(EEPROM_THRESH_ADDR, threshRaw);
        uint16_t fullRaw = (uint16_t)(fullVoltage * 10);
        EEPROM.put(EEPROM_FULL_V_ADDR, fullRaw);
        uint16_t lowRaw = (uint16_t)(lowVoltage * 10);
        EEPROM.put(EEPROM_LOW_V_ADDR, lowRaw);
    }

    void saveLedCounts() {
        EEPROM.update(EEPROM_LED_FORWARD_ADDR, numLedsForward);
        EEPROM.update(EEPROM_LED_REVERSE_ADDR, numLedsReverse);
        EEPROM.update(EEPROM_LED_FOOTPAD_ADDR, numLedsFootpad);
    }

    void saveFootpadSettings() {
        EEPROM.update(EEPROM_FP_WIDTH_ADDR, fpRidingWidth);
        EEPROM.update(EEPROM_FP_DELAY_ADDR, (uint8_t)(fpAnimationDelay >> 8));
        EEPROM.update(EEPROM_FP_DELAY_ADDR + 1, (uint8_t)(fpAnimationDelay & 0xFF));
        EEPROM.update(EEPROM_FP_FADE_ADDR, fpFadeAmount);
    }

    void saveFootpadModes() {
        EEPROM.update(EEPROM_RIDING_FOOTPAD_MODE_ADDR, packFootpadModes(ridingFootpadMode, idleFootpadMode));
        EEPROM.update(EEPROM_ADC_SWAP_ADDR, swapFootpadAdc ? 1 : 0);
    }

    void saveBrightnessSettings() {
        EEPROM.update(EEPROM_IDLE_BRIGHTNESS_ADDR, idleBrightness);
        EEPROM.update(EEPROM_RIDING_BRIGHTNESS_ADDR, ridingBrightness);
        EEPROM.update(EEPROM_STATUS_BRIGHTNESS_ADDR, statusBrightness);
    }

    void saveTelemetryRanges() {
        EEPROM.put(EEPROM_CURRENT_MIN_ADDR, currentMin);
        EEPROM.put(EEPROM_CURRENT_MAX_ADDR, currentMax);
        EEPROM.update(EEPROM_ROLL_MAX_ADDR, rollMax);
        EEPROM.put(EEPROM_FET_TEMP_MIN_ADDR, fetTempMin);
        EEPROM.put(EEPROM_FET_TEMP_MAX_ADDR, fetTempMax);
        EEPROM.put(EEPROM_MOTOR_TEMP_MIN_ADDR, motorTempMin);
        EEPROM.put(EEPROM_MOTOR_TEMP_MAX_ADDR, motorTempMax);
        saveDutyThresholds();
    }

    void saveDutyThresholds() {
        EEPROM.update(EEPROM_DUTY_MID_ADDR, dutyMidThreshold);
        EEPROM.update(EEPROM_DUTY_HIGH_ADDR, dutyHighThreshold);
    }
};
