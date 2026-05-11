#include "esc.cpp"
#include <EEPROM.h>

// CAN packet type used by VESC Express to send LencoLED config commands to this node
#define CAN_PACKET_LENCOLED_CONFIG 240

// Refloat LCM poll response layout (indices into lcmPollData[])
// [0]=COMM_CUSTOM_APP_DATA(36), [1]=pkg(101), [2]=cmd(24),
// [3]=state, [4]=fault, [5]=duty/pitch, [6-7]=erpm, [8-9]=current,
// [10-11]=voltage, [12]=brightness, [13]=brightness_idle, [14]=status_brightness
#define LCM_POLL_BRIGHTNESS_IDX 12

// EEPROM layout (49 bytes total)
#define EEPROM_SIZE            52
#define EEPROM_MAGIC           0xABD1
#define EEPROM_MAGIC_ADDR      0  // uint16: EEPROM_MAGIC if initialized
#define EEPROM_COLORS_ADDR     2  // 36 bytes: 12 groups × 3 (RGB)
#define EEPROM_THRESH_ADDR    38  // uint16: footpadThreshold × 100
#define EEPROM_FULL_V_ADDR    40  // uint16: fullVoltage × 10
#define EEPROM_LOW_V_ADDR     42  // uint16: lowVoltage × 10
#define EEPROM_LED_STATE_ADDR 44  // uint8: startup LED state (0/1)
#define EEPROM_FP_WIDTH_ADDR  45  // uint8: footpad knight rider blob width
#define EEPROM_FP_DELAY_ADDR  46  // uint16: footpad knight rider step delay, big-endian
#define EEPROM_FP_FADE_ADDR   48  // uint8: footpad knight rider tail fade
#define EEPROM_LED_FORWARD_ADDR 49 // uint8: active forward LED count
#define EEPROM_LED_REVERSE_ADDR 50 // uint8: active reverse LED count
#define EEPROM_LED_FOOTPAD_ADDR 51 // uint8: active footpad LED count

class LencoLED {
public:
    enum {
        MAX_LED_COUNT = 20,
        DEFAULT_FORWARD_LED_COUNT = 17,
        DEFAULT_REVERSE_LED_COUNT = 17,
        DEFAULT_FOOTPAD_LED_COUNT = 10,
        DEFAULT_FP_RIDING_WIDTH = 2,
        DEFAULT_FP_ANIMATION_DELAY = 85,
        DEFAULT_FP_FADE_AMOUNT = 3
    };

    bool    ledEnabled      = true;
    uint8_t startupLedState = 1;
    uint8_t fpRidingWidth = DEFAULT_FP_RIDING_WIDTH;
    uint16_t fpAnimationDelay = DEFAULT_FP_ANIMATION_DELAY;
    uint8_t fpFadeAmount = DEFAULT_FP_FADE_AMOUNT;
    uint8_t numLedsForward = DEFAULT_FORWARD_LED_COUNT;
    uint8_t numLedsReverse = DEFAULT_REVERSE_LED_COUNT;
    uint8_t numLedsFootpad = DEFAULT_FOOTPAD_LED_COUNT;
    double  lowVoltage      = 58.9;
    double  fullVoltage     = 79.8;
    uint8_t ledColors[12][3] = {
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
    };

    // Call from setup() after ESC is initialised
    void init(ESC& esc) {
        setDefaultLedCounts();
        setDefaultFootpadSettings();
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

    // Call from loop() when esc.lcmPollAvailable is true
    void handleLcmPoll(uint8_t* data, uint8_t len) {
        if (len <= LCM_POLL_BRIGHTNESS_IDX) return;
        ledEnabled = (data[LCM_POLL_BRIGHTNESS_IDX] > 0);
    }

    // Call from loop() when esc.appFrameAvailable is true
    void handleCommand(uint8_t* data, uint8_t len, ESC& esc) {
        if (len < 1) return;
        switch (data[0]) {
            case 0x01: // CMD_SET_COLOR
                if (len >= 5 && data[1] < 12) {
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

    void loadFromEEPROM(ESC& esc) {
        for (uint8_t i = 0; i < 12; i++) {
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
    }

    void saveAll(double footpadThreshold) {
        for (uint8_t i = 0; i < 12; i++) {
            EEPROM.update(EEPROM_COLORS_ADDR + i * 3,     ledColors[i][0]);
            EEPROM.update(EEPROM_COLORS_ADDR + i * 3 + 1, ledColors[i][1]);
            EEPROM.update(EEPROM_COLORS_ADDR + i * 3 + 2, ledColors[i][2]);
        }
        saveSettings(footpadThreshold);
        EEPROM.update(EEPROM_LED_STATE_ADDR, startupLedState);
        saveLedCounts();
        saveFootpadSettings();
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
};
