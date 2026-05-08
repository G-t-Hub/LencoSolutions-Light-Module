#include "esc.cpp"
#include <EEPROM.h>

// CAN packet type used by VESC Express to send LencoLED config commands to this node
#define CAN_PACKET_LENCOLED_CONFIG 240

// Refloat LCM poll response layout (indices into lcmPollData[])
// [0]=COMM_CUSTOM_APP_DATA(36), [1]=pkg(101), [2]=cmd(24),
// [3]=state, [4]=fault, [5]=duty/pitch, [6-7]=erpm, [8-9]=current,
// [10-11]=voltage, [12]=brightness, [13]=brightness_idle, [14]=status_brightness
#define LCM_POLL_BRIGHTNESS_IDX 12

// EEPROM layout (45 bytes total)
#define EEPROM_MAGIC_ADDR      0  // uint16: 0xABCE if initialized
#define EEPROM_COLORS_ADDR     2  // 36 bytes: 12 groups × 3 (RGB)
#define EEPROM_THRESH_ADDR    38  // uint16: footpadThreshold × 100
#define EEPROM_FULL_V_ADDR    40  // uint16: fullVoltage × 10
#define EEPROM_LOW_V_ADDR     42  // uint16: lowVoltage × 10
#define EEPROM_LED_STATE_ADDR 44  // uint8: startup LED state (0/1)

class LencoLED {
public:
    bool    ledEnabled      = true;
    uint8_t startupLedState = 1;
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
        uint16_t magic;
        EEPROM.get(EEPROM_MAGIC_ADDR, magic);
        if (magic != 0xABCE) {
            saveAll(esc.footpadThreshold);
            uint16_t m = 0xABCE;
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
        }
    }

private:
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
    }

    void saveAll(double footpadThreshold) {
        for (uint8_t i = 0; i < 12; i++) {
            EEPROM.update(EEPROM_COLORS_ADDR + i * 3,     ledColors[i][0]);
            EEPROM.update(EEPROM_COLORS_ADDR + i * 3 + 1, ledColors[i][1]);
            EEPROM.update(EEPROM_COLORS_ADDR + i * 3 + 2, ledColors[i][2]);
        }
        saveSettings(footpadThreshold);
        EEPROM.update(EEPROM_LED_STATE_ADDR, startupLedState);
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
};
