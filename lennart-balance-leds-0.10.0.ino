#include <Arduino.h>
#include <FastLED.h>
#include <EEPROM.h>

#undef SPI_CLOCK     // Prevent FastLED/MCP2515 macro conflict

#include "balance_beeper.cpp"
#include "esc.cpp"

// EEPROM layout (45 bytes total)
#define EEPROM_MAGIC_ADDR      0  // uint16: 0xABCD if initialized
#define EEPROM_COLORS_ADDR     2  // 36 bytes: 12 groups × 3 (RGB)
#define EEPROM_THRESH_ADDR    38  // uint16: footpadThreshold × 100
#define EEPROM_FULL_V_ADDR    40  // uint16: fullVoltage × 10
#define EEPROM_LOW_V_ADDR     42  // uint16: lowVoltage × 10
#define EEPROM_LED_STATE_ADDR 44  // uint8: startup LED state (0/1)

// LencoLED runtime config — updated via CAN commands from VESC Express, persisted in EEPROM
bool ledEnabled = true;
uint8_t startupLedState = 1;
double lowVoltage = 58.9;
double fullVoltage = 79.8;
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

#define BATTERY_INDICATOR_DURATION 5000 // 5 seconds
#define STARTUP_ANIMATION_DURATION 5000 // 5 seconds

#define THRESHOLD 5000
#define FAST_DELAY 20
#define SLOW_DELAY 50
#define STARTUP_BRIGHTNESS 30 
#define NORMAL_BRIGHTNESS 255 

#define NUM_LEDS 17 
#define NUM_LEDS_FOOTPAD 10

#define FORWARD_PIN 5
#define REVERSE_PIN 6
#define FOOTPAD_PIN A0

#define FORWARD 0
#define REVERSE 1

#define BRAKE_IDLE_THRESHOLD 200
#define BRAKE_THRESHOLD 15
#define BRAKE_ON_DEBOUNCE_COUNT 3 
#define BRAKE_OFF_DEBOUNCE_COUNT 3

CRGB forward_leds[NUM_LEDS];
CRGB reverse_leds[NUM_LEDS];
CRGB footpad_leds[NUM_LEDS_FOOTPAD];

ESC esc;
BalanceBeeper balanceBeeper;

// Global variables for ESC data
double globalErpm = 0.0;
double globalVoltage = 0.0;
double globalDutyCycle = 0.0;
//double globalAdc1 = 0.0;   // (for later use)
//double globalAdc2 = 0.0;   // (for later use)

// Polling configuration
const unsigned long CAN_POLLING_INTERVAL = 100; // every 100ms
unsigned long lastCanPollTime = 0;

// LED & animation states
unsigned long lastKnightRiderUpdate = 0;
unsigned long lastBrakeCheckMillis = 0;
const unsigned long brakeCheckInterval = 50;
unsigned long lastLEDUpdateMillis = 0;
const unsigned long LED_UPDATE_INTERVAL = 16; // ~60 FPS

// Battery percent variables
unsigned long voltageAcquiredMS = 0;
bool voltageAcquired = false;
bool returningToStartup = false;

// Footpad sensor variables
unsigned long lastFootpadTriggerMillis = 0;
bool isInitialStartup = true;

// Startup Animation variables
unsigned long startupBeginMS = 0;
bool startupAnimationComplete = false;

int currentLEDIndex = 0;
int direction = FORWARD;
int previousDirection = FORWARD;
int animationDirFlag = 1;
int previousErpm = 0;

bool ledFadeActive = false;
bool fadeForwardReverse = false;
bool fadeFootpad = false;
unsigned long ledFadeStartMS = 0;
bool wasMovingState = false;

int currentBrightness = STARTUP_BRIGHTNESS;
int targetBrightness = STARTUP_BRIGHTNESS;

// Footpad knight rider animation variables
int footpadCurrentLEDIndex = 0;
int footpadAnimationDirFlag = 1;
unsigned long lastFootpadKnightRiderUpdate = 0;

bool startupState = true; 
bool movingState = false; 
bool isBraking = false;

void knightRider(int red, int green, int blue, int ridingWidth);
void checkBraking();
void footpadKnightRider();
void footpadDutyCycleIndicator();
void requestLEDFade(bool forwardReverse, bool footpad);
void handleConfigCommand(uint8_t* data, uint8_t len);
void initEEPROM();
void loadFromEEPROM();
void saveAllToEEPROM();
void saveColorToEEPROM(uint8_t id);
void saveSettingsToEEPROM();

void setup() {
  // Serial.begin(115200);
  esc.setup();
  initEEPROM();
  balanceBeeper.setup();

  FastLED.addLeds<WS2812B, FORWARD_PIN, GRB>(forward_leds, NUM_LEDS)
      .setCorrection(TypicalLEDStrip);
  FastLED.addLeds<WS2812B, REVERSE_PIN, GRB>(reverse_leds, NUM_LEDS)
      .setCorrection(TypicalLEDStrip);
  FastLED.addLeds<WS2812B, FOOTPAD_PIN, GRB>(footpad_leds, NUM_LEDS_FOOTPAD)
      .setCorrection(TypicalLEDStrip);

  FastLED.setMaxPowerInVoltsAndMilliamps(5, 500);
  FastLED.setBrightness(STARTUP_BRIGHTNESS);
  FastLED.clear();

  // Initial LED pattern
  for (int i = 0; i < NUM_LEDS; i++) {
    forward_leds[i] = CRGB(ledColors[0][0], ledColors[0][1], ledColors[0][2]);
    reverse_leds[i] = (i % 2 == 0)
        ? CRGB(ledColors[1][0], ledColors[1][1], ledColors[1][2])
        : CRGB(0, 0, 0);
  }

  startupBeginMS = millis();

  FastLED.show();
}

void loop() {
  
  // Passive listening for status 6;
  esc.listenForMessages();
  if (esc.configFrameAvailable) {
    handleConfigCommand(esc.configFrameData, esc.configFrameLen);
    esc.configFrameAvailable = false;
  }

  // === Periodic CAN polling ===
  if (millis() - lastCanPollTime >= CAN_POLLING_INTERVAL) {
    esc.getRealtimeData();      // send request for realtime data

    lastCanPollTime = millis();

    // Update globals if valid data was parsed
    globalErpm = esc.erpm;
    globalVoltage = esc.voltage;
    globalDutyCycle = esc.dutyCycle;
    // globalAdc1 = esc.adc1;  // (commented for later use)
    // globalAdc2 = esc.adc2;
  }

  // === Use global data ===
  balanceBeeper.loop(globalDutyCycle, globalErpm, globalVoltage, lowVoltage);

  // === Determine direction and state ===
  if (globalErpm > 200) {
    startupState = false;
    movingState = true;
    direction = FORWARD;
    targetBrightness = NORMAL_BRIGHTNESS;
  } else if (globalErpm < -200) {
    startupState = false;
    movingState = true;
    direction = REVERSE;
    targetBrightness = NORMAL_BRIGHTNESS;
  } else {
    if (movingState && !startupState)
    {
      returningToStartup = true;
    }
    startupState = true;
    movingState = false;
    targetBrightness = STARTUP_BRIGHTNESS;
  }

  // === Detect state and direction transitions ===
  if (movingState != wasMovingState) requestLEDFade(true, true);
  wasMovingState = movingState;

  if (direction != previousDirection) {
    requestLEDFade(true, false);
    previousDirection = direction;
  }

  // === LED patterns ===
  if (!ledEnabled) {
    fill_solid(forward_leds, NUM_LEDS, CRGB::Black);
    fill_solid(reverse_leds, NUM_LEDS, CRGB::Black);
    fill_solid(footpad_leds, NUM_LEDS_FOOTPAD, CRGB::Black);
  } else if (ledFadeActive) {
    if (fadeForwardReverse) {
      for (int i = 0; i < NUM_LEDS; i++) {
        forward_leds[i].fadeToBlackBy(60);
        reverse_leds[i].fadeToBlackBy(60);
      }
    }
    if (fadeFootpad) {
      for (int i = 0; i < NUM_LEDS_FOOTPAD; i++) {
        footpad_leds[i].fadeToBlackBy(60);
      }
    }
    if (millis() - ledFadeStartMS >= 250) {
      ledFadeActive = false;
    }
  } else if (startupState) {
    processStartupAction();
  } else if (movingState) {
    knightRider(ledColors[0][0], ledColors[0][1], ledColors[0][2], 5);
    footpadDutyCycleIndicator();
  }

  // === Brake logic ===
  if (millis() - lastBrakeCheckMillis >= brakeCheckInterval) {
    checkBraking();
    lastBrakeCheckMillis = millis();
  }

  // === Throttled LED update ===
  if (millis() - lastLEDUpdateMillis >= LED_UPDATE_INTERVAL) {
    currentBrightness += constrain(targetBrightness - currentBrightness, -5, 5);
    FastLED.setBrightness(currentBrightness);
    FastLED.show();
    lastLEDUpdateMillis = millis();
  }
}

void checkBraking() {
  static int debounceOnCount = 0;
  static int debounceOffCount = 0;
  int erpmDifference = previousErpm - globalErpm;

  if ((direction == FORWARD && erpmDifference > BRAKE_THRESHOLD && globalErpm > BRAKE_IDLE_THRESHOLD) ||
      (direction == REVERSE && erpmDifference < -BRAKE_THRESHOLD && globalErpm < -BRAKE_IDLE_THRESHOLD)) {
    debounceOnCount++;
    debounceOffCount = 0;
    if (debounceOnCount >= BRAKE_ON_DEBOUNCE_COUNT) {
      isBraking = true;
      debounceOnCount = 0;
    }
  } else {
    debounceOffCount++;
    debounceOnCount = 0;
    if (debounceOffCount >= BRAKE_OFF_DEBOUNCE_COUNT) {
      isBraking = false;
      debounceOffCount = 0;
    }
  }

  previousErpm = globalErpm;

  if (!ledEnabled) return;

  CRGB *leds_const = (direction == FORWARD) ? reverse_leds : forward_leds;
  if (isBraking) {
    for (int i = 0; i < NUM_LEDS; i++) {
      leds_const[i].setRGB(ledColors[1][0], ledColors[1][1], ledColors[1][2]);
    }
  } else {
    for (int i = 0; i < NUM_LEDS; i++) {
      if (i % 2 == 0)
        leds_const[i].setRGB(ledColors[1][0], ledColors[1][1], ledColors[1][2]);
      else
        leds_const[i] = CRGB(0, 0, 0);
    }
  }
}

void knightRider(int red, int green, int blue, int ridingWidth) {
  // Select correct LED array based on drive direction
  CRGB *leds = (direction == FORWARD) ? forward_leds : reverse_leds;
  if (!leds) return;

  // === Stop animation when idle ===
  const long IDLE_ERPM = 200; // below this, animation stops
  if (abs(globalErpm) < IDLE_ERPM) {
    // Smooth fade out when idle
    for (int i = 0; i < NUM_LEDS; i++) {
      leds[i].fadeToBlackBy(40);
    }
    FastLED.show();
    return;
  }

  // === Calculate delay based on ERPM ===
  long erpm = abs(globalErpm);
  unsigned long delayDuration = (unsigned long)map(erpm, 200, 20000, 80, 5);
  delayDuration = constrain(delayDuration, 5UL, 250UL);

  // === Time to update ===
  if (millis() - lastKnightRiderUpdate >= delayDuration) {

    // Slightly dim all LEDs to create a smooth trail
    for (int i = 0; i < NUM_LEDS; i++) {
      leds[i].fadeToBlackBy(60);
    }

    // Ensure current index stays valid
    currentLEDIndex = constrain(currentLEDIndex, 0, NUM_LEDS - ridingWidth - 2);

    // --- Draw the moving bright segment with soft edges ---
    for (int j = -2; j < ridingWidth + 2; j++) {
      int idx = currentLEDIndex + j;
      if (idx >= 0 && idx < NUM_LEDS) {
        int fadeFactor;
        if (j < 0 || j >= ridingWidth) fadeFactor = 30;     // soft edge glow
        else fadeFactor = 100;                              // main bright part
      
        leds[idx].setRGB(
          (red   * fadeFactor) / 100,
          (green * fadeFactor) / 100,
          (blue  * fadeFactor) / 100
        );
      }
    }
  
    // --- Move the light bar ---
    currentLEDIndex += animationDirFlag;

    // --- Bounce when reaching edges ---
    if (currentLEDIndex >= NUM_LEDS - ridingWidth - 2) {
      animationDirFlag = -1;
    } else if (currentLEDIndex <= 0) {
      animationDirFlag = 1;
    }

    FastLED.show();
    lastKnightRiderUpdate = millis();
  }
}

void processStartupAction() {
  // Reset voltage acquired flag when returning to startup
  if (returningToStartup) {
    voltageAcquired = false;
    returningToStartup = false;
  }

  // === Forward/Reverse LEDs: Always show static pattern ===
  staticStartupLEDs();

  // === Footpad LEDs: Priority-based display ===

  // Priority 0: No CAN connection — voltage still 0 once startup animation has completed
  if (startupAnimationComplete && globalVoltage == 0.0) {
    warningLEDs();
    return;
  }

  // Priority 1: Startup animation (if not complete)
  if (!startupAnimationComplete) {
    startupAnimation();
    return;
  }

  // Priority 2: Low voltage warning — overrides all idle animations
  if (globalVoltage > 0.0 && (globalVoltage - lowVoltage) / (fullVoltage - lowVoltage) <= 0.10) {
    lowVoltageWarningLEDs();
    return;
  }

  // Acquire voltage and start timer when voltage becomes available
  if (!voltageAcquired && globalVoltage != 0.0) {
    voltageAcquired = true;
    voltageAcquiredMS = millis();
  }

  // Check if both footpad sensors were triggered
  if (esc.adc1 > esc.footpadThreshold && esc.adc2 > esc.footpadThreshold) {
    lastFootpadTriggerMillis = millis(); // ASK I think this should be renamed to lastBatteryFootpadTriggerMillis
    isInitialStartup = false;
  }

  // Clear initial startup flag after duration expires
  if (isInitialStartup && voltageAcquired && (millis() - voltageAcquiredMS > BATTERY_INDICATOR_DURATION)) {
    isInitialStartup = false;  
  }

  // Determine if we should show battery
  bool showBatteryOnTimer = voltageAcquired && (millis() - voltageAcquiredMS <= BATTERY_INDICATOR_DURATION);
  bool showBatteryOnFootpad = voltageAcquired && (millis() - lastFootpadTriggerMillis <= BATTERY_INDICATOR_DURATION);
  
  // Priority 2: Battery percent indicator
  if (showBatteryOnTimer || showBatteryOnFootpad) {
    batteryPercentStartupLEDs();
  }
  // Priority 3: Single footpad indicator
  else {
    bool onlyOneFootpad = (esc.adc1 > esc.footpadThreshold) != (esc.adc2 > esc.footpadThreshold);
    
    if (onlyOneFootpad) {
      singleFootpadTriggeredStartupLEDs();
    } 
    // Priority 4: Footpad knight rider (default fallback)
    else {
      footpadKnightRider(); //ASK
    }
  }
} 

void startupAnimation() {
  unsigned long elapsed = millis() - startupBeginMS;
  
  if (elapsed >= STARTUP_ANIMATION_DURATION) {
    startupAnimationComplete = true;
    return; // Animation complete, exit
  }
  
  // Calculate how many LEDs should be lit based on progress
  int numLeds = map(elapsed, 0, STARTUP_ANIMATION_DURATION, 0, NUM_LEDS_FOOTPAD);  //ASK
  numLeds = constrain(numLeds, 0, NUM_LEDS_FOOTPAD);
  
  // Light up footpad LEDs progressively
  for (int i = 0; i < NUM_LEDS_FOOTPAD; i++) {
    if (i < numLeds) {
      footpad_leds[i] = CRGB(ledColors[2][0], ledColors[2][1], ledColors[2][2]);
    } else {
      footpad_leds[i] = CRGB(0, 0, 0);
    }
  }
}

void staticStartupLEDs() {
     // Static startup LEDs
  for (int i = 0; i < NUM_LEDS; i++) {
    if (direction == FORWARD) {
      forward_leds[i] = CRGB(ledColors[0][0], ledColors[0][1], ledColors[0][2]);
      reverse_leds[i] = (i % 2 == 0)
          ? CRGB(ledColors[1][0], ledColors[1][1], ledColors[1][2])
          : CRGB(0, 0, 0);
    } else {
      reverse_leds[i] = CRGB(ledColors[0][0], ledColors[0][1], ledColors[0][2]); //swapped due to inverse direction
      forward_leds[i] = (i % 2 == 0)
          ? CRGB(ledColors[1][0], ledColors[1][1], ledColors[1][2])
          : CRGB(0, 0, 0);
    }
  }
}

void lowVoltageWarningLEDs() {
  // Flash all footpad LEDs bright red when battery is at or below 10%.
  // Faster than warningLEDs to convey urgency.
  bool flashOn = (millis() / 250) % 2 == 0;
  for (int i = 0; i < NUM_LEDS_FOOTPAD; i++) {
    footpad_leds[i] = flashOn ? CRGB(255, 0, 0) : CRGB(0, 0, 0);
  }
}

void requestLEDFade(bool forwardReverse, bool footpad) {
  ledFadeActive = true;
  fadeForwardReverse = forwardReverse;
  fadeFootpad = footpad;
  ledFadeStartMS = millis();
}

void warningLEDs() {
  // Flash all footpad LEDs orange to indicate no CAN connection or misconfigured voltage range.
  // On/off at 400ms intervals.
  bool flashOn = (millis() / 400) % 2 == 0;
  for (int i = 0; i < NUM_LEDS_FOOTPAD; i++) {
    footpad_leds[i] = flashOn ? CRGB(255, 80, 0) : CRGB(0, 0, 0);
  }
}

void batteryPercentStartupLEDs() {
  double batteryVoltagePercentage = (globalVoltage - lowVoltage) / (fullVoltage - lowVoltage);

  // Voltage is outside the expected range — likely misconfigured voltage constants
  if (batteryVoltagePercentage < -0.10 || batteryVoltagePercentage > 1.10) {
    warningLEDs();
    return;
  }

  // Low voltage (≤ 10%) is handled as Priority 2 in processStartupAction before we get here.

  batteryVoltagePercentage = constrain(batteryVoltagePercentage, 0.0, 1.0);

  int r, g, b;
  if (batteryVoltagePercentage <= 0.20) {
    r = ledColors[5][0]; g = ledColors[5][1]; b = ledColors[5][2]; // Battery Low
  } else if (batteryVoltagePercentage <= 0.40) {
    r = ledColors[4][0]; g = ledColors[4][1]; b = ledColors[4][2]; // Battery Mid
  } else {
    r = ledColors[3][0]; g = ledColors[3][1]; b = ledColors[3][2]; // Battery High
  }

  int numLedsLit = (int)(batteryVoltagePercentage * NUM_LEDS_FOOTPAD);
  for (int i = 0; i < NUM_LEDS_FOOTPAD; i++) {
    if (i < numLedsLit) {
      footpad_leds[i].setRGB(r, g, b);
    } else {
      footpad_leds[i].setRGB(ledColors[6][0], ledColors[6][1], ledColors[6][2]); // Battery Empty
    }
  }
}

void singleFootpadTriggeredStartupLEDs() {
  
  if (esc.adc1 > esc.footpadThreshold)
  {
    for (int i = 0; i < NUM_LEDS_FOOTPAD; i++)
    {
      if (i < NUM_LEDS_FOOTPAD/2){
        footpad_leds[i].setRGB(ledColors[7][0], ledColors[7][1], ledColors[7][2]);
      }
      else {
        footpad_leds[i].setRGB(0, 0, 0);
      }
      
    }
  }
  else
  if (esc.adc2 > esc.footpadThreshold)
  {
    for (int i = 0; i < NUM_LEDS_FOOTPAD; i++)
    {
      if (i > NUM_LEDS_FOOTPAD/2){
        footpad_leds[i].setRGB(ledColors[7][0], ledColors[7][1], ledColors[7][2]);
      }
      else {
        footpad_leds[i].setRGB(0, 0, 0);
      }
    }
  }
}

void footpadKnightRider() {
  const int ridingWidth = 5;
  const unsigned long animationDelay = 50; // Fixed speed for idle animation

  if (millis() - lastFootpadKnightRiderUpdate >= animationDelay) {

    for (int i = 0; i < NUM_LEDS_FOOTPAD; i++) {
      footpad_leds[i].fadeToBlackBy(60);
    }

    footpadCurrentLEDIndex = constrain(footpadCurrentLEDIndex, 0, NUM_LEDS_FOOTPAD - ridingWidth - 2);

    for (int j = -2; j < ridingWidth + 2; j++) {
      int idx = footpadCurrentLEDIndex + j;
      if (idx >= 0 && idx < NUM_LEDS_FOOTPAD) {
        int fadeFactor;
        if (j < 0 || j >= ridingWidth) fadeFactor = 30;     
        else fadeFactor = 100;                              
      
        footpad_leds[idx].setRGB(
          (ledColors[8][0] * fadeFactor) / 100,
          (ledColors[8][1] * fadeFactor) / 100,
          (ledColors[8][2] * fadeFactor) / 100
        );
      }
    }
  
    footpadCurrentLEDIndex += footpadAnimationDirFlag;

    if (footpadCurrentLEDIndex >= NUM_LEDS_FOOTPAD - ridingWidth - 2) {
      footpadAnimationDirFlag = -1;
    } else if (footpadCurrentLEDIndex <= 0) {
      footpadAnimationDirFlag = 1;
    }

    lastFootpadKnightRiderUpdate = millis();
  }
}

void footpadDutyCycleIndicator() {
  // Normalize duty cycle to 0-100 range (duty cycle can be negative when braking)
  double dutyAbs = abs(globalDutyCycle);
  dutyAbs = constrain(dutyAbs, 0.0, 100.0);
  
  // Calculate how many LEDs to light based on duty cycle percentage
  int numLedsToLight = (int)((dutyAbs / 100.0) * NUM_LEDS_FOOTPAD);
  numLedsToLight = constrain(numLedsToLight, 0, NUM_LEDS_FOOTPAD);
  
  int r, g, b;
  if (dutyAbs >= 80.0) {
    r = ledColors[11][0]; g = ledColors[11][1]; b = ledColors[11][2]; // Duty > 80%
  } else if (dutyAbs >= 70.0) {
    r = ledColors[10][0]; g = ledColors[10][1]; b = ledColors[10][2]; // Duty 70-80%
  } else {
    r = ledColors[9][0];  g = ledColors[9][1];  b = ledColors[9][2];  // Duty 0-70%
  }

  for (int i = 0; i < NUM_LEDS_FOOTPAD; i++) {
    if (i < numLedsToLight) {
      footpad_leds[i].setRGB(r, g, b);
    } else {
      footpad_leds[i].setRGB(0, 0, 0);
    }
  }
}

// === LencoLED config command handler ===

void handleConfigCommand(uint8_t* data, uint8_t len) {
  if (len < 1) return;
  switch (data[0]) {
    case 0x01: // CMD_SET_COLOR
      if (len >= 5 && data[1] < 12) {
        ledColors[data[1]][0] = data[2];
        ledColors[data[1]][1] = data[3];
        ledColors[data[1]][2] = data[4];
        saveColorToEEPROM(data[1]);
      }
      break;
    case 0x02: // CMD_SET_THRESHOLD
      if (len >= 3) {
        esc.footpadThreshold = ((uint16_t(data[1]) << 8) | data[2]) / 100.0;
        saveSettingsToEEPROM();
      }
      break;
    case 0x03: // CMD_SET_BATTERY
      if (len >= 5) {
        fullVoltage = ((uint16_t(data[1]) << 8) | data[2]) / 10.0;
        lowVoltage  = ((uint16_t(data[3]) << 8) | data[4]) / 10.0;
        saveSettingsToEEPROM();
      }
      break;
    case 0x04: // CMD_SET_LED_STATE
      if (len >= 2) {
        ledEnabled = (data[1] == 1);
        if (len >= 3 && data[2] == 1) {
          startupLedState = data[1];
          EEPROM.update(EEPROM_LED_STATE_ADDR, startupLedState);
        }
      }
      break;
    case 0x05: // CMD_READ_ALL — response deferred (requires VESC Express CAN receive, Step 10)
      break;
  }
}

// === EEPROM helpers ===

void loadFromEEPROM() {
  for (uint8_t i = 0; i < 12; i++) {
    ledColors[i][0] = EEPROM.read(EEPROM_COLORS_ADDR + i * 3);
    ledColors[i][1] = EEPROM.read(EEPROM_COLORS_ADDR + i * 3 + 1);
    ledColors[i][2] = EEPROM.read(EEPROM_COLORS_ADDR + i * 3 + 2);
  }
  uint16_t threshRaw; EEPROM.get(EEPROM_THRESH_ADDR, threshRaw);
  esc.footpadThreshold = threshRaw / 100.0;
  uint16_t fullRaw; EEPROM.get(EEPROM_FULL_V_ADDR, fullRaw);
  fullVoltage = fullRaw / 10.0;
  uint16_t lowRaw; EEPROM.get(EEPROM_LOW_V_ADDR, lowRaw);
  lowVoltage = lowRaw / 10.0;
  startupLedState = EEPROM.read(EEPROM_LED_STATE_ADDR);
  ledEnabled = (startupLedState == 1);
}

void saveAllToEEPROM() {
  for (uint8_t i = 0; i < 12; i++) {
    EEPROM.update(EEPROM_COLORS_ADDR + i * 3,     ledColors[i][0]);
    EEPROM.update(EEPROM_COLORS_ADDR + i * 3 + 1, ledColors[i][1]);
    EEPROM.update(EEPROM_COLORS_ADDR + i * 3 + 2, ledColors[i][2]);
  }
  saveSettingsToEEPROM();
  EEPROM.update(EEPROM_LED_STATE_ADDR, startupLedState);
}

void saveColorToEEPROM(uint8_t id) {
  EEPROM.update(EEPROM_COLORS_ADDR + id * 3,     ledColors[id][0]);
  EEPROM.update(EEPROM_COLORS_ADDR + id * 3 + 1, ledColors[id][1]);
  EEPROM.update(EEPROM_COLORS_ADDR + id * 3 + 2, ledColors[id][2]);
}

void saveSettingsToEEPROM() {
  uint16_t threshRaw = (uint16_t)(esc.footpadThreshold * 100);
  EEPROM.put(EEPROM_THRESH_ADDR, threshRaw);
  uint16_t fullRaw = (uint16_t)(fullVoltage * 10);
  EEPROM.put(EEPROM_FULL_V_ADDR, fullRaw);
  uint16_t lowRaw = (uint16_t)(lowVoltage * 10);
  EEPROM.put(EEPROM_LOW_V_ADDR, lowRaw);
}

void initEEPROM() {
  uint16_t magic;
  EEPROM.get(EEPROM_MAGIC_ADDR, magic);
  if (magic != 0xABCE) {
    saveAllToEEPROM();
    uint16_t m = 0xABCE;
    EEPROM.put(EEPROM_MAGIC_ADDR, m);
  } else {
    loadFromEEPROM();
  }
}