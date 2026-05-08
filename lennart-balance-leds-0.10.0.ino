#include <Arduino.h>
#include <FastLED.h>
#include <EEPROM.h>

#undef SPI_CLOCK     // Prevent FastLED/MCP2515 macro conflict

#include "balance_beeper.cpp"
#include "esc.cpp"
#include "lencoled.cpp"

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
LencoLED lencoLED;

// Global variables for ESC data
double globalErpm = 0.0;
double globalVoltage = 0.0;
double globalDutyCycle = 0.0;

// Polling configuration
const unsigned long CAN_POLLING_INTERVAL = 100; // every 100ms
unsigned long lastCanPollTime = 0;
const unsigned long LCM_POLL_INTERVAL = 500;
unsigned long lastLcmPollTime = 0;

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

void setup() {
  // Serial.begin(115200);
  esc.setup();
  lencoLED.init(esc);
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
    forward_leds[i] = CRGB(lencoLED.ledColors[0][0], lencoLED.ledColors[0][1], lencoLED.ledColors[0][2]);
    reverse_leds[i] = (i % 2 == 0)
        ? CRGB(lencoLED.ledColors[1][0], lencoLED.ledColors[1][1], lencoLED.ledColors[1][2])
        : CRGB(0, 0, 0);
  }

  startupBeginMS = millis();

  FastLED.show();
}

void loop() {

  esc.listenForMessages();
  if (esc.appFrameAvailable) {
    lencoLED.handleCommand(esc.appFrameData, esc.appFrameLen, esc);
    esc.appFrameAvailable = false;
  }
  if (esc.lcmPollAvailable) {
    lencoLED.handleLcmPoll(esc.lcmPollData, esc.lcmPollLen);
    esc.lcmPollAvailable = false;
  }

  // === Periodic LCM poll (Refloat light control) ===
  if (millis() - lastLcmPollTime >= LCM_POLL_INTERVAL) {
    esc.sendLcmPoll();
    lastLcmPollTime = millis();
  }

  // === Periodic CAN polling ===
  if (millis() - lastCanPollTime >= CAN_POLLING_INTERVAL) {
    esc.getRealtimeData();

    lastCanPollTime = millis();

    globalErpm = esc.erpm;
    globalVoltage = esc.voltage;
    globalDutyCycle = esc.dutyCycle;
  }

  // === Use global data ===
  balanceBeeper.loop(globalDutyCycle, globalErpm, globalVoltage, lencoLED.lowVoltage);

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
  // Expire transition fade regardless of LED toggle state.
  if (ledFadeActive && millis() - ledFadeStartMS >= 250) {
    ledFadeActive = false;
  }

  if (!lencoLED.ledEnabled) {
    fill_solid(forward_leds, NUM_LEDS, CRGB::Black);
    fill_solid(reverse_leds, NUM_LEDS, CRGB::Black);
  }

  if (lencoLED.ledEnabled && ledFadeActive) {
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
  } else if (!ledFadeActive && startupState) {
    processStartupAction();
  } else if (!ledFadeActive && movingState) {
    if (lencoLED.ledEnabled) {
      knightRider(lencoLED.ledColors[0][0], lencoLED.ledColors[0][1], lencoLED.ledColors[0][2], max(1, NUM_LEDS / 3));
      footpadDutyCycleIndicator();
    } else {
      fill_solid(footpad_leds, NUM_LEDS_FOOTPAD, CRGB::Black);
    }
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

  if (!lencoLED.ledEnabled) return;

  CRGB *leds_const = (direction == FORWARD) ? reverse_leds : forward_leds;
  if (isBraking) {
    for (int i = 0; i < NUM_LEDS; i++) {
      leds_const[i].setRGB(lencoLED.ledColors[1][0], lencoLED.ledColors[1][1], lencoLED.ledColors[1][2]);
    }
  } else {
    for (int i = 0; i < NUM_LEDS; i++) {
      if (i % 2 == 0)
        leds_const[i].setRGB(lencoLED.ledColors[1][0], lencoLED.ledColors[1][1], lencoLED.ledColors[1][2]);
      else
        leds_const[i] = CRGB(0, 0, 0);
    }
  }
}

void knightRider(int red, int green, int blue, int ridingWidth) {
  CRGB *leds = (direction == FORWARD) ? forward_leds : reverse_leds;
  if (!leds) return;

  const long IDLE_ERPM = 200;
  if (abs(globalErpm) < IDLE_ERPM) {
    for (int i = 0; i < NUM_LEDS; i++) {
      leds[i].fadeToBlackBy(40);
    }
    FastLED.show();
    return;
  }

  long erpm = abs(globalErpm);
  unsigned long delayDuration = (unsigned long)map(erpm, 200, 20000, 80, 5);
  delayDuration = constrain(delayDuration, 5UL, 250UL);

  if (millis() - lastKnightRiderUpdate >= delayDuration) {

    for (int i = 0; i < NUM_LEDS; i++) {
      leds[i].fadeToBlackBy(60);
    }

    currentLEDIndex = constrain(currentLEDIndex, 0, NUM_LEDS - ridingWidth - 2);

    for (int j = -2; j < ridingWidth + 2; j++) {
      int idx = currentLEDIndex + j;
      if (idx >= 0 && idx < NUM_LEDS) {
        int fadeFactor;
        if (j < 0 || j >= ridingWidth) fadeFactor = 30;
        else fadeFactor = 100;

        leds[idx].setRGB(
          (red   * fadeFactor) / 100,
          (green * fadeFactor) / 100,
          (blue  * fadeFactor) / 100
        );
      }
    }

    currentLEDIndex += animationDirFlag;

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
  if (returningToStartup) {
    voltageAcquired = false;
    returningToStartup = false;
  }

  staticStartupLEDs();  // skips forward/reverse when !ledEnabled

  if (!startupAnimationComplete) {
    startupAnimation();  // always runs regardless of ledEnabled
    return;
  }

  // Animation complete — remaining footpad patterns only show when LEDs are enabled.
  if (!lencoLED.ledEnabled) {
    fill_solid(footpad_leds, NUM_LEDS_FOOTPAD, CRGB::Black);
    return;
  }

  if (globalVoltage == 0.0) {
    warningLEDs();
    return;
  }

  if (globalVoltage > 0.0 && (globalVoltage - lencoLED.lowVoltage) / (lencoLED.fullVoltage - lencoLED.lowVoltage) <= 0.10) {
    lowVoltageWarningLEDs();
    return;
  }

  if (!voltageAcquired && globalVoltage != 0.0) {
    voltageAcquired = true;
    voltageAcquiredMS = millis();
  }

  if (esc.adc1 > esc.footpadThreshold && esc.adc2 > esc.footpadThreshold) {
    lastFootpadTriggerMillis = millis();
    isInitialStartup = false;
  }

  if (isInitialStartup && voltageAcquired && (millis() - voltageAcquiredMS > BATTERY_INDICATOR_DURATION)) {
    isInitialStartup = false;
  }

  bool showBatteryOnTimer   = voltageAcquired && (millis() - voltageAcquiredMS <= BATTERY_INDICATOR_DURATION);
  bool showBatteryOnFootpad = voltageAcquired && (millis() - lastFootpadTriggerMillis <= BATTERY_INDICATOR_DURATION);

  if (showBatteryOnTimer || showBatteryOnFootpad) {
    batteryPercentStartupLEDs();
  } else {
    bool onlyOneFootpad = (esc.adc1 > esc.footpadThreshold) != (esc.adc2 > esc.footpadThreshold);
    if (onlyOneFootpad) {
      singleFootpadTriggeredStartupLEDs();
    } else {
      footpadKnightRider();
    }
  }
}

void startupAnimation() {
  unsigned long elapsed = millis() - startupBeginMS;

  if (elapsed >= STARTUP_ANIMATION_DURATION) {
    startupAnimationComplete = true;
    return;
  }

  int numLeds = map(elapsed, 0, STARTUP_ANIMATION_DURATION, 0, NUM_LEDS_FOOTPAD);
  numLeds = constrain(numLeds, 0, NUM_LEDS_FOOTPAD);

  for (int i = 0; i < NUM_LEDS_FOOTPAD; i++) {
    if (i < numLeds) {
      footpad_leds[i] = CRGB(lencoLED.ledColors[2][0], lencoLED.ledColors[2][1], lencoLED.ledColors[2][2]);
    } else {
      footpad_leds[i] = CRGB(0, 0, 0);
    }
  }
}

void staticStartupLEDs() {
  if (!lencoLED.ledEnabled) return;
  for (int i = 0; i < NUM_LEDS; i++) {
    if (direction == FORWARD) {
      forward_leds[i] = CRGB(lencoLED.ledColors[0][0], lencoLED.ledColors[0][1], lencoLED.ledColors[0][2]);
      reverse_leds[i] = (i % 2 == 0)
          ? CRGB(lencoLED.ledColors[1][0], lencoLED.ledColors[1][1], lencoLED.ledColors[1][2])
          : CRGB(0, 0, 0);
    } else {
      reverse_leds[i] = CRGB(lencoLED.ledColors[0][0], lencoLED.ledColors[0][1], lencoLED.ledColors[0][2]);
      forward_leds[i] = (i % 2 == 0)
          ? CRGB(lencoLED.ledColors[1][0], lencoLED.ledColors[1][1], lencoLED.ledColors[1][2])
          : CRGB(0, 0, 0);
    }
  }
}

void lowVoltageWarningLEDs() {
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
  bool flashOn = (millis() / 400) % 2 == 0;
  for (int i = 0; i < NUM_LEDS_FOOTPAD; i++) {
    footpad_leds[i] = flashOn ? CRGB(255, 80, 0) : CRGB(0, 0, 0);
  }
}

void batteryPercentStartupLEDs() {
  double batteryVoltagePercentage = (globalVoltage - lencoLED.lowVoltage) / (lencoLED.fullVoltage - lencoLED.lowVoltage);

  if (batteryVoltagePercentage < -0.10 || batteryVoltagePercentage > 1.10) {
    warningLEDs();
    return;
  }

  batteryVoltagePercentage = constrain(batteryVoltagePercentage, 0.0, 1.0);

  int r, g, b;
  if (batteryVoltagePercentage <= 0.20) {
    r = lencoLED.ledColors[5][0]; g = lencoLED.ledColors[5][1]; b = lencoLED.ledColors[5][2];
  } else if (batteryVoltagePercentage <= 0.40) {
    r = lencoLED.ledColors[4][0]; g = lencoLED.ledColors[4][1]; b = lencoLED.ledColors[4][2];
  } else {
    r = lencoLED.ledColors[3][0]; g = lencoLED.ledColors[3][1]; b = lencoLED.ledColors[3][2];
  }

  int numLedsLit = (int)(batteryVoltagePercentage * NUM_LEDS_FOOTPAD);
  for (int i = 0; i < NUM_LEDS_FOOTPAD; i++) {
    if (i < numLedsLit) {
      footpad_leds[i].setRGB(r, g, b);
    } else {
      footpad_leds[i].setRGB(lencoLED.ledColors[6][0], lencoLED.ledColors[6][1], lencoLED.ledColors[6][2]);
    }
  }
}

void singleFootpadTriggeredStartupLEDs() {
  const int half = NUM_LEDS_FOOTPAD / 2;
  if (esc.adc1 > esc.footpadThreshold) {
    for (int i = 0; i < NUM_LEDS_FOOTPAD; i++) {
      if (i < half)
        footpad_leds[i].setRGB(lencoLED.ledColors[7][0], lencoLED.ledColors[7][1], lencoLED.ledColors[7][2]);
      else
        footpad_leds[i].setRGB(0, 0, 0);
    }
  } else if (esc.adc2 > esc.footpadThreshold) {
    for (int i = 0; i < NUM_LEDS_FOOTPAD; i++) {
      if (i >= half)
        footpad_leds[i].setRGB(lencoLED.ledColors[7][0], lencoLED.ledColors[7][1], lencoLED.ledColors[7][2]);
      else
        footpad_leds[i].setRGB(0, 0, 0);
    }
  }
}

void footpadKnightRider() {
  const int ridingWidth = max(1, NUM_LEDS_FOOTPAD / 6);
  const unsigned long animationDelay = max(20UL, 50UL * NUM_LEDS / NUM_LEDS_FOOTPAD);
  const int travel = max(1, NUM_LEDS_FOOTPAD - ridingWidth - 2);
  const uint8_t fadeAmount = (uint8_t)constrain(600 / travel, 50, 220);

  if (millis() - lastFootpadKnightRiderUpdate >= animationDelay) {

    for (int i = 0; i < NUM_LEDS_FOOTPAD; i++) {
      footpad_leds[i].fadeToBlackBy(fadeAmount);
    }

    footpadCurrentLEDIndex = constrain(footpadCurrentLEDIndex, 0, NUM_LEDS_FOOTPAD - ridingWidth - 2);

    for (int j = -2; j < ridingWidth + 2; j++) {
      int idx = footpadCurrentLEDIndex + j;
      if (idx >= 0 && idx < NUM_LEDS_FOOTPAD) {
        int fadeFactor;
        if (j < 0 || j >= ridingWidth) fadeFactor = 30;
        else fadeFactor = 100;

        footpad_leds[idx].setRGB(
          (lencoLED.ledColors[8][0] * fadeFactor) / 100,
          (lencoLED.ledColors[8][1] * fadeFactor) / 100,
          (lencoLED.ledColors[8][2] * fadeFactor) / 100
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
  double dutyAbs = abs(globalDutyCycle);
  dutyAbs = constrain(dutyAbs, 0.0, 100.0);

  int numLedsToLight = (int)((dutyAbs / 100.0) * NUM_LEDS_FOOTPAD);
  numLedsToLight = constrain(numLedsToLight, 0, NUM_LEDS_FOOTPAD);

  int r, g, b;
  if (dutyAbs >= 80.0) {
    r = lencoLED.ledColors[11][0]; g = lencoLED.ledColors[11][1]; b = lencoLED.ledColors[11][2];
  } else if (dutyAbs >= 70.0) {
    r = lencoLED.ledColors[10][0]; g = lencoLED.ledColors[10][1]; b = lencoLED.ledColors[10][2];
  } else {
    r = lencoLED.ledColors[9][0];  g = lencoLED.ledColors[9][1];  b = lencoLED.ledColors[9][2];
  }

  for (int i = 0; i < NUM_LEDS_FOOTPAD; i++) {
    if (i < numLedsToLight) {
      footpad_leds[i].setRGB(r, g, b);
    } else {
      footpad_leds[i].setRGB(0, 0, 0);
    }
  }
}
