#include <Arduino.h>
#include <FastLED.h>
#include <EEPROM.h>

#undef SPI_CLOCK     // Prevent FastLED/MCP2515 macro conflict

#include "balance_beeper.cpp"
#include "esc.cpp"
#include "lencoled.cpp"
#include "refloatintegration.cpp"

const uint8_t MAX_LEDS_PER_STRIP = LencoLED::MAX_LED_COUNT;

#define BATTERY_INDICATOR_DURATION 5000 // 5 seconds
#define STARTUP_ANIMATION_DURATION 5000 // 5 seconds

#define STARTUP_BRIGHTNESS 30
#define NORMAL_BRIGHTNESS 255

#define FORWARD_PIN 5
#define REVERSE_PIN 6
#define FOOTPAD_PIN A0

#define FORWARD 0
#define REVERSE 1

#define BRAKE_IDLE_THRESHOLD 200
#define BRAKE_THRESHOLD 15
#define BRAKE_ON_DEBOUNCE_COUNT 3
#define BRAKE_OFF_DEBOUNCE_COUNT 3
#define MOVING_ERPM_THRESHOLD 200
#define DIRECTION_ERPM_THRESHOLD 20
#define DIRECTION_FADE_DURATION_MS 500

CRGB forward_leds[MAX_LEDS_PER_STRIP];
CRGB reverse_leds[MAX_LEDS_PER_STRIP];
CRGB footpad_leds[MAX_LEDS_PER_STRIP];

ESC esc;
BalanceBeeper balanceBeeper;
LencoLED lencoLED;
RefloatIntegration refloat;

// Global variables for ESC data
float globalErpm = 0.0f;
float globalVoltage = 0.0f;
float globalDutyCycle = 0.0f;

// Polling configuration
const unsigned long CAN_POLLING_INTERVAL = 100; // every 100ms
unsigned long lastCanPollTime = 0;
const unsigned long LCM_POLL_INTERVAL = 500;
unsigned long lastLcmPollTime = 0;
const unsigned long REFLOAT_ALLDATA_POLL_INTERVAL = 100;
const unsigned long REFLOAT_BATTERY_POLL_INTERVAL = 500;
const unsigned long REFLOAT_DISABLED_PROBE_INTERVAL = 3000;
unsigned long lastRefloatAllDataPollTime = 0;
unsigned long lastRefloatBatteryPollTime = 0;

// LED & animation states
unsigned long lastKnightRiderUpdate = 0;
unsigned long lastBrakeCheckMillis = 0;
const unsigned long brakeCheckInterval = 50;
unsigned long lastLEDUpdateMillis = 0;
const unsigned long LED_UPDATE_INTERVAL = 16; // ~60 FPS
const unsigned long STATE_TRANSITION_DEBOUNCE_MS = 150;

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

int direction = FORWARD;
int previousDirection = FORWARD;
int previousErpm = 0;
int rideRiderPosition = 0;
int rideRiderDirection = 1;

bool ledFadeActive = false;
unsigned long ledFadeStartMS = 0;
int fadingDirection = FORWARD;
bool directionStripClearPending = false;
bool wasMovingState = false;
bool pendingMovingState = false;
bool pendingMovingStateActive = false;
unsigned long pendingMovingStateMS = 0;

int currentBrightness = STARTUP_BRIGHTNESS;
int targetBrightness = STARTUP_BRIGHTNESS;

// Footpad animation (per-animation state lives as statics inside each function)

bool startupState = true;
bool movingState = false;
bool isBraking = false;
bool batteryDisplayActive = false;

void knightRider(int red, int green, int blue, int ridingWidth);
void checkBraking();
bool footpadKnightRider();
void footpadDutyCycleIndicator();
void footpadCurrentIndicator();
void footpadRollIndicator();
void footpadSpeedIndicator();
void footpadFetTempIndicator();
void footpadMotorTempIndicator();
void footpadTelemetryBar(float value, int16_t minValue, int16_t maxValue, uint8_t colorIndex);
bool footpadModeUsesAllData(uint8_t mode);
bool footpadModeUsesBattery(uint8_t mode);
void renderFootpadMode(uint8_t mode, bool applyStatusBrightness);
void renderRidingFootpad();
void footpadHandtest();
void footpadRainbow();
void disabledFootpadIndicator();
void scaleFootpadLeds(float scale);
void staticStartupLEDs();
void requestDirectionFade(int oldDirection);
void applyDirectionFade();
void clearActiveRideStrip();
void renderRearLights();
void resetRideKnightRider();
void clearInactiveLEDs();

void setup() {
  // Serial.begin(115200);
  esc.setup();
  lencoLED.init(esc);
  balanceBeeper.setup();

  FastLED.addLeds<WS2812B, FORWARD_PIN, GRB>(forward_leds, MAX_LEDS_PER_STRIP)
      .setCorrection(TypicalLEDStrip);
  FastLED.addLeds<WS2812B, REVERSE_PIN, GRB>(reverse_leds, MAX_LEDS_PER_STRIP)
      .setCorrection(TypicalLEDStrip);
  FastLED.addLeds<WS2812B, FOOTPAD_PIN, GRB>(footpad_leds, MAX_LEDS_PER_STRIP)
      .setCorrection(TypicalLEDStrip);

  FastLED.setMaxPowerInVoltsAndMilliamps(5, 500);
  FastLED.setBrightness(STARTUP_BRIGHTNESS);
  FastLED.clear();

  // Initial LED pattern
  for (int i = 0; i < lencoLED.numLedsForward; i++) {
    forward_leds[i] = CRGB(lencoLED.ledColors[0][0], lencoLED.ledColors[0][1], lencoLED.ledColors[0][2]);
  }
  for (int i = 0; i < lencoLED.numLedsReverse; i++) {
    reverse_leds[i] = (i % 2 == 0)
        ? CRGB(lencoLED.ledColors[1][0], lencoLED.ledColors[1][1], lencoLED.ledColors[1][2])
        : CRGB(0, 0, 0);
  }
  clearInactiveLEDs();

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
    refloat.handleLcmPoll(esc.lcmPollData, esc.lcmPollLen);
    esc.lcmPollAvailable = false;
  }
  if (esc.refloatAllDataAvailable) {
    refloat.handleLegacyAllData(esc.refloatAllData, esc.refloatAllDataLen);
    esc.refloatAllDataAvailable = false;
    globalErpm = refloat.erpm;
    globalVoltage = refloat.voltage;
    globalDutyCycle = refloat.dutyCycle;
  }
  if (esc.refloatRealtimeAvailable) {
    refloat.handleRealtimeData(esc.refloatRealtimeData, esc.refloatRealtimeLen);
    esc.refloatRealtimeAvailable = false;
    globalErpm = refloat.erpm;
    globalVoltage = refloat.voltage;
    globalDutyCycle = refloat.dutyCycle;
  }
  if (esc.refloatBatteryAvailable) {
    refloat.handleLegacyBattery(esc.refloatBatteryData, esc.refloatBatteryLen);
    esc.refloatBatteryAvailable = false;
  }
  refloat.update(lencoLED.ledEnabled);

  // === Periodic LCM poll (Refloat light control) ===
  unsigned long lcmPollInterval = refloat.integrationEnabled()
      ? LCM_POLL_INTERVAL
      : REFLOAT_DISABLED_PROBE_INTERVAL;
  if (millis() - lastLcmPollTime >= lcmPollInterval) {
    esc.sendLcmPoll();
    lastLcmPollTime = millis();
  }

  bool idleFootpadModeDisplayed = startupState && !movingState && startupAnimationComplete &&
      refloat.lightsOn() && !(refloat.active && refloat.state == 5);

  if (refloat.active &&
      refloat.integrationEnabled() &&
      millis() - lastRefloatAllDataPollTime >= REFLOAT_ALLDATA_POLL_INTERVAL) {
    refloat.requestTelemetry(esc);
    lastRefloatAllDataPollTime = millis();
  }

  if (refloat.active &&
      refloat.needsLegacyBatteryRequest() &&
      (batteryDisplayActive ||
       (movingState && footpadModeUsesBattery(lencoLED.ridingFootpadMode)) ||
       (idleFootpadModeDisplayed && footpadModeUsesBattery(lencoLED.idleFootpadMode))) &&
      refloat.integrationEnabled() &&
      millis() - lastRefloatBatteryPollTime >= REFLOAT_BATTERY_POLL_INTERVAL) {
    esc.sendRefloatBatteryRequest();
    lastRefloatBatteryPollTime = millis();
  }
  batteryDisplayActive = false;

  // === Periodic CAN polling ===
  if (millis() - lastCanPollTime >= CAN_POLLING_INTERVAL) {
    if (!refloat.active) {
      esc.getRealtimeData();
    }

    lastCanPollTime = millis();

    globalErpm = refloat.active ? refloat.erpm : esc.erpm;
    globalVoltage = refloat.active ? refloat.voltage : esc.voltage;
    globalDutyCycle = refloat.active ? refloat.dutyCycle : esc.dutyCycle;
  }

  // === Use global data ===
  balanceBeeper.loop(globalDutyCycle, globalErpm, globalVoltage, lencoLED.lowVoltage);

  // === Determine direction and state ===
  bool refloatDisabledState = false;
  bool refloatFlywheelState = false;
  bool desiredStartupState = startupState;
  bool desiredMovingState = movingState;
  int desiredTargetBrightness = targetBrightness;
  float motionErpm = refloat.active ? refloat.erpm : globalErpm;

  if (refloat.active && refloat.state != REFLOAT_STATE_UNKNOWN) {
    uint8_t refloatState = refloat.state;
    refloatDisabledState = (refloatState == 15);
    refloatFlywheelState = (refloatState == 5);

    if (refloatState >= 1 && refloatState <= 3) {
      desiredStartupState = false;
      desiredMovingState = true;
      if (motionErpm > DIRECTION_ERPM_THRESHOLD) {
        direction = FORWARD;
      } else if (motionErpm < -DIRECTION_ERPM_THRESHOLD) {
        direction = REVERSE;
      }
      desiredTargetBrightness = NORMAL_BRIGHTNESS;
    } else if (!refloatDisabledState) {
      desiredStartupState = true;
      desiredMovingState = false;
      desiredTargetBrightness = STARTUP_BRIGHTNESS;
    } else {
      desiredStartupState = false;
      desiredMovingState = false;
      desiredTargetBrightness = STARTUP_BRIGHTNESS;
    }
  } else {
    if (motionErpm > MOVING_ERPM_THRESHOLD) {
      desiredStartupState = false;
      desiredMovingState = true;
      direction = FORWARD;
      desiredTargetBrightness = NORMAL_BRIGHTNESS;
    } else if (motionErpm < -MOVING_ERPM_THRESHOLD) {
      desiredStartupState = false;
      desiredMovingState = true;
      direction = REVERSE;
      desiredTargetBrightness = NORMAL_BRIGHTNESS;
    } else {
      desiredStartupState = true;
      desiredMovingState = false;
      desiredTargetBrightness = STARTUP_BRIGHTNESS;
    }
  }

  if (desiredMovingState != movingState) {
    unsigned long now = millis();
    if (!pendingMovingStateActive || pendingMovingState != desiredMovingState) {
      pendingMovingState = desiredMovingState;
      pendingMovingStateActive = true;
      pendingMovingStateMS = now;
    }
    if (now - pendingMovingStateMS >= STATE_TRANSITION_DEBOUNCE_MS) {
      if (movingState && !desiredMovingState && !startupState) {
        returningToStartup = true;
      }
      if (movingState != desiredMovingState) {
        isBraking = false;
        previousErpm = globalErpm;
      }
      startupState = desiredStartupState;
      movingState = desiredMovingState;
      targetBrightness = desiredTargetBrightness;
      pendingMovingStateActive = false;
    }
  } else {
    pendingMovingStateActive = false;
    startupState = desiredStartupState;
    targetBrightness = desiredTargetBrightness;
  }

  // === Detect state and direction transitions ===
  wasMovingState = movingState;

  if (direction != previousDirection) {
    requestDirectionFade(previousDirection);
    previousDirection = direction;
  }

  // === LED patterns ===
  if (ledFadeActive && millis() - ledFadeStartMS >= DIRECTION_FADE_DURATION_MS) {
    ledFadeActive = false;
  }

  bool lightsOn = refloat.lightsOn();
  if (refloatDisabledState) {
    fill_solid(forward_leds, lencoLED.numLedsForward, CRGB::Black);
    fill_solid(reverse_leds, lencoLED.numLedsReverse, CRGB::Black);
    disabledFootpadIndicator();
  } else {
    if (!lightsOn) {
      fill_solid(forward_leds, lencoLED.numLedsForward, CRGB::Black);
      fill_solid(reverse_leds, lencoLED.numLedsReverse, CRGB::Black);
    }

    if (lightsOn && refloatFlywheelState) {
      staticStartupLEDs();
      footpadRainbow();
    } else if (startupState) {
      processStartupAction();
    } else if (movingState) {
      if (lightsOn) {
        int rideLedCount = (direction == FORWARD) ? lencoLED.numLedsForward : lencoLED.numLedsReverse;
        clearActiveRideStrip();
        knightRider(lencoLED.ledColors[0][0], lencoLED.ledColors[0][1], lencoLED.ledColors[0][2], max(1, rideLedCount / 3));
        applyDirectionFade();
        renderRearLights();
        renderRidingFootpad();
      } else {
        fill_solid(footpad_leds, lencoLED.numLedsFootpad, CRGB::Black);
      }
    } else if (lightsOn) {
      applyDirectionFade();
    }
  }

  // === Brake logic ===
  if (!refloatDisabledState && !refloat.handtest && movingState && !startupState &&
      abs((int)globalErpm) >= BRAKE_IDLE_THRESHOLD &&
      millis() - lastBrakeCheckMillis >= brakeCheckInterval) {
    checkBraking();
    lastBrakeCheckMillis = millis();
  } else if (!movingState || startupState || abs((int)globalErpm) < BRAKE_IDLE_THRESHOLD) {
    isBraking = false;
    previousErpm = globalErpm;
  }

  // === Throttled LED update ===
  if (millis() - lastLEDUpdateMillis >= LED_UPDATE_INTERVAL) {
    currentBrightness += constrain(targetBrightness - currentBrightness, -5, 5);
    clearInactiveLEDs();
    float brightnessScale = 1.0f;
    if (refloatDisabledState) {
      brightnessScale = 1.0f;
    } else if (lightsOn) {
      brightnessScale = movingState ? refloat.ridingBrightnessScale(lencoLED.ridingBrightness) : refloat.idleBrightnessScale(lencoLED.idleBrightness);
    }
    int scaledBrightness = constrain((int)(currentBrightness * brightnessScale), 0, 255);
    FastLED.setBrightness(scaledBrightness);
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
}

void renderRearLights() {
  if (!refloat.lightsOn()) return;

  CRGB *leds_const = (direction == FORWARD) ? reverse_leds : forward_leds;
  int ledCount = (direction == FORWARD) ? lencoLED.numLedsReverse : lencoLED.numLedsForward;
  if (isBraking) {
    for (int i = 0; i < ledCount; i++) {
      leds_const[i].setRGB(lencoLED.ledColors[1][0], lencoLED.ledColors[1][1], lencoLED.ledColors[1][2]);
    }
  } else {
    for (int i = 0; i < ledCount; i++) {
      if (i % 2 == 0)
        leds_const[i].setRGB(lencoLED.ledColors[1][0], lencoLED.ledColors[1][1], lencoLED.ledColors[1][2]);
      else
        leds_const[i] = CRGB(0, 0, 0);
    }
  }
}

void knightRider(int red, int green, int blue, int ridingWidth) {
  CRGB *leds = (direction == FORWARD) ? forward_leds : reverse_leds;
  int ledCount = (direction == FORWARD) ? lencoLED.numLedsForward : lencoLED.numLedsReverse;
  ridingWidth = constrain(ridingWidth, 1, ledCount);
  int travel = max(0, ledCount - ridingWidth);

  const long IDLE_ERPM = 200;
  long erpm = max((long)abs(globalErpm), IDLE_ERPM);
  unsigned long delayDuration = (unsigned long)map(erpm, 200, 20000, 80, 5);
  delayDuration = constrain(delayDuration, 5UL, 250UL);

  if (millis() - lastKnightRiderUpdate >= delayDuration) {

    for (int i = 0; i < ledCount; i++) {
      leds[i].fadeToBlackBy(60);
    }

    rideRiderPosition = constrain(rideRiderPosition, 0, travel);

    for (int j = -2; j < ridingWidth + 2; j++) {
      int idx = rideRiderPosition + j;
      if (idx >= 0 && idx < ledCount) {
        int fadeFactor = (j < 0 || j >= ridingWidth) ? 30 : 100;
        leds[idx].setRGB(
          (red   * fadeFactor) / 100,
          (green * fadeFactor) / 100,
          (blue  * fadeFactor) / 100
        );
      }
    }

    rideRiderPosition += rideRiderDirection;

    if (rideRiderPosition >= travel) rideRiderDirection = -1;
    else if (rideRiderPosition <= 0)  rideRiderDirection =  1;

    lastKnightRiderUpdate = millis();
  }
}

void processStartupAction() {
  if (returningToStartup) {
    voltageAcquired = false;
    returningToStartup = false;
  }

  staticStartupLEDs();  // skips forward/reverse when lights_on is false

  if (!startupAnimationComplete) {
    startupAnimation();  // always runs regardless of lights_on
    return;
  }

  // Animation complete — remaining footpad patterns only show when LEDs are enabled.
  if (!refloat.lightsOn()) {
    fill_solid(footpad_leds, lencoLED.numLedsFootpad, CRGB::Black);
    return;
  }

  if (refloat.handtest && !movingState) {
    footpadHandtest();
    return;
  }

  if (globalVoltage < 0.1f) {
    warningLEDs();
    return;
  }

  if (globalVoltage >= 0.1f && (globalVoltage - lencoLED.lowVoltage) / (lencoLED.fullVoltage - lencoLED.lowVoltage) <= 0.10) {
    lowVoltageWarningLEDs();
    return;
  }

  if (!voltageAcquired && globalVoltage >= 0.1f) {
    voltageAcquired = true;
    voltageAcquiredMS = millis();
  }

  bool leftFootpad = refloat.leftFootpadPressed(esc, lencoLED.swapFootpadAdc);
  bool rightFootpad = refloat.rightFootpadPressed(esc, lencoLED.swapFootpadAdc);
  bool anyFootpad = leftFootpad || rightFootpad;
  if (leftFootpad && rightFootpad) {
    lastFootpadTriggerMillis = millis();
    isInitialStartup = false;
  }

  if (anyFootpad) {
    singleFootpadTriggeredStartupLEDs();
    return;
  }

  if (isInitialStartup && voltageAcquired && (millis() - voltageAcquiredMS > BATTERY_INDICATOR_DURATION)) {
    isInitialStartup = false;
  }

  bool showBatteryOnTimer   = voltageAcquired && (millis() - voltageAcquiredMS <= BATTERY_INDICATOR_DURATION);
  bool showBatteryOnFootpad = voltageAcquired && (millis() - lastFootpadTriggerMillis <= BATTERY_INDICATOR_DURATION);
  batteryDisplayActive = showBatteryOnTimer || showBatteryOnFootpad;

  if (showBatteryOnTimer || showBatteryOnFootpad) {
    batteryPercentStartupLEDs();
  } else {
    renderFootpadMode(lencoLED.idleFootpadMode, false);
  }
}

void startupAnimation() {
  unsigned long elapsed = millis() - startupBeginMS;

  if (elapsed >= STARTUP_ANIMATION_DURATION) {
    startupAnimationComplete = true;
    return;
  }

  int numLeds = map(elapsed, 0, STARTUP_ANIMATION_DURATION, 0, lencoLED.numLedsFootpad);
  numLeds = constrain(numLeds, 0, lencoLED.numLedsFootpad);

  for (int i = 0; i < lencoLED.numLedsFootpad; i++) {
    if (i < numLeds) {
      footpad_leds[i] = CRGB(lencoLED.ledColors[2][0], lencoLED.ledColors[2][1], lencoLED.ledColors[2][2]);
    } else {
      footpad_leds[i] = CRGB(0, 0, 0);
    }
  }
}

void staticStartupLEDs() {
  if (!refloat.lightsOn()) return;
  CRGB *frontLeds = (direction == FORWARD) ? forward_leds : reverse_leds;
  CRGB *rearLeds = (direction == FORWARD) ? reverse_leds : forward_leds;
  int frontCount = (direction == FORWARD) ? lencoLED.numLedsForward : lencoLED.numLedsReverse;
  int rearCount = (direction == FORWARD) ? lencoLED.numLedsReverse : lencoLED.numLedsForward;

  for (int i = 0; i < frontCount; i++) {
    frontLeds[i] = CRGB(lencoLED.ledColors[0][0], lencoLED.ledColors[0][1], lencoLED.ledColors[0][2]);
  }
  for (int i = 0; i < rearCount; i++) {
    rearLeds[i] = (i % 2 == 0)
        ? CRGB(lencoLED.ledColors[1][0], lencoLED.ledColors[1][1], lencoLED.ledColors[1][2])
        : CRGB(0, 0, 0);
  }
}

void lowVoltageWarningLEDs() {
  bool flashOn = (millis() / 250) % 2 == 0;
  for (int i = 0; i < lencoLED.numLedsFootpad; i++) {
    footpad_leds[i] = flashOn ? CRGB(255, 0, 0) : CRGB(0, 0, 0);
  }
}

void requestDirectionFade(int oldDirection) {
  ledFadeActive = true;
  fadingDirection = oldDirection;
  ledFadeStartMS = millis();
  lastKnightRiderUpdate = 0;
  directionStripClearPending = true;
  resetRideKnightRider();
}

void applyDirectionFade() {
  if (!ledFadeActive) return;

  CRGB *newLeds = (direction == FORWARD) ? forward_leds : reverse_leds;
  int newLedCount = (direction == FORWARD) ? lencoLED.numLedsForward : lencoLED.numLedsReverse;
  uint8_t fadeInAmount = (uint8_t)constrain(
      (int)map(millis() - ledFadeStartMS, 0, DIRECTION_FADE_DURATION_MS, 0, 255),
      0,
      255);

  for (int i = 0; i < newLedCount; i++) {
    newLeds[i].nscale8_video(fadeInAmount);
  }
}

void clearActiveRideStrip() {
  if (!directionStripClearPending) return;

  CRGB *leds = (direction == FORWARD) ? forward_leds : reverse_leds;
  int ledCount = (direction == FORWARD) ? lencoLED.numLedsForward : lencoLED.numLedsReverse;
  for (int i = 0; i < ledCount; i++) {
    leds[i] = CRGB::Black;
  }
  directionStripClearPending = false;
}

void resetRideKnightRider() {
  rideRiderPosition = 0;
  rideRiderDirection = 1;
}

void warningLEDs() {
  bool flashOn = (millis() / 400) % 2 == 0;
  for (int i = 0; i < lencoLED.numLedsFootpad; i++) {
    footpad_leds[i] = flashOn ? CRGB(255, 80, 0) : CRGB(0, 0, 0);
  }
}

void batteryPercentStartupLEDs() {
  double batteryVoltagePercentage = (globalVoltage - lencoLED.lowVoltage) / (lencoLED.fullVoltage - lencoLED.lowVoltage);
  bool usingRefloatBattery = refloat.active && refloat.batteryValid;
  if (usingRefloatBattery) {
    batteryVoltagePercentage = refloat.batteryPct;
  }

  if (!usingRefloatBattery && (batteryVoltagePercentage < -0.10 || batteryVoltagePercentage > 1.10)) {
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

  int numLedsLit = (int)(batteryVoltagePercentage * lencoLED.numLedsFootpad);
  for (int i = 0; i < lencoLED.numLedsFootpad; i++) {
    if (i < numLedsLit) {
      footpad_leds[i].setRGB(r, g, b);
    } else {
      footpad_leds[i].setRGB(lencoLED.ledColors[6][0], lencoLED.ledColors[6][1], lencoLED.ledColors[6][2]);
    }
  }
}

void singleFootpadTriggeredStartupLEDs() {
  const int half = lencoLED.numLedsFootpad / 2;
  bool leftFootpad = refloat.leftFootpadPressed(esc, lencoLED.swapFootpadAdc);
  bool rightFootpad = refloat.rightFootpadPressed(esc, lencoLED.swapFootpadAdc);

  if (leftFootpad || rightFootpad) {
    for (int i = 0; i < lencoLED.numLedsFootpad; i++) {
      if ((leftFootpad && i < half) || (rightFootpad && i >= half))
        footpad_leds[i].setRGB(lencoLED.ledColors[7][0], lencoLED.ledColors[7][1], lencoLED.ledColors[7][2]);
      else
        footpad_leds[i].setRGB(0, 0, 0);
    }
  }
}

bool footpadKnightRider() {
  static int   pos        = 0;
  static int   dir        = 1;
  static unsigned long lastUpdate = 0;
  static int   history[LencoLED::MAX_LED_COUNT] = {};
  static int   historyCount = 0;

  const int   ridingWidth = constrain((int)lencoLED.fpRidingWidth, 1, lencoLED.numLedsFootpad);
  const int   fadeDistance = (int)lencoLED.fpFadeAmount;
  const int   travel      = max(0, lencoLED.numLedsFootpad - ridingWidth);

  if (millis() - lastUpdate < lencoLED.fpAnimationDelay) return false;
  lastUpdate = millis();

  pos = constrain(pos, 0, travel);

  // Push current position into history (index 0 = most recent)
  int keep = min(fadeDistance, (int)LencoLED::MAX_LED_COUNT - 1);
  historyCount = min(historyCount + 1, keep + 1);
  for (int i = historyCount - 1; i > 0; i--) history[i] = history[i - 1];
  history[0] = pos;

  // Render oldest first so newer (brighter) entries overwrite where they overlap
  fill_solid(footpad_leds, lencoLED.numLedsFootpad, CRGB::Black);
  for (int h = historyCount - 1; h >= 0; h--) {
    uint8_t brightness;
    if (h == 0) {
      brightness = 255;
    } else {
      uint32_t num = (uint32_t)(keep + 1 - h) * (keep + 1 - h);
      uint32_t den = (uint32_t)(keep + 1) * (keep + 1);
      brightness = (uint8_t)(255UL * num / den);
    }
    for (int j = 0; j < ridingWidth; j++) {
      int idx = history[h] + j;
      if (idx >= 0 && idx < lencoLED.numLedsFootpad) {
        footpad_leds[idx].setRGB(
          (lencoLED.ledColors[8][0] * brightness) / 255,
          (lencoLED.ledColors[8][1] * brightness) / 255,
          (lencoLED.ledColors[8][2] * brightness) / 255
        );
      }
    }
  }

  if (pos >= travel) dir = -1;
  else if (pos <= 0)  dir =  1;
  pos += dir;
  return true;
}

bool footpadModeUsesAllData(uint8_t mode) {
  return mode == LencoLED::RIDING_FOOTPAD_CURRENT ||
         mode == LencoLED::RIDING_FOOTPAD_ROLL ||
         mode == LencoLED::RIDING_FOOTPAD_FET_TEMP ||
         mode == LencoLED::RIDING_FOOTPAD_MOTOR_TEMP ||
         mode == LencoLED::RIDING_FOOTPAD_SPEED;
}

bool footpadModeUsesBattery(uint8_t mode) {
  return mode == LencoLED::RIDING_FOOTPAD_BATTERY;
}

void renderFootpadMode(uint8_t mode, bool applyStatusBrightness) {
  if (footpadModeUsesAllData(mode) && !refloat.active) {
    fill_solid(footpad_leds, lencoLED.numLedsFootpad, CRGB::Black);
    return;
  }

  bool updated = true;
  switch (mode) {
    case LencoLED::RIDING_FOOTPAD_BATTERY:
      batteryPercentStartupLEDs();
      break;
    case LencoLED::RIDING_FOOTPAD_NONE:
      fill_solid(footpad_leds, lencoLED.numLedsFootpad, CRGB::Black);
      break;
    case LencoLED::RIDING_FOOTPAD_KNIGHTRIDER:
      updated = footpadKnightRider();
      break;
    case LencoLED::RIDING_FOOTPAD_CURRENT:
      footpadCurrentIndicator();
      break;
    case LencoLED::RIDING_FOOTPAD_ROLL:
      footpadRollIndicator();
      break;
    case LencoLED::RIDING_FOOTPAD_SPEED:
      footpadSpeedIndicator();
      break;
    case LencoLED::RIDING_FOOTPAD_FET_TEMP:
      footpadFetTempIndicator();
      break;
    case LencoLED::RIDING_FOOTPAD_MOTOR_TEMP:
      footpadMotorTempIndicator();
      break;
    case LencoLED::RIDING_FOOTPAD_DUTY:
    default:
      footpadDutyCycleIndicator();
      break;
  }

  if (updated && applyStatusBrightness && mode != LencoLED::RIDING_FOOTPAD_NONE) {
    scaleFootpadLeds(refloat.statusBrightnessScale(lencoLED.statusBrightness));
  }
}

void renderRidingFootpad() {
  renderFootpadMode(lencoLED.ridingFootpadMode, true);
}

void footpadTelemetryBar(float value, int16_t minValue, int16_t maxValue, uint8_t colorIndex) {
  fill_solid(footpad_leds, lencoLED.numLedsFootpad, CRGB::Black);
  if (maxValue <= minValue) return;

  float normalized = (value - minValue) / (float)(maxValue - minValue);
  normalized = constrain(normalized, 0.0f, 1.0f);
  int ledsToLight = constrain((int)(normalized * lencoLED.numLedsFootpad), 0, (int)lencoLED.numLedsFootpad);

  for (int i = 0; i < ledsToLight; i++) {
    footpad_leds[i].setRGB(
      lencoLED.ledColors[colorIndex][0],
      lencoLED.ledColors[colorIndex][1],
      lencoLED.ledColors[colorIndex][2]
    );
  }
}

void footpadCurrentIndicator() {
  fill_solid(footpad_leds, lencoLED.numLedsFootpad, CRGB::Black);
  if (lencoLED.currentMax <= lencoLED.currentMin || lencoLED.numLedsFootpad < 2) return;

  if (lencoLED.currentMin < 0 && lencoLED.currentMax > 0) {
    int zeroIdx = constrain((int)((0 - lencoLED.currentMin) / (float)(lencoLED.currentMax - lencoLED.currentMin) * lencoLED.numLedsFootpad), 0, (int)lencoLED.numLedsFootpad - 1);
    if (refloat.currentIn >= 0) {
      int ledsToLight = constrain((int)((refloat.currentIn / lencoLED.currentMax) * (lencoLED.numLedsFootpad - zeroIdx)), 0, (int)lencoLED.numLedsFootpad - zeroIdx);
      for (int i = 0; i < ledsToLight; i++) {
        int idx = zeroIdx + i;
        footpad_leds[idx].setRGB(lencoLED.ledColors[12][0], lencoLED.ledColors[12][1], lencoLED.ledColors[12][2]);
      }
    } else {
      int ledsToLight = constrain((int)((abs(refloat.currentIn) / abs(lencoLED.currentMin)) * (zeroIdx + 1)), 0, zeroIdx + 1);
      for (int i = 0; i < ledsToLight; i++) {
        int idx = zeroIdx - i;
        footpad_leds[idx].setRGB(lencoLED.ledColors[13][0], lencoLED.ledColors[13][1], lencoLED.ledColors[13][2]);
      }
    }
    return;
  }

  footpadTelemetryBar(refloat.currentIn, lencoLED.currentMin, lencoLED.currentMax, 12);
}

void footpadRollIndicator() {
  fill_solid(footpad_leds, lencoLED.numLedsFootpad, CRGB::Black);
  if (lencoLED.rollMax == 0 || lencoLED.numLedsFootpad < 2) return;

  int center = lencoLED.numLedsFootpad / 2;
  int sideLeds = lencoLED.numLedsFootpad / 2;
  int ledsToLight = constrain((int)((abs(refloat.roll) / lencoLED.rollMax) * sideLeds), 0, sideLeds);

  for (int i = 0; i < ledsToLight; i++) {
    int idx = refloat.roll > 0 ? center - 1 - i : center + i;
    if (idx >= 0 && idx < lencoLED.numLedsFootpad) {
      footpad_leds[idx].setRGB(
        lencoLED.ledColors[14][0],
        lencoLED.ledColors[14][1],
        lencoLED.ledColors[14][2]
      );
    }
  }
}

void footpadSpeedIndicator() {
  fill_solid(footpad_leds, lencoLED.numLedsFootpad, CRGB::Black);
  const float maxSpeedKmh = 40.0f;
  float normalized = constrain(refloat.speedKmh / maxSpeedKmh, 0.0f, 1.0f);
  int ledsToLight = constrain((int)(normalized * lencoLED.numLedsFootpad), 0, (int)lencoLED.numLedsFootpad);
  uint8_t colorIndex = normalized < 0.5f ? 9 : (normalized < 0.8f ? 10 : 11);
  for (int i = 0; i < ledsToLight; i++) {
    footpad_leds[i].setRGB(
      lencoLED.ledColors[colorIndex][0],
      lencoLED.ledColors[colorIndex][1],
      lencoLED.ledColors[colorIndex][2]
    );
  }
}

void footpadFetTempIndicator() {
  footpadTelemetryBar(refloat.fetTemp, lencoLED.fetTempMin, lencoLED.fetTempMax, 15);
}

void footpadMotorTempIndicator() {
  footpadTelemetryBar(refloat.motorTemp, lencoLED.motorTempMin, lencoLED.motorTempMax, 16);
}

void footpadHandtest() {
  fill_solid(footpad_leds, lencoLED.numLedsFootpad, CRGB::Black);
  if (lencoLED.numLedsFootpad == 0) return;

  int c = lencoLED.numLedsFootpad / 2;
  int start = max(0, c - 2);
  int end = min((int)lencoLED.numLedsFootpad - 1, c + 1);

  CRGB centerColor = (millis() / 500) % 2 == 0 ? CRGB(200, 0, 0) : CRGB(200, 70, 0);
  for (int i = start; i <= end; i++) {
    footpad_leds[i] = centerColor;
  }

  bool leftTriggered = refloat.leftFootpadPressed(esc, lencoLED.swapFootpadAdc);
  bool rightTriggered = refloat.rightFootpadPressed(esc, lencoLED.swapFootpadAdc);

  if (leftTriggered)
    footpad_leds[0].setRGB(lencoLED.ledColors[7][0], lencoLED.ledColors[7][1], lencoLED.ledColors[7][2]);
  if (rightTriggered)
    footpad_leds[lencoLED.numLedsFootpad - 1].setRGB(lencoLED.ledColors[7][0], lencoLED.ledColors[7][1], lencoLED.ledColors[7][2]);
}

void footpadRainbow() {
  static uint8_t rainbowHue = 0;
  static unsigned long lastRainbowUpdate = 0;
  if (millis() - lastRainbowUpdate < 30) return;
  lastRainbowUpdate = millis();
  rainbowHue += 2;

  for (int i = 0; i < lencoLED.numLedsFootpad; i++) {
    uint8_t hue = rainbowHue + (uint8_t)((uint16_t)i * 255 / lencoLED.numLedsFootpad);
    footpad_leds[i] = CHSV(hue, 255, 200);
  }
  scaleFootpadLeds(refloat.statusBrightnessScale(lencoLED.statusBrightness));
}

void disabledFootpadIndicator() {
  fill_solid(footpad_leds, lencoLED.numLedsFootpad, CRGB::Black);
  if (lencoLED.numLedsFootpad == 0) return;

  int c = lencoLED.numLedsFootpad / 2;
  if (c > 0) {
    footpad_leds[c - 1] = CRGB(255, 0, 0);
  }
  footpad_leds[c] = CRGB(255, 0, 0);
}

void scaleFootpadLeds(float scale) {
  uint8_t amount = constrain((int)(scale * 255.0f), 0, 255);
  if (amount == 0) {
    fill_solid(footpad_leds, lencoLED.numLedsFootpad, CRGB::Black);
    return;
  }
  for (int i = 0; i < lencoLED.numLedsFootpad; i++) {
    footpad_leds[i].nscale8_video(amount);
  }
}

void footpadDutyCycleIndicator() {
  double dutyAbs = abs(globalDutyCycle);
  dutyAbs = constrain(dutyAbs, 0.0, 1.0);

  int numLedsToLight = (int)(dutyAbs * lencoLED.numLedsFootpad);
  numLedsToLight = constrain(numLedsToLight, 0, lencoLED.numLedsFootpad);

  int r, g, b;
  float midThreshold = lencoLED.dutyMidThreshold / 100.0f;
  float highThreshold = lencoLED.dutyHighThreshold / 100.0f;

  if (dutyAbs >= highThreshold) {
    r = lencoLED.ledColors[11][0]; g = lencoLED.ledColors[11][1]; b = lencoLED.ledColors[11][2];
  } else if (dutyAbs >= midThreshold) {
    r = lencoLED.ledColors[10][0]; g = lencoLED.ledColors[10][1]; b = lencoLED.ledColors[10][2];
  } else {
    r = lencoLED.ledColors[9][0];  g = lencoLED.ledColors[9][1];  b = lencoLED.ledColors[9][2];
  }

  for (int i = 0; i < lencoLED.numLedsFootpad; i++) {
    if (i < numLedsToLight) {
      footpad_leds[i].setRGB(r, g, b);
    } else {
      footpad_leds[i].setRGB(0, 0, 0);
    }
  }
}

void clearInactiveLEDs() {
  for (int i = lencoLED.numLedsForward; i < MAX_LEDS_PER_STRIP; i++) {
    forward_leds[i] = CRGB::Black;
  }
  for (int i = lencoLED.numLedsReverse; i < MAX_LEDS_PER_STRIP; i++) {
    reverse_leds[i] = CRGB::Black;
  }
  for (int i = lencoLED.numLedsFootpad; i < MAX_LEDS_PER_STRIP; i++) {
    footpad_leds[i] = CRGB::Black;
  }
}
