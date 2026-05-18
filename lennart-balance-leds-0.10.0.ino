#include <Arduino.h>
#include <FastLED.h>
#include <EEPROM.h>

#undef SPI_CLOCK     // Prevent FastLED/MCP2515 macro conflict

#include "balance_beeper.cpp"
#include "esc.cpp"
#include "lencoled.cpp"

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

CRGB forward_leds[MAX_LEDS_PER_STRIP];
CRGB reverse_leds[MAX_LEDS_PER_STRIP];
CRGB footpad_leds[MAX_LEDS_PER_STRIP];

ESC esc;
BalanceBeeper balanceBeeper;
LencoLED lencoLED;

// Global variables for ESC data
float globalErpm = 0.0f;
float globalVoltage = 0.0f;
float globalDutyCycle = 0.0f;

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

int direction = FORWARD;
int previousDirection = FORWARD;
int previousErpm = 0;

bool ledFadeActive = false;
bool fadeForwardReverse = false;
bool fadeFootpad = false;
unsigned long ledFadeStartMS = 0;
bool wasMovingState = false;

int currentBrightness = STARTUP_BRIGHTNESS;
int targetBrightness = STARTUP_BRIGHTNESS;

// Footpad animation (per-animation state lives as statics inside each function)

bool startupState = true;
bool movingState = false;
bool isBraking = false;

void knightRider(int red, int green, int blue, int ridingWidth);
void checkBraking();
bool footpadKnightRider();
void footpadDutyCycleIndicator();
void renderRidingFootpad();
void footpadHandtest();
void footpadRainbow();
void disabledFootpadIndicator();
void scaleFootpadLeds(float scale);
void requestLEDFade(bool forwardReverse, bool footpad);
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
    lencoLED.handleLcmPoll(esc.lcmPollData, esc.lcmPollLen);
    esc.lcmPollAvailable = false;
  }
  lencoLED.updateRefloatState();

  // === Periodic LCM poll (Refloat light control) ===
  if (millis() - lastLcmPollTime >= LCM_POLL_INTERVAL) {
    esc.sendLcmPoll();
    lastLcmPollTime = millis();
  }

  // === Periodic CAN polling ===
  if (millis() - lastCanPollTime >= CAN_POLLING_INTERVAL) {
    esc.getRealtimeData();

    lastCanPollTime = millis();

    globalErpm = lencoLED.refloat_active ? lencoLED.refloat_erpm : esc.erpm;
    globalVoltage = lencoLED.refloat_active ? lencoLED.refloat_voltage : esc.voltage;
    globalDutyCycle = esc.dutyCycle;
  }

  // === Use global data ===
  balanceBeeper.loop(globalDutyCycle, globalErpm, globalVoltage, lencoLED.lowVoltage);

  // === Determine direction and state ===
  bool refloatDisabledState = false;
  bool refloatFlywheelState = false;

  if (lencoLED.refloat_active && lencoLED.refloat_state != REFLOAT_STATE_UNKNOWN) {
    uint8_t refloatState = lencoLED.refloat_state;
    refloatDisabledState = (refloatState == 15);
    refloatFlywheelState = (refloatState == 5);

    if (refloatState >= 1 && refloatState <= 3) {
      startupState = false;
      movingState = true;
      if (globalErpm > 200) {
        direction = FORWARD;
      } else if (globalErpm < -200) {
        direction = REVERSE;
      }
      targetBrightness = NORMAL_BRIGHTNESS;
    } else if (!refloatDisabledState) {
      if (movingState && !startupState)
      {
        returningToStartup = true;
      }
      startupState = true;
      movingState = false;
      targetBrightness = STARTUP_BRIGHTNESS;
    } else {
      if (movingState && !startupState)
      {
        returningToStartup = true;
      }
      startupState = false;
      movingState = false;
      targetBrightness = STARTUP_BRIGHTNESS;
    }
  } else {
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

  bool lightsOn = lencoLED.lightsOn();
  if (refloatDisabledState) {
    fill_solid(forward_leds, lencoLED.numLedsForward, CRGB::Black);
    fill_solid(reverse_leds, lencoLED.numLedsReverse, CRGB::Black);
    disabledFootpadIndicator();
  } else {
    if (!lightsOn) {
      fill_solid(forward_leds, lencoLED.numLedsForward, CRGB::Black);
      fill_solid(reverse_leds, lencoLED.numLedsReverse, CRGB::Black);
    }

    if (lightsOn && ledFadeActive) {
      if (fadeForwardReverse) {
        for (int i = 0; i < lencoLED.numLedsForward; i++) {
          forward_leds[i].fadeToBlackBy(60);
        }
        for (int i = 0; i < lencoLED.numLedsReverse; i++) {
          reverse_leds[i].fadeToBlackBy(60);
        }
      }
      if (fadeFootpad) {
        for (int i = 0; i < lencoLED.numLedsFootpad; i++) {
          footpad_leds[i].fadeToBlackBy(60);
        }
      }
    } else if (lightsOn && refloatFlywheelState && !ledFadeActive) {
      staticStartupLEDs();
      footpadRainbow();
    } else if (!ledFadeActive && startupState) {
      processStartupAction();
    } else if (!ledFadeActive && movingState) {
      if (lightsOn) {
        int rideLedCount = (direction == FORWARD) ? lencoLED.numLedsForward : lencoLED.numLedsReverse;
        knightRider(lencoLED.ledColors[0][0], lencoLED.ledColors[0][1], lencoLED.ledColors[0][2], max(1, rideLedCount / 3));
        renderRidingFootpad();
      } else {
        fill_solid(footpad_leds, lencoLED.numLedsFootpad, CRGB::Black);
      }
    }
  }

  // === Brake logic ===
  if (!refloatDisabledState && !lencoLED.refloat_handtest && millis() - lastBrakeCheckMillis >= brakeCheckInterval) {
    checkBraking();
    lastBrakeCheckMillis = millis();
  }

  // === Throttled LED update ===
  if (millis() - lastLEDUpdateMillis >= LED_UPDATE_INTERVAL) {
    currentBrightness += constrain(targetBrightness - currentBrightness, -5, 5);
    clearInactiveLEDs();
    float brightnessScale = 1.0f;
    if (refloatDisabledState) {
      brightnessScale = 1.0f;
    } else if (lightsOn) {
      brightnessScale = movingState ? lencoLED.ridingBrightnessScale() : lencoLED.idleBrightnessScale();
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

  if (!lencoLED.lightsOn()) return;

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
  static int pos = 0;
  static int animDir = 1;

  CRGB *leds = (direction == FORWARD) ? forward_leds : reverse_leds;
  int ledCount = (direction == FORWARD) ? lencoLED.numLedsForward : lencoLED.numLedsReverse;
  ridingWidth = constrain(ridingWidth, 1, ledCount);
  int travel = max(0, ledCount - ridingWidth);

  const long IDLE_ERPM = 200;
  if (abs(globalErpm) < IDLE_ERPM) {
    for (int i = 0; i < ledCount; i++) {
      leds[i].fadeToBlackBy(40);
    }
    clearInactiveLEDs();
    FastLED.show();
    return;
  }

  long erpm = abs(globalErpm);
  unsigned long delayDuration = (unsigned long)map(erpm, 200, 20000, 80, 5);
  delayDuration = constrain(delayDuration, 5UL, 250UL);

  if (millis() - lastKnightRiderUpdate >= delayDuration) {

    for (int i = 0; i < ledCount; i++) {
      leds[i].fadeToBlackBy(60);
    }

    pos = constrain(pos, 0, travel);

    for (int j = -2; j < ridingWidth + 2; j++) {
      int idx = pos + j;
      if (idx >= 0 && idx < ledCount) {
        int fadeFactor = (j < 0 || j >= ridingWidth) ? 30 : 100;
        leds[idx].setRGB(
          (red   * fadeFactor) / 100,
          (green * fadeFactor) / 100,
          (blue  * fadeFactor) / 100
        );
      }
    }

    pos += animDir;

    if (pos >= travel) animDir = -1;
    else if (pos <= 0)  animDir =  1;

    clearInactiveLEDs();
    FastLED.show();
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
  if (!lencoLED.lightsOn()) {
    fill_solid(footpad_leds, lencoLED.numLedsFootpad, CRGB::Black);
    return;
  }

  if (lencoLED.refloat_handtest && !movingState) {
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

  bool bothFootpads = lencoLED.refloat_active
      ? lencoLED.refloat_footpad == FOOT_BOTH
      : (esc.adc1 > esc.footpadThreshold && esc.adc2 > esc.footpadThreshold);
  if (bothFootpads) {
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
    bool onlyOneFootpad = lencoLED.refloat_active
        ? (lencoLED.refloat_footpad == FOOT_LEFT || lencoLED.refloat_footpad == FOOT_RIGHT)
        : ((esc.adc1 > esc.footpadThreshold) != (esc.adc2 > esc.footpadThreshold));
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
  if (!lencoLED.lightsOn()) return;
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

void requestLEDFade(bool forwardReverse, bool footpad) {
  ledFadeActive = true;
  fadeForwardReverse = forwardReverse;
  fadeFootpad = footpad;
  ledFadeStartMS = millis();
}

void warningLEDs() {
  bool flashOn = (millis() / 400) % 2 == 0;
  for (int i = 0; i < lencoLED.numLedsFootpad; i++) {
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
  bool leftFootpad = lencoLED.refloat_active
      ? (lencoLED.refloat_footpad == FOOT_LEFT)
      : (esc.adc1 > esc.footpadThreshold);
  bool rightFootpad = lencoLED.refloat_active
      ? (lencoLED.refloat_footpad == FOOT_RIGHT)
      : (esc.adc2 > esc.footpadThreshold);

  if (leftFootpad) {
    for (int i = 0; i < lencoLED.numLedsFootpad; i++) {
      if (i < half)
        footpad_leds[i].setRGB(lencoLED.ledColors[7][0], lencoLED.ledColors[7][1], lencoLED.ledColors[7][2]);
      else
        footpad_leds[i].setRGB(0, 0, 0);
    }
  } else if (rightFootpad) {
    for (int i = 0; i < lencoLED.numLedsFootpad; i++) {
      if (i >= half)
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

void renderRidingFootpad() {
  bool updated = true;
  switch (lencoLED.ridingFootpadMode) {
    case LencoLED::RIDING_FOOTPAD_BATTERY:
      batteryPercentStartupLEDs();
      break;
    case LencoLED::RIDING_FOOTPAD_NONE:
      fill_solid(footpad_leds, lencoLED.numLedsFootpad, CRGB::Black);
      break;
    case LencoLED::RIDING_FOOTPAD_KNIGHTRIDER:
      updated = footpadKnightRider();
      break;
    case LencoLED::RIDING_FOOTPAD_DUTY:
    default:
      footpadDutyCycleIndicator();
      break;
  }

  if (updated && lencoLED.ridingFootpadMode != LencoLED::RIDING_FOOTPAD_NONE) {
    scaleFootpadLeds(lencoLED.statusBrightnessScale());
  }
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

  bool leftTriggered = lencoLED.refloat_active
      ? (lencoLED.refloat_footpad == FOOT_LEFT || lencoLED.refloat_footpad == FOOT_BOTH)
      : (esc.adc1 > esc.footpadThreshold);
  bool rightTriggered = lencoLED.refloat_active
      ? (lencoLED.refloat_footpad == FOOT_RIGHT || lencoLED.refloat_footpad == FOOT_BOTH)
      : (esc.adc2 > esc.footpadThreshold);

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
  scaleFootpadLeds(lencoLED.statusBrightnessScale());
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
  dutyAbs = constrain(dutyAbs, 0.0, 100.0);

  int numLedsToLight = (int)((dutyAbs / 100.0) * lencoLED.numLedsFootpad);
  numLedsToLight = constrain(numLedsToLight, 0, lencoLED.numLedsFootpad);

  int r, g, b;
  if (dutyAbs >= 80.0) {
    r = lencoLED.ledColors[11][0]; g = lencoLED.ledColors[11][1]; b = lencoLED.ledColors[11][2];
  } else if (dutyAbs >= 70.0) {
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
