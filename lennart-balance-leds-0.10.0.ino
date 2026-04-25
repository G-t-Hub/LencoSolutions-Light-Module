#include <Arduino.h>
#include <FastLED.h>

#undef SPI_CLOCK     // Prevent FastLED/MCP2515 macro conflict

#include "balance_beeper.cpp"
#include "esc.cpp"   // includes your updated ESC class

// LED color defaults are defined in esc.cpp (esc.ledColors[id][r/g/b])
// and can be updated at runtime via LencoLED CAN commands.
// Color group IDs: 0=Forward, 1=Reverse, 2=Startup, 3=BattHigh, 4=BattMid,
// 5=BattLow, 6=BattEmpty, 7=FootpadIndicator, 8=FootpadKnightRider,
// 9=Duty0-70, 10=Duty70-80, 11=Duty>80

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

void setup() {
  // Serial.begin(115200);
  esc.setup();
  balanceBeeper.setup();

  FastLED.addLeds<WS2812B, FORWARD_PIN, GRB>(forward_leds, NUM_LEDS)
      .setCorrection(TypicalLEDStrip);
  FastLED.addLeds<WS2812B, REVERSE_PIN, GRB>(reverse_leds, NUM_LEDS)
      .setCorrection(TypicalLEDStrip);
  FastLED.addLeds<WS2812B, FOOTPAD_PIN, GRB>(footpad_leds, NUM_LEDS_FOOTPAD)
      .setCorrection(TypicalLEDStrip);

  FastLED.setMaxPowerInVoltsAndMilliamps(5, 1500);
  FastLED.setBrightness(STARTUP_BRIGHTNESS);
  FastLED.clear();

  // Initial LED pattern
  for (int i = 0; i < NUM_LEDS; i++) {
    forward_leds[i] = CRGB(esc.ledColors[0][0], esc.ledColors[0][1], esc.ledColors[0][2]);
    reverse_leds[i] = (i % 2 == 0)
        ? CRGB(esc.ledColors[1][0], esc.ledColors[1][1], esc.ledColors[1][2])
        : CRGB(0, 0, 0);
  }

  startupBeginMS = millis();

  FastLED.show();
}

void loop() {
  
  // Passive listening for status 6;
  esc.listenForMessages();

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
  balanceBeeper.loop(globalDutyCycle, globalErpm, globalVoltage, esc.lowVoltage);

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
  if (!esc.ledEnabled) {
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
    knightRider(esc.ledColors[0][0], esc.ledColors[0][1], esc.ledColors[0][2], 5);
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

  CRGB *leds_const = (direction == FORWARD) ? reverse_leds : forward_leds;
  if (isBraking) {
    for (int i = 0; i < NUM_LEDS; i++) {
      leds_const[i].setRGB(esc.ledColors[1][0], esc.ledColors[1][1], esc.ledColors[1][2]);
    }
  } else {
    for (int i = 0; i < NUM_LEDS; i++) {
      if (i % 2 == 0)
        leds_const[i].setRGB(esc.ledColors[1][0], esc.ledColors[1][1], esc.ledColors[1][2]);
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
  if (globalVoltage > 0.0 && (globalVoltage - esc.lowVoltage) / (esc.fullVoltage - esc.lowVoltage) <= 0.10) {
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
      footpad_leds[i] = CRGB(esc.ledColors[2][0], esc.ledColors[2][1], esc.ledColors[2][2]);
    } else {
      footpad_leds[i] = CRGB(0, 0, 0);
    }
  }
}

void staticStartupLEDs() {
     // Static startup LEDs
  for (int i = 0; i < NUM_LEDS; i++) {
    if (direction == FORWARD) {
      forward_leds[i] = CRGB(esc.ledColors[0][0], esc.ledColors[0][1], esc.ledColors[0][2]);
      reverse_leds[i] = (i % 2 == 0)
          ? CRGB(esc.ledColors[1][0], esc.ledColors[1][1], esc.ledColors[1][2])
          : CRGB(0, 0, 0);
    } else {
      reverse_leds[i] = CRGB(esc.ledColors[0][0], esc.ledColors[0][1], esc.ledColors[0][2]); //swapped due to inverse direction
      forward_leds[i] = (i % 2 == 0)
          ? CRGB(esc.ledColors[1][0], esc.ledColors[1][1], esc.ledColors[1][2])
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
  double batteryVoltagePercentage = (globalVoltage - esc.lowVoltage) / (esc.fullVoltage - esc.lowVoltage);

  // Voltage is outside the expected range — likely misconfigured voltage constants
  if (batteryVoltagePercentage < -0.10 || batteryVoltagePercentage > 1.10) {
    warningLEDs();
    return;
  }

  // Low voltage (≤ 10%) is handled as Priority 2 in processStartupAction before we get here.

  batteryVoltagePercentage = constrain(batteryVoltagePercentage, 0.0, 1.0);

  int r, g, b;
  if (batteryVoltagePercentage <= 0.20) {
    r = esc.ledColors[5][0]; g = esc.ledColors[5][1]; b = esc.ledColors[5][2]; // Battery Low
  } else if (batteryVoltagePercentage <= 0.40) {
    r = esc.ledColors[4][0]; g = esc.ledColors[4][1]; b = esc.ledColors[4][2]; // Battery Mid
  } else {
    r = esc.ledColors[3][0]; g = esc.ledColors[3][1]; b = esc.ledColors[3][2]; // Battery High
  }

  int numLedsLit = (int)(batteryVoltagePercentage * NUM_LEDS_FOOTPAD);
  for (int i = 0; i < NUM_LEDS_FOOTPAD; i++) {
    if (i < numLedsLit) {
      footpad_leds[i].setRGB(r, g, b);
    } else {
      footpad_leds[i].setRGB(esc.ledColors[6][0], esc.ledColors[6][1], esc.ledColors[6][2]); // Battery Empty
    }
  }
}

void singleFootpadTriggeredStartupLEDs() {
  
  if (esc.adc1 > esc.footpadThreshold)
  {
    for (int i = 0; i < NUM_LEDS_FOOTPAD; i++)
    {
      if (i < NUM_LEDS_FOOTPAD/2){
        footpad_leds[i].setRGB(esc.ledColors[7][0], esc.ledColors[7][1], esc.ledColors[7][2]);
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
        footpad_leds[i].setRGB(esc.ledColors[7][0], esc.ledColors[7][1], esc.ledColors[7][2]);
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
          (esc.ledColors[8][0] * fadeFactor) / 100,
          (esc.ledColors[8][1] * fadeFactor) / 100,
          (esc.ledColors[8][2] * fadeFactor) / 100
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
    r = esc.ledColors[11][0]; g = esc.ledColors[11][1]; b = esc.ledColors[11][2]; // Duty > 80%
  } else if (dutyAbs >= 70.0) {
    r = esc.ledColors[10][0]; g = esc.ledColors[10][1]; b = esc.ledColors[10][2]; // Duty 70-80%
  } else {
    r = esc.ledColors[9][0];  g = esc.ledColors[9][1];  b = esc.ledColors[9][2];  // Duty 0-70%
  }

  for (int i = 0; i < NUM_LEDS_FOOTPAD; i++) {
    if (i < numLedsToLight) {
      footpad_leds[i].setRGB(r, g, b);
    } else {
      footpad_leds[i].setRGB(0, 0, 0);
    }
  }
}