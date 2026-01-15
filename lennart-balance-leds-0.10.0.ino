#include <Arduino.h>
#include <FastLED.h>

#undef SPI_CLOCK     // Prevent FastLED/MCP2515 macro conflict

#include "balance_beeper.cpp"
#include "esc.cpp"   // includes your updated ESC class

// Front LEDs (U1)
#define FLASHING_LED_RED 228
#define FLASHING_LED_GREEN 158
#define FLASHING_LED_BLUE 0

// Rear LEDs (U2)
#define CONSTANT_LED_RED 255
#define CONSTANT_LED_GREEN 0
#define CONSTANT_LED_BLUE 0
 
// The color of the LEDs for startup animation
#define STARTUP_ANIMATION_LED_RED 50
#define STARTUP_ANIMATION_LED_GREEN 205
#define STARTUP_ANIMATION_LED_BLUE 50

// The color of the LEDs showing the battery percent
#define BATTERY_INDICATOR_LED_RED 0
#define BATTERY_INDICATOR_LED_GREEN 255
#define BATTERY_INDICATOR_LED_BLUE 0

// After battery Percent LEDs above, color of the remaining LEDs in the bar
#define BATTERY_INDICATOR_ALTERNATE_LED_RED 0
#define BATTERY_INDICATOR_ALTERNATE_LED_GREEN 0
#define BATTERY_INDICATOR_ALTERNATE_LED_BLUE 255

// When a single footpad is pressed, what color should it light up
#define FOOTPAD_INDICATOR_LED_RED 0
#define FOOTPAD_INDICATOR_LED_GREEN 0
#define FOOTPAD_INDICATOR_LED_BLUE 255

#define BATTERY_INDICATOR_DURATION 5000 // 5 seconds
#define STARTUP_ANIMATION_DURATION 5000 // 5 seconds

#define THRESHOLD 5000
#define FAST_DELAY 20
#define SLOW_DELAY 50
#define STARTUP_BRIGHTNESS 30 
#define NORMAL_BRIGHTNESS 255 

#define NUM_LEDS 17 
#define FORWARD_PIN 5
#define REVERSE_PIN 6
#define FORWARD 0
#define REVERSE 1

#define BRAKE_IDLE_THRESHOLD 200
#define BRAKE_THRESHOLD 15
#define BRAKE_ON_DEBOUNCE_COUNT 3 
#define BRAKE_OFF_DEBOUNCE_COUNT 3

CRGB forward_leds[NUM_LEDS];
CRGB reverse_leds[NUM_LEDS];

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
int animationDirFlag = 1;
int previousErpm = 0;

bool startupState = true; 
bool movingState = false; 
bool isBraking = false;

void knightRider(int red, int green, int blue, int ridingWidth);
void checkBraking();

void setup() {
  // Serial.begin(115200);
  esc.setup();
  balanceBeeper.setup();

  FastLED.addLeds<WS2812B, FORWARD_PIN, GRB>(forward_leds, NUM_LEDS)
      .setCorrection(TypicalLEDStrip);
  FastLED.addLeds<WS2812B, REVERSE_PIN, GRB>(reverse_leds, NUM_LEDS)
      .setCorrection(TypicalLEDStrip);

  FastLED.setMaxPowerInVoltsAndMilliamps(5, 1500);
  FastLED.setBrightness(STARTUP_BRIGHTNESS);
  FastLED.clear();

  // Initial LED pattern
  for (int i = 0; i < NUM_LEDS; i++) {
    forward_leds[i] = CRGB(FLASHING_LED_RED, FLASHING_LED_GREEN, FLASHING_LED_BLUE);
    reverse_leds[i] = (i % 2 == 0)
        ? CRGB(CONSTANT_LED_RED, CONSTANT_LED_GREEN, CONSTANT_LED_BLUE)
        : CRGB(0, 0, 0);
  }

  startupBeginMS = millis();

  FastLED.show();
}

void loop() {
  
  // Passive listenin for status 6;
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
  balanceBeeper.loop(globalDutyCycle, globalErpm, globalVoltage);

  // === Determine direction and state ===
  if (globalErpm > 200) {
    startupState = false;
    movingState = true;
    direction = FORWARD;
    FastLED.setBrightness(NORMAL_BRIGHTNESS);
  } else if (globalErpm < -200) {
    startupState = false;
    movingState = true;
    direction = REVERSE;
    FastLED.setBrightness(NORMAL_BRIGHTNESS);
  } else {
    if (movingState && !startupState)
    {
      returningToStartup = true;
    }
    startupState = true;
    movingState = false;
    FastLED.setBrightness(STARTUP_BRIGHTNESS);
  }

  // === LED patterns ===
  if (startupState) {
    processStartupAction();
  } else if (movingState) {
    knightRider(FLASHING_LED_RED, FLASHING_LED_GREEN, FLASHING_LED_BLUE, 5);
  }

  // === Brake logic ===
  if (millis() - lastBrakeCheckMillis >= brakeCheckInterval) {
    checkBraking();
    lastBrakeCheckMillis = millis();
  }

  // === Throttled LED update ===
  if (millis() - lastLEDUpdateMillis >= LED_UPDATE_INTERVAL) {
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
      leds_const[i].setRGB(CONSTANT_LED_RED, CONSTANT_LED_GREEN, CONSTANT_LED_BLUE);
    }
  } else {
    for (int i = 0; i < NUM_LEDS; i++) {
      if (i % 2 == 0)
        leds_const[i].setRGB(CONSTANT_LED_RED, CONSTANT_LED_GREEN, CONSTANT_LED_BLUE);
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

  // Show startup loading animation if not complete
  if (!startupAnimationComplete) {
    startupAnimation();
    return; // Don't proceed with other startup logic during animation
  }
  
  // Acquire voltage and start timer when voltage becomes available
  if (!voltageAcquired && globalVoltage != 0.0) {
    voltageAcquired = true;
    voltageAcquiredMS = millis();
  }

  // Check if both footpad sensors were triggered
  if (esc.adc1 > esc.footpadThreshold && esc.adc2 > esc.footpadThreshold) {
    lastFootpadTriggerMillis = millis();
    isInitialStartup = false;
  }

  // Clear initial startup flag after duration expires
  if (isInitialStartup && voltageAcquired && (millis() - voltageAcquiredMS > BATTERY_INDICATOR_DURATION)) {
    isInitialStartup = false;  
  }

  // Determine if we should show battery
  // Only show battery if voltage is acquired AND within timer window
  bool showBatteryOnTimer = voltageAcquired && (millis() - voltageAcquiredMS <= BATTERY_INDICATOR_DURATION);
  bool showBatteryOnFootpad = voltageAcquired && (millis() - lastFootpadTriggerMillis <= BATTERY_INDICATOR_DURATION);
  
  // Show battery only if voltage is acquired and within timer
  if (showBatteryOnTimer) {
    batteryPercentStartupLEDs();
  }
  // Else if footpad was triggered recently
  else if (showBatteryOnFootpad) {
    batteryPercentStartupLEDs();
  }
  // Else show static or single footpad indicator
  else {
    // Check if only one footpad is triggered 
    bool onlyOneFootpad = (esc.adc1 > esc.footpadThreshold) != (esc.adc2 > esc.footpadThreshold);
    
    if (onlyOneFootpad) {
      singleFootpadTriggeredStartupLEDs();
    } else {
      staticStartupLEDs();
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
  int numLeds = map(elapsed, 0, STARTUP_ANIMATION_DURATION, 0, NUM_LEDS);
  numLeds = constrain(numLeds, 0, NUM_LEDS);
  
  // Light up forward LEDs progressively
  for (int i = 0; i < NUM_LEDS; i++) {
    if (i < numLeds) {
      forward_leds[i] = CRGB(STARTUP_ANIMATION_LED_RED, STARTUP_ANIMATION_LED_GREEN, STARTUP_ANIMATION_LED_BLUE);
    } else {
      forward_leds[i] = CRGB(0, 0, 0);
    }
  }
  
  // Keep reverse LEDs in default pattern
  for (int i = 0; i < NUM_LEDS; i++) {
    reverse_leds[i] = (i % 2 == 0)
        ? CRGB(CONSTANT_LED_RED, CONSTANT_LED_GREEN, CONSTANT_LED_BLUE)
        : CRGB(0, 0, 0);
  }
}

void staticStartupLEDs() {
     // Static startup LEDs
  for (int i = 0; i < NUM_LEDS; i++) {
    if (direction == FORWARD) {
      forward_leds[i] = CRGB(FLASHING_LED_RED, FLASHING_LED_GREEN, FLASHING_LED_BLUE);
      reverse_leds[i] = (i % 2 == 0)
          ? CRGB(CONSTANT_LED_RED, CONSTANT_LED_GREEN, CONSTANT_LED_BLUE)
          : CRGB(0, 0, 0);
    } else {
      reverse_leds[i] = CRGB(FLASHING_LED_RED, FLASHING_LED_GREEN, FLASHING_LED_BLUE);
      forward_leds[i] = (i % 2 == 0)
          ? CRGB(CONSTANT_LED_RED, CONSTANT_LED_GREEN, CONSTANT_LED_BLUE)
          : CRGB(0, 0, 0);
    }
  }
}

void batteryPercentStartupLEDs() {

    if (!isInitialStartup) {
    // Only check footpads after initial startup (when triggered by footpad press)
    if (esc.adc1 < esc.footpadThreshold || esc.adc2 < esc.footpadThreshold) {
      return;
    }
  }

  // Get battery voltage
  double batteryVoltage = globalVoltage;

  // calculate the battery voltage to be a percentage of the difference between full voltage and low voltage
  double batteryVoltagePercentage = (batteryVoltage - LOW_VOLTAGE) / (FULL_VOLTAGE - LOW_VOLTAGE);

  //light up one led for each 10% of the battery voltage remaining and turn off the rest
  for (int i = 0; i < NUM_LEDS; i++) {
    if (i < batteryVoltagePercentage*NUM_LEDS) {
      forward_leds[i].setRGB(BATTERY_INDICATOR_LED_RED, BATTERY_INDICATOR_LED_GREEN, BATTERY_INDICATOR_LED_BLUE);
    } else {
      forward_leds[i].setRGB(BATTERY_INDICATOR_ALTERNATE_LED_RED, BATTERY_INDICATOR_ALTERNATE_LED_GREEN, BATTERY_INDICATOR_ALTERNATE_LED_BLUE);
    }
  }
}

void singleFootpadTriggeredStartupLEDs() {
  
  if (esc.adc1 > esc.footpadThreshold)
  {
    for (int i = 0; i < NUM_LEDS; i++)
    {
      if (i < NUM_LEDS/2){
        forward_leds[i].setRGB(FOOTPAD_INDICATOR_LED_RED, FOOTPAD_INDICATOR_LED_GREEN, FOOTPAD_INDICATOR_LED_BLUE);
      }
      else {
        forward_leds[i].setRGB(0, 0, 0);
      }
      
    }
  }
  else
  if (esc.adc2 > esc.footpadThreshold)
  {
    for (int i = 0; i < NUM_LEDS; i++)
    {
      if (i > NUM_LEDS/2){
        forward_leds[i].setRGB(FOOTPAD_INDICATOR_LED_RED, FOOTPAD_INDICATOR_LED_GREEN, FOOTPAD_INDICATOR_LED_BLUE);
      }
      else {
        forward_leds[i].setRGB(0, 0, 0);
      }
    }
  }
}