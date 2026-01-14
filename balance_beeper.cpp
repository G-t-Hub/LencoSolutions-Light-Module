#include "beeper.cpp"

#define BEEPER_PIN 4

#define PLAY_STARTUP true
#define DUTY_CYCLE_ALERT 0.75 // 0 to disable
#define LOW_VOLTAGE 58.9 // 0 to disable
#define FULL_VOLTAGE 79.8 // Voltage of battery when fully charged
#define LOW_VOLTAGE_INTERVAL 5 * 1000 // every 30 seconds

class BalanceBeeper {
  private:
    Beeper beeper;

    long lastLowVoltageMillis = 0;
    long lastDutyCycleAlertMillis = 0;
    const long DUTY_CYCLE_ALERT_INTERVAL = 1000; // Alert every 1 second max
    
    // Buzzer timing control to prevent fast loop interference
    long lastBuzzerUpdateMillis = 0;
    const long BUZZER_UPDATE_INTERVAL = 10; // Update buzzer every 10ms instead of every loop
    
    // Alert priority system
    enum AlertPriority {
      PRIORITY_NONE = 0,
      PRIORITY_DUTY_CYCLE = 1,    // Highest priority
      PRIORITY_LOW_VOLTAGE = 2,   // Lower priority
      PRIORITY_STARTUP = 3        // Lowest priority
    };
    
    AlertPriority currentPriority = PRIORITY_NONE;
  public:
    BalanceBeeper() :
      beeper(BEEPER_PIN){
    }
    void setup(){ 
      beeper.setup();
      if(PLAY_STARTUP){
        beeper.queueThreeShort();
        currentPriority = PRIORITY_STARTUP;
      }
    }
    
    // Check if buzzer is currently playing
    bool isPlaying() {
      return beeper.isBeeping;
    }
    
    // Update priority when buzzer finishes
    void updatePriority() {
      if (!isPlaying() && currentPriority != PRIORITY_NONE) {
        currentPriority = PRIORITY_NONE;
      }
    }

    void loop(double dutyCycle, double erpm, double voltage){
      // Only update buzzer at controlled intervals to prevent fast loop interference
      if (millis() - lastBuzzerUpdateMillis >= BUZZER_UPDATE_INTERVAL) {
        beeper.loop();
        lastBuzzerUpdateMillis = millis();
      }
      updatePriority();

      // Duty Cycle Alert - HIGHEST PRIORITY
      if(fabsf(dutyCycle) > DUTY_CYCLE_ALERT && DUTY_CYCLE_ALERT > 0 && 
         lastDutyCycleAlertMillis + DUTY_CYCLE_ALERT_INTERVAL < millis() &&
         (currentPriority == PRIORITY_NONE || currentPriority >= PRIORITY_DUTY_CYCLE)){
        beeper.queueShortSingle();
        lastDutyCycleAlertMillis = millis();
        currentPriority = PRIORITY_DUTY_CYCLE;
      }

      // Low voltage - LOWER PRIORITY (only if no higher priority alert is playing)
      if(voltage < LOW_VOLTAGE && LOW_VOLTAGE > 0 && 
         lastLowVoltageMillis + LOW_VOLTAGE_INTERVAL < millis() &&
         (currentPriority == PRIORITY_NONE || currentPriority >= PRIORITY_LOW_VOLTAGE)){
        beeper.queueSad();
        lastLowVoltageMillis = millis();
        currentPriority = PRIORITY_LOW_VOLTAGE;
      }
    }

};