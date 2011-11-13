#ifndef HEATER_H
#define HEATER_H

#include <WProgram.h>

template <class TempSensor>
class Heater {
  private:
    struct pt state;
    int heaterPin;
    TempSensor* tempSensor;
    int targetTemp;
    boolean onTarget;  
    int currentTemp;  
    int tempResidence;
    int hysteresisTempRange;
    int minResidenceTime;
  public:
    Heater(
      int heaterPin, 
      TempSensor* tempSensor,
      int hysteresisTempRange,
      int minResidenceTime
    ) {
      this->heaterPin = heaterPin;
      this->tempSensor = tempSensor;
      targetTemp = 0;
      tempResidence = 0;
      onTarget = true;
      
      this->hysteresisTempRange = hysteresisTempRange;
      this->minResidenceTime = minResidenceTime;
      
      pinMode(heaterPin, OUTPUT);
      digitalWrite(heaterPin, LOW);
    }

    int tick() {
      PT_YIELDING();
      PT_BEGIN(&state);

      while(1) {
        currentTemp = 
          tempSensor->getTemp();
        
        if (currentTemp < targetTemp) {
          // temp too low, heat up some more
          digitalWrite(heaterPin, HIGH);
        
        } else if (
          currentTemp > targetTemp ||
          currentTemp > MAXTEMP
        ) {
          // temp too high, let it cool
          digitalWrite(heaterPin, LOW);
        } 
        
        if (
          currentTemp > (targetTemp - hysteresisTempRange) &&
          currentTemp < (targetTemp + hysteresisTempRange)
        ) {
            tempResidence++;
            
            if (tempResidence > minResidenceTime) {
              onTarget = true;
            }
        } else {
           tempResidence = 0; 
           onTarget = false;
        }
        
        
        PT_DELAY(&state, 1000);  
      }
      
      PT_END(&state);
    }
  
    int getTemp() {
      return currentTemp;
    }
    
    void setTarget(int target) {
      // if the new target is outside the hysteresis range...
      if (
        target > targetTemp + hysteresisTempRange ||    
        target < targetTemp - hysteresisTempRange
      ) {
        // ... then reset the target flag
        onTarget = false;
      }
      
      // set the new target
      targetTemp = target; 
    }
    
    inline boolean atTargetTemp() {
      return onTarget;  
    }
};


#endif // HEATER_H
