#ifndef HEATER_H
#define HEATER_H

#include <WProgram.h>

#include "TempSensor.h"

class Heater {
  private:
    struct pt state;
    int heaterPin;
    TempSensor* tempSensor;
    int targetTemp;
    boolean onTarget;  
    int currentTemp;  
      
  public:
    Heater(
      int heaterPin, 
      TempSensor* tempSensor
    ) {
      this->heaterPin = heaterPin;
      this->tempSensor = tempSensor;
      targetTemp = 0;
      onTarget = true;
      
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
          digitalWrite(heaterPin, HIGH);
          
        } else {
          digitalWrite(heaterPin, LOW);
          onTarget = true;
        }
        
        PT_DELAY(&state, 1000);  
      }
      
      PT_END(&state);
    }
  
    int getTemp() {
      return currentTemp;
    }
    
    void setTarget(int target) {
      targetTemp = target; 
      onTarget = false;
    }
    
    inline boolean atTargetTemp() {
      return onTarget;  
    }
};


#endif // HEATER_H
