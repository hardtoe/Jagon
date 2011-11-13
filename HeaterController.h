#ifndef HEATER_CONTROLLER_H
#define HEATER_CONTROLLER_H

#include <WProgram.h>
#include <math.h>
#include "Heater.h"
#include "protothread.h"

class HeaterController : public GCodeHandler{
  private:
      struct pt state;
      struct pt subState;
      Heater* heaterZero; 
      Heater* heaterOne;
      
  public:
    HeaterController(
      Heater* heaterZero, 
      Heater* heaterOne
    ) {
      this->heaterZero = heaterZero;
      this->heaterOne = heaterOne;
      
      PT_INIT(&state);
      PT_INIT(&subState);
    }
    
    virtual boolean canHandle(GCode* gcode) {
      return 
        gcode->getType() == 'M' &&
        (
          gcode->getOpcode() == 104 ||
          gcode->getOpcode() == 105 ||
          gcode->getOpcode() == 109 ||
          gcode->getOpcode() == 190 ||
          gcode->getOpcode() == 140
        );
    }
    
    virtual boolean ready(GCode* gcode) {
      return true;
    }
    
    virtual int process(GCode* gcode) {  
      PT_YIELDING();
      PT_BEGIN(&state);
      
      if (
        gcode->getOpcode() == 105
      ) {
        readCurrentTemp();
        
      } else if (
        gcode->getOpcode() == 104
      ) {
        setExtruderTemp(gcode->getS());
        
      } else if (
        gcode->getOpcode() == 109
      ) {
        PT_SPAWN(&state, &subState, waitExtruderTemp(gcode->getS()));     
        
      } else if (
        gcode->getOpcode() == 190
      ) {
        PT_SPAWN(&state, &subState, waitBedTemp(gcode->getS()));     
        
      } else if (
        gcode->getOpcode() == 140
      ) {
        setBedTemp(gcode->getS());
      }
      
      PT_END(&state);
    }
    
    inline void readCurrentTemp() {
        Serial.print("T:");
        Serial.print(heaterZero->getTemp());
        Serial.print(" B:");
        Serial.println(heaterOne->getTemp());      
    }
    
    inline void setExtruderTemp(int targetTemp) {
      heaterZero->setTarget(targetTemp);
    }
    
    inline void setBedTemp(int targetTemp) {
      heaterOne->setTarget(targetTemp);
    }
    
    inline int waitExtruderTemp(float targetTemp) {
      PT_YIELDING();
      PT_BEGIN(&subState);

      // if targetTemp is a real number...
      if (!isnan(targetTemp)) {
        // ...tell heaterZero (extruder) to warm up to targetTemp
        heaterZero->setTarget(targetTemp);
      }

      // while heaterZero (extruder) has not yet reached the target temp...
      while (!heaterZero->atTargetTemp()) {
         // ...delay for 1 second (allowing other threads to execute in the meantime)...
         PT_DELAY(&subState, 1000);
         
         // ...and report the current temperatures back to the host
         readCurrentTemp(); 
      }

      PT_END(&subState);
    }
    
    inline int waitBedTemp(float targetTemp) {
      PT_YIELDING();
      PT_BEGIN(&subState);

      if (!isnan(targetTemp)) {
        heaterOne->setTarget(targetTemp);
      }

      while (!heaterOne->atTargetTemp()) {
         PT_DELAY(&subState, 1000);
         readCurrentTemp(); 
      }

      PT_END(&subState);
    }
};


#endif // HEATER_CONTROLLER_H
