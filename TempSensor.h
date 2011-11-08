#ifndef TEMP_SENSOR_H
#define TEMP_SENSOR_H

#include <WProgram.h>

#include "pins.h"
#include "configuration.h"
#include "thermistortables.h"

#ifdef HEATER_USES_THERMISTOR
    #define HEATERSOURCE 1
#endif
#ifdef HEATER_USES_AD595
    #define HEATERSOURCE 2
#endif
#ifdef HEATER_USES_MAX6675
    #define HEATERSOURCE 3
#endif
#ifdef BED_USES_THERMISTOR
    #define BEDSOURCE 1
#endif
#ifdef BED_USES_AD595
    #define BEDSOURCE 2
#endif
#ifdef BED_USES_MAX6675
    #define BEDSOURCE 3
#endif

#define analog2temp( c ) analog2tempu((c),temptable,NUMTEMPS,HEATERSOURCE)

class TempSensor {
  private:
    byte pin;
  
  public:
    TempSensor(byte tempPin) {
      this->pin = tempPin;
      
      pinMode(pin, INPUT);
    }  
      
    int analog2tempu(int raw,const short table[][2], int numtemps, int source) {
        int celsius = 0;
        byte i;
        
        raw = 1023 - raw;
    
        for (i=1; i<numtemps; i++)
        {
          if (table[i][0] > raw)
          {
            celsius  = table[i-1][1] + 
              (raw - table[i-1][0]) * 
              (table[i][1] - table[i-1][1]) /
              (table[i][0] - table[i-1][0]);
    
            break;
          }
        }
    
        // Overflow: Set to last value in the table
        if (i == numtemps) celsius = table[i-1][1];
    
        return celsius;
    }
  
    int getTemp() {
      return analog2temp(1023 - analogRead(pin));
    }
};


#endif // TEMP_SENSOR_H
