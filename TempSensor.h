#ifndef TEMP_SENSOR_H
#define TEMP_SENSOR_H

#include <WProgram.h>

#include "pins.h"
#include "configuration.h"
#include "thermistortables.h"

#ifdef HEATER_USES_THERMISTOR
    #define HEATERSOURCE 1
    #define ExtruderTempSensor ThermistorTempSensor
#endif
#ifdef HEATER_USES_AD595
    #define HEATERSOURCE 2
    #define ExtruderTempSensor AD595TempSensor
#endif
#ifdef HEATER_USES_MAX6675
    #define HEATERSOURCE 3
    #define ExtruderTempSensor Max6675TempSensor
#endif
#ifdef BED_USES_THERMISTOR
    #define BEDSOURCE 1
    #define BedTempSensor ThermistorTempSensor
#endif
#ifdef BED_USES_AD595
    #define BEDSOURCE 2
    #define BedTempSensor AD595TempSensor
#endif
#ifdef BED_USES_MAX6675
    #define BEDSOURCE 3
    #define BedTempSensor Max6675TempSensor
#endif



#define analog2temp( c ) analog2tempu((c),temptable,NUMTEMPS,HEATERSOURCE)

class ThermistorTempSensor {
  private:
    byte pin;
  
  public:
    ThermistorTempSensor(byte tempPin) {
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

class AD595TempSensor {
  private:
    byte pin;
  
  public:
    AD595TempSensor(byte tempPin) {
      this->pin = tempPin;
      
      pinMode(pin, INPUT);
    }  
      
    int analog2tempu(int raw) {
        return raw * 500 / 1024;
    }
  
    int getTemp() {
      return analog2tempu(analogRead(pin));
    }
};

class Max6675TempSensor {
  private:
  
  public:
    int read_max6675() {
      int max6675_temp = 0;
        
      #ifdef	PRR
        PRR &= ~(1<<PRSPI);
      #elif defined PRR0
        PRR0 &= ~(1<<PRSPI);
      #endif
      
      SPCR = (1<<MSTR) | (1<<SPE) | (1<<SPR0);
      
      // enable TT_MAX6675
      WRITE(MAX6675_SS, 0);
      
      // ensure 100ns delay - a bit extra is fine
      delay(1);
      
      // read MSB
      SPDR = 0;
      for (;(SPSR & (1<<SPIF)) == 0;);
      max6675_temp = SPDR;
      max6675_temp <<= 8;
      
      // read LSB
      SPDR = 0;
      for (;(SPSR & (1<<SPIF)) == 0;);
      max6675_temp |= SPDR;
      
      // disable TT_MAX6675
      WRITE(MAX6675_SS, 1);
    
      if (max6675_temp & 4) 
      {
        // thermocouple open
        max6675_temp = 2000;
      }
      else 
      {
        max6675_temp = max6675_temp >> 3;
      }
    
      return max6675_temp;
    }
  
    int getTemp() {
      return read_max6675() / 4;
    }
};


#endif // TEMP_SENSOR_H
