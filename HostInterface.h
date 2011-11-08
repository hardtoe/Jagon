#ifndef HOST_INTERFACE_H
#define HOST_INTERFACE_H

#include <WProgram.h>

#include "configuration.h"
#include "CircularBuffer.h"
#include "GCode.h"
#include "protothread.h"

#include "Assert.h"

class HostInterface {
  private:
    struct pt state;
    struct pt subState;
    CircularBuffer<GCode, GCODE_Q_SIZE>* gCodeBuffer;
    GCode* gCode;
    char numberConversionBuffer[20];
    byte numberLength;

    int bufferNumber() {
      int serialData;
      
      PT_YIELDING();
      PT_BEGIN(&subState);

      numberLength = 0;

      PT_WAIT_UNTIL(&subState, (serialData = Serial.peek()) > -1);    
      
      while(
        (serialData >= '0' && serialData <= '9') || 
        serialData == '-' ||
        serialData == '.' ||
        serialData == 'E' || 
        serialData == 'e' ||
        serialData == '+' 
      ) {
        Serial.read();
        numberConversionBuffer[numberLength++] = serialData;
        PT_WAIT_UNTIL(&subState, (serialData = Serial.peek()) > -1);  
      }
      
      numberConversionBuffer[numberLength] = 0;
      
      PT_END(&subState); 
    }

    int readInteger(int* value) {
      int threadResult =
        bufferNumber();
      
      if (threadResult == PT_THREAD_EXITED) {
        *value = strtol(numberConversionBuffer, NULL, 10);    
      }
      
      return threadResult;
    }

    int readFloat(float* value) {
      int threadResult =
        bufferNumber();
      
      if (threadResult == PT_THREAD_EXITED) {
        *value = strtod(numberConversionBuffer, NULL);    
      }
      
      return threadResult;
    }
    
    boolean isWhitespace(int character) {
      return 
        character == '\n' || 
        character =='\r' || 
        character == ' ' || 
        character == '\t';
    }
  
  public:
    HostInterface(CircularBuffer<GCode, GCODE_Q_SIZE>* gCodeBuffer) {
      PT_INIT(&state);
      PT_INIT(&subState);

      this->gCodeBuffer = gCodeBuffer;
    }

    inline int tick() {
      int serialData;
      int intValue;
      float floatValue;
      
      PT_YIELDING();
      PT_BEGIN(&state);

      while(1) {
        // wait until there is serial data available and buffer space for the next command
        PT_WAIT_UNTIL(&state, 
          (serialData = Serial.read()) > -1 && 
          gCodeBuffer->notFull()
        );

        if (isWhitespace(serialData)) {
          continue;  
        }
        
        // allocate a gcode from the buffer
        gCode = gCodeBuffer->create();
        gCode->clear();
      
        // process the gcode
        while (
          serialData != '\n' &&
          serialData != '\r'
        ) {
          if (serialData == 'G' || serialData == 'M' || serialData == 'T') {
              gCode->setType(serialData);
              PT_SPAWN(&state, &subState, readInteger(&intValue));
              gCode->setOpcode(intValue);
          
          } else if (serialData == 'X') {
              PT_SPAWN(&state, &subState, readFloat(&floatValue));
              gCode->setX(floatValue);
          
          } else if (serialData == 'Y') {
              PT_SPAWN(&state, &subState, readFloat(&floatValue));
              gCode->setY(floatValue);
          
          } else if (serialData == 'Z') {
              PT_SPAWN(&state, &subState, readFloat(&floatValue));
              gCode->setZ(floatValue);
          
          } else if (serialData == 'E') {
              PT_SPAWN(&state, &subState, readFloat(&floatValue));
              gCode->setE(floatValue);
          
          } else if (serialData == 'F') {
              PT_SPAWN(&state, &subState, readFloat(&floatValue));
              gCode->setF(floatValue);
          
          } else if (serialData == 'S') {
              PT_SPAWN(&state, &subState, readFloat(&floatValue));
              gCode->setS(floatValue);
          
          } else if (serialData == 'P') {
              PT_SPAWN(&state, &subState, readFloat(&floatValue));
              gCode->setP(floatValue);
          }
          
          serialData =
            Serial.read();
        }
   
        // queue up the gcode for the next stage in the pipeline
        gCodeBuffer->put();
        
        // wait until we have buffer space for the next gcode
        PT_WAIT_UNTIL(&state, 
          gCodeBuffer->notFull()
        );
        
        // report back to the host that we are ready for the next command
        Serial.println("ok");
      }
      
      PT_END(&state);
    }
};


#endif // HOST_INTERFACE_H
