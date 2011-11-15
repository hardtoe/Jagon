#ifndef GCODE_DECODER_H
#define GCODE_DECODER_H

#include <WProgram.h>

#include "GCode.h"

#define MAX_HANDLERS 10

class GCodeHandler {
  public:
    virtual boolean canHandle(GCode* gcode) = 0;
    virtual boolean ready(GCode* gcode) = 0;
    virtual int process(GCode* gcode) = 0;
};

class GCodeDecoder {
  private:
    struct pt state;
    struct pt subState;
    GCode* gCode;  
    GCodeHandler* gCodeHandlers[MAX_HANDLERS];
    byte numHandlers;
    byte currentHandler;
    CircularBuffer<GCode, GCODE_Q_SIZE>* gCodeBuffer;
  
  public:
    GCodeDecoder(CircularBuffer<GCode, GCODE_Q_SIZE>* gCodeBuffer) {
      ASSERT(gCodeBuffer != NULL);
      
      PT_INIT(&state);
      PT_INIT(&subState);
      
      this->gCodeBuffer = gCodeBuffer;
      numHandlers = 0;
    }

    void addHandler(GCodeHandler* handler) {
      ASSERT(handler != NULL);
      ASSERT(numHandlers < MAX_HANDLERS);
      
      gCodeHandlers[numHandlers] = handler;
      numHandlers++;  
    }

    int tick() {
      boolean hasOwner;
        
      PT_YIELDING();
      PT_BEGIN(&state);

      while(1) {
        // wait until there is a gcode available to process
        PT_WAIT_UNTIL(&state, 
          gCodeBuffer->notEmpty()
        );
        
        gCode = gCodeBuffer->peek();
        hasOwner = false;
        
        // query all the handlers to see who can process this gcode
        for (currentHandler = 0; currentHandler < numHandlers; currentHandler++) {
            if (gCodeHandlers[currentHandler]->canHandle(gCode)) {             
               hasOwner = true;
               
               if (gCodeHandlers[currentHandler]->ready(gCode)) {
                 PT_WAIT_THREAD(&state, gCodeHandlers[currentHandler]->process(gCode));                      
                 gCodeBuffer->remove();
                 break;
               }
            }
        }
        
        // if no one claimed this command, get rid of it
        if (!hasOwner) {
          Serial.print("// Unsupported GCODE: ");
          Serial.print(gCode->getType());
          Serial.println(gCode->getOpcode());
          gCodeBuffer->remove();
        }
       
       PT_YIELD(&state); 
      }
      
      PT_END(&state);
    }
};


#endif // GCODE_DECODER_H
