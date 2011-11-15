#ifndef STEPPER_DRIVER_H
#define STEPPER_DRIVER_H

#include <WProgram.h>
#include "configuration.h"
#include "pins.h"
#include "PololuStepper.h"
#include "StepCommand.h"
#include "CircularBuffer.h"
#include "TimerOne.h"

#include "Assert.h"

// stepper motors

class StepperDriver {
  private:
    PololuStepper(X_STEP_PIN, X_DIR_PIN, X_ENABLE_PIN, INVERT_X_DIR) xAxis;
    PololuStepper(Y_STEP_PIN, Y_DIR_PIN, Y_ENABLE_PIN, INVERT_Y_DIR) yAxis;
    PololuStepper(Z_STEP_PIN, Z_DIR_PIN, Z_ENABLE_PIN, INVERT_Z_DIR) zAxis;
    PololuStepper(E_STEP_PIN, E_DIR_PIN, E_ENABLE_PIN, INVERT_E_DIR) eAxis;

    CircularBuffer<StepCommand, STEP_COMMAND_Q_SIZE>* stepCommandBuffer;

    boolean skippedLast;
    byte skips;
    
  public:
    StepperDriver(
      CircularBuffer<StepCommand, STEP_COMMAND_Q_SIZE>* stepCommandBuffer
    ) {
      ASSERT(stepCommandBuffer != NULL);
      
      this->stepCommandBuffer = stepCommandBuffer;
      
      // steppers
      xAxis.init();
      yAxis.init();
      zAxis.init();
      eAxis.init();
      
      // endstops and pullups (copied directly from Sprinter)
      #ifdef ENDSTOPPULLUPS
        #if X_MIN_PIN > -1
          SET_INPUT(X_MIN_PIN); 
          WRITE(X_MIN_PIN,HIGH);
        #endif
        #if X_MAX_PIN > -1
          SET_INPUT(X_MAX_PIN); 
          WRITE(X_MAX_PIN,HIGH);
        #endif
        #if Y_MIN_PIN > -1
          SET_INPUT(Y_MIN_PIN); 
          WRITE(Y_MIN_PIN,HIGH);
        #endif
        #if Y_MAX_PIN > -1
          SET_INPUT(Y_MAX_PIN); 
          WRITE(Y_MAX_PIN,HIGH);
        #endif
        #if Z_MIN_PIN > -1
          SET_INPUT(Z_MIN_PIN); 
          WRITE(Z_MIN_PIN,HIGH);
        #endif
        #if Z_MAX_PIN > -1
          SET_INPUT(Z_MAX_PIN); 
          WRITE(Z_MAX_PIN,HIGH);
        #endif
      #else
        #if X_MIN_PIN > -1
          SET_INPUT(X_MIN_PIN); 
        #endif
        #if X_MAX_PIN > -1
          SET_INPUT(X_MAX_PIN); 
        #endif
        #if Y_MIN_PIN > -1
          SET_INPUT(Y_MIN_PIN); 
        #endif
        #if Y_MAX_PIN > -1
          SET_INPUT(Y_MAX_PIN); 
        #endif
        #if Z_MIN_PIN > -1
          SET_INPUT(Z_MIN_PIN); 
        #endif
        #if Z_MAX_PIN > -1
          SET_INPUT(Z_MAX_PIN); 
        #endif
      #endif
    }
    
    inline void setNextStepDelay(int nextStepDelay) {
      ASSERT(nextStepDelay > 0);
      
      Timer1.setPeriod(nextStepDelay);
    }
    
    inline boolean xAtMin() {
      return READ(X_MIN_PIN)^ENDSTOPS_INVERTING;
    }
    
    inline boolean yAtMin() {
      return READ(Y_MIN_PIN)^ENDSTOPS_INVERTING;
    }
    
    inline boolean zAtMin() {
      return READ(Z_MIN_PIN)^ENDSTOPS_INVERTING;
    }
    
    inline boolean axisAtMin(byte axis) {
      switch(axis) {
        case 0: return xAtMin();
        case 1: return yAtMin();
        case 2: return zAtMin();
        default: return false;
      } 
    }
      
    inline void interrupt() {
      StepCommand* currentCommand;
      
      if (stepCommandBuffer->notEmpty()) {     
        currentCommand = stepCommandBuffer->peek(); 

        if (currentCommand->hasNewEnableDirection()) {            
          xAxis.enable(!currentCommand->xEnabled());
          yAxis.enable(!currentCommand->yEnabled());
          zAxis.enable(!currentCommand->zEnabled());
          eAxis.enable(!currentCommand->eEnabled());       
        }   
             
        xAxis.setDirection(currentCommand->xDir());
        yAxis.setDirection(currentCommand->yDir());
        zAxis.setDirection(currentCommand->zDir());
        eAxis.setDirection(currentCommand->eDir());   

        if (currentCommand->xStep() && (!xAtMin() || xAxis.getDirection())) {
          xAxis.step(true);
        }
        
        if (currentCommand->yStep() && (!yAtMin() || yAxis.getDirection())) {
          yAxis.step(true);
        }
        
        if (currentCommand->zStep() && (!zAtMin() || zAxis.getDirection())) {
          zAxis.step(true);
        }
        
        if (currentCommand->eStep()) {
          eAxis.step(true);
        }

        setNextStepDelay(currentCommand->getStepDelay());
        
        stepCommandBuffer->remove(); 

        xAxis.step(false);
        yAxis.step(false);
        zAxis.step(false);
        eAxis.step(false);
        
        //skippedLast = false;
        
      } /*else {
        if (!skippedLast) {
          Serial.println((unsigned int) skips++); 
        }
        
        skippedLast = true;
      }*/
    }
};


#endif // STEPPER_DRIVER_H
