#ifndef STEP_PLANNER_H
#define STEP_PLANNER_H

#include <WProgram.h>

#include "configuration.h"
#include "CircularBuffer.h"
#include "MoveCommand.h"
#include "StepCommand.h"
#include "Vec.h"

#define VELOCITY_RESOLUTION 128
#define MIN_STEP_DELAY 40

class StepPlanner {
  private:
    struct pt state;
    struct pt subState;
    CircularBuffer<MoveCommand, MOVE_COMMAND_Q_SIZE>* moveCommandBuffer;
    CircularBuffer<StepCommand, STEP_COMMAND_Q_SIZE>* stepCommandBuffer;
      
    MoveCommand* currentCommand;
    StepCommand prototypeStep;
    StepCommand* currentStep;  
    
    
    long numSteps;
    
    byte longAxis;
      
    long stepNumber;  
      
    unsigned long currentVelocity;
    unsigned long currentVelocityError;
 
    unsigned int  accelerationConstant;   
    
    boolean coasting;
    boolean accelerating;

    long   
      Ad1,  Ad2,  Ad3,  Ad4,
      d1x2, d2x2, d3x2, d4x2,
      err1, err2, err3, err4;  

  public:
    StepPlanner(
      CircularBuffer<MoveCommand, MOVE_COMMAND_Q_SIZE>* moveCommandBuffer, 
      CircularBuffer<StepCommand, STEP_COMMAND_Q_SIZE>* stepCommandBuffer
    ) {
      ASSERT(moveCommandBuffer != NULL);
      ASSERT(stepCommandBuffer != NULL);
      
      this->moveCommandBuffer = moveCommandBuffer;
      this->stepCommandBuffer = stepCommandBuffer;
      
      PT_INIT(&state);
      PT_INIT(&subState);
    }

    int tick() {
      PT_YIELDING();
      PT_BEGIN(&state);

      while(1) {
        // wait until there is a move command available to process
        PT_WAIT_UNTIL(&state, 
          moveCommandBuffer->notEmpty()
        );
        
        // get the next move command from the buffer
        currentCommand =
          moveCommandBuffer->peek();
        
        // setup acceleration
        accelerationConstant = currentCommand->getAcceleration() * VELOCITY_RESOLUTION; 
        
        // setup our prototype step
        prototypeStep.clear(); 
        
        if (currentCommand->getXSteps() >= 0) {
          prototypeStep.setXDir();    
        }
        
        if (currentCommand->getYSteps() >= 0) {
          prototypeStep.setYDir();    
        }
        
        if (currentCommand->getZSteps() >= 0) {
          prototypeStep.setZDir();    
        }
        
        if (currentCommand->getESteps() >= 0) {
          prototypeStep.setEDir();    
        }
        
        prototypeStep.enableAxis(0xF);
      
        prototypeStep.setNewEnableDirection();

        Ad1 = abs(currentCommand->getXSteps());
        Ad2 = abs(currentCommand->getYSteps());
        Ad3 = abs(currentCommand->getZSteps());
        Ad4 = abs(currentCommand->getESteps());
      
        d1x2 = Ad1 * 2;
        d2x2 = Ad2 * 2;
        d3x2 = Ad3 * 2;
        d4x2 = Ad4 * 2;      
        
        if ((Ad1 >= Ad2) && (Ad1 >= Ad3) && (Ad1 >= Ad4)) {
          err1 = d2x2 - Ad1;
          err2 = d3x2 - Ad1;
          err3 = d4x2 - Ad1;
          numSteps = Ad1;
          
          currentVelocity = currentCommand->getVelocity();
          prototypeStep.setStepDelay(fastDivide(currentVelocity));
          
          coasting = false;
          accelerating = true;
          PT_SPAWN(&state, &subState, xPriMove(0, currentCommand->getAccelDistance()));
          
          coasting = true;
          accelerating = false;
          PT_SPAWN(&state, &subState, xPriMove(currentCommand->getAccelDistance(), currentCommand->getCoastDistance()));
          
          coasting = false;
          accelerating = false;
          PT_SPAWN(&state, &subState, xPriMove(currentCommand->getCoastDistance(), numSteps));
          
        } else if ((Ad2 >= Ad3) && (Ad2 >= Ad4)) {
          err1 = d1x2 - Ad2;
          err2 = d3x2 - Ad2;
          err3 = d4x2 - Ad2;
          numSteps = Ad2;
          
          currentVelocity = currentCommand->getVelocity();
          prototypeStep.setStepDelay(fastDivide(currentVelocity));
          
          coasting = false;
          accelerating = true;
          PT_SPAWN(&state, &subState, yPriMove(0, currentCommand->getAccelDistance()));
          
          coasting = true;
          accelerating = false;
          PT_SPAWN(&state, &subState, yPriMove(currentCommand->getAccelDistance(), currentCommand->getCoastDistance()));
          
          coasting = false;
          accelerating = false;
          PT_SPAWN(&state, &subState, yPriMove(currentCommand->getCoastDistance(), numSteps));
          
        } else if(Ad3 >= Ad4) {
          err1 = d2x2 - Ad3;
          err2 = d1x2 - Ad3;
          err3 = d4x2 - Ad3;
          numSteps = Ad3;
          
          currentVelocity = currentCommand->getVelocity();
          prototypeStep.setStepDelay(fastDivide(currentVelocity));
          
          coasting = false;
          accelerating = true;
          PT_SPAWN(&state, &subState, zPriMove(0, currentCommand->getAccelDistance()));
          
          coasting = true;
          accelerating = false;
          PT_SPAWN(&state, &subState, zPriMove(currentCommand->getAccelDistance(), currentCommand->getCoastDistance()));
          
          coasting = false;
          accelerating = false;
          PT_SPAWN(&state, &subState, zPriMove(currentCommand->getCoastDistance(), numSteps));
          
        } else {
          err1 = d1x2 - Ad4;
          err2 = d2x2 - Ad4;
          err3 = d3x2 - Ad4;
          numSteps = Ad4;
          
          currentVelocity = currentCommand->getVelocity();
          prototypeStep.setStepDelay(fastDivide(currentVelocity));
          
          coasting = false;
          accelerating = true;
          PT_SPAWN(&state, &subState, ePriMove(0, currentCommand->getAccelDistance()));
          
          coasting = true;
          accelerating = false;
          PT_SPAWN(&state, &subState, ePriMove(currentCommand->getAccelDistance(), currentCommand->getCoastDistance()));
          
          coasting = false;
          accelerating = false;
          PT_SPAWN(&state, &subState, ePriMove(currentCommand->getCoastDistance(), numSteps));
          
        }
      
        moveCommandBuffer->remove();
      }
      
      PT_END(&state);
    }
    
    int xPriMove(unsigned long startStep, unsigned long stopStep) {
      PT_BEGIN(&subState);
      
      for(
        stepNumber = startStep;
        stepNumber < stopStep;
        stepNumber++
      ) {
        // wait until we have space in the step command buffer
        PT_WAIT_UNTIL(&subState, 
          stepCommandBuffer->notFull()
        );
           
        currentStep =
          stepCommandBuffer->create();

        currentStep->setPrototype(prototypeStep);

        //////////
        // X
        if (err1 > 0) {
          currentStep->setYStep();
          err1 -= d1x2;
        }
    
        if (err2 > 0) {
          currentStep->setZStep();
          err2 -= d1x2;
        }
    
        if (err3 > 0) {
          currentStep->setEStep();
          err3 -= d1x2;
        }
    
        err1 += d2x2;
        err2 += d3x2;
        err3 += d4x2;
    
        currentStep->setXStep();
  
        stepCommandBuffer->put();
        
        // don't need to waste time setting IO registers for DIR or ENABLE for the rest of this move
        prototypeStep.clearNewEnableDirection();
  
        calculateVelocity();
      }      
      
      PT_END(&subState);
    }
    
    int yPriMove(unsigned long startStep, unsigned long stopStep) {
      PT_BEGIN(&subState);
      
      for(
        stepNumber = startStep;
        stepNumber < stopStep;
        stepNumber++
      ) {
        // wait until we have space in the step command buffer
        PT_WAIT_UNTIL(&subState, 
          stepCommandBuffer->notFull()
        );
           
        currentStep =
          stepCommandBuffer->create();

        currentStep->setPrototype(prototypeStep);

        //////////
        // Y
        if (err1 > 0) {
          currentStep->setXStep();
          err1 -= d2x2;
        }
    
        if (err2 > 0) {
          currentStep->setZStep();
          err2 -= d2x2;
        }
    
        if (err3 > 0) {
          currentStep->setEStep();
          err3 -= d2x2;
        }
    
        err1 += d1x2;
        err2 += d3x2;
        err3 += d4x2;
    
        currentStep->setYStep();

        stepCommandBuffer->put();
        
        // don't need to waste time setting IO registers for DIR or ENABLE for the rest of this move
        prototypeStep.clearNewEnableDirection();
  
        calculateVelocity();
      }      
      
      PT_END(&subState);
    }
    
    int zPriMove(unsigned long startStep, unsigned long stopStep) {
      PT_BEGIN(&subState);
      
      for(
        stepNumber = startStep;
        stepNumber < stopStep;
        stepNumber++
      ) {
        // wait until we have space in the step command buffer
        PT_WAIT_UNTIL(&subState, 
          stepCommandBuffer->notFull()
        );
           
        currentStep =
          stepCommandBuffer->create();

        currentStep->setPrototype(prototypeStep);

        //////////
        // Z
        if (err1 > 0) {
          currentStep->setYStep();
          err1 -= d3x2;
        }
    
        if (err2 > 0) {
          currentStep->setXStep();
          err2 -= d3x2;
        }
    
        if (err3 > 0) {
          currentStep->setEStep();
          err3 -= d3x2;
        }
    
        err1 += d2x2;
        err2 += d1x2;
        err3 += d4x2;
    
        currentStep->setZStep();

        stepCommandBuffer->put();
        
        // don't need to waste time setting IO registers for DIR or ENABLE for the rest of this move
        prototypeStep.clearNewEnableDirection();
  
        calculateVelocity();
      }      
      
      PT_END(&subState);
    }
    
    int ePriMove(unsigned long startStep, unsigned long stopStep) {
      PT_BEGIN(&subState);
      
      for(
        stepNumber = startStep;
        stepNumber < stopStep;
        stepNumber++
      ) {
        // wait until we have space in the step command buffer
         PT_WAIT_UNTIL(&subState, 
          stepCommandBuffer->notFull()
        );
        
        currentStep =
          stepCommandBuffer->create();
  
        currentStep->setPrototype(prototypeStep);
  
        //////////
        // E
        if (err1 > 0) {
          currentStep->setXStep();
          err1 -= d4x2;
        }
    
        if (err2 > 0) {
          currentStep->setYStep();
          err2 -= d4x2;
        }
    
        if (err3 > 0) {
          currentStep->setZStep();
          err3 -= d4x2;
        }
    
        err1 += d1x2;
        err2 += d2x2;
        err3 += d3x2;
    
        currentStep->setEStep();
  
        stepCommandBuffer->put();
        
        // don't need to waste time setting IO registers for DIR or ENABLE for the rest of this move
        prototypeStep.clearNewEnableDirection();
        
        calculateVelocity();
      }      
      
      PT_END(&subState);
    }
    
    /**
     * calculate velocity for next step.  this can be thought of as a 2D Bresenham 
     * algorithm where one dimension is the velocity in steps per second, and the
     * other dimension is time in microseconds.
     *
     * runtime is slower for long periods due to looping through the velocity
     * calculation multiple times.
     *
     * runtime for short periods is very fast *per period* because we may only 
     * loop once or even not at all.
     *
     * runtime for constant velocity moves should be very good as the entire
     * algorithm will be skipped. 
     */
    inline void calculateVelocity() {
      if (!coasting) {      
        // accumulate how long its been since the last velocity increase
        currentVelocityError +=  
          prototypeStep.getStepDelay();
          
        // if it's been longer than the period we need to increase velocity linearly...
        if (currentVelocityError > accelerationConstant) {
          do {
            // ...then remove the length of the period to increase velocity...
            currentVelocityError -= accelerationConstant;
            
            // ...figure out whether we should increase or decrease our velocity
            if (accelerating) {
              currentVelocity += VELOCITY_RESOLUTION;
            } else {
              currentVelocity -= VELOCITY_RESOLUTION;
            }
            
            // ...and keep looping until we have accumulated the correct velocity
          } while (currentVelocityError > accelerationConstant);
          
          // convert the calculated velocity into a step delay using a fast division algorithm
          prototypeStep.setStepDelay(fastDivide(currentVelocity));
        }
      }  
    }

#define DIVIDEND (F_CPU/16)

  inline unsigned int fastDivide(unsigned int divisor) {
    if (divisor >= (DIVIDEND/(MIN_STEP_DELAY+5))) {
      if (divisor >= DIVIDEND/MIN_STEP_DELAY) {
        return MIN_STEP_DELAY; 
        
      } else if (divisor >= DIVIDEND/(MIN_STEP_DELAY+1)) {
        return (MIN_STEP_DELAY+1); 
        
      } else if (divisor >= DIVIDEND/(MIN_STEP_DELAY+2)) {
        return (MIN_STEP_DELAY+2); 
        
      } else if (divisor >= DIVIDEND/(MIN_STEP_DELAY+3)) {
        return (MIN_STEP_DELAY+3); 
        
      } else if (divisor >= DIVIDEND/(MIN_STEP_DELAY+4)) {
        return (MIN_STEP_DELAY+4); 
        
      } else {
        return (MIN_STEP_DELAY+5);
      }
    } else if (divisor > 11236) { 
      if (divisor > 17544) { 
        if (divisor > 24390) {          
          if (divisor > 30303) {
            if (divisor > 34483) {
              if (divisor > 37037) {          
                if (divisor > 38462) {
                  return 26;
                } else {
                  return 27;
                }
              } else {      
                if (divisor > 35714) {
                  return 28;
                } else {
                  return 29;
                }
              }
            } else {
              if (divisor > 32258) {          
                if (divisor > 33333) {
                  return 30;
                } else {
                  return 31;
                }
              } else {      
                if (divisor > 31250) {
                  return 32;
                } else {
                  return 33;
                }
              }
            }            
          } else {
            if (divisor > 27027) {
              if (divisor > 28571) {          
                if (divisor > 29412) {
                  return 34;
                } else {
                  return 35;
                }
              } else {      
                if (divisor > 27778) {
                  return 36;
                } else {
                  return 37;
                }
              }
            } else {
              if (divisor > 25641) {          
                if (divisor >26316 ) {
                  return 38;
                } else {
                  return 39;
                }
              } else {      
                if (divisor > 25000) {
                  return 40;
                } else {
                  return 41;
                }
              }
            }
          }
        } else {      
          if (divisor > 20408) {
            if (divisor > 22222) {
              if (divisor > 23256) {          
                if (divisor > 23810) {
                  return 42;
                } else {
                  return 43;
                }
              } else {      
                if (divisor > 22727) {
                  return 44;
                } else {
                  return 45;
                }
              }
            } else {
              if (divisor > 21277) {          
                if (divisor >21739 ) {
                  return 46;
                } else {
                  return 47;
                }
              } else {      
                if (divisor > 20833) {
                  return 48;
                } else {
                  return 49;
                }
              }
            }  
          } else {
            if (divisor > 18868) {
              if (divisor > 19608) {          
                if (divisor > 20000) {
                  return 50;
                } else {
                  return 51;
                }
              } else {      
                if (divisor > 19231) {
                  return 52;
                } else {
                  return 53;
                }
              }
            } else {
              if (divisor > 18182) {          
                if (divisor >18519 ) {
                  return 54;
                } else {
                  return 55;
                }
              } else {      
                if (divisor > 17857) {
                  return 56;
                } else {
                  return 57;
                }
              }
            }  
          }
        }
      } else {
        if (divisor > 13699) {          
          if (divisor > 15385) {
            if (divisor > 16393) {
              if (divisor > 16949) {          
                if (divisor > 17241) {
                  return 58;
                } else {
                  return 59;
                }
              } else {      
                if (divisor > 16667) {
                  return 60;
                } else {
                  return 61;
                }
              }
            } else {
              if (divisor > 15873) {          
                if (divisor >16129 ) {
                  return 62;
                } else {
                  return 63;
                }
              } else {      
                if (divisor > 15625) {
                  return 64;
                } else {
                  return 65;
                }
              }
            } 
          } else {
            if (divisor > 14493) {
              if (divisor > 14925) {          
                if (divisor > 15152) {
                  return 66;
                } else {
                  return 67;
                }
              } else {      
                if (divisor > 14706) {
                  return 68;
                } else {
                  return 69;
                }
              }
            } else {
              if (divisor > 14085) {          
                if (divisor > 14286) {
                  return 70;
                } else {
                  return 71;
                }
              } else {      
                if (divisor >13889 ) {
                  return 72;
                } else {
                  return 73;
                }
              }
            } 
          }
        } else {      
          if (divisor > 12346) {
            if (divisor > 12987) {
              if (divisor > 13333) {          
                if (divisor > 13514) {
                  return 74;
                } else {
                  return 75;
                }
              } else {      
                if (divisor > 13158) {
                  return 76;
                } else {
                  return 77;
                }
              }
            } else {
              if (divisor > 12658) {          
                if (divisor > 12821) {
                  return 78;
                } else {
                  return 79;
                }
              } else {      
                if (divisor >12500 ) {
                  return 80;
                } else {
                  return 81;
                }
              }
            }
          } else {
            if (divisor > 11765) {
              if (divisor > 12048) {          
                if (divisor > 12195) {
                  return 82;
                } else {
                  return 83;
                }
              } else {      
                if (divisor > 11905) {
                  return 84;
                } else {
                  return 85;
                }
              }
            } else {
              if (divisor > 11494) {          
                if (divisor > 11628) {
                  return 86;
                } else {
                  return 87;
                }
              } else {      
                if (divisor >11364 ) {
                  return 88;
                } else {
                  return 89;
                }
              }
            } 
          }
        }
      } 
    } else {
      return DIVIDEND / divisor;
    }
  }

  // 32 x 16 bit division algorithm from repetier
  long Div4U2U(unsigned long a,unsigned int b) {
    // r14/r15 remainder
    // r16 counter
    __asm__ __volatile__ (
    "clr r14 \n\t"
    "sub r15,r15 \n\t"
    "tst %D0 \n\t"
    "brne do32%= \n\t"
    "tst %C0 \n\t"
    "breq donot24%= \n\t"
    "rjmp do24%= \n\t"
    "donot24%=:" "ldi r16,17 \n\t" // 16 Bit divide
    "d16u_1%=:" "rol %A0 \n\t"
    "rol %B0 \n\t"
    "dec r16 \n\t"
    "brne	d16u_2%= \n\t"
    "rjmp end%= \n\t"
    "d16u_2%=:" "rol r14 \n\t"
    "rol r15 \n\t"
    "sub r14,%A2 \n\t"
    "sbc r15,%B2 \n\t"
    "brcc	d16u_3%= \n\t"
    "add r14,%A2 \n\t"
    "adc r15,%B2 \n\t"
    "clc \n\t"
    "rjmp d16u_1%= \n\t"
    "d16u_3%=:" "sec \n\t"
    "rjmp d16u_1%= \n\t"
    "do32%=:" // divide full 32 bit
    "rjmp do32B%= \n\t"
    "do24%=:" // divide 24 bit
  
    "ldi r16,25 \n\t" // 24 Bit divide
    "d24u_1%=:" "rol %A0 \n\t"
    "rol %B0 \n\t"
    "rol %C0 \n\t"
    "dec r16 \n\t"
    "brne	d24u_2%= \n\t"
    "rjmp end%= \n\t"
    "d24u_2%=:" "rol r14 \n\t"
    "rol r15 \n\t"
    "sub r14,%A2 \n\t"
    "sbc r15,%B2 \n\t"
    "brcc	d24u_3%= \n\t"
    "add r14,%A2 \n\t"
    "adc r15,%B2 \n\t"
    "clc \n\t"
    "rjmp d24u_1%= \n\t"
    "d24u_3%=:" "sec \n\t"
    "rjmp d24u_1%= \n\t"
  
    "do32B%=:" // divide full 32 bit
  
    "ldi r16,33 \n\t" // 32 Bit divide
    "d32u_1%=:" "rol %A0 \n\t"
    "rol %B0 \n\t"
    "rol %C0 \n\t"
    "rol %D0 \n\t"
    "dec r16 \n\t"
    "brne	d32u_2%= \n\t"
    "rjmp end%= \n\t"
    "d32u_2%=:" "rol r14 \n\t"
    "rol r15 \n\t"
    "sub r14,%A2 \n\t"
    "sbc r15,%B2 \n\t"
    "brcc	d32u_3%= \n\t"
    "add r14,%A2 \n\t"
    "adc r15,%B2 \n\t"
    "clc \n\t"
    "rjmp d32u_1%= \n\t"
    "d32u_3%=:" "sec \n\t"
    "rjmp d32u_1%= \n\t"
  
    "end%=:" // end
    :"=&r"(a)
    :"0"(a),"r"(b)
    :"r14","r15","r16"
    );
   return a; 
  }

};


#endif // STEP_PLANNER_H
