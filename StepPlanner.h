#ifndef STEP_PLANNER_H
#define STEP_PLANNER_H

#include <WProgram.h>

#include "configuration.h"
#include "CircularBuffer.h"
#include "MoveCommand.h"
#include "StepCommand.h"
#include "Vec.h"

class StepPlanner {
  private:
    struct pt state;
    CircularBuffer<MoveCommand, MOVE_COMMAND_Q_SIZE>* moveCommandBuffer;
    CircularBuffer<StepCommand, STEP_COMMAND_Q_SIZE>* stepCommandBuffer;
      
    MoveCommand* currentCommand;
    StepCommand prototypeStep;
    StepCommand* currentStep;  
    
    Vec<float,4> previousLocation;
    Vec<float,4> currentLocation;
    Vec<float,4> finalLocation;
    unsigned int numSteps;
    Vec<float,4> increment;
    
    byte longAxis;
      
    unsigned int numStepsLeft;  
      
    unsigned int currentStepDelay;
    unsigned int currentVelocity;
    unsigned int minimumStepDelay; 
    unsigned long currentVelocityError;
 
    unsigned int accelerationConstant;   
    byte minStepDelay;  
      
  public:
    StepPlanner(
      CircularBuffer<MoveCommand, MOVE_COMMAND_Q_SIZE>* moveCommandBuffer, 
      CircularBuffer<StepCommand, STEP_COMMAND_Q_SIZE>* stepCommandBuffer
    ) {
      ASSERT(moveCommandBuffer != NULL);
      ASSERT(stepCommandBuffer != NULL);
      
      this->moveCommandBuffer = moveCommandBuffer;
      this->stepCommandBuffer = stepCommandBuffer;
      
      accelerationConstant = 50; // every 50 us, increment velocity
      minStepDelay = 100;
      PT_INIT(&state);
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
        
        // setup our prototype step
        prototypeStep.clear(); 
        
        if (currentCommand->getXSteps() >= 0 ^ INVERT_X_DIR) {
          prototypeStep.setXDir();    
        }
        
        if (currentCommand->getYSteps() >= 0 ^ INVERT_Y_DIR) {
          prototypeStep.setYDir();    
        }
        
        if (currentCommand->getZSteps() >= 0 ^ INVERT_Z_DIR) {
          prototypeStep.setZDir();    
        }
        
        if (currentCommand->getESteps() >= 0 ^ INVERT_E_DIR) {
          prototypeStep.setEDir();    
        }
        
        
        // StepPlanner motions are all relative to the previous move
        currentLocation = (float[]) {0, 0, 0, 0};
        
        // set the final location
        finalLocation.set(0, currentCommand->getXSteps());
        finalLocation.set(1, currentCommand->getYSteps());
        finalLocation.set(2, currentCommand->getZSteps());
        finalLocation.set(3, currentCommand->getESteps());
        
        numSteps = (finalLocation - currentLocation).absValue().maxReduce();
        increment = (finalLocation - currentLocation) / numSteps;

        currentVelocity = currentCommand->getVelocity();
        currentStepDelay = fastDivide(currentCommand->getVelocity());
        prototypeStep.setStepDelay(currentStepDelay);
        prototypeStep.enableAxis(0xF);  
        prototypeStep.setNewEnableDirection(true);
        
        for(
          numStepsLeft = 0;
          numStepsLeft < numSteps;
          numStepsLeft++
        ) {
          // wait until we have space in the step command buffer
          PT_WAIT_UNTIL(&state, 
            stepCommandBuffer->notFull()
          );
          
          previousLocation =
            currentLocation;
          
          currentLocation =
            currentLocation + increment;
          
          currentStep =
            stepCommandBuffer->create();

          *currentStep = prototypeStep;
          
          currentStep->setSteppingAxis(
            previousLocation != currentLocation); 

          stepCommandBuffer->put();

          prototypeStep.setNewEnableDirection(false);
          
          // check if we need to update velocity
          currentVelocityError +=  
            currentStepDelay;
            
          if (currentVelocityError > accelerationConstant) {
            do {
              currentVelocityError -= accelerationConstant;
              
              if (numStepsLeft < (numSteps >> 1)) {
                currentVelocity++;
              } else {
                currentVelocity--;
              }
            } while (currentVelocityError > accelerationConstant);
             
            currentStepDelay = fastDivide(currentVelocity);
          
            if (currentStepDelay >= minStepDelay) {  
              prototypeStep.setStepDelay(currentStepDelay);
            }
          }
        }
         
        moveCommandBuffer->remove();
      }
      
      PT_END(&state);
    }
    

inline unsigned int fastDivide(unsigned int divisor) {
  if (divisor >= 40000) {
    if (divisor >= 50000) {
      return 20;
  
    } else if (divisor >= 47619) {
      return 21;
  
    } else if (divisor >= 45455) {
      return 22;
  
    } else if (divisor >= 43478) {
      return 23;
  
    } else if (divisor >= 41667) {
      return 24;
  
    } else {
      return 25;
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
    return 1000000 / divisor;
  }
}







};


#endif // STEP_PLANNER_H
