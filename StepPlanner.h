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
    float numSteps;
    Vec<float,4> increment;
    
    byte longAxis;
      
    int numStepsLeft;  
      
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
        } else {
          prototypeStep.clearXDir();    
        }
        
        if (currentCommand->getYSteps() >= 0 ^ INVERT_Y_DIR) {
          prototypeStep.setYDir();    
        } else {
          prototypeStep.clearYDir();    
        }
        
        if (currentCommand->getZSteps() >= 0 ^ INVERT_Z_DIR) {
          prototypeStep.setZDir();    
        } else {
          prototypeStep.clearZDir();    
        }
        
        if (currentCommand->getESteps() >= 0 ^ INVERT_E_DIR) {
          prototypeStep.setEDir();    
        } else {
          prototypeStep.clearEDir();    
        }
        
        prototypeStep.setStepDelay(500);
        prototypeStep.enableAxis(0xF);  
        
        // StepPlanner motions are all relative to the previous move
        currentLocation = (float[]) {0, 0, 0, 0};
        
        // set the final location
        finalLocation.set(0, currentCommand->getXSteps());
        finalLocation.set(1, currentCommand->getYSteps());
        finalLocation.set(2, currentCommand->getZSteps());
        finalLocation.set(3, currentCommand->getESteps());
        
        numSteps = (finalLocation - currentLocation).absValue().maxReduce();
        increment = (finalLocation - currentLocation) / numSteps;

        prototypeStep.setStepDelay(1000000 / currentCommand->getVelocity());
        prototypeStep.enableAxis(0xF);  
        
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
        }
         
        moveCommandBuffer->remove();
      }
      
      PT_END(&state);
    }
};


#endif // STEP_PLANNER_H
