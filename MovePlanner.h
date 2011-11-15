#ifndef MOVE_PLANNER_H
#define MOVE_PLANNER_H

#include <WProgram.h>
#include <math.h>

#include "configuration.h"
#include "CircularBuffer.h"
#include "GCode.h"
#include "MoveCommand.h"
#include "Vec.h"

#include "Assert.h"


class MovePlanner : public GCodeHandler {
  private:
    struct pt state;
    struct pt subState;
    CircularBuffer<MoveCommand, MOVE_COMMAND_Q_SIZE>* moveCommandQueue;
    Vec<long,4> currentLocation;
    Vec<float,4> stepsPerUnit;
    Vec<long,4> maxFeedRate;
    Vec<long,4> maxAcceleration;
    Vec<long,4> maxAccelerationSteps;
    Vec<long,4> maxFeedRateSteps;
    StepperDriver* endstops;
    boolean relativeMode;
    boolean homingRelativeMode;
    
    byte longAxis;
    long longAxisDelta;
    
    float currentFeedrate;
  
  public:
    MovePlanner(
      CircularBuffer<MoveCommand, MOVE_COMMAND_Q_SIZE>* moveCommandQueue,
      StepperDriver* endstops
    ) {
      ASSERT(moveCommandQueue != NULL);
      
      PT_INIT(&state);
      PT_INIT(&subState);
      
      this->moveCommandQueue = moveCommandQueue;
      this->endstops = endstops;
      
      calculateConstants();
      
      currentFeedrate = 20;
    }
    
    void calculateConstants() {
      currentLocation = (long[]) {0, 0, 0, 0};
      stepsPerUnit = axis_steps_per_unit;
      relativeMode = false;
      
      maxFeedRate = max_feedrate;
      maxFeedRateSteps = (maxFeedRate * stepsPerUnit) / 60;
      
      maxAcceleration = max_acceleration_units_per_sq_second;
      maxAccelerationSteps = (F_CPU/16) / (maxAcceleration * stepsPerUnit);
    }
    
    ////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////
    virtual boolean canHandle(GCode* gcode) {
      ASSERT(gcode != NULL);
      
      return 
        (
          gcode->getType() == 'G' &&
          (
            gcode->getOpcode() == 0 ||
            gcode->getOpcode() == 1 ||
            gcode->getOpcode() == 4 ||
            gcode->getOpcode() == 28 ||
            gcode->getOpcode() == 90 ||
            gcode->getOpcode() == 91 ||
            gcode->getOpcode() == 92
          )
        ) ||
        (
          gcode->getType() == 'M' &&
          (
            gcode->getOpcode() == 92 ||
            gcode->getOpcode() == 201
          )
        );
    }
    
    ////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////
    virtual boolean ready(GCode* gcode) {
      ASSERT(gcode != NULL);
      
      return 
        moveCommandQueue->notFull();
    }
    
    ////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////
    virtual int process(GCode* gcode) {
      PT_YIELDING();
      PT_BEGIN(&state);
      
      ASSERT(gcode != NULL);
      ASSERT(canHandle(gcode));
      ASSERT(moveCommandQueue->notFull());
      
      
      DUMP("moveCommandQueue->size(): ", moveCommandQueue->size());
      
      if (gcode->getType() == 'G') {
        if (
          gcode->getOpcode() == 0 ||
          gcode->getOpcode() == 1
        ) {
            coordinatedMove(gcode);                 
  
        } else if (
          gcode->getOpcode() == 4
        ) {
            PT_SPAWN(&state, &subState, dwell(gcode));                       
       
        } else if (
          gcode->getOpcode() == 28
        ) {
            PT_SPAWN(&state, &subState, homeAxis(gcode));                  
       
        } else if (
          gcode->getOpcode() == 90
        ) {
            relativeMode = false;                          
       
        } else if (
          gcode->getOpcode() == 91
        ) {
            relativeMode = true;          
       
        } else if (
          gcode->getOpcode() == 92
        ) {
            setCurrentPosition(gcode);
        }
      } else if (gcode->getType() == 'M') {
        if (
          gcode->getOpcode() == 92
        ) {
            setAxisStepsPerUnit(gcode);
            calculateConstants();
            
        } else if (
          gcode->getOpcode() == 201
        ) {
            setAxisAccel(gcode);
            calculateConstants();
        } 
      }
      
      PT_END(&state); 
    }
    
    ////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////
    inline void coordinatedMove(GCode* gcode) {
      MoveCommand* moveCommand =
        moveCommandQueue->create();
      
      // calculate steps per axis
      for (int axis = 0; axis < 4; axis++) {
        if (isnan(gcode->get(axis))) {
          moveCommand->set(axis, 0);
        } else {
          if (relativeMode || homingRelativeMode) {
            moveCommand->set(axis, (long) (gcode->get(axis) * stepsPerUnit.get(axis)));
            currentLocation.set(axis, (long) (gcode->get(axis) * stepsPerUnit.get(axis)) + currentLocation.get(axis));
          } else {
            moveCommand->set(axis, (long) (gcode->get(axis) * stepsPerUnit.get(axis)) - currentLocation.get(axis));
            currentLocation.set(axis, (long) (gcode->get(axis) * stepsPerUnit.get(axis)));
          }
        }
      }
      
      if (!isnan(gcode->getF())) {
        currentFeedrate = gcode->getF()/60;
      }
      
      longAxis = moveCommand->absValue().maxReduceIndex();
      
      Vec<float,4> moveDelta = (moveCommand->asFloat() / stepsPerUnit.asFloat());
      float moveDistance_mm = moveDelta.magnitude();
      float desiredMoveTime_s = moveDistance_mm / (float) currentFeedrate;
      Vec<long,4> desiredStepsPerSecond = ((*moveCommand) / desiredMoveTime_s).absValue();
      
      moveCommand->setAcceleration(max(1, maxAccelerationSteps.get(longAxis)));  
      
      
      unsigned long numSteps = moveCommand->absValue().maxReduce();  
        
      unsigned long maxStartVelocity =
        max_start_speed_units_per_second[longAxis] * axis_steps_per_unit[longAxis];
        
      unsigned long finalVelocity = 
        min(desiredStepsPerSecond.get(longAxis), maxFeedRateSteps.get(longAxis));
       
      unsigned long initialVelocity;
    
      if (finalVelocity > maxStartVelocity) {
        // we need to ramp
        initialVelocity = maxStartVelocity;
        
        unsigned long numStepsToAccelerate = 
          (((finalVelocity * finalVelocity) - (initialVelocity * initialVelocity)) * 7) / 
          (((unsigned long) 2 * ((unsigned long) maxAcceleration.get(longAxis) * (unsigned long) stepsPerUnit.get(longAxis))) * 5);
                  
        moveCommand->setVelocity(initialVelocity);   
          
        if (numStepsToAccelerate >= (numSteps/2)) {
          // ramp looks like a pyramid, no coasting phase
          moveCommand->setAccelDistance(numSteps/2);
          moveCommand->setCoastDistance(numSteps/2);
          moveCommand->setDecelDistance(numSteps);
          
        } else {
          // ramp looks like a trapezoid    
          moveCommand->setAccelDistance(numStepsToAccelerate);
          moveCommand->setCoastDistance(numSteps - numStepsToAccelerate);
          moveCommand->setDecelDistance(numSteps);
        }  
        
      } else {
        // constant velocity, no acceleration needed
        moveCommand->setVelocity(finalVelocity); 
        moveCommand->setAccelDistance(0);
        moveCommand->setCoastDistance(numSteps);
        moveCommand->setDecelDistance(numSteps);
      }
        
      moveCommandQueue->put();
    }
    
    
    ////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////
    inline int homeAxis(GCode* gcode) {
      PT_YIELDING();
      PT_BEGIN(&subState);    
      
      homingRelativeMode = true;
      
      if (
        (
          isnan(gcode->getX()) && 
          isnan(gcode->getY()) && 
          isnan(gcode->getZ())
        ) ||
        !isnan(gcode->getZ()) 
      ) {
        // home z-axis
        while (!endstops->zAtMin()) {
          PT_WAIT_UNTIL(&subState, moveCommandQueue->isEmpty());    
          coordinatedMove(&(makeGCode(NAN, NAN, -.1, NAN, homing_feedrate[2])));
          setCurrentPosition(&(makeGCode(NAN, NAN, 0, NAN, 0)));
          PT_YIELD(&subState);
        }
        
        // move z up a little bit
        PT_WAIT_UNTIL(&subState, moveCommandQueue->notFull()); 
        coordinatedMove(&(makeGCode(NAN, NAN, 1, NAN, homing_feedrate[2])));
      }
      
      // home x-axis
      if (
        (
          isnan(gcode->getX()) && 
          isnan(gcode->getY()) && 
          isnan(gcode->getZ())
        ) ||
        !isnan(gcode->getX()) 
      ) {
        while (!endstops->xAtMin()) {
          PT_WAIT_UNTIL(&subState, moveCommandQueue->isEmpty());    
          coordinatedMove(&(makeGCode(-1, NAN, NAN, NAN, homing_feedrate[0])));
          setCurrentPosition(&(makeGCode(0, NAN, NAN, NAN, 0)));
          PT_YIELD(&subState);
        }
      }
      
      // home y-axis      
      if (
        (
          isnan(gcode->getX()) && 
          isnan(gcode->getY()) && 
          isnan(gcode->getZ())
        ) ||
        !isnan(gcode->getY()) 
      ) {
        while (!endstops->yAtMin()) {
          PT_WAIT_UNTIL(&subState, moveCommandQueue->isEmpty());    
          coordinatedMove(&(makeGCode(NAN, -1, NAN, NAN, homing_feedrate[1])));
          setCurrentPosition(&(makeGCode(NAN, 0, NAN, NAN, 0)));
          PT_YIELD(&subState);
        }
      }
      
      homingRelativeMode = false;
      
      PT_END(&subState); 
    }
    
    
    ////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////
    
    inline int dwell(GCode* gcode) {
      PT_YIELDING();
      PT_BEGIN(&subState);    
      
      if (!isnan(gcode->getS())) {
        PT_DELAY(&subState, gcode->getS() * 1000);
        
      } else if (!isnan(gcode->getP())) {
        PT_DELAY(&subState, gcode->getP());
      }
      
      PT_END(&subState); 
    }
    
    
    ////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////
    inline void setCurrentPosition(GCode* gcode) {
      for (int axis = 0; axis < 4; axis++) {
        if (!isnan(gcode->get(axis))) {
          currentLocation.set(axis, gcode->get(axis) * stepsPerUnit.get(axis));
        }
      }
    }
    
    
    ////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////
    inline void setAxisStepsPerUnit(GCode* gcode) {
      for (int axis = 0; axis < 4; axis++) {
        if (!isnan(gcode->get(axis))) {
          axis_steps_per_unit[axis] = gcode->get(axis);
        }
      }
    }
    
    
    ////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////
    inline void setAxisAccel(GCode* gcode) {
      for (int axis = 0; axis < 4; axis++) {
        if (!isnan(gcode->get(axis))) {
          max_acceleration_units_per_sq_second[axis] = gcode->get(axis);
        }
      }
    }
    
    ////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////
    GCode makeGCode(
      float x,
      float y,
      float z,
      float e,
      float f
    ) {
      GCode value;

      value.setX(x);
      value.setY(y);
      value.setZ(z);
      value.setE(e);
      value.setF(f);

      return value;      
    }
};


#endif // MOVE_PLANNER_H
