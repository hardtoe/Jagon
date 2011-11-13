#ifndef MOVE_COMMAND_H
#define MOVE_COMMAND_H

#include <WProgram.h>

#include "Vec.h"

class MoveCommand : public Vec<long,4> {
  private:
    unsigned int velocity;
    int acceleration;
    long accelDistance;
    long coastDistance;
    long decelDistance;    
 
  public: 
    void setAccelDistance(long accelDistance) {
      this->accelDistance = accelDistance;  
    }
    
    void setCoastDistance(long coastDistance) {
      this->coastDistance = coastDistance;  
    }
    
    void setDecelDistance(long decelDistance) {
      this->decelDistance = decelDistance;  
    } 
    
    long getAccelDistance() {
      return this->accelDistance;  
    }
    
    long getCoastDistance() {
      return this->coastDistance;  
    }
    
    long getDecelDistance() {
      return this->decelDistance;  
    }
  
    void setSteps(Vec<long,4> steps) {
      for (int i = 0; i < 4; i++) {
        set(i, steps.get(i));
      }  
    }
  
    void setXSteps(long xSteps) {
      set(0, xSteps);
    }
    
    void setYSteps(long ySteps) {
      set(1, ySteps);
    }
    
    void setZSteps(long zSteps) {
      set(2, zSteps);
    }
    
    void setESteps(long eSteps) {
      set(3, eSteps);
    }
    
    void setVelocity(unsigned int velocity) {
      this->velocity = velocity; 
    }
    
    void setAcceleration(int acceleration) {
      this->acceleration = acceleration; 
    }
    
    long getXSteps() {
      return get(0);
    }
    
    long getYSteps() {
      return get(1);
    }
    
    long getZSteps() {
      return get(2);
    }
    
    long getESteps() {
      return get(3);
    }
    
    /**
     * initial velocity in steps/second
     */
    unsigned int getVelocity() {
      return this->velocity; 
    }
    
    /**
     * acceleration in steps/second/second
     */
    unsigned int getAcceleration() {
      return this->acceleration; 
    }
};

#endif // MOVE_COMMAND_H
