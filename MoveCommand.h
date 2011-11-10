#ifndef MOVE_COMMAND_H
#define MOVE_COMMAND_H

#include <WProgram.h>

#include "Vec.h"

class MoveCommand : public Vec<long,4> {
  private:
    unsigned int velocity;
 
  public: 
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
    
    float getVelocity() {
      return this->velocity; 
    }
};

#endif // MOVE_COMMAND_H
