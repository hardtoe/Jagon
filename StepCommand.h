#ifndef STEP_COMMAND_H
#define STEP_COMMAND_H

#include <WProgram.h>

#include "Vec.h"

class StepCommand : public Vec<boolean,4> {
  private:
    int stepCommand;
    int stepDelay;
 
  public: 
    inline void clear() {
       stepCommand = 0; 
       set(0, false);
       set(1, false);
       set(2, false);
       set(3, false);
    }

    inline void setSteppingAxis(Vec<boolean,4> rhs) {
      set(0, rhs.get(0));  
      set(1, rhs.get(1));  
      set(2, rhs.get(2));  
      set(3, rhs.get(3));  
    }

    inline void setStepDelay(int stepDelay) {
      this->stepDelay = stepDelay;
    }
    
    inline int getStepDelay() {
      return this->stepDelay;
    }
  
    inline void enableAxis(int axis) {
      stepCommand |= (axis & 0xf);
    }
    
    inline void setXStep() {
      set(0, true);
    }
    
    inline void setYStep() {
      set(1, true);
    }
    
    inline void setZStep() {
      set(2, true);
    }
    
    inline void setEStep() {
      set(3, true);
    }

    inline void setXDir() {
      stepCommand |= (1 << 8);
    }
    
    inline void setYDir() {
      stepCommand |= (1 << 9);
    }
    
    inline void setZDir() {
      stepCommand |= (1 << 10);
    }
    
    inline void setEDir() {
      stepCommand |= (1 << 11);
    }  

    inline void clearXDir() {
      stepCommand &= ~(1 << 8);
    }
    
    inline void clearYDir() {
      stepCommand &= ~(1 << 9);
    }
    
    inline void clearZDir() {
      stepCommand &= ~(1 << 10);
    }
    
    inline void clearEDir() {
      stepCommand &= ~(1 << 11);
    }  

    inline boolean xEnabled() {
      return (stepCommand >> 0) & 1;
    }
    
    inline boolean yEnabled() {
      return (stepCommand >> 1) & 1;
    }
    
    inline boolean zEnabled() {
      return (stepCommand >> 2) & 1;
    }
    
    inline boolean eEnabled() {
      return (stepCommand >> 3) & 1;
    }
    
    inline boolean xStep() {
      return get(0);
    }
    
    inline boolean yStep() {
      return get(1);
    }
    
    inline boolean zStep() {
      return get(2);
    }
    
    inline boolean eStep() {
      return get(3);
    }

    inline boolean xDir() {
      return (stepCommand >> 8) & 1;
    }
    
    inline boolean yDir() {
      return (stepCommand >> 9) & 1;
    }
    
    inline boolean zDir() {
      return (stepCommand >> 10) & 1;
    }
    
    inline boolean eDir() {
      return (stepCommand >> 11) & 1;
    }
};

#endif // STEP_COMMAND_H
