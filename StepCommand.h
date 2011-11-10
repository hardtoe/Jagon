#ifndef STEP_COMMAND_H
#define STEP_COMMAND_H

#include <WProgram.h>

#include "Vec.h"

// stepCommand = {step[3:0],direction[3:0],enable[3:0],}
class StepCommand {
  private:
    unsigned int stepCommand;
    unsigned int stepDelay;
    boolean newEnableDirection;
 
  public: 
    inline void clear() {
       stepCommand = 0; 
       newEnableDirection = true;
    }

    inline void setSteppingAxis(Vec<boolean,4> rhs) {
      if (rhs.get(0)) setXStep();
      if (rhs.get(1)) setYStep();
      if (rhs.get(2)) setZStep();
      if (rhs.get(3)) setEStep();
    }

    inline boolean hasNewEnableDirection() {
        return newEnableDirection;
    }

    inline void setNewEnableDirection(boolean value) {
       newEnableDirection = value; 
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
      stepCommand |= 0x100;
    }
    
    inline void setYStep() {
      stepCommand |= 0x200;
    }
    
    inline void setZStep() {
      stepCommand |= 0x400;
    }
    
    inline void setEStep() {
      stepCommand |= 0x800;
    }

    inline void setXDir() {
      stepCommand |= 0x10;
    }
    
    inline void setYDir() {
      stepCommand |= 0x20;
    }
    
    inline void setZDir() {
      stepCommand |= 0x40;
    }
    
    inline void setEDir() {
      stepCommand |= 0x80;
    }  

    inline void clearXDir() {
      stepCommand &= ~0x10;
    }
    
    inline void clearYDir() {
      stepCommand &= ~0x20;
    }
    
    inline void clearZDir() {
      stepCommand &= ~0x40;
    }
    
    inline void clearEDir() {
      stepCommand &= ~0x80;
    }  

    inline boolean xEnabled() {
      return (stepCommand & 0x1);
    }
    
    inline boolean yEnabled() {
      return (stepCommand & 0x2);
    }
    
    inline boolean zEnabled() {
      return (stepCommand & 0x4);
    }
    
    inline boolean eEnabled() {
      return (stepCommand & 0x8);
    }
    
    inline boolean xStep() {
      return (stepCommand & 0x100);
    }
    
    inline boolean yStep() {
      return (stepCommand & 0x200);
    }
    
    inline boolean zStep() {
      return (stepCommand & 0x400);
    }
    
    inline boolean eStep() {
      return (stepCommand & 0x800);
    }

    inline boolean xDir() {
      return (stepCommand & 0x10);
    }
    
    inline boolean yDir() {
      return (stepCommand & 0x20);
    }
    
    inline boolean zDir() {
      return (stepCommand & 0x40);
    }
    
    inline boolean eDir() {
      return (stepCommand & 0x80);
    }
};

#endif // STEP_COMMAND_H
