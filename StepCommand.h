#ifndef STEP_COMMAND_H
#define STEP_COMMAND_H

#include <WProgram.h>

#include "Vec.h"


// stepCommand = {step[3:0],direction[3:0],enable[3:0],}
class StepCommand {
  private:
    unsigned int stepCommand;
    unsigned int stepDelay;

 
  public: 
    inline void clear() {
       stepCommand = 0; 
       setNewEnableDirection();
    }

    inline void setPrototype(StepCommand rhs) {
      stepCommand = rhs.stepCommand & 0x10FF;
      stepDelay = rhs.stepDelay;
    }

    inline boolean hasNewEnableDirection() {
        return (stepCommand & 0x1000) != 0;
    }

    inline void setNewEnableDirection() {
       stepCommand |= 0x1000; 
    }

    inline void clearNewEnableDirection() {
       stepCommand &= ~0x1000; 
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
      return (stepCommand & 0x1) != 0;
    }
    
    inline boolean yEnabled() {
      return (stepCommand & 0x2) != 0;
    }
    
    inline boolean zEnabled() {
      return (stepCommand & 0x4) != 0;
    }
    
    inline boolean eEnabled() {
      return (stepCommand & 0x8) != 0;
    }
    
    inline boolean xStep() {
      return (stepCommand & 0x100) != 0;
    }
    
    inline boolean yStep() {
      return (stepCommand & 0x200) != 0;
    }
    
    inline boolean zStep() {
      return (stepCommand & 0x400) != 0;
    }
    
    inline boolean eStep() {
      return (stepCommand & 0x800) != 0;
    }

    inline boolean xDir() {
      return (stepCommand & 0x10) != 0;
    }
    
    inline boolean yDir() {
      return (stepCommand & 0x20) != 0;
    }
    
    inline boolean zDir() {
      return (stepCommand & 0x40) != 0;
    }
    
    inline boolean eDir() {
      return (stepCommand & 0x80) != 0;
    }
};

#endif // STEP_COMMAND_H
