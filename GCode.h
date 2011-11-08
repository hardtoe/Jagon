#ifndef GCODE_H
#define GCODE_H

#include <WProgram.h>
#include <math.h>

#include "Vec.h"

class GCode : public Vec<float,4> {
  private:
    char type;
    int opcode;
    float f;
    float s;
    float p;
    
  public:  
    inline void clear() {
      type = ' ';
      opcode = 0; 
      set(0, NAN);
      set(1, NAN);
      set(2, NAN);
      set(3, NAN);
      f = NAN;
      s = NAN;
      p = NAN;
    }
    
    inline char getType() {
      return type; 
    }
    
    inline int getOpcode() {
      return opcode;
    }
    
    inline float getX() {
      return get(0); 
    }
    
    inline float getY() {
      return get(1); 
    }
    
    inline float getZ() {
      return get(2); 
    }
    
    inline float getE() {
      return get(3); 
    }
    
    inline float getF() {
      return f;
    }
    
    inline float getS() {
      return s;
    }
    
    inline float getP() {
      return p;
    }
    

    inline void setType(char newType) {
      type = newType; 
    }
    
    inline void setOpcode(int newOpcode) {
      opcode = newOpcode;
    }
    
    inline void setX(float x) {
      set(0, x); 
    }
    
    inline void setY(float y) {
      set(1, y); 
    }
    
    inline void setZ(float z) {
      set(2, z); 
    }
    
    inline void setE(float e) {
      set(3, e); 
    }
    
    inline void setF(float newF) {
      f = newF; 
    }
    
    inline void setS(float newS) {
      s = newS;
    }
    
    inline void setP(float newP) {
      p = newP;
    }
};


#endif // GCODE_H
