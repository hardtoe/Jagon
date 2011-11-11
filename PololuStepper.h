#ifndef POLOLU_STEPPER_H
#define POLOLU_STEPPER_H

#include <WProgram.h>
#include "fastio.h"

#define PololuStepper(STEP_PIN, DIR_PIN, ENABLE_PIN, INVERT_DIR)\
  class {\
    private:\
    public:\
      void init() {\
        SET_OUTPUT(STEP_PIN);\
        SET_OUTPUT(DIR_PIN);\
        SET_OUTPUT(ENABLE_PIN);\
      }\
    \
      void inline enable(boolean value) {WRITE(ENABLE_PIN, value);}\
      void inline step(boolean value) {WRITE(STEP_PIN, value);}\
      void inline setDirection(boolean value) {WRITE(DIR_PIN, value ^ INVERT_DIR);}\
      boolean inline getDirection() {return READ(DIR_PIN) ^ INVERT_DIR;}\
  }\

#endif // POLOLU_STEPPER_H
