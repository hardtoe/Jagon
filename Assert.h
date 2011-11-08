#ifndef ASSERT_H
#define ASSERT_H

#include <avr/pgmspace.h>

// from: http://www.controllerprojects.com/2011/05/23/saving-ram-space-on-arduino-when-you-do-a-serial-printstring-in-quotes/
void showString (PGM_P s) {
  char c;
  while ((c = pgm_read_byte(s++)) != 0) {
    Serial.print(c);
  }
}

#ifdef DEBUG
#define ASSERT(CONDITION)					               \
do {                                                                           \
  if (!(CONDITION)) {                                                          \
    showString(PSTR("!! Assertion failure at "));                              \
    showString(PSTR(__FILE__));                                                \
    showString(PSTR(":"));                                                     \
    Serial.print(__LINE__);                                                    \
    showString(PSTR(" "));                                                     \
    showString(PSTR(#CONDITION));                                              \
    Serial.println();                                                          \
  }                                                                            \
} while (0)                                                                    
#else
#define ASSERT(CONDITION) do {} while(0)
#endif // ASSERT

#ifdef DEBUG
#define LOG(MSG)					                       \
do {                                                                           \
  Serial.print("// ");                                                         \
  showString(PSTR(MSG));                                                       \
  Serial.println();                                                         \
} while (0)                                                                    
#else
#define LOG(MSG) do {} while(0)
#endif // ASSERT


#ifdef DEBUG
#define DUMP(MSG, VAL)					                       \
do {                                                                           \
  Serial.print("// ");                                                         \
  showString(PSTR(MSG));                                                       \
  Serial.println(VAL);                                                         \
} while (0)                                                                    
#else
#define DUMP(MSG, VAL) do {} while(0)
#endif // ASSERT

#endif // ASSERT_H
