//#define DEBUG

#include "configuration.h"
#include "pins.h"

#include "StepperDriver.h"
#include "HostInterface.h"
#include "TempSensor.h"
#include "Heater.h"
#include "GCode.h"
#include "GCodeDecoder.h"
#include "HeaterController.h"
#include "StepCommand.h"
#include "MoveCommand.h"
#include "MovePlanner.h"
#include "StepPlanner.h"
#include "TimerOne.h"

#include "Assert.h"

// queues for passing commands through pipeline stages
CircularBuffer<GCode, GCODE_Q_SIZE> gCodeQueue;
CircularBuffer<MoveCommand, MOVE_COMMAND_Q_SIZE> moveCommandQueue;
CircularBuffer<StepCommand, STEP_COMMAND_Q_SIZE> stepCommandQueue;

// interrupt-driven stepper driver 
StepperDriver stepperDriver(&stepCommandQueue);
StepperDriver* endstops = &stepperDriver;

// step planner
StepPlanner stepPlanner(&moveCommandQueue, &stepCommandQueue);

// move planner
MovePlanner movePlanner(&moveCommandQueue, endstops);

// heater controller
TempSensor tempZero(TEMP_0_PIN);
Heater heaterZero(HEATER_0_PIN, &tempZero);

TempSensor tempOne(TEMP_1_PIN);
Heater heaterOne(HEATER_1_PIN, &tempOne);

HeaterController heaterController(&heaterZero, &heaterOne);

// gcode decoder
GCodeDecoder gCodeDecoder(&gCodeQueue);

// host interface
HostInterface hostInterface(&gCodeQueue);


void stepperDriverInterrupt() {
  stepperDriver.interrupt(); 
}

void setup() {                
  Serial.begin(115200);
  
  // add gcode handlers
  gCodeDecoder.addHandler(&movePlanner);
  gCodeDecoder.addHandler(&heaterController);

  // setup stepper driver timer
  Timer1.initialize(100000);
  Timer1.attachInterrupt(&stepperDriverInterrupt);

  // turn on power supply
  SET_OUTPUT(PS_ON_PIN);
  WRITE(PS_ON_PIN, LOW);

  // set prescale to 16 for fast analog temperature reads
  sbi(ADCSRA,ADPS2);
  cbi(ADCSRA,ADPS1);
  cbi(ADCSRA,ADPS0);

  Serial.println("start");
}

void loop() {
  hostInterface.tick();
  gCodeDecoder.tick();
  stepPlanner.tick();
  heaterZero.tick();
  heaterOne.tick();              
}
