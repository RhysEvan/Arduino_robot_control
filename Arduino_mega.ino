#include <SoftwareSerial.h>                         //libraries for the processing of the serial command and to controll the stepper motors
#include <SerialCommand.h>
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <TimerThree.h>
#include "Sensors.h"
//#include "Controller.h"

SerialCommand SCmd;                                 // The SerialCommand object
Sensors sensors;
//Controller controller;
AccelStepper newStepper(int stepPin, int dirPin, int enablePin) {
  AccelStepper stepper = AccelStepper(stepper.DRIVER, stepPin,dirPin);
  stepper.setEnablePin(enablePin);
  stepper.setPinsInverted(false,false,true);
  stepper.setMaxSpeed(500);
  stepper.setAcceleration(2000);
  stepper.enableOutputs();
  return stepper;
}
AccelStepper steppers[6];

MultiStepper msteppers;

int uddir = 1;
unsigned long lastMillis;
bool b_move_complete = true;
const byte limitSwitch_x = 3; //pin for the microswitch using attachInterrupt()
const byte limitSwitch_y = 14; //pin for the microswitch using attachInterrupt()

bool switchFlipped = false; //stores the status for flipping
bool previousFlip = true; //stores the previous state for flipping - needed for the direction change

int lockx = 0;
int locky = 0;
long stepperPos[6] = {0, 0, 0, 0, 0, 0};
long stepsPerFullTurn[6] = {16000, 16000, 16000, 1350, 1350, 1350};

void setup() {
  steppers[0] = newStepper(26,28,24);
  steppers[1] = newStepper(32,47,45);
  steppers[2] = newStepper(36,34,30);
  steppers[3] = newStepper(54,55,38);
  steppers[4] = newStepper(60,61,56);
  steppers[5] = newStepper(46,48,62);
  for (int i = 0; i < 6; i++){msteppers.addStepper(steppers[i]);}

  SCmd.addCommand("M", move_stepper);
  SCmd.addCommand("V", change_velocity);
  SCmd.addCommand("STOP", stop_all);
  SCmd.addCommand("Home", homing);
  SCmd.addCommand("Info", send_info);
  SCmd.addCommand("Pos", send_position);
  SCmd.addCommand("Ready", check_move_complete);
  SCmd.addCommand("Position", check_position);
  SCmd.addCommand("completed?", is_complete);
  SCmd.addDefaultHandler(unrecognized);

  Serial.begin(115200);
  Serial.println("HangingArm");
  //
  Timer3.initialize(500);
  Timer3.attachInterrupt(runSteppers);

}
void runSteppers(void) {
  msteppers.run();  
}

void loop() {
  SCmd.readSerial(); 
  limitswitch();
  if (millis() - lastMillis > 10) {
    for (int i=0; i<6; i++) {
      double sensorPosition = convert(sensors.getAngle(i), i);
      double motorPosition = steppers[i].currentPosition();
      steppers[i].setCurrentPosition((0.9*motorPosition + 0.1*sensorPosition));
      msteppers.moveTo(stepperPos);
      // updateSpeeds();
    }
  }
  
}

// This gets set as the default handler, and gets called when no other command matches.
void unrecognized()
{
  Serial.println("Not recognized");            //returns not ok to software

}

void send_info() {
  Serial.println("Hanging Arm");
}

void send_position() {
  Serial.println("Hanging Arm");
}

void limitswitch(){
  
  if (digitalRead(limitSwitch_x) == 0 && lockx==0) {
    steppers[0].setCurrentPosition(0);
    stop_spec(0);
    lockx = lockx+1;    
    }
  if (digitalRead(limitSwitch_y) == 0 && locky==0) {
    steppers[1].setCurrentPosition(0);
    stop_spec(1);
    locky = locky+1;
    }
  
  if (digitalRead(limitSwitch_x) == 1 && lockx==1) {
    steppers[0].setCurrentPosition(0);
    stop_spec(0);
    lockx = lockx-1;    
    }
  if (digitalRead(limitSwitch_y) == 1 && locky==1) {
    steppers[1].setCurrentPosition(0);
    stop_spec(1);
    locky = locky-1;
    }

  }

void change_velocity()    //function called when a serial command is received
{
  char *arg;
  float velocity;

  arg = SCmd.next();
  if (arg == NULL) {
    Serial.println("Not recognized: No Velocity given");
    return;
  }

  velocity = atoi(arg);
  if (velocity == 0) {
    Serial.println("Not recognized: Velocity parameter could not get parsed");
    return;
  }

  for (int i = 0; i <= 6; i++) {
    steppers[i].setMaxSpeed(velocity);
  }

}

void check_move_complete() {

  if (b_move_complete) {
    Serial.println("Ready for next command");
    return;
  }

  bool b_all_done = true;
  for (int i = 0; i <= 6; i++) {
    if (steppers[i].distanceToGo() > 0) {
      b_all_done = false;
    }
  }

  if (b_all_done) {
    Serial.println("Ready for next command");
    b_move_complete = true;
  }
  else {
    Serial.println("Busy");
  }

}

void stop_all() {
  for (int i = 0; i < 6; i++) {
    stop_spec(i);
  }
}
void homing(){
  Serial.println("Home, x and y use -100000, other joints need angle turned to original reference (to be determined)");
  //for (int i = 0; i < 6; i++) {steppers[i].move(-100000);}
  }

void stop_spec(int value) {steppers[value].move(0);}

void move_stepper() {

  char *arg;
  int step_idx;
  double angle;
  double steps;

  arg = SCmd.next();

  if (arg == NULL)  {Serial.println("Not recognized: Stepper Number" );
                      return;}

  step_idx = atoi(arg);

  if (step_idx < 0) {
    Serial.print("Not recognized:");   Serial.println(step_idx);  return;
    Serial.print("ID ");
    Serial.print(step_idx);}

  arg = SCmd.next();

  if (arg == NULL)   {Serial.println("Not recognized: No height parameter given");
                      return;}

  angle = atof(arg);

  if (angle == 0) {Serial.println("Not recognized: Height parameter not parsed");
                      return;}

  Serial.print("moving ");
  Serial.print(angle);
  steps = convert(angle, step_idx);
  stepperPos[step_idx] = steps;
  //TODO FIX angle parameter and set up a limiter factor
  b_move_complete = false;
}

double convert(double angle, int i) {double steps;
                            steps = angle*stepsPerFullTurn[i]/360.0;
                            return steps;}

void is_complete() {
  for (int i = 0; i < 6; i++){
    if (steppers[i].distanceToGo() == 0)  {continue;}
    else  {return;}
  }
  Serial.print("Complete");
  }


void check_position() {
  //TODO have it memorize the angles measured by the magnets
  for (int i = 0; i < 6; i++){stepperPos[i] = steppers[i].currentPosition();}
  Serial.println(String(stepperPos[0]) + " " + String(stepperPos[1]) + " " + String(stepperPos[3]) + " " + String(stepperPos[4]) + " " + String(stepperPos[5]));
}
