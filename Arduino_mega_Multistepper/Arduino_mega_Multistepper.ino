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
AccelStepper newStepper(int stepPin, int dirPin, int enablePin, int maxSpeed) {
  AccelStepper stepper = AccelStepper(stepper.DRIVER, stepPin,dirPin);
  stepper.setEnablePin(enablePin);
  stepper.setPinsInverted(false,false,true);
  stepper.setMaxSpeed(maxSpeed);
  stepper.enableOutputs();
  return stepper;
}
// PUT THE NUMBER OF MOTORS YOU HAVE HERE:
const int MotorCount = 6;
AccelStepper steppers[MotorCount];

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
long stepperPos[MotorCount] = {0, 0, 0, 0, 0, 0};
long stepsPerFullTurn[MotorCount] = {16000, 16000, 16000, 1350, 1350, 1350};

void setup() {
  steppers[0] = newStepper(26,28,24, 2000);
  steppers[1] = newStepper(32,47,45, 2000);
  steppers[2] = newStepper(36,34,30, 2000);
  steppers[3] = newStepper(54,55,38, 2000);
  steppers[4] = newStepper(60,61,56, 2000);
  steppers[5] = newStepper(46,48,62, 2000);
  for (int i = 0; i < MotorCount; i++){msteppers.addStepper(steppers[i]);}

  SCmd.addCommand("M", move_stepper);
  SCmd.addCommand("V", change_velocity);
  SCmd.addCommand("A", change_acceleration);
  SCmd.addCommand("STOP", stop_all);
  SCmd.addCommand("Home", homing);
  SCmd.addCommand("Info", send_info);
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
    for (int i=0; i<MotorCount; i++) {
      //double sensorPosition = convert(sensors.getAngle(i), i);
      //double motorPosition = steppers[i].currentPosition();
      //steppers[i].setCurrentPosition((0.9*motorPosition + 0.1*sensorPosition));
      //msteppers.moveTo(stepperPos);
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

void limitswitch(){
  if (tempHoming == 0){
    if (digitalRead(limitSwitch_x) == 1 && digitalRead(limitSwitch_a) == 1) 
      {
        stop_spec(0);
        steppers[0].setCurrentPosition(0);
      }
    if (digitalRead(limitSwitch_y) == 1) 
      {
        stop_spec(1);
        steppers[1].setCurrentPosition(0);
      }
    if (digitalRead(limitSwitch_z) == 1) 
      {
        stop_spec(2);
        steppers[2].setCurrentPosition(0);
      }
  }
  if (tempHoming == 1){
    if (digitalRead(limitSwitch_x) == 1 && digitalRead(limitSwitch_a) == 1) 
      {
        stop_spec(0);
        steppers[0].setCurrentPosition(0);
        steppers[0].moveTo(400);
        delay(100);
        int lockx = 1;
      }
    if (digitalRead(limitSwitch_y) == 1)
      {
        stop_spec(1);
        Serial.println("y is stopping");
        Serial.println(String(digitalRead(limitSwitch_y)));
        steppers[1].setCurrentPosition(0);
        steppers[1].moveTo(200);
        delay(100);
        int locky = 1;
      }
    if (digitalRead(limitSwitch_z) == 1)
      {
        stop_spec(2);
        steppers[2].setCurrentPosition(0);
        steppers[2].moveTo(120);
        delay(100);
        int lockz = 1;
      }
    if (lockx == 1 && locky == 1 && lockz == 1){
      tempHoming = 0;
      lockx = 0;
      locky = 0;
      lockz = 0;
    }
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

void change_acceleration() {
  char *arg;
  int acceleration;
  arg = SCmd.next();
  if (arg == NULL) {
    Serial.println("Not recognized: No Acceleration given");
    return;
  }
  acceleration = atoi(arg);
  if (acceleration == 0) {
    Serial.println("Not recognized: Acceleration parameter could not get parsed");
    return;
  }
  for (int i = 0; i < MotorCount; i++) {
    steppers[i].setAcceleration(acceleration);
  }
}

void check_move_complete() {
  if (b_move_complete) {
    Serial.println("Ready for next command");
    return;
  }
  bool b_all_done = true;
  for (int i = 0; i <= MotorCount; i++) {
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
  for (int i = 0; i < MotorCount; i++) {
    stop_spec(i);
  }
}
void homing(){
  for (int i = 0; i < MotorCount; i++) {steppers[i].move(-100000);}
  tempHoming = 1;
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
  if (step_idx < 0 || step_idx >= MotorCount) {
    Serial.println("Not recognized: Invalid Stepper Index, pleas restart if unstable");
    Serial.print("Unrecognized index is: ");   Serial.println(step_idx);
    return;
  }
  arg = SCmd.next();
  if (arg == NULL)   {Serial.println("Not recognized: No height parameter given");
                      return;}
  angle = atof(arg);
  if (angle == 0) {Serial.println("Not recognized: Height parameter not parsed");
                      return;}
  Serial.print("moving ");
  Serial.print(angle);
  //steps = convert(angle, step_idx);
  //stepperPos[step_idx] = steps;
  //TODO FIX angle parameter and set up a limiter factor
  steps = angle;
  b_move_complete = false;
  steppers[step_idx].moveTo(steps);
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
  for (int i = 0; i < MotorCount; i++){stepperPos[i] = steppers[i].currentPosition();}
  Serial.println(String(stepperPos[0]) + " " + String(stepperPos[1]) + " " + String(stepperPos[3]) + " " + String(stepperPos[4]) + " " + String(stepperPos[5]));
}
