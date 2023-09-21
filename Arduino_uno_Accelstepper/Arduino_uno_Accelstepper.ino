#include <SoftwareSerial.h>                         //libraries for the processing of the serial command and to controll the stepper motors
#include <SerialCommand.h>
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <TimerOne.h>
//#include "Sensors.h"
//#include "Controller.h"

SerialCommand SCmd;                                 // The SerialCommand object
//Sensors sensors;
//Controller controller;
AccelStepper newStepper(int stepPin, int dirPin, int enablePin, int maxSpeed, int Acceleration) {
  AccelStepper stepper = AccelStepper(stepper.DRIVER, stepPin,dirPin);
  stepper.setEnablePin(enablePin);
  stepper.setPinsInverted(false,false,true);
  stepper.setMaxSpeed(maxSpeed);
  stepper.setAcceleration(Acceleration);
  stepper.enableOutputs();
  return stepper;
}

// PUT THE NUMBER OF MOTORS YOU HAVE HERE:
const int MotorCount = 3;
AccelStepper steppers[MotorCount];

int uddir = 1;
int tempHoming = 0;   //maakt variabelen voor homing te cheken zodat het trager voor en achteruit gaat
unsigned long lastMillis;
bool b_move_complete = true;
const byte limitSwitch_x = 9;   //pin for the limitswitch on CNC shield NC (PullUp) correct
const byte limitSwitch_a = 11;  //pin for the limitswitch on CNC shield NC (PullUp) 
const byte limitSwitch_y = 10;  //pin for the limitswitch on CNC shield NC (PullUp)
const byte limitSwitch_z = 12;  //pin for the limitswitch on CNC shield NC (PullUp)


bool switchFlipped = false; //stores the status for flipping
bool previousFlip = true; //stores the previous state for flipping - needed for the direction change

int lockx = 0;    //make a set/reset variable for the motors
int locka = 0;    // a and x are the motors at the bottom
int locky = 0;    // motor to go left and right
int lockz = 0;    // motor for the camera


long stepperPos[MotorCount] = {0, 0, 0};
long stepsPerFullTurn[MotorCount] = {16000, 16000, 16000};

void setup() {
  // newStepper(int stepPin, int dirPin, int enablePin, int maxSpeed, int Acceleration)
  steppers[0] = newStepper(2,5,8, 2000, 1500);
  steppers[1] = newStepper(3,6,8, 1500, 1000);
  steppers[2] = newStepper(4,7,8, 1500, 1000);

  pinMode(8, OUTPUT);     //enable pin 8 hardcoden in pinMode
  digitalWrite(8, LOW);

  pinMode(limitSwitch_x, INPUT_PULLUP);
  pinMode(limitSwitch_a, INPUT_PULLUP);
  pinMode(limitSwitch_y, INPUT_PULLUP);
  pinMode(limitSwitch_z, INPUT_PULLUP);

  // Max number of function is 10, so you can't add more functions ;-;
  SCmd.addCommand("M", move_stepper);
  SCmd.addCommand("V", change_velocity);
  SCmd.addCommand("A", change_acceleration);
  SCmd.addCommand("STOP", stop_all);
  SCmd.addCommand("Home", homing);
  SCmd.addCommand("HomeZ", homingZ);
  SCmd.addCommand("Info", send_info);
  SCmd.addCommand("Ready", check_move_complete);
  SCmd.addCommand("Position", check_position);
  SCmd.addCommand("completed?", is_complete);
  SCmd.addDefaultHandler(unrecognized);

  Serial.begin(115200);
  Serial.println("HangingArm");
  //
  Timer1.initialize(500);
  Timer1.attachInterrupt(runSteppers);

}
void runSteppers(void) {

  for (int i = 0; i < MotorCount; i++) {
    steppers[i].run(); 
  }

}

void loop() {
  SCmd.readSerial(); 
  limitswitch();
  if (millis() - lastMillis > 10) {
    for (int i=0; i<MotorCount; i++) {
      //double sensorPosition = convert(sensors.getAngle(i), i);
      //double motorPosition = steppers[i].currentPosition();
      //steppers[i].setCurrentPosition((0.9*motorPosition + 0.1*sensorPosition));
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
        steppers[0].moveTo(10);
        delay(100);
        int lockx = 1;
      }
    if (digitalRead(limitSwitch_y) == 1)
      {
        stop_spec(1);
        steppers[1].setCurrentPosition(0);
        steppers[1].moveTo(10);
        delay(100);
        int locky = 1;
      }
    if (digitalRead(limitSwitch_z) == 1)
      {
        stop_spec(2);
        steppers[2].setCurrentPosition(0);
        steppers[2].moveTo(4);
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

void change_velocity()    //function called when a Serial command is received
{
  char *arg;
  int velocity;

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
  Serial.println(velocity);
  for (int i = 0; i < MotorCount; i++) {
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
  Serial.println("stopping");

  for (int i = 0; i < MotorCount; i++) {
    stop_spec(i);
  }
}
void homing(){
  Serial.println("Home, x and y use -100000, other joints need angle turned to original reference (to be determined)");
  for (int i = 0; i < MotorCount; i++) {steppers[i].move(-100000);}
  tempHoming = 1;
  }

void homingZ() {
  Serial.println("Homing x, y and z using move -100000");  // check dir
  
  if (digitalRead(limitSwitch_z) == 0) {
    steppers[2].move(100000);
  }
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

  // checks the stepper index (motor can do weird stuff if you go over the motorcount)
  step_idx = atoi(arg);
  if (step_idx < 0 || step_idx >= MotorCount) {
    Serial.println("Not recognized: Invalid Stepper Index, pleas restart if unstable");
    Serial.print("Unrecognized index is: ");   Serial.println(step_idx);
    return;
  }

// Function above should do this but better. If it indeed works, delete commented func -- Ya boy, Juicy Potat
  // if (step_idx < 0) {
  //   Serial.print("Not recognized:");   Serial.println(step_idx);  return;
  //   Serial.print("ID ");
  //   Serial.print(step_idx);}

  arg = SCmd.next();

  if (arg == NULL)   {Serial.println("Not recognized: No height parameter given");
                      return;}

  angle = atof(arg);

  if (angle == 0) {Serial.println("Not recognized: Height parameter not parsed");
                      return;}

  Serial.print("moving ");
  Serial.println(angle);
  Serial.println(step_idx);
  steps = angle;  // if sensors implemented then convert(angle, step_idx)
  //TODO FIX angle parameter and set up a limiter factor
  b_move_complete = false;
  steppers[step_idx].moveTo(steps);
}

double convert(double angle, int i) {double steps;
                            steps = angle*stepsPerFullTurn[i]/360.0;
                            return steps;}

void is_complete() {
  for (int i = 0; i < MotorCount; i++){
    if (steppers[i].distanceToGo() == 0)  {continue;}
    else  {return;}
  }
  Serial.print("Complete");
  }


void check_position() {
  //TODO have it memorize the angles measured by the magnets
  for (int i = 0; i < MotorCount; i++){stepperPos[i] = steppers[i].currentPosition();}
  Serial.println(String(stepperPos[0]) + " " + String(stepperPos[1]) + " " + String(stepperPos[2]));
}
