#include <AccelStepper.h>
#include <MultiStepper.h>
#include "Sensors.h"
#include <TimerThree.h>

AccelStepper newStepper(int stepPin, int dirPin, int enablePin) {
  AccelStepper stepper = AccelStepper(stepper.DRIVER, stepPin,dirPin);
  stepper.setEnablePin(enablePin);
  stepper.setPinsInverted(false,false,true);
  stepper.setMaxSpeed(400);
  stepper.setAcceleration(2000);
  stepper.enableOutputs();
  return stepper;
}

unsigned long time;


AccelStepper steppers[6];

int incomingByte = 0; // for incoming serial data

MultiStepper msteppers;

long stepperPos[6] = {0, 0, 0, 0, 0, 0};
long stepsPerFullTurn[6] = {16000, 16000, 16000, 1350, 1350, 1350};

Sensors sensors;

void setup()
{  
  // Change these to suit your stepper if you want
  
  Serial.begin(9600);
  // init steppers based on RAMPS 1.4 pins
  steppers[0] = newStepper(26,28,24);
  steppers[1] = newStepper(32,47,45);
  steppers[2] = newStepper(36,34,30);
  steppers[3] = newStepper(54, 55, 38);
  steppers[4] = newStepper(60,61,56);
  steppers[5] = newStepper(46,48,62);

  msteppers.addStepper(steppers[0]);
  msteppers.addStepper(steppers[1]);
  msteppers.addStepper(steppers[2]);
  msteppers.addStepper(steppers[3]);
  msteppers.addStepper(steppers[4]);
  msteppers.addStepper(steppers[5]);

  /// init all the position sensors
  sensors = Sensors();
  for (int i=0; i<6; i++) {
    sensors.init(i);
  }
 
  // run stepper motors every 0.5ms
  Timer3.initialize(500);
  Timer3.attachInterrupt(runSteppers);
}

void runSteppers(void) {
  msteppers.run();
}

int tokenPos=0;
char token[10];

char whichAxis = NULL;

int ctoi( int c )
{
    return c - '0';
}

void moveDegrees(int stepper,double degrees) {
  double stepPos = stepsPerFullTurn[stepper] * degrees / 360.0 ;
  stepperPos[stepper] = (long)stepPos;
}

long currentPos[6] = {0,0,0,0,0,0};
void updateCurrentPos() {
  currentPos[0]=steppers[0].currentPosition();
  currentPos[1]=steppers[1].currentPosition();
  currentPos[2]=steppers[2].currentPosition();
  currentPos[3]=steppers[3].currentPosition();
  currentPos[4]=steppers[4].currentPosition();
  currentPos[5]=steppers[5].currentPosition();
}
float speed[6] = {800,900,800,200,200,200};
void updateSpeeds() {
  // if (speed[0] > 0 ) steppers[0].setMaxSpeed(speed[0]);
  // if (speed[1] > 0 ) steppers[1].setMaxSpeed(speed[1]);
  // if (speed[2] > 0 ) steppers[2].setMaxSpeed(speed[2]);
  // if (speed[3] > 0 ) steppers[3].setMaxSpeed(speed[3]);
  // if (speed[4] > 0 ) steppers[4].setMaxSpeed(speed[4]);
  // if (speed[5] > 0 ) steppers[5].setMaxSpeed(speed[5]);
}

void setSpeeds(float mtime) {
  updateCurrentPos();
  time=millis();
  for (int i=0;i<6;i++) {
    long difference=stepperPos[i] - currentPos[i];
    speed[i] = (float)difference/(mtime/1000);
  }
  
  updateSpeeds();

}

float moveTime = 0;

void readSerial() {
  if (Serial.available() > 0) {
    // read the incoming byte:
    incomingByte = Serial.read();
    
    
    if (incomingByte!=' ' && incomingByte!=10) {
      if (whichAxis == NULL) {
        switch (incomingByte) {
          // home
          case 'h':
            for (int i=0; i<6; i++) {
              moveDegrees(i, 0);
            }
            break;
          //set home
          case 's':
            steppers[0].setCurrentPosition(0);
            steppers[1].setCurrentPosition(0);
            steppers[2].setCurrentPosition(0);
            steppers[3].setCurrentPosition(0);
            steppers[4].setCurrentPosition(0);
            steppers[5].setCurrentPosition(0);
            for (int i=0; i<6; i++) {
              moveDegrees(i, 0);
            }
            break;
          //set move time
          case 't':
            whichAxis = 't';
          //move
          default: 
            whichAxis = incomingByte;
            break;
        }
      } else {
        token[tokenPos++]=(char)incomingByte;
      }
    } else {
      token[tokenPos]=NULL;
      double distance = atof(token);
      if (whichAxis=='t') {
        moveTime = distance;
      } else {
        int stepper = ctoi(whichAxis);

        moveDegrees(stepper, distance);
      }
      whichAxis = NULL;
      token[tokenPos]=NULL;
      tokenPos=0;
    }
    
    if (incomingByte==10) {
      msteppers.moveTo(stepperPos);
      //setSpeeds(moveTime);
    }
  }
}

bool oldRun = false;

unsigned long delaytime = millis();

void loop()
{

  readSerial();

  if (millis() - delaytime > 10) {
    for (int i=0; i<6; i++) {
      double sensorPosition = sensors.getAngle(i)*stepsPerFullTurn[i]/360.0;
      double motorPosition = steppers[i].currentPosition();
      steppers[i].setCurrentPosition((0.9*motorPosition + 0.1*sensorPosition));
      msteppers.moveTo(stepperPos);
      // updateSpeeds();
    }
  }

}
