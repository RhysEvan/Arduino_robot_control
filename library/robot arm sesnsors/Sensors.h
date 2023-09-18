#ifndef Sensors_h
#define Sensors_h
#include "Arduino.h" 
#include <Wire.h>
#include <AS5600.h>

#define TCAADDR 0x70

class Sensors {
public:
	Sensors();
  void init(int);
  double getAngle(int);
private:
  AMS_5600 ams5600;
  double amsOffsets[6] = {154,254,287,215,219,322};
  void tcaselect(uint8_t);
};
#endif