#include "Sensors.h"

Sensors::Sensors(void) {
  Wire.begin();
}

// select the input of the I2C multiplexer
void Sensors::tcaselect(uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}

void Sensors::init(int id) {
  Serial.print("selecting id:");
  Serial.println(id);
  tcaselect(id);
  Serial.print("initialising AS5600 id:");
  Serial.println(id);
  if(ams5600.detectMagnet() == 0 ){
    while(1){
        if(ams5600.detectMagnet() == 1 ){
            Serial.print("Current Magnitude: ");
            Serial.println(ams5600.getMagnitude());
            break;
        }
        else{
            Serial.println("Can not detect magnet");
        }
        delay(1000);
    }
  }
}

double Sensors::getAngle(int id) {
  tcaselect(id);
  /* Raw data reports 0 - 4095 segments, which is 0.087 of a degree */
  double retVal = (ams5600.getRawAngle() * 0.087) - amsOffsets[id];
  
  if (retVal<=-180.0) retVal=360+retVal;
  if (retVal>180.0) retVal=360-retVal;
  return retVal;
}