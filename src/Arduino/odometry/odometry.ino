#include "Bitcraze_PMW3901.h"

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
//#include <cmath>

#define BNO055_SAMPLERATE_DELAY_MS (100)

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29);

// Using digital pin 5 for chip select
Bitcraze_PMW3901 flow(5);

float yaw,pitch,roll;
float dx_world, dy_world;
float dx_raw,dy_raw;
float x_new, y_new;

void setup() {
  Serial.begin(115200);

  if (!flow.begin()) {
    Serial.println("Initialization of the flow sensor failed");
    while(1) { }
  }
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
   
  delay(1000);

  /* Use external crystal for better accuracy */
  bno.setExtCrystalUse(true);
}

int16_t deltaX,deltaY;

void loop() {
  // Get motion count since last call
  flow.readMotionCount(&deltaX, &deltaY);

  Serial.print("X: ");
  Serial.print(deltaX);
  Serial.print(", Y: ");
  Serial.print(deltaY);
  Serial.print("\n");

  sensors_event_t event;
  bno.getEvent(&event);

  yaw = event.orientation.x;
  pitch = event.orientation.y;
  roll = event.orientation.z;

  Serial.print(F("Orientation: x: "));
  Serial.print(yaw);
  Serial.print(F(" y: "));
  Serial.print(pitch);
  Serial.print(F(" z: "));
  Serial.print(roll);
  Serial.println(F(""));


  dx_world = dx_raw * cos(yaw) - dy_raw * sin(yaw);
  dy_world = dx_raw * sin(yaw) - dy_raw * cos(yaw);

  x_new = deltaX + dx_world;
  y_new = deltaY + dy_world;

  Serial.print("displacement x: ");
  Serial.print(x_new);
  Serial.print(F(" y: "));
  Serial.println(y_new);

  delay(100);
}