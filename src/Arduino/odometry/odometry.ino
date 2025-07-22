#include "Bitcraze_PMW3901.h"

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
// #include <cmath>

#define BNO055_SAMPLERATE_DELAY_MS (100)

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29);

// Using digital pin 5 for chip select
Bitcraze_PMW3901 flow(5);

float yaw, pitch, roll;
float dx_world, dy_world;
float dx_raw, dy_raw;
float x_new, y_new;

// Estimate angular velocity (omega) as change in yaw per loop iteration (rad/s)
static float last_yaw = 0;
static unsigned long last_time = millis();
unsigned long now = millis();
float dt = (now - last_time) / 1000.0f; // convert ms to seconds

float dyaw = yaw - last_yaw;


void setup()
{
  Serial.begin(115200);

  if (!flow.begin())
  {
    Serial.println("Initialization of the flow sensor failed");
    while (1)
    {
    }
  }
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1)
      ;
  }

  delay(1000);

  /* Use external crystal for better accuracy */
  bno.setExtCrystalUse(true);
}

int16_t deltaX, deltaY;

void loop()
{
  // Get motion count since last call
  flow.readMotionCount(&deltaX, &deltaY);
  sensors_event_t event;
  bno.getEvent(&event);

  yaw = event.orientation.z;

  dx_world = dx_raw * cos(yaw) - dy_raw * sin(yaw);
  dy_world = dx_raw * sin(yaw) - dy_raw * cos(yaw);

  x_new = deltaX + dx_world;
  y_new = deltaY + dy_world;

  // Create a struct to hold odometry data
  struct OdometryData
  {
    float theta;
    float x;
    float y;
    float vx;
    float vy;
    float omega;
  } odom;

  odom.theta = yaw;
  odom.x = x_new;
  odom.y = y_new;
  odom.vx = dx_world;
  odom.vy = dy_world;

  // Handle wrap-around for yaw (assuming yaw in degrees 0-360)
  if (dyaw > 180)
    dyaw -= 360;
  if (dyaw < -180)
    dyaw += 360;

  odom.omega = (dt > 0) ? (dyaw * DEG_TO_RAD) / dt : 0; // omega in rad/s

  last_yaw = yaw;
  last_time = now;

  // Send the struct as raw bytes over Serial
  Serial.write((uint8_t *)&odom, sizeof(odom));

  delay(100);
}