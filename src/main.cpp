#include <Arduino.h>
#include "config.h"
#include "battery.h"
#include "motors_helper.h"
#include "ros_interface.h"
#include "bno055.h"

void setup() {
  Serial.begin(115200);

  battery_init();
  encoders_init();
  motors_init();

  if (!imu_init()) {
    Serial.println("IMU init failed. Publishing IMU will stay at last values.");
  }

  ros_init();
}

void loop() {
  motors_update();
  battery_update();
  ros_update();
}