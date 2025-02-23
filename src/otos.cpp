#include "otos.hpp"
#include "Arduino.h"
#include "Wire.h"
#include "proto/messages.pb.h"
#include "sfeQwiicOtos.h"
#include <SparkFun_Qwiic_OTOS_Arduino_Library.h>

QwiicOTOS otos;

Status updateOTOS() {
  // Get the latest position, which includes the x and y coordinates, plus the
  // heading angle
  sfe_otos_pose2d_t pose;
  int success = otos.getPosition(pose);

  Status status = Status_init_zero;
  // pose.x = myPosition.x;
  // pose.y = myPosition.y;
  // pose.heading = myPosition.h;
  if (success == 0) {
    status.has_pos = true;
    status.pos.x = pose.x;
    status.pos.y = pose.y;
    status.has_heading = true;
    status.heading = pose.h;
  }
  return status;

  // Print measurement
  // Serial.println();
  // Serial.println("Position:");
  // Serial.print("X (Inches): ");
  // Serial.println(myPosition.x);
  // Serial.print("Y (Inches): ");
  // Serial.println(myPosition.y);
  // Serial.print("Heading (Degrees): ");
  // Serial.println(myPosition.h);

  // Wait a bit so we don't spam the serial port
  // delay(500);
}

bool trySetupOTOS() {
  // Attempt to begin the sensor
  // Serial.println("hmm");
  if (otos.begin() == false)
    return false;

  // Serial.println("OTOS connected!");

  // Serial.println("Ensure the OTOS is flat and stationary, then enter any key
  // to calibrate the IMU");

  // // Clear the serial buffer
  // while (Serial.available())
  //     Serial.read();
  // // Wait for user input
  // while (!Serial.available())
  //     ;

  // Serial.println("Calibrating IMU...");

  // Calibrate the IMU, which removes the accelerometer and gyroscope offsets
  otos.calibrateImu();

  // Reset the tracking algorithm - this resets the position to the origin,
  // but can also be used to recover from some rare tracking errors
  otos.resetTracking();
  return true;
}