#include "otos.hpp"
#include "Arduino.h"
#include "Wire.h"
#include "proto/messages.pb.h"
#include <SparkFun_Qwiic_OTOS_Arduino_Library.h>

QwiicOTOS otos = QwiicOTOS();

OTOSResult updateOTOS() {
  // Get the latest position, which includes the x and y coordinates, plus the
  // heading angle
  sfe_otos_pose2d_t otos_pose;
  int success = otos.getPosition(otos_pose);

  Pose pose = Pose_init_zero;
  // pose.x = myPosition.x;
  // pose.y = myPosition.y;
  // pose.heading = myPosition.h;
  if (success == 0) {
    pose.x = otos_pose.x;
    pose.y = otos_pose.y;
    pose.heading = otos_pose.h;
  }
  return OTOSResult{pose, success};

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

bool trySetupOTOS(bool calibrate) {
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

  // delay(2000);
  if (calibrate) {
    Serial.println("Calibrating IMU...");

    // Calibrate the IMU, which removes the accelerometer and gyroscope offsets
    otos.calibrateImu();

    // Reset the tracking algorithm - this resets the position to the origin,
    // but can also be used to recover from some rare tracking errors
  }
  otos.resetTracking();
  return true;
}

void recoverOTOS() {
  float ang;
  float lin;
  sfe_otos_pose2d_t off;
  sfe_otos_pose2d_t pose;
  otos.getAngularScalar(ang);
  otos.getLinearScalar(lin);
  otos.getOffset(off);
  otos.getPosition(pose);

  trySetupOTOS(false);
  otos.setAngularScalar(ang);
  otos.setLinearScalar(lin);
  otos.setOffset(off);
  otos.setPosition(pose);
}

void processOTOSCommand(const Command& command) {
  if (command.has_reset && command.reset) {
    // Serial.println("resetting!");
    // Serial.flush();
    digitalWrite(3, HIGH);
    trySetupOTOS(false);
    // delay(1000);
    digitalWrite(3, LOW);
  }

  if (command.has_calibrate && command.calibrate) {
    // Serial.println("calibrating!");
    // Serial.flush();
    digitalWrite(3, HIGH);
    trySetupOTOS(true);
    digitalWrite(3, LOW);
  }

  if (command.has_scalar) {
    // Serial.println("set scalar!");
    // Serial.println(command.scalar.angular);
    // Serial.println(command.scalar.linear);
    // Serial.flush();
    otos.setLinearScalar(command.scalar.linear);
    otos.setAngularScalar(command.scalar.angular);
    // Serial.println("done setting scalar!");
    // Serial.flush();
  }

  if (command.has_offset) {
    Serial.println("set offset!");
    Serial.flush();
    sfe_otos_pose2d_t pose;
    pose.x = command.offset.x;
    pose.y = command.offset.y;
    pose.h = command.offset.heading;
    otos.setOffset(pose);
  }

  if (command.has_set_pose) {
    Serial.println("set pose!");
    Serial.flush();
    sfe_otos_pose2d_t pose;
    pose.x = command.set_pose.x;
    pose.y = command.set_pose.y;
    pose.h = command.set_pose.heading;
    otos.setPosition(pose);
  }
  // delay(1000);
}