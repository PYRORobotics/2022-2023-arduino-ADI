//
// Created by charles on 9/13/22.
//
#include "Arduino.h"
#include "Wire.h"
#include "cobs/cobs.h"
#include "otos.hpp"
#include "proto/messages.pb.h"
#include "sfeQwiicOtos.h"
#include <cobs/Print.h>
#include <cobs/Stream.h>
#include <pb_arduino.h>
#include <stdio.h>

#define DE 9
#define RE 8
#define SEND HIGH
#define RECEIVE LOW

#define ITERATION_DELAY_MS 10
#define NAVX_SENSOR_DEVICE_I2C_ADDRESS_7BIT 0x32
#define NUM_BYTES_TO_READ 8
#define MAX_READ_TIME 10000

// packetio::COBSStream cobs_in(Serial);
// pb_istream_s pb_in = as_pb_istream(cobs_in);

// packetio::COBSPrint cobs_out(Serial);
// pb_ostream_s pb_out = as_pb_ostream(cobs_out);
// pb_ostream_s pb_out = as_pb_ostream(Serial);

Command command = Command_init_zero;
bool otosInitialized = false;
bool hasCalibrated = false;
unsigned long sparkfunTimeout = millis();
void setup() {
  Serial.begin(115200);
  pinMode(DE, OUTPUT);
  pinMode(RE, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  Wire.begin();
  delay(5000);
  otosInitialized = trySetupOTOS();
  if (otosInitialized) {
    hasCalibrated = true;
    sparkfunTimeout = millis();
  }
  Serial.println("setup!");
}
// int32_t i = 5;
char encode_buff[100];
char decode_buff[100];
char buff_cobs[100];
char print_buff[20];

void sendMessage(Pose pose) {
  while (!Serial.availableForWrite()) {
  }
  digitalWrite(DE, SEND);
  digitalWrite(RE, SEND);
  // delayMicroseconds(1000);
  // i++;

  // Serial.flush();
  // Serial.println("basic setup!");
  // delay(500);
  pb_ostream_s pb_out = pb_ostream_from_buffer((uint8_t *)&encode_buff, 30);
  // Serial.flush();
  // Serial.println("encode buf!");
  // delay(200);
  bool success = pb_encode(&pb_out, Pose_fields, &pose);

  // Serial.println("protobuf encode!");
  // delay(200);

  if (!success) {
    digitalWrite(3, HIGH);
    Serial.println("sad");
    delay(5);
  } else {
    digitalWrite(3, LOW);
  }
  bool write_success = true;

  cobs_encode_result encodeResult =
      cobs_encode_mine(&buff_cobs, 30, &encode_buff, pb_out.bytes_written);
  if (encodeResult.status != COBS_ENCODE_OK) {
    printf("COBS encode failed!\n");
    // printf("COBS status: %d\n", result.status);
    digitalWrite(3, HIGH);
    delay(500);
  }
  buff_cobs[encodeResult.out_len] = '\0';
  // Serial.println(encodeResult.out_len + 1);
  // delay(500);
  Serial.write((uint8_t *)&buff_cobs, encodeResult.out_len + 1);
  // delay(2);
  // cobs_in.flush();
  Serial.flush();
  // cobs_out.end();

  if (!write_success) {
    digitalWrite(3, HIGH);
    // sprintf(print_buf, "Write Failed.\n\r");
    // Serial.print(print_buf);
    Serial.println("write failed");
    delay(1);
  } else {
    digitalWrite(3, LOW);
  }
}

void receiveMessage() {
  digitalWrite(DE, RECEIVE);
  digitalWrite(RE, RECEIVE);

  Command command_tmp = Command_init_zero;
  char read_buff[100];
  unsigned long startTime = micros();

  unsigned int n = 0;
  bool done = false;
  while (Serial.available() == 0 && micros() - startTime < MAX_READ_TIME) {
  }
  while (!done && micros() - startTime < MAX_READ_TIME) {
    while (!done && Serial.available() > 0) {
      if (micros() - startTime >= MAX_READ_TIME)
        break;
      byte byte = Serial.read();
      // printf("waiting for byte\n");
      read_buff[n] = byte;
      // printf("received a byte: %02X\n", byte);
      n++;
      if (byte == '\0' || n >= 100) {
        done = true;
        break;
      }
    }
    if (done)
      break;
    while (Serial.available() == 0 && micros() - startTime < MAX_READ_TIME) {
    }
  }

  if (n == 0) return;

  // cobs_in.next();
  cobs_decode_result result =
      cobs_decode(&decode_buff, sizeof(decode_buff), &read_buff, n - 1);

  if (result.status != COBS_DECODE_OK) {
    Serial.println("cobs decode failed :(");
    delay(5000);
    return;
    // sprintf(print_buf, "cobs fail status: %\n\r", result.status);
    // Serial.println("nuh uh");
    // Serial.print(n);
    // Serial.println(" bytes received?");
    //   print_buff[0] = '\0';
    // for (unsigned int i = 0; i < n; i++) {
    //   sprintf(print_buff + strlen(print_buff), "%02X:", read_buff[i]);
    // }
    // sprintf(print_buff + strlen(print_buff), "\n");
    // Serial.print(print_buff);
  } else {
    // Serial.println("nuh uh");
    // Serial.print(n);
    // Serial.println(" bytes received?");
    // print_buff[0] = '\0';
    // for (unsigned int i = 0; i < n; i++) {
    //   sprintf(print_buff + strlen(print_buff), "%02X:", read_buff[i]);
    // }
    // sprintf(print_buff + strlen(print_buff), "\n");
    // Serial.print(print_buff);
    // if (n > 0) {
    //   Serial.println(n);
    //   Serial.println(" bytes received");
    // }
    /*sprintf(print_buf, "%d bytes received: ", n);
    Serial.print(print_buf);

    for (int j = 0; j < result.out_len; j++) {
        sprintf(print_buf, "%02X", decode_buff[j]);
        Serial.print(print_buf);
    }

    sprintf(print_buf, "\n\r");
    Serial.print(print_buf);*/
  }

  pb_istream_t istream =
      pb_istream_from_buffer((uint8_t *)decode_buff, (size_t)result.out_len);
  bool success = pb_decode(&istream, Command_fields, &command_tmp);
  if (success) {
    /*sprintf(print_buf, "D1: %d, D2: %d\n\r", command_tmp.out_1,
    command_tmp.out_2); Serial.print(print_buf);*/
    command = command_tmp;
    // Serial.println("success!");
    if (command.has_reset && command.reset) {
      Serial.println("resetting!");
      digitalWrite(3, HIGH);
      trySetupOTOS(false);
      delay(1000);
      digitalWrite(3, LOW);
    }

    if (command.has_calibrate && command.calibrate) {
      Serial.println("calibrating!");
      digitalWrite(3, HIGH);
      trySetupOTOS(true);
      delay(1000);
      digitalWrite(3, LOW);
    }

    // if (command.has_scalar) {
    //   Serial.println("set scalar!");
    //   otos.setAngularScalar(command.scalar.angular);
    //   otos.setLinearScalar(command.scalar.linear);
    //   delay(1000);
    // }

    // if (command.has_offset) {
    //   Serial.println("set offset!");
    //   sfe_otos_pose2d_t pose;
    //   pose.x = command.offset.x;
    //   pose.y = command.offset.y;
    //   pose.h = command.offset.heading;
    //   otos.setOffset(pose);
    //   delay(1000);
    // }

    // if (command.has_set_pose) {
    //   Serial.println("set pose!");
    //   sfe_otos_pose2d_t pose;
    //   pose.x = command.set_pose.x;
    //   pose.y = command.set_pose.y;
    //   pose.h = command.set_pose.heading;
    //   otos.setPosition(pose);
    //   delay(1000);
    // }
  } else {
    // sprintf(print_buf, "Decoding failed: %s\n\r", PB_GET_ERROR(&istream));
    // Serial.print(print_buf);
    // delay(1);
  }
}

void loop() {
  Pose pose = Pose_init_zero;
  // status.has_heading = true;
  // status.heading = 69;

  // Serial.println(otosInitialized);
  if (otosInitialized) {
    auto result = updateOTOS();
    pose = result.pose;
    if (result.code != 0) {
      if (millis() - sparkfunTimeout > 500) {
        recoverOTOS();
      }
    } else {
      sendMessage(pose);
      sparkfunTimeout = millis();
    }
  } else {
    Serial.println("trying to recover!");
    otosInitialized = trySetupOTOS(!hasCalibrated);
    if (otosInitialized && !hasCalibrated) {
      hasCalibrated = true;
      sparkfunTimeout = millis();
    }
  }
        // std::byte buf[255];
        // log_msg("waiting for message\n");
  // Serial.println(status.has_pos);
  // Serial.println(status.pos.x);
  // Serial.println(status.pos.y);
  // Serial.println(status.heading);
  // Serial.print("\n\n");

  receiveMessage();
  delay(1);
}