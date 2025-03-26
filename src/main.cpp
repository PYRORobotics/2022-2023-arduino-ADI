//
// Created by charles on 9/13/22.
//
#include "Arduino.h"
#include "Wire.h"
#include "cobs/cobs.h"
#include "otos.hpp"
#include "proto/messages.pb.h"
#include <cobs/Print.h>
#include <cobs/Stream.h>
#include <pb_arduino.h>
#include <stdio.h>

#define DE 9
#define RE 8
#define SEND HIGH
#define RECEIVE LOW

uint8_t data[512];

#define ITERATION_DELAY_MS 10
#define NAVX_SENSOR_DEVICE_I2C_ADDRESS_7BIT 0x32
#define NUM_BYTES_TO_READ 8

packetio::COBSStream cobs_in(Serial);
// pb_istream_s pb_in = as_pb_istream(cobs_in);

packetio::COBSPrint cobs_out(Serial);
// pb_ostream_s pb_out = as_pb_ostream(cobs_out);
// pb_ostream_s pb_out = as_pb_ostream(Serial);

Command command = Command_init_zero;
bool otosInitialized = false;
void setup() {
  Serial.begin(115200);
  pinMode(DE, OUTPUT);
  pinMode(RE, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  Wire.begin();
  otosInitialized = trySetupOTOS();
  Serial.println("setup!");
}
// int32_t i = 5;
char encode_buff[30];
char decode_buff[30];
char buff_cobs[30];
char padding[5] = {0};
// char print_buff[20];
void readWrite(Status status) {
  // send data////////////////////////////////////
  // sprintf(print_buf, "empty send buf size: %d\n\r", serial_free);
  // Serial.print(print_buf);
  // while(Serial.available() > 0) {
  //   Serial.read();
  // }
  while (!Serial.availableForWrite()) {
    delayMicroseconds(10);
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
  bool success = pb_encode(&pb_out, Status_fields, &status);

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
  Serial.write((uint8_t *)&padding, 2);
  Serial.write((uint8_t *)&buff_cobs, encodeResult.out_len + 1);
  // delay(2);
  Serial.flush();
  // Serial.println(encodeResult.out_len);

  /*for(int j = 0; j < pb_out.bytes_written; j++){
      if(!cobs_out.write(encode_buff[j])){
          write_success = false;
      }
  }*/
  //  printf("sent %d COBS-encoded bytes: ", encodeResult.out_len);
  //  for(int j = 0; j < encodeResult.out_len; j++){
  //      if (j > 0) printf(":");
  //      printf("%02X", buff_cobs[j]);
  //  }
  //  printf("\n");
  // print_buff[0] = '\0';
  // for (unsigned int i = 0; i < encodeResult.out_len; i++) {
  //   sprintf(print_buff + strlen(print_buff), "%02X:", buff_cobs[i]);
  // }
  // sprintf(print_buff + strlen(print_buff), "\n");
  // Serial.print(print_buff);

  /*if(!cobs_out.write('\0')){
      write_success = false;
  }*/
  cobs_out.end();

  if (!write_success) {
    digitalWrite(3, HIGH);
    // sprintf(print_buf, "Write Failed.\n\r");
    // Serial.print(print_buf);
    Serial.println("write failed");
    delay(1);
  } else {
    digitalWrite(3, LOW);
  }
  // delayMicroseconds(200); //TODO: this should instead be a wait until the tx
  // buffer is empty delayMicroseconds(500); //TODO: this should instead be a
  // wait until the tx buffer is empty
  // Serial.flush();
  // delayMicroseconds(
  // 1000); // TODO: this should instead be a wait until the tx buffer is empty
  digitalWrite(DE, RECEIVE);
  digitalWrite(RE, RECEIVE);

  Command command_tmp = Command_init_zero;
  cobs_in.flush();
  char read_buff[100];
  unsigned long startTime = millis();

  while (!Serial.available() && millis() - startTime < 100) {
  }

  // int n = Serial.readBytesUntil('\0', read_buff, sizeof(read_buff));
  unsigned int n = 0;
  bool done = false;
  while (Serial.available()) {
    // read until we get something
    // int c = cobs_in.read();
    int c = Serial.read();
    if (done)
      continue;
    // if(c == packetio::COBSStream::EOF) {
    if (c == -1) {
      continue;
    }

    // detect End Of Packet
    // if(c == packetio::COBSStream::EOP && n>0) break;
    if (c == '\0') {
      if (n > 2) {
        done = true;
      }
      continue;
    }

    if (n >= 100) {
      done = true;
      continue;
    }

    // save anything else
    read_buff[n++] = c;
  }
  // cobs_in.next();
  cobs_decode_result result =
      cobs_decode(&decode_buff, sizeof(decode_buff), &read_buff, n);

  if (result.status != COBS_DECODE_OK) {
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
  success = pb_decode(&istream, Command_fields, &command_tmp);
  if (success) {
    /*sprintf(print_buf, "D1: %d, D2: %d\n\r", command_tmp.out_1,
    command_tmp.out_2); Serial.print(print_buf);*/
    command = command_tmp;
    // Serial.println("success!");
    if (command.reset) {
      Serial.println("resetting!");
      digitalWrite(3, HIGH);
      trySetupOTOS();
      delay(1000);
      digitalWrite(3, LOW);
    }
  } else {
    digitalWrite(3, HIGH);
    // Serial.println("womp womp");
    digitalWrite(3, LOW);
    // sprintf(print_buf, "Decoding failed: %s\n\r", PB_GET_ERROR(&istream));
    // Serial.print(print_buf);
    // delay(1);
  }
}

void loop() {
  Status status = Status_init_zero;
  // status.has_heading = true;
  // status.heading = 69;

  if (otosInitialized) {
    status = updateOTOS();
  } else {
    otosInitialized = trySetupOTOS();
  }

  // Serial.println(status.has_pos);
  // Serial.println(status.pos.x);
  // Serial.println(status.pos.y);
  // Serial.println(status.heading);
  // Serial.print("\n\n");

  readWrite(status);
  delay(20);
}