//
// Created by charles on 9/13/22.
//
#include "Arduino.h"
#include "proto/messages.pb.h"
#include <pb_arduino.h>
#include <cobs/Stream.h>
#include <cobs/Print.h>

packetio::COBSStream cobs_in(Serial);
pb_istream_s pb_in = as_pb_istream(cobs_in);

packetio::COBSPrint cobs_out(Serial);
pb_ostream_s pb_out = as_pb_ostream(cobs_out);

void setup() {
    Serial.begin(115200);
    pinMode(2, OUTPUT);
}
int i = 0;
void loop() {
    Status status = Status_init_zero;
    status.counter = i;
    i++;
    bool success = pb_encode(&pb_out, Status_fields, &status);
    cobs_out.end();
    delay(1000);
}