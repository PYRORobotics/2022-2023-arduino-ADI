//
// Created by charles on 9/13/22.
//
#include "Arduino.h"
#include "proto/messages.pb.h"
#include <pb_arduino.h>
#include <cobs/Stream.h>
#include <cobs/Print.h>
#include "cobs/cobs.h"

#define DIRECTION_PIN 9
#define SEND HIGH
#define RECEIVE LOW


packetio::COBSStream cobs_in(Serial);
//pb_istream_s pb_in = as_pb_istream(cobs_in);

packetio::COBSPrint cobs_out(Serial);
//pb_ostream_s pb_out = as_pb_ostream(cobs_out);
//pb_ostream_s pb_out = as_pb_ostream(Serial);

Command command = Command_init_zero;
char print_buf[64];
int serial_free = 0;
void setup() {
    Serial.begin(115200);
    pinMode(DIRECTION_PIN, OUTPUT);
    pinMode(3, OUTPUT);
    pinMode(6, OUTPUT);
    pinMode(7, OUTPUT);
    serial_free = Serial.availableForWrite();
}
int32_t i = 5;

void loop() {
    //sprintf(print_buf, "empty send buf size: %d\n\r", serial_free);
    //Serial.print(print_buf);

    digitalWrite(DIRECTION_PIN, HIGH);
    delayMicroseconds(1000);
    Status status = Status_init_zero;
    status.has_counter = true;
    status.counter = i;
    i++;
    char encode_buff[255];
    pb_ostream_s pb_out = pb_ostream_from_buffer((uint8_t *) &encode_buff, 255);

    bool success = pb_encode(&pb_out, Status_fields, &status);
    if(!success){
        digitalWrite(3, HIGH);
        sprintf(print_buf, "Encoding failed: %s\n\r", PB_GET_ERROR(&pb_out));
        Serial.print(print_buf);
        delay(1);
    }
    else{
        digitalWrite(3, LOW);
    }
    bool write_success = true;

    for(int j = 0; j < pb_out.bytes_written; j++){
        if(!cobs_out.write(encode_buff[j])){
            write_success = false;
        }
    }
    /*if(!cobs_out.write('\0')){
        write_success = false;
    }*/
    cobs_out.end();

    if(!write_success){
        digitalWrite(3, HIGH);
        sprintf(print_buf, "Write Failed.\n\r");
        Serial.print(print_buf);
        delay(1);
    }
    else{
        digitalWrite(3, LOW);
    }
    //while (){}

    while(Serial.availableForWrite() < 63){}
    //delayMicroseconds(200); //TODO: this should instead be a wait until the tx buffer is empty
    delayMicroseconds(500); //TODO: this should instead be a wait until the tx buffer is empty
    digitalWrite(DIRECTION_PIN, LOW);

    Command command_tmp = Command_init_zero;
    cobs_in.flush();
    char read_buff[100];
    unsigned long startTime = millis();

    while(millis() - startTime < 100 && Serial.peek() == -1){

    }

    int n = 0;
    while(millis() - startTime < 100 && n < sizeof(read_buff)){
        // read until we get something
        //int c = cobs_in.read();
        int c  = Serial.read();
        //if(c == packetio::COBSStream::EOF) {
        if(c == -1) {
            continue;
        }

        // detect End Of Packet
        //if(c == packetio::COBSStream::EOP && n>0) break;
        if(c == '\0' && n>0) break;

        // save anything else
        read_buff[n++] = c;
    }
    //cobs_in.next();
    char decode_buff[100];
    cobs_decode_result result = cobs_decode(&decode_buff, sizeof(decode_buff), &read_buff, n);

    if(result.status != COBS_DECODE_OK){
        sprintf(print_buf, "cobs fail status: %\n\r", result.status);
        Serial.print(print_buf);
    }
    else {
        sprintf(print_buf, "%d bytes received: ", n);
        Serial.print(print_buf);

        for (int j = 0; j < result.out_len; j++) {
            sprintf(print_buf, "%02X", decode_buff[j]);
            Serial.print(print_buf);
        }

        sprintf(print_buf, "\n\r");
        Serial.print(print_buf);
    }

    pb_istream_t istream = pb_istream_from_buffer((uint8_t *) decode_buff, (size_t) result.out_len);
    success = pb_decode(&istream, Command_fields, &command_tmp);
    if(success) {
        sprintf(print_buf, "D1: %d, D2: %d\n\r", command_tmp.out_1, command_tmp.out_2);
        Serial.print(print_buf);
        command = command_tmp;
        digitalWrite(3, LOW);
    }
    else{
        digitalWrite(3, HIGH);
        sprintf(print_buf, "Decoding failed: %s\n\r", PB_GET_ERROR(&istream));
        Serial.print(print_buf);
        delay(1);
    }

    digitalWrite(6, command.out_1);
    digitalWrite(7, command.out_2);

    //delay(5);
}