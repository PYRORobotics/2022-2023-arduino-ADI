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


/**
 * NAVX COMMS IMPORTS/DEFINES
 */
#include "SPI.h"
#include "navx/AHRSProtocol.h"

#define SS_PIN 10
int ss = SS_PIN; //temporary TODO: Remove this and replace with define only

uint8_t data[512];

#define ITERATION_DELAY_MS                   10
#define NAVX_SENSOR_DEVICE_I2C_ADDRESS_7BIT  0x32
#define NUM_BYTES_TO_READ                    8




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

    pinMode(ss,OUTPUT);
    digitalWrite(SS, HIGH);
    SPI.begin();
    SPI.setBitOrder(MSBFIRST);
    SPI.setDataMode(SPI_MODE3);
    SPI.setClockDivider(8); /* 16Mhz/32 = 500kHz; /16=1Mhz; /8=2Mhz */
    for ( int i = 0; i < sizeof(data); i++ ) {
        data[i] = 0;
    }

    delay(100);
    uint8_t spi_data[3];
    /* Transmit SPI data */
    spi_data[0] = NAVX_REG_UPDATE_RATE_HZ | 0x80;                     // Start register address (high bit clear == read)
    spi_data[1] = 200;                  // setting refresh Hz to 200
    spi_data[2] = IMURegisters::getCRC(spi_data,2);   // Calculate CRC

    digitalWrite(SS, LOW);
    for ( int spi_data_index = 0; spi_data_index < 3; spi_data_index++ ) {
        SPI.transfer(spi_data[spi_data_index]);
    }
    digitalWrite(SS, HIGH);
    delay(100);
}
//int32_t i = 5;

void loop() {

    int32_t yaw = -1;

    //get navx data///////////////////////////////
    uint8_t spi_crc;
    uint8_t spi_data[3];

    /* Transmit SPI data */
    spi_data[0] = NAVX_REG_YAW_L;                     // Start register address (high bit clear == read)
    spi_data[1] = NUM_BYTES_TO_READ;                  // Number of bytes to read (not including the final CRC)
    spi_data[2] = IMURegisters::getCRC(spi_data, 2);   // Calculate CRC
    //Serial.print("SPI:  ");
    digitalWrite(SS, LOW);
    for (int spi_data_index = 0; spi_data_index < 3; spi_data_index++) {
        SPI.transfer(spi_data[spi_data_index]);
    }
    digitalWrite(SS, HIGH);

    /* Wait 200 microseconds, ensuring sensor is ready to reply */
    delayMicroseconds(200); // Read 0xFFs until ready

    /* Read requested data */
    /* NOTE:  One more byte than requested is */
    /* sent.  This is a Checksum. */
    digitalWrite(SS, LOW);
    for (int x = 0; x < (NUM_BYTES_TO_READ + 1); x++) {
        data[x] = SPI.transfer((byte) 0xFF);
    }
    digitalWrite(SS, HIGH);


    /* Verify CRC and display data */
    spi_crc = IMURegisters::getCRC(data, NUM_BYTES_TO_READ);
    if (spi_crc != data[NUM_BYTES_TO_READ]) {
        //Serial.print("SPI CRC ERROR!  ");
        //Serial.println(spi_crc);
    }
    else {
        /* Decode received data to floating-point orientation values */
        //float yaw =     IMURegisters::decodeProtocolSignedHundredthsFloat((char *)&data[0]);   // The cast is needed on arduino
        //float pitch =   IMURegisters::decodeProtocolSignedHundredthsFloat((char *)&data[2]);   // The cast is needed on arduino
        //float roll =    IMURegisters::decodeProtocolSignedHundredthsFloat((char *)&data[4]);   // The cast is needed on arduino
        yaw = IMURegisters::decodeProtocolInt16((char *) &data[0]); // The cast is needed on arduino
    }




    //send data////////////////////////////////////
    //sprintf(print_buf, "empty send buf size: %d\n\r", serial_free);
    //Serial.print(print_buf);
    digitalWrite(DIRECTION_PIN, HIGH);
    delayMicroseconds(1000);
    Status status = Status_init_zero;
    status.in_1 = false;
    status.in_2 = false;
    status.has_counter = true;
    status.counter = (yaw / 1); //TODO: rename "counter" to "heading" or "navx_heading"
    //i++;
    char encode_buff[255];
    pb_ostream_s pb_out = pb_ostream_from_buffer((uint8_t *) &encode_buff, 255);

    bool success = pb_encode(&pb_out, Status_fields, &status);


    if(!success){
        digitalWrite(3, HIGH);
        sprintf(print_buf, "Encoding failed: %s\n\r", PB_GET_ERROR(&pb_out));
        //Serial.print(print_buf);
        delay(5);
    }
    else{
        digitalWrite(3, LOW);
    }
    bool write_success = true;

    char buff_cobs[30];
    cobs_encode_result encodeResult = cobs_encode_mine(&buff_cobs, 30, &encode_buff, pb_out.bytes_written);
    if(encodeResult.status != COBS_ENCODE_OK){
        //printf("COBS encode failed!\n");
        //printf("COBS status: %d\n", result.status);
        digitalWrite(3, HIGH);
        delay(500);
    }
    buff_cobs[encodeResult.out_len] = '\0';
    Serial.write((uint8_t*)&buff_cobs, encodeResult.out_len+1);

    /*for(int j = 0; j < pb_out.bytes_written; j++){
        if(!cobs_out.write(encode_buff[j])){
            write_success = false;
        }
    }*/
//    printf("sent %d COBS-decoded bytes: ", pb_out.bytes_written);
//    for(int j = 0; j < result.out_len; j++){
//        if (j > 0) printf(":");
//        printf("%02X", buf_decoded[j]);
//    }

    /*if(!cobs_out.write('\0')){
        write_success = false;
    }*/
    cobs_out.end();

    if(!write_success){
        digitalWrite(3, HIGH);
        sprintf(print_buf, "Write Failed.\n\r");
        //Serial.print(print_buf);
        delay(1);
    }
    else{
        digitalWrite(3, LOW);
    }
    //while (){}

    while(Serial.availableForWrite() < 63){}
    //delayMicroseconds(200); //TODO: this should instead be a wait until the tx buffer is empty
    //delayMicroseconds(500); //TODO: this should instead be a wait until the tx buffer is empty
    delayMicroseconds(2000); //TODO: this should instead be a wait until the tx buffer is empty
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
        //Serial.print(print_buf);
    }
    else {
        /*sprintf(print_buf, "%d bytes received: ", n);
        Serial.print(print_buf);

        for (int j = 0; j < result.out_len; j++) {
            sprintf(print_buf, "%02X", decode_buff[j]);
            Serial.print(print_buf);
        }

        sprintf(print_buf, "\n\r");
        Serial.print(print_buf);*/
    }

    pb_istream_t istream = pb_istream_from_buffer((uint8_t *) decode_buff, (size_t) result.out_len);
    success = pb_decode(&istream, Command_fields, &command_tmp);
    if(success) {
        /*sprintf(print_buf, "D1: %d, D2: %d\n\r", command_tmp.out_1, command_tmp.out_2);
        Serial.print(print_buf);*/
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