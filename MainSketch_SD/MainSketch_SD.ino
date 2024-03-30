#include <Arduino.h>
#include "OPC.h"

// Define the SS pin for the OPC sensor
const uint8_t ssPin_OPC = 10; // SS pin for OPC
const uint8_t ssPin_SD = 4; // SS pin for SD

// Create an instance of the OPC class
OPC opc(ssPin_OPC, ssPin_SD);

void setup() {
  opc.begin();
}

void loop() {
  opc.loop();
}

// 3/13 added SDcard write_file function in the end of loop
// shound write SPI_in raw data into SD card
// TODO: test sd card, create array to store actual data

// 3/14 tested and read SD card 
//result: data not right, SD wrote once only
//TODO: change when to turn on and off SD ss pin
