/*********************************************************************
This is an example for our nRF8001 Bluetooth Low Energy Breakout

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/1697

Adafruit invests time and resources providing this open source code, 
please support Adafruit and open-source hardware by purchasing 
products from Adafruit!

Written by Kevin Townsend/KTOWN  for Adafruit Industries.
MIT license, check LICENSE for more information
All text above, and the splash screen below must be included in any redistribution
*********************************************************************/

/*
 Modified by Carlos Hernandez
 San Jose State University Fall 2015
 Wristband Bluetooth Firmware
 Modified for Tinyduino
 */


// This version uses the internal data queing so you can treat it like Serial (kinda)!

#include <SPI.h>
#include <Wire.h>
#include "Adafruit_BLE_UART.h"

// Connect CLK/MISO/MOSI to hardware SPI
// e.g. On UNO & compatible: CLK = 13, MISO = 12, MOSI = 11
#define ADAFRUITBLE_REQ 10
#define ADAFRUITBLE_RDY 2     // This should be an interrupt pin, on Uno thats #2 or #3
#define ADAFRUITBLE_RST 9

//Data from phone
volatile byte phone_data[8] = {0};
volatile uint8_t phone_byte = 0;
volatile int data_counter = 0;

//Data from SAMG
volatile uint8_t samg_data[8] = {0};
volatile int element = 0;
volatile byte test_element = 0;

//Predefined Arrays for Alert Messages
uint8_t ble_seizure[14] = {'S', 'E', 'I', 'Z',
                           'U', 'R', 'E', ' ',
                           'A', 'L', 'E', 'R',
                           'T', '!'};
              
uint8_t ble_fall[11] = {'F', 'A', 'L', 'L', 
                        ' ', 'A', 'L', 'E',
                        'R', 'T', '!'};
            
uint8_t ble_unconscious[14] = {'C', 'R', 'I', 'T',
                               'I', 'C', 'A', 'L',
                               ' ', 'F', 'A', 'L', 
                               'L', '!'};
                                                  
uint8_t ble_eda[5] = {'E', 'D', 'A', 0, 0};



Adafruit_BLE_UART BTLEserial = Adafruit_BLE_UART(ADAFRUITBLE_REQ, ADAFRUITBLE_RDY, ADAFRUITBLE_RST);
/**************************************************************************/
/*!
    Configure the Arduino and start advertising with the radio
*/
/**************************************************************************/
void setup(void)
{ 
  pinMode(7, OUTPUT);           // set pin 7 to output for debug
  Wire.begin(8);                // join i2c bus with address #8
  Wire.onReceive(receiveEvent); // register event
  Wire.onRequest(requestEvent); // register event
  Serial.begin(9600);           // start serial for output
  while(!Serial); // Leonardo/Micro should wait for serial init
  Serial.println(F("Adafruit Bluefruit Low Energy nRF8001 Print echo demo"));

  BTLEserial.setDeviceName("Wrist"); /* 7 characters max! */

  BTLEserial.begin();
}

/**************************************************************************/
/*!
    Constantly checks for new events on the nRF8001
*/
/**************************************************************************/
aci_evt_opcode_t laststatus = ACI_EVT_DISCONNECTED;

void loop()
{
  delay(1000);
  // Tell the nRF8001 to do whatever it should be working on.
  BTLEserial.pollACI();

  // Ask what is our current status
  aci_evt_opcode_t status = BTLEserial.getState();
  // If the status changed....
  if (status != laststatus) {
    // print it out!
    if (status == ACI_EVT_DEVICE_STARTED) {
        Serial.println(F("* Advertising started"));
    }
    if (status == ACI_EVT_CONNECTED) {
        Serial.println(F("* Connected!"));
    }
    if (status == ACI_EVT_DISCONNECTED) {
        Serial.println(F("* Disconnected or advertising timed out"));
    }
    // OK set the last status change to this one
    laststatus = status;
  }

  if (status == ACI_EVT_CONNECTED) {
    // OK while we still have something to read, get a character
    while (BTLEserial.available()) {
      phone_data[phone_byte] = BTLEserial.read();
      phone_byte++;
    }
   phone_byte = 0;

//    uint8_t samg_data_bytes[1];
//    samg_data_bytes[0] = samg_data;
//    BTLEserial.write(samg_data_bytes, 1);

      //use dummy array to force data type to match BLE argument type (uint8_t)
      //also to ensure data is not overwritten by i2c ISR
      uint8_t samg_data_bytes[8];

      for(int counter = 0; counter < 8; counter++){

        samg_data_bytes[counter]= (uint8_t) samg_data[counter];
      }
      
      BTLEserial.write(samg_data_bytes, 8);
      
//    switch(samg_data) {
//
//      //seizure detected
//      case 1:
//        BTLEserial.write(ble_seizure, 14);
//        samg_data = 0;        
//        break;
//
//      //fall detected
//      case 2:
//        BTLEserial.write(ble_fall, 11);
//        samg_data = 0; 
//        break;
//
//      //critical fall detected  
//      case 3:
//        BTLEserial.write(ble_unconscious, 14);
//        samg_data = 0;
//      default:
//        break;
//        
//    }
  }
}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int howMany)
{
  element = 0;
  
  while(Wire.available()){
//    Serial.println(samg_data[element]);
    samg_data[element] = Wire.read();    // receive byte as an integer 
    element++;
  }
}

void requestEvent()
{

  //recast as byte so twi function accepts arguments
  byte twi_data[8] = {0x00};  
//  byte twi_data[8] = {'c', 'a', 'r', 'l', 'o', 's', 'a', 'h'};
//   byte twi_data[8] = {11, 22, 33, 0x00, 0xFF, 0x00, 0xFF, 0x00};
  for(int i = 0; i < 8; i++){
    twi_data[i] = phone_data[i];
    phone_data[i] = 0;
  }
//  byte test_data = 0xFF;
//  byte test_data;
//  test_data = test_element;
  Wire.write(twi_data, 8); //respond with 8 bytes
//  Serial.println(test_data);
//  test_element++;
//  if(test_element == 200){
//    test_element = 0;
//    Serial.println("1s elapsed");
//  }
//  phone_data = 0;

}

