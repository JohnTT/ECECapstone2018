/*********************************************************************
  This is an example for our nRF51822 based Bluefruit LE modules

  Pick one up today in the adafruit shop!

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  MIT license, check LICENSE for more information
  All text above, and the splash screen below must be included in
  any redistribution
*********************************************************************/

/*
  This example shows how to send HID (keyboard/mouse/etc) data via BLE
  Note that not all devices support BLE Mouse!
  - OSX, Windows 10 both work
  - Android has limited support
  - iOS completely ignores mouse
*/
#include <Wire.h>
#include <Arduino.h>
#include <SPI.h>
#if not defined (_VARIANT_ARDUINO_DUE_X_) && not defined(ARDUINO_ARCH_SAMD)
#include <SoftwareSerial.h>
#endif

#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"

/*=========================================================================
    APPLICATION SETTINGS

      FACTORYRESET_ENABLE       Perform a factory reset when running this sketch
     
                                Enabling this will put your Bluefruit LE module
                              in a 'known good' state and clear any config
                              data set in previous sketches or projects, so
                                running this at least once is a good idea.
     
                                When deploying your project, however, you will
                              want to disable factory reset by setting this
                              value to 0.  If you are making changes to your
                                Bluefruit LE device via AT commands, and those
                              changes aren't persisting across resets, this
                              is the reason why.  Factory reset will erase
                              the non-volatile memory where config data is
                              stored, setting it back to factory default
                              values.
         
                                Some sketches that require you to bond to a
                              central device (HID mouse, keyboard, etc.)
                              won't work at all with this feature enabled
                              since the factory reset will clear all of the
                              bonding data stored on the chip, meaning the
                              central device won't be able to reconnect.
    MINIMUM_FIRMWARE_VERSION  Minimum firmware version to have some new features
    -----------------------------------------------------------------------*/
#define FACTORYRESET_ENABLE         0
#define MINIMUM_FIRMWARE_VERSION    "0.6.6"
/*=========================================================================*/

const int MPU_addr = 0x68; // I2C address of the MPU-6050
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;


// Create the bluefruit object, either software serial...uncomment these lines
/*
  SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);

  Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN,
                      BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);
*/

/* ...or hardware serial, which does not need the RTS/CTS pins. Uncomment this line */
// Adafruit_BluefruitLE_UART ble(BLUEFRUIT_HWSERIAL_NAME, BLUEFRUIT_UART_MODE_PIN);

/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

/* ...software SPI, using SCK/MOSI/MISO user-defined SPI pins and then user selected CS/IRQ/RST */
//Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_SCK, BLUEFRUIT_SPI_MISO,
//                             BLUEFRUIT_SPI_MOSI, BLUEFRUIT_SPI_CS,
//                             BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

/**************************************************************************/
/*!
    @brief  Sets up the HW an the BLE module (this function is called
            automatically on startup)
*/
/**************************************************************************/


void setup(void)
{
  while (!Serial);  // required for Flora & Micro
  delay(500);


  // i2c init for gy521

  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Serial.begin(9600);

  Serial.begin(115200);
  Serial.println(F("Adafruit Bluefruit HID Mouse Example"));
  Serial.println(F("---------------------------------------"));

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ) {
      error(F("Couldn't factory reset"));
    }
  }

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  // This demo only available for firmware from 0.6.6
  if ( !ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    error(F("This sketch requires firmware version " MINIMUM_FIRMWARE_VERSION " or higher!"));
  }

  /* Enable HID Service (including Mouse) */
  Serial.println(F("Enable HID Service (including Mouse): "));
  if (! ble.sendCommandCheckOK(F( "AT+BleHIDEn=On"  ))) {
    error(F("Failed to enable HID (firmware >=0.6.6?)"));
  }

  /* Add or remove service requires a reset */
  Serial.println(F("Performing a SW reset (service changes require a reset): "));
  if (! ble.reset() ) {
    error(F("Could not reset??"));
  }

  Serial.println();
  Serial.println(F("Go to your phone's Bluetooth settings to pair your device"));
  Serial.println(F("then open an application that accepts mouse input"));
  Serial.println();

  Serial.println(F("The example will try to draw a rectangle using the left mouse button with your input"));
  Serial.println(F("Parameters are a pair of 8-bit signed numbers (x,y) e.g:"));
  Serial.println(F("  100,100  : draw toward bottom right corner"));
  Serial.println(F("  -100,-100: draw toward top left corner"));

  Serial.println();
}

/**************************************************************************/
/*!
    @brief  Constantly poll for new command or response data
*/
/**************************************************************************/
void loop(void)
{
  char input[BUFSIZE + 1], cmd;



  // Read i2c data from gy521
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14, true); // request a total of 14 registers
  AcX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  Serial.print("AcX = "); Serial.print(AcX);
  Serial.print(" | AcY = "); Serial.print(AcY);
  Serial.print(" | AcZ = "); Serial.print(AcZ);
  Serial.print(" | Tmp = "); Serial.print(Tmp / 340.00 + 36.53); //equation for temperature in degrees C from datasheet
  Serial.print(" | GyX = "); Serial.print(GyX);
  Serial.print(" | GyY = "); Serial.print(GyY);
  Serial.print(" | GyZ = "); Serial.println(GyZ);


  //  Serial.println(F("Input Command (U - Up, D - Down, L - Left, R - Right, A - LClick, B - RClick, S - Scroll)"));
  //  getUserInput(input, BUFSIZE);
  //  cmd = input[0];
  //  Serial.println(cmd);
  //
  //  if (cmd == 'A' || cmd == 'B') {
  //    clickCommand(cmd);
  //  }
  //  else if (cmd == 'S') {
  //    scrollCommand();
  //  }
  //  else

  moveCommand(AcX);
  //delay(50);

  moveCommandup(AcY);
  delay(50);
//moveCommand(AcZ);
  //delay(50);

  
}

/**************************************************************************/
/*!
    @brief  Checks for user input (via the Serial Monitor)
*/
/**************************************************************************/
void moveCommand(int mag)
{
  // Right = -ve
  String mouseMove = String("AT+BleHidMouseMove=");
  String bleCmd = mouseMove + String(-mag / 1000) + String(",") + String(0);
  ble.println(bleCmd);
  delay(2);
  if (ble.waitForOK() == 0)
    ble.println(bleCmd);
}
void moveCommandup(int mag)
{
  String mouseMove = String("AT+BleHidMouseMove=");
  String bleCmd = mouseMove + String(0) + String(",") + String(-mag / 1000) ;
  ble.println(bleCmd);
  delay(2);
  if (ble.waitForOK() == 0)
    ble.println(bleCmd);
}
//
//  ble.println(bleCmd);
//  if ('U' == dir || 'D' == dir)
//  {
//    String mouseMove = String("AT+BleHidMouseMove=");
//    String bleCmd;
//
//    while (j > y) {
//      j += spd;
//      if (dir == 'U')
//        bleCmd = mouseMove + String(0) + String(",") + String(spd);
//      else if (dir == 'D')
//        bleCmd = mouseMove + String(0) + String(",") + String(-1 * spd);
//      ble.println(bleCmd);
//
//      // Check response status
//      delay(2);
//      if (ble.waitForOK() == 0)
//        ble.println(bleCmd);
//    }
//  }
//  else if ('L' == dir || 'R' == dir)
//  {
//    x = -mag;
//    spd = -spd;
//    // Send command
//    int j = 0;
//    String mouseMove = String("AT+BleHidMouseMove=");
//    String bleCmd;
//
//    while (j > x) {
//      j += spd;
//      if (dir == 'L')
//        bleCmd = mouseMove + String(spd) + String(",") + String(0);
//      else if (dir == 'R')
//
//
//        // Check response status
//
//      }
//  }


void scrollCommand()
{
  String scrollCmd = String("AT+BleHidMouseMove=,,");
  char input[BUFSIZE + 1];
  int mag, spd;

  Serial.println(F("Input Magnitude"));
  getUserInput(input, BUFSIZE);
  mag = atoi(input);

  Serial.println(F("Input Move Speed"));
  getUserInput(input, BUFSIZE);
  spd = atoi(input);


  int i = 0;
  int isMagNegative = 0;
  if (mag < 0)
  {
    isMagNegative = 1;
    mag = -mag;
  }
  String bleCmd;
  while (i < mag)
  {
    i += spd;

    if (isMagNegative)
      bleCmd = scrollCmd + String(spd);
    else
      bleCmd = scrollCmd + String(-1 * spd);

    // Positive mag is scrolling up
    ble.println(bleCmd);
    delay(2);
    if (ble.waitForOK() == 0)
      ble.println(bleCmd);
  }
}

void clickCommand(char button)
{
  String clickCmd = String("AT+BLEHIDMOUSEBUTTON=");

  if (button == 'A')
    button = 'L';
  else if (button == 'B')
    button = 'R';

  String bleCmd = clickCmd + String(button) + String(',') + String("CLICK");
  ble.println(bleCmd);

}


void getUserInput(char buffer[], uint8_t maxSize)
{
  memset(buffer, 0, maxSize);
  while ( Serial.available() == 0 ) {
    delay(1);
  }

  uint8_t count = 0;

  do
  {
    count += Serial.readBytes(buffer + count, maxSize);
    delay(2);
  } while ( (count < maxSize) && !(Serial.available() == 0) );
}
