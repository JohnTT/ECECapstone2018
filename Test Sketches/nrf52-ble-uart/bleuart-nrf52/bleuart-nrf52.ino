/*********************************************************************
  This is an example for our nRF52 based Bluefruit LE modules

  Pick one up today in the adafruit shop!

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  MIT license, check LICENSE for more information
  All text above, and the splash screen below must be included in
  any redistribution
*********************************************************************/
#include <bluefruit.h>

// For BNO0555
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 bno = Adafruit_BNO055();
SoftwareTimer integrationTimer;

// BLE Service
BLEDis  bledis;
BLEUart bleuart;
BLEBas  blebas;

// Software Timer for blinking RED LED
SoftwareTimer blinkTimer;

void setup()
{
  Serial.begin(9600);

  // BNO055
  //
  Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");

  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  delay(1000);

  /* Display the current temperature */
  int8_t temp = bno.getTemp();
  Serial.print("Current Temperature: ");
  Serial.print(temp);
  Serial.println(" C");
  Serial.println("");

  bno.setExtCrystalUse(true);

  Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");

  //
  // Start Integral Calcuation
  //
  //integrationTimer.begin(100, integration_timer_callback);
  //integrationTimer.start();


  //
  // Bluetooth BLEUart
  //
  Serial.println("Bluefruit52 BLEUART Example");
  Serial.println("---------------------------\n");

  //  // Initialize blinkTimer for 1000 ms and start it
  //  blinkTimer.begin(1000, blink_timer_callback);
  //  blinkTimer.start();

  // Setup the BLE LED to be enabled on CONNECT
  // Note: This is actually the default behaviour, but provided
  // here in case you want to control this LED manually via PIN 19
  Bluefruit.autoConnLed(true);

  Bluefruit.begin();
  // Set max power. Accepted values are: -40, -30, -20, -16, -12, -8, -4, 0, 4
  Bluefruit.setTxPower(4);
  Bluefruit.setName("Bluefruit52");
  //Bluefruit.setName(getMcuUniqueID()); // useful testing with multiple central connections
  Bluefruit.setConnectCallback(connect_callback);
  Bluefruit.setDisconnectCallback(disconnect_callback);

  // Configure and Start Device Information Service
  bledis.setManufacturer("Adafruit Industries");
  bledis.setModel("Bluefruit Feather52");
  bledis.begin();

  // Configure and Start BLE Uart Service
  bleuart.begin();

  // Start BLE Battery Service
  blebas.begin();
  blebas.write(100);

  // Set up and start advertising
  startAdv();

  Serial.println("Please use Adafruit's Bluefruit LE app to connect in UART mode");
  Serial.println("Once connected, enter character(s) that you wish to send");
}

void startAdv(void)
{
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  // Include bleuart 128-bit uuid
  Bluefruit.Advertising.addService(bleuart);

  // Secondary Scan Response packet (optional)
  // Since there is no room for 'Name' in Advertising packet
  Bluefruit.ScanResponse.addName();

  /* Start Advertising
     - Enable auto advertising if disconnected
     - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
     - Timeout for fast mode is 30 seconds
     - Start(timeout) with timeout = 0 will advertise forever (until connected)

     For recommended advertising interval
     https://developer.apple.com/library/content/qa/qa1931/_index.html
  */
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds
}

int sendBLEUartFlag() {
  return 1;

  int sensorValue = analogRead(A0);
  double sensorVoltage = 3.3 * (sensorValue / 1023.0);
  double straightVoltage = 1.7;
  double bentVoltage = 0.7;
  Serial.println(sensorVoltage);

  if (sensorVoltage < bentVoltage + 0.3)
    return 1;
  else
    return 0;
}

// integration variables
double accelX_now = 0;
double accelX_prev = 0;
double velX_now = 0;
double velX_prev = 0;

double accelY_now = 0;
double accelY_prev = 0;
double velY_now = 0;
double velY_prev = 0;

double accelZ_now = 0;
double accelZ_prev = 0;
double velZ_now = 0;
double velZ_prev = 0;

double posX = 0;
double posY = 0;
double posZ = 0;

imu::Vector<3> euler;
imu::Vector<3> accel;
imu::Quaternion quat;

void loop()
{
  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2

  // String buf
  char buf[64];

  while (sendBLEUartFlag()) {
    //itoa(counter,buf,10);
    //euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    //accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    quat = bno.getQuat();
    euler = quat.toEuler();


    // intCalc();

    // posX
    sprintf(buf, "posX=%d", posX);
    bleuart.write( buf, strlen(buf) );
    //Serial.println(buf);

    // posY
    sprintf(buf, "posY=%d", posY);
    bleuart.write( buf, strlen(buf) );
    //Serial.println(buf);

    // posZ
    sprintf(buf, "posZ=%d", posZ);
    bleuart.write( buf, strlen(buf) );
    //Serial.println(buf);
    /* Display the floating point data */
    Serial.print("eulerX: ");
    Serial.print((int)(euler.x()*180/3.14159));
    Serial.print(" eulerY: ");
    Serial.print((int)(euler.y()*180/3.14159));
    Serial.print(" eulerZ: ");
    Serial.print((int)(euler.z()*180/3.14159));
    Serial.println("\t\t");

//    Serial.print("accelX: ");
//    Serial.print(accel.x());
//    Serial.print(" accel: ");
//    Serial.print(accel.y());
//    Serial.print(" accelZ: ");
//    Serial.print(accel.z());
//    Serial.print("\t\t");
//
//    Serial.print("velX: ");
//    sprintf(buf, "%.6f", velX_now);
//    Serial.print(buf);
//
//    Serial.print(" velY: ");
//    sprintf(buf, "%.6f", velY_now);
//    Serial.print(buf);
//
//    Serial.print(" velZ: ");
//    sprintf(buf, "%.6f", velZ_now);
//    Serial.print(buf);
//    Serial.print("\t\t");
//
//
//    Serial.print("posX: ");
//    sprintf(buf, "%.6f", posX);
//    Serial.print(buf);
//
//    Serial.print(" posY: ");
//    sprintf(buf, "%.6f", posY);
//    Serial.print(buf);
//
//    Serial.print(" posZ: ");
//    sprintf(buf, "%.6f", posZ);
//    Serial.print(buf);
//    Serial.println("\t\t");


    // X
    sprintf(buf, "eulerX=%d", (int)(euler.x()*180/3.14159));
    bleuart.write( buf, strlen(buf) );
    //Serial.println(buf);

    // Y
    sprintf(buf, "eulerY=%d", (int)(euler.y()*180/3.14159));
    bleuart.write( buf, strlen(buf) );
    //Serial.println(buf);

    // Z
    sprintf(buf, "eulerZ=%d", (int)(euler.z()*180/3.14159));
    bleuart.write( buf, strlen(buf) );
    //Serial.println(buf);


    // posX
    sprintf(buf, "posX=%d", (int)posX);
    bleuart.write( buf, strlen(buf) );
    //Serial.println(buf);

    // posY
    sprintf(buf, "posY=%d", (int)posY);
    bleuart.write( buf, strlen(buf) );
    //Serial.println(buf);

    // posZ
    sprintf(buf, "posZ=%d", (int)posZ);
    bleuart.write( buf, strlen(buf) );
    //Serial.println(buf);


    // delay(10);
  }

  // Request CPU to enter low-power mode until an event/interrupt occurs
  //waitForEvent();
}

void connect_callback(uint16_t conn_handle)
{
  char central_name[32] = { 0 };
  Bluefruit.Gap.getPeerName(conn_handle, central_name, sizeof(central_name));

  Serial.print("Connected to ");
  Serial.println(central_name);
}

void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;

  Serial.println();
  Serial.println("Disconnected");
}

/**
   Software Timer callback is invoked via a built-in FreeRTOS thread with
   minimal stack size. Therefore it should be as simple as possible. If
   a periodically heavy task is needed, please use Scheduler.startLoop() to
   create a dedicated task for it.

   More information http://www.freertos.org/RTOS-software-timer.html
*/
/**
   RTOS Idle callback is automatically invoked by FreeRTOS
   when there are no active threads. E.g when loop() calls delay() and
   there is no bluetooth or hw event. This is the ideal place to handle
   background data.

   NOTE: It is recommended to call waitForEvent() to put MCU into low-power mode
   at the end of this callback. You could also turn off other Peripherals such as
   Serial/PWM and turn them back on if wanted

   e.g

   void rtos_idle_callback(void)
   {
      Serial.stop(); // will lose data when sleeping
      waitForEvent();
      Serial.begin(115200);
   }

   NOTE2: If rtos_idle_callback() is not defined at all. Bluefruit will force
   waitForEvent() to save power. If you don't want MCU to sleep at all, define
   an rtos_idle_callback() with empty body !

   WARNING: This function MUST NOT call any blocking FreeRTOS API
   such as delay(), xSemaphoreTake() etc ... for more information
   http://www.freertos.org/a00016.html
*/
void rtos_idle_callback(void)
{
  // Don't call any other FreeRTOS blocking API()
  // Perform background task(s) here

  // Request CPU to enter low-power mode until an event/interrupt occurs
  waitForEvent();
}

void intCalc() {
  double dt = 0.01;

  // Sample Acceleration
  accelX_now = accel.x();
  accelY_now = accel.y();
  accelZ_now = accel.z();

  if (abs(accelX_now) < 0.2)
    accelX_now = 0;
  if (abs(accelY_now) < 0.2)
    accelY_now = 0;
  if (abs(accelZ_now) < 0.2)
    accelZ_now = 0;

  // Integrate once to get velocity
  velX_now = velX_prev + dt * (accelX_now - accelX_prev);
  velY_now = velY_prev + dt * (accelY_now - accelY_prev);
  velZ_now = velZ_prev + dt * (accelZ_now - accelZ_prev);

  // Integrate again to get position
  posX = posX + dt * (velX_now - velX_prev);
  posY = posY + dt * (velY_now - velY_prev);
  posZ = posZ + dt * (velZ_now - velZ_prev);

  // replace prev with now
  accelX_prev = accelX_now;
  accelY_prev = accelY_now;
  accelZ_prev = accelZ_now;

  velX_prev = velX_now;
  velY_prev = velY_now;
  velZ_prev = velZ_now;
}

void integration_timer_callback(TimerHandle_t xTimerID)
{
  (void) xTimerID;

  intCalc();

  char buf[64];
  // posX
  sprintf(buf, "posX=%d", posX);
  bleuart.write( buf, strlen(buf) );
  //Serial.println(buf);

  // posY
  sprintf(buf, "posY=%d", posY);
  bleuart.write( buf, strlen(buf) );
  //Serial.println(buf);

  // posZ
  sprintf(buf, "posZ=%d", posZ);
  bleuart.write( buf, strlen(buf) );
  //Serial.println(buf);
}

