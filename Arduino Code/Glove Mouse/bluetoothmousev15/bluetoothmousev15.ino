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
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_NeoPixel.h>

/*--------------------------------*/
// BNO055
#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 bno = Adafruit_BNO055(55);


// Neopixel
#define PIN 7
#define VBAT_PIN          (A7)

#define VBAT_MV_PER_LSB   (0.73242188F)   // 3.0V ADC range and 12-bit ADC resolution = 3000mV/4096
#define VBAT_DIVIDER      (0.71275837F)   // 2M + 0.806M voltage divider on VBAT = (2M / (0.806M + 2M))
#define VBAT_DIVIDER_COMP (1.403F)        // Compensation factor for the VBAT divider

// Other
#define RESOLUTION_X 1440
#define RESOLUTION_Y 2560
#define ANGLE_RANGE (PI/3.0)
/*-----------------------------------------------*/

Adafruit_NeoPixel strip = Adafruit_NeoPixel(60, PIN, NEO_GRB + NEO_KHZ800);
BLEDis bledis;
BLEHidAdafruit blehid;
BLEBas  blebas;

// Flex State Machine
#define NUM_FLEX 4
int flexADC[NUM_FLEX] = {};
float flexVolts[NUM_FLEX] = {};
const float flexThreshold[NUM_FLEX] = {2.58, 2.8, 2.85, 2.6};
bool flexState[NUM_FLEX] = {}; // Set All to Zero (Not Flexed)
// Based off Right Hand Glove
enum flexPinNames {
  PIN_PINKY = 0,
  PIN_RING = 1,
  PIN_MIDDLE = 2,
  PIN_INDEX = 3
};
enum flexStateNames {
  MODE_MOVING = B0000,
  MODE_LEFTCLICK = B0100,
  MODE_RIGHTCLICK = B0010,
  MODE_CALIBRATION = B1000,
  MODE_POINTING = B0111,
  MODE_SCROLLING = B1111
};
int currentState = 0;
int prevState = 0;

// Mouse movement variables (Angles in Radians)
double referenceX = 0.0; // Reference Angle Line for X
double referenceY = 0.0; // Reference Angle Line for Y
const double noiseDelta = 0.005; // Reject Differences Below This
double stepX, stepY; // How many pixels to move in each axis
double rawStepX, rawStepY;
const double maxStep = 100.0;
const double sensitivity = 100.0;

// Pointing Mode
double lastX, lastY = 0.0; // Last Position
double pointX, pointY = 0.0; // How much to move

// Adafruit Orientation Classes
imu::Vector<3> euler;
imu::Quaternion quat;

// Adafruit Neopixel Variables
int vbat_raw;
uint8_t vbat_per;
float vbat_mv;

// Motor
const int PIN_MOTOR = 27;
int Motor_PWM_DutyCycle = 20;

// Laser Pointer
const int PIN_LASER = 11;
const int PIN_PRESSURE = 30;
bool sheetPressed = 0;


/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
*/
/**************************************************************************/
void displaySensorDetails(void)
{
  sensor_t sensor;
  bno.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

/**************************************************************************/
/*
    Display some basic info about the sensor status
*/
/**************************************************************************/
void displaySensorStatus(void)
{
  /* Get the system status values (mostly for debugging purposes) */
  uint8_t system_status, self_test_results, system_error;
  system_status = self_test_results = system_error = 0;
  bno.getSystemStatus(&system_status, &self_test_results, &system_error);

  /* Display the results in the Serial Monitor */
  Serial.println("");
  Serial.print("System Status: 0x");
  Serial.println(system_status, HEX);
  Serial.print("Self Test:     0x");
  Serial.println(self_test_results, HEX);
  Serial.print("System Error:  0x");
  Serial.println(system_error, HEX);
  Serial.println("");
  delay(500);
}

/**************************************************************************/
/*
    Display sensor calibration status
*/
/**************************************************************************/
void displayCalStatus(void)
{
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  /* The data should be ignored until the system calibration is > 0 */
  Serial.print("\t");
  if (!system)
  {
    Serial.print("! ");
  }

  /* Display the individual values */
  Serial.print("Sys:");
  Serial.print(system, DEC);
  Serial.print(" G:");
  Serial.print(gyro, DEC);
  Serial.print(" A:");
  Serial.print(accel, DEC);
  Serial.print(" M:");
  Serial.print(mag, DEC);
}

/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
void setup(void)
{
  Serial.begin(9600);

  // Motor
  pinMode(PIN_MOTOR , OUTPUT); // Set up pin 27 as output to the PWM pin to base of BJT
  analogWrite( PIN_MOTOR , 0 );  // OFF

  // Laser
  pinMode(PIN_LASER, OUTPUT);
  digitalWrite( PIN_LASER , 0 );  // OFF

  // Pressure Sheet
  pinMode(PIN_PRESSURE, INPUT);

  // NEOPIXEL CODE
  // Get a single ADC sample and throw it away
  readVBAT();

  strip.begin();
  strip.show(); // Initialize all pixels to 'off'

  Scheduler.startLoop(batteryStatus); // Start Thread batteryStatus (Every 30s)

  bnoInit();

  // BLUETOOTH CODE
  Bluefruit.begin();
  Bluefruit.clearBonds(); // Clear Bonding Information
  // HID Device can have a min connection interval of 9*1.25 = 11.25 ms
  Bluefruit.setConnInterval(9, 16); // min = 9*1.25=11.25 ms, max = 16*1.25=20ms
  // Set max power. Accepted values are: -40, -30, -20, -16, -12, -8, -4, 0, 4
  Bluefruit.setTxPower(4);
  Bluefruit.setName("G07AirMouse");

  // Configure and Start Device Information Service
  bledis.setManufacturer("Adafruit Industries");
  bledis.setModel("Bluefruit Feather 52");
  bledis.begin();

  // BLE HID
  blehid.begin();
  blebas.begin();
  blebas.write(100);

  // Set up and start advertising
  startAdv();
}

void startAdv(void)
{
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addAppearance(BLE_APPEARANCE_HID_MOUSE);

  // Include BLE HID service
  Bluefruit.Advertising.addService(blehid);

  // There is enough room for 'Name' in the advertising packet
  Bluefruit.Advertising.addName();

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

/**************************************************************************/
/*
    Arduino loop function, called once 'setup' is complete (your own code
    should go here)
*/
/**************************************************************************/


void loop(void)
{
  /* Get a new sensor event */
  // BNO055 CODE
  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2
  // euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  //  /* Display calibration status for each sensor. */
  //  uint8_t system, gyro, accel, mag = 0;
  //  bno.getCalibration(&system, &gyro, &accel, &mag);

  // Get Quaternion Vector from BNO055 and convert it to Euler
  quat = bno.getQuat();
  euler = quat.toEuler();

  // Read how much we have moved with respect to references
  readMovement();

  // Set Flex States
  setFlexStates();



  if (!sheetPressed) {
    digitalWrite(PIN_LASER, 1);
  }
  else {
    digitalWrite(PIN_LASER, 0);
    // State Machine (Highest Priority: Top);
    // Scrolling: State = B1111
    if ((currentState & MODE_SCROLLING) == MODE_SCROLLING) {
      //Serial.println("Scrolling");
      blehid.mouseScroll((int)(stepY / 10.0));
    }
    // Pointing: State = B0111
    else if ((currentState & MODE_POINTING) == MODE_POINTING) {
      //Serial.println("Moving");
      // Release Button Presses
      blehid.mouseButtonRelease();
      blehid.mouseMove((int)stepX, (int)stepY);

    }
    // Calibration: State = Bxx1x
    else if ((currentState & MODE_CALIBRATION) == MODE_CALIBRATION) {
      // Serial.println("Calibration");
      referenceX = euler.x();
      referenceY = euler.z();
    }
    // Right Click: State = Bx1xx
    else if ((currentState & MODE_RIGHTCLICK) == MODE_RIGHTCLICK) {
      if (prevState == MODE_MOVING) {
        //Serial.println("Right Click");
        blehid.mouseButtonPress(MOUSE_BUTTON_RIGHT);
        //delay(100);
      }
    }
    // Left Click: State = B1xxx
    else if ((currentState & MODE_LEFTCLICK) == MODE_LEFTCLICK) {
      if (prevState == MODE_MOVING) {
        //Serial.println("Left Click");
        blehid.mouseButtonPress(MOUSE_BUTTON_LEFT);
        //delay(100);
      }
    }

    if (currentState == MODE_MOVING && prevState != MODE_MOVING) {
      // Release Button Presses
      //Serial.println("Release Buttons");
      analogWrite( PIN_MOTOR , Motor_PWM_DutyCycle / 100.0 * 255 ); // 20% Duty Cycle
      blehid.mouseButtonRelease();
      delay(100);
      analogWrite( PIN_MOTOR , 0 );  // OFF
    }

    // Check if below Noise Threshold
    if (!isNoise(euler.x(), lastX) || !isNoise(euler.z(), lastY)) {
      // Get Angle then scale to pixels
      pointX = (getStep(lastX, euler.x())) * (RESOLUTION_X / ANGLE_RANGE);
      pointY = (getStep(lastY, euler.z())) * (RESOLUTION_Y / ANGLE_RANGE);

      lastX = euler.x();
      lastY = euler.z();

      blehid.mouseMove((int)pointX, (int)pointY);
    }


    prevState = currentState;
    //printDebug();
    //delay(BNO055_SAMPLERATE_DELAY_MS);
  }
}

void readMovement() {
  stepX = getStep(referenceX, euler.x());
  stepY = getStep(referenceY, euler.z());

  rawStepX = stepX;
  rawStepY = stepY;

  // See Function: We reject angles close to the reference
  if (!isNoise(stepX, referenceX) || !isNoise(stepY, referenceY)) {
    // Scale Step X and Step Y by Sensitivity
    stepX *= sensitivity;
    stepY *= sensitivity;
    // Limit X Step Movement to 5 Pixels
    if (stepX > maxStep)
      stepX = maxStep;
    else if (stepX < -maxStep)
      stepX = -maxStep;

    // Limit Y Step Movement to 5 Pixels
    if (stepY > maxStep)
      stepY = maxStep;
    else if (stepY < -maxStep)
      stepY = -maxStep;
  }
  else {
    stepX = 0;
    stepY = 0;
  }

}

void setFlexStates() {
  currentState = 0;
  sheetPressed = !digitalRead(PIN_PRESSURE);
  for (int i = A3; i >= A0; i--) {
    currentState <<= 1;
    flexADC[i - A0] = analogRead(i);
    flexVolts[i - A0] = (3.3 * flexADC[i - A0]) / 1023.0;
    flexState[i - A0] = (flexVolts[i - A0] <= flexThreshold[i - A0]) ? true : false;
    currentState |= flexState[i - A0];
  }
}

int isNoise(double x, double y) {
  if (abs(x - y) < noiseDelta)
    return 1;
  return 0;
}

void batteryStatus() {
  // Get Battery State and Display on Neopixel
  vbat_raw = readVBAT();

  // Convert from raw mv to percentage (based on LIPO chemistry)
  vbat_per = mvToPercent(vbat_raw * VBAT_MV_PER_LSB);
  blebas.write(vbat_per);

  // Convert the raw value to compensated mv, taking the resistor-
  // divider into account (providing the actual LIPO voltage)
  // ADC range is 0..3000mV and resolution is 12-bit (0..4095),
  // VBAT voltage divider is 2M + 0.806M, which needs to be added back
  vbat_mv = (float)vbat_raw * VBAT_MV_PER_LSB * VBAT_DIVIDER_COMP;

  delay(30 * 1000); // Delay 30 seconds
}


void printDebug() {
  //  Serial.print("Pressure Sheet: ");
  //  Serial.print(sheetPressed);
  //  Serial.print("\t");
  //
//    Serial.print("Pinky Flex: ");
//    Serial.print(flexVolts[PIN_PINKY]);
//    Serial.print("\t");
//  
    Serial.print("Ring Flex: ");
    Serial.print(flexVolts[PIN_RING]);
    Serial.print("\t");
  
    Serial.print("Middle Flex: ");
    Serial.print(flexVolts[PIN_MIDDLE]);
    Serial.print("\t");
//  
//    Serial.print("Index Flex: ");
//    Serial.print(flexVolts[PIN_INDEX]);
//    Serial.print("\t");
//  
    Serial.print("curState: ");
    Serial.print(currentState);
    Serial.print("\t");
    
    Serial.print("prevState: ");
    Serial.print(prevState);
    Serial.print("\t");
  //  //
  //  Serial.print("IsCalibration: ");
  //  Serial.print(currentState & MODE_CALIBRATION);
  //  Serial.print("\t");

  //  Serial.print("refX: ");
  //  Serial.print(referenceX);
  //  Serial.print(" refY: ");
  //  Serial.print(referenceY);
  //  Serial.print("\t");
  //
  //    Serial.print("eulerX: ");
  //    Serial.print(euler.x());
  //    Serial.print(" eulerY: ");
  //    Serial.print(euler.y());
  //    Serial.print(" eulerZ: ");
  //    Serial.print(euler.z());
  //    Serial.print("\t");

  //  Serial.print("RawStepX: ");
  //  Serial.print(rawStepX);
  //  Serial.print(" RawStepY: ");
  //  Serial.print(rawStepY);
  //  Serial.print("\t");
  //
  //  Serial.print("StepX: ");
  //  Serial.print(stepX);
  //  Serial.print(" StepY: ");
  //  Serial.print(stepY);
  //
  //


  //   //Display the results of the Battery State
  //  Serial.print("ADC = ");
  //  Serial.print(vbat_raw * VBAT_MV_PER_LSB);
  //  Serial.print(" mV (");
  //  Serial.print(vbat_raw);
  //  Serial.print(") ");
  //  Serial.print("LIPO = ");
  //  Serial.print(vbat_mv);
  //  Serial.print(" mV (");
  //  Serial.print(vbat_per);
  //  Serial.println("%)");

    Serial.println("\t\t");

}


// Neopixel Functions
int readVBAT(void) {
  int raw;

  // Set the analog reference to 3.0V (default = 3.6V)
  analogReference(AR_INTERNAL_3_0);

  // Set the resolution to 12-bit (0..4095)
  analogReadResolution(12); // Can be 8, 10, 12 or 14

  // Let the ADC settle
  delay(1);

  // Get the raw 12-bit, 0..3000mV ADC value
  raw = analogRead(VBAT_PIN);

  // Set the ADC back to the default settings
  analogReference(AR_DEFAULT);
  analogReadResolution(10);

  return raw;
}

uint8_t mvToPercent(float mvolts) {
  static const uint8_t maxBrightness = 100;
  uint8_t battery_level;
  uint32_t brightness;
  uint8_t wait = 0;

  if (mvolts >= 3000) {
    battery_level = 100;
    colorWipe(strip.Color(0, maxBrightness, 0), wait); // Green
  }

  else if (mvolts > 2900) {
    battery_level = 100 - ((3000 - mvolts) * 58) / 100;
    brightness = (battery_level / 100.0) * maxBrightness;
    colorWipe(strip.Color(0, brightness, 0), wait); // Green
  }

  else if (mvolts > 2740) {
    battery_level = 42 - ((2900 - mvolts) * 24) / 160;
    brightness = (battery_level / 100.0) * maxBrightness;
    colorWipe(strip.Color(brightness, brightness, 0), wait); // Green
  }

  else if (mvolts > 2440) {
    battery_level = 18 - ((2740 - mvolts) * 12) / 300;
    brightness = (battery_level / 100.0) * maxBrightness;
    colorWipe(strip.Color(brightness, brightness, 0), wait); // Green
  }

  else if (mvolts > 2100) {
    battery_level = 6 - ((2440 - mvolts) * 6) / 340;
    brightness = (battery_level / 100.0) * maxBrightness;
    colorWipe(strip.Color(brightness, 0, 0), wait); // Red
  }
  else {
    battery_level = 0;
    colorWipe(strip.Color(25, 0, 0), wait); // Red
  }

  return battery_level;
}


// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait) {
  for (uint16_t i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, c);
    strip.show();
    delay(wait);
  }
}

void theaterChase(uint32_t c, uint8_t wait) {
  for (int j = 0; j < 10; j++) { //do 10 cycles of chasing
    for (int q = 0; q < 3; q++) {
      for (uint16_t i = 0; i < strip.numPixels(); i = i + 3) {
        strip.setPixelColor(i + q, c);  //turn every third pixel on
      }
      strip.show();

      delay(wait);

      for (uint16_t i = 0; i < strip.numPixels(); i = i + 3) {
        strip.setPixelColor(i + q, 0);      //turn every third pixel off
      }
    }
  }
}

void bnoInit() {
  // BNO055 CODE
  //  Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");

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
  //  Serial.print("Current Temperature: ");
  //  Serial.print(temp);
  //  Serial.println(" C");
  //  Serial.println("");

  bno.setExtCrystalUse(true);
  //  Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");

  // Get Starting Quaternion and Euler
  quat = bno.getQuat();
  euler = quat.toEuler();
  referenceX = euler.x();
  referenceY = euler.z();
}

bool signChange(double x1, double x2) {
  if ((x1 * x2) < 0)
    return true;
  return false;
}

bool piCrossing(double x1, double x2) {
  if (abs(x1 - x2) > PI)
    return true;
  return false;
}

double getStep(double ref, double now) {
  // If there is a sign change
  if (signChange(ref, now) == true) {
    // Check if PI Crossing
    if (piCrossing(ref, now) == true) {
      if (ref > 0)
        return (ref - PI) + (-PI - now);
      else
        return (ref - (-PI)) + (PI - now);
    }
    // Else Zero Crossing
    else {
      return ref - now;
    }
  }
  // Else - Read how much we have moved with respect to referencde in X
  else {
    return ref - now;
  }
}


