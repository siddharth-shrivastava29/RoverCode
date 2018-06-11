/*****************************************************************
  LSM9DS0_Simple.ino
  SFE_LSM9DS0 Library Simple Example Code
  Jim Lindblom @ SparkFun Electronics
  Original Creation Date: February 18, 2014
  https://github.com/sparkfun/LSM9DS0_Breakout

  The LSM9DS0 is a versatile 9DOF sensor. It has a built-in
  accelerometer, gyroscope, and magnetometer. Very cool! Plus it
  functions over either SPI or I2C.

  This Arduino sketch is a demo of the simple side of the
  SFE_LSM9DS0 library. It'll demo the following:
  How to create a LSM9DS0 object, using a constructor (global
  variables section).
  How to use the begin() function of the LSM9DS0 class.
  How to read the gyroscope, accelerometer, and magnetometer
  using the readGryo(), readAccel(), readMag() functions and the
  gx, gy, gz, ax, ay, az, mx, my, and mz variables.
  How to calculate actual acceleration, rotation speed, magnetic
  field strength using the calcAccel(), calcGyro() and calcMag()
  functions.
  How to use the data from the LSM9DS0 to calculate orientation
  and heading.

  Hardware setup: This library supports communicating with the
  LSM9DS0 over either I2C or SPI. If you're using I2C, these are
  the only connections that need to be made:
  LSM9DS0 --------- Arduino
   SCL ---------- SCL (A5 on older 'Duinos')
   SDA ---------- SDA (A4 on older 'Duinos')
   VDD ------------- 3.3V
   GND ------------- GND
  (CSG, CSXM, SDOG, and SDOXM should all be pulled high jumpers on
  the breakout board will do this for you.)

  If you're using SPI, here is an example hardware setup:
  LSM9DS0 --------- Arduino
          CSG -------------- 9
          CSXM ------------- 10
          SDOG ------------- 12
          SDOXM ------------ 12 (tied to SDOG)
          SCL -------------- 13
          SDA -------------- 11
          VDD -------------- 3.3V
          GND -------------- GND

  The LSM9DS0 has a maximum voltage of 3.6V. Make sure you power it
  off the 3.3V rail! And either use level shifters between SCL
  and SDA or just use a 3.3V Arduino Pro.

  Development environment specifics:
  IDE: Arduino 1.0.5
  Hardware Platform: Arduino Pro 3.3V/8MHz
  LSM9DS0 Breakout Version: 1.0

  This code is beerware. If you see me (or any other SparkFun
  employee) at the local, and you've found our code helpful, please
  buy us a round!

  Distributed as-is; no warranty is given.
*****************************************************************/

// The SFE_LSM9DS0 requires both the SPI and Wire libraries.
// Unfortunately, you'll need to include both in the Arduino
// sketch, before including the SFE_LSM9DS0 library.
#include <SPI.h> // Included for SFE_LSM9DS0 library
#include <Wire.h>
#include <SFE_LSM9DS0.h>
#define BFD 10   // Connect Digital Pin 7 on Arduino to CH3 on Relay Module
#define BFU 11   // Connect Digital Pin 7 on Arduino to CH3 on Relay Module
#include <stdlib.h>
///////////////////////
// Example I2C Setup //
///////////////////////
// Comment out this section if you're using SPI
// SDO_XM and SDO_G are both grounded, so our addresses are:
#define LSM9DS0_XM  0x1D // Would be 0x1E if SDO_XM is LOW
#define LSM9DS0_G   0x6B // Would be 0x6A if SDO_G is LOW
// Create an instance of the LSM9DS0 library called `dof` the
// parameters for this constructor are:
// [SPI or I2C Mode declaration],[gyro I2C address],[xm I2C add.]
LSM9DS0 dof(MODE_I2C, LSM9DS0_G, LSM9DS0_XM);

///////////////////////
// Example SPI Setup //
///////////////////////
/* // Uncomment this section if you're using SPI
  #define LSM9DS0_CSG  9  // CSG connected to Arduino pin 9
  #define LSM9DS0_CSXM 10 // CSXM connected to Arduino pin 10
  LSM9DS0 dof(MODE_SPI, LSM9DS0_CSG, LSM9DS0_CSXM);
*/

// Do you want to print calculated values or raw ADC ticks read
// from the sensor? Comment out ONE of the two #defines below
// to pick:
#define PRINT_CALCULATED
//#define PRINT_RAW

#define PRINT_SPEED 10 // 10 ms between prints
float bedAngle;
int desiredBedAngle;
char inChar;
void setup()
{
  Serial.begin(9600); // Start serial at 115200 bps
  // Use the begin() function to initialize the LSM9DS0 library.
  // You can either call it with no parameters (the easy way):
  uint16_t status = dof.begin();
  // Or call it with declarations for sensor scales and data rates:
  //uint16_t status = dof.begin(dof.G_SCALE_2000DPS,
  //                            dof.A_SCALE_6G, dof.M_SCALE_2GS);
  Wire.begin();
  //Setup all the Arduino Pins
  pinMode(BFD, OUTPUT);
  pinMode(BFU, OUTPUT);

  //Turn OFF any power to the Relay channels
  digitalWrite(BFD, HIGH);
  digitalWrite(BFU, HIGH);

  digitalWrite(BFD, LOW);
  delay(5000);
  digitalWrite(BFD, HIGH);
  dof.readGyro();  // Print "G: gx, gy, gz"
  dof.readAccel(); //   "A: ax, ay, az"
  dof.readMag();  //    "M: mx, my, mz"
  // Print the heading and orientation for fun!
  printHeading((float) dof.mx, (float) dof.my);
  bedAngle = printOrientation(dof.calcAccel(dof.ax), dof.calcAccel(dof.ay),
                              dof.calcAccel(dof.az));

  delay(100);


}


void loop()
{

  if (Serial.available() > 0)
  {

    if  (bedAngle < desiredBedAngle) {
      while (bedAngle < desiredBedAngle) {

        digitalWrite(BFU, LOW); // Move bed firgelli down

        dof.readGyro();  //   "G: gx, gy, gz"
        dof.readAccel(); //   "A: ax, ay, az"
        dof.readMag();  //    "M: mx, my, mz"

        // Print the heading and orientation
        printHeading((float) dof.mx, (float) dof.my);
        bedAngle = printOrientation(dof.calcAccel(dof.ax), dof.calcAccel(dof.ay),
                                    dof.calcAccel(dof.az));
      }
      digitalWrite(BFU, HIGH); // Stop

    }
    else {
      while (bedAngle > desiredBedAngle) {

        digitalWrite(BFD, LOW); // Move bed firgelli down
        delay(10);
        dof.readGyro();  //   "G: gx, gy, gz"
        dof.readAccel(); //   "A: ax, ay, az"
        dof.readMag();  //    "M: mx, my, mz"

        // Print the heading and orientation
        printHeading((float) dof.mx, (float) dof.my);
        bedAngle = printOrientation(dof.calcAccel(dof.ax), dof.calcAccel(dof.ay),
                                    dof.calcAccel(dof.az));


        if (bedAngle < 0.5) {
          bedAngle = 0;
          digitalWrite(BFD, HIGH); // Stop
        }
        delay(100);

      }
      bedAngle = desiredBedAngle;
      digitalWrite(BFD, HIGH); // Stop

    }

  }
  Serial.println(bedAngle);
}


void printGyro()
{
  // To read from the gyroscope, you must first call the
  // readGyro() function. When this exits, it'll update the
  // gx, gy, and gz variables with the most current data.
  dof.readGyro();
}

void printAccel()
{
  // To read from the accelerometer, you must first call the
  // readAccel() function. When this exits, it'll update the
  // ax, ay, and az variables with the most current data.
  dof.readAccel();
}

void printMag()
{
  // To read from the magnetometer, you must first call the
  // readMag() function. When this exits, it'll update the
  // mx, my, and mz variables with the most current data.
  dof.readMag();
}

void printHeading(float hx, float hy)
{
  float heading;

  if (hy > 0)
  {
    heading = 90 - (atan(hx / hy) * (180 / PI));
  }
  else if (hy < 0)
  {
    heading = - (atan(hx / hy) * (180 / PI));
  }
  else // hy = 0
  {
    if (hx < 0) heading = 180;
    else heading = 0;
  }

}

// Another fun function that does calculations based on the
// acclerometer data. This function will print your LSM9DS0's
// orientation -- it's roll and pitch angles.
float printOrientation(float x, float y, float z)
{
  float pitch, roll;
  float correction = 3.5;
  pitch = atan2(x, sqrt(y * y) + (z * z));
  roll = atan2(y, sqrt(x * x) + (z * z));
  pitch *= 180.0 / PI;
  pitch = pitch + correction;
  if (pitch < 0) {
    pitch = 0;
  }
  roll *= 180.0 / PI;
  return pitch;

}
