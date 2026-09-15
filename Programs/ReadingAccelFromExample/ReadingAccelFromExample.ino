#include "SparkFun_LIS331.h"
#include <SPI.h>
#include <string>

LIS331 xl;

void setup() 
{
  // put your setup code here, to run once:
  pinMode(5, OUTPUT);    // CS for SPI
  digitalWrite(5, HIGH); // Make CS high
  pinMode(23, OUTPUT);    // MOSI for SPI
  pinMode(19, INPUT);     // MISO for SPI
  pinMode(18, OUTPUT);    // SCK for SPI
  SPI.begin();
  xl.setSPICSPin(5);     // This MUST be called BEFORE .begin() so 
                          //  .begin() can communicate with the chip
  xl.begin(LIS331::USE_SPI); // Selects the bus to be used and sets
                          //  the power up bit on the accelerometer.
                          //  Also zeroes out all accelerometer
                          //  registers that are user writable.
  // This next section configures an interrupt. It will cause pin
  //  INT1 on the accelerometer to go high when the absolute value
  //  of the reading on the Z-axis exceeds a certain level for a
  //  certain number of samples.
  xl.setFullScale(LIS331::LOW_RANGE);
  Serial.begin(115200);
}
 
#define MAX_SCALE 10
  
double getAccelY(double y) {
  xl.convertToG(MAX_SCALE, y);

  return (y-12)/21;
}
double getAccelZ(double z) {
  xl.convertToG(MAX_SCALE, z);

  return -(z-60)/24;
}

void printAccelValues() {
  int16_t x, y, z;
  xl.readAxes(x, y, z);

  Serial.print("Y: ");
  Serial.println(getAccelY(y));
  Serial.print("Z: ");
  Serial.println(getAccelZ(z));
  Serial.println(" "); 
}

void loop() 
{
  static long loopTimer = 0;
  if (millis() - loopTimer > 100)
  {
    loopTimer = millis();
    printAccelValues();  // The readAxes() function transfers the
                           //  current axis readings into the three
                           //  parameter variables passed to it.
                 // maximum g-rating.
  }
}
