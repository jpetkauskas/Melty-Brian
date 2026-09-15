#include <ESP32Servo.h>
#include <FastLED.h>
#include "SparkFun_LIS331.h"
#include <SPI.h>
#include <string>

#define IBUS_RX_PIN   4
#define IBUS_FRAME_LEN 32
#define LEFT_ESC_PIN  32
#define RIGHT_ESC_PIN 33
#define LED_PIN 21

#define NUM_LEDS    8
#define BRIGHTNESS  64

#define STICK_NEUTRAL 1500
#define STICK_MIN     1000 
#define STICK_MAX     2000  

#define DIAL_MIN 1066
#define DIAL_MAX 2000

#define DEADBAND 20         // microseconds of stick jitter to ignore around center

#define SPEED_LIMIT_DIFFERENTIAL 0.25  // % of full throttle/steering range allowed in 2WD mode
                                 // (1.00 = full range, no limit)
#define SPEED_LIMIT_MELTY 1

#define SPEED_LIMIT_DIFFERENTIAL 0.25  // % of full throttle/steering range allowed in 2WD mode (0.00 - 1.00)
#define SPEED_LIMIT_MELTY 1

#define ACCELEROMETER_OFFSET 0.75 // inches
#define BASE_MAX_TRANS_VELOCITY 5 // inches/second
#define MAX_SCALE 10

// Zeroes out small jitter around center so noise doesn't get amplified through mixing
int applyDeadband(int rawValue) {
  return (abs(rawValue) < DEADBAND) ? 0 : rawValue;
}

// Limits how much a pulse can change in one loop iteration, so transitions
// through neutral (direction reversals) ramp smoothly instead of snapping
// int slewLimit(int target, int lastValue, int maxStep) {
//   if (target > lastValue + maxStep) return lastValue + maxStep;
//   if (target < lastValue - maxStep) return lastValue - maxStep;
//   return target;
// }

HardwareSerial IBusSerial(1);
Servo leftESC;
Servo rightESC;
LIS331 xl;
CRGB leds[NUM_LEDS];

uint16_t channels[14];
unsigned long lastValidFrameMs = 0;
const unsigned long FAILSAFE_TIMEOUT_MS = 500; // if no frame in this long, stop motors

// =====================================================================================================================
// =====================================================================================================================

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

double readDial() { // Right dial on controller
  int DIAL_NEUTRAL = (DIAL_MAX+DIAL_MIN)/2;
  int dialConstrained = (channels[5] - DIAL_NEUTRAL) / (DIAL_MAX - DIAL_NEUTRAL); // Range -1:1
  int headingAdjustment = dialConstrained*20; // = Amount of RPM allowed for adjustment
}

double getHeading(double previousHeading, unsigned long tInitial) { // rotations
  int16_t x, y, z;
  xl.readAxes(x, y, z);

  // Get X and Y acceleration
  double accelerationY = getAccelY(y);

  double rotationalSpeed = sqrt(accelerationY / (ACCELEROMETER_OFFSET * 0.0000284)) * (60 * 1000000) + readDial(); // Rotations per microsecond

  unsigned long tFinal = micros();

  unsigned long t = tFinal - tInitial;
  Serial.println(t);
  double headingChange = rotationalSpeed * t;

  return std::remainder(previousHeading + headingChange, 360);
}

void differentialDrive() {
  int steeringRaw = channels[0] - STICK_NEUTRAL; // Ch.1
  int throttleRaw = channels[1] - STICK_NEUTRAL; // Ch.2

  int steering = applyDeadband(steeringRaw);
  int throttle = applyDeadband(throttleRaw);

  steering = steering * SPEED_LIMIT_DIFFERENTIAL;
  throttle = throttle * SPEED_LIMIT_DIFFERENTIAL;

  int leftMix  = STICK_NEUTRAL + throttle + steering;
  int rightMix = STICK_NEUTRAL + throttle - steering;

  leftESC.writeMicroseconds(leftMix);
  rightESC.writeMicroseconds(rightMix);
}

void meltyDrive(double heading) {
  int throttleLeft = (channels[2] - STICK_MIN)/2; // Bring to 0-500 range
  int throttleRight = (channels[2] - STICK_MIN)/2;

  int throttleConstrainedLeft = applyDeadband(throttleLeft) * SPEED_LIMIT_MELTY; // Apply deadband and speed limit
  int throttleConstrainedRight = applyDeadband(throttleRight) * SPEED_LIMIT_MELTY;

  double joystickX = (channels[0] - STICK_NEUTRAL)/500;
  double joystickY = (channels[1] - STICK_NEUTRAL)/500;

  double baseTransAng = atan2(joystickY, joystickX);
  double baseTransMag = hypot(joystickX, joystickY);

  double leftFluctuatingPower = baseTransMag * (500 - throttleConstrainedLeft) * cos(heading - baseTransAng);
  double rightFluctuatingPower = baseTransMag * (500 - throttleConstrainedRight) * cos(heading - baseTransAng);

  leftESC.writeMicroseconds(STICK_NEUTRAL + throttleConstrainedLeft + leftFluctuatingPower);
  rightESC.writeMicroseconds(STICK_NEUTRAL - (throttleConstrainedRight - rightFluctuatingPower));
}

void ledBlue() {
  fill_solid(leds, NUM_LEDS, CRGB::Blue);
  FastLED.show();
}

void ledRed() {
  fill_solid(leds, NUM_LEDS, CRGB::Red);
  FastLED.show();
}
void ledOff() {
  fill_solid(leds, NUM_LEDS, CRGB::Black);
  FastLED.show();
}

bool readIBus() {
  static uint8_t buf[IBUS_FRAME_LEN];
  static uint8_t idx = 0;
  bool newFrame = false;

  while (IBusSerial.available()) {
    uint8_t b = IBusSerial.read();

    if (idx == 0 && b != 0x20) continue;
    if (idx == 1 && b != 0x40) { idx = 0; continue; }

    buf[idx++] = b;

    if (idx == IBUS_FRAME_LEN) {
      idx = 0;

      uint16_t checksum = 0xFFFF;
      for (int i = 0; i < 30; i++) checksum -= buf[i];
      uint16_t received = buf[30] | (buf[31] << 8);

      if (checksum == received) {
        for (int ch = 0; ch < 14; ch++) {
          channels[ch] = buf[2 + ch * 2] | (buf[3 + ch * 2] << 8);
        }
        newFrame = true;
      }
    }
  }
  return newFrame;
}

// =====================================================================================================================
// =====================================================================================================================

void setup() {
  Serial.begin(921600);
  IBusSerial.begin(115200, SERIAL_8N1, IBUS_RX_PIN, -1);

  leftESC.attach(LEFT_ESC_PIN, STICK_MIN, STICK_MAX);
  rightESC.attach(RIGHT_ESC_PIN, STICK_MIN, STICK_MAX);

  FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(BRIGHTNESS);

  Serial.println("Arming ESCs - holding neutral throttle...");
  leftESC.writeMicroseconds(STICK_NEUTRAL);
  rightESC.writeMicroseconds(STICK_NEUTRAL);
  delay(3000); // arming window
  Serial.println("ESCs armed. Ready for input.");

  pinMode(5, OUTPUT);    // CS for SPI
  digitalWrite(5, HIGH); // Make CS high
  pinMode(23, OUTPUT);    // MOSI for SPI
  pinMode(19, INPUT);     // MISO for SPI
  pinMode(18, OUTPUT);    // SCK for SPI
  SPI.begin();

  xl.setSPICSPin(5);
  xl.begin(LIS331::USE_SPI);
  xl.setFullScale(LIS331::LOW_RANGE);

  lastValidFrameMs = millis();
}

double heading = 0;
void loop() {
  unsigned long tInitial = micros();
  static long loopTimer = 0;
  bool gotFrame = readIBus();
  bool driveMelty = channels[3] > 1500;

  if (gotFrame) {
    lastValidFrameMs = millis();
    if (driveMelty) {
      meltyDrive(heading);
      ledRed();

      if (heading < 5 || heading > 355) {
        ledRed();
      } else {
        ledOff();
      }
    } else {
      differentialDrive();
      ledBlue();
    }
  }

  if (millis() - loopTimer > 100)
  {
    loopTimer = millis();
    printAccelValues();  // The readAxes() function transfers the
                           //  current axis readings into the three
                           //  parameter variables passed to it.
                 // maximum g-rating.
  }

  // Failsafe: if we haven't seen a valid frame in a while, stop both motors
  if (millis() - lastValidFrameMs > FAILSAFE_TIMEOUT_MS) {
    leftESC.writeMicroseconds(STICK_NEUTRAL);
    rightESC.writeMicroseconds(STICK_NEUTRAL);
  }

  heading = getHeading(heading, tInitial);
}