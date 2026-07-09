/*
  2-Wheel Drive Combat Robot - iBUS to Dual Bidirectional ESC (Arcade Mixing)
  -----------------------------------------------------
  Reads Ch.1 (steering) and Ch.2 (throttle) from an FS-iA8X receiver
  over iBUS, mixes them into left/right motor commands (differential
  drive), applies a deadband + slew limit to prevent twitching near
  neutral, scales the output down to a configurable speed cap for
  careful 2WD driving mode, and drives two BIDIRECTIONAL brushless ESCs.

  Channel assumption (swap in loop() if your TX differs):
    Ch.1 = steering (left = pull left, right = pull right)
    Ch.2 = throttle  (push forward = forward)

  Mixing:
    left  = throttle + steering
    right = throttle - steering
  Both results are clamped back into valid stick range before being
  converted to ESC pulses.

  Direction convention:
    - The right motor is mounted mirrored relative to the left motor,
      so its final pulse is mirrored around neutral (forward mix
      value -> reverse-direction pulse, which spins the wheel forward
      due to mirrored mounting).

  Speed limiting:
    - SPEED_LIMIT_PERCENT scales throttle/steering down BEFORE mixing,
      so 2WD driving mode is capped at a safe fraction of full ESC
      range (25% by default). Neutral/arming/failsafe are untouched -
      only the maximum deviation from neutral is reduced.

  Anti-twitch measures:
    - DEADBAND ignores small stick/receiver jitter near center so it
      doesn't get amplified through the mix into a false direction flip.
    - MAX_PULSE_STEP slew-limits how fast the pulse can change per loop,
      so crossing the neutral/reverse boundary ramps smoothly instead
      of snapping, giving the ESC's internal brake/reverse logic time
      to settle instead of chattering.

  Wiring:
    FS-iA8X iBUS signal wire -> GPIO 4
    FS-iA8X GND              -> ESP32 GND
    Left ESC signal wire     -> GPIO 32
    Right ESC signal wire    -> GPIO 33
    ESC grounds              -> common ESP32 GND

  Library needed: ESP32Servo (by Kevin Harrington / John K. Bennett)
  Install via: Arduino IDE > Sketch > Include Library > Manage Libraries > "ESP32Servo"

  NOTES:
    - These ESCs are bidirectional: center stick = stopped, above center =
      forward, below center = reverse.
    - Bidirectional ESCs typically arm by seeing a steady NEUTRAL (1500us)
      pulse on startup, not minimum throttle. Confirm this matches your
      specific ESC's arming sequence in its manual.
    - If iBUS signal is lost mid-run, both wheels fail safe to neutral (stop).
    - Check your event's weight class / safety rules for spinner and
      speed-cap requirements before competing.
*/

#include <ESP32Servo.h>
#include <FastLED.h>

#define IBUS_RX_PIN   4
#define IBUS_FRAME_LEN 32
#define LEFT_ESC_PIN  32
#define RIGHT_ESC_PIN 33
#define LED_PIN 21

#define NUM_LEDS    8
#define BRIGHTNESS  64

#define STICK_NEUTRAL 1500  // stick center - adjust if your transmitter's center reads differently
#define STICK_MIN     1000  // stick full one way - adjust if yours doesn't quite reach 1000
#define STICK_MAX     2000  // stick full other way - adjust if yours doesn't quite reach 2000


#define DEADBAND 20         // microseconds of stick jitter to ignore around center

#define SPEED_LIMIT 0.25  // % of full throttle/steering range allowed in 2WD mode
                                 // (1.00 = full range, no limit)

// // Converts a raw stick-range value (post-mix) into an ESC pulse, passed straight through.
// int passThroughMap(int mixedValue) {
//   int clamped = constrain(mixedValue, STICK_MIN, STICK_MAX);
//   return map(clamped, STICK_MIN, STICK_MAX, STICK_MIN, STICK_MAX);
// }

// // Mirrors a mixed value around neutral before mapping, so "forward mix"
// // produces a reverse-direction pulse. Used for the right wheel to
// // compensate for its mirrored mounting.
// int mirroredMap(int mixedValue) {
//   int clamped = constrain(mixedValue, STICK_MIN, STICK_MAX);
//   int mirrored = (2 * STICK_NEUTRAL) - clamped;
//   mirrored = constrain(mirrored, STICK_MIN, STICK_MAX);
//   return map(mirrored, STICK_MIN, STICK_MAX, STICK_MIN, ESC_MAX);
// }

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
CRGB leds[NUM_LEDS];


uint16_t channels[14];
unsigned long lastValidFrameMs = 0;
const unsigned long FAILSAFE_TIMEOUT_MS = 500; // if no frame in this long, stop motors

// =====================================================================================================================
// =====================================================================================================================

void differentialDrive() {
  int steeringRaw = channels[0] - STICK_NEUTRAL; // Ch.1
  int throttleRaw = channels[1] - STICK_NEUTRAL; // Ch.2

  int steering = applyDeadband(steeringRaw);
  int throttle = applyDeadband(throttleRaw);

  steering = steering * SPEED_LIMIT;
  throttle = throttle * SPEED_LIMIT;

  int leftMix  = STICK_NEUTRAL + throttle + steering;
  int rightMix = STICK_NEUTRAL + throttle - steering;

  leftESC.writeMicroseconds(leftMix);
  rightESC.writeMicroseconds(rightMix);
}

void meltyDrive() {
  int throttle = (channels[2] - STICK_MIN)/2; // Bring to 0-500 range
  int throttleConstrained = applyDeadband(throttle) * 0.5; // Apply deadband and speed limit

  leftESC.writeMicroseconds(STICK_NEUTRAL + throttleConstrained);
  rightESC.writeMicroseconds(STICK_NEUTRAL - throttleConstrained);
}

void ledBlue() {
  fill_solid(leds, NUM_LEDS, CRGB::Blue);
  FastLED.show();
}

void ledRed() {
  fill_solid(leds, NUM_LEDS, CRGB::Red);
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

  lastValidFrameMs = millis();
}

void loop() {
  bool gotFrame = readIBus();
  bool driveMelty = channels[3] > 1500;

  if (gotFrame) {
    lastValidFrameMs = millis();
    if (driveMelty) {
      meltyDrive();
      ledRed();
    } else {
      differentialDrive();
      ledBlue();
    }
  }

  // Failsafe: if we haven't seen a valid frame in a while, stop both motors
  if (millis() - lastValidFrameMs > FAILSAFE_TIMEOUT_MS) {
    leftESC.writeMicroseconds(STICK_NEUTRAL);
    rightESC.writeMicroseconds(STICK_NEUTRAL);
  }
}

