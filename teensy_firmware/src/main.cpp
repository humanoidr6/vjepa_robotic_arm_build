#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ---------------------------------------------------------------------------
// Hardware configuration
// ---------------------------------------------------------------------------
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

#define NUM_SERVOS       7      // 6 arm axes + 1 gripper claw
#define GRIPPER_CHANNEL  6      // PCA9685 channel driving the claw servo

#define TOUCH_PIN        A0
#define ADC_RESOLUTION   12
#define TOUCH_THRESHOLD  2048   // out of 4095 (12-bit) -- calibrate to sensor

#define SERVO_FREQ       50     // standard analog servo PWM frequency (Hz)
#define SERVO_MIN_PULSE  102    // ~500us  ->   0 deg, out of 4096 ticks
#define SERVO_MAX_PULSE  512    // ~2500us -> 180 deg, out of 4096 ticks

#define TOUCH_READ_INTERVAL_MS 20

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------
int currentAngles[NUM_SERVOS] = {90, 90, 90, 90, 90, 90, 90};
bool gripperLocked = false;

String inputBuffer;
unsigned long lastTouchRead = 0;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
uint16_t angleToPulse(int angle) {
  angle = constrain(angle, 0, 180);
  return (uint16_t)map(angle, 0, 180, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
}

void setServoAngle(uint8_t channel, int angle) {
  pwm.setPWM(channel, 0, angleToPulse(angle));
}

void stopGripper() {
  // Safety reflex: hold the claw at its last known-safe position and refuse
  // further motion commands until the touch value drops back below threshold.
  pwm.setPWM(GRIPPER_CHANNEL, 0, angleToPulse(currentAngles[GRIPPER_CHANNEL]));
  gripperLocked = true;
}

// Parses a line of the form "a0,a1,a2,a3,a4,a5,a6" and applies it to the servos.
void parseAndApplyCommand(const String &cmd) {
  int angles[NUM_SERVOS];
  int count = 0;
  int start = 0;

  for (int i = 0; i <= (int)cmd.length(); i++) {
    if (i == (int)cmd.length() || cmd.charAt(i) == ',') {
      if (count < NUM_SERVOS) {
        angles[count] = cmd.substring(start, i).toInt();
        count++;
      }
      start = i + 1;
    }
  }

  if (count != NUM_SERVOS) {
    Serial.println("ERR: expected 7 comma-separated angles");
    return;
  }

  for (int i = 0; i < NUM_SERVOS; i++) {
    if (i == GRIPPER_CHANNEL && gripperLocked) {
      continue; // safety reflex overrides incoming gripper commands
    }
    currentAngles[i] = constrain(angles[i], 0, 180);
    setServoAngle(i, currentAngles[i]);
  }
}

void readSerialCommands() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (inputBuffer.length() > 0) {
        parseAndApplyCommand(inputBuffer);
        inputBuffer = "";
      }
    } else {
      inputBuffer += c;
    }
  }
}

void checkTouchSensor() {
  unsigned long now = millis();
  if (now - lastTouchRead < TOUCH_READ_INTERVAL_MS) return;
  lastTouchRead = now;

  int touchValue = analogRead(TOUCH_PIN);

  Serial.print("TOUCH:");
  Serial.println(touchValue);

  if (touchValue > TOUCH_THRESHOLD) {
    stopGripper();
    Serial.println("SAFETY: gripper stopped, touch threshold exceeded");
  } else {
    gripperLocked = false;
  }
}

// ---------------------------------------------------------------------------
// Setup / Loop
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {
    ; // wait briefly for USB serial, but don't hang forever if untethered
  }

  Wire.begin();
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);
  delay(10);

  analogReadResolution(ADC_RESOLUTION);

  for (int i = 0; i < NUM_SERVOS; i++) {
    setServoAngle(i, currentAngles[i]);
  }

  Serial.println("READY");
}

void loop() {
  readSerialCommands();
  checkTouchSensor();
}
