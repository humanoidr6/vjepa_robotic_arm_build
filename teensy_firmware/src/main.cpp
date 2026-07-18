#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ---------------------------------------------------------------------------
// Hardware configuration
// ---------------------------------------------------------------------------
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

#define NUM_SERVOS       7      // 6 arm axes + 1 gripper claw
#define GRIPPER_CHANNEL  6      // PCA9685 channel driving the claw servo

// Direction the claw travels to CLOSE. +1 means higher angle = tighter grip.
// Flip to -1 if your claw linkage is mirrored -- the safety reflex depends on
// this being correct, so verify it by hand before trusting the touch sensor.
#define GRIPPER_CLOSE_DIR (+1)

#define TOUCH_PIN        A0
#define ADC_RESOLUTION   12
#define TOUCH_TRIP       2048   // out of 4095 (12-bit) -- calibrate to sensor
#define TOUCH_RELEASE    1600   // hysteresis: must fall this low to re-arm
#define TOUCH_DEBOUNCE_N 3      // consecutive samples required to change state

#define SERVO_FREQ       50     // standard analog servo PWM frequency (Hz)
#define SERVO_MIN_PULSE  102    // ~500us  ->   0 deg, out of 4096 ticks
#define SERVO_MAX_PULSE  512    // ~2500us -> 180 deg, out of 4096 ticks
#define PCA9685_OSC_HZ   27000000  // measured onboard oscillator, not the 25MHz default

#define TOUCH_READ_INTERVAL_MS 20
#define SERVO_UPDATE_INTERVAL_MS 20
#define TELEMETRY_INTERVAL_MS 500

// Max degrees any joint may move per servo update tick. At 20ms/tick, 2 deg
// gives ~100 deg/sec -- brisk but not a slam. Lower this if the arm lurches.
#define MAX_STEP_DEG     2

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------
// targetAngles is where we want each joint to end up; currentAngles is what we
// have actually commanded so far. The gap between them is walked down at
// MAX_STEP_DEG per tick so no command can ever slam a joint.
int targetAngles[NUM_SERVOS]  = {90, 90, 90, 90, 90, 90, 90};
int currentAngles[NUM_SERVOS] = {90, 90, 90, 90, 90, 90, 90};

// Servos are left limp until an explicit HOME command. We cannot know where the
// arm physically rests at power-up, so driving any angle at boot would yank it
// there at full torque. The operator positions the arm near neutral by hand
// (servos back-drive freely with no pulse), then sends HOME.
bool homed = false;

bool gripperLocked = false;
int touchStreak = 0;            // consecutive samples agreeing with a state change
int lastTouchValue = 0;

String inputBuffer;
unsigned long lastTouchRead = 0;
unsigned long lastServoUpdate = 0;
unsigned long lastTelemetry = 0;

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

void releaseAllServos() {
  // A zero-length pulse leaves the servo unpowered and back-drivable.
  for (int i = 0; i < NUM_SERVOS; i++) {
    pwm.setPWM(i, 0, 0);
  }
}

// Freeze the claw exactly where it is right now. Because currentAngles is the
// slew-limited value we are actively commanding, this genuinely halts travel
// rather than re-issuing the target the claw was still driving toward.
void stopGripper() {
  targetAngles[GRIPPER_CHANNEL] = currentAngles[GRIPPER_CHANNEL];
  setServoAngle(GRIPPER_CHANNEL, currentAngles[GRIPPER_CHANNEL]);
  gripperLocked = true;
}

// Parses a line of the form "a0,a1,a2,a3,a4,a5,a6" and sets it as the target.
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

  if (!homed) {
    Serial.println("ERR: not homed, send HOME first");
    return;
  }

  for (int i = 0; i < NUM_SERVOS; i++) {
    int requested = constrain(angles[i], 0, 180);

    if (i == GRIPPER_CHANNEL && gripperLocked) {
      // The reflex blocks tightening only. Commands that open the claw are
      // still honoured, otherwise a triggered grip could never be released.
      int delta = requested - currentAngles[GRIPPER_CHANNEL];
      if (delta * GRIPPER_CLOSE_DIR > 0) {
        continue;
      }
    }

    targetAngles[i] = requested;
  }
}

void handleLine(const String &line) {
  if (line == "HOME") {
    // Assumes the operator has already placed the arm near neutral by hand.
    for (int i = 0; i < NUM_SERVOS; i++) {
      currentAngles[i] = 90;
      targetAngles[i] = 90;
      setServoAngle(i, 90);
    }
    homed = true;
    gripperLocked = false;
    touchStreak = 0;
    Serial.println("HOMED");
    return;
  }

  if (line == "RELAX") {
    releaseAllServos();
    homed = false;
    Serial.println("RELAXED");
    return;
  }

  parseAndApplyCommand(line);
}

void readSerialCommands() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (inputBuffer.length() > 0) {
        handleLine(inputBuffer);
        inputBuffer = "";
      }
    } else {
      inputBuffer += c;
    }
  }
}

// Walk every joint one bounded step toward its target.
void updateServos() {
  unsigned long now = millis();
  if (now - lastServoUpdate < SERVO_UPDATE_INTERVAL_MS) return;
  lastServoUpdate = now;

  if (!homed) return;

  for (int i = 0; i < NUM_SERVOS; i++) {
    int delta = targetAngles[i] - currentAngles[i];
    if (delta == 0) continue;

    int step = constrain(delta, -MAX_STEP_DEG, MAX_STEP_DEG);
    currentAngles[i] += step;
    setServoAngle(i, currentAngles[i]);
  }
}

void checkTouchSensor() {
  unsigned long now = millis();
  if (now - lastTouchRead < TOUCH_READ_INTERVAL_MS) return;
  lastTouchRead = now;

  lastTouchValue = analogRead(TOUCH_PIN);

  // Separate trip and release thresholds with an N-sample streak, so a sensor
  // hovering near the boundary cannot chatter the lock on and off.
  if (!gripperLocked && lastTouchValue > TOUCH_TRIP) {
    touchStreak++;
    if (touchStreak >= TOUCH_DEBOUNCE_N) {
      stopGripper();
      touchStreak = 0;
      Serial.println("SAFETY: gripper stopped, touch threshold exceeded");
    }
  } else if (gripperLocked && lastTouchValue < TOUCH_RELEASE) {
    touchStreak++;
    if (touchStreak >= TOUCH_DEBOUNCE_N) {
      gripperLocked = false;
      touchStreak = 0;
      Serial.println("SAFETY: gripper re-armed");
    }
  } else {
    touchStreak = 0;
  }
}

void reportTelemetry() {
  unsigned long now = millis();
  if (now - lastTelemetry < TELEMETRY_INTERVAL_MS) return;
  lastTelemetry = now;

  Serial.print("TOUCH:");
  Serial.print(lastTouchValue);
  Serial.print(" LOCK:");
  Serial.print(gripperLocked ? 1 : 0);
  Serial.print(" POS:");
  for (int i = 0; i < NUM_SERVOS; i++) {
    Serial.print(currentAngles[i]);
    if (i < NUM_SERVOS - 1) Serial.print(",");
  }
  Serial.println();
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
  pwm.setOscillatorFrequency(PCA9685_OSC_HZ);
  pwm.setPWMFreq(SERVO_FREQ);
  delay(10);

  analogReadResolution(ADC_RESOLUTION);

  // Deliberately do NOT drive the servos here -- see the `homed` comment above.
  releaseAllServos();

  Serial.println("READY (limp -- position arm near neutral, then send HOME)");
}

void loop() {
  readSerialCommands();
  updateServos();
  checkTouchSensor();
  reportTelemetry();
}
