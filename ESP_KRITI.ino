/*************************************************************
   ESP32 + A4988 + Servo + IR + Blynk
   Includes One-Time Startup Self Test
 *************************************************************/

#define BLYNK_TEMPLATE_ID           "TMPL3_p0RC-dP"
#define BLYNK_TEMPLATE_NAME         "Quickstart Device"
#define BLYNK_AUTH_TOKEN            "-ipMOEm8sYgurgJhUEYxYQUr-PXe_AFb"

#define BLYNK_PRINT Serial

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <ESP32Servo.h>

/* ---------------- WIFI ---------------- */
char ssid[] = "BigD";
char pass[] = "snfc7882";

/* ---------------- PIN DEFINITIONS ---------------- */
#define IR1_PIN 14
#define IR2_PIN 27
#define IR3_PIN 26
#define SERVO_PIN 25

#define STEP_PIN 33
#define DIR_PIN 32
#define ENABLE_PIN 4

/* ---------------- STEPPER ---------------- */
const int stepsPerRevolution = 200;
int rpm = 60;
unsigned long stepInterval;
unsigned long lastStepTime = 0;
bool motorRunning = false;
bool systemEnabled = true;
int controlSig = 1;
int count1 = 0;
String msg1;
/* ---------------- SERVO ---------------- */
Servo bumpGate;
int gateOpenAngle = 180;
int gateClosedAngle = 10;
bool servoPosition = false;
unsigned long servoLastToggle = 0;
unsigned long servoInterval = 500;
int count = 0;
bool lastIR2State = HIGH;

/* ---------------- FLOW DETECTION ---------------- */
bool lastIR1State = LOW;
unsigned long lastEdgeTime = 0;
unsigned long highStartTime = 0;
int pulseCount = 0;
unsigned long windowStart = 0;
unsigned long nowTimeL = 0;
unsigned long lastTimeL = 0;
const unsigned long continuousThreshold = 600;
const unsigned long windowDuration = (controlSig+1)*1000;
const unsigned long jamTimeout = (controlSig+1)*1000;
bool lastIR2Pulse = HIGH;
int ir2PulseCount = 0;
unsigned long ir2WindowStart = 0;
String msg;
enum TriggerState {
  NO_FEED,
  INTERMITTENT,
  CONTINUOUS
};

TriggerState triggerState = NO_FEED;

/* ================================================== */
/* ---------------- STARTUP SELF TEST --------------- */
/* ================================================== */
void updateBallLap() {

  bool currentState = digitalRead(IR2_PIN);

  // detect new ball (HIGH → LOW transition)
  if (currentState == LOW && lastIR2State == HIGH) {

    nowTimeL = millis();

    if (lastTimeL != 0) {

      unsigned long lap = nowTimeL - lastTimeL;
      count++;
      Serial.print("Ball interval: ");
      Serial.print(lap);
      Serial.println(" ms");
      if (lap < 8000) {
        msg = String(lap);
      }
      else {
        msg = "Jam";
      }
      // optional: send to Blynk
      Blynk.virtualWrite(V5, msg);
      Blynk.virtualWrite(V6, count);
    }

    lastTimeL = nowTimeL;
  }

  lastIR2State = currentState;
}

void startupSelfTest() {
  Serial.println("Running startup test...");
  count=0;
  /* --- Servo Test --- */
  bumpGate.write(gateClosedAngle);
  delay(500);

  bumpGate.write(gateOpenAngle);
  delay(800);

  bumpGate.write(gateClosedAngle);
  delay(500);

  /* --- Stepper Test --- */
  digitalWrite(ENABLE_PIN, LOW);

  // Forward
  digitalWrite(DIR_PIN, HIGH);
  for (int i = 0; i < stepsPerRevolution; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(800);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(800);
  }

  delay(500);

  // Reverse
  digitalWrite(DIR_PIN, LOW);
  for (int i = 0; i < stepsPerRevolution; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(800);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(800);
  }

  Serial.println("Startup test complete.");
}

/* ================================================== */
/* ---------------- BLYNK CONTROL ------------------- */
/* ================================================== */

BLYNK_WRITE(V0) {
  systemEnabled = param.asInt();
}
BLYNK_WRITE(V4) {
  controlSig = param.asInt();
  servoInterval = controlSig * 500;
}

/* ================================================== */
/* ---------------- MOTOR CONTROL ------------------- */
/* ================================================== */

void startMotor(bool dir) {
  digitalWrite(DIR_PIN, dir);
  motorRunning = true;
}

void stopMotor() {
  motorRunning = false;
}

void updateMotor() {

  if (!motorRunning) return;

  unsigned long now = micros();

  if (now - lastStepTime >= stepInterval) {
    lastStepTime = now;
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(2);
    digitalWrite(STEP_PIN, LOW);
  }
}

/* ================================================== */
/* ---------------- SERVO --------------------------- */
/* ================================================== */

void updateServo() {

  if (triggerState != INTERMITTENT &&
      triggerState != CONTINUOUS) {

    if (servoPosition) {
      bumpGate.write(gateClosedAngle);
      servoPosition = false;
    }
    return;
  }

  unsigned long now = millis();
  if (servoInterval < 600) servoInterval = 600;
  if (now - servoLastToggle >= servoInterval) {
    servoLastToggle = now;

    servoPosition = !servoPosition;

    if (servoPosition) {
      bumpGate.write(gateOpenAngle);
    }
    else
      bumpGate.write(gateClosedAngle);
  }
}

/* ================================================== */
/* ---------------- TRIGGER LOGIC ------------------- */
/* ================================================== */

void updateTriggerLogic() {

  unsigned long now = millis();

  /* ---------- IR1 logic (NO_FEED / CONTINUOUS) ---------- */

  bool ir1State = !digitalRead(IR1_PIN);

  if (ir1State != lastIR1State) {

    lastEdgeTime = now;

    if (ir1State == HIGH) {
      highStartTime = now;
    }

    lastIR1State = ir1State;
  }

  if (ir1State == HIGH &&
      (now - highStartTime) > continuousThreshold) {
    if (triggerState != INTERMITTENT) triggerState = CONTINUOUS;
    return;
  }

  /*if ((now - lastEdgeTime) > jamTimeout) {
    triggerState = NO_FEED;
  }
*/
  /* ---------- IR2 logic (INTERMITTENT only) ---------- */

  bool ir2State = digitalRead(IR2_PIN);
  if (ir2State == LOW && lastIR2Pulse == HIGH) {
    if (ir2WindowStart == 0)
      ir2WindowStart = now;

    ir2PulseCount++;
    triggerState = INTERMITTENT;
  }
  if (lastTimeL!=0 && ir2WindowStart == 0 &&
    (now - lastTimeL) > jamTimeout) {

  triggerState = NO_FEED;
}

  lastIR2Pulse = ir2State;

  if (ir2WindowStart != 0 &&
      (now - ir2WindowStart) > windowDuration) {

    if (ir2PulseCount >= 1) {
      triggerState = INTERMITTENT;
    }

    ir2PulseCount = 0;
    ir2WindowStart = 0;
  }
}

/* ================================================== */
/* ---------------- SETUP --------------------------- */
/* ================================================== */

void setup() {

  Serial.begin(115200);

  pinMode(IR1_PIN, INPUT);
  pinMode(IR2_PIN, INPUT);
  pinMode(IR3_PIN, INPUT);

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);

  digitalWrite(ENABLE_PIN, LOW);

  stepInterval = (60L * 1000000L) / (stepsPerRevolution * rpm);

  // Proper ESP32 servo initialization
  ESP32PWM::allocateTimer(0);
  bumpGate.setPeriodHertz(50);
  bumpGate.attach(SERVO_PIN, 500, 2400);
  bumpGate.write(gateClosedAngle);

  // Run one-time hardware self test
  startupSelfTest();
  // Connect to Blynk after test
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
}

/* ================================================== */
/* ---------------- LOOP ---------------------------- */
/* ================================================== */

void loop() {

  Blynk.run();
  updateBallLap();
  if (!systemEnabled) {
    stopMotor();
    return;
  }

  updateTriggerLogic();

  switch(triggerState) {

    case CONTINUOUS:
      msg1="Continuous";
      stopMotor();
      break;

    case INTERMITTENT:
      msg1="Intermittent";
      startMotor(HIGH);
      break;

    case NO_FEED:
    msg1="No Feed";
      if (count1 < 100000) {
        startMotor(LOW);
      }
      else if (count1 < 200000) {
        startMotor(HIGH);
      }
      else {
        count1 = 0;
      } 
      count1++;

  break;
  }
  Blynk.virtualWrite(V1, msg1);
  updateMotor();
  updateServo();
  Serial.println(triggerState);
}