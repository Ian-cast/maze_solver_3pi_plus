#include <Wire.h>
#include <VL53L0X.h>

// Set to 1 to only print encoder counts (for checking/calibrating), 0 to run the maze
#define ENCODER_TEST 1

// ---------- Motor pins ----------
const int R_PWM = 14, R_DIR = 10;
const int L_PWM = 15, L_DIR = 11;

// ---------- Encoder pins ----------
const int R_A = 8,  R_B = 9;
const int L_A = 12, L_B = 13;

volatile long cntL = 0, cntR = 0;
volatile uint8_t prevL = 0, prevR = 0;
const int8_t QEM[16] = {0,1,-1,0, -1,0,0,1, 1,0,0,-1, 0,-1,1,0};

void isrL() {
  uint8_t s = (digitalRead(L_A) << 1) | digitalRead(L_B);
  cntL += QEM[(prevL << 2) | s];
  prevL = s;
}
void isrR() {
  uint8_t s = (digitalRead(R_A) << 1) | digitalRead(R_B);
  cntR += QEM[(prevR << 2) | s];
  prevR = s;
}

void resetCounts() {
  noInterrupts(); cntL = 0; cntR = 0; interrupts();
}
long absL() { noInterrupts(); long v = labs(cntL); interrupts(); return v; }
long absR() { noInterrupts(); long v = labs(cntR); interrupts(); return v; }

// ---------- Robot geometry (approximate: check and calibrate!) ----------
const float CPR      = 180;   // counts per wheel revolution (12 x 29.86 for 30:1 motors)
const float WHEEL_MM = 32.0;    // wheel diameter
const float TRACK_MM = 96.0;    // distance between wheels
const float TURN_FUDGE = 1.0;   // multiply to correct under/over-turning

const float COUNTS_PER_MM = CPR / (PI * WHEEL_MM);
const long  COUNTS_90  = (long)(CPR * TRACK_MM / (4.0 * WHEEL_MM) * TURN_FUDGE);
const long  COUNTS_180 = COUNTS_90 * 2;

// ---------- Sensors ----------
const uint8_t MUX_ADDR = 0x70;
const uint8_t N = 5;
enum { S_LEFT = 0, S_FL = 1, S_FRONT = 2, S_FR = 3, S_RIGHT = 4 };
VL53L0X sensor[N];
uint16_t d[N];

// ---------- Tunable settings ----------
const int   BASE_SPEED   = 90;
const float KP           = 0.5;
const int   TARGET_RIGHT = 60;
const int   FRONT_STOP   = 80;
const int   OPEN_SIDE    = 150;

const int   TURN_SPEED     = 100;
const int   MIN_TURN_SPEED = 60;   // slow speed near the end of a turn
const int   ADVANCE_MM     = 20;   // forward before turning at an opening
const int   ENTER_MM       = 30;   // forward after turning
const int   COOLDOWN_MS    = 500;
const float KP_STRAIGHT    = 2.0;  // keeps both wheels at the same count

unsigned long lastTurn = 0;
unsigned long lastPrint = 0;

// ---------- Multiplexer ----------
void canal(uint8_t ch) {
  Wire.beginTransmission(MUX_ADDR);
  Wire.write(1 << ch);
  Wire.endTransmission();
}

void readSensors() {
  for (uint8_t i = 0; i < N; i++) {
    canal(i);
    uint16_t mm = sensor[i].readRangeContinuousMillimeters();
    if (!sensor[i].timeoutOccurred()) d[i] = mm;
  }
}

// ---------- Motors: -255..255, positive = forward ----------
void setMotors(int left, int right) {
  left  = constrain(left,  -255, 255);
  right = constrain(right, -255, 255);
  digitalWrite(L_DIR, left  >= 0 ? HIGH : LOW);
  digitalWrite(R_DIR, right >= 0 ? HIGH : LOW);
  analogWrite(L_PWM, abs(left));
  analogWrite(R_PWM, abs(right));
}

void stopMotors() { setMotors(0, 0); }

// ---------- Encoder-based moves ----------
void forwardMm(int mm) {
  long target = (long)(mm * COUNTS_PER_MM);
  resetCounts();
  unsigned long t0 = millis();
  while (millis() - t0 < 3000) {
    long l = absL(), r = absR();
    if ((l + r) / 2 >= target) break;
    int err = (int)(l - r);   // left ahead -> slow left, speed up right
    setMotors(BASE_SPEED - (int)(KP_STRAIGHT * err),
              BASE_SPEED + (int)(KP_STRAIGHT * err));
  }
  stopMotors();
}

void turnCounts(bool toRight, long target) {
  stopMotors();
  delay(50);
  resetCounts();
  int sgn = toRight ? 1 : -1;       // right turn: left wheel forward, right wheel back
  unsigned long t0 = millis();
  while (millis() - t0 < 2500) {
    long l = absL(), r = absR();
    if (l >= target && r >= target) break;
    int sl = 0, sr = 0;
    if (l < target) sl = (target - l > 30) ? TURN_SPEED : MIN_TURN_SPEED;
    if (r < target) sr = (target - r > 30) ? TURN_SPEED : MIN_TURN_SPEED;
    setMotors(sgn * sl, -sgn * sr);
  }
  stopMotors();
  delay(50);
  readSensors();
  readSensors();
  lastTurn = millis();
}

void followRightWall() {
  int right = constrain((int)d[S_RIGHT], 0, TARGET_RIGHT * 2);
  float error = right - TARGET_RIGHT;
  int correction = (int)(KP * error);
  setMotors(BASE_SPEED + correction, BASE_SPEED - correction);
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}

  pinMode(R_PWM, OUTPUT); pinMode(R_DIR, OUTPUT);
  pinMode(L_PWM, OUTPUT); pinMode(L_DIR, OUTPUT);
  stopMotors();

  pinMode(L_A, INPUT); pinMode(L_B, INPUT);
  pinMode(R_A, INPUT); pinMode(R_B, INPUT);
  prevL = (digitalRead(L_A) << 1) | digitalRead(L_B);
  prevR = (digitalRead(R_A) << 1) | digitalRead(R_B);
  attachInterrupt(digitalPinToInterrupt(L_A), isrL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(L_B), isrL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(R_A), isrR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(R_B), isrR, CHANGE);

#if ENCODER_TEST
  Serial.println("Encoder test: turn each wheel by hand.");
  return;
#endif

  Wire.begin();
  for (uint8_t i = 0; i < N; i++) {
    d[i] = 8190;
    canal(i);
    sensor[i].setTimeout(200);
    if (!sensor[i].init()) {
      Serial.print("Sensor on channel ");
      Serial.print(i);
      Serial.println(" FAILED. Stopping.");
      while (true) delay(1000);
    }
    sensor[i].setMeasurementTimingBudget(20000);
    sensor[i].startContinuous();
  }

  Serial.print("COUNTS_90 = ");
  Serial.println(COUNTS_90);
  Serial.println("Ready. Starting in 3 seconds...");
  delay(3000);
}

void loop() {
#if ENCODER_TEST
  Serial.print("L: ");
  Serial.print(cntL);
  Serial.print("\tR: ");
  Serial.println(cntR);
  delay(200);
  return;
#endif

  readSensors();

  bool rightOpen    = d[S_RIGHT] > OPEN_SIDE;
  bool frontBlocked = d[S_FRONT] < FRONT_STOP;
  bool leftOpen     = d[S_LEFT]  > OPEN_SIDE;

  if (rightOpen && (millis() - lastTurn > COOLDOWN_MS)) {
    forwardMm(ADVANCE_MM);
    turnCounts(true, COUNTS_90);
    forwardMm(ENTER_MM);
  }
  else if (!frontBlocked) {
    followRightWall();
  }
  else if (leftOpen) {
    turnCounts(false, COUNTS_90);
  }
  else {
    turnCounts(true, COUNTS_180);
  }

  if (millis() - lastPrint > 200) {
    lastPrint = millis();
    for (uint8_t i = 0; i < N; i++) {
      Serial.print(d[i]);
      Serial.print("\t");
    }
    Serial.println();
  }
}