struct PIDState {
  float integ = 0;
  long  prevPos = 0;
  float velF = 0;
};

// ---------- Pines de motores ----------
const int R_PWM = 14, R_DIR = 10;
const int L_PWM = 15, L_DIR = 11;

// ---------- Pines de encoders ----------
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

void resetCounts() { noInterrupts(); cntL = 0; cntR = 0; interrupts(); }
long getL() { noInterrupts(); long v = cntL; interrupts(); return v; }
long getR() { noInterrupts(); long v = cntR; interrupts(); return v; }

// =====================================================
//  PARAMETROS AJUSTABLES (edita aqui y vuelve a subir)
// =====================================================
const float ANGULO_GRADOS = -90.0;   // + derecha, - izquierda
const bool  ALTERNAR      = false;  // true: alterna derecha/izquierda
const int   PAUSA_MS      = 800;    // pausa entre giros

// Signo de cada encoder: 1 si cuenta positivo cuando la rueda va adelante,
// -1 si cuenta negativo. Si una rueda se descontrola, cambia su signo.
const int ENC_SIGN_L = 1;
const int ENC_SIGN_R = 1;

// Geometria
const float CPR        = 182.0;
const float WHEEL_MM   = 32.0;
const float TRACK_MM   = 86.0;
const float TURN_FUDGE = 0.96;

// PID
const float KP = 2.0;
const float KI = 0.0;
const float KD = 0.05;

const int MAX_PWM = 100;
const int MIN_PWM = 45;
const int TOL     = 2;

const int   SETTLE_MS  = 150;
const int   TIMEOUT_MS = 3000;
const int   I_ZONE     = 15;
const float I_MAX      = 15.0;
const unsigned long DT_US = 10000;   // lazo de 10 ms

// ---------- Estado ----------
long  nGiro      = 0;
float netaGrados = 0;    // rotacion neta ordenada (derecha +)

// =====================================================
//  MOTORES
// =====================================================
void setMotors(int left, int right) {
  left  = constrain(left,  -255, 255);
  right = constrain(right, -255, 255);
  digitalWrite(L_DIR, left  >= 0 ? HIGH : LOW);
  digitalWrite(R_DIR, right >= 0 ? HIGH : LOW);
  analogWrite(L_PWM, abs(left));
  analogWrite(R_PWM, abs(right));
}

void stopMotors() { setMotors(0, 0); }

// =====================================================
//  PID
// =====================================================
int pidStep(PIDState &s, long target, long pos, float dt) {
  long e = target - pos;

  float vel = (pos - s.prevPos) / dt;
  s.prevPos = pos;
  s.velF = 0.7f * s.velF + 0.3f * vel;

  if (labs(e) <= I_ZONE) {
    s.integ += e * dt;
    s.integ = constrain(s.integ, -I_MAX, I_MAX);
  } else {
    s.integ = 0;
  }

  if (labs(e) <= TOL) return 0;

  float u = KP * e + KI * s.integ - KD * s.velF;
  u = constrain(u, -MAX_PWM, MAX_PWM);

  if (fabs(u) < MIN_PWM) u = (u >= 0) ? MIN_PWM : -MIN_PWM;
  return (int)u;
}

long objetivoCuentas(float grados) {
  return lroundf(CPR * TRACK_MM * fabsf(grados) / (360.0f * WHEEL_MM) * TURN_FUDGE);
}

bool turnPID(bool toRight, long target) {
  stopMotors();
  delay(100);
  resetCounts();

  long tgtL = toRight ?  target : -target;
  long tgtR = toRight ? -target :  target;

  PIDState sL, sR;
  long maxOverL = 0, maxOverR = 0;
  unsigned long t0 = millis();
  unsigned long tLast = micros();
  unsigned long settleStart = 0;
  bool done = false;

  while (millis() - t0 < TIMEOUT_MS) {
    unsigned long now = micros();
    if (now - tLast < DT_US) continue;
    float dt = (now - tLast) / 1e6f;
    tLast = now;

    long posL = ENC_SIGN_L * getL();
    long posR = ENC_SIGN_R * getR();

    int uL = pidStep(sL, tgtL, posL, dt);
    int uR = pidStep(sR, tgtR, posR, dt);
    setMotors(uL, uR);

    long oL = toRight ? (posL - tgtL) : (tgtL - posL);
    long oR = toRight ? (tgtR - posR) : (posR - tgtR);
    if (oL > maxOverL) maxOverL = oL;
    if (oR > maxOverR) maxOverR = oR;

    if (labs(tgtL - posL) <= TOL && labs(tgtR - posR) <= TOL) {
      if (settleStart == 0) settleStart = millis();
      if (millis() - settleStart >= SETTLE_MS) { done = true; break; }
    } else {
      settleStart = 0;
    }
  }
  stopMotors();
  unsigned long dur = millis() - t0;
  delay(250);

  Serial.print("  objetivo=");    Serial.print(target);
  Serial.print("  final L=");     Serial.print(abs(ENC_SIGN_L * getL()));
  Serial.print(" R=");            Serial.print(abs(ENC_SIGN_R * getR()));
  Serial.print("  sobrepaso L="); Serial.print(maxOverL);
  Serial.print(" R=");            Serial.print(maxOverR);
  Serial.print("  t=");           Serial.print(dur);
  Serial.println(done ? " ms OK" : " ms TIMEOUT");
  return done;
}

void imprimirParametros() {
  Serial.println("---- Parametros ----");
  Serial.print("angulo=");    Serial.print(ANGULO_GRADOS);
  Serial.print("  alternar=");Serial.print(ALTERNAR ? 1 : 0);
  Serial.print("  pausa=");   Serial.print(PAUSA_MS); Serial.println(" ms");
  Serial.print("KP=");        Serial.print(KP);
  Serial.print("  KI=");      Serial.print(KI);
  Serial.print("  KD=");      Serial.println(KD);
  Serial.print("CPR=");       Serial.print(CPR);
  Serial.print("  WHEEL=");   Serial.print(WHEEL_MM);
  Serial.print("  TRACK=");   Serial.print(TRACK_MM);
  Serial.print("  FUDGE=");   Serial.println(TURN_FUDGE, 4);
  Serial.print("MIN_PWM=");   Serial.print(MIN_PWM);
  Serial.print("  MAX_PWM="); Serial.print(MAX_PWM);
  Serial.print("  TOL=");     Serial.println(TOL);
  Serial.print("ENC_SIGN L=");Serial.print(ENC_SIGN_L);
  Serial.print("  R=");       Serial.println(ENC_SIGN_R);
  Serial.print("Objetivo por giro = ");
  Serial.print(objetivoCuentas(ANGULO_GRADOS));
  Serial.println(" cuentas por rueda");
  Serial.println("--------------------");
}

// =====================================================
//  SETUP / LOOP
// =====================================================
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

  imprimirParametros();
  Serial.println("Marca la orientacion inicial. Empieza en 5 s...");
  delay(5000);
}

void loop() {
  bool derecha = (ANGULO_GRADOS >= 0);
  if (ALTERNAR && (nGiro % 2 == 1)) derecha = !derecha;

  float grados = fabsf(ANGULO_GRADOS);
  long target = objetivoCuentas(grados);

  nGiro++;
  netaGrados += derecha ? grados : -grados;

  Serial.print("Giro #");   Serial.print(nGiro);
  Serial.print(derecha ? " DERECHA " : " IZQUIERDA ");
  Serial.print(grados);
  Serial.print(" grados | neto ordenado = ");
  Serial.print(netaGrados);
  Serial.println(" grados");

  turnPID(derecha, target);
  delay(PAUSA_MS);
}