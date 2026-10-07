#include <Wire.h>
#include <VL53L0X.h>

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

// ---------- Boton A (GP25, comparte pin con el LED amarillo) ----------
const int BTN_A = 25;

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
//  GEOMETRIA (calibrada)
// =====================================================
const int ENC_SIGN_L = 1;
const int ENC_SIGN_R = 1;

const float CPR        = 182.0;
const float WHEEL_MM   = 32.0;
const float TRACK_MM   = 86.0;
const float TURN_FUDGE = 0.96;

const float COUNTS_PER_MM = CPR / (PI * WHEEL_MM);

// =====================================================
//  PID DE GIRO
// =====================================================
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
const unsigned long DT_US = 10000;

// =====================================================
//  PARAMETROS DEL LABERINTO
// =====================================================
const float GIRO_GRADOS = 90.0;    // giro lateral
const float GIRO_VUELTA = 180.0;   // callejon sin salida

const int   BASE_SPEED = 80;       // velocidad al avanzar (0-255)
const float KP_WALL    = 0.0;      // correccion con paredes (sube a 0.2 cuando el frenado este bien)
const float KP_SYNC    = 2.0;      // correccion con encoders (sin paredes)
const int   CORR_MAX   = 30;       // limite de correccion (PWM)
const int   DEADBAND   = 3;        // zona muerta de la correccion (mm)

const int REF_SIDE   = 110;        // distancia a UNA pared estando centrado (mm)
const int FRONT_STOP = 100;        // frente a menos de esto = cerrado (mm)
const int OPEN_SIDE  = 160;        // lado a mas de esto = abertura (mm)

// Frenado al acercarse a una pared de frente
const int SLOW_DIST      = 250;    // empieza a frenar a esta distancia (mm)
const int APPROACH_SPEED = 55;     // velocidad minima al acercarse (debe superar la zona muerta)
const int FRONT_TURN_MIN = 70;     // si el frente esta a menos de esto antes de girar, retrocede (mm)

const int STEP_MM      = 20;       // avance por ciclo de decision
const int ADVANCE_MM   = 20;       // avance antes de girar en una abertura
const int ENTER_MM     = 30;       // avance despues de girar
const int MIN_MM_ENTRE_GIROS = 60; // distancia minima entre giros a la derecha

const bool DEBUG = true;

// ---------- Sensores ----------
const uint8_t MUX_ADDR = 0x70;
const uint8_t N = 5;
enum { S_LEFT = 4, S_FL = 3, S_FRONT = 2, S_FR = 1, S_RIGHT = 0 };
VL53L0X sensor[N];
uint16_t d[N];

// ---------- Estado ----------
float mmDesdeGiro = 0;
int   lastCorr = 0;
unsigned long lastPrint = 0;

// =====================================================
//  MULTIPLEXOR Y SENSORES
// =====================================================
void canal(uint8_t ch) {
  Wire.beginTransmission(MUX_ADDR);
  Wire.write(1 << ch);
  Wire.endTransmission();
}

void readSensors() {
  for (uint8_t i = 0; i < N; i++) {
    canal(i);
    uint16_t mm = sensor[i].readRangeContinuousMillimeters();
    if (!sensor[i].timeoutOccurred()) d[i] = mm;   // si hay timeout, conserva el valor anterior
  }
}

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
//  PID Y GIROS
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

    if (labs(tgtL - posL) <= TOL && labs(tgtR - posR) <= TOL) {
      if (settleStart == 0) settleStart = millis();
      if (millis() - settleStart >= SETTLE_MS) { done = true; break; }
    } else {
      settleStart = 0;
    }
  }
  stopMotors();
  delay(250);

  if (DEBUG) {
    Serial.print("  giro objetivo=");  Serial.print(target);
    Serial.print("  final L=");        Serial.print(abs(ENC_SIGN_L * getL()));
    Serial.print(" R=");               Serial.print(abs(ENC_SIGN_R * getR()));
    Serial.println(done ? "  OK" : "  TIMEOUT");
  }
  return done;
}

// grados positivos = derecha, negativos = izquierda
void girar(float grados) {
  bool derecha = (grados >= 0);
  turnPID(derecha, objetivoCuentas(grados));
  delay(50);
  readSensors();
  readSensors();
  mmDesdeGiro = 0;
}

// =====================================================
//  AVANCE RECTO
// =====================================================
// Devuelve la correccion de direccion (positivo = girar a la derecha).
//  - Dos paredes: iguala las distancias (sin referencias).
//  - Una pared: mantiene REF_SIDE.
//  - Sin paredes: va recto con los encoders.
int correccionLateral(long errEnc) {
  bool paredD = d[S_RIGHT] < OPEN_SIDE;
  bool paredI = d[S_LEFT]  < OPEN_SIDE;
  int corr;

  if (paredD && paredI) {
    // positivo = mas espacio a la derecha -> girar a la derecha
    int e = ((int)d[S_RIGHT] - (int)d[S_LEFT]) / 2;
    e = constrain(e, -40, 40);
    if (abs(e) <= DEADBAND) e = 0;
    corr = (int)(KP_WALL * e);
  } else if (paredD) {
    int e = constrain((int)d[S_RIGHT] - REF_SIDE, -40, 40);   // positivo = lejos de la derecha
    if (abs(e) <= DEADBAND) e = 0;
    corr = (int)(KP_WALL * e);
  } else if (paredI) {
    int e = constrain((int)d[S_LEFT] - REF_SIDE, -40, 40);    // positivo = lejos de la izquierda
    if (abs(e) <= DEADBAND) e = 0;
    corr = -(int)(KP_WALL * e);
  } else {
    corr = -(int)(KP_SYNC * errEnc);   // izquierda adelantada -> frena la izquierda
  }

  lastCorr = constrain(corr, -CORR_MAX, CORR_MAX);
  return lastCorr;
}

// Reduce la velocidad al acercarse a una pared de frente
int velocidadAvance() {
  int f = (int)d[S_FRONT];
  if (f >= SLOW_DIST)  return BASE_SPEED;
  if (f <= FRONT_STOP) return APPROACH_SPEED;
  return (int)map(f, FRONT_STOP, SLOW_DIST, APPROACH_SPEED, BASE_SPEED);
}

// Avanza 'mm' milimetros. Si frenar es false, deja los motores andando al terminar
// (salvo que haya pared al frente: en ese caso frena de inmediato).
void avanzarMm(int mm, bool frenar) {
  long target = lroundf(mm * COUNTS_PER_MM);
  resetCounts();
  unsigned long t0 = millis();
  bool bloqueado = false;

  while (millis() - t0 < 4000) {
    long posL = ENC_SIGN_L * getL();
    long posR = ENC_SIGN_R * getR();
    if ((posL + posR) / 2 >= target) break;

    readSensors();
    if (d[S_FRONT] < FRONT_STOP) { bloqueado = true; break; }

    int corr = correccionLateral(posL - posR);
    int v = velocidadAvance();
    setMotors(v + corr, v - corr);
  }

  long recorrido = (ENC_SIGN_L * getL() + ENC_SIGN_R * getR()) / 2;
  mmDesdeGiro += recorrido / COUNTS_PER_MM;

  if (frenar || bloqueado) stopMotors();
}

void retrocederMm(int mm) {
  long target = lroundf(mm * COUNTS_PER_MM);
  resetCounts();
  unsigned long t0 = millis();
  while (millis() - t0 < 2000) {
    long recorrido = -(ENC_SIGN_L * getL() + ENC_SIGN_R * getR()) / 2;
    if (recorrido >= target) break;
    setMotors(-APPROACH_SPEED, -APPROACH_SPEED);
  }
  stopMotors();
  delay(100);
}

// Frena, mide y, si esta muy pegado a la pared, retrocede para tener espacio de giro
void acomodarParaGiro() {
  stopMotors();
  delay(150);
  readSensors();
  readSensors();
  if (DEBUG) {
    Serial.print("frente al detenerse = ");
    Serial.println(d[S_FRONT]);
  }
  if ((int)d[S_FRONT] < FRONT_TURN_MIN) {
    retrocederMm(FRONT_TURN_MIN - (int)d[S_FRONT] + 10);
  }
}

// =====================================================
//  BOTON A
// =====================================================
void esperarBotonA() {
  pinMode(BTN_A, INPUT_PULLUP);
  Serial.print("Estado del boton A sin presionar = ");
  Serial.println(digitalRead(BTN_A));       // debe ser 1 (HIGH)
  Serial.println("Presiona el boton A para iniciar...");

  while (digitalRead(BTN_A) == HIGH) delay(10);   // esperar a que lo presionen
  delay(30);                                       // antirrebote
  while (digitalRead(BTN_A) == LOW) delay(10);     // esperar a que lo suelten
  Serial.println("Iniciando en 1.5 s...");
  delay(1500);                                     // tiempo para quitar la mano
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

  Wire.begin();
  for (uint8_t i = 0; i < N; i++) {
    d[i] = 8190;
    canal(i);
    sensor[i].setTimeout(200);
    if (!sensor[i].init()) {
      Serial.print("Sensor del canal ");
      Serial.print(i);
      Serial.println(" FALLO. Detenido.");
      while (true) delay(1000);
    }
    sensor[i].setMeasurementTimingBudget(20000);
    sensor[i].startContinuous();
  }

  Serial.print("Objetivo giro 90 = ");  Serial.print(objetivoCuentas(GIRO_GRADOS));
  Serial.print("   giro 180 = ");       Serial.println(objetivoCuentas(GIRO_VUELTA));

  esperarBotonA();
}

void loop() {
  readSensors();

  bool derechaLibre = d[S_RIGHT] > OPEN_SIDE;
  bool frenteBloq   = d[S_FRONT] < FRONT_STOP;
  bool izqLibre     = d[S_LEFT]  > OPEN_SIDE;
  bool puedeGirar   = mmDesdeGiro > MIN_MM_ENTRE_GIROS;

  if (derechaLibre && (puedeGirar || frenteBloq)) {
    avanzarMm(ADVANCE_MM, true);     // pasar la abertura con el eje de las ruedas
    acomodarParaGiro();
    girar(GIRO_GRADOS);              // 90 grados a la derecha
    avanzarMm(ENTER_MM, false);      // entrar al nuevo pasillo
  }
  else if (!frenteBloq) {
    avanzarMm(STEP_MM, false);       // seguir recto
  }
  else if (izqLibre) {
    acomodarParaGiro();
    girar(-GIRO_GRADOS);             // 90 grados a la izquierda
  }
  else {
    acomodarParaGiro();
    girar(GIRO_VUELTA);              // callejon sin salida: 180 grados
  }

  if (DEBUG && millis() - lastPrint > 200) {
    lastPrint = millis();
    for (uint8_t i = 0; i < N; i++) {
      Serial.print("C");
      Serial.print(i);
      Serial.print(": ");
      Serial.print(d[i]);
      Serial.print("\t");
    }
    Serial.print("corr=");
    Serial.println(lastCorr);
  }
}