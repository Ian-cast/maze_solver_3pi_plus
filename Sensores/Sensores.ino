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

// ---------- Boton que dispara la accion ----------
// A = 25 (comprobado). B no se puede leer con digitalRead.
const int BTN_ACTION = 25;

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

const int STUCK_CYCLES = 5;
const int BOOST_STEP   = 4;
const int BOOST_MAX    = 30;

// =====================================================
//  PARAMETROS (los mismos que el laberinto)
// =====================================================
const float GIRO_GRADOS = 90.0;
const float GIRO_VUELTA = 180.0;

const int BASE_SPEED     = 80;
const int APPROACH_SPEED = 60;

const int WALL_BELOW = 170;
const int OPEN_ABOVE = 260;
const int FRONT_STOP = 125;
const int SLOW_DIST  = 170;
const int FRONT_TURN_MIN = 70;

const int REF_RIGHT = 103;
const int REF_LEFT  = 115;

const int   WALL_SIGN    = 1;
const float KP_LAT       = 0.30;
const int   LAT_DEAD     = 6;
const int   YAW_MAX      = 10;
const float YAW_SLEW     = 0.05;
const float KP_YAW       = 2.0;
const int   CORR_MAX     = 14;
const int   FREEZE_FRONT = 220;

const int BRAKE_PWM = 70;
const int BRAKE_MS  = 20;

const int ADVANCE_MM = 100;
const int ENTER_MM   = 30;

const unsigned long LOOP_US = 5000;

const uint16_t INVALID_MM  = 1500;
const uint8_t  INVALID_MAX = 4;
const uint16_t FAR_MM      = 1200;

// =====================================================
//  PARAMETROS DE LA PRUEBA
// =====================================================
const bool FRENTE_PRIMERO   = true;   // true: si el frente esta libre, avanza
                                       // false: orden del laberinto (derecha libre primero)
const int  AVANCE_PRUEBA_MS = 1000;    // cuanto avanza en la accion "avanzar"
const int  ESPERA_TRAS_BOTON_MS = 800; // tiempo para quitar el dedo antes de moverse
const int  PRINT_MS         = 150;     // cada cuanto se imprimen las lecturas

// Acciones
const int ACC_AVANZAR = 0;
const int ACC_DERECHA = 1;
const int ACC_IZQUIERDA = 2;
const int ACC_VUELTA = 3;

// ---------- Sensores ----------
const uint8_t MUX_ADDR = 0x70;
const uint8_t N = 5;
enum { S_LEFT = 4, S_FL = 3, S_FRONT = 2, S_FR = 1, S_RIGHT = 0 };
VL53L0X sensor[N];
uint16_t d[N];
uint16_t hist[N][3];
uint8_t  hIdx[N];
uint8_t  invCnt[N];
bool wallR = true, wallL = true;
bool leerDiagonales = true;            // true en reposo (muestra los 5), false al moverse

// ---------- Estado ----------
float mmDesdeGiro = 0;
long  yawBase = 0;
float yawTarget = 0;
float eFilt = 0;
int   modoPrev = -1;
unsigned long lastPrint = 0;

// =====================================================
//  MULTIPLEXOR Y SENSORES
// =====================================================
void canal(uint8_t ch) {
  Wire.beginTransmission(MUX_ADDR);
  Wire.write(1 << ch);
  Wire.endTransmission();
}

bool sensorUsado(uint8_t i) {
  return leerDiagonales || (i != S_FL && i != S_FR);
}

uint16_t mediana3(uint16_t a, uint16_t b, uint16_t c) {
  if (a > b) { uint16_t t = a; a = b; b = t; }
  if (b > c) { b = c; }
  return (a > b) ? a : b;
}

void guardarLectura(uint8_t i, uint16_t v) {
  if (v >= INVALID_MM) {
    if (invCnt[i] < 255) invCnt[i]++;
    if (invCnt[i] < INVALID_MAX) return;
    v = FAR_MM;
  } else {
    invCnt[i] = 0;
  }
  hist[i][hIdx[i]] = v;
  hIdx[i] = (hIdx[i] + 1) % 3;
  d[i] = mediana3(hist[i][0], hist[i][1], hist[i][2]);
}

void actualizarParedes() {
  if (d[S_RIGHT] < WALL_BELOW) wallR = true;
  else if (d[S_RIGHT] > OPEN_ABOVE) wallR = false;
  if (d[S_LEFT] < WALL_BELOW) wallL = true;
  else if (d[S_LEFT] > OPEN_ABOVE) wallL = false;
}

void readSensors() {
  for (uint8_t i = 0; i < N; i++) {
    if (!sensorUsado(i)) continue;
    canal(i);
    if (sensor[i].readReg(0x13) & 0x07) {
      guardarLectura(i, sensor[i].readRangeContinuousMillimeters());
    }
  }
  actualizarParedes();
}

void readSensorsBlocking() {
  for (uint8_t i = 0; i < N; i++) {
    if (!sensorUsado(i)) continue;
    canal(i);
    guardarLectura(i, sensor[i].readRangeContinuousMillimeters());
  }
  actualizarParedes();
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

void frenarMotores() {
  if (BRAKE_MS > 0) {
    setMotors(-BRAKE_PWM, -BRAKE_PWM);
    delay(BRAKE_MS);
  }
  stopMotors();
}

void avance(int v, int corr) {
  int l = max(v + corr, MIN_PWM);
  int r = max(v - corr, MIN_PWM);
  setMotors(l, r);
}

// =====================================================
//  PID Y GIROS
// =====================================================
int pidStep(PIDState &s, long target, long pos, float dt, int boost) {
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

  int minP = MIN_PWM + boost;
  if (fabs(u) < minP) u = (u >= 0) ? minP : -minP;
  return (int)u;
}

void actualizarBoost(long pos, long tgt, long &last, int &stuck, int &boost) {
  if (labs(tgt - pos) <= TOL) {
    boost = 0;
    stuck = 0;
  } else if (pos == last) {
    if (++stuck >= STUCK_CYCLES) {
      boost = min(boost + BOOST_STEP, BOOST_MAX);
      stuck = 0;
    }
  } else {
    stuck = 0;
    if (boost > 0) boost -= 1;
  }
  last = pos;
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
  int  boostL = 0, boostR = 0, stuckL = 0, stuckR = 0;
  long lastL = 0, lastR = 0;
  int  boostMax = 0;
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

    actualizarBoost(posL, tgtL, lastL, stuckL, boostL);
    actualizarBoost(posR, tgtR, lastR, stuckR, boostR);
    boostMax = max(boostMax, max(boostL, boostR));

    int uL = pidStep(sL, tgtL, posL, dt, boostL);
    int uR = pidStep(sR, tgtR, posR, dt, boostR);
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

  Serial.print("  giro objetivo=");  Serial.print(target);
  Serial.print("  final L=");        Serial.print(abs(ENC_SIGN_L * getL()));
  Serial.print(" R=");               Serial.print(abs(ENC_SIGN_R * getR()));
  Serial.print("  refuerzo max=");   Serial.print(boostMax);
  Serial.println(done ? "  OK" : "  TIMEOUT");
  return done;
}

void girar(float grados) {
  bool derecha = (grados >= 0);
  turnPID(derecha, objetivoCuentas(grados));
  delay(50);
  for (int k = 0; k < 3; k++) readSensorsBlocking();
  mmDesdeGiro = 0;
  yawBase = 0;
  yawTarget = 0;
  eFilt = 0;
  modoPrev = -1;
}

// =====================================================
//  AVANCE RECTO CON CENTRADO EN CASCADA
// =====================================================
void actualizarRumboObjetivo(bool activo) {
  float deseado = 0;

  if (activo && (wallR || wallL)) {
    float e;
    int modo;
    if (wallR && wallL) {
      modo = 0;
      e = (((int)d[S_RIGHT] - REF_RIGHT) - ((int)d[S_LEFT] - REF_LEFT)) / 2.0f;
    } else if (wallR) {
      modo = 1;
      e = (int)d[S_RIGHT] - REF_RIGHT;
    } else {
      modo = 2;
      e = REF_LEFT - (int)d[S_LEFT];
    }
    e = constrain(e, -60.0f, 60.0f);

    if (modo != modoPrev) {
      eFilt = e;
      modoPrev = modo;
    } else {
      eFilt = 0.8f * eFilt + 0.2f * e;
    }

    float a = fabsf(eFilt);
    if (a > LAT_DEAD) {
      deseado = KP_LAT * (a - LAT_DEAD) * ((eFilt > 0) ? 1.0f : -1.0f);
    }
    deseado *= WALL_SIGN;
    deseado = constrain(deseado, -(float)YAW_MAX, (float)YAW_MAX);
  } else {
    modoPrev = -1;
  }

  if (deseado > yawTarget + YAW_SLEW)      yawTarget += YAW_SLEW;
  else if (deseado < yawTarget - YAW_SLEW) yawTarget -= YAW_SLEW;
  else                                      yawTarget = deseado;
}

int velocidadAvance() {
  int f = (int)d[S_FRONT];
  if (f >= SLOW_DIST)  return BASE_SPEED;
  if (f <= FRONT_STOP) return APPROACH_SPEED;
  return (int)map(f, FRONT_STOP, SLOW_DIST, APPROACH_SPEED, BASE_SPEED);
}

// Avanza hasta 'mm' milimetros o hasta que pasen 'maxMs' milisegundos.
void avanzarMm(int mm, bool frenar, bool centrar, unsigned long maxMs) {
  long target = lroundf(mm * COUNTS_PER_MM);
  resetCounts();
  unsigned long t0 = millis();
  unsigned long tLast = micros();
  bool bloqueado = false;

  while (millis() - t0 < maxMs) {
    unsigned long now = micros();
    if (now - tLast < LOOP_US) continue;
    tLast = now;

    long posL = ENC_SIGN_L * getL();
    long posR = ENC_SIGN_R * getR();
    if ((posL + posR) / 2 >= target) break;

    readSensors();
    if (d[S_FRONT] < FRONT_STOP) { bloqueado = true; break; }

    long yaw = yawBase + (posL - posR);
    actualizarRumboObjetivo(centrar && (int)d[S_FRONT] > FREEZE_FRONT);

    float c = KP_YAW * (yawTarget - yaw);
    int corr = (int)constrain(c, -(float)CORR_MAX, (float)CORR_MAX);
    avance(velocidadAvance(), corr);
  }

  long posL = ENC_SIGN_L * getL();
  long posR = ENC_SIGN_R * getR();
  yawBase += (posL - posR);
  mmDesdeGiro += ((posL + posR) / 2) / COUNTS_PER_MM;

  if (frenar || bloqueado) frenarMotores();
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

void acomodarParaGiro() {
  stopMotors();
  delay(120);
  for (int k = 0; k < 3; k++) readSensorsBlocking();
  Serial.print("frente al detenerse = ");
  Serial.println(d[S_FRONT]);
  if ((int)d[S_FRONT] < FRONT_TURN_MIN) {
    retrocederMm(FRONT_TURN_MIN - (int)d[S_FRONT] + 10);
  }
}

// =====================================================
//  DECISION Y ACCION
// =====================================================
int decidir() {
  bool derechaLibre = !wallR;
  bool izqLibre     = !wallL;
  bool frenteBloq   = d[S_FRONT] < FRONT_STOP;

  if (FRENTE_PRIMERO) {
    if (!frenteBloq)  return ACC_AVANZAR;
    if (derechaLibre) return ACC_DERECHA;
    if (izqLibre)     return ACC_IZQUIERDA;
    return ACC_VUELTA;
  } else {
    if (derechaLibre) return ACC_DERECHA;
    if (!frenteBloq)  return ACC_AVANZAR;
    if (izqLibre)     return ACC_IZQUIERDA;
    return ACC_VUELTA;
  }
}

const char* nombreAccion(int a) {
  switch (a) {
    case ACC_AVANZAR:   return "AVANZAR 1 s";
    case ACC_DERECHA:   return "GIRO DERECHA 90";
    case ACC_IZQUIERDA: return "GIRO IZQUIERDA 90";
    default:            return "VUELTA 180";
  }
}

void ejecutarAccion(int accion) {
  leerDiagonales = false;          // al moverse, solo se leen los 3 sensores del laberinto

  // Estado limpio, como despues de un giro
  yawBase = 0;
  yawTarget = 0;
  eFilt = 0;
  modoPrev = -1;
  mmDesdeGiro = 1000;              // evita el bloqueo por "giro reciente"

  switch (accion) {
    case ACC_AVANZAR:
      avanzarMm(10000, true, true, AVANCE_PRUEBA_MS);
      break;
    case ACC_DERECHA:
      avanzarMm(ADVANCE_MM, true, false, 4000);
      acomodarParaGiro();
      girar(GIRO_GRADOS);
      avanzarMm(ENTER_MM, true, false, 4000);
      break;
    case ACC_IZQUIERDA:
      acomodarParaGiro();
      girar(-GIRO_GRADOS);
      break;
    default:
      acomodarParaGiro();
      girar(GIRO_VUELTA);
      break;
  }

  stopMotors();
  leerDiagonales = true;
  delay(500);
  for (int k = 0; k < 3; k++) readSensorsBlocking();
}

// =====================================================
//  BOTON
// =====================================================
bool botonPulsado() {
  static bool prev = false;
  bool ahora = (digitalRead(BTN_ACTION) == LOW);
  bool flanco = ahora && !prev;
  prev = ahora;
  if (flanco) delay(30);           // antirrebote
  return flanco;
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

  pinMode(BTN_ACTION, INPUT_PULLUP);

  Wire.begin();
  Wire.setClock(400000);

  for (uint8_t i = 0; i < N; i++) {
    d[i] = FAR_MM;
    hIdx[i] = 0;
    invCnt[i] = 0;
    for (uint8_t k = 0; k < 3; k++) hist[i][k] = FAR_MM;

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

  for (int k = 0; k < 3; k++) readSensorsBlocking();

  Serial.println("Prueba de situaciones. Muestra lecturas y la accion que haria.");
  Serial.println("Pulsa el boton para ejecutar la accion.");
  Serial.print("FRENTE_PRIMERO = ");
  Serial.println(FRENTE_PRIMERO ? "true" : "false");
}

void loop() {
  readSensorsBlocking();

  int accion = decidir();

  if (millis() - lastPrint > PRINT_MS) {
    lastPrint = millis();
    for (uint8_t i = 0; i < N; i++) {
      Serial.print("C");
      Serial.print(i);
      Serial.print(": ");
      Serial.print(d[i]);
      Serial.print("\t");
    }
    Serial.print("| paredD=");  Serial.print(wallR);
    Serial.print(" paredI=");    Serial.print(wallL);
    Serial.print(" frente=");    Serial.print(d[S_FRONT] < FRONT_STOP ? "BLOQ" : "libre");
    Serial.print(" -> ");
    Serial.println(nombreAccion(accion));
  }

  if (botonPulsado()) {
    Serial.println();
    Serial.print(">> EJECUTANDO: ");
    Serial.println(nombreAccion(accion));
    delay(ESPERA_TRAS_BOTON_MS);          // tiempo para quitar el dedo
    ejecutarAccion(accion);
    Serial.println(">> LISTO. Mueve el robot a otra situacion y vuelve a pulsar.");
    Serial.println();
  }
}