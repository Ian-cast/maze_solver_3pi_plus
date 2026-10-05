#include <Wire.h>
#include <VL53L0X.h>

const uint8_t MUX_ADDR = 0x70;
const uint8_t N = 5;                 // canales 0 a 4

VL53L0X sensor[N];
bool ok[N];

void canal(uint8_t ch) {
  Wire.beginTransmission(MUX_ADDR);
  Wire.write(1 << ch);
  Wire.endTransmission();
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}
  Wire.begin();

  for (uint8_t i = 0; i < N; i++) {
    canal(i);
    sensor[i].setTimeout(200);
    ok[i] = sensor[i].init();
    if (ok[i]) sensor[i].startContinuous();
    Serial.print("Canal ");
    Serial.print(i);
    Serial.println(ok[i] ? ": OK" : ": FALLO");
  }
}

void loop() {
  for (uint8_t i = 0; i < N; i++) {
    Serial.print("C");
    Serial.print(i);
    Serial.print(": ");
    if (!ok[i]) {
      Serial.print("---");
    } else {
      canal(i);
      uint16_t mm = sensor[i].readRangeContinuousMillimeters();
      if (sensor[i].timeoutOccurred()) Serial.print("TIMEOUT");
      else Serial.print(mm);
    }
    Serial.print("\t");
  }
  Serial.println();
  delay(50);
}