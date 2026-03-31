// =====================================================
//  Flying Wing Flight Controller
//  Hardware : Raspberry Pi Pico
//  IMU      : BMI160  — GP0 (SDA), GP1 (SCL)
//  RC input : TBS Crossfire — GP5 (SerialUART)
//  Outputs  : GP2=Throttle  GP3=Left elevon  GP4=Right elevon
//  Arming   : CH8 switch (any position above center)
// =====================================================

#include <Wire.h>
#include <BMI160Gen.h>
#include <Servo.h>
#include <math.h>

#define PIN_THROTTLE  2
#define PIN_ELEV_L    3
#define PIN_ELEV_R    4
#define PIN_CRSF_RX   5
#define BMI160_ADDR   0x68

#define CRSF_BAUDRATE     420000
#define CRSF_ADDR_FC      0xC8
#define CRSF_FRAMETYPE_RC 0x16
#define CRSF_MAX_PACKET   64

uint16_t ch[16] = {992,992,172,992,992,992,992,172,
                   992,992,992,992,992,992,992,992};
uint32_t crsfPackets = 0;

SerialUART crsfSerial(uart1, PIN_CRSF_RX, UART_PIN_NOT_DEFINED);

Servo servoL, servoR, servoThrottle;

struct PID {
  float kP, kI, kD;
  float integral  = 0;
  float lastError = 0;
  float lastD     = 0;   // low-pass filtered derivative
  float iLimit;
  // D-term LPF alpha ~0.45 → ~13 Hz cutoff at 100 Hz loop rate.
  // Keeps enough phase margin for control while rejecting vibration noise.
  float compute(float error, float dt) {
    integral += error * dt;
    integral = constrain(integral, -iLimit, iLimit);
    float d = (error - lastError) / dt;
    lastD = 0.36f * d + 0.64f * lastD;
    lastError = error;
    return kP * error + kI * integral + kD * lastD;
  }
};
PID pidRoll  = {2.5f, 0.05f, 0.8f, 0, 0, 50.0f};
PID pidPitch = {2.5f, 0.05f, 0.8f, 0, 0, 50.0f};

#define CF_ALPHA 0.95f
float cfRoll  = 0.0f;
float cfPitch = 0.0f;

// Gyro low-pass filter state — alpha 0.7 → ~37 Hz cutoff at 100 Hz loop rate.
// Rejects high-frequency vibration before it reaches the CF filter and D-term.
float filtGX = 0.0f, filtGY = 0.0f;

unsigned long lastLoop = 0;
unsigned long lastLog  = 0;
bool armed = false;

// ── CRSF parser ───────────────────────────────────
uint8_t crsfBuf[CRSF_MAX_PACKET];
uint8_t crsfIdx = 0;

// CRSF uses DVB-S2 CRC-8 (poly 0xD5) over [type + payload], not including CRC byte.
static uint8_t crsfCRC8(const uint8_t *buf, uint8_t len) {
  uint8_t crc = 0;
  while (len--) {
    crc ^= *buf++;
    for (uint8_t i = 0; i < 8; i++)
      crc = (crc & 0x80) ? (crc << 1) ^ 0xD5 : crc << 1;
  }
  return crc;
}

void parseCRSF(uint8_t b) {
  if (crsfIdx == 0 && b != CRSF_ADDR_FC) return;
  crsfBuf[crsfIdx++] = b;
  if (crsfIdx < 2) return;
  uint8_t frameLen = crsfBuf[1];
  if (frameLen > CRSF_MAX_PACKET - 2) { crsfIdx = 0; return; }
  if (crsfIdx < (uint8_t)(frameLen + 2)) return;
  // Verify CRC before trusting any data.
  // CRC covers bytes [type .. last payload byte]; CRC byte is at crsfBuf[frameLen+1].
  if (crsfCRC8(&crsfBuf[2], frameLen - 1) != crsfBuf[frameLen + 1]) {
    crsfIdx = 0; return;
  }
  if (crsfBuf[2] == CRSF_FRAMETYPE_RC) {
    uint8_t *p = &crsfBuf[3];
    ch[0]  = ((p[0]       | p[1]  << 8) & 0x07FF);
    ch[1]  = ((p[1]  >> 3 | p[2]  << 5) & 0x07FF);
    ch[2]  = ((p[2]  >> 6 | p[3]  << 2 | p[4]  << 10) & 0x07FF);
    ch[3]  = ((p[4]  >> 1 | p[5]  << 7) & 0x07FF);
    ch[4]  = ((p[5]  >> 4 | p[6]  << 4) & 0x07FF);
    ch[5]  = ((p[6]  >> 7 | p[7]  << 1 | p[8]  << 9)  & 0x07FF);
    ch[6]  = ((p[8]  >> 2 | p[9]  << 6) & 0x07FF);
    ch[7]  = ((p[9]  >> 5 | p[10] << 3) & 0x07FF);
    ch[8]  = ((p[11]      | p[12] << 8) & 0x07FF);
    ch[9]  = ((p[12] >> 3 | p[13] << 5) & 0x07FF);
    ch[10] = ((p[13] >> 6 | p[14] << 2 | p[15] << 10) & 0x07FF);
    ch[11] = ((p[15] >> 1 | p[16] << 7) & 0x07FF);
    ch[12] = ((p[16] >> 4 | p[17] << 4) & 0x07FF);
    ch[13] = ((p[17] >> 7 | p[18] << 1 | p[19] << 9)  & 0x07FF);
    ch[14] = ((p[19] >> 2 | p[20] << 6) & 0x07FF);
    ch[15] = ((p[20] >> 5 | p[21] << 3) & 0x07FF);
    crsfPackets++;
  }
  crsfIdx = 0;
}

float crsfToFloat(uint16_t raw) {
  return constrain((raw - 992.0f) / 819.0f, -1.0f, 1.0f);
}
float crsfToThrottle(uint16_t raw) {
  return constrain((raw - 172.0f) / 1639.0f, 0.0f, 1.0f);
}
void writeServo(Servo &s, float val) {
  s.writeMicroseconds(constrain((int)(1500 + val * 500.0f), 1000, 2000));
}
void writeThrottle(float val) {
  servoThrottle.writeMicroseconds(constrain((int)(1000 + val * 1000.0f), 1000, 2000));
}

// ═════════════════════════════════════════════════
void setup() {
  Serial.begin(921600);
  // Do NOT delay here — a long blocking delay creates a ~6s reset loop
  // because Arduino IDE serial monitor triggers a USB CDC reset on RP2040,
  // then the sketch restarts and delays again indefinitely.

  Serial.println("Flying Wing — CH8 arm");

  Wire.setSDA(0);
  Wire.setSCL(1);
  Wire.begin();
  Wire.setTimeout(5); // 5 ms timeout so I2C never stalls the loop
  BMI160.begin(BMI160GenClass::I2C_MODE, Wire, BMI160_ADDR);
  delay(300);
  Wire.beginTransmission(BMI160_ADDR);
  Wire.write(0x00);
  Wire.endTransmission();
  Wire.requestFrom(BMI160_ADDR, 1);
  if (Wire.read() != 0xD1) { Serial.println("BMI160 ERR"); while(1); }
  BMI160.setAccelerometerRange(4);
  BMI160.setGyroRange(500);
  Serial.println("BMI160 OK");

  crsfSerial.begin(CRSF_BAUDRATE);
  Serial.println("CRSF OK");

  servoThrottle.attach(PIN_THROTTLE, 1000, 2000);
  servoL.attach(PIN_ELEV_L, 1000, 2000);
  servoR.attach(PIN_ELEV_R, 1000, 2000);
  servoThrottle.writeMicroseconds(1000);
  servoL.writeMicroseconds(1500);
  servoR.writeMicroseconds(1500);
  // Hold low throttle for 2s so the ESC can complete its arming sequence.
  // This is safe because it is a short fixed delay, not a USB-triggered loop.
  delay(2000);
  Serial.println("Servos OK");
  Serial.println("CH8 UP to arm (throttle low)");

  lastLoop = micros();
  lastLog  = millis();
}

// ═════════════════════════════════════════════════
void loop() {

  while (crsfSerial.available()) parseCRSF(crsfSerial.read());

  unsigned long now = micros();
  if (now - lastLoop < 10000UL) return;
  float dt = constrain((now - lastLoop) / 1000000.0f, 0.001f, 0.05f);
  lastLoop = now;

  // ── IMU ───────────────────────────────────────
  int rawAX, rawAY, rawAZ, rawGX, rawGY, rawGZ;
  BMI160.readAccelerometer(rawAX, rawAY, rawAZ);
  BMI160.readGyro(rawGX, rawGY, rawGZ);

  float aX = rawAX * 4.0f / 32768.0f;
  float aY = rawAY * 4.0f / 32768.0f;
  float aZ = rawAZ * 4.0f / 32768.0f;
  float gX = rawGX * 500.0f / 32768.0f;
  float gY = rawGY * 500.0f / 32768.0f;

  // Gyro LPF — reject vibration noise before it reaches CF filter and D-term
  filtGX = 0.76f * filtGX + 0.24f * gX;
  filtGY = 0.76f * filtGY + 0.24f * gY;

  // Accel-only angle — stable reference when still
  float accelRoll  = atan2f(aY, aZ) * 180.0f / M_PI;
  float accelPitch = atan2f(-aX, sqrtf(aY*aY + aZ*aZ)) * 180.0f / M_PI;

  // Use lower alpha (0.8) so accel dominates more — stops gyro drift
  // causing wild pitch oscillation when sitting still
  const float alpha = 0.80f;
  cfRoll  = alpha * (cfRoll  + filtGX * dt) + (1.0f - alpha) * accelRoll;
  cfPitch = alpha * (cfPitch + filtGY * dt) + (1.0f - alpha) * accelPitch;

  // ── RC ────────────────────────────────────────
  // Log all 8 channels so we can identify which stick = which channel
  float rcRoll     = crsfToFloat(ch[0]);   // CH1
  float rcPitch    = crsfToFloat(ch[1]);   // CH2
  float rcThrottle = crsfToThrottle(ch[2]); // CH3
  float rcYaw      = crsfToFloat(ch[3]);   // CH4

  // ── Arming — CH8 switch ───────────────────────
  bool switchOn = (ch[7] > 650);
  if (!armed &&  switchOn) armed = true;
  if ( armed && !switchOn) armed = false;

  float elevL, elevR;

  if (armed) {
    // Stabilised — PID corrects for attitude
    float corrRoll  = pidRoll.compute (rcRoll  * 30.0f - cfRoll,  dt);
    float corrPitch = pidPitch.compute(rcPitch * 30.0f - cfPitch, dt);
    elevL = constrain(corrPitch + corrRoll,  -100.0f, 100.0f) / 100.0f;
    elevR = constrain(corrPitch - corrRoll,  -100.0f, 100.0f) / 100.0f;
  } else {
    // Pass-through — direct stick to elevons, no stabilisation
    elevL = constrain(rcPitch + rcRoll, -1.0f, 1.0f);
    elevR = constrain(rcPitch - rcRoll, -1.0f, 1.0f);
    // Reset PID so no jolt when arming
    pidRoll.integral  = 0; pidRoll.lastError  = 0; pidRoll.lastD  = 0;
    pidPitch.integral = 0; pidPitch.lastError = 0; pidPitch.lastD = 0;
  }

  // Always output
  writeThrottle(rcThrottle);
  writeServo(servoL, -elevL);
  writeServo(servoR, elevR);

  // ── Log at 2 Hz — only if serial buffer has room ─
  if (millis() - lastLog >= 500 && Serial.availableForWrite() > 100) {
    lastLog = millis();
    // Print all 8 raw channels so you can identify stick mapping
    char buf[160];
    int n = sprintf(buf,
      "%s THR:%d%% CH1:%d CH2:%d CH3:%d CH4:%d CH8:%d | Roll:%.1f Pitch:%.1f | EL:%.2f ER:%.2f\r\n",
      armed ? "ARMED  " : "DISARMD",
      (int)(rcThrottle * 100),
      (int)ch[0], (int)ch[1], (int)ch[2], (int)ch[3], (int)ch[7],
      cfRoll, cfPitch,
      elevL, elevR);
    Serial.write((uint8_t*)buf, n);
  }
}
