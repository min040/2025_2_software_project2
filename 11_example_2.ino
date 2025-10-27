#include <Servo.h>

// Arduino pin assignment
#define PIN_LED   9    // LED active-low
#define PIN_TRIG  12   // sonar sensor TRIGGER
#define PIN_ECHO  13   // sonar sensor ECHO
#define PIN_SERVO 10   // servo motor

// configurable parameters for sonar
#define SND_VEL 346.0      // sound velocity at 24°C (m/s)
#define INTERVAL 25        // sampling interval (ms)
#define PULSE_DURATION 10  // ultra-sound pulse duration (us)
#define _DIST_MIN 180.0    // min distance (mm)  == 18 cm
#define _DIST_MAX 360.0    // max distance (mm)  == 36 cm

#define TIMEOUT ((INTERVAL / 2) * 1000.0)   // echo timeout (us)
#define SCALE (0.001 * 0.5 * SND_VEL)       // duration->distance (mm)

/* ---------- Filters ---------- */
#define _EMA_ALPHA 0.30     // 0~1, 1이면 EMA 미적용(샘플 그대로)

/* ---------- Target window (for LED) ---------- */
#define _TARGET_LOW  250.0  // mm  (예: 25.0 cm)
#define _TARGET_HIGH 290.0  // mm  (예: 29.0 cm)

/* ---------- Servo microseconds (모터마다 튜닝 필요) ---------- */
#define _DUTY_MIN  800   // 0°
#define _DUTY_NEU 1500   // 90°
#define _DUTY_MAX 2200   // 180°

/* ---------- Globals ---------- */
float dist_prev = _DIST_MAX;    // last valid raw (mm)
float dist_ema  = _DIST_MAX;    // EMA distance (mm)
unsigned long last_sampling_time = 0;

Servo myservo;

/* ---------- Helpers ---------- */
static inline int deg_to_us(float deg) {
  // 0~180° -> microseconds
  deg = constrain(deg, 0.0, 180.0);
  return (int)(_DUTY_MIN + (deg / 180.0f) * (_DUTY_MAX - _DUTY_MIN));
}

float USS_measure(int TRIG, int ECHO) {
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(PULSE_DURATION);
  digitalWrite(TRIG, LOW);
  return pulseIn(ECHO, HIGH, TIMEOUT) * SCALE;  // mm
}

void setup() {
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  digitalWrite(PIN_TRIG, LOW);

  myservo.attach(PIN_SERVO);
  myservo.writeMicroseconds(_DUTY_NEU);  // start at 90°

  Serial.begin(57600);
  last_sampling_time = millis();
}

void loop() {
  // wait until next sampling time
  if (millis() - last_sampling_time < INTERVAL) return;
  last_sampling_time += INTERVAL;

  /* ---------- 1) 초음파 읽기 ---------- */
  float dist_raw = USS_measure(PIN_TRIG, PIN_ECHO);   // mm

  /* ---------- 2) 범위 필터 (18~36 cm만 유효) ---------- */
  float dist_filtered;
  if (dist_raw == 0.0 || dist_raw < _DIST_MIN || dist_raw > _DIST_MAX) {
    // 튀는 샘플/범위 밖이면 직전 값 유지
    dist_filtered = dist_prev;
  } else {
    dist_filtered = dist_raw;
    dist_prev = dist_raw;
  }

  /* ---------- 3) EMA 필터 ---------- */
  // EMA: y[n] = a*x[n] + (1-a)*y[n-1]
  if (_EMA_ALPHA >= 1.0f) {
    dist_ema = dist_filtered;                 // EMA 미적용 모드
  } else {
    dist_ema = _EMA_ALPHA * dist_filtered + (1.0f - _EMA_ALPHA) * dist_ema;
  }

  /* ---------- 4) 거리에 따른 서보 각도 매핑 ----------
     - d <= 180mm  -> 0°
     - 180~360mm   -> 0~180° 선형
     - d >= 360mm  -> 180°
     제어에는 EMA 값을 사용(노이즈 완화) */
  float d = dist_ema;  // 제어용 거리(mm)
  float servo_deg;
  if (d <= _DIST_MIN) {
    servo_deg = 0.0f;
  } else if (d >= _DIST_MAX) {
    servo_deg = 180.0f;
  } else {
    // 180~360mm 선형 매핑
    servo_deg = (d - _DIST_MIN) * (180.0f / (_DIST_MAX - _DIST_MIN));
  }
  int servo_us = deg_to_us(servo_deg);
  myservo.writeMicroseconds(servo_us);

  /* ---------- 5) 타깃 윈도우 LED (active-low) ----------
     - EMA 거리가 [_TARGET_LOW, _TARGET_HIGH]이면 LED ON(LOW), 아니면 OFF(HIGH) */
  if (d >= _TARGET_LOW && d <= _TARGET_HIGH) {
    digitalWrite(PIN_LED, LOW);   // ON
  } else {
    digitalWrite(PIN_LED, HIGH);  // OFF
  }

  /* ---------- 6) 플로터 친화적 시리얼 출력 ----------
     과하게 큰 값이 찍히지 않도록 상한 클리핑 */
  float clip_raw = min(dist_raw, _DIST_MAX + 100.0f);
  float clip_ema = min(dist_ema, _DIST_MAX + 100.0f);

  Serial.print("Min:");    Serial.print(_DIST_MIN);
  Serial.print(",dist:");  Serial.print(clip_raw);
  Serial.print(",ema:");   Serial.print(clip_ema);
  Serial.print(",ServoDeg:"); Serial.print(servo_deg);
  Serial.print(",ServoUS:");  Serial.print(servo_us);
  Serial.print(",Low:");   Serial.print(_TARGET_LOW);
  Serial.print(",High:");  Serial.print(_TARGET_HIGH);
  Serial.print(",Max:");   Serial.print(_DIST_MAX);
  Serial.println();
}
