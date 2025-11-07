#include <Servo.h>

// Arduino pin assignment
#define PIN_IR    0         // IR sensor at Pin A0
#define PIN_LED   9
#define PIN_SERVO 10

// ---- servo microseconds (SG90 typical) ----
#define _DUTY_MIN 1000      // servo full CW (≈0°)
#define _DUTY_NEU 1500      // servo neutral (≈90°)
#define _DUTY_MAX 2000      // servo full CCW (≈180°)

// ---- distance range (mm) ----
#define _DIST_MIN  100.0    // minimum distance 100 mm
#define _DIST_MAX  250.0    // maximum distance 250 mm

#define EMA_ALPHA  0.20     // 0.1~0.3 권장 (더 크면 반응 빠름, 더 작으면 부드러움)
#define LOOP_INTERVAL 25    // msec, 20ms 이상 유지

Servo myservo;
unsigned long last_loop_time;   // msec

float dist_prev = _DIST_MIN;
float dist_ema  = _DIST_MIN;

void setup() {
  pinMode(PIN_LED, OUTPUT);

  myservo.attach(PIN_SERVO);
  myservo.writeMicroseconds(_DUTY_NEU);

  Serial.begin(1000000);        // 1,000,000 bps
  last_loop_time = millis();
}

void loop() {
  unsigned long time_curr = millis();
  if (time_curr < (last_loop_time + LOOP_INTERVAL)) return;
  last_loop_time += LOOP_INTERVAL;

  int   duty;
  float a_value, dist_raw, dist_clamped;

  a_value  = analogRead(PIN_IR);
  // IR(A0) → distance(mm): 예시 공식
  dist_raw = ((6762.0 / (a_value - 9.0)) - 4.0) * 10.0;

  // ---- range filter & LED ----
  bool in_range = (dist_raw >= _DIST_MIN && dist_raw <= _DIST_MAX);
  digitalWrite(PIN_LED, in_range ? HIGH : LOW);

  // 범위를 벗어나면 clamp
  dist_clamped = dist_raw;
  if (dist_clamped < _DIST_MIN) dist_clamped = _DIST_MIN;
  if (dist_clamped > _DIST_MAX) dist_clamped = _DIST_MAX;

  // ---- EMA filter ----
  dist_ema = EMA_ALPHA * dist_clamped + (1.0 - EMA_ALPHA) * dist_prev;
  dist_prev = dist_ema;

  // ---- map() 등가식 (선형 보간) ----
  // duty = map(dist_ema, _DIST_MIN, _DIST_MAX, _DUTY_MIN, _DUTY_MAX);
  duty = _DUTY_MIN + (int)((dist_ema - _DIST_MIN) * (_DUTY_MAX - _DUTY_MIN)
                           / (_DIST_MAX - _DIST_MIN));

  myservo.writeMicroseconds(duty);

  // ---- Serial Plotter용 출력 ----
  Serial.print("_DUTY_MIN:");  Serial.print(_DUTY_MIN);
  Serial.print(",_DIST_MIN:"); Serial.print(_DIST_MIN);
  Serial.print(",IR:");        Serial.print(a_value);
  Serial.print(",dist_raw:");  Serial.print(dist_raw);
  Serial.print(",ema:");       Serial.print(dist_ema);
  Serial.print(",servo:");     Serial.print(duty);
  Serial.print(",_DIST_MAX:"); Serial.print(_DIST_MAX);
  Serial.print(",_DUTY_MAX:"); Serial.print(_DUTY_MAX);
  Serial.println();
}
