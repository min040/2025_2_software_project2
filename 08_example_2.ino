// Arduino pin assignment
#define PIN_LED  9
#define PIN_TRIG 12   // sonar sensor TRIGGER
#define PIN_ECHO 13   // sonar sensor ECHO

// configurable parameters
#define SND_VEL 346.0     // sound velocity at 24 celsius degree (unit: m/sec)
#define INTERVAL 25       // sampling interval (unit: msec)
#define PULSE_DURATION 10 // ultra-sound Pulse Duration (unit: usec)
#define _DIST_MIN 100.0   // minimum distance to be measured (unit: mm)
#define _DIST_MAX 300.0   // maximum distance to be measured (unit: mm)

#define TIMEOUT ((INTERVAL / 2) * 1000.0) // maximum echo waiting time (unit: usec)
#define SCALE (0.001 * 0.5 * SND_VEL)     // coefficent to convert duration to distance

unsigned long last_sampling_time;   // unit: msec

void setup() {
  // initialize GPIO pins
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_TRIG, OUTPUT);  // sonar TRIGGER
  pinMode(PIN_ECHO, INPUT);   // sonar ECHO
  digitalWrite(PIN_TRIG, LOW);  // turn-off Sonar 
  
  // initialize serial port
  Serial.begin(115200);
}

float distance = _DIST_MAX;

void loop() { 
  // wait until next sampling time. // polling
  if (millis() < (last_sampling_time + INTERVAL))
    return;

  // ---------- 밝기 제어 (사진 조건 반영) ----------
  // 200mm에서 최대 밝기, 100/300mm에서 0이 되도록 삼각형 매핑
  // 액티브-로우 회로이므로 PWM은 반전(255 - brightness)하여 출력
  if ((distance == 0.0) || (distance > _DIST_MAX)) {
      distance = _DIST_MAX + 10.0;    // Set Higher Value (범위 밖 표시)
      digitalWrite(PIN_LED, 1);       // LED OFF
      analogWrite(PIN_LED, 255);      // PWM도 OFF(액티브-로우 → High 듀티 100%)
  } else if (distance < _DIST_MIN) {
      distance = _DIST_MIN - 10.0;    // Set Lower Value (범위 밖 표시)
      digitalWrite(PIN_LED, 1);       // LED OFF
      analogWrite(PIN_LED, 255);      // PWM도 OFF
  } else {    // In desired Range [100, 300]
      digitalWrite(PIN_LED, 0);       // 기본 ON (아래 PWM이 최종 적용)

      int center    = (int)((_DIST_MIN + _DIST_MAX) / 2);   // 200
      int halfRange = (int)((_DIST_MAX - _DIST_MIN) / 2);   // 100
      int d         = (int)distance;
      int offset    = abs(d - center);                      // 200으로부터의 거리
      int brightness = map(offset, 0, halfRange, 255, 0);   // 200→255, 100/300→0
      brightness = constrain(brightness, 0, 255);

      int pwm = 255 - brightness;                           // 액티브-로우 반전
      analogWrite(PIN_LED, pwm);                            // 밝기 적용
  }
  // ---------- 밝기 제어 끝 ----------

  // output the distance to the serial port
  Serial.print("Min:");        Serial.print(_DIST_MIN);
  Serial.print(",distance:");  Serial.print(distance);
  Serial.print(",Max:");       Serial.print(_DIST_MAX);
  Serial.println("");

  distance = USS_measure(PIN_TRIG, PIN_ECHO); // read distance
  
  // update last sampling time
  last_sampling_time += INTERVAL;
}

// get a distance reading from USS. return value is in millimeter.
float USS_measure(int TRIG, int ECHO)
{
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(PULSE_DURATION);
  digitalWrite(TRIG, LOW);
  
  return pulseIn(ECHO, HIGH, TIMEOUT) * SCALE; // unit: mm

  // Pulse duration to distance conversion example (target distance = 17.3m)
  // - pulseIn(ECHO, HIGH, timeout) returns microseconds (음파의 왕복 시간)
  // - 편도 거리 = (pulseIn() / 1,000,000) * SND_VEL / 2 (미터 단위)
  //   mm 단위로 하려면 * 1,000이 필요 ==>  SCALE = 0.001 * 0.5 * SND_VEL
  //
  // - 예, pusseIn()이 100,000 이면 (= 0.1초, 왕복 거리 34.6m)
  //        = 100,000 micro*sec * 0.001 milli/micro * 0.5 * 346 meter/sec
  //        = 100,000 * 0.001 * 0.5 * 346
  //        = 17,300 mm  ==> 17.3m
}
