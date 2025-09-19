int ledPin = 7;

void setup() {
  pinMode(ledPin, OUTPUT);
}

void loop() {
  // ① 처음 1초간 LED 켜기 (LOW가 켜짐!)
  digitalWrite(ledPin, LOW);
  delay(1000);

  // ② 5회 깜빡이기
  for (int i = 0; i < 5; i++) {
    digitalWrite(ledPin, HIGH);  // 끄기
    delay(250);
    digitalWrite(ledPin, LOW);   // 켜기
    delay(250);
  }

  // ③ LED 끄기
  digitalWrite(ledPin, HIGH);

  // ④ 무한 루프
  while (1) {}
}
