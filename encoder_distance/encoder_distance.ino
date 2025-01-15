// 計算常數
const float wheelDiameter = 6.0;  // 輪子直徑，單位：公分 (可根據實際情況調整)
const int pulsesPerRevolution = 1000;  // 每圈的脈衝數
const float distancePerPulse = (PI * wheelDiameter) / pulsesPerRevolution;  // 每脈衝距離，單位：公分

volatile long pulseCount = 0;  // 記錄脈衝數
unsigned long previousMillis = 0;
float distanceTraveled = 0;  // 總行駛距離
float velocity = 0;  // 速度

void setup() {
  Serial.begin(115200);

  pinMode(2, INPUT_PULLUP);  // 使用支援中斷的 GPIO 2
  pinMode(4, INPUT_PULLUP);  // 使用支援中斷的 GPIO 4

  // 設置中斷來計算脈衝
  attachInterrupt(digitalPinToInterrupt(2), countPulse, RISING);
  attachInterrupt(digitalPinToInterrupt(4), countPulse, RISING);
}

void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= 10) {  // 每10毫秒取樣一次
    noInterrupts();  // 暫停中斷
    long pulses = pulseCount;
    pulseCount = 0;  // 重置計數
    interrupts();  // 重新啟用中斷

    // 計算行駛距離
    distanceTraveled += pulses * distancePerPulse;  // 總距離，單位：公分

    // 輸出結果到 Serial Monitor
    Serial.print("距離: ");
    Serial.println(distanceTraveled);

    previousMillis = currentMillis;  // 更新時間
  }
}

void countPulse() {
  pulseCount++;  // 每次中斷時增加脈衝數
}
