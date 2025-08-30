#include "wheel.hpp"

#define PULSES_PER_METER 7800.0f

// ロボットの車輪間距離（例：15.8cm）
#define WHEEL_BASE 0.158f

// 左車輪の設定
Wheel left_wheel(10, PULSES_PER_METER);
// 右車輪の設定
Wheel right_wheel(18, PULSES_PER_METER);

// オドメトリクラスのインスタンス
Odometry odometry(&left_wheel, &right_wheel, WHEEL_BASE);

void setup() {
  Serial.begin(9600);
  odometry.begin();
  Serial.println("Odometry calculation started.");
}

void loop() {
  // オドメトリを定期的に更新
  odometry.update();
  
  // 1秒ごとにオドメトリ情報をシリアルに出力
  static long last_print_time = 0;
  if (millis() - last_print_time > 1000) {
    float x = odometry.get_x();
    float y = odometry.get_y();
    float theta = odometry.get_theta();
    
    Serial.print("X: ");
    Serial.print(x);
    Serial.print(" m, Y: ");
    Serial.print(y);
    Serial.print(" m, Theta: ");
    Serial.print(theta * 180.0 / PI); // ラジアンから度へ変換
    Serial.println(" deg");
    
    last_print_time = millis();
  }

  // ここにモーターを制御するコードを追加
  // (例) モーターを前進させる、回転させるなど
}