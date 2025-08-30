#ifndef WHEEL_HPP
#define WHEEL_HPP

#include <Arduino.h>
#include <pio_encoder.h>

/**
 * @class Wheel
 * @brief エンコーダ付きの単一の車輪を表し、速度を追跡します。
 */
class Wheel {
public:
    Wheel(int encoder_pin, float pulse_per_meter);
    void begin();
    float get_velocity_pulses_ms();
    float get_pulse_per_meter();

private:
    PioEncoder encoder_;
    float pulse_per_meter_;
    long last_count_;
    long last_time_;
};

/**
 * @class Odometry
 * @brief 車輪のオドメトリを使用して、ロボットの位置と向きを計算します。
 */
class Odometry {
public:
    Odometry(Wheel* left_wheel, Wheel* right_wheel, float wheel_base);
    void begin();
    void update();
    
    // オドメトリ情報のゲッター
    float get_x();
    float get_y();
    float get_theta();

    float get_linear_x_velocity();
    float get_linear_y_velocity();
    float get_angular_z_velocity();

private:
    Wheel* left_wheel_;
    Wheel* right_wheel_;
    float wheel_base_;

    float linear_x_velocity_;
    float linear_y_velocity_;
    float angular_z_velocity_;

    float x_;
    float y_;
    float theta_;
    long last_update_time_;
};

#endif // WHEEL_HPP