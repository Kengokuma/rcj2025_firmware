#include "wheel.hpp"
#include <Arduino.h>

Odometry::Odometry(Wheel* left_wheel, Wheel* right_wheel, float wheel_base)
    : left_wheel_(left_wheel), right_wheel_(right_wheel), wheel_base_(wheel_base),
      x_(0.0f), y_(0.0f), theta_(0.0f), last_update_time_(0), linear_x_velocity_(0.0f), linear_y_velocity_(0.0f), angular_z_velocity_(0.0f) {}

void Odometry::begin() {
    left_wheel_->begin();
    right_wheel_->begin();
    last_update_time_ = millis();
}

void Odometry::update() {
    // 左右の車輪の速度をパルス/ミリ秒で取得
    float left_velocity_pulses_ms = left_wheel_->get_velocity_pulses_ms() * -1;
    float right_velocity_pulses_ms = right_wheel_->get_velocity_pulses_ms();

    // パルス/ミリ秒をメートル/秒に変換
    // 計算: (パルス/ms) / (パルス/m) * 1000 = m/s
    float left_velocity_m_s = (left_velocity_pulses_ms / left_wheel_->get_pulse_per_meter()) * 1000.0f;
    float right_velocity_m_s = (right_velocity_pulses_ms / right_wheel_->get_pulse_per_meter()) * 1000.0f;

    // ロボットの線形速度と角速度を計算
    float linear_velocity = (left_velocity_m_s + right_velocity_m_s) / 2.0f;
    angular_z_velocity_ = (right_velocity_m_s - left_velocity_m_s) / wheel_base_;
    
    long current_time = millis();
    float dt = (current_time - last_update_time_) / 1000.0f;
    
    if (dt > 0) {
        theta_ += angular_z_velocity_ * dt;
        
        float dx = linear_velocity * cos(theta_) * dt;
        float dy = linear_velocity * sin(theta_) * dt;

        linear_x_velocity_ = linear_velocity * cos(theta_);
        linear_y_velocity_ = linear_velocity * sin(theta_);
        
        x_ += dx;
        y_ += dy;
    }
    
    last_update_time_ = current_time;
}

float Odometry::get_x() {
    return x_;
}

float Odometry::get_y() {
    return y_;
}

float Odometry::get_theta() {
    return theta_;
}

float Odometry::get_linear_x_velocity() {
    return linear_x_velocity_;
}

float Odometry::get_linear_y_velocity() {
    return linear_y_velocity_;
}

float Odometry::get_angular_z_velocity() {
    return angular_z_velocity_;
}