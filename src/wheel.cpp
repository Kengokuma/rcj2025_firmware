#include "wheel.hpp"
#include <Arduino.h>

Wheel::Wheel(int encoder_pin, float pulse_per_meter)
    : encoder_(encoder_pin, encoder_pin),
      pulse_per_meter_(pulse_per_meter),
      last_count_(0),
      last_time_(0) {}

void Wheel::begin() {
    last_time_ = millis();
    encoder_.begin();
}

/**
 * @brief 車輪の速度をパルス/ミリ秒で取得します。
 * @return 速度（パルス/ミリ秒）
 */
float Wheel::get_velocity_pulses_ms() {
    long current_time = millis();
    long time_diff_ms = current_time - last_time_;
    
    long current_count = encoder_.getCount();
    
    if (time_diff_ms == 0) {
        return 0;
    }
    
    float velocity = (float)(current_count - last_count_) / (float)time_diff_ms;
    
    last_count_ = current_count;
    last_time_ = current_time;
    
    return velocity;
}

float Wheel::get_pulse_per_meter() {
    return pulse_per_meter_;
}