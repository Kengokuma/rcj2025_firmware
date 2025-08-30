#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>

#include <SPI.h>
#include <geometry_msgs/msg/quaternion.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <std_msgs/msg/u_int8.h>

#include "CytronMotorDriver.h"
#include "wheel.hpp"

#include "Adafruit_TCS34725.h"

#include "rescue_kit.hpp"

#define PULSES_PER_METER 7800.0f

// ロボットの車輪間距離（例：15.8cm）
#define WHEEL_BASE 0.158f

RescueKit kit(8, 9, 20, 27);

MbedI2C myi2c(p12, p13);

// 左車輪の設定
Wheel left_wheel(10, PULSES_PER_METER);
// 右車輪の設定
Wheel right_wheel(18, PULSES_PER_METER);

Adafruit_TCS34725 tcs =
    Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_1X);

// オドメトリクラスのインスタンス
Odometry odometry(&left_wheel, &right_wheel, WHEEL_BASE);

CytronMD motor_left_backward(PWM_PWM, 0, 1);  // PWM 1A = Pin 0, PWM 1B = Pin 1.
CytronMD motor_right_backward(PWM_PWM, 2, 3); // PWM 2A = Pin 2, PWM 2B = Pin 3.
CytronMD motor_right_forward(PWM_PWM, 4, 5);  // PWM 3A = Pin 4, PWM 3B = Pin 5.
CytronMD motor_left_forward(PWM_PWM, 6, 7);   // PWM 4A = Pin 6, PWM 4B = Pin 7.

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only avaliable for Arduino framework with serial transport.
#endif

rcl_subscription_t cmd_vel_subscriber;
geometry_msgs__msg__Twist cmd_vel_msg;

rcl_subscription_t victim_subscriber;
std_msgs__msg__UInt8 victim_msg;
// Left : Green U 0
//      : Yellow S 1
//      : Red H 2
// Right : Green U 3
//       : Yellow S 4
//       : Red H 5

rcl_publisher_t odom_publisher;
nav_msgs__msg__Odometry odom_msg;

rcl_publisher_t color_publisher;
std_msgs__msg__UInt8 color_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

bool is_blinking = false;

#define RCCHECK(fn)                                                            \
  {                                                                            \
    rcl_ret_t temp_rc = fn;                                                    \
    if ((temp_rc != RCL_RET_OK)) {                                             \
      error_loop();                                                            \
    }                                                                          \
  }
#define RCSOFTCHECK(fn)                                                        \
  {                                                                            \
    rcl_ret_t temp_rc = fn;                                                    \
    if ((temp_rc != RCL_RET_OK)) {                                             \
    }                                                                          \
  }

// エラーハンドリングループ
void error_loop() {
  while (1) {
    delay(100);
  }
}

// 角度をクォータニオンに変換するヘルパー関数
void euler_to_quaternion(float yaw, float pitch, float roll,
                         geometry_msgs__msg__Quaternion *q) {
  float cy = cos(yaw * 0.5);
  float sy = sin(yaw * 0.5);
  float cp = cos(pitch * 0.5);
  float sp = sin(pitch * 0.5);
  float cr = cos(roll * 0.5);
  float sr = sin(roll * 0.5);

  q->w = cr * cp * cy + sr * sp * sy;
  q->x = sr * cp * cy - cr * sp * sy;
  q->y = cr * sp * cy + sr * cp * sy;
  q->z = cr * cp * sy - sr * sp * cy;
}

void cmd_vel_callback(const void *msgin) {
  const geometry_msgs__msg__Twist *twist_msg =
      (const geometry_msgs__msg__Twist *)msgin;

  // ロボットの線形速度と角速度から左右の車輪の速度を計算
  // v_left = v - (omega * L) / 2
  // v_right = v + (omega * L) / 2
  float linear_vel = twist_msg->linear.x;
  float angular_vel = twist_msg->angular.z;
  float wheel_base = WHEEL_BASE;

  float left_vel_m_s = linear_vel - (angular_vel * wheel_base) / 2.0f;
  float right_vel_m_s = linear_vel + (angular_vel * wheel_base) / 2.0f;

  motor_left_forward.setSpeed(-left_vel_m_s);
  motor_left_backward.setSpeed(-left_vel_m_s);
  motor_right_forward.setSpeed(right_vel_m_s);
  motor_right_backward.setSpeed(right_vel_m_s);
}

void victim_callback(const void *msgin) {
  const std_msgs__msg__UInt8 *victim_msg = (const std_msgs__msg__UInt8 *)msgin;

  uint8_t victim_color = victim_msg->data;
  for (int i = 0; i < 10; i++)
  {
    digitalWrite(LED_BUILTIN, HIGH); // Turn the LED on
    delay(260);                      // Wait for a second
    digitalWrite(LED_BUILTIN, LOW);  // Turn the LED off
    delay(260);                      // Wait for a second
  }
  

  bool is_victim_detected = (victim_color < 3);
  kit.deploy((victim_color % 3), is_victim_detected);
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    // オドメトリを更新
    odometry.update();

    // メッセージのヘッダーとフレームIDを設定
    odom_msg.header.stamp.nanosec = rmw_uros_epoch_nanos();
    odom_msg.header.frame_id.data = (char *)"odom";
    odom_msg.header.frame_id.size = strlen(odom_msg.header.frame_id.data);
    odom_msg.child_frame_id.data = (char *)"base_footprint";
    odom_msg.child_frame_id.size = strlen(odom_msg.child_frame_id.data);

    // ポーズ（位置と向き）を設定
    odom_msg.pose.pose.position.x = odometry.get_x();
    odom_msg.pose.pose.position.y = odometry.get_y();
    odom_msg.pose.pose.position.z = 0.0;
    euler_to_quaternion(odometry.get_theta(), 0, 0,
                        &odom_msg.pose.pose.orientation);

    // ツイスト（速度）を設定
    odom_msg.twist.twist.linear.x = odometry.get_linear_x_velocity();
    odom_msg.twist.twist.linear.y = odometry.get_linear_y_velocity();
    odom_msg.twist.twist.angular.z = odometry.get_angular_z_velocity();

    // /odomトピックにメッセージをパブリッシュ
    RCSOFTCHECK(rcl_publish(&odom_publisher, &odom_msg, NULL));

    // カラーセンサーのデータを取得
    uint16_t r, g, b, c;
    tcs.getRawData(&r, &g, &b, &c);
    if (c < 400) {
      // 黒色を検出
      color_msg.data = 1;
      RCSOFTCHECK(rcl_publish(&color_publisher, &color_msg, NULL));
    } else if (c < 2000 && b > g && b > r) {
      // 青色を検出
      color_msg.data = 2;
      RCSOFTCHECK(rcl_publish(&color_publisher, &color_msg, NULL));
    } else {
      // 赤色を検出
      color_msg.data = 0;
      RCSOFTCHECK(rcl_publish(&color_publisher, &color_msg, NULL));
    }


  }
}

void setup() {
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  delay(2000);

  allocator = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(
      rclc_node_init_default(&node, "micro_ros_platformio_node", "", &support));

  // create publisher for /odom
  RCCHECK(rclc_publisher_init_default(
      &odom_publisher, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "odom"));

  RCCHECK(rclc_publisher_init_default(
      &color_publisher, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8), "color"));

  RCCHECK(rclc_subscription_init_default(
      &cmd_vel_subscriber, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));

  RCCHECK(rclc_subscription_init_default(
      &victim_subscriber, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8), "victim"));

  // create timer for 20ms update rate
  const unsigned int timer_timeout = 20; // 50 Hz
  RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(timer_timeout),
                                  timer_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 6, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_subscriber,
                                         &cmd_vel_msg, &cmd_vel_callback,
                                         ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &victim_subscriber,
                                         &victim_msg, &victim_callback,
                                         ON_NEW_DATA));

  // オドメトリを初期化
  odometry.begin();

  tcs.begin(TCS34725_ADDRESS, &myi2c);

  pinMode(LED_BUILTIN, OUTPUT);

  kit.begin();
}

void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
