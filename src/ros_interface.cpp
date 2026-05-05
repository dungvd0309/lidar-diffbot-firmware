#include <stdio.h>
#include <string.h>

#include <micro_ros_arduino.h>
#include <rmw_microros/time_sync.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <rosidl_runtime_c/string_functions.h>
#include <std_msgs/msg/float32.h>
#include <sensor_msgs/msg/joint_state.h>
#include <sensor_msgs/msg/imu.h>
#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/battery_state.h>

#include "config.h"
#include "motors_helper.h"
#include "battery.h"
#include "bno055.h"

// ROS executor and support structures
static rclc_executor_t executor;
static rclc_support_t support;
static rcl_allocator_t allocator;
static rcl_node_t node;
static rcl_timer_t timer;

// ROS publishers and subscribers
static rcl_publisher_t encoders_pub; 
static rcl_publisher_t imu_pub; 
static rcl_publisher_t battery_pub;
static rcl_subscription_t cmd_vel_sub;

// ROS messages
static sensor_msgs__msg__JointState encoders_pub_msg;
static sensor_msgs__msg__Imu imu_pub_msg;
static geometry_msgs__msg__Twist twist_sub_msg;
static sensor_msgs__msg__BatteryState battery_pub_msg;

// Overrides the weak default transport_open from micro_ros_arduino so the
// serial link uses CONFIG::ros_serial_baudrate on every (re)open.
extern "C" bool arduino_transport_open(struct uxrCustomTransport * /*transport*/)
{
    Serial.begin(CONFIG::ros_serial_baudrate);
    return true;
}

enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

size_t number_of_handles = 2; // timer + cmd_vel subscription

#define LED_PIN 2

// Macro for checking return codes of rcl functions
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){return false;}}
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  int64_t now = uxr_millis(); \
  if (init == -1) { init = now; } \
  if (now - init >= (MS)) { X; init = now; } \
} while (0)

void ros_time_sync() {
    EXECUTE_EVERY_N_MS(60000, (void)rmw_uros_sync_session(10));
}

// publishing timer cb
static void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) {
        // Encoder to msg
        float left_pos = 0.0f;
        float right_pos = 0.0f;
        get_motors_pos(&left_pos, &right_pos);
        encoders_pub_msg.position.data[0] = left_pos;
        encoders_pub_msg.position.data[1] = right_pos;

        float left_rpm = 0.0f;
        float right_rpm = 0.0f;
        get_motors_rpm(&left_rpm, &right_rpm);
        encoders_pub_msg.velocity.data[0] = CONFIG::rpm_to_linear_speed(left_rpm);
        encoders_pub_msg.velocity.data[1] = CONFIG::rpm_to_linear_speed(right_rpm);

        // IMU to msg
        ImuData imu_data;
        if (imu_read(&imu_data)) {
            imu_pub_msg.orientation.w = imu_data.orientation_w;
            imu_pub_msg.orientation.x = imu_data.orientation_x;
            imu_pub_msg.orientation.y = imu_data.orientation_y;
            imu_pub_msg.orientation.z = imu_data.orientation_z;

            imu_pub_msg.angular_velocity.x = imu_data.angular_velocity_x;
            imu_pub_msg.angular_velocity.y = imu_data.angular_velocity_y;
            imu_pub_msg.angular_velocity.z = imu_data.angular_velocity_z;

            imu_pub_msg.linear_acceleration.x = imu_data.linear_acceleration_x;
            imu_pub_msg.linear_acceleration.y = imu_data.linear_acceleration_y;
            imu_pub_msg.linear_acceleration.z = imu_data.linear_acceleration_z;
        }

        // Battery to msg
        BatteryData battery_data = battery_read();
        battery_pub_msg.voltage = battery_data.voltage;
        battery_pub_msg.percentage = battery_data.percentage;

        // Bind timestamp to msg 
        int64_t time_ns = rmw_uros_epoch_nanos();
        encoders_pub_msg.header.stamp.sec = time_ns / 1000000000;
        encoders_pub_msg.header.stamp.nanosec = time_ns % 1000000000;
        imu_pub_msg.header.stamp.sec = encoders_pub_msg.header.stamp.sec;
        imu_pub_msg.header.stamp.nanosec = encoders_pub_msg.header.stamp.nanosec;
        battery_pub_msg.header.stamp.sec = encoders_pub_msg.header.stamp.sec;
        battery_pub_msg.header.stamp.nanosec = encoders_pub_msg.header.stamp.nanosec;
        
        // Publish msg
        rcl_publish(&encoders_pub, &encoders_pub_msg, NULL);
        rcl_publish(&imu_pub, &imu_pub_msg, NULL);
        rcl_publish(&battery_pub, &battery_pub_msg, NULL);
    }
}

// /cmd_vel topic callback
void subscription_callback(const void *msgin) {
    const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;

    // Extract linear and angular velocity from the Twist msg
    float target_speed_lin_x = msg->linear.x;
    float target_speed_ang_z = msg->angular.z;

    // Serial.print("linear.x ");
    // Serial.print(msg->linear.x);
    // Serial.print(", angular.z ");
    // Serial.println(msg->angular.z);

    // Convert to wheel speeds
    float twist_target_speed_right = 0;
    float twist_target_speed_left = 0;
    CONFIG::twistToWheelSpeeds(target_speed_lin_x, target_speed_ang_z,
    &twist_target_speed_right, &twist_target_speed_left);

    // Serial.print("twist_target_speed_right ");
    // Serial.print(twist_target_speed_right);
    // Serial.print(", twist_target_speed_left ");
    // Serial.println(twist_target_speed_left);
    
    // Convert to target RPM
    float twist_target_rpm_right = CONFIG::linear_speed_to_rpm(twist_target_speed_right);
    float twist_target_rpm_left = CONFIG::linear_speed_to_rpm(twist_target_speed_left);
    
    set_motors_rpm(twist_target_rpm_left, twist_target_rpm_right);
}

bool create_entities()
{
    allocator = rcl_get_default_allocator();

    // create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // create node
    RCCHECK(rclc_node_init_default(&node, "esp32_node", "", &support));

    // create publishers: encoders, imu, battery
    RCCHECK(rclc_publisher_init_default(
        &encoders_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState), 
        "/encoders/data_raw"));

    RCCHECK(rclc_publisher_init_default(
        &imu_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), 
        "/imu/data_raw"));

    RCCHECK(rclc_publisher_init_default(
        &battery_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, BatteryState), 
        "/battery_state"));

    // create timer
    const unsigned int timer_timeout = 20; // 50hz
    RCCHECK(rclc_timer_init_default2(
        &timer,
        &support,
        RCL_MS_TO_NS(timer_timeout),
        timer_callback, true));

    // create subscriber
    RCCHECK(rclc_subscription_init_default(
        &cmd_vel_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/cmd_vel"));
    
    // create executor
    RCCHECK(rclc_executor_init(&executor, &support.context, number_of_handles, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));
    RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_sub, &twist_sub_msg, &subscription_callback, ON_NEW_DATA));
    
    ros_time_sync();
    
    return true;
}

void destroy_entities()
{
    rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
    (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    rcl_publisher_fini(&encoders_pub, &node);
    rcl_publisher_fini(&imu_pub, &node);
    rcl_publisher_fini(&battery_pub, &node);
    rcl_subscription_fini(&cmd_vel_sub, &node);
    rcl_timer_fini(&timer);
    rclc_executor_fini(&executor);
    rcl_node_fini(&node);
    rclc_support_fini(&support);
}

// Initialize ROS message 
static void ros_msg_init()
{
    const char * joint_names[] = {"left_wheel_joint", "right_wheel_joint"};
    sensor_msgs__msg__JointState__init(&encoders_pub_msg);
    sensor_msgs__msg__Imu__init(&imu_pub_msg);
    sensor_msgs__msg__BatteryState__init(&battery_pub_msg);

    rosidl_runtime_c__String__Sequence__init(&encoders_pub_msg.name, 2);
    rosidl_runtime_c__String__assign(&encoders_pub_msg.name.data[0], joint_names[0]);
    rosidl_runtime_c__String__assign(&encoders_pub_msg.name.data[1], joint_names[1]);
    rosidl_runtime_c__String__assign(&encoders_pub_msg.header.frame_id, "base_link");
    rosidl_runtime_c__String__assign(&imu_pub_msg.header.frame_id, "imu_link");
    rosidl_runtime_c__String__assign(&battery_pub_msg.header.frame_id, "base_link");

    encoders_pub_msg.position.data = (double*)malloc(sizeof(double) * 2);
    encoders_pub_msg.position.size = 2;
    encoders_pub_msg.position.capacity = 2;
    encoders_pub_msg.position.data[0] = 0.0;
    encoders_pub_msg.position.data[1] = 0.0;

    encoders_pub_msg.velocity.data = (double*)malloc(sizeof(double) * 2);
    encoders_pub_msg.velocity.size = 2;
    encoders_pub_msg.velocity.capacity = 2;
    encoders_pub_msg.velocity.data[0] = 0.0;
    encoders_pub_msg.velocity.data[1] = 0.0;

    // Diagonal covariance only. Off-diagonal entries remain zero.
    imu_pub_msg.orientation_covariance[0] = CONFIG::imu_orientation_covariance;
    imu_pub_msg.orientation_covariance[4] = CONFIG::imu_orientation_covariance;
    imu_pub_msg.orientation_covariance[8] = CONFIG::imu_orientation_covariance;

    imu_pub_msg.angular_velocity_covariance[0] = CONFIG::imu_angular_velocity_covariance;
    imu_pub_msg.angular_velocity_covariance[4] = CONFIG::imu_angular_velocity_covariance;
    imu_pub_msg.angular_velocity_covariance[8] = CONFIG::imu_angular_velocity_covariance;

    imu_pub_msg.linear_acceleration_covariance[0] = CONFIG::imu_linear_acceleration_covariance;
    imu_pub_msg.linear_acceleration_covariance[4] = CONFIG::imu_linear_acceleration_covariance;
    imu_pub_msg.linear_acceleration_covariance[8] = CONFIG::imu_linear_acceleration_covariance;

    battery_pub_msg.present = true;
    battery_pub_msg.percentage = 0.0f;

    // joint_state_pub_msg.header.frame_id.data = NULL;
    // joint_state_pub_msg.header.frame_id.size = 0;
    // joint_state_pub_msg.header.frame_id.capacity = 0;
}


void ros_init()
{   
    set_microros_transports();
    pinMode(LED_PIN, OUTPUT);
    delay(2000);
    ros_msg_init();
    state = WAITING_AGENT;
}      


void ros_update()
{
    switch (state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
      break;
    case AGENT_AVAILABLE:
      state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) {
        destroy_entities();
      };
      break;
    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
      ros_time_sync();
      if (state == AGENT_CONNECTED) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
      }
      break;
    case AGENT_DISCONNECTED:
      destroy_entities();
      state = WAITING_AGENT;
      break;
    default:
      break;
  }

  if (state == AGENT_CONNECTED) {
    digitalWrite(LED_PIN, 1);
  } else {
    digitalWrite(LED_PIN, 0);
  }
}