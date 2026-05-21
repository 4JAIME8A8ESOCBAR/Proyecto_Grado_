#include <micro_ros_arduino.h>

#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>

#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>

// ======================================================
// CONFIG
// ======================================================

#define LED_PIN 2

// ======================================================
// MACROS
// ======================================================

#define RCCHECK(fn) \
{ \
  rcl_ret_t temp_rc = fn; \
  if ((temp_rc != RCL_RET_OK)) { \
    error_loop(); \
  } \
}

#define RCSOFTCHECK(fn) \
{ \
  rcl_ret_t temp_rc = fn; \
  if ((temp_rc != RCL_RET_OK)) {} \
}

// ======================================================
// micro-ROS VARIABLES
// ======================================================

// Support
rclc_support_t support;

// Allocator
rcl_allocator_t allocator;

// Node
rcl_node_t node;

// Executor
rclc_executor_t executor;

// Publisher
rcl_publisher_t publisher;
std_msgs__msg__Int32 pub_msg;

// Subscriber
rcl_subscription_t subscriber;
std_msgs__msg__Int32 sub_msg;

// Timer
rcl_timer_t timer;

// ======================================================
// ERROR LOOP
// ======================================================

void error_loop()
{
  while (1)
  {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

// ======================================================
// SUBSCRIBER CALLBACK
// ======================================================

void subscription_callback(const void * msgin)
{
  const std_msgs__msg__Int32 * msg =
    (const std_msgs__msg__Int32 *)msgin;

  if (msg->data == 0)
  {
    digitalWrite(LED_PIN, LOW);
  }
  else
  {
    digitalWrite(LED_PIN, HIGH);
  }
}

// ======================================================
// TIMER CALLBACK
// ======================================================

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);

  if (timer != NULL)
  {
    RCSOFTCHECK(
      rcl_publish(
        &publisher,
        &pub_msg,
        NULL));

    pub_msg.data++;
  }
}

// ======================================================
// SETUP
// ======================================================

void setup()
{
  // LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // micro-ROS transport
  set_microros_transports();

  delay(2000);

  // Wait for agent
  while (rmw_uros_ping_agent(1000, 1) != RMW_RET_OK)
  {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(200);
  }

  digitalWrite(LED_PIN, HIGH);

  // Allocator
  allocator = rcl_get_default_allocator();

  // Support
  RCCHECK(
    rclc_support_init(
      &support,
      0,
      NULL,
      &allocator));

  // Node
  RCCHECK(
    rclc_node_init_default(
      &node,
      "esp32_node",
      "",
      &support));

  // ======================================================
  // Publisher
  // ======================================================

  RCCHECK(
    rclc_publisher_init_best_effort(
      &publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      "/contador"));

  // ======================================================
  // Subscriber
  // ======================================================

  RCCHECK(
    rclc_subscription_init_best_effort(
      &subscriber,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      "/led_control"));

  // ======================================================
  // Timer
  // ======================================================

  RCCHECK(
    rclc_timer_init_default(
      &timer,
      &support,
      RCL_MS_TO_NS(1000),
      timer_callback));

  // ======================================================
  // Executor
  // ======================================================

  RCCHECK(
    rclc_executor_init(
      &executor,
      &support.context,
      2,
      &allocator));

  // Add timer
  RCCHECK(
    rclc_executor_add_timer(
      &executor,
      &timer));

  // Add subscriber
  RCCHECK(
    rclc_executor_add_subscription(
      &executor,
      &subscriber,
      &sub_msg,
      &subscription_callback,
      ON_NEW_DATA));

  // Initial value
  pub_msg.data = 0;
}

// ======================================================
// LOOP
// ======================================================

void loop()
{
  RCSOFTCHECK(
    rclc_executor_spin_some(
      &executor,
      RCL_MS_TO_NS(10)));

  delay(10);
}