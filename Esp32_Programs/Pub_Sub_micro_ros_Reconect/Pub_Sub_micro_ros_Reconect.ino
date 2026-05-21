#include <micro_ros_arduino.h>

#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>

#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <rmw_microros/rmw_microros.h>

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
      return false; \
    } \
  }

#define RCSOFTCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) {} \
  }

#define EXECUTE_EVERY_N_MS(MS, X)  \
  do { \
    static volatile int64_t init = -1; \
    if (init == -1) { \
      init = uxr_millis(); \
    } \
    if (uxr_millis() - init > MS) { \
      X; \
      init = uxr_millis(); \
    } \
  } while (0)

// ======================================================
// ROS VARIABLES
// ======================================================

rcl_allocator_t allocator;
rclc_support_t support;

rcl_node_t node;

rcl_publisher_t publisher;
rcl_subscription_t subscriber;

rcl_timer_t timer;

rclc_executor_t executor;

std_msgs__msg__Int32 pub_msg;
std_msgs__msg__Int32 sub_msg;

// ======================================================
// STATES
// ======================================================

enum states
{
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
};

states state;

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
  (void) last_call_time;

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
// CREATE ENTITIES
// ======================================================

bool create_entities()
{
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

  const unsigned int timer_timeout = 1000;

  RCCHECK(
    rclc_timer_init_default(
      &timer,
      &support,
      RCL_MS_TO_NS(timer_timeout),
      timer_callback));

  // ======================================================
  // Executor
  // ======================================================

  executor = rclc_executor_get_zero_initialized_executor();

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

  // Initial publisher value
  pub_msg.data = 0;

  return true;
}

// ======================================================
// DESTROY ENTITIES
// ======================================================

void destroy_entities()
{
  rmw_context_t * rmw_context =
    rcl_context_get_rmw_context(&support.context);

  (void) rmw_uros_set_context_entity_destroy_session_timeout(
    rmw_context,
    0);

  rcl_publisher_fini(&publisher, &node);

  rcl_subscription_fini(&subscriber, &node);

  rcl_timer_fini(&timer);

  rclc_executor_fini(&executor);

  rcl_node_fini(&node);

  rclc_support_fini(&support);
}

// ======================================================
// SETUP
// ======================================================

void setup()
{
  pinMode(LED_PIN, OUTPUT);

  digitalWrite(LED_PIN, LOW);

  // micro-ROS transport
  set_microros_transports();

  delay(2000);

  state = WAITING_AGENT;
}

// ======================================================
// LOOP
// ======================================================

void loop()
{
  switch (state)
  {
    // ==================================================
    // WAITING AGENT
    // ==================================================

    case WAITING_AGENT:

      EXECUTE_EVERY_N_MS(
        500,
        state =
          (RMW_RET_OK ==
           rmw_uros_ping_agent(100, 1))
          ? AGENT_AVAILABLE
          : WAITING_AGENT;
      );

      break;

    // ==================================================
    // AGENT AVAILABLE
    // ==================================================

    case AGENT_AVAILABLE:

      state =
        (true == create_entities())
        ? AGENT_CONNECTED
        : WAITING_AGENT;

      if (state == WAITING_AGENT)
      {
        destroy_entities();
      }

      break;

    // ==================================================
    // AGENT CONNECTED
    // ==================================================

    case AGENT_CONNECTED:

      EXECUTE_EVERY_N_MS(
        200,
        state =
          (RMW_RET_OK ==
           rmw_uros_ping_agent(100, 1))
          ? AGENT_CONNECTED
          : AGENT_DISCONNECTED;
      );

      if (state == AGENT_CONNECTED)
      {
        rclc_executor_spin_some(
          &executor,
          RCL_MS_TO_NS(10));
      }

      break;

    // ==================================================
    // AGENT DISCONNECTED
    // ==================================================

    case AGENT_DISCONNECTED:

      destroy_entities();

      state = WAITING_AGENT;

      break;

    default:
      break;
  }

  // ======================================================
  // STATUS LED
  // ======================================================

  if (state == AGENT_CONNECTED)
  {
    digitalWrite(LED_PIN, HIGH);
  }
  else
  {
    digitalWrite(LED_PIN, LOW);
  }
}