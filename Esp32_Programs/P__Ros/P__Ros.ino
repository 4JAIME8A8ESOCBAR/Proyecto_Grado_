#include <micro_ros_arduino.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>

#include <std_msgs/msg/int32.h>

rcl_publisher_t publisher;
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;

std_msgs__msg__Int32 msg;

void setup()
{
  set_microros_transports();

  delay(2000);

  allocator = rcl_get_default_allocator();

  rclc_support_init(&support, 0, NULL, &allocator);

  rclc_node_init_default(
    &node,
    "esp32_node",
    "",
    &support);

  rclc_publisher_init_best_effort(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "/contador");

  msg.data = 0;
}

void loop()
{
  rcl_publish(&publisher, &msg, NULL);

  msg.data++;

  delay(1000);
}