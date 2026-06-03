// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from depthai_ros_msgs:srv/SetLocalTransform.idl
// generated code does not contain a copyright notice

#ifndef DEPTHAI_ROS_MSGS__SRV__DETAIL__SET_LOCAL_TRANSFORM__STRUCT_H_
#define DEPTHAI_ROS_MSGS__SRV__DETAIL__SET_LOCAL_TRANSFORM__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'local_transform'
#include "geometry_msgs/msg/detail/pose__struct.h"

/// Struct defined in srv/SetLocalTransform in the package depthai_ros_msgs.
typedef struct depthai_ros_msgs__srv__SetLocalTransform_Request
{
  geometry_msgs__msg__Pose local_transform;
} depthai_ros_msgs__srv__SetLocalTransform_Request;

// Struct for a sequence of depthai_ros_msgs__srv__SetLocalTransform_Request.
typedef struct depthai_ros_msgs__srv__SetLocalTransform_Request__Sequence
{
  depthai_ros_msgs__srv__SetLocalTransform_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} depthai_ros_msgs__srv__SetLocalTransform_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/SetLocalTransform in the package depthai_ros_msgs.
typedef struct depthai_ros_msgs__srv__SetLocalTransform_Response
{
  bool success;
} depthai_ros_msgs__srv__SetLocalTransform_Response;

// Struct for a sequence of depthai_ros_msgs__srv__SetLocalTransform_Response.
typedef struct depthai_ros_msgs__srv__SetLocalTransform_Response__Sequence
{
  depthai_ros_msgs__srv__SetLocalTransform_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} depthai_ros_msgs__srv__SetLocalTransform_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DEPTHAI_ROS_MSGS__SRV__DETAIL__SET_LOCAL_TRANSFORM__STRUCT_H_
