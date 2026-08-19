// Workaround for _Check_return_ annotation
#ifdef _WIN32
#ifndef _Check_return_
#define _Check_return_
#endif
#endif

// ros client library
#include <rcl/rcl.h>

// query the network
#include <rcl/graph.h>

// logging
#include <rcl/logging.h>

// per-node /rosout publisher (rcl does not create it in rcl_node_init;
// the client library has to do it, like rclcpp/rclpy do)
#include <rcl/logging_rosout.h>

// errors
#include <rcutils/error_handling.h>

// low level msg type handling
#include <rosidl_runtime_c/string.h>
#include <rosidl_runtime_c/string_functions.h>

#include <rosidl_runtime_c/u16string.h>
#include <rosidl_runtime_c/u16string_functions.h>

#include <rosidl_runtime_c/primitives_sequence.h>
#include <rosidl_runtime_c/primitives_sequence_functions.h>

#include <rosidl_runtime_c/message_type_support_struct.h>
#include <rosidl_runtime_c/service_type_support_struct.h>
#include <rosidl_runtime_c/action_type_support_struct.h>
#include <rosidl_typesupport_introspection_c/message_introspection.h>
#include <rosidl_typesupport_introspection_c/field_types.h>

// parameters
#include <rcl_yaml_param_parser/parser.h>
