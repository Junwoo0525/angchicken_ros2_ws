// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from image_transport_tutorials:msg/ResizedImage.idl
// generated code does not contain a copyright notice

#ifndef IMAGE_TRANSPORT_TUTORIALS__MSG__DETAIL__RESIZED_IMAGE__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define IMAGE_TRANSPORT_TUTORIALS__MSG__DETAIL__RESIZED_IMAGE__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "image_transport_tutorials/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "image_transport_tutorials/msg/detail/resized_image__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace image_transport_tutorials
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_image_transport_tutorials
cdr_serialize(
  const image_transport_tutorials::msg::ResizedImage & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_image_transport_tutorials
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  image_transport_tutorials::msg::ResizedImage & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_image_transport_tutorials
get_serialized_size(
  const image_transport_tutorials::msg::ResizedImage & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_image_transport_tutorials
max_serialized_size_ResizedImage(
  bool & full_bounded,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace image_transport_tutorials

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_image_transport_tutorials
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, image_transport_tutorials, msg, ResizedImage)();

#ifdef __cplusplus
}
#endif

#endif  // IMAGE_TRANSPORT_TUTORIALS__MSG__DETAIL__RESIZED_IMAGE__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
