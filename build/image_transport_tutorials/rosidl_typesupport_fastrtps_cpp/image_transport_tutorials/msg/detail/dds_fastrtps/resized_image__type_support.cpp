// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from image_transport_tutorials:msg/ResizedImage.idl
// generated code does not contain a copyright notice
#include "image_transport_tutorials/msg/detail/resized_image__rosidl_typesupport_fastrtps_cpp.hpp"
#include "image_transport_tutorials/msg/detail/resized_image__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions
namespace sensor_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const sensor_msgs::msg::Image &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  sensor_msgs::msg::Image &);
size_t get_serialized_size(
  const sensor_msgs::msg::Image &,
  size_t current_alignment);
size_t
max_serialized_size_Image(
  bool & full_bounded,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace sensor_msgs


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
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: original_height
  cdr << ros_message.original_height;
  // Member: original_width
  cdr << ros_message.original_width;
  // Member: image
  sensor_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.image,
    cdr);
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_image_transport_tutorials
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  image_transport_tutorials::msg::ResizedImage & ros_message)
{
  // Member: original_height
  cdr >> ros_message.original_height;

  // Member: original_width
  cdr >> ros_message.original_width;

  // Member: image
  sensor_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.image);

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_image_transport_tutorials
get_serialized_size(
  const image_transport_tutorials::msg::ResizedImage & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: original_height
  {
    size_t item_size = sizeof(ros_message.original_height);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: original_width
  {
    size_t item_size = sizeof(ros_message.original_width);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: image

  current_alignment +=
    sensor_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.image, current_alignment);

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_image_transport_tutorials
max_serialized_size_ResizedImage(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;


  // Member: original_height
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: original_width
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: image
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        sensor_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_Image(
        full_bounded, current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}

static bool _ResizedImage__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const image_transport_tutorials::msg::ResizedImage *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _ResizedImage__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<image_transport_tutorials::msg::ResizedImage *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _ResizedImage__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const image_transport_tutorials::msg::ResizedImage *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _ResizedImage__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_ResizedImage(full_bounded, 0);
}

static message_type_support_callbacks_t _ResizedImage__callbacks = {
  "image_transport_tutorials::msg",
  "ResizedImage",
  _ResizedImage__cdr_serialize,
  _ResizedImage__cdr_deserialize,
  _ResizedImage__get_serialized_size,
  _ResizedImage__max_serialized_size
};

static rosidl_message_type_support_t _ResizedImage__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_ResizedImage__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace image_transport_tutorials

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_image_transport_tutorials
const rosidl_message_type_support_t *
get_message_type_support_handle<image_transport_tutorials::msg::ResizedImage>()
{
  return &image_transport_tutorials::msg::typesupport_fastrtps_cpp::_ResizedImage__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, image_transport_tutorials, msg, ResizedImage)() {
  return &image_transport_tutorials::msg::typesupport_fastrtps_cpp::_ResizedImage__handle;
}

#ifdef __cplusplus
}
#endif
