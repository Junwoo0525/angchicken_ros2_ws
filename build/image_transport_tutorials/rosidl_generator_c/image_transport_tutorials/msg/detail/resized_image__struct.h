// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from image_transport_tutorials:msg/ResizedImage.idl
// generated code does not contain a copyright notice

#ifndef IMAGE_TRANSPORT_TUTORIALS__MSG__DETAIL__RESIZED_IMAGE__STRUCT_H_
#define IMAGE_TRANSPORT_TUTORIALS__MSG__DETAIL__RESIZED_IMAGE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'image'
#include "sensor_msgs/msg/detail/image__struct.h"

// Struct defined in msg/ResizedImage in the package image_transport_tutorials.
typedef struct image_transport_tutorials__msg__ResizedImage
{
  uint32_t original_height;
  uint32_t original_width;
  sensor_msgs__msg__Image image;
} image_transport_tutorials__msg__ResizedImage;

// Struct for a sequence of image_transport_tutorials__msg__ResizedImage.
typedef struct image_transport_tutorials__msg__ResizedImage__Sequence
{
  image_transport_tutorials__msg__ResizedImage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} image_transport_tutorials__msg__ResizedImage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // IMAGE_TRANSPORT_TUTORIALS__MSG__DETAIL__RESIZED_IMAGE__STRUCT_H_
