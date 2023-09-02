// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from image_transport_tutorials:msg/ResizedImage.idl
// generated code does not contain a copyright notice

#ifndef IMAGE_TRANSPORT_TUTORIALS__MSG__DETAIL__RESIZED_IMAGE__TRAITS_HPP_
#define IMAGE_TRANSPORT_TUTORIALS__MSG__DETAIL__RESIZED_IMAGE__TRAITS_HPP_

#include "image_transport_tutorials/msg/detail/resized_image__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'image'
#include "sensor_msgs/msg/detail/image__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<image_transport_tutorials::msg::ResizedImage>()
{
  return "image_transport_tutorials::msg::ResizedImage";
}

template<>
inline const char * name<image_transport_tutorials::msg::ResizedImage>()
{
  return "image_transport_tutorials/msg/ResizedImage";
}

template<>
struct has_fixed_size<image_transport_tutorials::msg::ResizedImage>
  : std::integral_constant<bool, has_fixed_size<sensor_msgs::msg::Image>::value> {};

template<>
struct has_bounded_size<image_transport_tutorials::msg::ResizedImage>
  : std::integral_constant<bool, has_bounded_size<sensor_msgs::msg::Image>::value> {};

template<>
struct is_message<image_transport_tutorials::msg::ResizedImage>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // IMAGE_TRANSPORT_TUTORIALS__MSG__DETAIL__RESIZED_IMAGE__TRAITS_HPP_
