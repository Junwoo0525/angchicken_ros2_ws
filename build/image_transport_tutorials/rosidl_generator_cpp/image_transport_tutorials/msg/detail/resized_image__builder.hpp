// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from image_transport_tutorials:msg/ResizedImage.idl
// generated code does not contain a copyright notice

#ifndef IMAGE_TRANSPORT_TUTORIALS__MSG__DETAIL__RESIZED_IMAGE__BUILDER_HPP_
#define IMAGE_TRANSPORT_TUTORIALS__MSG__DETAIL__RESIZED_IMAGE__BUILDER_HPP_

#include "image_transport_tutorials/msg/detail/resized_image__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace image_transport_tutorials
{

namespace msg
{

namespace builder
{

class Init_ResizedImage_image
{
public:
  explicit Init_ResizedImage_image(::image_transport_tutorials::msg::ResizedImage & msg)
  : msg_(msg)
  {}
  ::image_transport_tutorials::msg::ResizedImage image(::image_transport_tutorials::msg::ResizedImage::_image_type arg)
  {
    msg_.image = std::move(arg);
    return std::move(msg_);
  }

private:
  ::image_transport_tutorials::msg::ResizedImage msg_;
};

class Init_ResizedImage_original_width
{
public:
  explicit Init_ResizedImage_original_width(::image_transport_tutorials::msg::ResizedImage & msg)
  : msg_(msg)
  {}
  Init_ResizedImage_image original_width(::image_transport_tutorials::msg::ResizedImage::_original_width_type arg)
  {
    msg_.original_width = std::move(arg);
    return Init_ResizedImage_image(msg_);
  }

private:
  ::image_transport_tutorials::msg::ResizedImage msg_;
};

class Init_ResizedImage_original_height
{
public:
  Init_ResizedImage_original_height()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ResizedImage_original_width original_height(::image_transport_tutorials::msg::ResizedImage::_original_height_type arg)
  {
    msg_.original_height = std::move(arg);
    return Init_ResizedImage_original_width(msg_);
  }

private:
  ::image_transport_tutorials::msg::ResizedImage msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::image_transport_tutorials::msg::ResizedImage>()
{
  return image_transport_tutorials::msg::builder::Init_ResizedImage_original_height();
}

}  // namespace image_transport_tutorials

#endif  // IMAGE_TRANSPORT_TUTORIALS__MSG__DETAIL__RESIZED_IMAGE__BUILDER_HPP_
