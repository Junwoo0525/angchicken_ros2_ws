// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from image_transport_tutorials:msg/ResizedImage.idl
// generated code does not contain a copyright notice
#include "image_transport_tutorials/msg/detail/resized_image__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `image`
#include "sensor_msgs/msg/detail/image__functions.h"

bool
image_transport_tutorials__msg__ResizedImage__init(image_transport_tutorials__msg__ResizedImage * msg)
{
  if (!msg) {
    return false;
  }
  // original_height
  // original_width
  // image
  if (!sensor_msgs__msg__Image__init(&msg->image)) {
    image_transport_tutorials__msg__ResizedImage__fini(msg);
    return false;
  }
  return true;
}

void
image_transport_tutorials__msg__ResizedImage__fini(image_transport_tutorials__msg__ResizedImage * msg)
{
  if (!msg) {
    return;
  }
  // original_height
  // original_width
  // image
  sensor_msgs__msg__Image__fini(&msg->image);
}

bool
image_transport_tutorials__msg__ResizedImage__are_equal(const image_transport_tutorials__msg__ResizedImage * lhs, const image_transport_tutorials__msg__ResizedImage * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // original_height
  if (lhs->original_height != rhs->original_height) {
    return false;
  }
  // original_width
  if (lhs->original_width != rhs->original_width) {
    return false;
  }
  // image
  if (!sensor_msgs__msg__Image__are_equal(
      &(lhs->image), &(rhs->image)))
  {
    return false;
  }
  return true;
}

bool
image_transport_tutorials__msg__ResizedImage__copy(
  const image_transport_tutorials__msg__ResizedImage * input,
  image_transport_tutorials__msg__ResizedImage * output)
{
  if (!input || !output) {
    return false;
  }
  // original_height
  output->original_height = input->original_height;
  // original_width
  output->original_width = input->original_width;
  // image
  if (!sensor_msgs__msg__Image__copy(
      &(input->image), &(output->image)))
  {
    return false;
  }
  return true;
}

image_transport_tutorials__msg__ResizedImage *
image_transport_tutorials__msg__ResizedImage__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  image_transport_tutorials__msg__ResizedImage * msg = (image_transport_tutorials__msg__ResizedImage *)allocator.allocate(sizeof(image_transport_tutorials__msg__ResizedImage), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(image_transport_tutorials__msg__ResizedImage));
  bool success = image_transport_tutorials__msg__ResizedImage__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
image_transport_tutorials__msg__ResizedImage__destroy(image_transport_tutorials__msg__ResizedImage * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    image_transport_tutorials__msg__ResizedImage__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
image_transport_tutorials__msg__ResizedImage__Sequence__init(image_transport_tutorials__msg__ResizedImage__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  image_transport_tutorials__msg__ResizedImage * data = NULL;

  if (size) {
    data = (image_transport_tutorials__msg__ResizedImage *)allocator.zero_allocate(size, sizeof(image_transport_tutorials__msg__ResizedImage), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = image_transport_tutorials__msg__ResizedImage__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        image_transport_tutorials__msg__ResizedImage__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
image_transport_tutorials__msg__ResizedImage__Sequence__fini(image_transport_tutorials__msg__ResizedImage__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      image_transport_tutorials__msg__ResizedImage__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

image_transport_tutorials__msg__ResizedImage__Sequence *
image_transport_tutorials__msg__ResizedImage__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  image_transport_tutorials__msg__ResizedImage__Sequence * array = (image_transport_tutorials__msg__ResizedImage__Sequence *)allocator.allocate(sizeof(image_transport_tutorials__msg__ResizedImage__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = image_transport_tutorials__msg__ResizedImage__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
image_transport_tutorials__msg__ResizedImage__Sequence__destroy(image_transport_tutorials__msg__ResizedImage__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    image_transport_tutorials__msg__ResizedImage__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
image_transport_tutorials__msg__ResizedImage__Sequence__are_equal(const image_transport_tutorials__msg__ResizedImage__Sequence * lhs, const image_transport_tutorials__msg__ResizedImage__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!image_transport_tutorials__msg__ResizedImage__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
image_transport_tutorials__msg__ResizedImage__Sequence__copy(
  const image_transport_tutorials__msg__ResizedImage__Sequence * input,
  image_transport_tutorials__msg__ResizedImage__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(image_transport_tutorials__msg__ResizedImage);
    image_transport_tutorials__msg__ResizedImage * data =
      (image_transport_tutorials__msg__ResizedImage *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!image_transport_tutorials__msg__ResizedImage__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          image_transport_tutorials__msg__ResizedImage__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!image_transport_tutorials__msg__ResizedImage__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
