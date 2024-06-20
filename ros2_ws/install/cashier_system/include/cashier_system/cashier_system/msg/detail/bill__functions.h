// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from cashier_system:msg/Bill.idl
// generated code does not contain a copyright notice

#ifndef CASHIER_SYSTEM__MSG__DETAIL__BILL__FUNCTIONS_H_
#define CASHIER_SYSTEM__MSG__DETAIL__BILL__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "cashier_system/msg/rosidl_generator_c__visibility_control.h"

#include "cashier_system/msg/detail/bill__struct.h"

/// Initialize msg/Bill message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * cashier_system__msg__Bill
 * )) before or use
 * cashier_system__msg__Bill__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_cashier_system
bool
cashier_system__msg__Bill__init(cashier_system__msg__Bill * msg);

/// Finalize msg/Bill message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_cashier_system
void
cashier_system__msg__Bill__fini(cashier_system__msg__Bill * msg);

/// Create msg/Bill message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * cashier_system__msg__Bill__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_cashier_system
cashier_system__msg__Bill *
cashier_system__msg__Bill__create();

/// Destroy msg/Bill message.
/**
 * It calls
 * cashier_system__msg__Bill__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_cashier_system
void
cashier_system__msg__Bill__destroy(cashier_system__msg__Bill * msg);

/// Check for msg/Bill message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_cashier_system
bool
cashier_system__msg__Bill__are_equal(const cashier_system__msg__Bill * lhs, const cashier_system__msg__Bill * rhs);

/// Copy a msg/Bill message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_cashier_system
bool
cashier_system__msg__Bill__copy(
  const cashier_system__msg__Bill * input,
  cashier_system__msg__Bill * output);

/// Initialize array of msg/Bill messages.
/**
 * It allocates the memory for the number of elements and calls
 * cashier_system__msg__Bill__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_cashier_system
bool
cashier_system__msg__Bill__Sequence__init(cashier_system__msg__Bill__Sequence * array, size_t size);

/// Finalize array of msg/Bill messages.
/**
 * It calls
 * cashier_system__msg__Bill__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_cashier_system
void
cashier_system__msg__Bill__Sequence__fini(cashier_system__msg__Bill__Sequence * array);

/// Create array of msg/Bill messages.
/**
 * It allocates the memory for the array and calls
 * cashier_system__msg__Bill__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_cashier_system
cashier_system__msg__Bill__Sequence *
cashier_system__msg__Bill__Sequence__create(size_t size);

/// Destroy array of msg/Bill messages.
/**
 * It calls
 * cashier_system__msg__Bill__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_cashier_system
void
cashier_system__msg__Bill__Sequence__destroy(cashier_system__msg__Bill__Sequence * array);

/// Check for msg/Bill message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_cashier_system
bool
cashier_system__msg__Bill__Sequence__are_equal(const cashier_system__msg__Bill__Sequence * lhs, const cashier_system__msg__Bill__Sequence * rhs);

/// Copy an array of msg/Bill messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_cashier_system
bool
cashier_system__msg__Bill__Sequence__copy(
  const cashier_system__msg__Bill__Sequence * input,
  cashier_system__msg__Bill__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // CASHIER_SYSTEM__MSG__DETAIL__BILL__FUNCTIONS_H_
