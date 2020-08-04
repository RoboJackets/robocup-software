#pragma once

/**
 * \file logging.hpp
 * \brief This file defines macros that wrap the ones in rclcpp/logging.hpp
 * but with a NOLINT at the end, so that we don't trip up clang_tidy every time
 * we make a logging call. This also defines EZ prefixed log calls that
 * automatically use get_logger() to get the current logger, so that logging
 * calls can be shorter.
 * See link for log descriptions
 * http://docs.ros2.org/foxy/api/rclcpp/logging_8hpp.html
 */

#include <rclcpp/logging.hpp>

// Disable formatting on this entire section so NOLINT works properly
// clang-format off
//======================================================================
// DEBUG
//======================================================================
#define RJ_DEBUG(logger, ...) RCLCPP_DEBUG(logger, __VA_ARGS__)  // NOLINT
#define RJ_DEBUG_ONCE(logger, ...) RCLCPP_DEBUG_ONCE(logger, __VA_ARGS__)  // NOLINT
#define RJ_DEBUG_EXPRESSION(logger, expression, ...) RCLCPP_DEBUG_EXPRESSION(logger, expression, __VA_ARGS__)  // NOLINT
#define RJ_DEBUG_FUNCTION(logger, function, ...) RCLCPP_DEBUG_FUNCTION(logger, function, __VA_ARGS__)  // NOLINT
#define RJ_DEBUG_SKIPFIRST(logger, ...) RCLCPP_DEBUG_SKIPFIRST(logger, __VA_ARGS__)  // NOLINT
#define RJ_DEBUG_THROTTLE(logger, clock, duration_ms, ...) RCLCPP_DEBUG_THROTTLE(logger, clock, duration_ms, __VA_ARGS__)  // NOLINT
#define RJ_DEBUG_SKIPFIRST_THROTTLE(logger, clock, duration_ms, ...) RCLCPP_DEBUG_SKIPFIRST_THROTTLE(logger, clock, duration_ms, __VA_ARGS__)  // NOLINT

#define RJ_DEBUG_STREAM(logger, stream_arg) RCLCPP_DEBUG_STREAM(logger, stream_arg)  // NOLINT
#define RJ_DEBUG_STREAM_ONCE(logger, stream_arg) RCLCPP_DEBUG_STREAM_ONCE(logger, stream_arg)  // NOLINT
#define RJ_DEBUG_STREAM_EXPRESSION(logger, expression, stream_arg) RCLCPP_DEBUG_STREAM_EXPRESSION(logger, expression, stream_arg)  // NOLINT
#define RJ_DEBUG_STREAM_FUNCTION(logger, function, stream_arg) RCLCPP_DEBUG_STREAM_FUNCTION(logger, function, stream_arg)  // NOLINT
#define RJ_DEBUG_STREAM_SKIPFIRST(logger, stream_arg) RCLCPP_DEBUG_STREAM_SKIPFIRST(logger, stream_arg)  // NOLINT
#define RJ_DEBUG_STREAM_THROTTLE(logger, clock, duration_ms, stream_arg) RCLCPP_DEBUG_STREAM_THROTTLE(logger, clock, duration_ms, stream_arg)  // NOLINT
#define RJ_DEBUG_STREAM_SKIPFIRST_THROTTLE(logger, clock, duration_ms, stream_arg) RCLCPP_DEBUG_STREAM_SKIPFIRST_THROTTLE(logger, clock, duration_ms, stream_arg)  // NOLINT

#define EZ_DEBUG(...) RCLCPP_DEBUG(get_logger(), __VA_ARGS__)  // NOLINT
#define EZ_DEBUG_ONCE(...) RCLCPP_DEBUG_ONCE(get_logger(), __VA_ARGS__)  // NOLINT
#define EZ_DEBUG_EXPRESSION(expression, ...) RCLCPP_DEBUG_EXPRESSION(get_logger(), expression, __VA_ARGS__)  // NOLINT
#define EZ_DEBUG_FUNCTION(function, ...) RCLCPP_DEBUG_FUNCTION(get_logger(), function, __VA_ARGS__)  // NOLINT
#define EZ_DEBUG_SKIPFIRST(...) RCLCPP_DEBUG_SKIPFIRST(get_logger(), __VA_ARGS__)  // NOLINT
#define EZ_DEBUG_THROTTLE(duration_ms, ...) RCLCPP_DEBUG_THROTTLE(get_logger(), get_clock(), duration_ms, __VA_ARGS__)  // NOLINT
#define EZ_DEBUG_SKIPFIRST_THROTTLE(duration_ms, ...) RCLCPP_DEBUG_SKIPFIRST_THROTTLE(get_logger(), get_clock(), duration_ms, __VA_ARGS__)  // NOLINT

#define EZ_DEBUG_STREAM(stream_arg) RCLCPP_DEBUG_STREAM(get_logger(), stream_arg)  // NOLINT
#define EZ_DEBUG_STREAM_ONCE(stream_arg) RCLCPP_DEBUG_STREAM_ONCE(get_logger(), stream_arg)  // NOLINT
#define EZ_DEBUG_STREAM_EXPRESSION(expression, stream_arg) RCLCPP_DEBUG_STREAM_EXPRESSION(get_logger(), expression, stream_arg)  // NOLINT
#define EZ_DEBUG_STREAM_FUNCTION(function, stream_arg) RCLCPP_DEBUG_STREAM_FUNCTION(get_logger(), function, stream_arg)  // NOLINT
#define EZ_DEBUG_STREAM_SKIPFIRST(stream_arg) RCLCPP_DEBUG_STREAM_SKIPFIRST(get_logger(), stream_arg)  // NOLINT
#define EZ_DEBUG_STREAM_THROTTLE(duration_ms, stream_arg) RCLCPP_DEBUG_STREAM_THROTTLE(get_logger(), get_clock(), duration_ms, stream_arg)  // NOLINT
#define EZ_DEBUG_STREAM_SKIPFIRST_THROTTLE(duration_ms, stream_arg) RCLCPP_DEBUG_STREAM_SKIPFIRST_THROTTLE(get_logger(), get_clock(), duration_ms, stream_arg)  // NOLINT

//======================================================================
// INFO
//======================================================================
#define RJ_INFO(logger, ...) RCLCPP_INFO(logger, __VA_ARGS__)  // NOLINT
#define RJ_INFO_ONCE(logger, ...) RCLCPP_INFO_ONCE(logger, __VA_ARGS__)  // NOLINT
#define RJ_INFO_EXPRESSION(logger, expression, ...) RCLCPP_INFO_EXPRESSION(logger, expression, __VA_ARGS__)  // NOLINT
#define RJ_INFO_FUNCTION(logger, function, ...) RCLCPP_INFO_FUNCTION(logger, function, __VA_ARGS__)  // NOLINT
#define RJ_INFO_SKIPFIRST(logger, ...) RCLCPP_INFO_SKIPFIRST(logger, __VA_ARGS__)  // NOLINT
#define RJ_INFO_THROTTLE(logger, clock, duration_ms, ...) RCLCPP_INFO_THROTTLE(logger, clock, duration_ms, __VA_ARGS__)  // NOLINT
#define RJ_INFO_SKIPFIRST_THROTTLE(logger, clock, duration_ms, ...) RCLCPP_INFO_SKIPFIRST_THROTTLE(logger, clock, duration_ms, __VA_ARGS__)  // NOLINT

#define RJ_INFO_STREAM(logger, stream_arg) RCLCPP_INFO_STREAM(logger, stream_arg)  // NOLINT
#define RJ_INFO_STREAM_ONCE(logger, stream_arg) RCLCPP_INFO_STREAM_ONCE(logger, stream_arg)  // NOLINT
#define RJ_INFO_STREAM_EXPRESSION(logger, expression, stream_arg) RCLCPP_INFO_STREAM_EXPRESSION(logger, expression, stream_arg)  // NOLINT
#define RJ_INFO_STREAM_FUNCTION(logger, function, stream_arg) RCLCPP_INFO_STREAM_FUNCTION(logger, function, stream_arg)  // NOLINT
#define RJ_INFO_STREAM_SKIPFIRST(logger, stream_arg) RCLCPP_INFO_STREAM_SKIPFIRST(logger, stream_arg)  // NOLINT
#define RJ_INFO_STREAM_THROTTLE(logger, clock, duration_ms, stream_arg) RCLCPP_INFO_STREAM_THROTTLE(logger, clock, duration_ms, stream_arg)  // NOLINT
#define RJ_INFO_STREAM_SKIPFIRST_THROTTLE(logger, clock, duration_ms, stream_arg) RCLCPP_INFO_STREAM_SKIPFIRST_THROTTLE(logger, clock, duration_ms, stream_arg)  // NOLINT

#define EZ_INFO(...) RCLCPP_INFO(get_logger(), __VA_ARGS__)  // NOLINT
#define EZ_INFO_ONCE(...) RCLCPP_INFO_ONCE(get_logger(), __VA_ARGS__)  // NOLINT
#define EZ_INFO_EXPRESSION(expression, ...) RCLCPP_INFO_EXPRESSION(get_logger(), expression, __VA_ARGS__)  // NOLINT
#define EZ_INFO_FUNCTION(function, ...) RCLCPP_INFO_FUNCTION(get_logger(), function, __VA_ARGS__)  // NOLINT
#define EZ_INFO_SKIPFIRST(...) RCLCPP_INFO_SKIPFIRST(get_logger(), __VA_ARGS__)  // NOLINT
#define EZ_INFO_THROTTLE(duration_ms, ...) RCLCPP_INFO_THROTTLE(get_logger(), get_clock(), duration_ms, __VA_ARGS__)  // NOLINT
#define EZ_INFO_SKIPFIRST_THROTTLE(duration_ms, ...) RCLCPP_INFO_SKIPFIRST_THROTTLE(get_logger(), get_clock(), duration_ms, __VA_ARGS__)  // NOLINT

#define EZ_INFO_STREAM(stream_arg) RCLCPP_INFO_STREAM(get_logger(), stream_arg)  // NOLINT
#define EZ_INFO_STREAM_ONCE(stream_arg) RCLCPP_INFO_STREAM_ONCE(get_logger(), stream_arg)  // NOLINT
#define EZ_INFO_STREAM_EXPRESSION(expression, stream_arg) RCLCPP_INFO_STREAM_EXPRESSION(get_logger(), expression, stream_arg)  // NOLINT
#define EZ_INFO_STREAM_FUNCTION(function, stream_arg) RCLCPP_INFO_STREAM_FUNCTION(get_logger(), function, stream_arg)  // NOLINT
#define EZ_INFO_STREAM_SKIPFIRST(stream_arg) RCLCPP_INFO_STREAM_SKIPFIRST(get_logger(), stream_arg)  // NOLINT
#define EZ_INFO_STREAM_THROTTLE(duration_ms, stream_arg) RCLCPP_INFO_STREAM_THROTTLE(get_logger(), get_clock(), duration_ms, stream_arg)  // NOLINT
#define EZ_INFO_STREAM_SKIPFIRST_THROTTLE(duration_ms, stream_arg) RCLCPP_INFO_STREAM_SKIPFIRST_THROTTLE(get_logger(), get_clock(), duration_ms, stream_arg)  // NOLINT

//======================================================================
// WARN
//======================================================================
#define RJ_WARN(logger, ...) RCLCPP_WARN(logger, __VA_ARGS__)  // NOLINT
#define RJ_WARN_ONCE(logger, ...) RCLCPP_WARN_ONCE(logger, __VA_ARGS__)  // NOLINT
#define RJ_WARN_EXPRESSION(logger, expression, ...) RCLCPP_WARN_EXPRESSION(logger, expression, __VA_ARGS__)  // NOLINT
#define RJ_WARN_FUNCTION(logger, function, ...) RCLCPP_WARN_FUNCTION(logger, function, __VA_ARGS__)  // NOLINT
#define RJ_WARN_SKIPFIRST(logger, ...) RCLCPP_WARN_SKIPFIRST(logger, __VA_ARGS__)  // NOLINT
#define RJ_WARN_THROTTLE(logger, clock, duration_ms, ...) RCLCPP_WARN_THROTTLE(logger, clock, duration_ms, __VA_ARGS__)  // NOLINT
#define RJ_WARN_SKIPFIRST_THROTTLE(logger, clock, duration_ms, ...) RCLCPP_WARN_SKIPFIRST_THROTTLE(logger, clock, duration_ms, __VA_ARGS__)  // NOLINT

#define RJ_WARN_STREAM(logger, stream_arg) RCLCPP_WARN_STREAM(logger, stream_arg)  // NOLINT
#define RJ_WARN_STREAM_ONCE(logger, stream_arg) RCLCPP_WARN_STREAM_ONCE(logger, stream_arg)  // NOLINT
#define RJ_WARN_STREAM_EXPRESSION(logger, expression, stream_arg) RCLCPP_WARN_STREAM_EXPRESSION(logger, expression, stream_arg)  // NOLINT
#define RJ_WARN_STREAM_FUNCTION(logger, function, stream_arg) RCLCPP_WARN_STREAM_FUNCTION(logger, function, stream_arg)  // NOLINT
#define RJ_WARN_STREAM_SKIPFIRST(logger, stream_arg) RCLCPP_WARN_STREAM_SKIPFIRST(logger, stream_arg)  // NOLINT
#define RJ_WARN_STREAM_THROTTLE(logger, clock, duration_ms, stream_arg) RCLCPP_WARN_STREAM_THROTTLE(logger, clock, duration_ms, stream_arg)  // NOLINT
#define RJ_WARN_STREAM_SKIPFIRST_THROTTLE(logger, clock, duration_ms, stream_arg) RCLCPP_WARN_STREAM_SKIPFIRST_THROTTLE(logger, clock, duration_ms, stream_arg)  // NOLINT

#define EZ_WARN(...) RCLCPP_WARN(get_logger(), __VA_ARGS__)  // NOLINT
#define EZ_WARN_ONCE(...) RCLCPP_WARN_ONCE(get_logger(), __VA_ARGS__)  // NOLINT
#define EZ_WARN_EXPRESSION(expression, ...) RCLCPP_WARN_EXPRESSION(get_logger(), expression, __VA_ARGS__)  // NOLINT
#define EZ_WARN_FUNCTION(function, ...) RCLCPP_WARN_FUNCTION(get_logger(), function, __VA_ARGS__)  // NOLINT
#define EZ_WARN_SKIPFIRST(...) RCLCPP_WARN_SKIPFIRST(get_logger(), __VA_ARGS__)  // NOLINT
#define EZ_WARN_THROTTLE(duration_ms, ...) RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), duration_ms, __VA_ARGS__)  // NOLINT
#define EZ_WARN_SKIPFIRST_THROTTLE(duration_ms, ...) RCLCPP_WARN_SKIPFIRST_THROTTLE(get_logger(), *get_clock(), duration_ms, __VA_ARGS__)  // NOLINT

#define EZ_WARN_STREAM(stream_arg) RCLCPP_WARN_STREAM(get_logger(), stream_arg)  // NOLINT
#define EZ_WARN_STREAM_ONCE(stream_arg) RCLCPP_WARN_STREAM_ONCE(get_logger(), stream_arg)  // NOLINT
#define EZ_WARN_STREAM_EXPRESSION(expression, stream_arg) RCLCPP_WARN_STREAM_EXPRESSION(get_logger(), expression, stream_arg)  // NOLINT
#define EZ_WARN_STREAM_FUNCTION(function, stream_arg) RCLCPP_WARN_STREAM_FUNCTION(get_logger(), function, stream_arg)  // NOLINT
#define EZ_WARN_STREAM_SKIPFIRST(stream_arg) RCLCPP_WARN_STREAM_SKIPFIRST(get_logger(), stream_arg)  // NOLINT
#define EZ_WARN_STREAM_THROTTLE(duration_ms, stream_arg) RCLCPP_WARN_STREAM_THROTTLE(get_logger(), *get_clock(), duration_ms, stream_arg)  // NOLINT
#define EZ_WARN_STREAM_SKIPFIRST_THROTTLE(duration_ms, stream_arg) RCLCPP_WARN_STREAM_SKIPFIRST_THROTTLE(get_logger(), *get_clock(), duration_ms, stream_arg)  // NOLINT

//======================================================================
// ERROR
//======================================================================
#define RJ_ERROR(logger, ...) RCLCPP_ERROR(logger, __VA_ARGS__)  // NOLINT
#define RJ_ERROR_ONCE(logger, ...) RCLCPP_ERROR_ONCE(logger, __VA_ARGS__)  // NOLINT
#define RJ_ERROR_EXPRESSION(logger, expression, ...) RCLCPP_ERROR_EXPRESSION(logger, expression, __VA_ARGS__)  // NOLINT
#define RJ_ERROR_FUNCTION(logger, function, ...) RCLCPP_ERROR_FUNCTION(logger, function, __VA_ARGS__)  // NOLINT
#define RJ_ERROR_SKIPFIRST(logger, ...) RCLCPP_ERROR_SKIPFIRST(logger, __VA_ARGS__)  // NOLINT
#define RJ_ERROR_THROTTLE(logger, clock, duration_ms, ...) RCLCPP_ERROR_THROTTLE(logger, clock, duration_ms, __VA_ARGS__)  // NOLINT
#define RJ_ERROR_SKIPFIRST_THROTTLE(logger, clock, duration_ms, ...) RCLCPP_ERROR_SKIPFIRST_THROTTLE(logger, clock, duration_ms, __VA_ARGS__)  // NOLINT

#define RJ_ERROR_STREAM(logger, stream_arg) RCLCPP_ERROR_STREAM(logger, stream_arg)  // NOLINT
#define RJ_ERROR_STREAM_ONCE(logger, stream_arg) RCLCPP_ERROR_STREAM_ONCE(logger, stream_arg)  // NOLINT
#define RJ_ERROR_STREAM_EXPRESSION(logger, expression, stream_arg) RCLCPP_ERROR_STREAM_EXPRESSION(logger, expression, stream_arg)  // NOLINT
#define RJ_ERROR_STREAM_FUNCTION(logger, function, stream_arg) RCLCPP_ERROR_STREAM_FUNCTION(logger, function, stream_arg)  // NOLINT
#define RJ_ERROR_STREAM_SKIPFIRST(logger, stream_arg) RCLCPP_ERROR_STREAM_SKIPFIRST(logger, stream_arg)  // NOLINT
#define RJ_ERROR_STREAM_THROTTLE(logger, clock, duration_ms, stream_arg) RCLCPP_ERROR_STREAM_THROTTLE(logger, clock, duration_ms, stream_arg)  // NOLINT
#define RJ_ERROR_STREAM_SKIPFIRST_THROTTLE(logger, clock, duration_ms, stream_arg) RCLCPP_ERROR_STREAM_SKIPFIRST_THROTTLE(logger, clock, duration_ms, stream_arg)  // NOLINT

#define EZ_ERROR(...) RCLCPP_ERROR(get_logger(), __VA_ARGS__)  // NOLINT
#define EZ_ERROR_ONCE(...) RCLCPP_ERROR_ONCE(get_logger(), __VA_ARGS__)  // NOLINT
#define EZ_ERROR_EXPRESSION(expression, ...) RCLCPP_ERROR_EXPRESSION(get_logger(), expression, __VA_ARGS__)  // NOLINT
#define EZ_ERROR_FUNCTION(function, ...) RCLCPP_ERROR_FUNCTION(get_logger(), function, __VA_ARGS__)  // NOLINT
#define EZ_ERROR_SKIPFIRST(...) RCLCPP_ERROR_SKIPFIRST(get_logger(), __VA_ARGS__)  // NOLINT
#define EZ_ERROR_THROTTLE(duration_ms, ...) RCLCPP_ERROR_THROTTLE(get_logger(), get_clock(), duration_ms, __VA_ARGS__)  // NOLINT
#define EZ_ERROR_SKIPFIRST_THROTTLE(duration_ms, ...) RCLCPP_ERROR_SKIPFIRST_THROTTLE(get_logger(), get_clock(), duration_ms, __VA_ARGS__)  // NOLINT

#define EZ_ERROR_STREAM(stream_arg) RCLCPP_ERROR_STREAM(get_logger(), stream_arg)  // NOLINT
#define EZ_ERROR_STREAM_ONCE(stream_arg) RCLCPP_ERROR_STREAM_ONCE(get_logger(), stream_arg)  // NOLINT
#define EZ_ERROR_STREAM_EXPRESSION(expression, stream_arg) RCLCPP_ERROR_STREAM_EXPRESSION(get_logger(), expression, stream_arg)  // NOLINT
#define EZ_ERROR_STREAM_FUNCTION(function, stream_arg) RCLCPP_ERROR_STREAM_FUNCTION(get_logger(), function, stream_arg)  // NOLINT
#define EZ_ERROR_STREAM_SKIPFIRST(stream_arg) RCLCPP_ERROR_STREAM_SKIPFIRST(get_logger(), stream_arg)  // NOLINT
#define EZ_ERROR_STREAM_THROTTLE(duration_ms, stream_arg) RCLCPP_ERROR_STREAM_THROTTLE(get_logger(), get_clock(), duration_ms, stream_arg)  // NOLINT
#define EZ_ERROR_STREAM_SKIPFIRST_THROTTLE(duration_ms, stream_arg) RCLCPP_ERROR_STREAM_SKIPFIRST_THROTTLE(get_logger(), get_clock(), duration_ms, stream_arg)  // NOLINT

//======================================================================
// FATAL
//======================================================================
#define RJ_FATAL(logger, ...) RCLCPP_FATAL(logger, __VA_ARGS__)  // NOLINT
#define RJ_FATAL_ONCE(logger, ...) RCLCPP_FATAL_ONCE(logger, __VA_ARGS__)  // NOLINT
#define RJ_FATAL_EXPRESSION(logger, expression, ...) RCLCPP_FATAL_EXPRESSION(logger, expression, __VA_ARGS__)  // NOLINT
#define RJ_FATAL_FUNCTION(logger, function, ...) RCLCPP_FATAL_FUNCTION(logger, function, __VA_ARGS__)  // NOLINT
#define RJ_FATAL_SKIPFIRST(logger, ...) RCLCPP_FATAL_SKIPFIRST(logger, __VA_ARGS__)  // NOLINT
#define RJ_FATAL_THROTTLE(logger, clock, duration_ms, ...) RCLCPP_FATAL_THROTTLE(logger, clock, duration_ms, __VA_ARGS__)  // NOLINT
#define RJ_FATAL_SKIPFIRST_THROTTLE(logger, clock, duration_ms, ...) RCLCPP_FATAL_SKIPFIRST_THROTTLE(logger, clock, duration_ms, __VA_ARGS__)  // NOLINT

#define RJ_FATAL_STREAM(logger, stream_arg) RCLCPP_FATAL_STREAM(logger, stream_arg)  // NOLINT
#define RJ_FATAL_STREAM_ONCE(logger, stream_arg) RCLCPP_FATAL_STREAM_ONCE(logger, stream_arg)  // NOLINT
#define RJ_FATAL_STREAM_EXPRESSION(logger, expression, stream_arg) RCLCPP_FATAL_STREAM_EXPRESSION(logger, expression, stream_arg)  // NOLINT
#define RJ_FATAL_STREAM_FUNCTION(logger, function, stream_arg) RCLCPP_FATAL_STREAM_FUNCTION(logger, function, stream_arg)  // NOLINT
#define RJ_FATAL_STREAM_SKIPFIRST(logger, stream_arg) RCLCPP_FATAL_STREAM_SKIPFIRST(logger, stream_arg)  // NOLINT
#define RJ_FATAL_STREAM_THROTTLE(logger, clock, duration_ms, stream_arg) RCLCPP_FATAL_STREAM_THROTTLE(logger, clock, duration_ms, stream_arg)  // NOLINT
#define RJ_FATAL_STREAM_SKIPFIRST_THROTTLE(logger, clock, duration_ms, stream_arg) RCLCPP_FATAL_STREAM_SKIPFIRST_THROTTLE(logger, clock, duration_ms, stream_arg)  // NOLINT

#define EZ_FATAL(...) RCLCPP_FATAL(get_logger(), __VA_ARGS__)  // NOLINT
#define EZ_FATAL_ONCE(...) RCLCPP_FATAL_ONCE(get_logger(), __VA_ARGS__)  // NOLINT
#define EZ_FATAL_EXPRESSION(expression, ...) RCLCPP_FATAL_EXPRESSION(get_logger(), expression, __VA_ARGS__)  // NOLINT
#define EZ_FATAL_FUNCTION(function, ...) RCLCPP_FATAL_FUNCTION(get_logger(), function, __VA_ARGS__)  // NOLINT
#define EZ_FATAL_SKIPFIRST(...) RCLCPP_FATAL_SKIPFIRST(get_logger(), __VA_ARGS__)  // NOLINT
#define EZ_FATAL_THROTTLE(duration_ms, ...) RCLCPP_FATAL_THROTTLE(get_logger(), get_clock(), duration_ms, __VA_ARGS__)  // NOLINT
#define EZ_FATAL_SKIPFIRST_THROTTLE(duration_ms, ...) RCLCPP_FATAL_SKIPFIRST_THROTTLE(get_logger(), get_clock(), duration_ms, __VA_ARGS__)  // NOLINT

#define EZ_FATAL_STREAM(stream_arg) RCLCPP_FATAL_STREAM(get_logger(), stream_arg)  // NOLINT
#define EZ_FATAL_STREAM_ONCE(stream_arg) RCLCPP_FATAL_STREAM_ONCE(get_logger(), stream_arg)  // NOLINT
#define EZ_FATAL_STREAM_EXPRESSION(expression, stream_arg) RCLCPP_FATAL_STREAM_EXPRESSION(get_logger(), expression, stream_arg)  // NOLINT
#define EZ_FATAL_STREAM_FUNCTION(function, stream_arg) RCLCPP_FATAL_STREAM_FUNCTION(get_logger(), function, stream_arg)  // NOLINT
#define EZ_FATAL_STREAM_SKIPFIRST(stream_arg) RCLCPP_FATAL_STREAM_SKIPFIRST(get_logger(), stream_arg)  // NOLINT
#define EZ_FATAL_STREAM_THROTTLE(duration_ms, stream_arg) RCLCPP_FATAL_STREAM_THROTTLE(get_logger(), get_clock(), duration_ms, stream_arg)  // NOLINT
#define EZ_FATAL_STREAM_SKIPFIRST_THROTTLE(duration_ms, stream_arg) RCLCPP_FATAL_STREAM_SKIPFIRST_THROTTLE(get_logger(), get_clock(), duration_ms, stream_arg)  // NOLINT
