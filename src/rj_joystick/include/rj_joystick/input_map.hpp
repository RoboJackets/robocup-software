#pragma once

namespace joystick {

namespace button {
inline constexpr size_t kA             = 0;
inline constexpr size_t kB             = 1;
inline constexpr size_t kX             = 2;
inline constexpr size_t kY             = 3;
inline constexpr size_t kBack          = 4;
inline constexpr size_t kGuide         = 5;
inline constexpr size_t kStart         = 6;
inline constexpr size_t kLeftStick     = 7;
inline constexpr size_t kRightStick    = 8;
inline constexpr size_t kLeftShoulder  = 9;
inline constexpr size_t kRightShoulder = 10;
inline constexpr size_t kDpadUp        = 11;
inline constexpr size_t kDpadDown      = 12;
inline constexpr size_t kDpadLeft      = 13;
inline constexpr size_t kDpadRight     = 14;
inline constexpr size_t kMisc1         = 15;
inline constexpr size_t kPaddle1       = 16;
inline constexpr size_t kPaddle2       = 17;
inline constexpr size_t kPaddle3       = 18;
inline constexpr size_t kPaddle4       = 19;
inline constexpr size_t kTouchpad      = 20;
}  // namespace button

namespace axis {
inline constexpr size_t kLeftX        = 0;
inline constexpr size_t kLeftY        = 1;
inline constexpr size_t kRightX       = 2;
inline constexpr size_t kRightY       = 3;
inline constexpr size_t kTriggerLeft  = 4;
inline constexpr size_t kTriggerRight = 5;
}  // namespace axis

}  // namespace joystick