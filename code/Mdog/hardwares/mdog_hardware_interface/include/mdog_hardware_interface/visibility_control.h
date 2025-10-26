// 版权所有 (c) 2022，Stogl Robotics Consulting UG (haftungsbeschränkt)（模板）
//
// 根据 Apache License 2.0 版（下称“许可证”）授权；
// 未遵循许可证条款前不得使用本文件。
// 可在以下地址获取许可证副本：
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// 除非适用法律要求或书面同意，否则依据许可证分发的软件
// 均以“按原样”方式提供，不附带任何明示或暗示的担保或条件。
// 更多权限与限制条款请参阅许可证文本。

#ifndef MDOG_HARDWARE_INTERFACE__VISIBILITY_CONTROL_H_
#define MDOG_HARDWARE_INTERFACE__VISIBILITY_CONTROL_H_

// 该逻辑借鉴并命名空间化自 GCC Wiki 的示例：
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define TEMPLATES__ROS2_CONTROL__VISIBILITY_EXPORT __attribute__((dllexport))
#define TEMPLATES__ROS2_CONTROL__VISIBILITY_IMPORT __attribute__((dllimport))
#else
#define TEMPLATES__ROS2_CONTROL__VISIBILITY_EXPORT __declspec(dllexport)
#define TEMPLATES__ROS2_CONTROL__VISIBILITY_IMPORT __declspec(dllimport)
#endif
#ifdef TEMPLATES__ROS2_CONTROL__VISIBILITY_BUILDING_DLL
#define TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC TEMPLATES__ROS2_CONTROL__VISIBILITY_EXPORT
#else
#define TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC TEMPLATES__ROS2_CONTROL__VISIBILITY_IMPORT
#endif
#define TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC_TYPE TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
#define TEMPLATES__ROS2_CONTROL__VISIBILITY_LOCAL
#else
#define TEMPLATES__ROS2_CONTROL__VISIBILITY_EXPORT __attribute__((visibility("default")))
#define TEMPLATES__ROS2_CONTROL__VISIBILITY_IMPORT
#if __GNUC__ >= 4
#define TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC __attribute__((visibility("default")))
#define TEMPLATES__ROS2_CONTROL__VISIBILITY_LOCAL __attribute__((visibility("hidden")))
#else
#define TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
#define TEMPLATES__ROS2_CONTROL__VISIBILITY_LOCAL
#endif
#define TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC_TYPE
#endif

#endif  // MDOG_HARDWARE_INTERFACE__VISIBILITY_CONTROL_H_
