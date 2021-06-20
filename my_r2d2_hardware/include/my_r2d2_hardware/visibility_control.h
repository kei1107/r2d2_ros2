#ifndef MY_R2D2_HARDWARE__VISIBILITY_CONTROL_H_
#define MY_R2D2_HARDWARE__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define MY_R2D2_HARDWARE_EXPORT __attribute__((dllexport))
#define MY_R2D2_HARDWARE_IMPORT __attribute__((dllimport))
#else
#define MY_R2D2_HARDWARE_EXPORT __declspec(dllexport)
#define MY_R2D2_HARDWARE_IMPORT __declspec(dllimport)
#endif
#ifdef MY_R2D2_HARDWARE_BUILDING_DLL
#define MY_R2D2_HARDWARE_PUBLIC MY_R2D2_HARDWARE_EXPORT
#else
#define MY_R2D2_HARDWARE_PUBLIC MY_R2D2_HARDWARE_IMPORT
#endif
#define MY_R2D2_HARDWARE_PUBLIC_TYPE MY_R2D2_HARDWARE_PUBLIC
#define MY_R2D2_HARDWARE_LOCAL
#else
#define MY_R2D2_HARDWARE_EXPORT __attribute__((visibility("default")))
#define MY_R2D2_HARDWARE_IMPORT
#if __GNUC__ >= 4
#define MY_R2D2_HARDWARE_PUBLIC __attribute__((visibility("default")))
#define MY_R2D2_HARDWARE_LOCAL __attribute__((visibility("hidden")))
#else
#define MY_R2D2_HARDWARE_PUBLIC
#define MY_R2D2_HARDWARE_LOCAL
#endif
#define MY_R2D2_HARDWARE_PUBLIC_TYPE
#endif

#endif  // MY_R2D2_HARDWARE__VISIBILITY_CONTROL_H_