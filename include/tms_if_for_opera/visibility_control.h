#ifndef TMS_IF_FOR_OPERA__VISIBILITY_CONTROL_H_
#define TMS_IF_FOR_OPERA__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C" {
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define TMS_IF_FOR_OPERA_EXPORT __attribute__((dllexport))
#define TMS_IF_FOR_OPERA_IMPORT __attribute__((dllimport))
#else
#define TMS_IF_FOR_OPERA_EXPORT __declspec(dllexport)
#define TMS_IF_FOR_OPERA_IMPORT __declspec(dllimport)
#endif
#ifdef TMS_IF_FOR_OPERA_BUILDING_DLL
#define TMS_IF_FOR_OPERA_PUBLIC TMS_IF_FOR_OPERA_EXPORT
#else
#define TMS_IF_FOR_OPERA_PUBLIC TMS_IF_FOR_OPERA_IMPORT
#endif
#define TMS_IF_FOR_OPERA_PUBLIC_TYPE TMS_IF_FOR_OPERA_PUBLIC
#define TMS_IF_FOR_OPERA_LOCAL
#else
#define TMS_IF_FOR_OPERA_EXPORT __attribute__((visibility("default")))
#define TMS_IF_FOR_OPERA_IMPORT
#if __GNUC__ >= 4
#define TMS_IF_FOR_OPERA_PUBLIC __attribute__((visibility("default")))
#define TMS_IF_FOR_OPERA_LOCAL __attribute__((visibility("hidden")))
#else
#define TMS_IF_FOR_OPERA_PUBLIC
#define TMS_IF_FOR_OPERA_LOCAL
#endif
#define TMS_IF_FOR_OPERA_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // TMS_IF_FOR_OPERA__VISIBILITY_CONTROL_H_