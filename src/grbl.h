/* grbl.h - main grblHAL-style include for compile-time configuration
 *
 * This header is intended to be the single "front door" include for your core CNC
 * firmware. It centralizes build-time feature flags, limits, and module selection.
 *
 * Pattern:
 *   - Toolchain defines board/platform symbols (ex: GRBL_PLATFORM_STM32)
 *   - Optional user overrides live in grbl_config.h (ignored if not present)
 *   - This file sets sane defaults, then includes core module headers.
 *
 * Keep vendor SDK headers OUT of here. Use hal.h as the hardware boundary.
 */

#pragma once

/* ----------------------------- Versioning ----------------------------- */

#define GRBL_CORE_NAME            "grbl-core"
#define GRBL_CORE_VERSION_MAJOR   0
#define GRBL_CORE_VERSION_MINOR   1
#define GRBL_CORE_VERSION_PATCH   0

/* ----------------------------- Standard includes ----------------------------- */

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/* ----------------------------- User overrides ----------------------------- */
/* Create a grbl_config.h next to this file to override defaults per-project. */
#if defined(__has_include)
  #if __has_include("grbl_config.h")
    #include "grbl_config.h"
  #endif
#endif

/* ----------------------------- Platform selection ----------------------------- */
/* Your build system should define ONE of these (or none for a generic build). */
/* Examples:
 *   -DGRBL_PLATFORM_STM32
 *   -DGRBL_PLATFORM_LINUX_SIM
 */
#if !defined(GRBL_PLATFORM_STM32) && !defined(GRBL_PLATFORM_LINUX_SIM)
  #define GRBL_PLATFORM_GENERIC 1
#endif

/* ----------------------------- Core limits ----------------------------- */

#ifndef GRBL_CART_AXES
  #define GRBL_CART_AXES 3u
#endif

#ifndef GRBL_JOINT_AXES
  #define GRBL_JOINT_AXES 3u
#endif

#ifndef GRBL_LINE_MAX
  #define GRBL_LINE_MAX 96u   /* protocol line length */
#endif

#ifndef GRBL_LINE_QUEUE_DEPTH
  #define GRBL_LINE_QUEUE_DEPTH 8u
#endif

#ifndef GRBL_RX_CHUNK
  #define GRBL_RX_CHUNK 64u   /* how many bytes to read from HAL per poll */
#endif

/* ----------------------------- Feature flags ----------------------------- */
/* 0/1 toggles. Keep defaults minimal; enable more as you need them. */

#ifndef GRBL_FEATURE_STATUS_REPORTS
  #define GRBL_FEATURE_STATUS_REPORTS 1
#endif

#ifndef GRBL_FEATURE_REALTIME_CMDS
  #define GRBL_FEATURE_REALTIME_CMDS 1
#endif

#ifndef GRBL_FEATURE_HOMING
  #define GRBL_FEATURE_HOMING 1
#endif

#ifndef GRBL_FEATURE_LIMITS
  #define GRBL_FEATURE_LIMITS 1
#endif

#ifndef GRBL_FEATURE_PROBE
  #define GRBL_FEATURE_PROBE 0
#endif

#ifndef GRBL_FEATURE_COOLANT
  #define GRBL_FEATURE_COOLANT 0
#endif

#ifndef GRBL_FEATURE_SPINDLE_PWM
  #define GRBL_FEATURE_SPINDLE_PWM 1
#endif

#ifndef GRBL_FEATURE_CHECK_MODE
  #define GRBL_FEATURE_CHECK_MODE 0
#endif

#ifndef GRBL_FEATURE_JOG
  #define GRBL_FEATURE_JOG 0
#endif

#ifndef GRBL_FEATURE_SD_STREAM
  #define GRBL_FEATURE_SD_STREAM 0
#endif

/* ----------------------------- Module selection ----------------------------- */
/* Choose which kinematics implementation to compile in (one active at runtime). */

#ifndef GRBL_KINEMATICS_COREXY
  #define GRBL_KINEMATICS_COREXY 1
#endif

#ifndef GRBL_KINEMATICS_CARTESIAN
  #define GRBL_KINEMATICS_CARTESIAN 0
#endif

/* ----------------------------- Sanity checks ----------------------------- */

#if (GRBL_CART_AXES == 0u) || (GRBL_CART_AXES > 6u)
  #error "GRBL_CART_AXES must be 1..6"
#endif

#if (GRBL_LINE_MAX < 32u) || (GRBL_LINE_MAX > 256u)
  #error "GRBL_LINE_MAX must be 32..256"
#endif

#if (GRBL_LINE_QUEUE_DEPTH < 1u) || (GRBL_LINE_QUEUE_DEPTH > 32u)
  #error "GRBL_LINE_QUEUE_DEPTH must be 1..32"
#endif

/* Tie protocol limits to build-time config if you use those module headers. */
#ifndef PROTOCOL_LINE_MAX
  #define PROTOCOL_LINE_MAX GRBL_LINE_MAX
#endif

#ifndef PROTOCOL_LINE_QUEUE_DEPTH
  #define PROTOCOL_LINE_QUEUE_DEPTH GRBL_LINE_QUEUE_DEPTH
#endif

/* ----------------------------- Core includes ----------------------------- */
/* These are your project headers from earlier steps. Adjust paths as needed. */

#include "hal.h"
#include "protocol.h"
#include "kinematics.h"
#include "gcode.h"
#include "planner.h"
#include "stepper.h"

#if GRBL_KINEMATICS_COREXY
  #include "kin_corexy.h"
#endif

/* Add more core modules as you create them:
 *  - settings.h
 *  - report.h
 */

/* ----------------------------- Main entry points ----------------------------- */
/* These are optional but nice: a consistent top-level init + loop contract. */

#ifdef __cplusplus
extern "C" {
#endif

/* Call once at boot. Should initialize HAL + core modules. */
void grbl_init(void);

/* Call repeatedly in main while(1). Should poll HAL, protocol, planner, etc. */
void grbl_poll(void);

#ifdef __cplusplus
}
#endif