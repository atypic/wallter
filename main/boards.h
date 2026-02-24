#ifndef BOARDS_H
#define BOARDS_H

#include "motordriver.hpp"

// Unified board selection mechanism.
// Define one of the legacy BOARD_* macros OR define BOARD_TYPE to one of the
// BOARD_TYPE_* constants below. Legacy defines are mapped to BOARD_TYPE for
// backward compatibility.

#define BOARD_TYPE_LEANGEN_4_MOTOR 1
#define BOARD_TYPE_ARCTIC_WITH_KEYPAD 2
#define BOARD_TYPE_ARCTIC_CYTRON 3
#define BOARD_TYPE_SLUPPEN_PROTO 4

// Select board via unified BOARD_TYPE macro (see boards.h for constants)
// #define BOARD_TYPE BOARD_TYPE_LEANGEN_4_MOTOR
// #define BOARD_TYPE BOARD_TYPE_ARCTIC_WITH_KEYPAD
// #define BOARD_TYPE BOARD_TYPE_ARCTIC_CYTRON

// -----------------------------------------------------------------------------
// LEANGEN 4 MOTOR
// -----------------------------------------------------------------------------
#if defined(BOARD_TYPE) && (BOARD_TYPE == BOARD_TYPE_LEANGEN_4_MOTOR)
bool g_has_keypad = false;

#define LOWEST_ANGLE 10
#define HIGHEST_ANGLE 70
#define ANGLE_STEP 5

#define MASTER_MOTOR 0

#define MINSPEED (0)
#define MAXSPEED (255)
#define MASTER_MAX (MAXSPEED - 30)
#define ACCEL_STEP 10

#define NUM_MOTORS 4

#define TICK_TO_M 0.0001
#define HINGE_TO_ACT_MOUNT_REAR 0.45
#define HINGE_TO_ACT_MOUNT_FRONT 4.2

#define VERSION_STRING "FW: GRIP_R_CYTRON"

#define LOCK_TIME_MS 10 * 60

unsigned int HAL_CNT[NUM_MOTORS] = {22, 24, 26, 28};
unsigned int HAL_CLK[NUM_MOTORS] = {23, 25, 27, 29};

#define BUTTON_EXTEND_PIN 52
#define BUTTON_RETRACT_PIN 53

CytronMD motors[NUM_MOTORS] = {CytronMD(PWM_DIR, 3, 36),  // MASTER
                               CytronMD(PWM_DIR, 2, 34),
                               CytronMD(PWM_DIR, 5, 32),
                               CytronMD(PWM_DIR, 4, 30)};
#endif

// -----------------------------------------------------------------------------
// ARCTIC WITH KEYPAD
// -----------------------------------------------------------------------------
#if defined(BOARD_TYPE) && (BOARD_TYPE == BOARD_TYPE_ARCTIC_WITH_KEYPAD)

#define LOWEST_ANGLE 10
#define HIGHEST_ANGLE 60
#define ANGLE_STEP 5

#define MASTER_MOTOR 0

#define ACCEL_STEP 1
#define MINSPEED (249)
#define MAXSPEED (255)
#define MASTER_MAX (MAXSPEED - 1)

#define NUM_MOTORS 2

#define TICK_TO_M 0.0001
#define HINGE_TO_ACT_MOUNT_REAR 0.45
#define HINGE_TO_ACT_MOUNT_FRONT 4.2

#define VERSION_STRING "FW: ARCTIC_3_KP"

#define LOCK_TIME_MS 10 * 60

bool g_has_keypad = true;

//                          master: 0  slv:  1
unsigned int HAL_CNT[NUM_MOTORS] = {30, 31};  // black wires
unsigned int HAL_CLK[NUM_MOTORS] = {28, 29};  // white wires

#define BUTTON_EXTEND_PIN 53
#define BUTTON_RETRACT_PIN 52

CytronMD motors[NUM_MOTORS] = {CytronMD(PWM_DIR, 2, 22),  // MASTER
                               CytronMD(PWM_DIR, 3, 23)};
#endif

// -----------------------------------------------------------------------------
// ARCTIC CYTRON (ESP-IDF GPIO mapping)
// -----------------------------------------------------------------------------
#if defined(BOARD_TYPE) && (BOARD_TYPE == BOARD_TYPE_ARCTIC_CYTRON)

static const bool g_has_keypad = false;

#define LOWEST_ANGLE 10
#define HIGHEST_ANGLE 60
#define ANGLE_STEP 5

// Default calibration range (used when NVS has no calibration meta stored).
// We only use 30..60 by default on this hardware.
#define DEFAULT_CAL_MIN_ANGLE 30
#define DEFAULT_CAL_MAX_ANGLE 60

#define MASTER_MOTOR 0

#define ACCEL_STEP 1
#define MINSPEED (250)
#define MAXSPEED (253)
#define MASTER_MAX (MAXSPEED - 1)

#define NUM_MOTORS 2

#define TICK_TO_M 0.0001
#define HINGE_TO_ACT_MOUNT_REAR 0.45
#define HINGE_TO_ACT_MOUNT_FRONT 4.2

#define VERSION_STRING "FW: 07-2025"

#define LOCK_TIME_MS 10 * 60

// LCD I2C configuration (single source of truth)
#define LCD_SDA_PIN 7
#define LCD_SCL_PIN 6
#define LCD_I2C_CLOCK_HZ 50000

// HAL encoder pin mapping varies across board spins.
// Select the pinset here to match your wiring.
// - 1: legacy pinout (matches commit 9794e3c)
// - 2: newer pinout (latest board spin)
#ifndef ARCTIC_CYTRON_ENCODER_PINSET
#define ARCTIC_CYTRON_ENCODER_PINSET 2
#endif

#if ARCTIC_CYTRON_ENCODER_PINSET == 1
// Legacy (commit 9794e3c):
// Motor 0: HAL_CLK=GPIO12, HAL_CNT=GPIO11
// Motor 1: HAL_CLK=GPIO10, HAL_CNT=GPIO9
static const unsigned int HAL_CLK[NUM_MOTORS] = {12, 10};
static const unsigned int HAL_CNT[NUM_MOTORS] = {11, 9};
#else
// Newer board spin:
// Motor 0: HAL_CLK=GPIO46, HAL_CNT=GPIO9
// Motor 1: HAL_CLK=GPIO10, HAL_CNT=GPIO11
static const unsigned int HAL_CLK[NUM_MOTORS] = {46, 10};
static const unsigned int HAL_CNT[NUM_MOTORS] = {9, 11};
#endif

// If you accidentally swapped the HAL encoder signals when soldering (CLK<->CNT),
// set this to 1 and reflash.
#define HAL_AB_SWAPPED 0

// If HAL_CNT (direction) is floating/miswired, the direction bit will look random
// and you'll accumulate both stepsIn and stepsOut during a single move.
// Set this to 1 to derive direction from the commanded motor direction instead.
#define HAL_DIR_FROM_MOTOR 0

// Optional: reject encoder edges whose sensed direction disagrees with the
// commanded motor direction. Helps when HAL_CNT is noisy.
#define HAL_VALIDATE_DIR_WITH_MOTOR 0

// Minimum HAL_CLK pulse width to accept as a real tick.
// This mirrors the old Arduino Due behavior where we rejected narrow spikes
// (e.g. `setDebounce(HAL_CLK, 1000)` for ~1ms).
#define HAL_CLK_MIN_PULSE_US 1000

// The HAL_CNT polarity appears inverted on this board spin:
// sampling HAL_CNT on HAL_CLK rising yields 0/1 swapped vs expected direction.
#define HAL_CNT_INVERT 1

// Button pins (swapped on hardware: EXTEND/RETRACT are flipped)
#define BUTTON_EXTEND_PIN 47
#define BUTTON_RETRACT_PIN 21

// Motor driver pins (PWM/DIR)
// Motor 0: PWM=GPIO16, DIR=GPIO17
// Motor 1: PWM=GPIO18, DIR=GPIO8
static const unsigned int HAL_PWM[NUM_MOTORS] = {16, 18};
static const unsigned int HAL_DIR[NUM_MOTORS] = {17, 8};

// Motor output inversion.
// If homing drives the wrong physical direction, set this to 1.
#define MOTOR_OUTPUT_INVERT 0

#endif

// Default: motor output sign is not inverted.
#ifndef MOTOR_OUTPUT_INVERT
#define MOTOR_OUTPUT_INVERT 0
#endif

// Default calibration meta fallbacks (board may override above).
#ifndef DEFAULT_CAL_MIN_ANGLE
#define DEFAULT_CAL_MIN_ANGLE LOWEST_ANGLE
#endif

#ifndef DEFAULT_CAL_MAX_ANGLE
#define DEFAULT_CAL_MAX_ANGLE HIGHEST_ANGLE
#endif

// Default: do not invert HAL_CNT unless a board override says so.
#ifndef HAL_CNT_INVERT
#define HAL_CNT_INVERT 0
#endif

// Default: HAL encoder channels are not swapped.
#ifndef HAL_AB_SWAPPED
#define HAL_AB_SWAPPED 0
#endif

// Default: derive direction from HAL_CNT sampling.
#ifndef HAL_DIR_FROM_MOTOR
#define HAL_DIR_FROM_MOTOR 0
#endif

// Default: do not validate HAL direction vs motor command.
#ifndef HAL_VALIDATE_DIR_WITH_MOTOR
#define HAL_VALIDATE_DIR_WITH_MOTOR 0
#endif

// Default: accept all HAL_CLK pulses (no width filter).
#ifndef HAL_CLK_MIN_PULSE_US
#define HAL_CLK_MIN_PULSE_US 0
#endif

// -----------------------------------------------------------------------------
// SLUPPEN PROTO (placeholder)
// -----------------------------------------------------------------------------
#if defined(BOARD_TYPE) && (BOARD_TYPE == BOARD_TYPE_SLUPPEN_PROTO)
// Add configuration here when available.
bool g_has_keypad = false;

#define LOWEST_ANGLE 30
#define HIGHEST_ANGLE 70
#define ANGLE_STEP 5

#define MASTER_MOTOR 0
#define MINSPEED (0)
#define MAXSPEED (250)
#define MASTER_MAX (MAXSPEED - 30)
#define ACCEL_STEP 10

#define NUM_MOTORS 4

#define TICK_TO_M 0.0001
#define HINGE_TO_ACT_MOUNT_REAR 0.45
#define HINGE_TO_ACT_MOUNT_FRONT 4.2

#define VERSION_STRING "SLUPPEN_001"

#define LOCK_TIME_MS 10 * 60

unsigned int HAL_CNT[NUM_MOTORS] = {28, 26, 24, 22};
unsigned int HAL_CLK[NUM_MOTORS] = {29, 27, 25, 23};

#define BUTTON_EXTEND_PIN 51
#define BUTTON_RETRACT_PIN 50

MotorDriver motors[NUM_MOTORS] = {MotorDriver(13, 2, 3),  // #1
                                  MotorDriver(12, 4, 5),  // #2
                                  MotorDriver(11, 8, 9),
                                  MotorDriver(10, 6, 7)};
#endif

#endif  // BOARDS_H