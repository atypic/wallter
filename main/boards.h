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
#define HIGHEST_ANGLE 60
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

// HAL encoder pins
static const unsigned int HAL_CLK[NUM_MOTORS] = {12, 10};
static const unsigned int HAL_CNT[NUM_MOTORS] = {11, 9};

// Button pins
#define BUTTON_EXTEND_PIN 21
#define BUTTON_RETRACT_PIN 47

// Motor driver pins (PWM/DIR)
static const unsigned int HAL_PWM[NUM_MOTORS] = {8, 3};
static const unsigned int HAL_DIR[NUM_MOTORS] = {17, 18};

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