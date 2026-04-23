#pragma once

#include <stdint.h>

#include "calibration_store.hpp"

class Display;
class MotorDriver;

namespace wallter::modes {

enum BootMenuChoice {
    MENU_SET_MAX_ANGLE = 0,
    MENU_SET_MIN_ANGLE = 1,
    MENU_SET_OFFSET = 2,
    MENU_SELF_TEST = 3,
    MENU_JOG = 4,
    MENU_HAL_TEST = 5,
    MENU_RESET_CAL = 6,
    MENU_TEST_ACCEL = 7,
    MENU_CAL_ACCEL = 8,
};

struct Services {
    Display *display{};
    MotorDriver *motors{};
    int num_motors{};
    int master_motor_index{};

    int max_angles{};
    uint32_t *target_idx{};

    // Angle range limits (min is always the hard-stop; only max is user-configurable).
    wallter::calibration::CalMeta *cal_meta{};

    // Hardware button level reads (active-low -> true means pressed)
    bool (*read_extend_pressed)() = nullptr;
    bool (*read_retract_pressed)() = nullptr;

    // Motion primitives provided by the control layer
    void (*set_current_command)(int cmd) = nullptr;
    void (*motor_control_iteration)() = nullptr;
    void (*run_homing_blocking)(uint32_t timeout_ms) = nullptr;
    void (*reset_tick_counters)(uint32_t m, bool all) = nullptr;

    // Fatal error handler (must not return)
    void (*panicf)(const char *fmt, ...) = nullptr;

    // Time source
    uint64_t (*now_ms)() = nullptr;
};

BootMenuChoice run_boot_menu(Services &svc);
void run_set_max_angle_mode(Services &svc);
void run_set_min_angle_mode(Services &svc);
void run_set_angle_offset_mode(Services &svc);
void run_self_test_sequence(Services &svc);
void run_hal_feedback_test(Services &svc);
void run_reset_calibration_data(Services &svc);
void run_jog_mode(Services &svc);
void run_test_accel_mode(Services &svc);
void run_cal_accel_mode(Services &svc);

} // namespace wallter::modes
