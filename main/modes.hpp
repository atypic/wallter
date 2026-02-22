#pragma once

#include <stdint.h>

#include "calibration_store.hpp"

class Display;
class MotorDriver;

namespace wallter::modes {

enum BootMenuChoice { MENU_CALIBRATE = 0, MENU_SELF_TEST = 1 };

struct Services {
    Display *display{};
    MotorDriver *motors{};
    int num_motors{};
    int master_motor_index{};

    // Target tick table
    uint32_t *target_ticks{};
    int max_angles{};
    uint32_t *target_idx{};

    // Calibration metadata (min/max usable angles) and derived homing offset.
    wallter::calibration::CalMeta *cal_meta{};
    uint32_t *home_offset_raw_ticks{};

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
void run_calibration_mode(Services &svc);
void run_self_test_sequence(Services &svc);

} // namespace wallter::modes
