#pragma once

#include <stdint.h>

class Display;
class MotorDriver;

namespace wallter::control {

struct Context {
    Display *display{};
    MotorDriver *motors{};
    int num_motors{};
    int master_motor_index{};

    uint32_t *target_ticks{};
    int max_angles{};
    uint32_t *target_idx{};
};

void init(const Context &ctx);

// Motion/command state
void set_command(int cmd);
int command();

// Core periodic work
void iterate();
void print_state(uint32_t print_rate_ms);
void error_check_motor_positions();

// Homing sequence used by startup & calibration
void run_homing_blocking(uint32_t timeout_ms);

// Button-driven target selection logic (expects edge-like events)
void handle_buttons(bool extend_event, bool retract_event);

// Helpers used by the UI loop
uint8_t compute_progress_percent(int32_t current_pos, int32_t start_pos, int32_t target_pos);
int32_t master_position();
int32_t progress_start_ticks();

// Resets used during setup/self-test
void reset_tick_counters(uint32_t m, bool all);
void reset_motor_positions();
void reset_idle_timer();

// Fatal error handler (never returns)
void panicf(const char *fmt, ...);

} // namespace wallter::control
