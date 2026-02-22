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

    // Usable target index range (inclusive). These are logical indexes into target_ticks.
    int min_target_idx{0};
    int max_target_idx{0};

    // If non-zero, homing (CMD_HOME) will: retract to hard-stop, then extend by this many raw ticks,
    // then stop. This supports boards whose lowest usable angle is not the physical hard-stop.
    uint32_t home_offset_raw_ticks{0};
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

// Updates usable angle limits and homing offset at runtime.
void update_limits(int min_target_idx, int max_target_idx, uint32_t home_offset_raw_ticks);

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
