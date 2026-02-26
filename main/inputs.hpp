#pragma once

#include <cstdint>

class MotorDriver;

namespace wallter::inputs {

struct Events {
    bool extend{};
    bool retract{};
};

void init(MotorDriver *motors, int num_motors);

// Edge-like events latched by ISRs (typically cleared once per main-loop iteration)
Events poll_button_events();
void clear_button_events();

// Button level reads (active-low -> true means pressed)
bool read_extend_pressed();
bool read_retract_pressed();

bool boot_menu_requested();
// Legacy behavior: holding BOTH buttons at boot skips the automatic self-test.
bool boot_skip_self_test_requested();
void clear_boot_menu_requested();

// Boot-time request to skip the power-on self-test (latched during init).
bool skip_self_test_requested();
void clear_skip_self_test_requested();

} // namespace wallter::inputs
