#pragma once
#include <stdint.h>

// Pure helper functions to make button handling testable.

// Compute next target index based on extend/retract presses and bounds.
int compute_next_target_index(bool extend,
                              bool retract,
                              int current_idx,
                              int max_angles,
                              int min_idx,
                              int max_idx);

// Decide intended command given target vs current position.
// Returns one of CMD_STOP, CMD_EXTEND, CMD_RETRACT, CMD_HOME.
int decide_command_for_target(int32_t current_pos,
                              uint32_t new_target_pos,
                              int cmd_stop,
                              int cmd_extend,
                              int cmd_retract,
                              int cmd_home);

// Validate mid-move target change: only allow changes that keep direction.
bool is_target_change_permitted(int32_t current_pos,
                                uint32_t new_target_pos,
                                int current_cmd,
                                int cmd_extend,
                                int cmd_retract);
