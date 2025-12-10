#include "buttons.hpp"
#include "esp_log.h"

int compute_next_target_index(bool extend, bool retract, int current_idx, int max_angles) {
    // Log button presses
    if (extend) {
        ESP_LOGI("buttons", "Extend pressed.");
    }
    if (retract) {
        ESP_LOGI("buttons", "Retract pressed.");
    }

    int idx = current_idx;
    if (extend && !retract) {
        if (current_idx < (max_angles - 1)) {
            idx = current_idx + 1;
        } else {
            ESP_LOGW("buttons", "Max target; ignoring extend.");
        }
    } else if (!extend && retract) {
        if (current_idx > 0) {
            idx = current_idx - 1;
        } else {
            ESP_LOGW("buttons", "Min target; ignoring retract.");
        }
    }
    if (idx < 0) {
        idx = 0;
    }
    if (idx >= max_angles) {
        idx = max_angles - 1;
    }
    return idx;
}

int decide_command_for_target(int32_t current_pos,
                              uint32_t new_target_pos,
                              int cmd_stop,
                              int cmd_extend,
                              int cmd_retract,
                              int cmd_home) {
    if (new_target_pos == current_pos) {
        return cmd_stop;
    }
    if (new_target_pos == 0 && new_target_pos < (uint32_t)current_pos) {
        ESP_LOGI("buttons", "Homing.");
        return cmd_home;
    }
    if (new_target_pos > (uint32_t)current_pos) {
        ESP_LOGI("buttons", "Extending (idle).");
        return cmd_extend;
    }
    ESP_LOGI("buttons", "Retracting (idle).");
    return cmd_retract;
}

bool is_target_change_permitted(int32_t current_pos,
                                uint32_t new_target_pos,
                                int current_cmd,
                                int cmd_extend,
                                int cmd_retract) {
    if (current_cmd == cmd_extend && new_target_pos < (uint32_t)current_pos) {
        ESP_LOGW("buttons", "Direction mismatch (extend->retract).");
        return false;
    }
    if (current_cmd == cmd_retract && new_target_pos > (uint32_t)current_pos) {
        ESP_LOGW("buttons", "Direction mismatch (retract->extend).");
        return false;
    }
    return true;
}
