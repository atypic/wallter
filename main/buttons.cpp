#include "buttons.hpp"
#include "esp_log.h"

int compute_next_target_index(bool extend,
                              bool retract,
                              int current_idx,
                              int max_angles,
                              int min_idx,
                              int max_idx) {
    // Log button presses
    if (extend) {
        ESP_LOGI("buttons", "Extend pressed.");
    }
    if (retract) {
        ESP_LOGI("buttons", "Retract pressed.");
    }

    if (min_idx < 0) min_idx = 0;
    if (max_idx < 0) max_idx = 0;
    if (max_idx >= max_angles) max_idx = max_angles - 1;
    if (min_idx > max_idx) {
        min_idx = 0;
        max_idx = max_angles - 1;
    }

    int idx = current_idx;
    if (idx < min_idx) idx = min_idx;
    if (idx > max_idx) idx = max_idx;
    if (extend && !retract) {
        if (idx < max_idx) {
            idx = idx + 1;
        } else {
            ESP_LOGW("buttons", "Max target; ignoring extend.");
        }
    } else if (!extend && retract) {
        if (idx > min_idx) {
            idx = idx - 1;
        } else {
            ESP_LOGW("buttons", "Min target; ignoring retract.");
        }
    }
    if (idx < min_idx) idx = min_idx;
    if (idx > max_idx) idx = max_idx;
    return idx;
}

int decide_command_for_target(int32_t current_pos,
                              uint32_t new_target_pos,
                              int cmd_stop,
                              int cmd_extend,
                              int cmd_retract,
                              int cmd_home) {
    int32_t pos = current_pos;
    if (pos < 0) pos = 0;
    int32_t tgt = (new_target_pos > (uint32_t)INT32_MAX) ? INT32_MAX : (int32_t)new_target_pos;

    if (tgt == pos) {
        return cmd_stop;
    }
    if (tgt == 0 && tgt < pos) {
        ESP_LOGI("buttons", "Homing.");
        return cmd_home;
    }
    if (tgt > pos) {
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
    int32_t pos = current_pos;
    if (pos < 0) pos = 0;
    int32_t tgt = (new_target_pos > (uint32_t)INT32_MAX) ? INT32_MAX : (int32_t)new_target_pos;

    if (current_cmd == cmd_extend && tgt < pos) {
        ESP_LOGW("buttons", "Direction mismatch (extend->retract).");
        return false;
    }
    if (current_cmd == cmd_retract && tgt > pos) {
        ESP_LOGW("buttons", "Direction mismatch (retract->extend).");
        return false;
    }
    return true;
}
