#pragma once
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

// C API bridging to C++ classes for display and motor control

void bridge_init(void);
void bridge_refresh_display(void);
void bridge_print(const char *l1, const char *l2);
void bridge_show_short(const char *l1, const char *l2, uint32_t time_ms);
void bridge_set_refresh_rate(float seconds);

int  bridge_num_motors(void);
void bridge_motor_set_speed(int idx, int32_t speed);
int32_t bridge_motor_get_speed(int idx);
void bridge_motor_reset_steps(int idx, int all);
void bridge_motor_set_position(int idx, int32_t pos);
int32_t bridge_motor_get_position(int idx);
uint32_t bridge_motor_get_steps_in(int idx);
uint32_t bridge_motor_get_steps_out(int idx);
void bridge_motor_increment_in(int idx);
void bridge_motor_increment_out(int idx);

#ifdef __cplusplus
}
#endif
