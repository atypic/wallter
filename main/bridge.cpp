#include "bridge.hpp"
#include "display.hpp"
#include "motordriver.hpp"
#include "boards.h"

static Display g_display;
static MotorDriver motors[NUM_MOTORS] = {
    MotorDriver(HAL_PWM[0], HAL_DIR[0]),
    MotorDriver(HAL_PWM[1], HAL_DIR[1])
#if NUM_MOTORS > 2
    , MotorDriver(HAL_PWM[2], HAL_DIR[2])
#endif
#if NUM_MOTORS > 3
    , MotorDriver(HAL_PWM[3], HAL_DIR[3])
#endif
};

extern "C" void bridge_init(void) {
    g_display.init();
    for (int i = 0; i < NUM_MOTORS; ++i) {
        motors[i].init();
    }
}

extern "C" void bridge_refresh_display(void) { g_display.refresh(); }
extern "C" void bridge_print(const char *l1, const char *l2) { g_display.print(l1, l2); }
extern "C" void bridge_show_short(const char *l1, const char *l2, uint32_t time_ms) {
    g_display.show_short_message((char*)l1, (char*)l2, time_ms);
}
extern "C" void bridge_set_refresh_rate(float seconds) { g_display.set_refresh_rate(seconds); }

extern "C" int bridge_num_motors(void) { return NUM_MOTORS; }
extern "C" void bridge_motor_set_speed(int idx, int32_t speed) { motors[idx].setSpeed(speed); }
extern "C" int32_t bridge_motor_get_speed(int idx) { return motors[idx].getSpeed(); }
extern "C" void bridge_motor_reset_steps(int idx, int all) { motors[idx].resetSteps(all != 0); }
extern "C" void bridge_motor_set_position(int idx, int32_t pos) { motors[idx].setPosition(pos); }
extern "C" int32_t bridge_motor_get_position(int idx) { return motors[idx].getPosition(); }
extern "C" uint32_t bridge_motor_get_steps_in(int idx) { return motors[idx].getStepsIn(); }
extern "C" uint32_t bridge_motor_get_steps_out(int idx) { return motors[idx].getStepsOut(); }
extern "C" void bridge_motor_increment_in(int idx) { motors[idx].incrementStepIn(); }
extern "C" void bridge_motor_increment_out(int idx) { motors[idx].incrementStepOut(); }
