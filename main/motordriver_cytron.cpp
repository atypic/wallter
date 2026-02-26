#include "motordriver.hpp"
#include "cytron_md.hpp"
#include "esp_log.h"
#include "esp_attr.h"
#include "esp_timer.h"

static const char *TAG = "MotorDriver";

MotorDriver::MotorDriver(uint8_t pwm_pin, uint8_t dir_pin, ledc_channel_t pwm_channel, bool invert_output)
    : controller(CYTRON_PWM_DIR,
                             static_cast<gpio_num_t>(pwm_pin),
                             static_cast<gpio_num_t>(dir_pin),
                             pwm_channel,
                             /*ch2*/ static_cast<ledc_channel_t>((int)pwm_channel + 1)),
    pwm_pin(pwm_pin),
    dir_pin(dir_pin),
    invert_output_(invert_output),
    last_speed(0),
    last_dir(DIR_STOP),
    last_check_ms(0),
    last_steps_in(0),
    last_steps_out(0),
    steps_in(0),
    steps_out(0),
    position(0) {}

void MotorDriver::init() {
    esp_err_t err = controller.init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "CytronMD init failed: %d", (int)err);
    }
    setSpeed(0);
}

void MotorDriver::setSpeed(int32_t speed) {
    int32_t s = speed;
    if (s > 255) s = 255;
    if (s < -255) s = -255;

    int32_t applied = invert_output_ ? -s : s;

    if (s == 0) {
        esp_err_t err = controller.setSpeed(0);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "CytronMD setSpeed(0) failed: %d", (int)err);
        }
        last_speed = 0;
        last_dir   = DIR_STOP;
        return;
    }

    esp_err_t err = controller.setSpeed((int16_t)applied);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "CytronMD setSpeed(%ld -> %ld) failed: %d", (long)s, (long)applied, (int)err);
    }
    last_speed = s;
    last_dir   = (s > 0) ? DIR_EXTEND : DIR_RETRACT;
}

int32_t IRAM_ATTR MotorDriver::getSpeed() const { return last_speed; }
MotorDirection IRAM_ATTR MotorDriver::getDirection() const { return last_dir; }

bool MotorDriver::errorCheck(uint32_t steps_in,
                             uint32_t steps_out,
                             uint32_t min_delta,
                             uint32_t window_ms) {
    if (last_dir == DIR_STOP) {
        return true;
    }

    uint32_t now = (uint32_t)(esp_timer_get_time() / 1000ULL);
    if (last_check_ms == 0) {
        last_check_ms = now;
    }
    if ((now - last_check_ms) < window_ms) {
        return true;
    }

    bool ok = true;
    if (last_dir == DIR_EXTEND) {
        uint32_t delta_out = (steps_out >= last_steps_out) ? (steps_out - last_steps_out) : 0;
        if (delta_out < min_delta) ok = false;
    } else if (last_dir == DIR_RETRACT) {
        uint32_t delta_in = (steps_in >= last_steps_in) ? (steps_in - last_steps_in) : 0;
        if (delta_in < min_delta) ok = false;
    }

    if (!ok) {
        ESP_LOGE(TAG, "Stall: dir=%d in:%lu out:%lu",
                 (int)last_dir,
                 (unsigned long)steps_in,
                 (unsigned long)steps_out);
    }

    last_steps_in  = steps_in;
    last_steps_out = steps_out;
    last_check_ms  = now;
    return ok;
}

void IRAM_ATTR MotorDriver::incrementStepIn() {
    steps_in.fetch_add(1, std::memory_order_relaxed);
    // Mirror old driver: stepping IN decrements position
    position -= 1;
    last_move_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
}
void IRAM_ATTR MotorDriver::incrementStepOut() {
    steps_out.fetch_add(1, std::memory_order_relaxed);
    // Mirror old driver: stepping OUT increments position
    position += 1;
    last_move_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
}
uint32_t MotorDriver::getStepsIn() const { return steps_in.load(std::memory_order_relaxed); }
uint32_t MotorDriver::getStepsOut() const { return steps_out.load(std::memory_order_relaxed); }
void MotorDriver::resetSteps(bool /*all*/) { steps_in.store(0, std::memory_order_relaxed); steps_out.store(0, std::memory_order_relaxed); }
void MotorDriver::setPosition(int32_t pos) { position = pos; }
int32_t MotorDriver::getPosition() const { return position; }

bool MotorDriver::isIdle(uint32_t window_ms) const {
    uint32_t now = (uint32_t)(esp_timer_get_time() / 1000ULL);
    if (last_move_ms == 0) {
        // Establish baseline if never moved
        return false;
    }
    return (now - last_move_ms) >= window_ms;
}

uint32_t MotorDriver::getIdleDurationMs() const {
    uint32_t now = (uint32_t)(esp_timer_get_time() / 1000ULL);
    if (last_move_ms == 0) {
        return 0;
    }
    return now - last_move_ms;
}

void MotorDriver::resetIdle() {
    uint32_t now = (uint32_t)(esp_timer_get_time() / 1000ULL);
    last_move_ms = now;
}

void MotorDriver::pidBegin(double kp, double ki, double kd) {
    espp::Pid::Config cfg = pid.get_config();
    cfg.kp = (float)kp;
    cfg.ki = (float)ki;
    cfg.kd = (float)kd;
    pid.set_config(cfg, /*reset_state*/true);
}
void MotorDriver::pidConfigureLimits(double outMin, double outMax) {
    espp::Pid::Config cfg = pid.get_config();
    cfg.output_min = (float)outMin;
    cfg.output_max = (float)outMax;
    // Keep integrator limits consistent with output limits unless user set otherwise
    cfg.integrator_min = (float)outMin;
    cfg.integrator_max = (float)outMax;
    pid.set_config(cfg, /*reset_state*/false);
}
void MotorDriver::pidSetSampleTime(uint16_t /*ms*/) { /* espp::Pid tracks dt internally */ }
void MotorDriver::pidStart() { /* no-op */ }
void MotorDriver::pidReset() { pid.clear(); }

int32_t MotorDriver::computeFollow(bool reverse,
                                   bool accel_phase,
                                   int32_t master_speed,
                                   const MotorDriver& master) {
    if (accel_phase) {
        return master_speed * (reverse ? -1 : 1);
    }
    if (reverse) {
        pid_input    = (double)this->getStepsIn();
        pid_setpoint = (double)master.getStepsIn();
    } else {
        pid_input    = (double)this->getStepsOut();
        pid_setpoint = (double)master.getStepsOut();
    }
    float error = (float)(pid_setpoint - pid_input);
    pid_output = (double)pid.update(error);
    return reverse ? (int32_t)pid_output * -1 : (int32_t)pid_output;
}
