#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <stdint.h>
#include "cytron_md.hpp"
#include <pid.hpp>

// Simple wrapper for a single Cytron motor with internal state
enum MotorDirection { DIR_STOP = 0, DIR_EXTEND = 1, DIR_RETRACT = -1 };

class MotorDriver {
  public:
    // Construct with Cytron PWM/DIR pins
    MotorDriver(uint8_t pwm_pin, uint8_t dir_pin);

    void init();
    void setSpeed(int32_t speed);  // [-MAXSPEED, MAXSPEED]
    int32_t getSpeed() const;      // current commanded speed
    MotorDirection getDirection() const;

    // Tick counters maintained per motor
    void incrementStepIn();
    void incrementStepOut();
    uint32_t getStepsIn() const;
    uint32_t getStepsOut() const;
    void resetSteps(bool all = true);

    // Position tracking
    void incrementPosition(int32_t delta);
    void setPosition(int32_t pos);
    int32_t getPosition() const;

    // Basic error check: when moving, ensure relevant step counter increases.
    // Pass the current cumulative step counts (in/out) observed for this motor.
    // Returns true if OK, false if suspicious (no movement reported).
    bool errorCheck(uint32_t steps_in,
                    uint32_t steps_out,
                    uint32_t min_delta = 3,
                    uint32_t window_ms = 1000);

    // Idle detection: returns true if position hasn't changed for window_ms
    bool isIdle(uint32_t window_ms = 200) const;
    // Idle duration in ms accumulated over successive idle windows
    uint32_t getIdleDurationMs() const;
    void resetIdle();
#ifdef UNIT_TEST
    // Test-only introspection helpers
    uint32_t _test_last_idle_check_ms() const {
        return last_idle_check_ms;
    }
    int32_t _test_last_idle_position() const {
        return last_idle_position;
    }
    uint32_t _test_idle_accum_ms() const {
        return idle_accum_ms;
    }
#endif

    // PID control embedded per motor (followers only; master doesn't use PID)
    void pidBegin(double kp, double ki, double kd);
    void pidConfigureLimits(double outMin, double outMax);
    void pidSetSampleTime(uint16_t ms);
    void pidStart();
    void pidReset();
    // Common follow computation for non-master motors.
    // Returns signed speed to apply (handles accel-phase override).
    int32_t
    computeFollow(bool reverse, bool accel_phase, int32_t master_speed, const MotorDriver& master);

  private:
    CytronMD controller;
    uint8_t pwm_pin;
    uint8_t dir_pin;

    int32_t last_speed;
    MotorDirection last_dir;

    uint32_t last_check_ms;
    uint32_t last_steps_in;
    uint32_t last_steps_out;

    std::atomic<uint32_t> steps_in{0};
    std::atomic<uint32_t> steps_out{0};

    volatile int32_t position;

    // PID state
    espp::Pid pid{ espp::Pid::Config{
        /*kp*/ 0.0f,
        /*ki*/ 0.0f,
        /*kd*/ 0.0f,
        /*integrator_min*/ -255.0f,
        /*integrator_max*/  255.0f,
        /*output_min*/     -255.0f,
        /*output_max*/      255.0f
    }};
    double pid_input    = 0.0;
    double pid_output   = 0.0;
    double pid_setpoint = 0.0;

    // Idle tracking
    mutable uint32_t last_idle_check_ms = 0;
    mutable int32_t last_idle_position  = 0;
    mutable uint32_t idle_accum_ms      = 0; // deprecated; retained for ABI
    mutable uint32_t last_move_ms       = 0; // timestamp of last step change
};

#endif
