#ifndef STEPPER_AXIS_H_
#define STEPPER_AXIS_H_

#include <stdint.h>

#define STEPPER_AXIS_MIN_RPM          2.0f
#define STEPPER_AXIS_MAX_RPM          40.0f
#define STEPPER_AXIS_STEPS_PER_REV    6400.0f
#define STEPPER_AXIS_GEAR_RATIO       1.0f
#define STEPPER_AXIS_SOFT_LIMIT_EN    1U
#define STEPPER_AXIS_MIN_ANGLE_DEG    (-8.0f)
#define STEPPER_AXIS_MAX_ANGLE_DEG      8.0f

void stepper_axis_init(void);
void stepper_axis_set_velocity(float rpm);
void stepper_axis_stop(void);
float stepper_axis_get_angle_estimate(void);
int32_t stepper_axis_get_pulse_count(void);
void stepper_axis_calibrate_zero(void);
uint8_t stepper_axis_velocity_allowed(float rpm);
void stepper_axis_on_pulse(uint8_t direction);

#endif
