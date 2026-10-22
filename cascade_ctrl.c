#include "cascade_ctrl.h"

#include "speed_ctrl.h"
#include "stepper_axis.h"
#include "yaw.h"

static volatile float serial_position = 0.0f;
static volatile float target_position = CASCADE_TARGET_POSITION;
static volatile float position_error = 0.0f;
static volatile float position_velocity = 0.0f;
static volatile float angle_reference = 0.0f;
static volatile float angle_feedback = 0.0f;
static volatile float rpm_command = 0.0f;
static volatile float vehicle_acceleration_mps2 = 0.0f;
static volatile float vehicle_feedforward_angle = 0.0f;
static volatile float position_proportional_angle = 0.0f;
static volatile float position_integral_output_angle = 0.0f;
static volatile float position_derivative_output_angle = 0.0f;
static volatile float slope_compensation_angle = 0.0f;
static volatile float imu_trim_angle = 0.0f;
static volatile float imu_rail_acceleration_mps2 = 0.0f;
static volatile uint8_t input_valid = 0U;
static volatile uint8_t controller_enabled = 1U;
static volatile uint8_t hold_active = 0U;
static volatile uint8_t angle_saturated = 0U;
static volatile uint8_t vehicle_feedforward_enabled =
    CASCADE_MOTION_FF_ENABLE;
static volatile uint32_t last_input_ms = 0U;

static float previous_position = 0.0f;
static uint32_t previous_position_ms = 0U;
static uint8_t velocity_ready = 0U;
static uint8_t outer_tick_count = 0U;
static float position_integral_angle = 0.0f;
static float previous_position_error = 0.0f;
static uint8_t integral_condition_active = 0U;
static uint32_t integral_condition_start_ms = 0U;
static volatile uint8_t stiction_compensation_active = 0U;
static uint8_t stiction_condition_active = 0U;
static uint32_t stiction_condition_start_ms = 0U;
static float slope_acceleration_estimate_mps2 = 0.0f;
static uint8_t slope_rest_condition_active = 0U;
static uint32_t slope_rest_condition_start_ms = 0U;
static uint8_t vehicle_motion_active = 0U;
static uint32_t vehicle_motion_start_ms = 0U;

static float cascade_abs(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static float cascade_limit(float value, float min_value, float max_value)
{
    if (value > max_value) {
        return max_value;
    }
    if (value < min_value) {
        return min_value;
    }
    return value;
}

static float cascade_slew(float current, float target, float max_step)
{
    float delta = target - current;

    if (delta > max_step) {
        return current + max_step;
    }
    if (delta < -max_step) {
        return current - max_step;
    }
    return target;
}

static float cascade_apply_position_direction(float angle)
{
#if CASCADE_ANGLE_DIRECTION_INVERT
    return -angle;
#else
    return angle;
#endif
}

static float cascade_deadzone_error(float error)
{
    float error_abs = cascade_abs(error);

    if (error_abs <= CASCADE_POSITION_DEADBAND_MM) {
        return 0.0f;
    }
    if (error > 0.0f) {
        return error_abs - CASCADE_POSITION_DEADBAND_MM;
    }
    return -(error_abs - CASCADE_POSITION_DEADBAND_MM);
}

static void cascade_reset_integral(void)
{
    position_integral_angle = 0.0f;
    position_integral_output_angle = 0.0f;
    integral_condition_active = 0U;
    integral_condition_start_ms = 0U;
}

static void cascade_reset_stiction_compensation(void)
{
    stiction_compensation_active = 0U;
    stiction_condition_active = 0U;
    stiction_condition_start_ms = 0U;
}

static float cascade_apply_stiction_compensation(float target_angle,
    float control_error, uint32_t now_ms)
{
    float direction_angle;
    uint8_t compensation_allowed;

    compensation_allowed =
        (cascade_abs(position_error) > CASCADE_STICTION_ERROR_MIN_MM) &&
        (cascade_abs(position_velocity) <=
            CASCADE_STICTION_VELOCITY_MAX) &&
        (hold_active == 0U) &&
        (SpeedCtrl_IsAtRest() != 0U);

    if (compensation_allowed == 0U) {
        cascade_reset_stiction_compensation();
        return target_angle;
    }

    if (stiction_condition_active == 0U) {
        stiction_condition_active = 1U;
        stiction_condition_start_ms = now_ms;
        return target_angle;
    }
    if ((uint32_t)(now_ms - stiction_condition_start_ms) <
        CASCADE_STICTION_CONFIRM_MS) {
        return target_angle;
    }

    stiction_compensation_active = 1U;
    direction_angle = cascade_apply_position_direction(control_error);
    if ((direction_angle > 0.0f) &&
        (target_angle < CASCADE_STICTION_MIN_ANGLE_DEG)) {
        return CASCADE_STICTION_MIN_ANGLE_DEG;
    }
    if ((direction_angle < 0.0f) &&
        (target_angle > -CASCADE_STICTION_MIN_ANGLE_DEG)) {
        return -CASCADE_STICTION_MIN_ANGLE_DEG;
    }
    return target_angle;
}

static void cascade_reset_slope_compensation(void)
{
    slope_compensation_angle = 0.0f;
    imu_trim_angle = 0.0f;
    imu_rail_acceleration_mps2 = 0.0f;
    slope_acceleration_estimate_mps2 = 0.0f;
    slope_rest_condition_active = 0U;
    slope_rest_condition_start_ms = 0U;
    vehicle_motion_active = 0U;
    vehicle_motion_start_ms = 0U;
}

static void cascade_reset_velocity(void)
{
    position_velocity = 0.0f;
    previous_position = serial_position;
    previous_position_ms = last_input_ms;
    velocity_ready = 0U;
}

static void cascade_update_velocity(float position, uint32_t now_ms)
{
    uint32_t dt_ms;
    float measured_velocity;

    if (velocity_ready == 0U) {
        previous_position = position;
        previous_position_ms = now_ms;
        position_velocity = 0.0f;
        velocity_ready = 1U;
        return;
    }

    dt_ms = (uint32_t)(now_ms - previous_position_ms);
    if ((dt_ms == 0U) || (dt_ms >= CASCADE_INPUT_TIMEOUT_MS)) {
        previous_position = position;
        previous_position_ms = now_ms;
        position_velocity = 0.0f;
        return;
    }

    measured_velocity = (position - previous_position) * 1000.0f /
        (float)dt_ms;
    measured_velocity = cascade_limit(measured_velocity,
        -CASCADE_VELOCITY_MAX, CASCADE_VELOCITY_MAX);
    position_velocity += CASCADE_VELOCITY_FILTER_ALPHA *
        (measured_velocity - position_velocity);
    previous_position = position;
    previous_position_ms = now_ms;
}

static void cascade_update_vehicle_feedforward(void)
{
    float profile_acceleration_mps2;

    if (vehicle_feedforward_enabled == 0U) {
        vehicle_acceleration_mps2 = 0.0f;
        vehicle_feedforward_angle = 0.0f;
        return;
    }

    /* SpeedCtrl reports cm/s^2. Its jerk-limited profile changes before the
     * chassis responds, so braking compensation is applied in advance. */
    profile_acceleration_mps2 =
        SpeedCtrl_GetProfileAcceleration() * 0.01f;
    vehicle_acceleration_mps2 = profile_acceleration_mps2 *
        CASCADE_VEHICLE_FF_FORWARD_PROJECTION;
    vehicle_feedforward_angle = CASCADE_VEHICLE_FF_DIRECTION *
        CASCADE_VEHICLE_FF_GAIN * vehicle_acceleration_mps2 /
        CASCADE_GRAVITY_MPS2 * CASCADE_RAD_TO_DEG;
    vehicle_feedforward_angle = cascade_limit(vehicle_feedforward_angle,
        -CASCADE_VEHICLE_FF_MAX_DEG, CASCADE_VEHICLE_FF_MAX_DEG);
}

static void cascade_update_slope_compensation(uint32_t now_ms)
{
    float trim_target = 0.0f;
    float acceleration_residual;

    if (vehicle_feedforward_enabled == 0U) {
        cascade_reset_slope_compensation();
        return;
    }

    if (Yaw_IsFaulted() != 0U) {
        slope_rest_condition_active = 0U;
        vehicle_motion_active = 0U;
        imu_trim_angle = cascade_slew(imu_trim_angle, 0.0f,
            CASCADE_IMU_TRIM_SLEW_DEG);
        return;
    }

    imu_rail_acceleration_mps2 =
        CASCADE_SLOPE_AXIS_X * Yaw_GetAccelX() +
        CASCADE_SLOPE_AXIS_Y * Yaw_GetAccelY() +
        CASCADE_SLOPE_AXIS_Z * Yaw_GetAccelZ();

    if ((SpeedCtrl_IsAtRest() != 0U) &&
        (cascade_abs(vehicle_acceleration_mps2) <=
            CASCADE_SLOPE_CMD_ACCEL_MAX_MPS2)) {
        vehicle_motion_active = 0U;
        if (slope_rest_condition_active == 0U) {
            slope_rest_condition_active = 1U;
            slope_rest_condition_start_ms = now_ms;
        } else if ((uint32_t)(now_ms - slope_rest_condition_start_ms) >=
            CASCADE_SLOPE_REST_CONFIRM_MS) {
            slope_acceleration_estimate_mps2 +=
                CASCADE_SLOPE_FILTER_ALPHA *
                (imu_rail_acceleration_mps2 -
                    slope_acceleration_estimate_mps2);
        }
        imu_trim_angle = cascade_slew(imu_trim_angle, 0.0f,
            CASCADE_IMU_TRIM_SLEW_DEG);
    } else {
        slope_rest_condition_active = 0U;
        if (vehicle_motion_active == 0U) {
            vehicle_motion_active = 1U;
            vehicle_motion_start_ms = now_ms;
        }
        if ((uint32_t)(now_ms - vehicle_motion_start_ms) >=
            CASCADE_IMU_TRIM_START_DELAY_MS) {
            acceleration_residual = imu_rail_acceleration_mps2 -
                slope_acceleration_estimate_mps2 -
                CASCADE_IMU_COMMAND_MATCH_SIGN *
                    vehicle_acceleration_mps2;
            trim_target = CASCADE_SLOPE_FF_DIRECTION *
                CASCADE_IMU_TRIM_GAIN * acceleration_residual /
                CASCADE_GRAVITY_MPS2 * CASCADE_RAD_TO_DEG;
            trim_target = cascade_limit(trim_target,
                -CASCADE_IMU_TRIM_MAX_DEG,
                CASCADE_IMU_TRIM_MAX_DEG);
        }
        imu_trim_angle = cascade_slew(imu_trim_angle, trim_target,
            CASCADE_IMU_TRIM_SLEW_DEG);
    }

    slope_compensation_angle = CASCADE_SLOPE_FF_DIRECTION *
        slope_acceleration_estimate_mps2 /
        CASCADE_GRAVITY_MPS2 * CASCADE_RAD_TO_DEG;
    slope_compensation_angle = cascade_limit(slope_compensation_angle,
        -CASCADE_SLOPE_FF_MAX_DEG, CASCADE_SLOPE_FF_MAX_DEG);
}

static void cascade_update_hold_state(void)
{
    float error_abs = cascade_abs(position_error);
    float velocity_abs = cascade_abs(position_velocity);

    if (hold_active != 0U) {
        if ((error_abs >= CASCADE_POSITION_HOLD_EXIT_MM) ||
            (velocity_abs > CASCADE_HOLD_VELOCITY_MAX)) {
            hold_active = 0U;
        }
    } else if ((error_abs <= CASCADE_POSITION_DEADBAND_MM) &&
        (velocity_abs <= CASCADE_HOLD_VELOCITY_MAX)) {
        hold_active = 1U;
    }
}

static void cascade_update_integral(float control_error, float p_angle,
    float other_angle, uint32_t now_ms)
{
    float candidate_integral;
    float candidate_integral_output;
    float candidate_total_angle;
    float integral_delta_output;
    uint8_t integrate_allowed;

    integrate_allowed =
        (cascade_abs(position_error) > CASCADE_POSITION_DEADBAND_MM) &&
        (cascade_abs(position_error) <=
            CASCADE_POSITION_I_ERROR_MAX_MM) &&
        (cascade_abs(position_velocity) <=
            CASCADE_POSITION_I_VELOCITY_MAX) &&
        (cascade_abs(vehicle_feedforward_angle) <=
            CASCADE_POSITION_I_FF_PAUSE_DEG) &&
        (cascade_abs(imu_rail_acceleration_mps2 -
            slope_acceleration_estimate_mps2) <=
            CASCADE_POSITION_I_SLOPE_RESIDUAL_MAX_MPS2) &&
        (SpeedCtrl_IsAtRest() != 0U);

    if (integrate_allowed == 0U) {
        integral_condition_active = 0U;
        integral_condition_start_ms = 0U;
        return;
    }

    if (integral_condition_active == 0U) {
        integral_condition_active = 1U;
        integral_condition_start_ms = now_ms;
        return;
    }
    if ((uint32_t)(now_ms - integral_condition_start_ms) <
        CASCADE_POSITION_I_CONFIRM_MS) {
        return;
    }

    candidate_integral = position_integral_angle +
        CASCADE_POSITION_KI_DEG_PER_MM_S * control_error *
        ((float)CASCADE_OUTER_PERIOD_MS / 1000.0f);
    candidate_integral = cascade_limit(candidate_integral,
        -CASCADE_POSITION_I_MAX_DEG,
        CASCADE_POSITION_I_MAX_DEG);
    candidate_integral_output =
        cascade_apply_position_direction(candidate_integral);
    candidate_total_angle = p_angle + candidate_integral_output +
        other_angle;
    integral_delta_output = candidate_integral_output -
        cascade_apply_position_direction(position_integral_angle);

    if (((candidate_total_angle > CASCADE_TARGET_ANGLE_MAX_DEG) &&
            (integral_delta_output > 0.0f)) ||
        ((candidate_total_angle < -CASCADE_TARGET_ANGLE_MAX_DEG) &&
            (integral_delta_output < 0.0f))) {
        return;
    }
    position_integral_angle = candidate_integral;
}

static void cascade_run_position_loop(uint32_t now_ms)
{
    float control_error;
    float p_angle_logical;
    float d_angle_logical;
    float other_angle;
    float target_angle_unlimited;
    float target_angle;

    position_error = target_position - serial_position;
    if ((position_error * previous_position_error) < 0.0f) {
        cascade_reset_integral();
        cascade_reset_stiction_compensation();
    }
    previous_position_error = position_error;
    cascade_update_hold_state();

    control_error = (hold_active != 0U) ? 0.0f :
        cascade_deadzone_error(position_error);
    p_angle_logical = CASCADE_POSITION_KP_DEG_PER_MM * control_error;
    position_proportional_angle =
        cascade_apply_position_direction(p_angle_logical);
    d_angle_logical = (hold_active != 0U) ? 0.0f : cascade_limit(
        -CASCADE_POSITION_KD_DEG_PER_MM_PER_S * position_velocity,
        -CASCADE_POSITION_D_MAX_DEG, CASCADE_POSITION_D_MAX_DEG);
    position_derivative_output_angle =
        cascade_apply_position_direction(d_angle_logical);
    other_angle = position_derivative_output_angle +
        vehicle_feedforward_angle +
        slope_compensation_angle + imu_trim_angle;
    cascade_update_integral(control_error,
        position_proportional_angle, other_angle, now_ms);
    position_integral_output_angle =
        cascade_apply_position_direction(position_integral_angle);

    target_angle_unlimited = position_proportional_angle +
        position_integral_output_angle + other_angle;
    target_angle_unlimited = cascade_apply_stiction_compensation(
        target_angle_unlimited, control_error, now_ms);
    angle_saturated = ((target_angle_unlimited >
            CASCADE_TARGET_ANGLE_MAX_DEG) ||
        (target_angle_unlimited <
            -CASCADE_TARGET_ANGLE_MAX_DEG)) ? 1U : 0U;
    target_angle = cascade_limit(target_angle_unlimited,
        -CASCADE_TARGET_ANGLE_MAX_DEG,
        CASCADE_TARGET_ANGLE_MAX_DEG);
    angle_reference = cascade_slew(angle_reference, target_angle,
        CASCADE_TARGET_ANGLE_SLEW_DEG);
}

static void cascade_run_angle_loop(void)
{
    float angle_error;
    float target_rpm;

    angle_feedback = stepper_axis_get_angle_estimate();
    angle_error = angle_reference - angle_feedback;
    if (cascade_abs(angle_error) <= CASCADE_ANGLE_DEADBAND_DEG) {
        rpm_command = 0.0f;
        stepper_axis_set_velocity(0.0f);
        return;
    }

    target_rpm = CASCADE_ANGLE_KP_RPM_PER_DEG * angle_error;
    target_rpm = cascade_limit(target_rpm,
        -CASCADE_RPM_MAX, CASCADE_RPM_MAX);
    rpm_command = cascade_slew(rpm_command, target_rpm,
        CASCADE_RPM_SLEW);
    stepper_axis_set_velocity(rpm_command);
}

void CascadeCtrl_Init(void)
{
    serial_position = 0.0f;
    target_position = CASCADE_TARGET_POSITION;
    position_error = 0.0f;
    position_velocity = 0.0f;
    angle_reference = 0.0f;
    angle_feedback = 0.0f;
    rpm_command = 0.0f;
    vehicle_acceleration_mps2 = 0.0f;
    vehicle_feedforward_angle = 0.0f;
    position_proportional_angle = 0.0f;
    position_integral_output_angle = 0.0f;
    position_derivative_output_angle = 0.0f;
    slope_compensation_angle = 0.0f;
    imu_trim_angle = 0.0f;
    imu_rail_acceleration_mps2 = 0.0f;
    input_valid = 0U;
    controller_enabled = 1U;
    hold_active = 0U;
    angle_saturated = 0U;
    vehicle_feedforward_enabled = CASCADE_MOTION_FF_ENABLE;
    last_input_ms = 0U;
    previous_position = 0.0f;
    previous_position_ms = 0U;
    velocity_ready = 0U;
    outer_tick_count = 0U;
    previous_position_error = 0.0f;
    cascade_reset_integral();
    cascade_reset_stiction_compensation();
    cascade_reset_slope_compensation();
    stepper_axis_stop();
}

uint8_t CascadeCtrl_SetSerialPosition(uint16_t position, uint8_t valid,
    uint32_t now_ms)
{
    if ((valid == 0U) || ((float)position < CASCADE_INPUT_MIN) ||
        ((float)position > CASCADE_INPUT_MAX)) {
        input_valid = 0U;
        hold_active = 0U;
        angle_reference = 0.0f;
        cascade_reset_velocity();
        cascade_reset_integral();
        cascade_reset_stiction_compensation();
        position_proportional_angle = 0.0f;
        position_derivative_output_angle = 0.0f;
        angle_saturated = 0U;
        return 0U;
    }

    serial_position = (float)position;
    cascade_update_velocity(serial_position, now_ms);
    position_error = target_position - serial_position;
    input_valid = 1U;
    last_input_ms = now_ms;
    return 1U;
}

void CascadeCtrl_SetTarget(float target)
{
    float new_target = cascade_limit(target,
        CASCADE_INPUT_MIN, CASCADE_INPUT_MAX);

    cascade_reset_integral();
    cascade_reset_stiction_compensation();
    position_derivative_output_angle = 0.0f;
    target_position = new_target;
    position_error = target_position - serial_position;
    previous_position_error = position_error;
    hold_active = 0U;
}

void CascadeCtrl_SetEnabled(uint8_t enabled)
{
    controller_enabled = (enabled != 0U) ? 1U : 0U;
    hold_active = 0U;
    if (controller_enabled == 0U) {
        angle_reference = 0.0f;
        position_proportional_angle = 0.0f;
        position_derivative_output_angle = 0.0f;
        angle_saturated = 0U;
        cascade_reset_integral();
        cascade_reset_stiction_compensation();
    }
}

void CascadeCtrl_SetMotionFeedforwardEnabled(uint8_t enabled)
{
    vehicle_feedforward_enabled = (enabled != 0U) ? 1U : 0U;
    if (vehicle_feedforward_enabled == 0U) {
        vehicle_acceleration_mps2 = 0.0f;
        vehicle_feedforward_angle = 0.0f;
        cascade_reset_slope_compensation();
    }
}

void CascadeCtrl_Task5ms(uint32_t now_ms)
{
    uint8_t outer_due = 0U;

    if (++outer_tick_count >=
        (CASCADE_OUTER_PERIOD_MS / CASCADE_INNER_PERIOD_MS)) {
        outer_tick_count = 0U;
        outer_due = 1U;
    }

    if ((input_valid != 0U) &&
        ((uint32_t)(now_ms - last_input_ms) >=
            CASCADE_INPUT_TIMEOUT_MS)) {
        input_valid = 0U;
        hold_active = 0U;
        angle_reference = 0.0f;
        cascade_reset_velocity();
        cascade_reset_integral();
        cascade_reset_stiction_compensation();
        position_proportional_angle = 0.0f;
        position_derivative_output_angle = 0.0f;
        angle_saturated = 0U;
    }

    if (outer_due != 0U) {
        cascade_update_vehicle_feedforward();
        cascade_update_slope_compensation(now_ms);
        if ((controller_enabled != 0U) && (input_valid != 0U)) {
            cascade_run_position_loop(now_ms);
        }
    }
    if ((controller_enabled == 0U) || (input_valid == 0U)) {
        angle_reference = 0.0f;
        position_proportional_angle = 0.0f;
        position_derivative_output_angle = 0.0f;
        angle_saturated = 0U;
    }

    cascade_run_angle_loop();
}

void CascadeCtrl_Stop(void)
{
    input_valid = 0U;
    controller_enabled = 0U;
    hold_active = 0U;
    target_position = CASCADE_TARGET_POSITION;
    position_error = 0.0f;
    angle_reference = 0.0f;
    rpm_command = 0.0f;
    vehicle_acceleration_mps2 = 0.0f;
    vehicle_feedforward_angle = 0.0f;
    position_proportional_angle = 0.0f;
    position_derivative_output_angle = 0.0f;
    angle_saturated = 0U;
    cascade_reset_velocity();
    cascade_reset_integral();
    cascade_reset_stiction_compensation();
    cascade_reset_slope_compensation();
    stepper_axis_stop();
}

float CascadeCtrl_GetRawPosition(void)
{
    return serial_position;
}

float CascadeCtrl_GetTarget(void)
{
    return target_position;
}

float CascadeCtrl_GetPositionError(void)
{
    return position_error;
}

float CascadeCtrl_GetPositionVelocity(void)
{
    return position_velocity;
}

float CascadeCtrl_GetVelocityReference(void)
{
    return 0.0f;
}

float CascadeCtrl_GetAngleReference(void)
{
    return angle_reference;
}

float CascadeCtrl_GetAngleFeedback(void)
{
    return angle_feedback;
}

float CascadeCtrl_GetRpmCommand(void)
{
    return rpm_command;
}

float CascadeCtrl_GetEstimatedPosition(void)
{
    return serial_position;
}

float CascadeCtrl_GetPredictedPosition(void)
{
    return serial_position;
}

float CascadeCtrl_GetBallAcceleration(void)
{
    return 0.0f;
}

float CascadeCtrl_GetBallFeedforwardAcceleration(void)
{
    return 0.0f;
}

float CascadeCtrl_GetBallFeedforwardAngle(void)
{
    return 0.0f;
}

float CascadeCtrl_GetRailAcceleration(void)
{
    return vehicle_acceleration_mps2;
}

float CascadeCtrl_GetFeedforwardAngle(void)
{
    return vehicle_feedforward_angle;
}

float CascadeCtrl_GetPositionIntegralAngle(void)
{
    return position_integral_output_angle;
}

float CascadeCtrl_GetPositionDerivativeAngle(void)
{
    return position_derivative_output_angle;
}

float CascadeCtrl_GetSlopeCompensationAngle(void)
{
    return slope_compensation_angle;
}

float CascadeCtrl_GetImuTrimAngle(void)
{
    return imu_trim_angle;
}

uint8_t CascadeCtrl_IsAngleSaturated(void)
{
    return angle_saturated;
}

uint8_t CascadeCtrl_IsRecoveryActive(void)
{
    return stiction_compensation_active;
}

uint8_t CascadeCtrl_IsHoldActive(void)
{
    return hold_active;
}

uint8_t CascadeCtrl_IsInputValid(void)
{
    return input_valid;
}

uint8_t CascadeCtrl_IsEnabled(void)
{
    return controller_enabled;
}
