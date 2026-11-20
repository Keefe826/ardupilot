/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <SRV_Channel/SRV_Channel.h>
#include "AP_MotorsTrident.h"

extern const AP_HAL::HAL& hal;

// Parameter indices >= 50 stay cleanly clear of AP_MotorsMulticopter (0..45)
const AP_Param::GroupInfo AP_MotorsTrident::var_info[] = {
    AP_NESTEDGROUPINFO(AP_MotorsMulticopter, 0),

    // @Param: TR_THR_F
    // @DisplayName: Trident front/center fan hover thrust weight
    // @Description: Relative thrust weight applied to center 7-inch fan (Motor1/ESC1) in VTOL hover to balance CG moment arm.
    // @Range: 0.2 2.0
    // @Increment: 0.02
    // @User: Advanced
    AP_GROUPINFO("TR_THR_F", 50, AP_MotorsTrident, _front_thr, 1.0f),

    // @Param: TR_THR_R
    // @DisplayName: Trident rear fan hover thrust weight
    // @Description: Relative thrust weight applied to rear 6-inch tilting fan (Motor2/ESC2) in VTOL hover to balance CG moment arm (d_front/T_max1 vs d_rear/T_max2).
    // @Range: 0.2 2.0
    // @Increment: 0.02
    // @User: Advanced
    AP_GROUPINFO("TR_THR_R", 51, AP_MotorsTrident, _rear_thr, 0.66f),

    // @Param: TR_PTC_G
    // @DisplayName: Trident differential pitch gain
    // @Description: Differential throttle authority between front center fan and rear tilting fan for VTOL pitch control.
    // @Range: 0.1 1.5
    // @Increment: 0.05
    // @User: Advanced
    AP_GROUPINFO("TR_PTC_G", 52, AP_MotorsTrident, _pitch_gain, 0.50f),

    // @Param: TR_ROL_G
    // @DisplayName: Trident vane collective roll gain
    // @Description: Scale factor (and polarity sign) mapping roll command into collective front+rear center-duct vane deflection.
    // @Range: -2.0 2.0
    // @Increment: 0.05
    // @User: Advanced
    AP_GROUPINFO("TR_ROL_G", 53, AP_MotorsTrident, _vane_roll_gain, 1.0f),

    // @Param: TR_YAW_G
    // @DisplayName: Trident vane differential yaw gain
    // @Description: Scale factor (and polarity sign) mapping yaw command into opposing front-vs-rear center-duct vane deflection.
    // @Range: -2.0 2.0
    // @Increment: 0.05
    // @User: Advanced
    AP_GROUPINFO("TR_YAW_G", 54, AP_MotorsTrident, _vane_yaw_gain, 1.0f),

    // @Param: TR_YAW_T
    // @DisplayName: Trident static yaw vane trim
    // @Description: Constant differential vane offset (-0.5 to +0.5 of full throw) to counter steady reaction torque when both fans rotate CW.
    // @Range: -0.5 0.5
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("TR_YAW_T", 55, AP_MotorsTrident, _yaw_trim, 0.0f),

    // @Param: TR_TLT_P
    // @DisplayName: Trident Option-B hover tilt pitch gain
    // @Description: Gain coupling VTOL pitch output into rear fan tilt servo (0 disables direct pitch-tilt coupling; use Q_FWD_THR_GAIN for position-hold forward tilt). Positive tilts rear nozzle more forward on nose-down pitch demand.
    // @Range: -1.0 1.0
    // @Increment: 0.05
    // @User: Advanced
    AP_GROUPINFO("TR_TLT_P", 56, AP_MotorsTrident, _tilt_pitch_gain, 0.0f),

    // @Param: TR_TLT_M
    // @DisplayName: Trident Option-B max hover pitch tilt angle
    // @Description: Maximum deflection in degrees added to rear tilt servo from Q_M_TR_TLT_P during hover.
    // @Units: deg
    // @Range: 0 30
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("TR_TLT_M", 57, AP_MotorsTrident, _tilt_max_deg, 15.0f),

    // @Param: TR_V_MIN
    // @DisplayName: Trident minimum vane downwash floor
    // @Description: Minimum normalized ESC1 thrust used when scaling vane angles by center-duct downwash velocity, preventing excessive deflection at idle throttle.
    // @Range: 0.15 0.8
    // @Increment: 0.05
    // @User: Advanced
    AP_GROUPINFO("TR_V_MIN", 58, AP_MotorsTrident, _vane_thr_min, 0.35f),

    AP_GROUPEND
};

AP_MotorsTrident::AP_MotorsTrident(uint16_t speed_hz) :
    AP_MotorsMulticopter(speed_hz)
{
    AP_Param::setup_object_defaults(this, var_info);
}

void AP_MotorsTrident::init(motor_frame_class frame_class, motor_frame_type frame_type)
{
    // Default carrier outputs: ESC1=CH8 (Motor1), ESC2=CH9 (Motor2), VANE1=CH10, VANE2=CH11
    SRV_Channels::set_aux_channel_default(SRV_Channel::k_motor1, CH_8);
    SRV_Channels::set_aux_channel_default(SRV_Channel::k_motor2, CH_9);
    SRV_Channels::set_aux_channel_default(SRV_Channel::k_trident_vane_front, CH_10);
    SRV_Channels::set_aux_channel_default(SRV_Channel::k_trident_vane_rear, CH_11);

    SRV_Channels::set_angle(SRV_Channel::k_trident_vane_front, AP_MOTORS_TRIDENT_SERVO_RANGE);
    SRV_Channels::set_angle(SRV_Channel::k_trident_vane_rear, AP_MOTORS_TRIDENT_SERVO_RANGE);

    motor_enabled[AP_MOTORS_MOT_1] = true;
    motor_enabled[AP_MOTORS_MOT_2] = true;

    _mav_type = MAV_TYPE_VTOL_TILTROTOR;
    set_update_rate(_speed_hz);
    set_initialised_ok(frame_class == MOTOR_FRAME_TRIDENT);
}

void AP_MotorsTrident::set_update_rate(uint16_t speed_hz)
{
    _speed_hz = speed_hz;
    const uint32_t mask = (1U << AP_MOTORS_MOT_1) | (1U << AP_MOTORS_MOT_2);
    rc_set_freq(mask, _speed_hz);
}

uint32_t AP_MotorsTrident::get_motor_mask()
{
    const uint32_t motor_mask = (1U << AP_MOTORS_MOT_1) | (1U << AP_MOTORS_MOT_2);
    uint32_t mask = motor_mask_to_srv_channel_mask(motor_mask);
    mask |= AP_MotorsMulticopter::get_motor_mask();
    return mask;
}

bool AP_MotorsTrident::arming_checks(size_t buflen, char *buffer) const
{
    if (!AP_MotorsMulticopter::arming_checks(buflen, buffer)) {
        return false;
    }
    uint8_t chan;
    if (!SRV_Channels::find_channel(SRV_Channel::k_trident_vane_front, chan)) {
        hal.util->snprintf(buffer, buflen, "no SERVOx_FUNCTION set to TridentFrontVane");
        return false;
    }
    if (!SRV_Channels::find_channel(SRV_Channel::k_trident_vane_rear, chan)) {
        hal.util->snprintf(buffer, buflen, "no SERVOx_FUNCTION set to TridentRearVane");
        return false;
    }
    return true;
}

void AP_MotorsTrident::output_armed_stabilizing()
{
    const float compensation_gain = thr_lin.get_compensation_gain();
    _roll_cmd = (_roll_in + _roll_in_ff) * compensation_gain;
    const float pitch_thrust = (_pitch_in + _pitch_in_ff) * compensation_gain;
    _yaw_cmd = (_yaw_in + _yaw_in_ff) * compensation_gain;

    float throttle_thrust = get_throttle() * compensation_gain;
    const float throttle_thrust_max = _throttle_thrust_max * compensation_gain;

    if (throttle_thrust <= 0.0f) {
        throttle_thrust = 0.0f;
        limit.throttle_lower = true;
    }
    if (throttle_thrust >= throttle_thrust_max) {
        throttle_thrust = throttle_thrust_max;
        limit.throttle_upper = true;
    }

    const float throttle_avg_max = constrain_float(_throttle_avg_max * compensation_gain,
                                                   throttle_thrust,
                                                   throttle_thrust_max);

    const float w_f = constrain_float(_front_thr, 0.2f, 2.0f);
    const float w_r = constrain_float(_rear_thr, 0.2f, 2.0f);
    const float pitch_k = constrain_float(_pitch_gain, 0.05f, 1.5f);

    float dp_f = pitch_thrust * pitch_k * w_f;
    float dp_r = -pitch_thrust * pitch_k * w_r;

    float t_f = throttle_thrust * w_f + dp_f;
    float t_r = throttle_thrust * w_r + dp_r;

    // Shift collective if a motor drops below 0 or exceeds 1 while preserving pitch priority within throttle_avg_max
    float thr_shift = 0.0f;
    const float t_max = MAX(t_f, t_r);
    const float t_min = MIN(t_f, t_r);
    if (t_max > 1.0f) {
        thr_shift = 1.0f - t_max;
        limit.throttle_upper = true;
    } else if (t_min < 0.0f) {
        const float boost_cap = MAX(0.0f, (throttle_avg_max - throttle_thrust) * MIN(w_f, w_r));
        thr_shift = MIN(-t_min, boost_cap);
        limit.throttle_lower = true;
    }

    t_f += thr_shift;
    t_r += thr_shift;

    // If differential pitch still exceeds [0, 1] on either fan, scale pitch delta proportionally
    if (t_f > 1.0f || t_f < 0.0f || t_r > 1.0f || t_r < 0.0f) {
        float scale = 1.0f;
        const float base_f = constrain_float(throttle_thrust * w_f + thr_shift, 0.0f, 1.0f);
        const float base_r = constrain_float(throttle_thrust * w_r + thr_shift, 0.0f, 1.0f);

        if (dp_f > 0.0f) {
            scale = MIN(scale, (1.0f - base_f) / dp_f);
        } else if (dp_f < 0.0f) {
            scale = MIN(scale, base_f / (-dp_f));
        }
        if (dp_r > 0.0f) {
            scale = MIN(scale, (1.0f - base_r) / dp_r);
        } else if (dp_r < 0.0f) {
            scale = MIN(scale, base_r / (-dp_r));
        }
        scale = constrain_float(scale, 0.0f, 1.0f);
        if (scale < 1.0f) {
            limit.pitch = true;
        }
        t_f = base_f + dp_f * scale;
        t_r = base_r + dp_r * scale;
    }

    _thrust_raw[0] = constrain_float(t_f, 0.0f, 1.0f);
    _thrust_raw[1] = constrain_float(t_r, 0.0f, 1.0f);

    const float avg_thr = 0.5f * ((_thrust_raw[0] / w_f) + (_thrust_raw[1] / w_r));
    _throttle_out = constrain_float(avg_thr / compensation_gain, 0.0f, 1.0f);

    // Smooth raw pitch command for optional Option-B hover tilt vectoring
    const float raw_tilt_pitch = _pitch_in + _pitch_in_ff;
    if (_dt_s > 0.0f) {
        _tilt_pitch_filt.apply(raw_tilt_pitch, _dt_s);
    }
}

void AP_MotorsTrident::update_vane_mix()
{
    const float downwash_floor = constrain_float(_vane_thr_min, 0.15f, 0.8f);
    const float esc1_downwash = MAX(_thrust_raw[0], downwash_floor);

    const float roll_v = (_roll_cmd * _vane_roll_gain) / esc1_downwash;
    const float yaw_v  = ((_yaw_cmd * _vane_yaw_gain) / esc1_downwash) + _yaw_trim;

    float vf = roll_v + yaw_v;
    float vr = roll_v - yaw_v;

    const float vmax = MAX(fabsf(vf), fabsf(vr));
    if (vmax > 1.0f) {
        vf /= vmax;
        vr /= vmax;
        limit.roll = true;
        limit.yaw = true;
    }

    _vane_front_out = constrain_float(vf, -1.0f, 1.0f);
    _vane_rear_out  = constrain_float(vr, -1.0f, 1.0f);
}

void AP_MotorsTrident::thrust_compensation(void)
{
    if (_thrust_compensation_callback) {
        _thrust_compensation_callback(_thrust_raw, 2);
    }
    update_vane_mix();
}

void AP_MotorsTrident::output_to_motors()
{
    if (!initialised_ok()) {
        return;
    }

    switch (_spool_state) {
        case SpoolState::SHUT_DOWN:
            for (uint8_t i = 0; i < 2; i++) {
                if (motor_enabled_mask(i)) {
                    _actuator[i] = 0.0f;
                    rc_write(i, output_to_pwm(0.0f));
                }
            }
            // Keep vanes strictly centered at zero deflection during cruise / disarm
            SRV_Channels::set_output_scaled(SRV_Channel::k_trident_vane_front, 0.0f);
            SRV_Channels::set_output_scaled(SRV_Channel::k_trident_vane_rear, 0.0f);
            _tilt_pitch_filt.reset(0.0f);
            break;

        case SpoolState::GROUND_IDLE:
            for (uint8_t i = 0; i < 2; i++) {
                if (motor_enabled_mask(i)) {
                    set_actuator_with_slew(_actuator[i], actuator_spin_up_to_ground_idle());
                    rc_write(i, output_to_pwm(_actuator[i]));
                }
            }
            SRV_Channels::set_output_scaled(SRV_Channel::k_trident_vane_front,
                                            _spin_up_ratio * _vane_front_out * AP_MOTORS_TRIDENT_SERVO_RANGE);
            SRV_Channels::set_output_scaled(SRV_Channel::k_trident_vane_rear,
                                            _spin_up_ratio * _vane_rear_out * AP_MOTORS_TRIDENT_SERVO_RANGE);
            _tilt_pitch_filt.reset(0.0f);
            break;

        case SpoolState::SPOOLING_UP:
        case SpoolState::THROTTLE_UNLIMITED:
        case SpoolState::SPOOLING_DOWN:
            for (uint8_t i = 0; i < 2; i++) {
                if (motor_enabled_mask(i)) {
                    set_actuator_with_slew(_actuator[i], thr_lin.thrust_to_actuator(_thrust_raw[i]));
                    rc_write(i, output_to_pwm(_actuator[i]));
                }
            }
            SRV_Channels::set_output_scaled(SRV_Channel::k_trident_vane_front,
                                            _vane_front_out * AP_MOTORS_TRIDENT_SERVO_RANGE);
            SRV_Channels::set_output_scaled(SRV_Channel::k_trident_vane_rear,
                                            _vane_rear_out * AP_MOTORS_TRIDENT_SERVO_RANGE);
            break;
    }
}

float AP_MotorsTrident::get_tilt_pitch_offset() const
{
    if (_spool_state != SpoolState::THROTTLE_UNLIMITED || is_zero(_tilt_pitch_gain)) {
        return 0.0f;
    }
    const float max_deg = constrain_float(_tilt_max_deg, 0.0f, 30.0f);
    // Negative pitch command (nose down / forward) produces positive forward tilt (>0 toward cruise)
    const float tilt_deg = constrain_float(-_tilt_pitch_filt.get() * _tilt_pitch_gain * max_deg, -max_deg, max_deg);
    return tilt_deg * (1000.0f / 90.0f);
}

void AP_MotorsTrident::_output_test_seq(uint8_t motor_seq, int16_t pwm)
{
    switch (motor_seq) {
        case 1:
            rc_write(AP_MOTORS_MOT_1, pwm);
            break;
        case 2:
            rc_write(AP_MOTORS_MOT_2, pwm);
            break;
        case 3:
            SRV_Channels::set_output_pwm(SRV_Channel::k_trident_vane_front, pwm);
            break;
        case 4:
            SRV_Channels::set_output_pwm(SRV_Channel::k_trident_vane_rear, pwm);
            break;
        default:
            break;
    }
}
