/// @file    AP_MotorsTrident.h
/// @brief   Custom tandem ducted VTOL + tilting rear pusher mixer for Trident BWB UAV
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <Filter/LowPassFilter.h>
#include "AP_MotorsMulticopter.h"

#define AP_MOTORS_TRIDENT_SERVO_RANGE 4500

/// @class   AP_MotorsTrident
/// @brief   2-fan tandem VTOL mixer:
///          - Motor 1 (ESC1): Center fixed vertical 7" ducted fan (lift + pitch differential + downwash)
///          - Motor 2 (ESC2): Rear tilting 6" fan (lift + pitch differential in VTOL, pusher in FW)
///          - VANE1 / VANE2: Tandem 4-sub-vane banks below center duct (collective=Roll, differential=Yaw)
class AP_MotorsTrident : public AP_MotorsMulticopter {
public:
    AP_MotorsTrident(uint16_t speed_hz = AP_MOTORS_SPEED_DEFAULT);

    void init(motor_frame_class frame_class, motor_frame_type frame_type) override;
    void set_frame_class_and_type(motor_frame_class frame_class, motor_frame_type frame_type) override {}
    void set_update_rate(uint16_t speed_hz) override;
    void output_to_motors() override;
    uint32_t get_motor_mask() override;
    bool arming_checks(size_t buflen, char *buffer) const override;

    // Return optional Option-B hover rear-tilt pitch/surge offset in k_motor_tilt units (0..1000 full 90 deg)
    float get_tilt_pitch_offset() const;

    static const struct AP_Param::GroupInfo var_info[];

protected:
    void output_armed_stabilizing() override;
    void thrust_compensation(void) override;
    const char* _get_frame_string() const override { return "TRIDENT_VTOL"; }
    void _output_test_seq(uint8_t motor_seq, int16_t pwm) override;

private:
    void update_vane_mix();

    // Parameters (Q_M_ prefix in Plane; all suffixes <= 8 chars so total name <= 12 chars)
    AP_Float _front_thr;      // Center 7" static hover thrust multiplier
    AP_Float _rear_thr;       // Rear 6" static hover thrust multiplier
    AP_Float _pitch_gain;     // Differential throttle pitch authority
    AP_Float _vane_roll_gain; // Collective vane gain for roll
    AP_Float _vane_yaw_gain;  // Differential vane gain for yaw
    AP_Float _yaw_trim;       // Constant vane yaw trim to counter co-rotating CW motor torque
    AP_Float _tilt_pitch_gain;// Option-B hover rear-tilt pitch/surge coupling gain
    AP_Float _tilt_max_deg;   // Max Option-B hover tilt vector offset (deg)
    AP_Float _vane_thr_min;   // Minimum downwash denominator floor for vane scaling

    // Computed mixing state
    float _thrust_raw[2];     // Uncompensated normalized thrust for Motor1 (ESC1) and Motor2 (ESC2)
    float _roll_cmd;          // Voltage/air-density compensated roll command
    float _yaw_cmd;           // Voltage/air-density compensated yaw command
    float _vane_front_out;    // Front vane normalized deflection (-1..+1)
    float _vane_rear_out;     // Rear vane normalized deflection (-1..+1)
    LowPassFilterFloat _tilt_pitch_filt{8.0f}; // ~8 Hz smoothing so D-term noise doesn't chatter TILT1
};
