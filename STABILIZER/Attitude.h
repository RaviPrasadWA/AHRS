#ifndef __ATTITUDE_H__
#define __ATTITUDE_H__

#include "AP_Param.h"
#include "PID.h"
#include "LowPassFilter.h"

#define LEAKY                                        1 // Leak Intergrator enable/disable
#define LIMIT_ANGLE_TO_RATE                          1 // constrain angle to limit -3000 -> 3000
#define ATTITUDE_100HZ_DT                            0.0100f // delta time in seconds for 100hz update rate
#define ATTITUDE_CONTROL_DEGX100                     5729.57795f // constant to convert from radians to centi-degrees
#define ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX          5000.0f // body-frame rate controller maximum output (for roll-pitch axis)
#define ATTITUDE_RATE_YAW_CONTROLLER_OUT_MAX         4500.0f // body-frame rate controller maximum output (for yaw axis)
#define ATTITUDE_ANGLE_CONTROLLER_ANGLE_MAX          4500.0f 
#define ATTITUDE_HELI_RATE_INTEGRATOR_LEAK_RATE      0.02f
#define ATTITUDE_HELI_RATE_FF_FILTER                 5.0f
#define ATTITUDE_RATE_STAB_ACRO_OVERSHOOT_ANGLE_MAX  1000.0f // earth-frame rate stabilize controller's maximum overshoot angle
#define ATTITUDE_CONTROL_RATE_RP_MAX_DEFAULT         18000
#define ATTITUDE_CONTROL_RATE_Y_MAX_DEFAULT          9000

//#define ATTITUDE_RATE_STAB_ROLL_OVERSHOOT_ANGLE_MAX  3000.0f
//#define ATTITUDE_RATE_STAB_PITCH_OVERSHOOT_ANGLE_MAX 3000.0f 
#define ATTITUDE_RATE_STAB_ROLL_OVERSHOOT_ANGLE_MAX  100.0f
#define ATTITUDE_RATE_STAB_PITCH_OVERSHOOT_ANGLE_MAX 100.0f 

#define ATTITUDE_RATE_STAB_YAW_OVERSHOOT_ANGLE_MAX   1000.0f
#define ATTITUDE_RATE_RP_PID_DTERM_FILTER            20      // D-term filter rate cutoff frequency for Roll and Pitch rate controllers

class Attitude 
{
  public:
    Attitude( S_P& p_angle_roll, 
              S_P& p_angle_pitch, 
              S_P& p_angle_yaw,
              HELICOPTER_PID& pid_rate_roll, 
              HELICOPTER_PID& pid_rate_pitch, 
              HELICOPTER_PID& pid_rate_yaw ):
                _pid_rate_roll(pid_rate_roll),
                _pid_rate_pitch(pid_rate_pitch),
                _pid_rate_yaw(pid_rate_yaw),
                _p_angle_roll(p_angle_roll),
                _p_angle_pitch(p_angle_pitch),
                _p_angle_yaw(p_angle_yaw),
                _dt(ATTITUDE_100HZ_DT)
              {
                _accel_rp_max         = 108000;          // @Values: 0:Disabled, 72000:Slow, 108000:Medium, 162000:Fast
                _accel_y_max          = 36000;          // @Values: 0:Disabled, 18000:Slow, 36000:Medium, 54000:Fast
                _rate_bf_ff_enabled   = 1;             // @Values: Enabled:1 , Disabled:0
              }
    void rate_controller_run();
    void relax_bf_rate_controller();
    void rate_bf_to_motor_roll_pitch(float rate_roll_target_cds, float rate_pitch_target_cds);
    float rate_bf_to_motor_yaw(float rate_target_cds);
    void update_feedforward_filter_rates(float time_step);
    void angle_ef_roll_pitch_rate_ef_yaw_smooth(float roll_angle_ef, float pitch_angle_ef, float yaw_rate_ef, float smoothing_gain);
    void update_ef_roll_angle_and_error(float roll_rate_ef, Vector3f &angle_ef_error, float overshoot_max);
    void update_ef_pitch_angle_and_error(float pitch_rate_ef, Vector3f &angle_ef_error, float overshoot_max);
    void update_ef_yaw_angle_and_error(float yaw_rate_ef, Vector3f &angle_ef_error, float overshoot_max);
    void frame_conversion_ef_to_bf(const Vector3f& ef_vector, Vector3f& bf_vector);
    void update_rate_bf_targets();
    void set_dt(float delta_sec);
    
  protected:
  
    AP_Float            _accel_rp_max;
    AP_Float            _accel_y_max;           // maximum rotation acceleration for earth-frame yaw axis
    AP_Int8             _rate_bf_ff_enabled;    // Enable/Disable body frame rate feed forward
    
    float               _dt;                    // time delta in seconds
    
    Vector3f            _angle_ef_target;       // angle controller earth-frame targets
    Vector3f            _rate_ef_desired;       // earth-frame feed forward rates
    
    Vector3f            _rate_bf_target;        // rate controller body-frame targets
    Vector3f            _angle_bf_error;        // angle controller body-frame error
    Vector3f            _rate_bf_desired;       // body-frame feed forward rates
    
    PID&                _pid_rate_roll;
    PID&                _pid_rate_pitch;
    PID&                _pid_rate_yaw;
    S_P&	        _p_angle_roll;
    S_P&	        _p_angle_pitch;
    S_P&	        _p_angle_yaw;
    
    LowPassFilterInt32 pitch_feedforward_filter;
    LowPassFilterInt32 roll_feedforward_filter;
    LowPassFilterInt32 yaw_feedforward_filter;
};
#endif //__ATTITUDE_H__ 
