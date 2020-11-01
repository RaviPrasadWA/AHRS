#include "Attitude.h"


void Attitude::set_dt(float delta_sec)
{
    _dt = delta_sec;

    // get filter from ahrs
    const InertialSensor &ins = _ahrs.get_ins();
    float ins_filter = (float)ins.get_filter();

    // sanity check filter
    if (ins_filter <= 0.0f) {
        ins_filter = ATTITUDE_RATE_RP_PID_DTERM_FILTER;
    }

    // set attitude controller's D term filters
    _pid_rate_roll.set_d_lpf_alpha(ins_filter, _dt);
    _pid_rate_pitch.set_d_lpf_alpha(ins_filter, _dt);
    _pid_rate_yaw.set_d_lpf_alpha(ins_filter/2.0f, _dt);  // half
}


void Attitude::relax_bf_rate_controller()
{
    const Vector3f& gyro = _ahrs.get_gyro();
    _rate_bf_target = gyro * ATTITUDE_CONTROL_DEGX100;
}

void Attitude::rate_controller_run()
{
  rate_bf_to_motor_roll_pitch(_rate_bf_target.x, _rate_bf_target.y);
  SERVO_OUT[YAW_CH] = rate_bf_to_motor_yaw(_rate_bf_target.z);
}

void Attitude::rate_bf_to_motor_roll_pitch(float rate_roll_target_cds, float rate_pitch_target_cds)
{
    float roll_pd, roll_i, roll_ff;             // used to capture pid values
    float pitch_pd, pitch_i, pitch_ff;          // used to capture pid values
    float rate_roll_error, rate_pitch_error;    // simply target_rate - current_rate
    float roll_out, pitch_out;
    const Vector3f& gyro = _ahrs.get_gyro();     // get current rates

    // calculate error
    rate_roll_error = rate_roll_target_cds - gyro.x * ATTITUDE_CONTROL_DEGX100;
    rate_pitch_error = rate_pitch_target_cds - gyro.y * ATTITUDE_CONTROL_DEGX100;

    // call p and d controllers
    roll_pd = _pid_rate_roll.get_p(rate_roll_error) + _pid_rate_roll.get_d(rate_roll_error, _dt);
    pitch_pd = _pid_rate_pitch.get_p(rate_pitch_error) + _pid_rate_pitch.get_d(rate_pitch_error, _dt);

    // get roll i term
    roll_i = _pid_rate_roll.get_integrator();

    if (LEAKY)
    {
      roll_i = ((HELICOPTER_PID&)_pid_rate_roll).get_leaky_i(rate_roll_error, _dt, ATTITUDE_HELI_RATE_INTEGRATOR_LEAK_RATE);
    }else{
      roll_i = _pid_rate_roll.get_i(rate_roll_error, _dt);
    }

    // get pitch i term
    pitch_i = _pid_rate_pitch.get_integrator();

    if (LEAKY) 
    {
      pitch_i = ((HELICOPTER_PID&)_pid_rate_pitch).get_leaky_i(rate_pitch_error, _dt, ATTITUDE_HELI_RATE_INTEGRATOR_LEAK_RATE);
    }else{
      pitch_i = _pid_rate_pitch.get_i(rate_pitch_error, _dt);
    }
    
    roll_ff = roll_feedforward_filter.apply(((HELICOPTER_PID&)_pid_rate_roll).get_ff(rate_roll_target_cds));
    pitch_ff = pitch_feedforward_filter.apply(((HELICOPTER_PID&)_pid_rate_pitch).get_ff(rate_pitch_target_cds));

    // add feed forward and final output
    roll_out = roll_pd + roll_i + roll_ff;
    pitch_out = pitch_pd + pitch_i + pitch_ff;

    // constrain output and update limit flags
    if ((float)fabs(roll_out) > ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX) {
        roll_out = constrain_float(roll_out,-ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX,ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX);
    }
    
    if ((float)fabs(pitch_out) > ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX) {
        pitch_out = constrain_float(pitch_out,-ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX,ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX);
    }

    SERVO_OUT[ROLL_CH] = roll_out;
    SERVO_OUT[PITCH_CH] = pitch_out;
}

// rate_bf_to_motor_yaw - ask the rate controller to calculate the motor outputs to achieve the target rate in centi-degrees / second
float Attitude::rate_bf_to_motor_yaw(float rate_target_cds)
{
    float pd,i,ff;            // used to capture pid values for logging
    float current_rate;     // this iteration's rate
    float rate_error;       // simply target_rate - current_rate
    float yaw_out;

    // get current rate
    // To-Do: make getting gyro rates more efficient?
    current_rate = (_ahrs.get_gyro().z * ATTITUDE_CONTROL_DEGX100);

    // calculate error and call pid controller
    rate_error  = rate_target_cds - current_rate;
    pd = _pid_rate_yaw.get_p(rate_error) + _pid_rate_yaw.get_d(rate_error, _dt);

    // get i term
    i = _pid_rate_yaw.get_integrator();

    if (LEAKY) {
      i = ((HELICOPTER_PID&)_pid_rate_yaw).get_leaky_i(rate_error, _dt, ATTITUDE_HELI_RATE_INTEGRATOR_LEAK_RATE);
    } else {
      i = _pid_rate_yaw.get_i(rate_error, _dt);
    }

    
    ff = yaw_feedforward_filter.apply(((HELICOPTER_PID&)_pid_rate_yaw).get_ff(rate_target_cds));
    
    // add feed forward
    yaw_out = pd + i + ff;

    // constrain output and update limit flag
    if ((float)fabs(yaw_out) > ATTITUDE_RATE_YAW_CONTROLLER_OUT_MAX) {
        yaw_out = constrain_float(yaw_out,-ATTITUDE_RATE_YAW_CONTROLLER_OUT_MAX,ATTITUDE_RATE_YAW_CONTROLLER_OUT_MAX);
    }
    
    return yaw_out;
}

void Attitude::update_feedforward_filter_rates(float time_step)
{
    pitch_feedforward_filter.set_cutoff_frequency(time_step, ATTITUDE_HELI_RATE_FF_FILTER);
    roll_feedforward_filter.set_cutoff_frequency(time_step, ATTITUDE_HELI_RATE_FF_FILTER);
    yaw_feedforward_filter.set_cutoff_frequency(time_step, ATTITUDE_HELI_RATE_FF_FILTER);
}

void Attitude::angle_ef_roll_pitch_rate_ef_yaw_smooth(float roll_angle_ef, float pitch_angle_ef, float yaw_rate_ef, float smoothing_gain)
{
  Vector3f angle_ef_error;    // earth frame angle errors
  float rate_change_limit;
  smoothing_gain = constrain_float(smoothing_gain,1.0f,50.0f);
  float linear_angle = _accel_rp_max/(smoothing_gain*smoothing_gain);
  rate_change_limit = _accel_rp_max * _dt;
  float rate_ef_desired;
  float angle_to_target;

  if (_accel_rp_max > 0.0f) {

        // calculate earth-frame feed forward roll rate using linear response when close to the target, sqrt response when we're further away
        angle_to_target = roll_angle_ef - _angle_ef_target.x;        
        if (angle_to_target > linear_angle) {
            rate_ef_desired = safe_sqrt(2.0f*_accel_rp_max*((float)fabs(angle_to_target)-(linear_angle/2.0f)));
        } else if (angle_to_target < -linear_angle) {
            rate_ef_desired = -safe_sqrt(2.0f*_accel_rp_max*((float)fabs(angle_to_target)-(linear_angle/2.0f)));
        } else {
            rate_ef_desired = smoothing_gain*angle_to_target;
        }
        _rate_ef_desired.x = constrain_float(rate_ef_desired, _rate_ef_desired.x-rate_change_limit, _rate_ef_desired.x+rate_change_limit);

        // update earth-frame roll angle target using desired roll rate
        update_ef_roll_angle_and_error(_rate_ef_desired.x, angle_ef_error, ATTITUDE_RATE_STAB_ROLL_OVERSHOOT_ANGLE_MAX);

        // calculate earth-frame feed forward pitch rate using linear response when close to the target, sqrt response when we're further away
        angle_to_target = pitch_angle_ef - _angle_ef_target.y;
        if (angle_to_target > linear_angle) {
            rate_ef_desired = safe_sqrt(2.0f*_accel_rp_max*((float)fabs(angle_to_target)-(linear_angle/2.0f)));
        } else if (angle_to_target < -linear_angle) {
            rate_ef_desired = -safe_sqrt(2.0f*_accel_rp_max*((float)fabs(angle_to_target)-(linear_angle/2.0f)));
        } else {
            rate_ef_desired = smoothing_gain*angle_to_target;
        }
        _rate_ef_desired.y = constrain_float(rate_ef_desired, _rate_ef_desired.y-rate_change_limit, _rate_ef_desired.y+rate_change_limit);

        // update earth-frame pitch angle target using desired pitch rate
        update_ef_pitch_angle_and_error(_rate_ef_desired.y, angle_ef_error, ATTITUDE_RATE_STAB_PITCH_OVERSHOOT_ANGLE_MAX);
    }
    
    // constrain earth-frame angle targets
    _angle_ef_target.x = constrain_float(_angle_ef_target.x, -ATTITUDE_ANGLE_CONTROLLER_ANGLE_MAX, ATTITUDE_ANGLE_CONTROLLER_ANGLE_MAX);
    _angle_ef_target.y = constrain_float(_angle_ef_target.y, -ATTITUDE_ANGLE_CONTROLLER_ANGLE_MAX, ATTITUDE_ANGLE_CONTROLLER_ANGLE_MAX);
    
    if (_accel_y_max > 0.0f) {
      rate_change_limit = _accel_y_max * _dt;
      float rate_change = yaw_rate_ef - _rate_ef_desired.z;
      rate_change = constrain_float(rate_change, -rate_change_limit, rate_change_limit);
      _rate_ef_desired.z += rate_change;
      update_ef_yaw_angle_and_error(_rate_ef_desired.z, angle_ef_error, ATTITUDE_RATE_STAB_YAW_OVERSHOOT_ANGLE_MAX);
    }
    
    frame_conversion_ef_to_bf(angle_ef_error, _angle_bf_error);
    update_rate_bf_targets();
   
    if (_rate_bf_ff_enabled) {
      frame_conversion_ef_to_bf(_rate_ef_desired, _rate_bf_desired);
      _rate_bf_target += _rate_bf_desired;
    } else {
      frame_conversion_ef_to_bf(Vector3f(0,0,_rate_ef_desired.z), _rate_bf_desired);
      _rate_bf_target += _rate_bf_desired;
    }

}

void Attitude::update_ef_roll_angle_and_error(float roll_rate_ef, Vector3f &angle_ef_error, float overshoot_max)
{
    // calculate angle error with maximum of +- max angle overshoot
    angle_ef_error.x  = wrap_180_cd(_angle_ef_target.x - _ahrs.roll_sensor);
    angle_ef_error.x  = constrain_float(angle_ef_error.x, -overshoot_max, overshoot_max);
    
    // update roll angle target to be within max angle overshoot of our roll angle
    _angle_ef_target.x = angle_ef_error.x + _ahrs.roll_sensor;
    
    // increment the roll angle target
    _angle_ef_target.x += roll_rate_ef * _dt;
    _angle_ef_target.x = wrap_180_cd(_angle_ef_target.x);
    
}

void Attitude::update_ef_pitch_angle_and_error(float pitch_rate_ef, Vector3f &angle_ef_error, float overshoot_max)
{
    angle_ef_error.y = wrap_180_cd(_angle_ef_target.y - _ahrs.pitch_sensor);
    angle_ef_error.y  = constrain_float(angle_ef_error.y, -overshoot_max, overshoot_max);
    
    // update pitch angle target to be within max angle overshoot of our pitch angle
    _angle_ef_target.y = angle_ef_error.y + _ahrs.pitch_sensor;

    // increment the pitch angle target
    _angle_ef_target.y += pitch_rate_ef * _dt;
    _angle_ef_target.y = wrap_180_cd(_angle_ef_target.y);
}

void Attitude::update_ef_yaw_angle_and_error(float yaw_rate_ef, Vector3f &angle_ef_error, float overshoot_max)
{
    // calculate angle error with maximum of +- max angle overshoot
    angle_ef_error.z = wrap_180_cd(_angle_ef_target.z - _ahrs.yaw_sensor);
    angle_ef_error.z  = constrain_float(angle_ef_error.z, -overshoot_max, overshoot_max);

    // update yaw angle target to be within max angle overshoot of our current heading
    _angle_ef_target.z = angle_ef_error.z + _ahrs.yaw_sensor;

    // increment the yaw angle target
    _angle_ef_target.z += yaw_rate_ef * _dt;
    _angle_ef_target.z = wrap_360_cd(_angle_ef_target.z);
}

void Attitude::frame_conversion_ef_to_bf(const Vector3f& ef_vector, Vector3f& bf_vector)
{
    // convert earth frame rates to body frame rates
    bf_vector.x = ef_vector.x - _ahrs.sin_pitch() * ef_vector.z;
    bf_vector.y = _ahrs.cos_roll()  * ef_vector.y + _ahrs.sin_roll() * _ahrs.cos_pitch() * ef_vector.z;
    bf_vector.z = -_ahrs.sin_roll() * ef_vector.y + _ahrs.cos_pitch() * _ahrs.cos_roll() * ef_vector.z;
}

void Attitude::update_rate_bf_targets()
{
    // roll/pitch/yaw calculation
    _rate_bf_target.x = _p_angle_roll.kP() * _angle_bf_error.x;
    _rate_bf_target.y = _p_angle_pitch.kP() * _angle_bf_error.y;
    _rate_bf_target.z = _p_angle_yaw.kP() * _angle_bf_error.z;

    if (LIMIT_ANGLE_TO_RATE) {
        _rate_bf_target.x = constrain_float(_rate_bf_target.x,-ATTITUDE_CONTROL_RATE_RP_MAX_DEFAULT,ATTITUDE_CONTROL_RATE_RP_MAX_DEFAULT);
    }
    
    if (LIMIT_ANGLE_TO_RATE) {
        _rate_bf_target.y = constrain_float(_rate_bf_target.y,-ATTITUDE_CONTROL_RATE_RP_MAX_DEFAULT,ATTITUDE_CONTROL_RATE_RP_MAX_DEFAULT);
    }

    if (LIMIT_ANGLE_TO_RATE) {
        _rate_bf_target.z = constrain_float(_rate_bf_target.z,-ATTITUDE_CONTROL_RATE_Y_MAX_DEFAULT,ATTITUDE_CONTROL_RATE_Y_MAX_DEFAULT);
    }
    
    _rate_bf_target.x += _angle_bf_error.y * _ahrs.get_gyro().z;
    _rate_bf_target.y += -_angle_bf_error.x * _ahrs.get_gyro().z;
}

