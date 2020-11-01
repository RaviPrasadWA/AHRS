#include "DCM.h"

// set_trim
void DCM_BASE::set_trim(Vector3f new_trim)
{
    Vector3f trim;
    trim.x = constrain_float(new_trim.x, ToRad(-AHRS_TRIM_LIMIT), ToRad(AHRS_TRIM_LIMIT));
    trim.y = constrain_float(new_trim.y, ToRad(-AHRS_TRIM_LIMIT), ToRad(AHRS_TRIM_LIMIT));
    _trim.set(trim);
}

// add_trim - adjust the roll and pitch trim up to a total of 10 degrees
void DCM_BASE::add_trim(float roll_in_radians, float pitch_in_radians, bool save_to_eeprom)
{
    Vector3f trim = _trim.get();

    // add new trim
    trim.x = constrain_float(trim.x + roll_in_radians, ToRad(-AHRS_TRIM_LIMIT), ToRad(AHRS_TRIM_LIMIT));
    trim.y = constrain_float(trim.y + pitch_in_radians, ToRad(-AHRS_TRIM_LIMIT), ToRad(AHRS_TRIM_LIMIT));

    // set new trim values
    _trim.set(trim);

    // save to eeprom
    if( save_to_eeprom ) {
        //_trim.save();
    }
}


// update_trig - recalculates _cos_roll, _cos_pitch, etc based on latest attitude
//      should be called after _dcm_matrix is updated
void DCM_BASE::update_trig(void)
{
    Vector2f yaw_vector;
    const Matrix3f &temp = get_dcm_matrix();

    // sin_yaw, cos_yaw
    yaw_vector.x = temp.a.x;
    yaw_vector.y = temp.b.x;
    yaw_vector.normalize();
    _sin_yaw = constrain_float(yaw_vector.y, -1.0, 1.0);
    _cos_yaw = constrain_float(yaw_vector.x, -1.0, 1.0);

    // cos_roll, cos_pitch
    _cos_pitch = safe_sqrt(1 - (temp.c.x * temp.c.x));
    _cos_roll = temp.c.z / _cos_pitch;
    _cos_pitch = constrain_float(_cos_pitch, 0, 1.0);
    _cos_roll = constrain_float(_cos_roll, -1.0, 1.0); // this relies on constrain_float() of infinity doing the right thing,which it does do in avr-libc

    // sin_roll, sin_pitch
    _sin_pitch = -temp.c.x;
    _sin_roll = temp.c.y / _cos_pitch;
}

/*
  update the centi-degree values
 */
void DCM_BASE::update_cd_values(void)
{
    roll_sensor  = degrees(roll) * 100;
    pitch_sensor = degrees(pitch) * 100;
    yaw_sensor   = degrees(yaw) * 100;
 
    if (yaw_sensor < 0)
        yaw_sensor += 36000;
}

// reset the current gyro drift estimate
//  should be called if gyro offsets are recalculated
void DCM::reset_gyro_drift(void)
{
    _omega_I.zero();
    _omega_I_sum.zero();
    _omega_I_sum_time = 0;
}

// run a full DCM update round
void DCM::update(void)
{
    float delta_t;

    // tell the IMU to grab some data
    _ins.update();

    // ask the IMU how much time this sensor reading represents
    delta_t = _ins.get_delta_time();

    if (delta_t > 0.2f) {
        memset(&_ra_sum[0], 0, sizeof(_ra_sum));
        _ra_deltat = 0;
        return;
    }

    // Integrate the DCM matrix using gyro inputs
    matrix_update(delta_t);

    // Normalize the DCM matrix
    normalize();

    // Perform drift correction
    drift_correction(delta_t);

    // paranoid check for bad values in the DCM matrix
    check_matrix();

    // Calculate pitch, roll, yaw for stabilization and navigation
    euler_angles();

    // update trig values including _cos_roll, cos_pitch
    update_trig();
}

// update the DCM matrix using only the gyros
void DCM::matrix_update(float _G_Dt)
{
    // note that we do not include the P terms in _omega. This is
    // because the spin_rate is calculated from _omega.length(),
    // and including the P terms would give positive feedback into
    // the _P_gain() calculation, which can lead to a very large P value
    _omega.zero();

    // average across all healthy gyros. This reduces noise on systems
    // with more than one gyro    
    uint8_t healthy_count = 0;    
    for (uint8_t i=0; i<_ins.get_gyro_count(); i++) {
        if (_ins.get_gyro_health(i)) {
            _omega += _ins.get_gyro(i);
            healthy_count++;
        }
    }
    if (healthy_count > 1) {
        _omega /= healthy_count;
    }
    _omega += _omega_I;
    _dcm_matrix.rotate((_omega + _omega_P + _omega_yaw_P) * _G_Dt);
}

/*
 *  reset the DCM matrix and omega. Used on ground start, and on
 *  extreme errors in the matrix
 */
void DCM::reset(bool recover_eulers)
{
    // reset the integration terms
    _omega_I.zero();
    _omega_P.zero();
    _omega_yaw_P.zero();
    _omega.zero();

    // if the caller wants us to try to recover to the current
    // attitude then calculate the dcm matrix from the current
    // roll/pitch/yaw values
    if (recover_eulers && !isnan(roll) && !isnan(pitch) && !isnan(yaw)) {
        _dcm_matrix.from_euler(roll, pitch, yaw);
    } else {
        // otherwise make it flat
        _dcm_matrix.from_euler(0, 0, 0);
    }
}

// reset the current attitude, used by HIL
void DCM::reset_attitude(const float &_roll, const float &_pitch, const float &_yaw)
{
    _dcm_matrix.from_euler(_roll, _pitch, _yaw);    
}

/*
 *  check the DCM matrix for pathological values
 */
void DCM::check_matrix(void)
{
    if (_dcm_matrix.is_nan()) {
        //Serial.printf("ERROR: DCM matrix NAN\n");
        DCM::reset(true);
        return;
    }
    // some DCM matrix values can lead to an out of range error in
    // the pitch calculation via asin().  These NaN values can
    // feed back into the rest of the DCM matrix via the
    // error_course value.
    if (!(_dcm_matrix.c.x < 1.0f &&
          _dcm_matrix.c.x > -1.0f)) {
        // We have an invalid matrix. Force a normalisation.
        normalize();

        if (_dcm_matrix.is_nan() ||
            fabsf(_dcm_matrix.c.x) > 10) {
              DCM::reset(true);
        }
    }
}

// renormalise one vector component of the DCM matrix
// this will return false if renormalization fails
bool DCM::renorm(Vector3f const &a, Vector3f &result)
{
    float renorm_val;

    // numerical errors will slowly build up over time in DCM,
    // causing inaccuracies. We can keep ahead of those errors
    // using the renormalization technique from the DCM IMU paper
    // (see equations 18 to 21).

    // optimisation from the paper as on our 2560 CPU the cost of
    // the sqrt() is 44 microseconds, and the small time saving of
    // the taylor expansion is not worth the potential of
    // additional error buildup.

    renorm_val = 1.0f / a.length();

    // keep the average for reporting
    _renorm_val_sum += renorm_val;
    _renorm_val_count++;

    if (!(renorm_val < 2.0f && renorm_val > 0.5f)) {
        // this is larger than it should get - log it as a warning
        if (!(renorm_val < 1.0e6f && renorm_val > 1.0e-6f)) {
            // we are getting values which are way out of
            // range, we will reset the matrix and hope we
            // can recover our attitude using drift
            // correction before we hit the ground!
            //Serial.printf("ERROR: DCM renormalisation error. renorm_val=%f\n",
            //	   renorm_val);
            return false;
        }
    }

    result = a * renorm_val;
    return true;
}

/*************************************************
 *  Direction Cosine Matrix IMU: Theory
 *  William Premerlani and Paul Bizard
 *
 *  Numerical errors will gradually reduce the orthogonality conditions expressed by equation 5
 *  to approximations rather than identities. In effect, the axes in the two frames of reference no
 *  longer describe a rigid body. Fortunately, numerical error accumulates very slowly, so it is a
 *  simple matter to stay ahead of it.
 *  We call the process of enforcing the orthogonality conditions -> renormalization.
 */
void DCM::normalize(void)
{
    float error;
    Vector3f t0, t1, t2;

    error = _dcm_matrix.a * _dcm_matrix.b;                                              // eq.18

    t0 = _dcm_matrix.a - (_dcm_matrix.b * (0.5f * error));              // eq.19
    t1 = _dcm_matrix.b - (_dcm_matrix.a * (0.5f * error));              // eq.19
    t2 = t0 % t1;                                                       // c= a x b // eq.20

    if (!renorm(t0, _dcm_matrix.a) ||
        !renorm(t1, _dcm_matrix.b) ||
        !renorm(t2, _dcm_matrix.c)) 
    {
        _last_failure_ms = millis();
        DCM::reset(true);
    }
}

// produce a yaw error value. The returned value is proportional
// to sin() of the current heading error in earth frame
float DCM::yaw_error_compass(void)
{
    const Vector3f &mag = _compass->get_field();
    // get the mag vector in the earth frame
    Vector2f rb = _dcm_matrix.mulXY(mag);

    rb.normalize();
    if (rb.is_inf()) {
        // not a valid vector
        return 0.0;
    }

    // update vector holding earths magnetic field (if required)
    if( _last_declination != _compass->get_declination() ) {
        _last_declination = _compass->get_declination();
        _mag_earth.x = cosf(_last_declination);
        _mag_earth.y = sinf(_last_declination);
    }

    // calculate the error term in earth frame
    // calculate the Z component of the cross product of rb and _mag_earth
    return rb % _mag_earth;
}


// the _P_gain raises the gain of the PI controller
// when we are spinning fast. See the fastRotations
// paper from Bill.
float DCM::_P_gain(float spin_rate)
{
    if (spin_rate < ToRad(50)) {
        return 1.0f;
    }
    if (spin_rate > ToRad(500)) {
        return 10.0f;
    }
    return spin_rate/ToRad(50);
}

// _yaw_gain reduces the gain of the PI controller applied to heading errors
// when observability from change of velocity is good (eg changing speed or turning)
// This reduces unwanted roll and pitch coupling due to compass errors for planes.
// High levels of noise on _accel_ef will cause the gain to drop and could lead to 
// increased heading drift during straight and level flight, however some gain is
// always available. TODO check the necessity of adding adjustable acc threshold 
// and/or filtering accelerations before getting magnitude
float DCM::_yaw_gain(void) const
{
    float VdotEFmag = pythagorous2(_accel_ef[_active_accel_instance].x,
                                   _accel_ef[_active_accel_instance].y);
    if (VdotEFmag <= 4.0f) {
        return 0.2f*(4.5f - VdotEFmag);
    }
    return 0.1f;
}

// return true if we should use the compass for yaw correction
bool DCM::use_compass(void)
{
  if (!_compass || !_compass->use_for_yaw()) {
        // no compass available
        return false;
    }
    
  _last_consistent_heading = millis();
  return true;
}

// yaw drift correction using the compass 
// this function prodoces the _omega_yaw_P vector, and also
// contributes to the _omega_I.z long term yaw drift estimate
void DCM::drift_correction_yaw(void)
{
    bool new_value = false;
    float yaw_error;
    float yaw_deltat;
    
    if (DCM::use_compass()) {
      /*
          we are using compass for yaw
      */
      if (_compass->last_update != _compass_last_update) {
        yaw_deltat = (_compass->last_update - _compass_last_update) * 1.0e-6f;
        _compass_last_update = _compass->last_update;
        // we force an additional compass read()
        // here. This has the effect of throwing away
        // the first compass value, which can be bad
        if (!_flags.have_initial_yaw && _compass->read()) {
          float heading = _compass->calculate_heading(_dcm_matrix);
          _dcm_matrix.from_euler(roll, pitch, heading);
          _omega_yaw_P.zero();
          _flags.have_initial_yaw = true;
         }
         new_value = true;
         yaw_error = yaw_error_compass();
       }
     }
     
      if (!new_value) {
        // we don't have any new yaw information
        // slowly decay _omega_yaw_P to cope with loss
        // of our yaw source
        _omega_yaw_P *= 0.97f;
        return;
    }

    // convert the error vector to body frame
    float error_z = _dcm_matrix.c.z * yaw_error;

    // the spin rate changes the P gain, and disables the
    // integration at higher rates
    float spin_rate = _omega.length();

    // sanity check _kp_yaw
    if (_kp_yaw < AHRS_YAW_P_MIN) {
        _kp_yaw = AHRS_YAW_P_MIN;
    }

    // update the proportional control to drag the
    // yaw back to the right value. We use a gain
    // that depends on the spin rate. See the fastRotations.pdf
    // paper from Bill Premerlani
    // We also adjust the gain depending on the rate of change of horizontal velocity which
    // is proportional to how observable the heading is from the acceerations and GPS velocity
    // The accelration derived heading will be more reliable in turns than compass or GPS

    _omega_yaw_P.z = error_z * _P_gain(spin_rate) * _kp_yaw * _yaw_gain();
    if (_flags.fast_ground_gains) {
        _omega_yaw_P.z *= 8;
    }

    // don't update the drift term if we lost the yaw reference
    // for more than 2 seconds
    if (yaw_deltat < 2.0f && spin_rate < ToRad(SPIN_RATE_LIMIT)) {
        // also add to the I term
        _omega_I_sum.z += error_z * _ki_yaw * yaw_deltat;
    }

    _error_yaw_sum += fabsf(yaw_error);
    _error_yaw_count++;    
}

/**
   return an accel vector delayed by AHRS_ACCEL_DELAY samples for a
   specific accelerometer instance
 */
Vector3f DCM::ra_delayed(uint8_t instance, const Vector3f &ra)
{
    // get the old element, and then fill it with the new element
    Vector3f ret = _ra_delay_buffer[instance];
    _ra_delay_buffer[instance] = ra;
    if (ret.is_zero()) {
        // use the current vector if the previous vector is exactly
        // zero. This prevents an error on initialisation
        return ra;
    }
    return ret;
}

// update our wind speed estimate
void DCM::estimate_wind(void)
{
    const Vector3f &velocity = _last_velocity;
    
    if (!_flags.wind_estimation) {
        return;
    }
    
    Vector3f fuselageDirection = _dcm_matrix.colx();
    Vector3f fuselageDirectionDiff = fuselageDirection - _last_fuse;
    uint32_t now = millis();
    
    // scrap our data and start over if we're taking too long to get a direction change
    if (now - _last_wind_time > 10000) {
        _last_wind_time = now;
        _last_fuse = fuselageDirection;
        _last_vel = velocity;
        return;
    }
    
    float diff_length = fuselageDirectionDiff.length();
    if (diff_length > 0.2f) {
        // when turning, use the attitude response to estimate
        // wind speed
        float V;
        Vector3f velocityDiff = velocity - _last_vel;

        // estimate airspeed it using equation 6
        V = velocityDiff.length() / diff_length;

        _last_fuse = fuselageDirection;
        _last_vel = velocity;

        Vector3f fuselageDirectionSum = fuselageDirection + _last_fuse;
        Vector3f velocitySum = velocity + _last_vel;

        float theta = atan2f(velocityDiff.y, velocityDiff.x) - atan2f(fuselageDirectionDiff.y, fuselageDirectionDiff.x);
        float sintheta = sinf(theta);
        float costheta = cosf(theta);

        Vector3f wind = Vector3f();
        wind.x = velocitySum.x - V * (costheta * fuselageDirectionSum.x - sintheta * fuselageDirectionSum.y);
        wind.y = velocitySum.y - V * (sintheta * fuselageDirectionSum.x + costheta * fuselageDirectionSum.y);
        wind.z = velocitySum.z - V * fuselageDirectionSum.z;
        wind *= 0.5f;

        if (wind.length() < _wind.length() + 20) {
            _wind = _wind * 0.95f + wind * 0.05f;
        }

        _last_wind_time = now;
    }
}


// perform drift correction. This function aims to update _omega_P and
// _omega_I with our best estimate of the short term and long term
// gyro error. The _omega_P value is what pulls our attitude solution
// back towards the reference vector quickly. The _omega_I term is an
// attempt to learn the long term drift rate of the gyros.
void DCM::drift_correction(float deltat)
{
    Vector3f velocity;
    uint32_t last_correction_time;
    float airspeed;
    AP_Float gain;
    gain.set(1.0f);

    // perform yaw drift correction if we have a new yaw reference
    // vector
    drift_correction_yaw();

    // rotate accelerometer values into the earth frame
    for (uint8_t i=0; i<_ins.get_accel_count(); i++) {
        if (_ins.get_accel_health(i)) {
            _accel_ef[i] = _dcm_matrix * _ins.get_accel(i);
            // integrate the accel vector in the earth frame between GPS readings
            _ra_sum[i] += _accel_ef[i] * deltat;
        }
    }

    // keep a sum of the deltat values, so we know how much time
    // we have integrated over
    _ra_deltat += deltat;
    
    if (_ra_deltat < 0.2f) {
      // not enough time has accumulated
      return;
    }
    airspeed = _last_airspeed;
    // use airspeed to estimate our ground velocity in
    // earth frame by subtracting the wind
    //velocity = _dcm_matrix.colx() * airspeed;
    velocity = _dcm_matrix.colx();

    estimate_wind();
    // add in wind estimate
    velocity += _wind;

    last_correction_time = millis();
    
    // see if this is our first time through - in which case we
    // just setup the start times and return
    if (_ra_sum_start == 0) {
        _ra_sum_start = last_correction_time;
        _last_velocity = velocity;
        return;
    }
    
    // equation 9: get the corrected acceleration vector in earth frame. Units
    // are m/s/s
    Vector3f GA_e;
    GA_e = Vector3f(0, 0, -1.0f);
    float ra_scale = 1.0f/(_ra_deltat*GRAVITY_MSS);
    float v_scale = 1.0f * ra_scale;
    Vector3f vdelta = (velocity - _last_velocity) * v_scale;
    GA_e += vdelta;
    GA_e.normalize();
    
    if (GA_e.is_inf()) {
        // wait for some non-zero acceleration information
        _last_failure_ms = millis();
        return;
    }
    
    Vector3f error[INS_MAX_INSTANCES];
    Vector3f GA_b[INS_MAX_INSTANCES];
    int8_t besti = -1;
    float best_error = 0;
    for (uint8_t i=0; i<_ins.get_accel_count(); i++) {
        if (!_ins.get_accel_health(i)) {
            // only use healthy sensors
            continue;
        }
        _ra_sum[i] *= ra_scale;
        
        GA_b[i] = _ra_sum[i];
        
        if (GA_b[i].is_zero()) {
            // wait for some non-zero acceleration information
            continue;
        }
        GA_b[i].normalize();
        if (GA_b[i].is_inf()) {
            // wait for some non-zero acceleration information
            continue;
        }
        error[i] = GA_b[i] % GA_e;
        float error_length = error[i].length();
        if (besti == -1 || error_length < best_error) {
            besti = i;
            best_error = error_length;
        }
    }

    if (besti == -1) {
        // no healthy accelerometers!
        _last_failure_ms = millis();
        return;
    }

    _active_accel_instance = besti;
    
    if (DCM::use_compass()) {
      error[besti].z = 0;
    }
    
    // if ins is unhealthy then stop attitude drift correction and
    // hope the gyros are OK for a while. Just slowly reduce _omega_P
    // to prevent previous bad accels from throwing us off
    if (!_ins.healthy()) {
        error[besti].zero();
    } else {
        // convert the error term to body frame
        error[besti] = _dcm_matrix.mul_transpose(error[besti]);
    }

    if (error[besti].is_nan() || error[besti].is_inf()) {
        // don't allow bad values
        check_matrix();
        _last_failure_ms = millis();
        return;
    }

    _error_rp_sum += best_error;
    _error_rp_count++;

    // base the P gain on the spin rate
    float spin_rate = _omega.length();

    // sanity check _kp value
    if (_kp < AHRS_RP_P_MIN) {
        _kp = AHRS_RP_P_MIN;
    }

    // we now want to calculate _omega_P and _omega_I. The
    // _omega_P value is what drags us quickly to the
    // accelerometer reading.
    _omega_P = error[besti] * _P_gain(spin_rate) * _kp;
   // if (_flags.fast_ground_gains) {
        //_omega_P *= 8;
    //}

    // accumulate some integrator error
    if (spin_rate < ToRad(SPIN_RATE_LIMIT)) {
        _omega_I_sum += error[besti] * _ki * _ra_deltat;
        _omega_I_sum_time += _ra_deltat;
    }
   
    if (_omega_I_sum_time >= 5) {
        
        // limit the rate of change of omega_I to the hardware
        // reported maximum gyro drift rate. This ensures that
        // short term errors don't cause a buildup of omega_I
        // beyond the physical limits of the device
        float change_limit = _gyro_drift_limit * _omega_I_sum_time;
        _omega_I_sum.x = constrain_float(_omega_I_sum.x, -change_limit, change_limit);
        _omega_I_sum.y = constrain_float(_omega_I_sum.y, -change_limit, change_limit);
        _omega_I_sum.z = constrain_float(_omega_I_sum.z, -change_limit, change_limit);
        _omega_I += _omega_I_sum;
        _omega_I_sum.zero();
        _omega_I_sum_time = 0;
    }

    // zero our accumulator ready for the next GPS step
    memset(&_ra_sum[0], 0, sizeof(_ra_sum));
    _ra_deltat = 0;
    _ra_sum_start = last_correction_time;

    // remember the velocity for next time
    _last_velocity = velocity;


}

// calculate the euler angles and DCM matrix which will be used for high level
// navigation control. Apply trim such that a positive trim value results in a 
// positive vehicle rotation about that axis (ie a negative offset)
void DCM::euler_angles(void)
{
    _body_dcm_matrix = _dcm_matrix;
    _body_dcm_matrix.rotateXYinv(_trim);
    _body_dcm_matrix.to_euler(&roll, &pitch, &yaw);

    update_cd_values();
}

// average error_roll_pitch since last call
float DCM::get_error_rp(void)
{
    if (_error_rp_count == 0) {
        // this happens when telemetry is setup on two
        // serial ports
        return _error_rp_last;
    }
    _error_rp_last = _error_rp_sum / _error_rp_count;
    _error_rp_sum = 0;
    _error_rp_count = 0;
    return _error_rp_last;
}

// average error_yaw since last call
float DCM::get_error_yaw(void)
{
    if (_error_yaw_count == 0) {
        // this happens when telemetry is setup on two
        // serial ports
        return _error_yaw_last;
    }
    _error_yaw_last = _error_yaw_sum / _error_yaw_count;
    _error_yaw_sum = 0;
    _error_yaw_count = 0;
    return _error_yaw_last;
}

/*
  check if the AHRS subsystem is healthy
*/
bool DCM::healthy(void)
{
    // consider ourselves healthy if there have been no failures for 5 seconds
    return (_last_failure_ms == 0 || millis() - _last_failure_ms > 5000);
}
