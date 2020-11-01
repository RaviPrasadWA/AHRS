#include "InertialSensor.h"

InertialSensor::InertialSensor() :
    _gyro_count(0),
    _accel_count(0),
    _backend_count(0),
    _accel(),
    _gyro(),
    _board_orientation(ROTATION_NONE), 
    _hil_mode(false),
    _have_3D_calibration(false)
{      
    for (uint8_t i=0; i<INS_MAX_BACKENDS; i++) {
        _backends[i] = NULL;
    }
    for (uint8_t i=0; i<INS_MAX_INSTANCES; i++) {
        _accel_error_count[i] = 0;
        _gyro_error_count[i] = 0;
    }

}

/*
  register a new gyro instance
 */
uint8_t InertialSensor::register_gyro(void)
{
    if (_gyro_count == INS_MAX_INSTANCES) {
        Serial.println(F("[x] INS[warning]: Too many gyros"));
    }
    return _gyro_count++;
}

/*
  register a new accel instance
 */
uint8_t InertialSensor::register_accel(void)
{
    if (_accel_count == INS_MAX_INSTANCES) {
        Serial.println(F("[x] INS[warning]: Too many accelerometer"));
    }
    return _accel_count++;
}

void
InertialSensor::init( Start_style style,
                      Sample_rate sample_rate)
{
    // remember the sample rate
    _sample_rate = sample_rate;

    if (_gyro_count == 0 && _accel_count == 0) {
        // detect available backends. Only called once
        _detect_backends();
    }

    _product_id = 0; // FIX

    // initialise accel scale if need be. This is needed as we can't
    // give non-zero default values for vectors in AP_Param
    for (uint8_t i=0; i<get_accel_count(); i++) {
        if (_accel_scale[i].get().is_zero()) {
            _accel_scale[i].set(Vector3f(1,1,1));
        }
    }

    // remember whether we have 3D calibration so this can be used for
    // AHRS health
    check_3D_calibration();

    if (WARM_START != style) {
        // do cold-start calibration for gyro only
        _init_gyro();
    }

    switch (sample_rate) {
    case RATE_50HZ:
        _sample_period_usec = 20000;
        break;
    case RATE_100HZ:
        _sample_period_usec = 10000;
        break;
    case RATE_200HZ:
        _sample_period_usec = 5000;
        break;
    case RATE_400HZ:
    default:
        _sample_period_usec = 2500;
        break;
    }

    // establish the baseline time between samples
    _delta_time = 0;
    _next_sample_usec = 0;
    _last_sample_usec = 0;
    _have_sample = false;
}

/*
  try to load a backend
 */
void InertialSensor::_add_backend(InertialSensor_Backend *(detect)(InertialSensor &))
{
    if (_backend_count == INS_MAX_BACKENDS) {
        Serial.println(F("[x] INS[warning]: Too many INS backends"));
    }
    _backends[_backend_count] = detect(*this);
    if (_backends[_backend_count] != NULL) {
        _backend_count++;
    }
}

/*
  detect available backends for this board
 */
void 
InertialSensor::_detect_backends(void)
{
  _add_backend(InertialSensor_MPU9250::detect);
  if (_backend_count == 0 ||
        _gyro_count == 0 ||
        _accel_count == 0) {
          Serial.println(F("[x] INS[warning]: No INS backends"));
      }
    // set the product ID to the ID of the first backend
   _product_id.set(_backends[0]->product_id());
}

void
InertialSensor::init_accel()
{
    _init_accel();

    // save calibration
    //_save_parameters();

    check_3D_calibration();
}

bool InertialSensor::calibrate_accel(float &trim_roll,float &trim_pitch)
{
    uint8_t num_accels = min(get_accel_count(), INS_MAX_INSTANCES);
    Vector3f samples[INS_MAX_INSTANCES][6];
    Vector3f new_offsets[INS_MAX_INSTANCES];
    Vector3f new_scaling[INS_MAX_INSTANCES];
    Vector3f orig_offset[INS_MAX_INSTANCES];
    Vector3f orig_scale[INS_MAX_INSTANCES];
    uint8_t num_ok = 0;

    for (uint8_t k=0; k<num_accels; k++) {
        // backup original offsets and scaling
        orig_offset[k] = _accel_offset[k].get();
        orig_scale[k]  = _accel_scale[k].get();

        // clear accelerometer offsets and scaling
        _accel_offset[k] = Vector3f(0,0,0);
        _accel_scale[k] = Vector3f(1,1,1);
        
    }
    Serial.println(F("[o] MPU9250[info]: Starting Calibration"));
    // capture data from 6 positions
    for (uint8_t i=0; i<6; i++) {

        // display message to user
        switch ( i ) {
            case 0:
                Serial.println(F("[o] MPU9250[info]: Place vehicle on its Level"));
                break;
            case 1:
                Serial.println(F("[o] MPU9250[info]: Place vehicle on its LEFT side"));
                break;
            case 2:
                Serial.println(F("[o] MPU9250[info]: Place vehicle on its RIGHT side"));
                break;
            case 3:
                Serial.println(F("[o] MPU9250[info]: Place vehicle nose DOWN"));
                break;
            case 4:
                Serial.println(F("[o] MPU9250[info]: Place vehicle nose UP"));
                break;
            default:    // default added to avoid compiler warning
            case 5:
                Serial.println(F("[o] MPU9250[info]: Place vehicle on its BACK"));
                break;
        }
  
        delay(4000);

        // clear out any existing samples from ins
        update();

        // average 32 samples
        for (uint8_t k=0; k<num_accels; k++) {
            samples[k][i] = Vector3f();
        }
        uint8_t num_samples = 0;
        while (num_samples < 32) {
            wait_for_sample();
            // read samples from ins
            update();
            // capture sample
            for (uint8_t k=0; k<num_accels; k++) {
                samples[k][i] += get_accel(k);
            }
            delay(10);
            num_samples++;
        }
        for (uint8_t k=0; k<num_accels; k++) {
            samples[k][i] /= num_samples;
        }
    }

    // run the calibration routine
    for (uint8_t k=0; k<num_accels; k++) {
        bool success = _calibrate_accel(samples[k], new_offsets[k], new_scaling[k]);
        
        Serial.print(F("[o] MPU9250[info]: Offsets["));
        Serial.print( (unsigned)k );
        Serial.print(F("]: "));
        Serial.print( new_offsets[k].x ); 
        Serial.print(F(" "));
        Serial.print( new_offsets[k].y );
        Serial.print(F(" "));
        Serial.print( new_offsets[k].z );
        Serial.println();
        
        Serial.print(F("[o] MPU9250[info]: Scaling["));
        Serial.print( (unsigned)k );
        Serial.print(F("]: "));
        Serial.print( new_scaling[k].x ); 
        Serial.print(F(" "));
        Serial.print( new_scaling[k].y );
        Serial.print(F(" "));
        Serial.print( new_scaling[k].z );
        Serial.println();

        if (success) num_ok++;
    }

    if (num_ok == num_accels) {
        Serial.println(F("[o] MPU9250[success]:Calibration successful"));

        for (uint8_t k=0; k<num_accels; k++) {
            // set and save calibration
            _accel_offset[k].set(new_offsets[k]);
            _accel_scale[k].set(new_scaling[k]);
        }
        //_save_parameters();

        check_3D_calibration();

        // calculate the trims as well from primary accels and pass back to caller
        _calculate_trim(samples[0][0], trim_roll, trim_pitch);

        return true;
    }

    Serial.println(F("[x] MPU9250[error]:Calibration FAILED"));
    // restore original scaling and offsets
    for (uint8_t k=0; k<num_accels; k++) {
        _accel_offset[k].set(orig_offset[k]);
        _accel_scale[k].set(orig_scale[k]);
    }
    return false;
}


/*
  check if the accelerometers are calibrated in 3D. Called on startup
  and any accel cal
 */
void InertialSensor::check_3D_calibration()
{
    _have_3D_calibration = false;
    // check each accelerometer has offsets saved
    for (uint8_t i=0; i<get_accel_count(); i++) {
        // exactly 0.0 offset is extremely unlikely
        if (_accel_offset[i].get().is_zero()) {
            return;
        }
        // exactly 1.0 scaling is extremely unlikely
        const Vector3f &scaling = _accel_scale[i].get();
        if (fabsf(scaling.x - 1.0f) < 0.00001f &&
            fabsf(scaling.y - 1.0f) < 0.00001f &&
            fabsf(scaling.z - 1.0f) < 0.00001f) {
            return;
        }
    }
    // if we got this far the accelerometers must have been calibrated
    _have_3D_calibration = true;
}

/*
  return true if we have 3D calibration values
 */
bool InertialSensor::calibrated()
{
    return _have_3D_calibration;
}

void InertialSensor::init_gyro()
{
    _init_gyro();

    // save calibration
    //_save_parameters();
}

// get_gyro_health_all - return true if all gyros are healthy
bool InertialSensor::get_gyro_health_all(void) const
{
    for (uint8_t i=0; i<get_gyro_count(); i++) {
        if (!get_gyro_health(i)) {
            return false;
        }
    }
    // return true if we have at least one gyro
    return (get_gyro_count() > 0);
}

// gyro_calibration_ok_all - returns true if all gyros were calibrated successfully
bool InertialSensor::gyro_calibrated_ok_all() const
{
    for (uint8_t i=0; i<get_gyro_count(); i++) {
        if (!gyro_calibrated_ok(i)) {
            return false;
        }
    }
    return (get_gyro_count() > 0);
}

// get_accel_health_all - return true if all accels are healthy
bool InertialSensor::get_accel_health_all(void) const
{
    for (uint8_t i=0; i<get_accel_count(); i++) {
        if (!get_accel_health(i)) {
            return false;
        }
    }
    // return true if we have at least one accel
    return (get_accel_count() > 0);
}

void InertialSensor::_init_accel()
{
    uint8_t num_accels = min(get_accel_count(), INS_MAX_INSTANCES);
    uint8_t flashcount = 0;
    Vector3f prev[INS_MAX_INSTANCES];
    Vector3f accel_offset[INS_MAX_INSTANCES];
    float total_change[INS_MAX_INSTANCES];
    float max_offset[INS_MAX_INSTANCES];

    memset(max_offset, 0, sizeof(max_offset));
    memset(total_change, 0, sizeof(total_change));

    // cold start
    delay(100);

    Serial.print(F("[o] MPU9250[info]: Initialize Accelerometer"));

    // clear accelerometer offsets and scaling
    for (uint8_t k=0; k<num_accels; k++) {
        _accel_offset[k] = Vector3f(0,0,0);
        _accel_scale[k] = Vector3f(1,1,1);

        // initialise accel offsets to a large value the first time
        // this will force us to calibrate accels at least twice
        accel_offset[k] = Vector3f(500, 500, 500);
    }

    // loop until we calculate acceptable offsets
    while (true) {
        // get latest accelerometer values
        update();

        for (uint8_t k=0; k<num_accels; k++) {
            // store old offsets
            prev[k] = accel_offset[k];

            // get new offsets
            accel_offset[k] = get_accel(k);
        }

        // We take some readings...
        for(int8_t i = 0; i < 50; i++) {

            delay(20);
            update();

            // low pass filter the offsets
            for (uint8_t k=0; k<num_accels; k++) {
                accel_offset[k] = accel_offset[k] * 0.9f + get_accel(k) * 0.1f;
            }

            // display some output to the user
            if(flashcount >= 10) {
                Serial.print("*");
                flashcount = 0;
            }
            flashcount++;
        }

        for (uint8_t k=0; k<num_accels; k++) {
            // null gravity from the Z accel
            accel_offset[k].z += GRAVITY_MSS;

            total_change[k] = 
                fabsf(prev[k].x - accel_offset[k].x) + 
                fabsf(prev[k].y - accel_offset[k].y) + 
                fabsf(prev[k].z - accel_offset[k].z);
            max_offset[k] = (accel_offset[k].x > accel_offset[k].y) ? accel_offset[k].x : accel_offset[k].y;
            max_offset[k] = (max_offset[k] > accel_offset[k].z) ? max_offset[k] : accel_offset[k].z;
        }

        uint8_t num_converged = 0;
        for (uint8_t k=0; k<num_accels; k++) {
            if (total_change[k] <= INERTIAL_SENSOR_ACCEL_TOT_MAX_OFFSET_CHANGE && 
                max_offset[k] <= INERTIAL_SENSOR_ACCEL_MAX_OFFSET) {
                num_converged++;
            }
        }

        if (num_converged == num_accels) break;

        delay(500);
    }

    // set the global accel offsets
    for (uint8_t k=0; k<num_accels; k++) {
        _accel_offset[k] = accel_offset[k];
    }

    Serial.print(F("[successfull]"));
    Serial.println();

}

void InertialSensor::_init_gyro()
{
    uint8_t num_gyros = min(get_gyro_count(), INS_MAX_INSTANCES);
    Vector3f last_average[INS_MAX_INSTANCES], best_avg[INS_MAX_INSTANCES];
    float best_diff[INS_MAX_INSTANCES];
    bool converged[INS_MAX_INSTANCES];

    // cold start
    Serial.print(F("[o] MPU9250[info]: Init Gyroscope"));

    // remove existing gyro offsets
    for (uint8_t k=0; k<num_gyros; k++) {
        _gyro_offset[k] = Vector3f(0,0,0);
        best_diff[k] = 0;
        last_average[k].zero();
        converged[k] = false;
        _gyro_cal_ok[k] = true; // default calibration ok flag to true
    }

    for(int8_t c = 0; c < 5; c++) {
        delay(5);
        update();
    }

    // the strategy is to average 50 points over 0.5 seconds, then do it
    // again and see if the 2nd average is within a small margin of
    // the first

    uint8_t num_converged = 0;
   
    // we try to get a good calibration estimate for up to 30 seconds
    // if the gyros are stable, we should get it in 1 second
    for (int16_t j = 0; j <= 30*4 && num_converged < num_gyros; j++) {
        Vector3f gyro_sum[INS_MAX_INSTANCES], gyro_avg[INS_MAX_INSTANCES], gyro_diff[INS_MAX_INSTANCES];
        float diff_norm[INS_MAX_INSTANCES];
        uint8_t i;

        memset(diff_norm, 0, sizeof(diff_norm));

        Serial.print(F("*"));

        for (uint8_t k=0; k<num_gyros; k++) {
            gyro_sum[k].zero();
        }
        for (i=0; i<50; i++) {
            update();
            for (uint8_t k=0; k<num_gyros; k++) {
                gyro_sum[k] += get_gyro(k);
            }
            delay(5);
        }
        for (uint8_t k=0; k<num_gyros; k++) {
            gyro_avg[k] = gyro_sum[k] / i;
            gyro_diff[k] = last_average[k] - gyro_avg[k];
            diff_norm[k] = gyro_diff[k].length();
        }

        for (uint8_t k=0; k<num_gyros; k++) {
            if (converged[k]) continue;
            if (j == 0) {
                best_diff[k] = diff_norm[k];
                best_avg[k] = gyro_avg[k];
            } else if (gyro_diff[k].length() < ToRad(0.1f)) {
                // we want the average to be within 0.1 bit, which is 0.04 degrees/s
                last_average[k] = (gyro_avg[k] * 0.5f) + (last_average[k] * 0.5f);
                _gyro_offset[k] = last_average[k];
                converged[k] = true;
                num_converged++;
            } else if (diff_norm[k] < best_diff[k]) {
                best_diff[k] = diff_norm[k];
                best_avg[k] = (gyro_avg[k] * 0.5f) + (last_average[k] * 0.5f);
            }
            last_average[k] = gyro_avg[k];
        }
    }

    if (num_converged == num_gyros) {
        // all OK
        Serial.print(F("[succesfull]\n"));
        return;
    }

    // we've kept the user waiting long enough - use the best pair we
    // found so far
    for (uint8_t k=0; k<num_gyros; k++) {
        if (!converged[k]) {
            Serial.print(F("[x] MPU9250[error]:"));
            Serial.print( (unsigned)k );
            Serial.print(F(" did not converge: diff="));
            Serial.print( ToDeg(best_diff[k]) );
            Serial.print(F(" dps\n"));
            _gyro_offset[k] = best_avg[k];
            // flag calibration as failed for this gyro
            _gyro_cal_ok[k] = false;
        }
    }
}

bool InertialSensor::_calibrate_accel( Vector3f accel_sample[6], Vector3f& accel_offsets, Vector3f& accel_scale )
{
    int16_t i;  
    int16_t num_iterations = 0;
    float eps = 0.000000001;
    float change = 100.0;
    float data[3];
    float beta[6];
    float delta[6];
    float ds[6];
    float JS[6][6];
    bool success = true;
    
    // reset
    beta[0] = beta[1] = beta[2] = 0;
    beta[3] = beta[4] = beta[5] = 1.0f/GRAVITY_MSS;
    
    while( num_iterations < 20 && change > eps ) {
        num_iterations++;

        _calibrate_reset_matrices(ds, JS);

        for( i=0; i<6; i++ ) {
            data[0] = accel_sample[i].x;
            data[1] = accel_sample[i].y;
            data[2] = accel_sample[i].z;   
            _calibrate_update_matrices(ds, JS, beta, data);
        }
        
        _calibrate_find_delta(ds, JS, delta);

        change =    delta[0]*delta[0] +
                    delta[0]*delta[0] +
                    delta[1]*delta[1] +
                    delta[2]*delta[2] +
                    delta[3]*delta[3] / (beta[3]*beta[3]) +
                    delta[4]*delta[4] / (beta[4]*beta[4]) +
                    delta[5]*delta[5] / (beta[5]*beta[5]);
        

        for( i=0; i<6; i++ ) {
            beta[i] -= delta[i];
        }
    }

    // copy results out
    accel_scale.x = beta[3] * GRAVITY_MSS;
    accel_scale.y = beta[4] * GRAVITY_MSS;
    accel_scale.z = beta[5] * GRAVITY_MSS;
    accel_offsets.x = beta[0] * accel_scale.x;
    accel_offsets.y = beta[1] * accel_scale.y;
    accel_offsets.z = beta[2] * accel_scale.z;

    // sanity check scale
    if( accel_scale.is_nan() || fabsf(accel_scale.x-1.0f) > 0.1f || fabsf(accel_scale.y-1.0f) > 0.1f || fabsf(accel_scale.z-1.0f) > 0.1f ) {
        success = false;
    }
    // sanity check offsets (3.5 is roughly 3/10th of a G, 5.0 is roughly half a G)
    if( accel_offsets.is_nan() || fabsf(accel_offsets.x) > 3.5f || fabsf(accel_offsets.y) > 3.5f || fabsf(accel_offsets.z) > 3.5f ) {
        success = false;
    }

    // return success or failure
    return success;
}

void InertialSensor::_calibrate_update_matrices(float dS[6], float JS[6][6],
                                                float beta[6], float data[3])
{
    int16_t j, k;
    float dx, b;
    float residual = 1.0;
    float jacobian[6];
    
    for( j=0; j<3; j++ ) {
        b = beta[3+j];
        dx = (float)data[j] - beta[j];
        residual -= b*b*dx*dx;
        jacobian[j] = 2.0f*b*b*dx;
        jacobian[3+j] = -2.0f*b*dx*dx;
    }
    
    for( j=0; j<6; j++ ) {
        dS[j] += jacobian[j]*residual;
        for( k=0; k<6; k++ ) {
            JS[j][k] += jacobian[j]*jacobian[k];
        }
    }
}


// _calibrate_reset_matrices - clears matrices
void InertialSensor::_calibrate_reset_matrices(float dS[6], float JS[6][6])
{
    int16_t j,k;
    for( j=0; j<6; j++ ) {
        dS[j] = 0.0f;
        for( k=0; k<6; k++ ) {
            JS[j][k] = 0.0f;
        }
    }
}


void InertialSensor::_calibrate_find_delta(float dS[6], float JS[6][6], float delta[6])
{
    //Solve 6-d matrix equation JS*x = dS
    //first put in upper triangular form
    int16_t i,j,k;
    float mu;

    //make upper triangular
    for( i=0; i<6; i++ ) {
        //eliminate all nonzero entries below JS[i][i]
        for( j=i+1; j<6; j++ ) {
            mu = JS[i][j]/JS[i][i];
            if( mu != 0.0f ) {
                dS[j] -= mu*dS[i];
                for( k=j; k<6; k++ ) {
                    JS[k][j] -= mu*JS[k][i];
                }
            }
        }
    }

    //back-substitute
    for( i=5; i>=0; i-- ) {
        dS[i] /= JS[i][i];
        JS[i][i] = 1.0f;
        
        for( j=0; j<i; j++ ) {
            mu = JS[i][j];
            dS[j] -= mu*dS[i];
            JS[i][j] = 0.0f;
        }
    }

    for( i=0; i<6; i++ ) {
        delta[i] = dS[i];
    }
}


void InertialSensor::_calculate_trim(Vector3f accel_sample, float& trim_roll, float& trim_pitch)
{
    // scale sample and apply offsets
    Vector3f accel_scale = _accel_scale[0].get();
    Vector3f accel_offsets = _accel_offset[0].get();
    Vector3f scaled_accels_x( accel_sample.x * accel_scale.x - accel_offsets.x,
                              0,
                              accel_sample.z * accel_scale.z - accel_offsets.z );
    Vector3f scaled_accels_y( 0,
                              accel_sample.y * accel_scale.y - accel_offsets.y,
                              accel_sample.z * accel_scale.z - accel_offsets.z );

    // calculate x and y axis angle (i.e. roll and pitch angles)
    Vector3f vertical = Vector3f(0,0,-1);
    trim_roll = scaled_accels_y.angle(vertical);
    trim_pitch = scaled_accels_x.angle(vertical);

    // angle call doesn't return the sign so take care of it here
    if( scaled_accels_y.y > 0 ) {
        trim_roll = -trim_roll;
    }
    if( scaled_accels_x.x < 0 ) {
        trim_pitch = -trim_pitch;
    }
}

/*
void InertialSensor::_save_parameters()
{
    _product_id.save();
    for (uint8_t i=0; i<INS_MAX_INSTANCES; i++) {
        _accel_scale[i].save();
        _accel_offset[i].save();
        _gyro_offset[i].save();
    }
}
*/

void InertialSensor::update(void)
{
    // during initialisation update() may be called without
    // wait_for_sample(), and a wait is implied
    wait_for_sample();

    if (!_hil_mode) {
        for (uint8_t i=0; i<INS_MAX_INSTANCES; i++) {
            // mark sensors unhealthy and let update() in each backend
            // mark them healthy via _rotate_and_offset_gyro() and
            // _rotate_and_offset_accel() 
            _gyro_healthy[i] = false;
            _accel_healthy[i] = false;
        }
        for (uint8_t i=0; i<_backend_count; i++) {
            _backends[i]->update();
        }

        // adjust health status if a sensor has a non-zero error count
        // but another sensor doesn't. 
        bool have_zero_accel_error_count = false;
        bool have_zero_gyro_error_count = false;
        for (uint8_t i=0; i<INS_MAX_INSTANCES; i++) {
            if (_accel_healthy[i] && _accel_error_count[i] == 0) {
                have_zero_accel_error_count = true;
            }
            if (_gyro_healthy[i] && _gyro_error_count[i] == 0) {
                have_zero_gyro_error_count = true;
            }
        }

        for (uint8_t i=0; i<INS_MAX_INSTANCES; i++) {
            if (_gyro_healthy[i] && _gyro_error_count[i] != 0 && have_zero_gyro_error_count) {
                // we prefer not to use a gyro that has had errors
                _gyro_healthy[i] = false;
            }
            if (_accel_healthy[i] && _accel_error_count[i] != 0 && have_zero_accel_error_count) {
                // we prefer not to use a accel that has had errors
                _accel_healthy[i] = false;
            }
        }

        // set primary to first healthy accel and gyro
        for (uint8_t i=0; i<INS_MAX_INSTANCES; i++) {
            if (_gyro_healthy[i]) {
                _primary_gyro = i;
                break;
            }
        }
        for (uint8_t i=0; i<INS_MAX_INSTANCES; i++) {
            if (_accel_healthy[i]) {
                _primary_accel = i;
                break;
            }
        }
    }

    _have_sample = false;
}


void InertialSensor::wait_for_sample(void)
{
    if (_have_sample) {
        // the user has called wait_for_sample() again without
        // consuming the sample with update()
        return;
    }

    uint32_t now = micros();

    if (_next_sample_usec == 0 && _delta_time <= 0) {
        // this is the first call to wait_for_sample()
        _last_sample_usec = now - _sample_period_usec;
        _next_sample_usec = now + _sample_period_usec;
        goto check_sample;
    }

    // see how long it is till the next sample is due
    if (_next_sample_usec - now <=_sample_period_usec) {
        // we're ahead on time, schedule next sample at expected period
        uint32_t wait_usec = _next_sample_usec - now;
        if (wait_usec > 200) {
            delayMicroseconds(wait_usec);
        }
        _next_sample_usec += _sample_period_usec;
    } else if (now - _next_sample_usec < _sample_period_usec/8) {
        // we've overshot, but only by a small amount, keep on
        // schedule with no delay
        _next_sample_usec += _sample_period_usec;
    } else {
        // we've overshot by a larger amount, re-zero scheduling with
        // no delay
        _next_sample_usec = now + _sample_period_usec;
    }

check_sample:
    if (!_hil_mode) {
        // we also wait for at least one backend to have a sample of both
        // accel and gyro. This normally completes immediately.
        bool gyro_available = false;
        bool accel_available = false;
       
        while (!gyro_available || !accel_available) {
            for (uint8_t i=0; i<_backend_count; i++) {
                gyro_available |= _backends[i]->gyro_sample_available();
                accel_available |= _backends[i]->accel_sample_available();
            }
            if (!gyro_available || !accel_available) {
                delayMicroseconds(100);
            }
        }
    }

    now = micros();
    _delta_time = (now - _last_sample_usec) * 1.0e-6f;
    _last_sample_usec = now;
    _have_sample = true;
}

/*
  support for setting accel and gyro vectors, for use by HIL
 */
void InertialSensor::set_accel(uint8_t instance, const Vector3f &accel)
{
    if (instance < INS_MAX_INSTANCES) {
        _accel[instance] = accel;
        _accel_healthy[instance] = true;
    }
}

void InertialSensor::set_gyro(uint8_t instance, const Vector3f &gyro)
{
    if (instance < INS_MAX_INSTANCES) {
        _gyro[instance] = gyro;
        _gyro_healthy[instance] = true;
    }
}


InertialSensor_Backend::InertialSensor_Backend(InertialSensor &imu) :
    _imu(imu),
    _product_id(AP_PRODUCT_ID_NONE)
{}

/*
  rotate gyro vector and add the gyro offset
 */
void InertialSensor_Backend::_rotate_and_offset_gyro(uint8_t instance, const Vector3f &gyro)
{
    _imu._gyro[instance] = gyro;
    _imu._gyro[instance].rotate(_imu._board_orientation);
    _imu._gyro[instance] -= _imu._gyro_offset[instance];
    _imu._gyro_healthy[instance] = true;
}

/*
  rotate accel vector, scale and add the accel offset
 */
void InertialSensor_Backend::_rotate_and_offset_accel(uint8_t instance, const Vector3f &accel)
{
    _imu._accel[instance] = accel;
    _imu._accel[instance].rotate(_imu._board_orientation);

    const Vector3f &accel_scale = _imu._accel_scale[instance].get();
    _imu._accel[instance].x *= accel_scale.x;
    _imu._accel[instance].y *= accel_scale.y;
    _imu._accel[instance].z *= accel_scale.z;
    _imu._accel[instance] -= _imu._accel_offset[instance];
    _imu._accel_healthy[instance] = true;
}

// set accelerometer error_count
void InertialSensor_Backend::_set_accel_error_count(uint8_t instance, uint32_t error_count)
{
    _imu._accel_error_count[instance] = error_count;
}

// set gyro error_count
void InertialSensor_Backend::_set_gyro_error_count(uint8_t instance, uint32_t error_count)
{
    _imu._gyro_error_count[instance] = error_count;
}

/*
  return the default filter frequency in Hz for the sample rate
  
  This uses the sample_rate as a proxy for what type of vehicle it is
  (ie. plane and rover run at 50Hz). Copters need a bit more filter
  bandwidth
 */
uint8_t InertialSensor_Backend::_default_filter(void) const
{
    switch (_imu.get_sample_rate()) {
    case InertialSensor::RATE_50HZ:
        // on Rover and plane use a lower filter rate
        return 15;
    case InertialSensor::RATE_100HZ:
        return 30;
    case InertialSensor::RATE_200HZ:
        return 30;
    case InertialSensor::RATE_400HZ:
        return 30;
    }
    return 30;
}




