#include "Compass.h"

Compass::Compass(void) :
    product_id(COMPASS_TYPE_UNKNOWN),
    last_update(0),
    _null_init_done(false),
    _thr_or_curr(0.0f),
    _board_orientation(ROTATION_NONE)
{}

// Default init method, just returns success.
//
bool Compass::init()
{
    return true;
}

void Compass::set_offsets(uint8_t i, const Vector3f &offsets)
{
    // sanity check compass instance provided
    if (i < COMPASS_MAX_INSTANCES) {
        _offset[i].set(offsets);
    }
}

void Compass::set_and_save_offsets(uint8_t i, const Vector3f &offsets)
{
    // sanity check compass instance provided
    if (i < COMPASS_MAX_INSTANCES) {
        _offset[i].set(offsets);
        //save_offsets(i);
    }
}

void Compass::save_offsets(uint8_t i)
{
    //_offset[i].save();  // save offsets
}

void Compass::save_offsets(void)
{
    for (uint8_t i=0; i<COMPASS_MAX_INSTANCES; i++) {
        //save_offsets(i);
    }
}

void Compass::set_motor_compensation(uint8_t i, const Vector3f &motor_comp_factor)
{
    _motor_compensation[i].set(motor_comp_factor);
}

void Compass::save_motor_compensation()
{
    //_motor_comp_type.save();
    for (uint8_t k=0; k<COMPASS_MAX_INSTANCES; k++) {
        //_motor_compensation[k].save();
    }
}

void Compass::set_declination(float radians, bool save_to_eeprom)
{
    if (save_to_eeprom) {
        //_declination.set_and_save(radians);
    }else{
        _declination.set(radians);
    }
}

float Compass::get_declination() const
{
    return _declination.get();
}

/*
  calculate a compass heading given the attitude from DCM and the mag vector
 */
float Compass::calculate_heading(const Matrix3f &dcm_matrix) const
{
    float cos_pitch_sq = 1.0f-(dcm_matrix.c.x*dcm_matrix.c.x);

    // Tilt compensated magnetic field Y component:
    float headY = _field[0].y * dcm_matrix.c.z - _field[0].z * dcm_matrix.c.y;

    // Tilt compensated magnetic field X component:
    float headX = _field[0].x * cos_pitch_sq - dcm_matrix.c.x * (_field[0].y * dcm_matrix.c.y + _field[0].z * dcm_matrix.c.z);

    // magnetic heading
    // 6/4/11 - added constrain to keep bad values from ruining DCM Yaw - Jason S.
    float heading = constrain_float(atan2f(-headY,headX), -3.15f, 3.15f);

    // Declination correction (if supplied)
    if( fabsf(_declination) > 0.0f )
    {
        heading = heading + _declination;
        if (heading > PI)    // Angle normalization (-180 deg, 180 deg)
            heading -= (2.0f * PI);
        else if (heading < -PI)
            heading += (2.0f * PI);
    }

    return heading;
}

/// Returns True if the compasses have been configured (i.e. offsets saved)
///
/// @returns                    True if compass has been configured
///
bool Compass::configured(uint8_t i)
{
    // exit immediately if instance is beyond the number of compasses we have available
    if (i > get_count()) {
        return false;
    }

    // exit immediately if all offsets are zero
    if (get_offsets(i).length() == 0.0f) {
        return false;
    }
    // if we got here then it must be configured
    return true;
}

bool Compass::configured(void)
{
    bool all_configured = true;
    for(uint8_t i=0; i<get_count(); i++) {
        all_configured = all_configured && configured(i);
    }
    return all_configured;
}

void Compass::learn_offsets(void)
{
    if (_learn == 0) {
        // auto-calibration is disabled
        return;
    }
    // this gain is set so we converge on the offsets in about 5
    // minutes with a 10Hz compass
    const float gain = 0.01;
    const float max_change = 10.0;
    const float min_diff = 50.0;

    if (!_null_init_done) {
        // first time through
        _null_init_done = true;
        for (uint8_t k=0; k<COMPASS_MAX_INSTANCES; k++) {
            const Vector3f &ofs = _offset[k].get();
            for (uint8_t i=0; i<_mag_history_size; i++) {
                // fill the history buffer with the current mag vector,
                // with the offset removed
                _mag_history[k][i] = Vector3i((_field[k].x+0.5f) - ofs.x, (_field[k].y+0.5f) - ofs.y, (_field[k].z+0.5f) - ofs.z);
            }
            _mag_history_index[k] = 0;
        }
        return;
    }

    for (uint8_t k=0; k<COMPASS_MAX_INSTANCES; k++) {
        const Vector3f &ofs = _offset[k].get();
        Vector3f b1, diff;
        float length;

        if (ofs.is_nan()) {
            // offsets are bad possibly due to a past bug - zero them
            _offset[k].set(Vector3f());
        }

        // get a past element
        b1 = Vector3f(_mag_history[k][_mag_history_index[k]].x,
                      _mag_history[k][_mag_history_index[k]].y,
                      _mag_history[k][_mag_history_index[k]].z);

        // the history buffer doesn't have the offsets
        b1 += ofs;

        // get the current vector
        const Vector3f &b2 = _field[k];

        // calculate the delta for this sample
        diff = b2 - b1;
        length = diff.length();
        if (length < min_diff) {
            // the mag vector hasn't changed enough - we don't get
            // enough information from this vector to use it.
            // Note that we don't put the current vector into the mag
            // history here. We want to wait for a larger rotation to
            // build up before calculating an offset change, as accuracy
            // of the offset change is highly dependent on the size of the
            // rotation.
            _mag_history_index[k] = (_mag_history_index[k] + 1) % _mag_history_size;
            continue;
        }

        // put the vector in the history
        _mag_history[k][_mag_history_index[k]] = Vector3i((_field[k].x+0.5f) - ofs.x, 
                                                          (_field[k].y+0.5f) - ofs.y, 
                                                          (_field[k].z+0.5f) - ofs.z);
        _mag_history_index[k] = (_mag_history_index[k] + 1) % _mag_history_size;

        // equation 6 of Bills paper
        diff = diff * (gain * (b2.length() - b1.length()) / length);

        // limit the change from any one reading. This is to prevent
        // single crazy readings from throwing off the offsets for a long
        // time
        length = diff.length();
        if (length > max_change) {
            diff *= max_change / length;
        }

        Vector3f new_offsets = _offset[k].get() - diff;

        if (new_offsets.is_nan()) {
            // don't apply bad offsets
            continue;
        }

        // constrain offsets
        new_offsets.x = constrain_float(new_offsets.x, -COMPASS_OFS_LIMIT, COMPASS_OFS_LIMIT);
        new_offsets.y = constrain_float(new_offsets.y, -COMPASS_OFS_LIMIT, COMPASS_OFS_LIMIT);
        new_offsets.z = constrain_float(new_offsets.z, -COMPASS_OFS_LIMIT, COMPASS_OFS_LIMIT);
            
        // set the new offsets
        _offset[k].set(new_offsets);
    }
}
