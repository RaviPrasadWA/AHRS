#ifndef __COMPASS_H__
#define __COMPASS_H__

#include "AP_Param.h"
#include "Math.h"



#define COMPASS_TYPE_UNKNOWN  0x00
#define COMPASS_TYPE_HMC5883L 0x03
#define COMPASS_OFS_LIMIT 2000

// motor compensation types (for use with motor_comp_enabled)
#define COMPASS_MOT_COMP_DISABLED    0x00
#define COMPASS_MOT_COMP_THROTTLE    0x01
#define COMPASS_MOT_COMP_CURRENT     0x02

#define MAG_BOARD_ORIENTATION ROTATION_NONE
#define COMPASS_MAX_INSTANCES 1

class Compass
{
  public:
    int16_t product_id;                         /// product id
    uint32_t last_update;               ///< micros() time of last update
    Compass();
    /// Initialize the compass device.
    ///
    /// @returns    True if the compass was initialized OK, false if it was not
    ///             found or is not functioning.
    ///
    virtual bool init();

    /// Read the compass and update the mag_ variables.
    ///
    virtual bool read(void) = 0;

    

    /// use spare CPU cycles to accumulate values from the compass if
    /// possible
    virtual void accumulate(void) = 0;

    /// Calculate the tilt-compensated heading_ variables.
    ///
    /// @param dcm_matrix			The current orientation rotation matrix
    ///
    /// @returns heading in radians
    ///
    float calculate_heading(const Matrix3f &dcm_matrix) const;

    /// Sets offset x/y/z values.
    ///
    /// @param  i                   compass instance
    /// @param  offsets             Offsets to the raw mag_ values.
    ///
    void set_offsets(uint8_t i, const Vector3f &offsets);

    /// Sets and saves the compass offset x/y/z values.
    ///
    /// @param  i                   compass instance
    /// @param  offsets             Offsets to the raw mag_ values.
    ///
    void set_and_save_offsets(uint8_t i, const Vector3f &offsets);

    /// Saves the current offset x/y/z values for one or all compasses
    ///
    /// @param  i                   compass instance
    ///
    /// This should be invoked periodically to save the offset values maintained by
    /// ::learn_offsets.
    ///
    void save_offsets(uint8_t i);
    void save_offsets(void);

    // return the number of compass instances
    virtual uint8_t get_count(void) const { return 1; }

    /// Return the current field as a Vector3f
    const Vector3f &get_field(uint8_t i) const { return _field[i]; }
    const Vector3f &get_field(void) const { return get_field(get_primary()); }

    /// Return the health of a compass
    bool healthy(uint8_t i) const { return _healthy[i]; }
    bool healthy(void) const { return healthy(get_primary()); }

    /// set the current field as a Vector3f
    void set_field(const Vector3f &field) { _field[0] = field; }

    /// Returns the current offset values
    ///
    /// @returns                    The current compass offsets.
    ///
    const Vector3f &get_offsets(uint8_t i) const { return _offset[i]; }
    const Vector3f &get_offsets(void) const { return get_offsets(get_primary()); }


    /// Program new offset values.
    ///
    /// @param  i                   compass instance
    /// @param  x                   Offset to the raw mag_x value.
    /// @param  y                   Offset to the raw mag_y value.
    /// @param  z                   Offset to the raw mag_z value.
    ///
    void set_and_save_offsets(uint8_t i, int x, int y, int z) {
        set_and_save_offsets(i, Vector3f(x, y, z));
    }

    // learn offsets accessor
    bool learn_offsets_enabled() const { return _learn; }

    /// Perform automatic offset updates
    ///
    void learn_offsets(void);

    /// return true if the compass should be used for yaw calculations
    bool use_for_yaw(void) const {
        return _healthy[0] && _use_for_yaw;
    }

    /// Sets the local magnetic field declination.
    ///
    /// @param  radians             Local field declination.
    /// @param save_to_eeprom       true to save to eeprom (false saves only to memory)
    ///
    void set_declination(float radians, bool save_to_eeprom = true);
    float get_declination() const;

    // set overall board orientation
    void set_board_orientation(enum Rotation orientation) {
        _board_orientation = orientation;
    }

    /// Set the motor compensation type
    ///
    /// @param  comp_type           0 = disabled, 1 = enabled use throttle, 2 = enabled use current
    ///
    void motor_compensation_type(const uint8_t comp_type) {
        if (_motor_comp_type <= COMPASS_MOT_COMP_CURRENT && _motor_comp_type != (int8_t)comp_type) {
            _motor_comp_type = (int8_t)comp_type;
            _thr_or_curr = 0;                               // set current current or throttle to zero
            for (uint8_t i=0; i<COMPASS_MAX_INSTANCES; i++) {
                set_motor_compensation(i, Vector3f(0,0,0)); // clear out invalid compensation vectors
            }
        }
    }

    /// get the motor compensation value.
    uint8_t motor_compensation_type() const {
        return _motor_comp_type;
    }

    /// Set the motor compensation factor x/y/z values.
    ///
    /// @param  i                   instance of compass
    /// @param  offsets             Offsets multiplied by the throttle value and added to the raw mag_ values.
    ///
    void set_motor_compensation(uint8_t i, const Vector3f &motor_comp_factor);

    /// get motor compensation factors as a vector
    const Vector3f& get_motor_compensation(uint8_t i) const { return _motor_compensation[i]; }
    const Vector3f& get_motor_compensation(void) const { return get_motor_compensation(get_primary()); }

    /// Saves the current motor compensation x/y/z values.
    ///
    /// This should be invoked periodically to save the offset values calculated by the motor compensation auto learning
    ///
    void save_motor_compensation();

    /// Returns the current motor compensation offset values
    ///
    /// @returns                    The current compass offsets.
    ///
    const Vector3f &get_motor_offsets(uint8_t i) const { return _motor_offset[i]; }
    const Vector3f &get_motor_offsets(void) const { return get_motor_offsets(get_primary()); }

    /// Set the throttle as a percentage from 0.0 to 1.0
    /// @param thr_pct              throttle expressed as a percentage from 0 to 1.0
    void set_throttle(float thr_pct) {
        if(_motor_comp_type == COMPASS_MOT_COMP_THROTTLE) {
            _thr_or_curr = thr_pct;
        }
    }

    /// Set the current used by system in amps
    /// @param amps                 current flowing to the motors expressed in amps
    void set_current(float amps) {
        if(_motor_comp_type == COMPASS_MOT_COMP_CURRENT) {
            _thr_or_curr = amps;
        }
    }

    /// Returns True if the compasses have been configured (i.e. offsets saved)
    ///
    /// @returns                    True if compass has been configured
    ///
    bool configured(uint8_t i);
    bool configured(void);

    /// Returns the instance of the primary compass
    ///
    /// @returns                    the instance number of the primary compass
    ///
    virtual uint8_t get_primary(void) const { return 0; }

    // settable parameters
    AP_Int8 _learn;                             ///<enable calibration learning

  protected:

    bool _healthy[COMPASS_MAX_INSTANCES];
    Vector3f _field[COMPASS_MAX_INSTANCES];     ///< magnetic field strength

    AP_Int8 _orientation;
    AP_Vector3f _offset[COMPASS_MAX_INSTANCES];
    AP_Float _declination;
    AP_Int8 _use_for_yaw;                       ///<enable use for yaw calculation
    AP_Int8 _auto_declination;                  ///<enable automatic declination code
    AP_Int8 _external;                          ///<compass is external
    bool _null_init_done;                           ///< first-time-around flag used by offset nulling

    ///< used by offset correction
    static const uint8_t _mag_history_size = 20;
    uint8_t _mag_history_index[COMPASS_MAX_INSTANCES];
    Vector3i _mag_history[COMPASS_MAX_INSTANCES][_mag_history_size];

    // motor compensation
    AP_Int8     _motor_comp_type;               // 0 = disabled, 1 = enabled for throttle, 2 = enabled for current
    AP_Vector3f _motor_compensation[COMPASS_MAX_INSTANCES]; // factors multiplied by throttle and added to compass outputs
    Vector3f    _motor_offset[COMPASS_MAX_INSTANCES]; // latest compensation added to compass
    float       _thr_or_curr;                   // throttle expressed as a percentage from 0 ~ 1.0 or current expressed in amps

    // board orientation from AHRS
    enum Rotation _board_orientation;
};

#endif //__COMPASS_H__
