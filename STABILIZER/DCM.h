#ifndef __DCM_H__
#define __DCM_H__

#include "AP_Param.h"

#define SPIN_RATE_LIMIT 20

enum AHRS_VehicleClass {
    AHRS_VEHICLE_UNKNOWN,
    AHRS_VEHICLE_GROUND,
    AHRS_VEHICLE_COPTER,
    AHRS_VEHICLE_FIXED_WING,
};


#define AHRS_TRIM_LIMIT 10.0f        // maximum trim angle in degrees
#define AHRS_RP_P_MIN   0.05f        // minimum value for AHRS_RP_P parameter
#define AHRS_YAW_P_MIN  0.05f        // minimum value for AHRS_YAW_P parameter

class DCM_BASE
{
  public:
    DCM_BASE(InertialSensor &ins) :
      roll(0.0f),
      pitch(0.0f),
      yaw(0.0f),
      roll_sensor(0),
      pitch_sensor(0),
      yaw_sensor(0),
      _vehicle_class(AHRS_VEHICLE_UNKNOWN),
      _compass(NULL),
      _compass_last_update(0),
      _ins(ins),
      _cos_roll(1.0f),
      _cos_pitch(1.0f),
      _cos_yaw(1.0f),
      _sin_roll(0.0f),
      _sin_pitch(0.0f),
      _sin_yaw(0.0f),
      //_trim(0),
      _active_accel_instance(0)
      {
        _gyro_drift_limit = ins.get_gyro_drift_rate();
        
        // enable centrifugal correction by default
        _flags.correct_centrifugal = true;
        _kp_yaw = 0.2f;
        _kp = 0.2f;
        _board_orientation = 0;
      
      }

    // empty virtual destructor
    virtual ~DCM_BASE() {}
    
    // init sets up INS board orientation
    virtual void init() {
        set_orientation();
    };
    
    AHRS_VehicleClass get_vehicle_class(void) const {
        return _vehicle_class;
    }

    void set_vehicle_class(AHRS_VehicleClass vclass) {
        _vehicle_class = vclass;
    }

    void set_compass(Compass *compass) {
        _compass = compass;
        set_orientation();
    }

    const Compass* get_compass() const {
        return _compass;
    }
        
    // allow for runtime change of orientation
    // this makes initial config easier
    void set_orientation() {
        _ins.set_board_orientation((enum Rotation)_board_orientation.get());
        if (_compass != NULL) {
            _compass->set_board_orientation((enum Rotation)_board_orientation.get());
        }
    }
    
     const InertialSensor &get_ins() const {
	    return _ins;
    }
    
    // accelerometer values in the earth frame in m/s/s
    const Vector3f &get_accel_ef(void) const { return _accel_ef[_ins.get_primary_accel()]; }
    
    // Methods
    virtual void update(void) = 0;

    // Euler angles (radians)
    float roll;
    float pitch;
    float yaw;

    // integer Euler angles (Degrees * 100)
    int32_t roll_sensor;
    int32_t pitch_sensor;
    int32_t yaw_sensor;

    // return a smoothed and corrected gyro vector
    virtual const Vector3f &get_gyro(void) const = 0;

    // return the current estimate of the gyro drift
    virtual const Vector3f &get_gyro_drift(void) const = 0;

    // reset the current gyro drift estimate
    //  should be called if gyro offsets are recalculated
    virtual void reset_gyro_drift(void) = 0;

    // reset the current attitude, used on new IMU calibration
    virtual void reset(bool recover_eulers=false) = 0;

    // reset the current attitude, used on new IMU calibration
    virtual void reset_attitude(const float &roll, const float &pitch, const float &yaw) = 0;

    // return the average size of the roll/pitch error estimate
    // since last call
    virtual float get_error_rp(void) = 0;

    // return the average size of the yaw error estimate
    // since last call
    virtual float get_error_yaw(void) = 0;

    // return a DCM rotation matrix representing our current
    // attitude
    virtual const Matrix3f &get_dcm_matrix(void) const = 0;

    
    // return true if we will use compass for yaw
    virtual bool use_compass(void) { return _compass && _compass->use_for_yaw(); }

    // return true if yaw has been initialised
    bool yaw_initialised(void) const {
        return _flags.have_initial_yaw;
    }

    // set the fast gains flag
    void set_fast_gains(bool setting) {
        _flags.fast_ground_gains = setting;
    }

    // set the correct centrifugal flag
    // allows arducopter to disable corrections when disarmed
    void set_correct_centrifugal(bool setting) {
        _flags.correct_centrifugal = setting;
    }

    // get the correct centrifugal flag
    bool get_correct_centrifugal(void) const {
        return _flags.correct_centrifugal;
    }
    
    // get trim
    const Vector3f &get_trim() const { return _trim.get(); }

    // set trim
    virtual void            set_trim(Vector3f new_trim);

    // add_trim - adjust the roll and pitch trim up to a total of 10 degrees
    virtual void            add_trim(float roll_in_radians, float pitch_in_radians, bool save_to_eeprom = true);

    // helper trig value accessors
    float cos_roll() const  { return _cos_roll; }
    float cos_pitch() const { return _cos_pitch; }
    float cos_yaw() const   { return _cos_yaw; }
    float sin_roll() const  { return _sin_roll; }
    float sin_pitch() const { return _sin_pitch; }
    float sin_yaw() const   { return _sin_yaw; }

    // these are public for ArduCopter
    AP_Float _kp_yaw;
    AP_Float _kp;
    AP_Float gps_gain;
    
    // return true if the AHRS object supports inertial navigation,
    // with very accurate position and velocity
    virtual bool have_inertial_nav(void) const { return false; }

    // return the active accelerometer instance
    uint8_t get_active_accel_instance(void) const { return _active_accel_instance; }

    // is the AHRS subsystem healthy?
    virtual bool healthy(void) = 0;

    // true if the AHRS has completed initialisation
    virtual bool initialised(void) const { return true; };
  
  protected:
     AHRS_VehicleClass _vehicle_class;
     AP_Int8 _board_orientation;
     // time in microseconds of last compass update
     uint32_t _compass_last_update;
     
     // flags structure
    struct ahrs_flags {
        uint8_t have_initial_yaw        : 1;    // whether the yaw value has been intialised with a reference
        uint8_t fast_ground_gains       : 1;    // should we raise the gain on the accelerometers for faster convergence, used when disarmed for ArduCopter
        uint8_t correct_centrifugal     : 1;    // 1 if we should correct for centrifugal forces 
        uint8_t wind_estimation         : 1;    // 1 if we should do wind estimation
    } _flags;
    
    // update_trig - recalculates _cos_roll, _cos_pitch, etc based on latest attitude
    //      should be called after _dcm_matrix is updated
    void update_trig(void);

    // update roll_sensor, pitch_sensor and yaw_sensor
    void update_cd_values(void);

    // pointer to compass object, if available
    Compass         * _compass;
    InertialSensor   &_ins;
    
    // a vector to capture the difference between the controller and body frames
    AP_Vector3f         _trim;

    // the limit of the gyro drift claimed by the sensors, in
    // radians/s/s
    float _gyro_drift_limit;

    // accelerometer values in the earth frame in m/s/s
    Vector3f        _accel_ef[INS_MAX_INSTANCES];
    
    // helper trig variables
    float _cos_roll, _cos_pitch, _cos_yaw;
    float _sin_roll, _sin_pitch, _sin_yaw;

    // which accelerometer instance is active
    uint8_t _active_accel_instance;

};

class DCM : public DCM_BASE
{
 public:
  DCM(InertialSensor &ins):
    DCM_BASE(ins),
    _omega_I_sum_time(0.0f),
    _renorm_val_sum(0.0f),
    _renorm_val_count(0),
    _error_rp_sum(0.0f),
    _error_rp_count(0),
    _error_rp_last(0.0f),
    _error_yaw_sum(0.0f),
    _error_yaw_count(0),
    _error_yaw_last(0.0f),
    _ra_deltat(0.0f),
    _ra_sum_start(0),
    _last_declination(0.0f),
    _mag_earth(1,0),
    _last_wind_time(0),
    _last_airspeed(0.0f),
    _position_offset_north(0.0f),
    _position_offset_east(0.0f),
    _last_consistent_heading(0),
    _last_failure_ms(0)
    {
        _dcm_matrix.identity();

        // these are experimentally derived from the simulator
        // with large drift levels
        _ki = 0.0087;
        _ki_yaw = 0.01;
    }
    
    // return the smoothed gyro vector corrected for drift
    const Vector3f &get_gyro(void) const {
        return _omega;
    }

    // return rotation matrix representing rotaton from body to earth axes
    const Matrix3f &get_dcm_matrix(void) const {
        return _body_dcm_matrix;
    }

    // return the current drift correction integrator value
    const Vector3f &get_gyro_drift(void) const {
        return _omega_I;
    }

    // reset the current gyro drift estimate
    //  should be called if gyro offsets are recalculated
    void reset_gyro_drift(void);

    // Methods
    void            update(void);
    void            reset(bool recover_eulers = false);

    // reset the current attitude, used on new IMU calibration
    void reset_attitude(const float &roll, const float &pitch, const float &yaw);

    // status reporting
    float           get_error_rp(void);
    float           get_error_yaw(void);
    bool            use_compass(void);
    
    // return a wind estimation vector, in m/s
    Vector3f wind_estimate(void) {
        return _wind;
    }
    void estimate_wind(void);
    // is the AHRS subsystem healthy?
    bool healthy(void);
    
  private:
    float _ki;
    float _ki_yaw;

    // Methods
    void            matrix_update(float _G_Dt);
    void            normalize(void);
    void            check_matrix(void);
    bool            renorm(Vector3f const &a, Vector3f &result);
    void            drift_correction(float deltat);
    void            drift_correction_yaw(void);
    float           yaw_error_compass();
    void            euler_angles(void);
    
    // primary representation of attitude of board used for all inertial calculations
    Matrix3f _dcm_matrix;

    // primary representation of attitude of flight vehicle body
    Matrix3f _body_dcm_matrix;

    Vector3f _omega_P;                          // accel Omega proportional correction
    Vector3f _omega_yaw_P;                      // proportional yaw correction
    Vector3f _omega_I;                          // Omega Integrator correction
    Vector3f _omega_I_sum;
    float _omega_I_sum_time;
    Vector3f _omega;                            // Corrected Gyro_Vector data

    // variables to cope with delaying the GA sum to match GPS lag
    Vector3f ra_delayed(uint8_t instance, const Vector3f &ra);
    Vector3f _ra_delay_buffer[INS_MAX_INSTANCES];

    // P term gain based on spin rate
    float           _P_gain(float spin_rate);

    // P term yaw gain based on rate of change of horiz velocity
    float           _yaw_gain(void) const;

    // state to support status reporting
    float _renorm_val_sum;
    uint16_t _renorm_val_count;
    float _error_rp_sum;
    uint16_t _error_rp_count;
    float _error_rp_last;
    float _error_yaw_sum;
    uint16_t _error_yaw_count;
    float _error_yaw_last;
    
    // state of accel drift correction
    Vector3f _ra_sum[INS_MAX_INSTANCES];
    Vector3f _last_velocity;
    float _ra_deltat;
    uint32_t _ra_sum_start;

    // the earths magnetic field
    float _last_declination;
    Vector2f _mag_earth;
    
    // support for wind estimation
    Vector3f _last_fuse;
    Vector3f _last_vel;
    uint32_t _last_wind_time;
    float _last_airspeed;
    uint32_t _last_consistent_heading;
    
    float _position_offset_north;
    float _position_offset_east;

    // estimated wind in m/s
    Vector3f _wind;
    
    // last time AHRS failed in milliseconds
    uint32_t _last_failure_ms;
};

#endif //__AHRS_H__
