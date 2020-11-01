#ifndef __INERTIALSENSOR_H__
#define __INERTIALSENSOR_H__

#include <stdint.h> 
#include "AP_Param.h"
#include "Math.h"


// Gyro and Accelerometer calibration criteria
#define INERTIAL_SENSOR_ACCEL_TOT_MAX_OFFSET_CHANGE  4.0f
#define INERTIAL_SENSOR_ACCEL_MAX_OFFSET             250.0f
#define INS_MAX_INSTANCES 1
#define INS_MAX_BACKENDS  1
#define AP_PRODUCT_ID_NONE                      0x00    // Hardware in the loop

class InertialSensor_Backend;

class InertialSensor
{
  friend class InertialSensor_Backend;

  public:
    InertialSensor();

    enum Start_style {
        COLD_START = 0,
        WARM_START
    };

    // the rate that updates will be available to the application
    enum Sample_rate {
        RATE_50HZ,
        RATE_100HZ,
        RATE_200HZ,
        RATE_400HZ
    };
    
    void init( Start_style style, Sample_rate sample_rate);
    void init_accel();
    uint8_t register_gyro(void);
    uint8_t register_accel(void);
    bool calibrate_accel(float& trim_roll,float& trim_pitch);
    bool calibrated();
    void init_gyro(void);
    void               set_gyro(uint8_t instance, const Vector3f &gyro);
    void               set_accel(uint8_t instance, const Vector3f &accel);
    
    const Vector3f     &get_gyro(uint8_t i) const { return _gyro[i]; }
    const Vector3f     &get_gyro(void) const { return get_gyro(_primary_gyro); }
    const Vector3f     &get_accel(uint8_t i) const { return _accel[i]; }
    const Vector3f     &get_accel(void) const { return get_accel(_primary_accel); }
    
    const Vector3f     &get_gyro_offsets(uint8_t i) const { return _gyro_offset[i]; }
    const Vector3f     &get_gyro_offsets(void) const { return get_gyro_offsets(_primary_gyro); }
    
    uint32_t           get_gyro_error_count(uint8_t i) const { return _gyro_error_count[i]; }
    uint32_t           get_accel_error_count(uint8_t i) const { return _accel_error_count[i]; }
    bool               get_gyro_health(uint8_t instance) const { return _gyro_healthy[instance]; }
    bool               get_gyro_health(void) const { return get_gyro_health(_primary_gyro); }
    bool               get_gyro_health_all(void) const;
    uint8_t            get_gyro_count(void) const { return _gyro_count; }
    bool               gyro_calibrated_ok(uint8_t instance) const { return _gyro_cal_ok[instance]; }
    bool               gyro_calibrated_ok_all() const;
    
    bool               get_accel_health(uint8_t instance) const { return _accel_healthy[instance]; }
    bool               get_accel_health(void) const { return get_accel_health(_primary_accel); }
    bool               get_accel_health_all(void) const;
    uint8_t            get_accel_count(void) const { return _accel_count; };
    
    // get accel offsets in m/s/s
    const Vector3f     &get_accel_offsets(uint8_t i) const { return _accel_offset[i]; }
    const Vector3f     &get_accel_offsets(void) const { return get_accel_offsets(_primary_accel); }

    // get accel scale
    const Vector3f     &get_accel_scale(uint8_t i) const { return _accel_scale[i]; }
    const Vector3f     &get_accel_scale(void) const { return get_accel_scale(_primary_accel); }
    /* get_delta_time returns the time period in seconds
     * overwhich the sensor data was collected
     */
    float get_delta_time() const { return _delta_time; }

    // return the maximum gyro drift rate in radians/s/s. This
    // depends on what gyro chips are being used
    float get_gyro_drift_rate(void) const { return ToRad(0.5f/60); }

    // update gyro and accel values from accumulated samples
    void update(void);

    // wait for a sample to be available
    void wait_for_sample(void);

    // set overall board orientation
    void set_board_orientation(enum Rotation orientation) {
        _board_orientation = orientation;
    }

    // override default filter frequency
  
    void set_default_filter(float filter_hz) {
      _mpu6000_filter.set(filter_hz);
    }
   

    // get_filter - return filter in hz
    uint8_t get_filter() const { return _mpu6000_filter.get(); }

    // return the selected sample rate
    Sample_rate get_sample_rate(void) const { return _sample_rate; }

    uint16_t error_count(void) const { return 0; }
    bool healthy(void) const { return get_gyro_health() && get_accel_health(); }

    uint8_t get_primary_accel(void) const { return 0; }

    // enable HIL mode
    void set_hil_mode(void) { _hil_mode = true; }
    
    // backend objects
    InertialSensor_Backend *_backends[INS_MAX_BACKENDS];
    
  private:
    // load backend drivers
    void _add_backend(InertialSensor_Backend *(detect)(InertialSensor &));
    void _detect_backends(void);

    // accel and gyro initialisation
    void _init_accel();
    void _init_gyro();
    
    bool _calibrate_accel(Vector3f accel_sample[6], Vector3f& accel_offsets, Vector3f& accel_scale);
    void _calibrate_update_matrices(float dS[6], float JS[6][6], float beta[6], float data[3]);
    void _calibrate_reset_matrices(float dS[6], float JS[6][6]);
    void _calibrate_find_delta(float dS[6], float JS[6][6], float delta[6]);
    void _calculate_trim(Vector3f accel_sample, float& trim_roll, float& trim_pitch);
    
    // check if we have 3D accel calibration
    void check_3D_calibration(void);

    // save parameters to eeprom
    //void  _save_parameters();

    // number of gyros and accel drivers. Note that most backends
    // provide both accel and gyro data, so will increment both
    // counters on initialisation
    uint8_t _gyro_count;
    uint8_t _accel_count;
    uint8_t _backend_count;

    // the selected sample rate
    Sample_rate _sample_rate;
    
    // Most recent accelerometer reading
    Vector3f _accel[INS_MAX_INSTANCES];

    // Most recent gyro reading
    Vector3f _gyro[INS_MAX_INSTANCES];

    // product id
    AP_Int16 _product_id;

    // accelerometer scaling and offsets
    AP_Vector3f _accel_scale[INS_MAX_INSTANCES];
    AP_Vector3f _accel_offset[INS_MAX_INSTANCES];
    AP_Vector3f _gyro_offset[INS_MAX_INSTANCES];

    // filtering frequency (0 means default)
    AP_Int8     _mpu6000_filter;

    // board orientation from AHRS
    enum Rotation _board_orientation;

    // calibrated_ok flags
    bool _gyro_cal_ok[INS_MAX_INSTANCES];

    // primary accel and gyro
    uint8_t _primary_gyro;
    uint8_t _primary_accel;

    // has wait_for_sample() found a sample?
    bool _have_sample:1;

    // are we in HIL mode?
    bool _hil_mode:1;

    // do we have offsets/scaling from a 3D calibration?
    bool _have_3D_calibration:1;

    // the delta time in seconds for the last sample
    float _delta_time;

    // last time a wait_for_sample() returned a sample
    uint32_t _last_sample_usec;

    // target time for next wait_for_sample() return
    uint32_t _next_sample_usec;
    
    // time between samples in microseconds
    uint32_t _sample_period_usec;

    // health of gyros and accels
    bool _gyro_healthy[INS_MAX_INSTANCES];
    bool _accel_healthy[INS_MAX_INSTANCES];

    uint32_t _accel_error_count[INS_MAX_INSTANCES];
    uint32_t _gyro_error_count[INS_MAX_INSTANCES];
    
};

class InertialSensor_Backend
{
  public:
    InertialSensor_Backend(InertialSensor &imu);

    // we declare a virtual destructor so that drivers can
    // override with a custom destructor if need be.
    virtual ~InertialSensor_Backend(void) {}

    /* 
     * Update the sensor data. Called by the frontend to transfer
     * accumulated sensor readings to the frontend structure via the
     * _rotate_and_offset_gyro() and _rotate_and_offset_accel() functions
     */
    virtual bool update() = 0;

    /* 
     * return true if at least one accel sample is available in the backend
     * since the last call to update()
     */
    virtual bool accel_sample_available() = 0;

    /* 
     * return true if at least one gyro sample is available in the backend
     * since the last call to update()
     */
    virtual bool gyro_sample_available() = 0;

    /*
      return the product ID
     */
    int16_t product_id(void) const { return _product_id; }

  protected:
    // access to frontend
    InertialSensor &_imu;

    // rotate gyro vector and offset
    void _rotate_and_offset_gyro(uint8_t instance, const Vector3f &gyro);

    // rotate accel vector, scale and offset
    void _rotate_and_offset_accel(uint8_t instance, const Vector3f &accel);

    // set accelerometer error_count
    void _set_accel_error_count(uint8_t instance, uint32_t error_count);

    // set gyro error_count
    void _set_gyro_error_count(uint8_t instance, uint32_t error_count);

    // backend should fill in its product ID from AP_PRODUCT_ID_*
    int16_t _product_id;

    // return the default filter frequency in Hz for the sample rate
    uint8_t _default_filter(void) const;

    // note that each backend is also expected to have a static detect()
    // function which instantiates an instance of the backend sensor
    // driver if the sensor is available
};

#include "InertialSensor_MPU9250.h"

#endif //
