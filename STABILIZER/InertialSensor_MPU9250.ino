#include "InertialSensor_MPU9250.h"

InertialSensor_MPU9250::InertialSensor_MPU9250(InertialSensor &imu) :
    InertialSensor_Backend(imu),
    _last_filter_hz(-1),
    _shared_data_idx(0),
    _accel_filter_x(1000, 15),
    _accel_filter_y(1000, 15),
    _accel_filter_z(1000, 15),
    _gyro_filter_x(1000, 15),
    _gyro_filter_y(1000, 15),
    _gyro_filter_z(1000, 15),
    _have_sample_available(false)
{
    _spi->init();
}

/*
  detect the sensor
 */
InertialSensor_Backend *InertialSensor_MPU9250::detect(InertialSensor &_imu)
{   
    InertialSensor_MPU9250 * sensor = new InertialSensor_MPU9250(_imu);
    if (sensor == NULL) {
        return NULL;
    }
    if (!sensor->_init_sensor()) {
        delete sensor;
        return NULL;
    }

    return sensor;
}

/*
  initialise the sensor
 */
bool InertialSensor_MPU9250::_init_sensor(void)
{
    uint8_t whoami = _register_read(MPUREG_WHOAMI);
    if (whoami != 0x71) { //0x70
        // TODO: we should probably accept multiple chip
        // revisions. This is the one on the PXF
        Serial.print(F("[x] MPU9250[error]: unexpected WHOAMI "));
        Serial.print( (unsigned)whoami );
        Serial.println();
        return false;
    }
    
    uint8_t tries = 0;
    do {
        bool success = _hardware_init();
        if (success) {
            delay(10);
            uint8_t status = _register_read(MPUREG_INT_STATUS);
            if ((status & BIT_RAW_RDY_INT) != 0) {
                break;
            }
        }
        if (tries++ > 5) {
            return false;
        }
    } while (1);

    _gyro_instance = _imu.register_gyro();
    _accel_instance = _imu.register_accel();

    _product_id = AP_PRODUCT_ID_MPU9250;

    return true;
}

/*
  update the accel and gyro vectors
 */
bool InertialSensor_MPU9250::update( void )
{
    // pull the data from the timer shared data buffer
    uint8_t idx = _shared_data_idx;
    Vector3f gyro = _shared_data[idx]._gyro_filtered;
    Vector3f accel = _shared_data[idx]._accel_filtered;

    _have_sample_available = false;

    accel *= MPU9250_ACCEL_SCALE_1G;
    gyro *= GYRO_SCALE;

    _rotate_and_offset_gyro(_gyro_instance, gyro);
    _rotate_and_offset_accel(_accel_instance, accel);

    if (_last_filter_hz != _imu.get_filter()) {
        _set_filter(_imu.get_filter());
        _last_filter_hz = _imu.get_filter();
    }
    _poll_data();
    return true;
}

void InertialSensor_MPU9250::_poll_data(void)
{
    _read_data_transaction();
}

/*
  read from the data registers and update filtered data
 */
void InertialSensor_MPU9250::_read_data_transaction() 
{
    /* one resister address followed by seven 2-byte registers */
    struct PACKED {
        uint8_t cmd;
        uint8_t int_status;
        uint8_t v[14];
    } rx, tx = { cmd : MPUREG_INT_STATUS | 0x80, };

    _spi->transaction((const uint8_t *)&tx, (uint8_t *)&rx, sizeof(rx));

#define int16_val(v, idx) ((int16_t)(((uint16_t)v[2*idx] << 8) | v[2*idx+1]))

    Vector3f _accel_filtered = Vector3f(_accel_filter_x.apply(int16_val(rx.v, 1)), 
                                        _accel_filter_y.apply(int16_val(rx.v, 0)), 
                                        _accel_filter_z.apply(-int16_val(rx.v, 2)));

    Vector3f _gyro_filtered = Vector3f(_gyro_filter_x.apply(int16_val(rx.v, 5)), 
                                       _gyro_filter_y.apply(int16_val(rx.v, 4)), 
                                       _gyro_filter_z.apply(-int16_val(rx.v, 6)));
    // update the shared buffer
    uint8_t idx = _shared_data_idx ^ 1;
    _shared_data[idx]._accel_filtered = _accel_filtered;
    _shared_data[idx]._gyro_filtered = _gyro_filtered;
    _shared_data_idx = idx;

    _have_sample_available = true;
}

/*
  read an 8 bit register
 */
uint8_t InertialSensor_MPU9250::_register_read( uint8_t reg )
{
    uint8_t addr = reg | 0x80; // Set most significant bit

    uint8_t tx[2];
    uint8_t rx[2];

    tx[0] = addr;
    tx[1] = 0;
    _spi->transaction(tx, rx, 2);
    return rx[1];
}

/*
  write an 8 bit register
 */
void InertialSensor_MPU9250::_register_write(uint8_t reg, uint8_t val)
{
    uint8_t tx[2];
    uint8_t rx[2];

    tx[0] = reg;
    tx[1] = val;
    _spi->transaction(tx, rx, 2);
}

/*
  set the accel/gyro filter frequency
 */
void InertialSensor_MPU9250::_set_filter(uint8_t filter_hz)
{
    if (filter_hz == 0) {
        filter_hz = _default_filter_hz;
    }

    _accel_filter_x.set_cutoff_frequency(1000, filter_hz);
    _accel_filter_y.set_cutoff_frequency(1000, filter_hz);
    _accel_filter_z.set_cutoff_frequency(1000, filter_hz);

    _gyro_filter_x.set_cutoff_frequency(1000, filter_hz);
    _gyro_filter_y.set_cutoff_frequency(1000, filter_hz);
    _gyro_filter_z.set_cutoff_frequency(1000, filter_hz);    
}

/*
  initialise the sensor configuration registers
 */
bool InertialSensor_MPU9250::_hardware_init(void)
{
    // Chip reset
    _spi->set_bus_speed(SPIDriver::SPI_SPEED_LOW);
    uint8_t tries;
    for (tries = 0; tries<5; tries++) {

        //hal.scheduler->delay(100);

        // Wake up device and select GyroZ clock. Note that the
        // MPU6000 starts up in sleep mode, and it can take some time
        // for it to come out of sleep
        _register_write(MPUREG_PWR_MGMT_1, BIT_PWR_MGMT_1_CLK_ZGYRO);
        delay(5);

        // check it has woken up
        if (_register_read(MPUREG_PWR_MGMT_1) == BIT_PWR_MGMT_1_CLK_ZGYRO) {
            break;
        }
    }
    if (tries == 5) {
        Serial.println(F("[x] MPU9250[error]: Failed to boot 5 times"));
        return false;
    }

    _register_write(MPUREG_PWR_MGMT_2, 0x00);            // only used for wake-up in accelerometer only low power mode


    _default_filter_hz = _default_filter();

    // used a fixed filter of 42Hz on the sensor, then filter using
    // the 2-pole software filter
    _register_write(MPUREG_CONFIG, BITS_DLPF_CFG_42HZ);

    // set sample rate to 1kHz, and use the 2 pole filter to give the
    // desired rate
    _register_write(MPUREG_SMPLRT_DIV, MPUREG_SMPLRT_1000HZ);
    _register_write(MPUREG_GYRO_CONFIG, BITS_GYRO_FS_2000DPS);  // Gyro scale 2000º/s

    // RM-MPU-9250A-00.pdf, pg. 15, select accel full scale 8g
    _register_write(MPUREG_ACCEL_CONFIG,2<<3);

    // configure interrupt to fire when new data arrives
    _register_write(MPUREG_INT_ENABLE, BIT_RAW_RDY_EN);

    // clear interrupt on any read, and hold the data ready pin high
    // until we clear the interrupt
    _register_write(MPUREG_INT_PIN_CFG, BIT_INT_RD_CLEAR | BIT_LATCH_INT_EN);
  
    _spi->set_bus_speed(SPIDriver::SPI_SPEED_HIGH);
    
    return true;
}

