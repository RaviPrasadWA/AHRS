#ifndef __COMPASS_HMC5883L_H__
#define __COMPASS_HMC5883L_H__

#include "Compass.h"

#define COMPASS_ADDRESS      0x1E
#define ConfigRegA           0x00
#define ConfigRegB           0x01
#define magGain              0x20
#define PositiveBiasConfig   0x11
#define NegativeBiasConfig   0x12
#define NormalOperation      0x10
#define ModeRegister         0x02
#define ContinuousConversion 0x00
#define SingleConversion     0x01

// ConfigRegA valid sample averaging for 5883L
#define SampleAveraging_1    0x00
#define SampleAveraging_2    0x01
#define SampleAveraging_4    0x02
#define SampleAveraging_8    0x03

// ConfigRegA valid data output rates for 5883L
#define DataOutputRate_0_75HZ 0x00
#define DataOutputRate_1_5HZ  0x01
#define DataOutputRate_3HZ    0x02
#define DataOutputRate_7_5HZ  0x03
#define DataOutputRate_15HZ   0x04
#define DataOutputRate_30HZ   0x05
#define DataOutputRate_75HZ   0x06

class Compass_HMC5883L : public Compass
{
  private:
    float               calibration[3];
    bool                _initialised;
    virtual bool        read_raw(void);
    uint8_t             _base_config;
    virtual bool        re_initialise(void);
    bool                read_register(uint8_t address, uint8_t *value);
    void                read_registers(uint8_t address, uint8_t len, uint8_t * buff);
    bool                write_register(uint8_t address, uint8_t value);
    uint32_t            _retry_time; // when unhealthy the millis() value to retry at

    int16_t		_mag_x;
    int16_t             _mag_y;
    int16_t		_mag_z;
    int16_t             _mag_x_accum;
    int16_t             _mag_y_accum;
    int16_t             _mag_z_accum;
    uint8_t		_accum_count;
    uint32_t            _last_accum_time;

  public:
    Compass_HMC5883L() : Compass() {
    }
    bool        init(void);
    bool        read(void);
    void        accumulate(void);
};

#endif //__COMPASS_HMC5883L_H__
