#ifndef __SPI_DRIVER_H__
#define __SPI_DRIVER_H__

#include <stdint.h>


#define SPI0_MISO_PIN 50
#define SPI0_MOSI_PIN 51
#define SPI0_SCK_PIN  52
#define SPI0_CS_PIN  53

class SPIDriver
{
  public:
    enum bus_speed { SPI_SPEED_LOW, SPI_SPEED_HIGH };
    void init();

    void transaction(const uint8_t *tx, uint8_t *rx, uint16_t len);

    void cs_assert();
    void cs_release();
    uint8_t transfer(uint8_t data);
    void transfer(const uint8_t *data, uint16_t len);
    void set_bus_speed(enum bus_speed speed);
    
  private:
    void _cs_assert();
    void _cs_release();
    uint8_t _transfer(uint8_t data);
   
    void _transfer16(const uint8_t *tx, uint8_t *rx);

    static bool _force_low_speed;

    const uint8_t _spcr_lowspeed;
    const uint8_t _spcr_highspeed;
    uint8_t _spcr;
    const uint8_t _spsr;

};

#endif //__SPI_DRIVER_H__
