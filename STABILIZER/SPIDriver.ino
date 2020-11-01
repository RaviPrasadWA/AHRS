#include "SPIDriver.h"
#include <SPI.h>

bool SPIDriver::_force_low_speed;
static volatile bool spi0_transferflag = false;

void SPIDriver::init() {    
    SPI.begin( );
    SPI.setDataMode( SPI_MODE0 );
    SPI.setBitOrder( MSBFIRST );
    SPI.setClockDivider( 16 ); // 1MHz
    pinMode(SPI0_CS_PIN, OUTPUT);

    digitalWrite(SPI0_CS_PIN, HIGH);

    /* Enable the SPI0 peripheral as a master */
    //SPCR = _BV(SPE) | _BV(MSTR);
}

void SPIDriver::_cs_assert() 
{
  
    /*  
    const uint8_t valid_spcr_mask = (_BV(CPOL) | _BV(CPHA) | _BV(SPR1) | _BV(SPR0));
    if (_force_low_speed) {
        _spcr = _spcr_lowspeed;
    }
    uint8_t new_spcr = (SPCR & ~valid_spcr_mask) | (_spcr & valid_spcr_mask);
    SPCR = new_spcr;  

    const uint8_t valid_spsr_mask = _BV(SPI2X);
    uint8_t new_spsr = (SPSR & ~valid_spsr_mask) | (_spsr & valid_spsr_mask);
    SPSR = new_spsr;
    */
    digitalWrite(SPI0_CS_PIN, LOW);
}

void SPIDriver::_cs_release() 
{
    digitalWrite(SPI0_CS_PIN, HIGH);
}

uint8_t SPIDriver::_transfer(uint8_t data) 
{
    /*
    if (spi0_transferflag) {
        Serial.println("[o] SPI[info]: SPI0 transfer collision");
    }
    spi0_transferflag = true;
    SPDR = data;
    if (SPSR & _BV(WCOL)) {
        Serial.println("[o] SPI[info]: SPI0 write collision");
        return 0;
    }
    while(!(SPSR & _BV(SPIF)));
    uint8_t read_spdr = SPDR;
    spi0_transferflag = false;
    
    return read_spdr;
    */
    return SPI.transfer( data );
}

/**
   a specialised transfer function for the MPU6k. This saves 2 usec
   per byte
 */
void SPIDriver::_transfer16(const uint8_t *tx, uint8_t *rx) 
{
    spi0_transferflag = true;
#define TRANSFER1(i) do { SPDR = tx[i];  while(!(SPSR & _BV(SPIF))); rx[i] = SPDR; } while(0)
    TRANSFER1(0);
    TRANSFER1(1);
    TRANSFER1(2);
    TRANSFER1(3);
    TRANSFER1(4);
    TRANSFER1(5);
    TRANSFER1(6);
    TRANSFER1(7);
    TRANSFER1(8);
    TRANSFER1(9);
    TRANSFER1(10);
    TRANSFER1(11);
    TRANSFER1(12);
    TRANSFER1(13);
    TRANSFER1(14);
    TRANSFER1(15);
    spi0_transferflag = false;
}

void SPIDriver::transfer(const uint8_t *tx, uint16_t len) {
    for (uint16_t i = 0; i < len; i++) {
            _transfer(tx[i]);
    }
}

void SPIDriver::transaction(const uint8_t *tx, uint8_t *rx, uint16_t len) 
{
    _cs_assert();
    if (rx == NULL) {
        for (uint16_t i = 0; i < len; i++) {
            _transfer(tx[i]);
        }
    } else {
        /*
        while (len >= 16) { //
            _transfer16(tx, rx);
            tx += 16;
            rx += 16;
            len -= 16;
        }*/
        for (uint16_t i = 0; i < len; i++) {
            rx[i] = _transfer(tx[i]);
        }
    }
    _cs_release();
}

void SPIDriver::cs_assert() {
    _cs_assert();
}

void SPIDriver::cs_release() {
    _cs_release();
}

uint8_t SPIDriver::transfer(uint8_t data) {
    return _transfer(data);
}

/**
   allow on the fly bus speed changes for MPU6000
 */
void SPIDriver::set_bus_speed(SPIDriver::bus_speed speed) 
{
    if (speed == SPIDriver::SPI_SPEED_HIGH) {
        _spcr = _spcr_highspeed;
        _force_low_speed = false;
    } else {
        _spcr = _spcr_lowspeed;
        _force_low_speed = true;
    }
}

