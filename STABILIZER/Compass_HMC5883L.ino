#include "Compass_HMC5883L.h"
#include <Wire.h>

void Compass_HMC5883L::read_registers(uint8_t address, uint8_t len, uint8_t* buff)
{
    Wire.beginTransmission((uint8_t)COMPASS_ADDRESS);
    #if ARDUINO >= 100
        Wire.write(address);
    #else
        Wire.send(address);
    #endif
    Wire.endTransmission();

    Wire.beginTransmission((uint8_t)COMPASS_ADDRESS);
    Wire.requestFrom((uint8_t)COMPASS_ADDRESS, len);
    while(!Wire.available()) {};
    #if ARDUINO >= 100
        for( uint8_t iter=0 ; iter<len ; iter++)
        {
          buff[iter] = Wire.read();
        }
    #else
        for( uint8_t iter=0 ; iter<len ; iter++)
        {
          buff[iter] = Wire.receive();
        }
    #endif
    Wire.endTransmission();
}

// read_register - read a register value
bool Compass_HMC5883L::read_register(uint8_t address, uint8_t *value)
{
    read_registers(address, 1, value);

    if(value==NULL){
      return false;
    }
    return true;
}

// write_register - update a register value
bool Compass_HMC5883L::write_register(uint8_t address, uint8_t value)
{
    Wire.beginTransmission((uint8_t)COMPASS_ADDRESS);
    #if ARDUINO >= 100
        Wire.write(address);
        Wire.write(value);
    #else
        Wire.send(address);
        Wire.send(value);
    #endif
    Wire.endTransmission();
    return true;
}

// Read Sensor data
bool Compass_HMC5883L::read_raw()
{
    uint8_t buff[6];
    read_registers(0x03, 6, buff);

    int16_t rx, ry, rz;
    rx = (((int16_t)buff[0]) << 8) | buff[1];
    if (product_id == COMPASS_TYPE_HMC5883L) {
        rz = (((int16_t)buff[2]) << 8) | buff[3];
        ry = (((int16_t)buff[4]) << 8) | buff[5];
    } else {
        ry = (((int16_t)buff[2]) << 8) | buff[3];
        rz = (((int16_t)buff[4]) << 8) | buff[5];
    }
    if (rx == -4096 || ry == -4096 || rz == -4096) {
        // no valid data available
        return false;
    }

    _mag_x = -rx;
    _mag_y =  ry;
    _mag_z = -rz;

    return true;
}


// accumulate a reading from the magnetometer
void Compass_HMC5883L::accumulate(void)
{
    if (!_initialised) {
        // someone has tried to enable a compass for the first time
        // mid-flight .... we can't do that yet (especially as we won't
        // have the right orientation!)
        return;
    }
   uint32_t tnow = micros();
   if (_healthy[0] && _accum_count != 0 && (tnow - _last_accum_time) < 13333) {
	  // the compass gets new data at 75Hz
	  return;
   }


   if (read_raw()) {
	  // the _mag_N values are in the range -2048 to 2047, so we can
	  // accumulate up to 15 of them in an int16_t. Let's make it 14
	  // for ease of calculation. We expect to do reads at 10Hz, and
	  // we get new data at most 75Hz, so we don't expect to
	  // accumulate more than 8 before a read
	  _mag_x_accum += _mag_x;
	  _mag_y_accum += _mag_y;
	  _mag_z_accum += _mag_z;
	  _accum_count++;
	  if (_accum_count == 14) {
		 _mag_x_accum /= 2;
		 _mag_y_accum /= 2;
		 _mag_z_accum /= 2;
		 _accum_count = 7;
	  }
	  _last_accum_time = tnow;
   }
}


/*
 *  re-initialise after a IO error
 */
bool Compass_HMC5883L::re_initialise()
{
    if (!write_register(ConfigRegA, _base_config) ||
        !write_register(ConfigRegB, magGain) ||
        !write_register(ModeRegister, ContinuousConversion))
        return false;
    return true;
}


// Public Methods //////////////////////////////////////////////////////////////
bool Compass_HMC5883L::init()
{
    int numAttempts = 0, good_count = 0;
    bool success = false;
    uint8_t calibration_gain = 0x20;
    uint16_t expected_x = 715;
    uint16_t expected_yz = 715;
    float gain_multiple = 1.0;

    delay(10);
    
    Wire.begin();

    // determine if we are using 5843 or 5883L
    _base_config = 0;
    if (!write_register(ConfigRegA, SampleAveraging_8<<5 | DataOutputRate_75HZ<<2 | NormalOperation) ||
        !read_register(ConfigRegA, &_base_config)) {
        _healthy[0] = false;
        return false;
    }
    if ( _base_config == (SampleAveraging_8<<5 | DataOutputRate_75HZ<<2 | NormalOperation)) {
        // a 5883L supports the sample averaging config
        product_id = COMPASS_TYPE_HMC5883L;
        calibration_gain = 0x60;
        /*
          note that the HMC5883 datasheet gives the x and y expected
          values as 766 and the z as 713. Experiments have shown the x
          axis is around 766, and the y and z closer to 713.
         */
        expected_x = 766;
        expected_yz  = 713;
        gain_multiple = 660.0 / 1090;  // adjustment for runtime vs calibration gain
    } else if (_base_config == (NormalOperation | DataOutputRate_75HZ<<2)) {
        product_id = COMPASS_TYPE_UNKNOWN;
    } else {
        // not behaving like either supported compass type
        return false;
    }

    calibration[0] = 0;
    calibration[1] = 0;
    calibration[2] = 0;

    while ( success == 0 && numAttempts < 25 && good_count < 5)
    {
        // record number of attempts at initialisation
        numAttempts++;

        // force positiveBias (compass should return 715 for all channels)
        if (!write_register(ConfigRegA, PositiveBiasConfig))
            continue;      // compass not responding on the bus
        delay(50);

        // set gains
        if (!write_register(ConfigRegB, calibration_gain) ||
            !write_register(ModeRegister, SingleConversion))
            continue;

        // read values from the compass
        delay(50);
        if (!read_raw())
            continue;      // we didn't read valid values

        delay(10);

        float cal[3];

        cal[0] = fabsf(expected_x / (float)_mag_x);
        cal[1] = fabsf(expected_yz / (float)_mag_y);
        cal[2] = fabsf(expected_yz / (float)_mag_z);

        // we throw away the first two samples as the compass may
        // still be changing its state from the application of the
        // strap excitation. After that we accept values in a
        // reasonable range
        if (numAttempts > 2 &&
            cal[0] > 0.7f && cal[0] < 1.35f &&
            cal[1] > 0.7f && cal[1] < 1.35f &&
            cal[2] > 0.7f && cal[2] < 1.35f) {
            // hal.console->printf_P(PSTR("cal=%.2f %.2f %.2f good\n"), cal[0], cal[1], cal[2]);
            good_count++;
            calibration[0] += cal[0];
            calibration[1] += cal[1];
            calibration[2] += cal[2];
        }
        /* useful for debugging */
        //hal.console->printf_P(PSTR("[o] HMC5883L[info]: MagneticX: %d MagneticY: %d MagneticZ: %d\n"), (int)_mag_x, (int)_mag_y, (int)_mag_z);
        //hal.console->printf_P(PSTR("[o] HMC5883L[info]: CalibrateX: %.2f CalibrateY: %.2f CalibrateZ: %.2f\n"), cal[0], cal[1], cal[2]);
    }

    if (good_count >= 5) {
        calibration[0] = calibration[0] * gain_multiple / good_count;
        calibration[1] = calibration[1] * gain_multiple / good_count;
        calibration[2] = calibration[2] * gain_multiple / good_count;
        success = true;
    } else {
        /* best guess */
        calibration[0] = 1.0;
        calibration[1] = 1.0;
        calibration[2] = 1.0;
    }

    // leave test mode
    if (!re_initialise()) {
        return false;
    }

    _initialised = true;

	// perform an initial read
	_healthy[0] = true;
	read();
    Serial.println(F("[o] HMC5883L[info]: Calibration Succesful"));
    Serial.print(F("[o] HMC5883L[info]: "));
    Serial.print(F("CalX: "));
    Serial.print( calibration[0] );
    Serial.print(F(" CalY: "));
    Serial.print( calibration[1] );
    Serial.print(F(" CalZ: "));
    Serial.print( calibration[2] );
    Serial.println();
    return success;
}

// Read Sensor data
bool Compass_HMC5883L::read()
{
    if (!_initialised) {
        // someone has tried to enable a compass for the first time
        // mid-flight .... we can't do that yet (especially as we won't
        // have the right orientation!)
        return false;
    }
    if (!_healthy[0]) {
        if (millis() < _retry_time) {
            return false;
        }
        if (!re_initialise()) {
            _retry_time = millis() + 1000;
            return false;
        }
    }

	if (_accum_count == 0) {
	   accumulate();
	   if (!_healthy[0] || _accum_count == 0) {
		  // try again in 1 second, and set I2c clock speed slower
		  _retry_time = millis() + 1000;
		  return false;
	   }
	}

	_field[0].x = _mag_x_accum * calibration[0] / _accum_count;
	_field[0].y = _mag_y_accum * calibration[1] / _accum_count;
	_field[0].z = _mag_z_accum * calibration[2] / _accum_count;
	_accum_count = 0;
	_mag_x_accum = _mag_y_accum = _mag_z_accum = 0;

    last_update = micros(); // record time of update

    // rotate to the desired orientation
    if (product_id == COMPASS_TYPE_HMC5883L) {
        _field[0].rotate(MAG_BOARD_ORIENTATION);
    }

    // apply default board orientation for this compass type. This is
    // a noop on most boards
    _field[0].rotate(MAG_BOARD_ORIENTATION);

    // add user selectable orientation
    _field[0].rotate((enum Rotation)_orientation.get());

    if (!_external) {
        // and add in AHRS_ORIENTATION setting if not an external compass
        _field[0].rotate(_board_orientation);
    }

    _field[0] += _offset[0].get();

    // apply motor compensation
    if(_motor_comp_type != COMPASS_MOT_COMP_DISABLED && _thr_or_curr != 0.0f) {
        _motor_offset[0] = _motor_compensation[0].get() * _thr_or_curr;
        _field[0] += _motor_offset[0];
    }else{
        _motor_offset[0].zero();
    }

    _healthy[0] = true;

    return true;
}
