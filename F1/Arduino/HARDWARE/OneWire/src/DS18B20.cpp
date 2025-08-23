//
//    FILE: DS18B20.cpp
//  AUTHOR: Rob.Tillaart@gmail.com
// VERSION: 0.1.0
//    DATE: 2017-07-25
//
// PUPROSE: library for DS18B20 temperature sensor with minimal footprint
//
// HISTORY:
// 0.1.0 = 2017-07-25 initial version

#include "Arduino.h"
#include "DS18B20.h"

// OneWire commands
#define STARTCONVO      0x44  // Tells device to take a temperature reading and put it on the scratchpad
#define COPYSCRATCH     0x48  // Copy EEPROM
#define READSCRATCH     0xBE  // Read EEPROM
#define WRITESCRATCH    0x4E  // Write to EEPROM
#define RECALLSCRATCH   0xB8  // Reload from last known
#define READPOWERSUPPLY 0xB4  // Determine if device needs parasite power
#define ALARMSEARCH     0xEC  // Query bus for devices with an alarm condition

// Scratchpad locations
#define TEMP_LSB        0
#define TEMP_MSB        1
// #define HIGH_ALARM_TEMP 2
// #define LOW_ALARM_TEMP  3
// #define CONFIGURATION   4
// #define INTERNAL_BYTE   5
// #define COUNT_REMAIN    6
// #define COUNT_PER_C     7
// #define SCRATCHPAD_CRC  8

// Device resolution
#define TEMP_9_BIT  0x1F //  9 bit
#define TEMP_10_BIT 0x3F // 10 bit
#define TEMP_11_BIT 0x5F // 11 bit
#define TEMP_12_BIT 0x7F // 12 bit

DeviceAddress deviceAddress;
boolean parasite= false;
uint8_t devices = 0;

DS18B20::DS18B20(OneWire* _oneWire)
{
  _wire = _oneWire;
}

bool DS18B20::begin(void)
{
   

    _wire->reset_search();
    devices = 0; // Reset the number of devices when we enumerate wire devices
		
    while (_wire->search(deviceAddress)){

        if (validAddress(deviceAddress)){

        if (!parasite && readPowerSupply(deviceAddress))
				parasite = true;			
				
				devices++;
        }
    }
		return (_wire->crc8(deviceAddress, 7) == deviceAddress[7]);
}

// returns the number of devices found on the bus
uint8_t DS18B20::getDeviceCount(void){
    return devices;
}
// finds an address at a given index on the bus
// returns true if the device was found
bool DS18B20::getAddress(uint8_t* deviceAddress, uint8_t index) {

	uint8_t depth = 0;

	_wire->reset_search();

	while (depth <= index && _wire->search(deviceAddress)) {
		if (depth == index && validAddress(deviceAddress))
			return true;
		depth++;
	}

	return false;

}
// returns true if address is valid
bool DS18B20::validAddress(const uint8_t* deviceAddress){
    return (_wire->crc8(deviceAddress, 7) == deviceAddress[7]);
}

bool DS18B20::readPowerSupply(const uint8_t* deviceAddress) {

	bool ret = false;
	_wire->reset();
	_wire->select(deviceAddress);
	_wire->write(READPOWERSUPPLY,0);
	if (_wire->read_bit() == 0)
		ret = true;
	_wire->reset();
	return ret;

}


void DS18B20::readScratchPad(uint8_t *scratchPad, uint8_t fields)
{
  _wire->reset();
  _wire->select(deviceAddress);
  _wire->write(READSCRATCH);

  for(uint8_t i=0; i < fields; i++)
  {
    scratchPad[i] = _wire->read();
  }
  _wire->reset();
	
}

bool DS18B20::isConversionComplete(void)
{
  return (_wire->read_bit() == 1);
}	

void DS18B20::requestTemperatures(void)
{
  _wire->reset();
  _wire->skip();
  _wire->write(STARTCONVO, 0);
}

float DS18B20::getTempC(void)
{
  ScratchPad scratchPad;
	 int16_t rawTemperature = 0;

  float temp = 0;
	
  readScratchPad(scratchPad, 2);//doc 2 byte
	
	rawTemperature = (((int16_t)scratchPad[TEMP_MSB]) << 8) | scratchPad[TEMP_LSB];
	temp = 0.0625 * rawTemperature;

  if (temp < -55) return DEVICE_DISCONNECTED;
  return temp;
}

bool DS18B20::sendCommand(uint8_t *address, uint8_t command)
{
  if (!_wire->reset())
    return false;

  _wire->select(address);
  _wire->write(command,0);

  return true;
}


float DS18B20::readTemperature(uint8_t *address)
{
	
	// Read all registers in a simple loop
	// byte 0: temperature LSB
	// byte 1: temperature MSB
	// byte 2: high alarm temp
	// byte 3: low alarm temp
	// byte 4: DS18S20: store for crc
	//         DS18B20 & DS1822: configuration register
	// byte 5: internal use & crc
	// byte 6: DS18S20: COUNT_REMAIN
	//         DS18B20 & DS1822: store for crc
	// byte 7: DS18S20: COUNT_PER_C
	//         DS18B20 & DS1822: store for crc
	// byte 8: SCRATCHPAD_CRC
	
  uint8_t scratchpad[9];
	int16_t raw =0;
	float quality[] = {0.5, 0.25, 0.125, 0.0625};
  uint8_t shift[] = {3, 2, 1, 0};

  if (!sendCommand(address, 0xbe))
    return -127;

  _wire->read_bytes(scratchpad, 9);

  if (_wire->crc8(scratchpad, 8) != scratchpad[8])
    return -127;

  
		raw = word(scratchpad[1], scratchpad[0]);
  raw >>= shift[12-9];//_quality=12

  return raw * quality[12-9];//_quality=12
}


void DS18B20::setResolution(uint8_t newResolution)
{
  _wire->reset();
  _wire->select(deviceAddress);
  _wire->write(WRITESCRATCH);
  // two dummy values for LOW & HIGH ALARM
  _wire->write(0);
  _wire->write(100);
  switch (newResolution)
  {
  case 12:
    _wire->write(TEMP_12_BIT);
    break;
  case 11:
    _wire->write(TEMP_11_BIT);
    break;
  case 10:
    _wire->write(TEMP_10_BIT);
    break;
  case 9:
  default:
    _wire->write(TEMP_9_BIT);
    break;
  }
  _wire->reset();
}

//  END OF FILE