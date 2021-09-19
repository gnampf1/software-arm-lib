#ifndef _SPI_H_INCLUDED
#define _SPI_H_INCLUDED

#include <Arduino.h>
#include <sblib/spi.h>

enum ArduinoSpiMode {
	SPI_MODE0,
	SPI_MODE1,
	SPI_MODE2,
	SPI_MODE3
};

enum ArduinoSpiClockDivider {
	SPI_CLOCK_DIV1 = 1,
	SPI_CLOCK_DIV2 = 2,
	SPI_CLOCK_DIV4 = 4,
	SPI_CLOCK_DIV8 = 8,
	SPI_CLOCK_DIV16 = 256,
};

class SPIClass : public SPI
{
public:
    SPIClass(int spiPort, ArduinoSpiMode mode = SPI_MODE0) : SPI(spiPort, SPI_DEFAULT_MODE)
	{
    	setDataMode(mode);
    	setDataSize(SPI_DATA_8BIT);
    }

    inline void setDataMode(ArduinoSpiMode dataMode)
    {
	}

    inline uint8_t transfer(uint8_t data)
    {
    	return (uint8_t)SPI::transfer(data, SPI_CONTINUE);
    }

    inline void setBitOrder(uint8_t bitOrder)
	{
	}
protected:
};

extern SPIClass SPI;
#endif
