#include <string.h>
#include "ms5611.hpp"
#include "bcm2835.hpp"

static void eeprom_write_enable()
{
	bcm2835_gpio_fsel(CALIBRATE, BCM2835_GPIO_FSEL_OUTP);
	bcm2835_gpio_write(CALIBRATE, LOW);
}

void eeprom_write_disable()
{
	bcm2835_gpio_fsel(CALIBRATE, BCM2835_GPIO_FSEL_INPT);
	bcm2835_gpio_set_pud(CALIBRATE, BCM2835_GPIO_PUD_UP);
}

bool eeprom_read(char *s, const unsigned offset, const unsigned count)
{
	bcm2835_gpio_write(I2C_SEL, HIGH);
	bcm2835_i2c_setSlaveAddress(0x50 + (offset>>8));

	char cmd = (char)offset;
	bcm2835_i2c_write(&cmd, 1);
	return bcm2835_i2c_read(s, count);
}

bool eeprom_write(const char *s, const unsigned offset, const unsigned count)
{
	if (offset & 0xf) printf("offset should by 16 byte aligned\n");
	if (count > 16) printf("max count is 16\n");

        char d[17];
        memcpy(&d[1], s, count);
	d[0] = (char)offset;

	bcm2835_gpio_write(I2C_SEL, HIGH);
	eeprom_write_enable();
	bcm2835_i2c_setSlaveAddress(0x50 + (offset>>8));
	bcm2835_i2c_write(d, count+1);
	bcm2835_delay(10);
	eeprom_write_disable();
	return false;
}

