int eeprom_write_disable();
int eeprom_read(char *s, const unsigned offset, const unsigned count);
int eeprom_write(const char *s, const unsigned offset, const unsigned count);

// EEPROM usage
#define EEP_OFFSET	0
#define	EEP_ADCCAL	16
#define	EEP_BOARD	32
#define MPU9150_CAL	128

