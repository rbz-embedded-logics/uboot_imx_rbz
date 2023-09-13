#ifndef __EEPROM_HW_H_
#define __EEPROM_HW_H_

int board_read_rom_eeprom(unsigned char *config);
long get_ramsize(unsigned char config);

#endif
