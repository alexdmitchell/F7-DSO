//spi.h file
#ifndef _spi_h
#define _spi_h

void dac_set_level(int level);
void dac_increment_level(void);
void dac_decrement_level(void);
void pga_control(int gain);
void pga_increment_gain(void);
void pga_decrement_gain(void);
#endif
