#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <inttypes.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "config.h"
#include "i2c_master.h"
#include "lcd_interface.h"

void lcdtry_run()
{

  uint8_t i, j;
  
  // enable TWI pullups
  TWI_PORT |= _BV(TWI_PIN_SDA);
  TWI_PORT |= _BV(TWI_PIN_SCL);

  lcd_init(LCD_I2C_ADDR);

  lcd_write('A');
  lcd_print("Hello World!");
  lcd_print("Hello World!");

  _delay_ms(3000);
  
  i = 0;
  
  while(1) {
    lcd_setCursor(0, 0);

    for (j = 0; j < 64; j++) {
      lcd_write(i++);
    }
    _delay_ms(2000);
  }
}
