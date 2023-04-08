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

  // keys
  KEYS_READ_DDR &= ~_BV(KEY_SELECT_PIN);
  KEYS_READ_DDR &= ~_BV(KEY_ABORT_PIN);
  KEYS_READ_DDR &= ~_BV(KEY_PREV_PIN);
  KEYS_READ_DDR &= ~_BV(KEY_NEXT_PIN);
  
  KEYS_READ_PORT |= _BV(KEY_SELECT_PIN);
  KEYS_READ_PORT |= _BV(KEY_ABORT_PIN);
  KEYS_READ_PORT |= _BV(KEY_PREV_PIN);
  KEYS_READ_PORT |= _BV(KEY_NEXT_PIN);

  // enable TWI pullups
  TWI_PORT |= _BV(TWI_PIN_SDA);
  TWI_PORT |= _BV(TWI_PIN_SCL);

  lcd_init(LCD_I2C_ADDR);
//  lcd_setCursor(0, 0);

//  lcd_write('A');
  lcd_backlight();
  lcd_print("\xC1RV\xCDZT\xDBR\xD5 T\xDCK\xD6RF\xDAR\xD3G\xC9P,");
  _delay_ms(500);
  while (KEYS_READ_PINS & _BV(KEY_SELECT_PIN));

  lcd_clear();
  lcd_print("\xE1rv\xEDzt\xFBr\xF5 t\xFCk\xF6rf\xFAr\xF3g\xE9p");
  _delay_ms(500);
  while (KEYS_READ_PINS & _BV(KEY_SELECT_PIN));

  lcd_clear();
  lcd_print("A\xC1I\xCDU\xDBO\xD5U\xDCO\xD6U\xDAO\xD3""E\xC9");
  _delay_ms(500);
  while (KEYS_READ_PINS & _BV(KEY_SELECT_PIN));

  lcd_clear();
  lcd_print("a\xE1i\xEDu\xFBo\xF5u\xFCo\xF6u\xFAo\xF3""e\xE9");
  _delay_ms(500);
  while (KEYS_READ_PINS & _BV(KEY_SELECT_PIN));

//  lcd_print("Hello World!");
//  lcd_print("Hello World!");
 
//  _delay_ms(10000);
  
  i = 0;
  
  while(1) {
    lcd_setCursor(0, 0);

    for (j = 0; j < 32; j++) {
      lcd_write(i++);
    }

    lcd_noBacklight();
    _delay_ms(1000);
    lcd_backlight();
    _delay_ms(500);

    while (KEYS_READ_PINS & _BV(KEY_SELECT_PIN));
  }
}
