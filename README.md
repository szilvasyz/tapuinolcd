# tapuinolcd
 Minimal config to develop new display drivers to tapuino

Added display:
* 128x64 GLCD, ST7920 driven through PCF8574 I2C backpack (4-bit parallel) 
* 128x32/128x64 OLED with SH1106 (capable to init also SSD1306)

Added features: 
* true double height characters on graphic displays

Added fonts: 
* full ISO8859-1/2 character tables for graphic displays
* ISO8859-1 character mapping for HD44780-A00 (original, Japanese ROM)
