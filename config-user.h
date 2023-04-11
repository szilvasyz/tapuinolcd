/********************************************************************************/
// User selectable configuration settings
//
/********************************************************************************/

/********************************************************************************/
// LCD Definitions
/********************************************************************************/

// uncomment one of these sets for your specific LCD

// #define LCD_USE_1602_LCD_MODULE
// #define LCD_I2C_ADDR        0x27 // I2C address for the LCD

// #define LCD_USE_SSD1306_OLED_MODULE
// #define LCD_I2C_ADDR        0x3C // I2C address for the OLED
// #define LCD_BIG_FONTS            // define this for ... bigger fonts...
// choose one of these depending on your display
// #define LCD_SSD1306_128x64
// #define LCD_SSD1306_128x32


// #define LCD_USE_SSD131X_OLED_MODULE
// #define LCD_I2C_ADDR        0x3C // I2C address for the OLED

// #define LCD_USE_ST7920_LCD_MODULE
// #define LCD_I2C_ADDR        0x27 // I2C address for the LCD
// #define LCD_BIG_FONTS            // define this for ... bigger fonts...

// #define LCD_USE_SH1106_OLED_MODULE
// #define LCD_I2C_ADDR        0x3C // I2C address for the OLED
// #define LCD_BIG_FONTS            // define this for double height fonts...
// #define LCD_HUGE_FONTS           // define this for quadruple height fonts...
// choose one of these depending on your display
// #define LCD_SH1106_128x64
// #define LCD_SH1106_128x32
// #define LCD_SH1106_COLOFFSET 2    // column offset, SH1106 has 132 columns
// #define LCD_SH1106_UPSIDEDOWN    // rotate with 180 degrees
// #define LCD_SH1106_SSD1306INIT   // send additional init commands required by SSD1306

#define LCD_USE_ILI9341_TFT_MODULE
#define LCD_I2C_ADDR        0x27 // dummy address for init - not used


/********************************************************************************/
// Language Definitions
/********************************************************************************/

// uncomment one of these for your language

// #define TAPUINO_LANGUAGE_EN
// #define TAPUINO_LANGUAGE_IT
// #define TAPUINO_LANGUAGE_TR
// #define TAPUINO_LANGUAGE_ES
// #define TAPUINO_LANGUAGE_DE
// #define TAPUINO_LANGUAGE_HU
 #define TAPUINO_LANGUAGE_HU_ANSI
