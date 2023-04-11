#include "config.h"
#ifdef LCD_USE_ILI9341_TFT_MODULE

#include <string.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include "integer.h"
#include "i2c_master.h"
#include "spi.h"

#include "orig8x8-h-iso8859-2.h"
//#include "orig8x8-iso8859-1.h"
//#include "font8x8.h"


// ILI9340 specific
#define GAMSET      0x26    /* Gamma Set */
#define FRMCTR1     0xB1    /* Frame Rate Control (In Normal Mode / Full colors) */
#define FRMCTR2     0xB2    /* Frame Rate Control (In Idle Mode / 8 colors) */
#define FRMCTR3     0xB3    /* Frame Rate Control (In Partial Mode / Full colors) */
#define DISCTRL     0xB6    /* Display Function Control */
#define PWCTRL1     0xC0    /* Power Control 1 */
#define PWCTRL2     0xC1    /* Power Control 2 */
#define VMCTRL1     0xC5    /* VCOM Control 1 */
#define VMCTRL2     0xC7    /* VCOM Control 2 */
#define PWCTRLA     0xCB    /* Power control A */
#define PWCTRLB     0xCF    /* Power control B */
#define PGAMCTRL    0xE0    /* Positive Gamma Control */
#define NGAMCTRL    0xE1    /* Negative Gamma Control */
#define EN3GCTRL    0xF2    /* Enable 3 Gamma Control */
#define DRVTCA      0xE8    /* Driver timing control A */
#define DRVTCB      0xEA    /* Driver timing control B */
#define PWONSEQC    0xED    /* Power on sequence control */
#define PUMPRC      0xF7    /* Pump ratio control */

// driver specific
#define SWRESET     0x01    /* Software Reset */
#define BSTRON      0x03
#define RDDIDIF     0x04    /* Read Display Identification Information */
#define RDDST     0x09    /* Read Display Status */
#define SLEEPIN     0x10    /* Enter Sleep Mode */
#define SLEEPOUT    0x11    /* Sleep Out */
#define NORON     0x13    /* Normal Display Mode On */
#define INVOFF      0x20    /* Display Inversion OFF */
#define INVON     0x21    /* Display Inversion ON */
#define DISPOFF     0x28    /* Display OFF */
#define DISPON      0x29    /* Display ON */
#define CASET     0x2A    /* Column Address Set */
#define PASET     0x2B    /* Page Address Set */
#define RAMWR     0x2C    /* Memory Write */
#define RGBSET        0x2D    /* Color Set */
#define MADCTL      0x36    /* Memory Access Control */
#define VSCRSADD    0x37    /* Vertical Scrolling Start Address */
#define PIXSET      0x3A    /* Pixel Format Set */
#define BLCTRL1       0xB8    /* Backlight Control 1 */
#define BLCTRL2       0xB9    /* Backlight Control 2 */
#define BLCTRL3       0xBA    /* Backlight Control 3 */

const uint8_t initseq[] PROGMEM = {
  /* Power control A, Vcore=1.6V DDVDH=5.6V */
  PWCTRLA, 5, 0x39, 0x2C, 0x00, 0x34, 0x02,
  /* Power control B, Discharge path enable for ESD protection */
  PWCTRLB, 3, 0x00, 0xC1, 0x30,
  /* Driver Timing Control A */
  DRVTCA, 3, 0x85, 0x00, 0x78,
  /* Driver Timing Control B */
  DRVTCB, 2, 0x00, 0x00,
  /* Power on sequence control */
  PWONSEQC, 4, 0x64, 0x03, 0x12, 0x81,
  
  /* Pump ratio control */
  PUMPRC, 1, 0x20,
  /* Power Control 1, GVDD=4.60V */
  PWCTRL1, 1, 0x23,
  /* Power Control 2 */
  PWCTRL2, 1, 0x10,
  /* VCOM Control 1, VCOMH=4.250V, VCOML=-1.500V */
  VMCTRL1, 2, 0x3e, 0x28,
  /* VCOM Control 2, Set the VCOM offset voltage */
  VMCTRL2, 1, 0x86,

  /* Orientation 0xE8=horizontal, 0x28:rotated horizontal*/
  MADCTL, 1, 0xE8,

  /* Pixel Format Set, RGB and MCU Interface Format = 16 bits/pixel */
  PIXSET, 1, 0x55,
  /* Frame Rate Control, = 79Hz */
  FRMCTR1, 2, 0x00, 0x18,

  /* Display Function Control */
  DISCTRL, 3, 0x08, 0x82, 0x27,

  /* Enable 3 gamma control = not enabled */
  EN3GCTRL, 1, 0x00,
  /* Gamma Set */
  GAMSET, 1, 0x01,

  /* Positive Gamma Control */
  PGAMCTRL, 15, 0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08, 0x4E, 0xF1, 0x37, 0x07, 0x10, 0x03, 0x0E, 0x09, 0x00,
  /* Negative Gamma Control */
  NGAMCTRL, 15, 0x00, 0x0E, 0x14, 0x03, 0x11, 0x07, 0x31, 0xC1, 0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36, 0x0F,

  /* END */
  0};




// 8 - SCLK   PB7
// 7 - MISO   PB6
// 6 - MOSI   PB5
// 5 - SDCS   PB4
// 20 - DPDC  PD6
// 19 - DPRST PD5
// 18 - DPCS  PD4
#define TFT_DC_PORT         PORTD
#define TFT_DC_DDR          DDRD
#define TFT_DC_PIN          6

#define TFT_CS_PORT         PORTD
#define TFT_CS_DDR          DDRD
#define TFT_CS_PIN          4

#define TFT_RST_PORT        PORTD
#define TFT_RST_DDR         DDRD
#define TFT_RST_PIN         5

#define LCD_W 320
#define LCD_H 240

#define BGCOLOR (0b0111100111100000)
#define FGCOLOR (0b0011101111111111)

#define NUMCOLS (LCD_W / 16)
#define NUMROWS (LCD_H / 32)


uint8_t _row, _col;

void lcd_init(uint8_t lcd_addr);
void lcd_cursor();
void lcd_noCursor();
void lcd_backlight();
void lcd_noBacklight();
void lcd_clear(); 
void lcd_setCursor(uint8_t col, uint8_t row); 
void lcd_print(char* msg);
void lcd_write(uint8_t value);

void ili9341_writecommand8(uint8_t com);
void ili9341_writedata8(uint8_t data);
void ili9341_setaddress(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2);
void ili9341_pushcolour(uint16_t colour);
void ili9341_clear(uint16_t colour);


void ili9341_writecommand8(uint8_t com)   //command write
{
  TFT_DC_PORT &= ~_BV(TFT_DC_PIN);
  TFT_CS_PORT &= ~_BV(TFT_CS_PIN);   //dc and cs both low to send command
//  _delay_us(5);//little delay
  SPI_Send(com);
  TFT_CS_PORT |= _BV(TFT_CS_PIN);    //pull high cs
}

void ili9341_writedata8(uint8_t data)   //data write
{
  TFT_DC_PORT |= _BV(TFT_DC_PIN);    //set dc high for data
//  _delay_us(1);//delay
  TFT_CS_PORT &= ~_BV(TFT_CS_PIN);   //set cs low for operation
  SPI_Send(data);
  TFT_CS_PORT |= _BV(TFT_CS_PIN);    //pull high cs
}

void lcd_init(uint8_t lcd_addr) {
  int p, i;
  uint8_t d;

  TFT_DC_DDR  |= _BV(TFT_DC_PIN);
  TFT_DC_PORT |= _BV(TFT_DC_PIN);

  TFT_CS_DDR  |= _BV(TFT_CS_PIN);
  TFT_CS_PORT |= _BV(TFT_CS_PIN);

  TFT_RST_DDR  |= _BV(TFT_RST_PIN);
  TFT_RST_PORT |= _BV(TFT_RST_PIN);

  SPI_Init();
  SPI_Speed_Fast();

  TFT_RST_PORT |= _BV(TFT_RST_PIN);   //pull high if low previously
  _delay_ms(200);
  TFT_RST_PORT &= ~_BV(TFT_RST_PIN);  //low for reset
  _delay_ms(200);
  TFT_RST_PORT |= _BV(TFT_RST_PIN);   //again pull high for normal operation

  ili9341_writecommand8(0x01);//soft reset
  _delay_ms(1000);

  p = 0;

  while (d = pgm_read_byte(&initseq[p++])) {
    ili9341_writecommand8(d);
    i = pgm_read_byte(&initseq[p++]);
    while (i--) {     
      d = pgm_read_byte(&initseq[p++]);
      ili9341_writedata8(d);
    }
  }

  //exit sleep
  ili9341_writecommand8(SLEEPOUT);
  _delay_ms(120);
  //display on
  ili9341_writecommand8(DISPON);

  ili9341_clear(0b0111100111100000);

}

void ili9341_setaddress(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2)//set coordinate for print or other function
{
  ili9341_writecommand8(0x2A);
  ili9341_writedata8(x1>>8);
  ili9341_writedata8(x1);
  ili9341_writedata8(x2>>8);
  ili9341_writedata8(x2);

  ili9341_writecommand8(0x2B);
  ili9341_writedata8(y1>>8);
  ili9341_writedata8(y1);
  ili9341_writedata8(y2);
  ili9341_writedata8(y2);

  ili9341_writecommand8(0x2C);//meory write
}

//set colour for drawing
void ili9341_pushcolour(uint16_t colour)
{
  ili9341_writedata8(colour>>8);
  ili9341_writedata8(colour);
}

//clear lcd and fill with colour
void ili9341_clear(uint16_t colour)
{
  uint16_t i,j;
  ili9341_setaddress(0,0,LCD_W-1,LCD_H-1);

  TFT_DC_PORT |= _BV(TFT_DC_PIN);    //set dc high for data
  TFT_CS_PORT &= ~_BV(TFT_CS_PIN);   //set cs low for operation

  for(i=0;i<LCD_W;i++)
  {
    for(j=0;j<LCD_H;j++)
    {
      SPI_Send(colour>>8);
      SPI_Send(colour);
      //ili9341_pushcolour(colour);
    }
  }
  
  TFT_CS_PORT |= _BV(TFT_CS_PIN);    //pull high cs
}

void lcd_cursor(){}
void lcd_noCursor(){}
void lcd_backlight(){}
void lcd_noBacklight(){}


void lcd_clear(){
  _row = 0;
  _col = 0;
  ili9341_clear(BGCOLOR);
} 


void lcd_setCursor(uint8_t col, uint8_t row){
  _row = 0;
  _col = 0;

}
 

void lcd_print(char* msg){
  while(*msg)
    lcd_write(*msg++);
}


void lcd_write(uint8_t value){
  int col, row;
  uint8_t * p;
  uint8_t i, j, k, d;
  uint16_t c;
  uint8_t b1[8], b2[8];
  
  col = _col * 16;
  row = _row * 32;

  if (++_col >= NUMCOLS) {
    _col = 0;
    if (++_row >= NUMROWS)
      _row = 0;
  }

  value = value < 32 ? 0 : value - 32;
  
  p = &console_font[value * 8];
  ili9341_setaddress(col, row, col + 15, row + 31);
  TFT_DC_PORT |= _BV(TFT_DC_PIN);    //set dc high for data
  TFT_CS_PORT &= ~_BV(TFT_CS_PIN);   //set cs low for operation

  for (i = 0; i < 8; i++) {
    d = pgm_read_byte(p++);
    for (j = 0; j < 8; j++) {
      c = d & 0x80 ? FGCOLOR : BGCOLOR;
      b1[j] = c >> 8;
      b2[j] = c;
      d <<= 1;
    }
    for (k = 0; k < 4; k++) {
      for (j = 0; j < 8; j++) {
        SPI_Send(b1[j]); SPI_Send(b2[j]);
        SPI_Send(b1[j]); SPI_Send(b2[j]);
      }
    }
  }
  
  TFT_CS_PORT |= _BV(TFT_CS_PIN);    //pull high cs
}


#endif
