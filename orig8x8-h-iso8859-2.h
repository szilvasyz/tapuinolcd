#define LCD_FONT_HEIGHT (8)
#define LCD_CHAR_TO_POS(x) (x < 32 ? 0 : x - 32)

const unsigned char console_font[] PROGMEM = {

// generated character bitmaps

// code 32, 0x20 - generated blank
0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,
// code 33, 0x21 - not mapped
0b00000000, 0b00011000, 0b00011000, 0b00011000, 0b00011000, 0b00011000, 0b00000000, 0b00011000,
// code 34, 0x22 - not mapped
0b00000000, 0b01101100, 0b01101100, 0b00100100, 0b00000000, 0b00000000, 0b00000000, 0b00000000,
// code 35, 0x23 - not mapped
0b00000000, 0b00000000, 0b01101100, 0b11111110, 0b01101100, 0b11111110, 0b01101100, 0b00000000,
// code 36, 0x24 - not mapped
0b00000000, 0b00011000, 0b01111110, 0b11011000, 0b01111100, 0b00110110, 0b11111100, 0b00110000,
// code 37, 0x25 - not mapped
0b00000000, 0b01100010, 0b01100110, 0b00001100, 0b00011000, 0b00110000, 0b01100110, 0b01000110,
// code 38, 0x26 - not mapped
0b00000000, 0b01110000, 0b11011000, 0b11011000, 0b01110010, 0b11011100, 0b11001110, 0b01111010,
// code 39, 0x27 - not mapped
0b00000000, 0b00011000, 0b00011000, 0b00001000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,
// code 40, 0x28 - not mapped
0b00000000, 0b00011000, 0b00110000, 0b00110000, 0b00110000, 0b00110000, 0b00110000, 0b00011000,
// code 41, 0x29 - not mapped
0b00000000, 0b00110000, 0b00011000, 0b00011000, 0b00011000, 0b00011000, 0b00011000, 0b00110000,
// code 42, 0x2A - not mapped
0b00000000, 0b00010000, 0b11010110, 0b01111100, 0b00111000, 0b01111100, 0b11010110, 0b00010000,
// code 43, 0x2B - not mapped
0b00000000, 0b00000000, 0b00011000, 0b00011000, 0b01111110, 0b00011000, 0b00011000, 0b00000000,
// code 44, 0x2C - not mapped
0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00011000, 0b00011000, 0b00110000,
// code 45, 0x2D - not mapped
0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b01111110, 0b00000000, 0b00000000, 0b00000000,
// code 46, 0x2E - not mapped
0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00011000, 0b00011000,
// code 47, 0x2F - not mapped
0b00000000, 0b00000100, 0b00001100, 0b00011000, 0b00110000, 0b01100000, 0b11000000, 0b10000000,
// code 48, 0x30 - not mapped
0b00000000, 0b01111100, 0b11000110, 0b11001110, 0b11010110, 0b11100110, 0b11000110, 0b01111100,
// code 49, 0x31 - not mapped
0b00000000, 0b00011000, 0b01111000, 0b00011000, 0b00011000, 0b00011000, 0b00011000, 0b01111110,
// code 50, 0x32 - not mapped
0b00000000, 0b01111100, 0b11000110, 0b00000110, 0b00011100, 0b01110000, 0b11000000, 0b11111110,
// code 51, 0x33 - not mapped
0b00000000, 0b01111100, 0b11000110, 0b00000110, 0b00011100, 0b00000110, 0b11000110, 0b01111100,
// code 52, 0x34 - not mapped
0b00000000, 0b00011000, 0b00110000, 0b01100110, 0b11000110, 0b11111110, 0b00000110, 0b00000110,
// code 53, 0x35 - not mapped
0b00000000, 0b11111110, 0b11000000, 0b11111100, 0b00000110, 0b00000110, 0b11000110, 0b01111100,
// code 54, 0x36 - not mapped
0b00000000, 0b00111100, 0b01100000, 0b11000000, 0b11111100, 0b11000110, 0b11000110, 0b01111100,
// code 55, 0x37 - not mapped
0b00000000, 0b11111110, 0b00000110, 0b00001100, 0b00011000, 0b00110000, 0b00110000, 0b00110000,
// code 56, 0x38 - not mapped
0b00000000, 0b01111100, 0b11000110, 0b11000110, 0b01111100, 0b11000110, 0b11000110, 0b01111100,
// code 57, 0x39 - not mapped
0b00000000, 0b01111100, 0b11000110, 0b11000110, 0b01111110, 0b00000110, 0b00000110, 0b01111100,
// code 58, 0x3A - not mapped
0b00000000, 0b00000000, 0b00011000, 0b00011000, 0b00000000, 0b00011000, 0b00011000, 0b00000000,
// code 59, 0x3B - not mapped
0b00000000, 0b00000000, 0b00011000, 0b00011000, 0b00000000, 0b00011000, 0b00011000, 0b00110000,
// code 60, 0x3C - not mapped
0b00000000, 0b00001100, 0b00011000, 0b00110000, 0b01100000, 0b00110000, 0b00011000, 0b00001100,
// code 61, 0x3D - not mapped
0b00000000, 0b00000000, 0b00000000, 0b01111110, 0b00000000, 0b01111110, 0b00000000, 0b00000000,
// code 62, 0x3E - not mapped
0b00000000, 0b01100000, 0b00110000, 0b00011000, 0b00001100, 0b00011000, 0b00110000, 0b01100000,
// code 63, 0x3F - not mapped
0b00000000, 0b01111100, 0b11000110, 0b00001100, 0b00011000, 0b00110000, 0b00000000, 0b00110000,
// code 64, 0x40 - not mapped
0b00000000, 0b01111100, 0b11000110, 0b11011110, 0b11010110, 0b11011100, 0b11000000, 0b01111100,
// code 65, 0x41 - not mapped
0b00000000, 0b01111100, 0b11000110, 0b11000110, 0b11111110, 0b11000110, 0b11000110, 0b11000110,
// code 66, 0x42 - not mapped
0b00000000, 0b11111100, 0b11000110, 0b11000110, 0b11111100, 0b11000110, 0b11000110, 0b11111100,
// code 67, 0x43 - not mapped
0b00000000, 0b01111100, 0b11000110, 0b11000000, 0b11000000, 0b11000000, 0b11000110, 0b01111100,
// code 68, 0x44 - not mapped
0b00000000, 0b11111000, 0b11001100, 0b11000110, 0b11000110, 0b11000110, 0b11001100, 0b11111000,
// code 69, 0x45 - not mapped
0b00000000, 0b11111110, 0b11000000, 0b11000000, 0b11111000, 0b11000000, 0b11000000, 0b11111110,
// code 70, 0x46 - not mapped
0b00000000, 0b11111110, 0b11000000, 0b11000000, 0b11111000, 0b11000000, 0b11000000, 0b11000000,
// code 71, 0x47 - not mapped
0b00000000, 0b01111100, 0b11000110, 0b11000000, 0b11011110, 0b11000110, 0b11000110, 0b01111110,
// code 72, 0x48 - not mapped
0b00000000, 0b11000110, 0b11000110, 0b11000110, 0b11111110, 0b11000110, 0b11000110, 0b11000110,
// code 73, 0x49 - not mapped
0b00000000, 0b01111110, 0b00011000, 0b00011000, 0b00011000, 0b00011000, 0b00011000, 0b01111110,
// code 74, 0x4A - not mapped
0b00000000, 0b01111110, 0b00011000, 0b00011000, 0b00011000, 0b00011000, 0b11011000, 0b01110000,
// code 75, 0x4B - not mapped
0b00000000, 0b11000110, 0b11001100, 0b11011000, 0b11110000, 0b11011000, 0b11001100, 0b11000110,
// code 76, 0x4C - not mapped
0b00000000, 0b11000000, 0b11000000, 0b11000000, 0b11000000, 0b11000000, 0b11000000, 0b11111110,
// code 77, 0x4D - not mapped
0b00000000, 0b11000110, 0b11101110, 0b11111110, 0b11010110, 0b11000110, 0b11000110, 0b11000110,
// code 78, 0x4E - not mapped
0b00000000, 0b11000110, 0b11100110, 0b11110110, 0b11011110, 0b11001110, 0b11000110, 0b11000110,
// code 79, 0x4F - not mapped
0b00000000, 0b01111100, 0b11000110, 0b11000110, 0b11000110, 0b11000110, 0b11000110, 0b01111100,
// code 80, 0x50 - not mapped
0b00000000, 0b11111100, 0b11000110, 0b11000110, 0b11111100, 0b11000000, 0b11000000, 0b11000000,
// code 81, 0x51 - not mapped
0b00000000, 0b01111100, 0b11000110, 0b11000110, 0b11000110, 0b11011110, 0b01111100, 0b00000110,
// code 82, 0x52 - not mapped
0b00000000, 0b11111100, 0b11000110, 0b11000110, 0b11111100, 0b11000110, 0b11000110, 0b11000110,
// code 83, 0x53 - not mapped
0b00000000, 0b01111100, 0b11000110, 0b11000000, 0b01111100, 0b00000110, 0b11000110, 0b01111100,
// code 84, 0x54 - not mapped
0b00000000, 0b01111110, 0b00011000, 0b00011000, 0b00011000, 0b00011000, 0b00011000, 0b00011000,
// code 85, 0x55 - not mapped
0b00000000, 0b11000110, 0b11000110, 0b11000110, 0b11000110, 0b11000110, 0b11000110, 0b01111110,
// code 86, 0x56 - not mapped
0b00000000, 0b11000110, 0b11000110, 0b11000110, 0b01101100, 0b01101100, 0b00111000, 0b00010000,
// code 87, 0x57 - not mapped
0b00000000, 0b11000110, 0b11000110, 0b11000110, 0b11010110, 0b11111110, 0b11101110, 0b11000110,
// code 88, 0x58 - not mapped
0b00000000, 0b11000110, 0b11000110, 0b01101100, 0b00111000, 0b01101100, 0b11000110, 0b11000110,
// code 89, 0x59 - not mapped
0b00000000, 0b11000110, 0b11000110, 0b11000110, 0b01111110, 0b00001100, 0b00011000, 0b01110000,
// code 90, 0x5A - not mapped
0b00000000, 0b11111110, 0b00001100, 0b00011000, 0b00110000, 0b01100000, 0b11000000, 0b11111110,
// code 91, 0x5B - not mapped
0b00000000, 0b00111100, 0b00110000, 0b00110000, 0b00110000, 0b00110000, 0b00110000, 0b00111100,
// code 92, 0x5C - not mapped
0b00000000, 0b10000000, 0b11000000, 0b01100000, 0b00110000, 0b00011000, 0b00001100, 0b00000100,
// code 93, 0x5D - not mapped
0b00000000, 0b00111100, 0b00001100, 0b00001100, 0b00001100, 0b00001100, 0b00001100, 0b00111100,
// code 94, 0x5E - not mapped
0b00000000, 0b00010000, 0b00111000, 0b01101100, 0b00000000, 0b00000000, 0b00000000, 0b00000000,
// code 95, 0x5F - not mapped
0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b11111110,
// code 96, 0x60 - not mapped
0b00000000, 0b00110000, 0b00110000, 0b00011000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,
// code 97, 0x61 - not mapped
0b00000000, 0b00000000, 0b00000000, 0b01111100, 0b11000110, 0b11000110, 0b11001110, 0b01110110,
// code 98, 0x62 - not mapped
0b00000000, 0b11000000, 0b11000000, 0b11111100, 0b11000110, 0b11000110, 0b11100110, 0b11011100,
// code 99, 0x63 - not mapped
0b00000000, 0b00000000, 0b00000000, 0b01111110, 0b11000000, 0b11000000, 0b11000000, 0b01111110,
// code 100, 0x64 - not mapped
0b00000000, 0b00000110, 0b00000110, 0b01111110, 0b11000110, 0b11000110, 0b11001110, 0b01110110,
// code 101, 0x65 - not mapped
0b00000000, 0b00000000, 0b00000000, 0b01111100, 0b11000110, 0b11111100, 0b11000000, 0b01111100,
// code 102, 0x66 - not mapped
0b00000000, 0b00011100, 0b00110000, 0b01111000, 0b00110000, 0b00110000, 0b00110000, 0b00110000,
// code 103, 0x67 - not mapped
0b00000000, 0b00000000, 0b00000000, 0b01111100, 0b11000110, 0b01111110, 0b00000110, 0b01111100,
// code 104, 0x68 - not mapped
0b00000000, 0b11000000, 0b11000000, 0b11011100, 0b11100110, 0b11000110, 0b11000110, 0b11000110,
// code 105, 0x69 - not mapped
0b00000000, 0b00011000, 0b00000000, 0b00011000, 0b00011000, 0b00011000, 0b00011000, 0b00011000,
// code 106, 0x6A - not mapped
0b00000000, 0b00001100, 0b00000000, 0b00001100, 0b00001100, 0b00001100, 0b00001100, 0b01111000,
// code 107, 0x6B - not mapped
0b00000000, 0b11000000, 0b11000000, 0b11000110, 0b11001100, 0b11111000, 0b11001100, 0b11000110,
// code 108, 0x6C - not mapped
0b00000000, 0b00110000, 0b00110000, 0b00110000, 0b00110000, 0b00110000, 0b00110000, 0b00110000,
// code 109, 0x6D - not mapped
0b00000000, 0b00000000, 0b00000000, 0b11001100, 0b11111110, 0b11010110, 0b11000110, 0b11000110,
// code 110, 0x6E - not mapped
0b00000000, 0b00000000, 0b00000000, 0b11011100, 0b11100110, 0b11000110, 0b11000110, 0b11000110,
// code 111, 0x6F - not mapped
0b00000000, 0b00000000, 0b00000000, 0b01111100, 0b11000110, 0b11000110, 0b11000110, 0b01111100,
// code 112, 0x70 - not mapped
0b00000000, 0b00000000, 0b00000000, 0b11011100, 0b11100110, 0b11000110, 0b11111100, 0b11000000,
// code 113, 0x71 - not mapped
0b00000000, 0b00000000, 0b00000000, 0b01111110, 0b11000110, 0b11001110, 0b01110110, 0b00000110,
// code 114, 0x72 - not mapped
0b00000000, 0b00000000, 0b00000000, 0b11011100, 0b11100110, 0b11000000, 0b11000000, 0b11000000,
// code 115, 0x73 - not mapped
0b00000000, 0b00000000, 0b00000000, 0b01111100, 0b11000000, 0b01111100, 0b00000110, 0b11111100,
// code 116, 0x74 - not mapped
0b00000000, 0b01100000, 0b01100000, 0b11111000, 0b01100000, 0b01100000, 0b01100110, 0b00111100,
// code 117, 0x75 - not mapped
0b00000000, 0b00000000, 0b00000000, 0b11000110, 0b11000110, 0b11000110, 0b11001110, 0b01110110,
// code 118, 0x76 - not mapped
0b00000000, 0b00000000, 0b00000000, 0b11000110, 0b11000110, 0b01101100, 0b00111000, 0b00010000,
// code 119, 0x77 - not mapped
0b00000000, 0b00000000, 0b00000000, 0b11000110, 0b11010110, 0b11010110, 0b11111110, 0b01101100,
// code 120, 0x78 - not mapped
0b00000000, 0b00000000, 0b00000000, 0b11000110, 0b01101100, 0b00111000, 0b01101100, 0b11000110,
// code 121, 0x79 - not mapped
0b00000000, 0b00000000, 0b00000000, 0b11000110, 0b11000110, 0b01111110, 0b00000110, 0b01111100,
// code 122, 0x7A - not mapped
0b00000000, 0b00000000, 0b00000000, 0b11111110, 0b00001100, 0b00111000, 0b01100000, 0b11111110,
// code 123, 0x7B - not mapped
0b00000000, 0b00011000, 0b00110000, 0b00110000, 0b01100000, 0b00110000, 0b00110000, 0b00011000,
// code 124, 0x7C - not mapped
0b00000000, 0b00110000, 0b00110000, 0b00110000, 0b00000000, 0b00110000, 0b00110000, 0b00110000,
// code 125, 0x7D - not mapped
0b00000000, 0b01100000, 0b00110000, 0b00110000, 0b00011000, 0b00110000, 0b00110000, 0b01100000,
// code 126, 0x7E - not mapped
0b00000000, 0b01110110, 0b11011100, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,
// code 127, 0x7F - generated blank
0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,
// code 128, 0x80 - generated blank
0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,
// code 129, 0x81 - generated blank
0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,
// code 130, 0x82 - generated blank
0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,
// code 131, 0x83 - generated blank
0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,
// code 132, 0x84 - generated blank
0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,
// code 133, 0x85 - generated blank
0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,
// code 134, 0x86 - generated blank
0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,
// code 135, 0x87 - generated blank
0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,
// code 136, 0x88 - generated blank
0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,
// code 137, 0x89 - generated blank
0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,
// code 138, 0x8A - generated blank
0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,
// code 139, 0x8B - generated blank
0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,
// code 140, 0x8C - generated blank
0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,
// code 141, 0x8D - generated blank
0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,
// code 142, 0x8E - generated blank
0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,
// code 143, 0x8F - generated blank
0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,
// code 144, 0x90 - generated blank
0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,
// code 145, 0x91 - generated blank
0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,
// code 146, 0x92 - generated blank
0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,
// code 147, 0x93 - generated blank
0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,
// code 148, 0x94 - generated blank
0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,
// code 149, 0x95 - generated blank
0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,
// code 150, 0x96 - generated blank
0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,
// code 151, 0x97 - generated blank
0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,
// code 152, 0x98 - generated blank
0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,
// code 153, 0x99 - generated blank
0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,
// code 154, 0x9A - generated blank
0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,
// code 155, 0x9B - generated blank
0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,
// code 156, 0x9C - generated blank
0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,
// code 157, 0x9D - generated blank
0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,
// code 158, 0x9E - generated blank
0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,
// code 159, 0x9F - generated blank
0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,
// code 160, 0xA0 - generated blank
0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,
// code 161, 0xA1 - mapped unicode 0104
0b00000000, 0b01111100, 0b11000110, 0b11000110, 0b11111110, 0b11000110, 0b11001110, 0b11000111,
// code 162, 0xA2 - not mapped
0b00000000, 0b00010000, 0b01111110, 0b11010000, 0b11010000, 0b11010000, 0b01111110, 0b00010000,
// code 163, 0xA3 - mapped unicode 0141
0b00000000, 0b01100000, 0b01100000, 0b01110000, 0b11100000, 0b01100000, 0b01100000, 0b01111110,
// code 164, 0xA4 - not mapped
0b00000000, 0b00000000, 0b11000110, 0b01111100, 0b00101000, 0b00101000, 0b01111100, 0b11000110,
// code 165, 0xA5 - mapped unicode 013D
0b00000000, 0b11010000, 0b11010000, 0b11000000, 0b11000000, 0b11000000, 0b11000000, 0b11111110,
// code 166, 0xA6 - mapped unicode 015A
0b00001000, 0b00010000, 0b01111100, 0b11000000, 0b01111100, 0b00000110, 0b11000110, 0b01111100,
// code 167, 0xA7 - not mapped
0b00000000, 0b01111100, 0b11000000, 0b11111100, 0b11000110, 0b01111110, 0b00000110, 0b01111100,
// code 168, 0xA8 - not mapped
0b00000000, 0b01101100, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,
// code 169, 0xA9 - mapped unicode 0160
0b00101000, 0b00010000, 0b01111100, 0b11000000, 0b01111100, 0b00000110, 0b11000110, 0b01111100,
// code 170, 0xAA - mapped unicode 015E
0b00000000, 0b01111100, 0b11000000, 0b01111100, 0b00000110, 0b11000110, 0b01111100, 0b11100000,
// code 171, 0xAB - mapped unicode 0164
0b00010100, 0b00001000, 0b01111110, 0b00011000, 0b00011000, 0b00011000, 0b00011000, 0b00011000,
// code 172, 0xAC - mapped unicode 0179
0b00001000, 0b00010000, 0b11111110, 0b00001100, 0b00011000, 0b00110000, 0b01100000, 0b11111110,
// code 173, 0xAD - generated blank
0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,
// code 174, 0xAE - mapped unicode 017D
0b00101000, 0b00010000, 0b11111110, 0b00001100, 0b00011000, 0b00110000, 0b01100000, 0b11111110,
// code 175, 0xAF - mapped unicode 017B
0b00010000, 0b00000000, 0b11111110, 0b00001100, 0b00011000, 0b00110000, 0b01100000, 0b11111110,
// code 176, 0xB0 - not mapped
0b00000000, 0b00111000, 0b01101100, 0b00111000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,
// code 177, 0xB1 - mapped unicode 0105
0b00000000, 0b00000000, 0b00000000, 0b01111100, 0b11000110, 0b11001110, 0b01110110, 0b00000111,
// code 178, 0xB2 - not mapped
0b00000000, 0b11110000, 0b00110000, 0b11100000, 0b11110000, 0b00000000, 0b00000000, 0b00000000,
// code 179, 0xB3 - mapped unicode 0142
0b00000000, 0b00110000, 0b00110000, 0b00111000, 0b01110000, 0b00110000, 0b00110000, 0b00110000,
// code 180, 0xB4 - not mapped
0b00000000, 0b00110000, 0b01100000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,
// code 181, 0xB5 - mapped unicode 013E
0b00000000, 0b00110100, 0b00110100, 0b00110000, 0b00110000, 0b00110000, 0b00110000, 0b00110000,
// code 182, 0xB6 - mapped unicode 015B
0b00000000, 0b00001000, 0b00010000, 0b01111100, 0b11000000, 0b01111100, 0b00000110, 0b11111100,
// code 183, 0xB7 - not mapped
0b00000000, 0b00000000, 0b00000000, 0b00111000, 0b00111000, 0b00000000, 0b00000000, 0b00000000,
// code 184, 0xB8 - not mapped
0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00011000, 0b11011000, 0b01110000,
// code 185, 0xB9 - mapped unicode 0161
0b00000000, 0b00101000, 0b00010000, 0b01111100, 0b11000000, 0b01111100, 0b00000110, 0b11111100,
// code 186, 0xBA - mapped unicode 015F
0b00000000, 0b00000000, 0b01111100, 0b11000000, 0b01111100, 0b00000110, 0b01111100, 0b11100000,
// code 187, 0xBB - mapped unicode 0165
0b00000100, 0b01101000, 0b01100000, 0b01100000, 0b11111000, 0b01100000, 0b01100110, 0b00111100,
// code 188, 0xBC - mapped unicode 017A
0b00000000, 0b00001000, 0b00010000, 0b11111110, 0b00001100, 0b00111000, 0b01100000, 0b11111110,
// code 189, 0xBD - mapped unicode 02DD
0b00000000, 0b00110110, 0b00110110, 0b00100100, 0b00000000, 0b00000000, 0b00000000, 0b00000000,
// code 190, 0xBE - mapped unicode 017E
0b00000000, 0b00101000, 0b00010000, 0b11111110, 0b00001100, 0b00111000, 0b01100000, 0b11111110,
// code 191, 0xBF - mapped unicode 017C
0b00000000, 0b00010000, 0b00000000, 0b11111110, 0b00001100, 0b00111000, 0b01100000, 0b11111110,
// code 192, 0xC0 - mapped unicode 0154
0b00001000, 0b00010000, 0b11111100, 0b11000110, 0b11111100, 0b11000110, 0b11000110, 0b11000110,
// code 193, 0xC1 - not mapped
0b00001000, 0b00010000, 0b01111100, 0b11000110, 0b11111110, 0b11000110, 0b11000110, 0b11000110,
// code 194, 0xC2 - not mapped
0b00010000, 0b00101000, 0b01111100, 0b11000110, 0b11111110, 0b11000110, 0b11000110, 0b11000110,
// code 195, 0xC3 - mapped unicode 0102
0b01000100, 0b00111000, 0b01111100, 0b11000110, 0b11111110, 0b11000110, 0b11000110, 0b11000110,
// code 196, 0xC4 - not mapped
0b00101000, 0b00000000, 0b01111100, 0b11000110, 0b11111110, 0b11000110, 0b11000110, 0b11000110,
// code 197, 0xC5 - mapped unicode 0139
0b00010000, 0b00100000, 0b11000000, 0b11000000, 0b11000000, 0b11000000, 0b11000000, 0b11111110,
// code 198, 0xC6 - mapped unicode 0106
0b00001000, 0b00010000, 0b01111100, 0b11000110, 0b11000000, 0b11000000, 0b11000110, 0b01111100,
// code 199, 0xC7 - not mapped
0b00000000, 0b01111100, 0b11000110, 0b11000000, 0b11000000, 0b11000110, 0b01111100, 0b11100000,
// code 200, 0xC8 - mapped unicode 010C
0b00101000, 0b00010000, 0b01111100, 0b11000110, 0b11000000, 0b11000000, 0b11000110, 0b01111100,
// code 201, 0xC9 - not mapped
0b00001000, 0b00010000, 0b11111110, 0b11000000, 0b11111000, 0b11000000, 0b11000000, 0b11111110,
// code 202, 0xCA - mapped unicode 0118
0b00000000, 0b11111100, 0b11000000, 0b11111000, 0b11000000, 0b11000000, 0b11111100, 0b00000110,
// code 203, 0xCB - not mapped
0b00101000, 0b00000000, 0b11111110, 0b11000000, 0b11111000, 0b11000000, 0b11000000, 0b11111110,
// code 204, 0xCC - mapped unicode 011A
0b00101000, 0b00010000, 0b11111110, 0b11000000, 0b11111000, 0b11000000, 0b11000000, 0b11111110,
// code 205, 0xCD - not mapped
0b00001000, 0b00010000, 0b01111110, 0b00011000, 0b00011000, 0b00011000, 0b00011000, 0b01111110,
// code 206, 0xCE - not mapped
0b00001000, 0b00010100, 0b01111110, 0b00011000, 0b00011000, 0b00011000, 0b00011000, 0b01111110,
// code 207, 0xCF - mapped unicode 010E
0b00101000, 0b00010000, 0b11111000, 0b11001100, 0b11000110, 0b11000110, 0b11001100, 0b11111000,
// code 208, 0xD0 - mapped unicode 0110
0b00000000, 0b01111000, 0b01101100, 0b01100110, 0b11110110, 0b01100110, 0b01101100, 0b01111000,
// code 209, 0xD1 - mapped unicode 0143
0b00001000, 0b00010000, 0b11000110, 0b11100110, 0b11110110, 0b11011110, 0b11001110, 0b11000110,
// code 210, 0xD2 - mapped unicode 0147
0b00101000, 0b00010000, 0b11000110, 0b11100110, 0b11110110, 0b11011110, 0b11001110, 0b11000110,
// code 211, 0xD3 - not mapped
0b00001000, 0b00010000, 0b01111100, 0b11000110, 0b11000110, 0b11000110, 0b11000110, 0b01111100,
// code 212, 0xD4 - not mapped
0b00010000, 0b00101000, 0b01111100, 0b11000110, 0b11000110, 0b11000110, 0b11000110, 0b01111100,
// code 213, 0xD5 - mapped unicode 0150
0b00010100, 0b00101000, 0b01111100, 0b11000110, 0b11000110, 0b11000110, 0b11000110, 0b01111100,
// code 214, 0xD6 - not mapped
0b00101000, 0b00000000, 0b01111100, 0b11000110, 0b11000110, 0b11000110, 0b11000110, 0b01111100,
// code 215, 0xD7 - not mapped
0b00000000, 0b01100110, 0b00111100, 0b00011000, 0b00111100, 0b01100110, 0b00000000, 0b00000000,
// code 216, 0xD8 - mapped unicode 0158
0b00101000, 0b00010000, 0b11111100, 0b11000110, 0b11111100, 0b11000110, 0b11000110, 0b11000110,
// code 217, 0xD9 - mapped unicode 016E
0b00010000, 0b00101000, 0b11010110, 0b11000110, 0b11000110, 0b11000110, 0b11000110, 0b01111110,
// code 218, 0xDA - not mapped
0b00001000, 0b00010000, 0b11000110, 0b11000110, 0b11000110, 0b11000110, 0b11000110, 0b01111110,
// code 219, 0xDB - mapped unicode 0170
0b00010100, 0b00101000, 0b11000110, 0b11000110, 0b11000110, 0b11000110, 0b11000110, 0b01111110,
// code 220, 0xDC - not mapped
0b00101000, 0b00000000, 0b11000110, 0b11000110, 0b11000110, 0b11000110, 0b11000110, 0b01111110,
// code 221, 0xDD - not mapped
0b00001000, 0b00010000, 0b11000110, 0b11000110, 0b01111110, 0b00001100, 0b00011000, 0b01110000,
// code 222, 0xDE - mapped unicode 0162
0b00000000, 0b01111110, 0b00011000, 0b00011000, 0b00011000, 0b00011000, 0b00011000, 0b00110000,
// code 223, 0xDF - not mapped
0b00000000, 0b01111000, 0b11001100, 0b11011000, 0b11001100, 0b11000110, 0b11000110, 0b11001100,
// code 224, 0xE0 - mapped unicode 0155
0b00000000, 0b00001000, 0b00010000, 0b11011100, 0b11100110, 0b11000000, 0b11000000, 0b11000000,
// code 225, 0xE1 - not mapped
0b00000000, 0b00001000, 0b00010000, 0b01111100, 0b11000110, 0b11000110, 0b11001110, 0b01110110,
// code 226, 0xE2 - not mapped
0b00000000, 0b00010000, 0b00101000, 0b01111100, 0b11000110, 0b11000110, 0b11001110, 0b01110110,
// code 227, 0xE3 - mapped unicode 0103
0b00000000, 0b01000100, 0b00111000, 0b01111100, 0b11000110, 0b11000110, 0b11001110, 0b01110110,
// code 228, 0xE4 - not mapped
0b00000000, 0b00101000, 0b00000000, 0b01111100, 0b11000110, 0b11000110, 0b11001110, 0b01110110,
// code 229, 0xE5 - mapped unicode 013A
0b00000100, 0b00001000, 0b00110000, 0b00110000, 0b00110000, 0b00110000, 0b00110000, 0b00110000,
// code 230, 0xE6 - mapped unicode 0107
0b00000000, 0b00001000, 0b00010000, 0b01111110, 0b11000000, 0b11000000, 0b11000000, 0b01111110,
// code 231, 0xE7 - not mapped
0b00000000, 0b00000000, 0b00000000, 0b01111110, 0b11000000, 0b11000000, 0b01111110, 0b11100000,
// code 232, 0xE8 - mapped unicode 010D
0b00000000, 0b00101000, 0b00010000, 0b01111110, 0b11000000, 0b11000000, 0b11000000, 0b01111110,
// code 233, 0xE9 - not mapped
0b00000000, 0b00001000, 0b00010000, 0b01111100, 0b11000110, 0b11111100, 0b11000000, 0b01111100,
// code 234, 0xEA - mapped unicode 0119
0b00000000, 0b00000000, 0b01111100, 0b11000110, 0b11111100, 0b11000000, 0b01111100, 0b00001110,
// code 235, 0xEB - not mapped
0b00000000, 0b00101000, 0b00000000, 0b01111100, 0b11000110, 0b11111100, 0b11000000, 0b01111100,
// code 236, 0xEC - mapped unicode 011B
0b00000000, 0b00101000, 0b00010000, 0b01111100, 0b11000110, 0b11111100, 0b11000000, 0b01111100,
// code 237, 0xED - not mapped
0b00001000, 0b00010000, 0b00000000, 0b00011000, 0b00011000, 0b00011000, 0b00011000, 0b00011000,
// code 238, 0xEE - not mapped
0b00001000, 0b00010100, 0b00000000, 0b00011000, 0b00011000, 0b00011000, 0b00011000, 0b00011000,
// code 239, 0xEF - mapped unicode 010F
0b00000000, 0b00011010, 0b00011010, 0b01111000, 0b11011000, 0b11011000, 0b11111000, 0b01011000,
// code 240, 0xF0 - mapped unicode 0111
0b00001100, 0b00011110, 0b00001100, 0b01111100, 0b11001100, 0b11001100, 0b11011100, 0b01101100,
// code 241, 0xF1 - mapped unicode 0144
0b00000000, 0b00001000, 0b00010000, 0b11011100, 0b11100110, 0b11000110, 0b11000110, 0b11000110,
// code 242, 0xF2 - mapped unicode 0148
0b00000000, 0b00101000, 0b00010000, 0b11011100, 0b11100110, 0b11000110, 0b11000110, 0b11000110,
// code 243, 0xF3 - not mapped
0b00000000, 0b00001000, 0b00010000, 0b01111100, 0b11000110, 0b11000110, 0b11000110, 0b01111100,
// code 244, 0xF4 - not mapped
0b00000000, 0b00010000, 0b00101000, 0b01111100, 0b11000110, 0b11000110, 0b11000110, 0b01111100,
// code 245, 0xF5 - mapped unicode 0151
0b00000000, 0b00010100, 0b00101000, 0b01111100, 0b11000110, 0b11000110, 0b11000110, 0b01111100,
// code 246, 0xF6 - not mapped
0b00000000, 0b00101000, 0b00000000, 0b01111100, 0b11000110, 0b11000110, 0b11000110, 0b01111100,
// code 247, 0xF7 - not mapped
0b00000000, 0b00000000, 0b00011000, 0b00000000, 0b01111110, 0b00000000, 0b00011000, 0b00000000,
// code 248, 0xF8 - mapped unicode 0159
0b00000000, 0b00101000, 0b00010000, 0b11011100, 0b11100110, 0b11000000, 0b11000000, 0b11000000,
// code 249, 0xF9 - mapped unicode 016F
0b00010000, 0b00101000, 0b00010000, 0b11000110, 0b11000110, 0b11000110, 0b11001110, 0b01110110,
// code 250, 0xFA - not mapped
0b00000000, 0b00001000, 0b00010000, 0b11000110, 0b11000110, 0b11000110, 0b11001110, 0b01110110,
// code 251, 0xFB - mapped unicode 0171
0b00000000, 0b00010100, 0b00101000, 0b11000110, 0b11000110, 0b11000110, 0b11001110, 0b01110110,
// code 252, 0xFC - not mapped
0b00000000, 0b00101000, 0b00000000, 0b11000110, 0b11000110, 0b11000110, 0b11001110, 0b01110110,
// code 253, 0xFD - not mapped
0b00000000, 0b00001000, 0b00010000, 0b11000110, 0b11000110, 0b01111110, 0b00000110, 0b01111100,
// code 254, 0xFE - mapped unicode 0163
0b00000000, 0b01100000, 0b01100000, 0b01111000, 0b01100000, 0b01100110, 0b00111100, 0b11100000,
// code 255, 0xFF - not mapped
0b00000000, 0b00101000, 0b00000000, 0b11000110, 0b11000110, 0b01111110, 0b00000110, 0b01111100

// total 256 characters, 68 substituted with blank

};