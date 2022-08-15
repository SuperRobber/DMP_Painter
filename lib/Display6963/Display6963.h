#ifndef Display6963_h
#define Display6963_h

#include <Arduino.h>
// #include <SdFat.h>
// #include "font.h"

extern uint8_t screen_buffer[32*64];
// extern uint8_t bitmapfont_8x8 [16*128];
// extern uint8_t bitmapfont_16x24 [32*288];
extern uint16_t lcd_drawstep;

struct Sprite {
  uint8_t width;
  uint8_t height;
  uint8_t xpos;
  uint8_t ypos;
  uint8_t data[9];
  uint8_t speed;
};

void setPixel(uint8_t x, uint8_t y);
void clearPixel(uint8_t x, uint8_t y);

void drawSprite(Sprite s);
void drawChar(uint8_t font[],int charindex,int xpos, int ypos, bool inverse);
void drawBigChar(uint8_t font[],int charindex,int xpos, int ypos);

void drawText(uint8_t x, uint8_t y, String text, uint8_t font[], bool inverse=false);
void drawTextBig(uint8_t x, uint8_t y, String text, uint8_t font[], bool inverse=false);

void drawSquare(uint8_t x_start, uint8_t y_start, uint8_t x_end, uint8_t y_end);
void drawFrame(uint8_t x_start, uint8_t y_start, uint8_t x_end, uint8_t y_end);
void drawLine(uint8_t x_start, uint8_t y_start, uint8_t x_end, uint8_t y_end,bool inverse=false);

void drawScreen();
void drawScreenStep();

// uint32_t loadSample(FsFile f, int16_t *buf,uint32_t sample_start);
// void loadBitmapFont(FsFile f, uint8_t *buf, uint8_t charsize_x, uint8_t charsize_y, uint8_t width, uint8_t height);

void lcd_send_command(uint8_t b);
void lcd_send_data(uint8_t b);
void lcd_init(uint8_t write_pin, uint8_t command_pin);

#endif