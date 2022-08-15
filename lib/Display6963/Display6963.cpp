#include "Display6963.h"
#include <SPI.h>
// #include "font.h"

int local_lcd_write_pin = 0;
int local_lcd_command_pin = 0;

uint16_t lcd_drawstep = 0;

uint8_t screen_buffer[32 * 64];
// uint8_t bitmapfont_8x8 [16*128];
// uint8_t bitmapfont_16x24 [32*288];

SPISettings spi_lcd_config(30000000, MSBFIRST, SPI_MODE0);

void drawText(uint8_t x, uint8_t y, String text, uint8_t font[], bool inverse) {
    uint8_t offset = 0;
    for (unsigned i = 0; i < text.length(); ++i) {
        char c = text.charAt(i);
        drawChar(font, (int)c, x + i * 7 + offset, y, inverse);
        if (c == 77)
            offset += 1; // M
        if (c == 87)
            offset += 1; // W
    }
}

void drawTextBig(uint8_t x, uint8_t y, String text, uint8_t font[], bool inverse) {
    uint8_t offset = 0;
    for (unsigned i = 0; i < text.length(); ++i) {
        char c = text.charAt(i);
        drawBigChar(font, (int)c, x + i * 12 + offset, y);
        if (c == 77)
            offset += 1; // M
        if (c == 87)
            offset += 1; // W
    }
}

void drawSquare(uint8_t x_start, uint8_t y_start, uint8_t x_end, uint8_t y_end) {
    if (y_start <= y_end) {
        for (int y = y_start; y <= y_end; y++) {
            if (x_start <= x_end) {
                for (int x = x_start; x <= x_end; x++) {
                    setPixel(x, y);
                }
            } else {
                for (int x = x_end; x <= x_start; x++) {
                    setPixel(x, y);
                }
            }
        }
    } else {
        for (int y = y_end; y <= y_start; y++) {
            if (x_start <= x_end) {
                for (int x = x_start; x <= x_end; x++) {
                    setPixel(x, y);
                }
            } else {
                for (int x = x_end; x <= x_start; x++) {
                    setPixel(x, y);
                }
            }
        }
    }
}

void drawFrame(uint8_t x_start, uint8_t y_start, uint8_t x_end, uint8_t y_end) {
    if (y_start <= y_end) {
        for (int y = y_start; y <= y_end; y++) {
            setPixel(x_start, y);
            setPixel(x_end, y);
        }
    } else {
        for (int y = y_end; y <= y_start; y++) {
            setPixel(x_start, y);
            setPixel(x_end, y);
        }
    }
    if (x_start <= x_end) {
        for (int x = x_start; x <= x_end; x++) {
            setPixel(x, y_start);
            setPixel(x, y_end);
        }
    } else {
        for (int x = x_end; x <= x_start; x++) {
            setPixel(x, y_start);
            setPixel(x, y_end);
        }
    }
}

void drawLine(uint8_t x_start, uint8_t y_start, uint8_t x_end, uint8_t y_end, bool inverse) {
    int x_size = abs(x_end - x_start);
    int y_size = abs(y_end - y_start);
    int x, y;
    if (x_size >= y_size) {
        if (x_start <= x_end) {
            for (x = x_start; x <= x_end; x++) {
                int xy = x - x_start;
                if (y_start <= y_end) {
                    y = min(y_end, y_start + (xy * (y_size + 1)) / x_size);
                } else {
                    y = max(y_end, y_start - (xy * (y_size + 1)) / x_size);
                }
                if (inverse) {
                    clearPixel(x, y);
                } else {
                    setPixel(x, y);
                }
            }
        } else {
            for (x = x_end; x <= x_start; x++) {
                int xy = x - x_end;
                if (y_start <= y_end) {
                    y = min(y_end, y_start + (xy * (y_size + 1)) / x_size);
                } else {
                    y = max(y_end, y_start - (xy * (y_size + 1)) / x_size);
                }
                if (inverse) {
                    clearPixel(x, y);
                } else {
                    setPixel(x, y);
                }
            }
        }
    } else {
        if (y_start <= y_end) {
            for (y = y_start; y <= y_end; y++) {
                int xy = y - y_start;
                if (x_start <= x_end) {
                    x = min(x_end, x_start + (xy * (x_size + 1)) / y_size);
                } else {
                    x = max(x_end, x_start - (xy * (x_size + 1)) / y_size);
                }
                if (inverse) {
                    clearPixel(x, y);
                } else {
                    setPixel(x, y);
                }
            }
        } else {
            for (y = y_end; y <= y_start; y++) {
                int xy = y - y_end;
                if (x_start <= x_end) {
                    x = max(x_start, x_end - (xy * (x_size + 1)) / y_size);
                } else {
                    x = min(x_start, x_end + (xy * (x_size + 1)) / y_size);
                }
                if (inverse) {
                    clearPixel(x, y);
                } else {
                    setPixel(x, y);
                }
            }
        }
    }
}

void setPixel(uint8_t x, uint8_t y) {
    int index = x / 8 + y * 32;
    int subindex = x % 8;
    bitSet(screen_buffer[index], 7 - subindex);
}

void clearPixel(uint8_t x, uint8_t y) {
    int index = x / 8 + y * 32;
    int subindex = x % 8;
    bitClear(screen_buffer[index], 7 - subindex);
}

void drawSprite(Sprite s) {
    for (int y = 0; y < s.height; y++) {
        for (int x = 0; x < 8; x++) {
            if (bitRead(s.data[y], x)) {
                setPixel((s.xpos - x), (s.ypos + y));
            }
        }
    }
}

void drawScreenStep() {
    if (lcd_drawstep == 0)
        lcd_send_command(0x80); // set OR-MODE & set INTERNAL CGROM
    if (lcd_drawstep == 1)
        lcd_send_data(0x00);
    if (lcd_drawstep == 2)
        lcd_send_data(0x00);

    if (lcd_drawstep == 3)
        lcd_send_command(0x42); // set Graphic Home Address
    if (lcd_drawstep == 4)
        lcd_send_data(0x20);
    if (lcd_drawstep == 5)
        lcd_send_data(0x00);

    if (lcd_drawstep == 6)
        lcd_send_command(0x43); // set Graphic Area
    if (lcd_drawstep == 7)
        lcd_send_data(0x00);
    if (lcd_drawstep == 8)
        lcd_send_data(0x00);

    if (lcd_drawstep == 9)
        lcd_send_command(0x24); // set adress
    if (lcd_drawstep == 10)
        lcd_send_command(0xB0); // enable auto write

    if (lcd_drawstep >= 11 && lcd_drawstep < (32 * 64 + 11)) {
        lcd_send_data(screen_buffer[lcd_drawstep - 11]);
        // clear buffer
        screen_buffer[lcd_drawstep - 11] = 0;
        // screen_buffer[lcd_drawstep-11]=255;
    }

    if (lcd_drawstep == (11 + 32 * 64))
        lcd_send_command(0xB2); // reset auto write
    if (lcd_drawstep == (12 + 32 * 64)) {
        lcd_send_command(0b10011000); // display mode: Graphic
        lcd_drawstep = 0;
    } else {
        lcd_drawstep++;
    }
}

void drawScreen() {
    // lcd_send_command(0x80);             // set OR-MODE & set INTERNAL CGROM

    // lcd_send_data(0x00);
    // lcd_send_data(0x00);
    // lcd_send_command(0x42);             // set Graphic Home Address

    // lcd_send_data(0x20);
    // lcd_send_data(0x00);

    // lcd_send_command(0x43);             // set Graphic Area
    // lcd_send_data(0x00);
    // lcd_send_data(0x00);

    lcd_send_command(0x24); // set adress
    lcd_send_command(0xB0); // enable auto write

    for (int i = 0; i < 32 * 64; i++) {
        lcd_send_data(screen_buffer[i]);
        // clear buffer
        screen_buffer[i] = 0;
        // screen_buffer[i]=255; //invert clear
    }
    lcd_send_command(0xB2); // reset auto write
                            // lcd_send_command(0b10011000);       // display mode: Graphic
}

// void loadBitmapFont(FsFile f, uint8_t *buf,uint8_t charsize_x, uint8_t charsize_y, uint8_t char_width, uint8_t char_height) {
//     uint32_t pixeldata_offset;
//     uint16_t bits_per_pixel;
//     f.seek(10);
//     f.read(&pixeldata_offset, 4);
//     f.seek(28);
//     f.read(&bits_per_pixel, 2);
//     f.seek(pixeldata_offset);

//     uint32_t b;
//     for (int y=0; y < charsize_y*char_height*8; y++) {
//         for (int x=0; x < charsize_x*char_width; x++) {
//             int findex = y*charsize_x*char_width+x;
//             buf[findex] = 0;
//             for (int i = 0; i<8;i++) {
//                 buf[findex] = buf[findex] << 1;
//                 f.read(&b, bits_per_pixel / 8);
//                 buf[findex] |= bitRead(b,0);
//             }
//         }
//     }
// }

void drawChar(uint8_t font[], int charindex, int xpos, int ypos, bool inverse = false) {
    int charindex_y = 15 - charindex / 16;
    int charindex_x = charindex % 16;
    for (int y = 0; y < 8; y++) {
        for (int x = 0; x < 8; x++) {
            if (bitRead(font[(charindex_y * 8 + (7 - y)) * 16 + charindex_x], x)) {
                int index = (xpos - x + 7) / 8 + (ypos + y) * 32;
                int subindex = (xpos - x + 7) % 8;
                if (inverse) {
                    bitClear(screen_buffer[index], 7 - subindex);
                } else {
                    bitSet(screen_buffer[index], 7 - subindex);
                }
            }
        }
    }
}

void drawBigChar(uint8_t font[], int charindex, int xpos, int ypos) {
    int charindex_y = 11 - charindex / 16;
    int charindex_x = charindex % 16;
    for (int y = 0; y < 24; y++) {
        for (int x = 0; x < 8; x++) {
            int index = (xpos - x + 7) / 8 + (ypos + y) * 32;
            int subindex = (xpos - x + 7) % 8;
            if (bitRead(font[(23 - y) * 32 + (charindex_y * 24 * 16 * 2 + charindex_x * 2)], x)) {
                bitSet(screen_buffer[index], 7 - subindex);
            }
            if (bitRead(font[(23 - y) * 32 + (charindex_y * 24 * 16 * 2 + charindex_x * 2) + 1], x)) {
                bitSet(screen_buffer[index + 1], 7 - subindex);
            }
        }
    }
}

void lcd_send(uint8_t b) {
    SPI.beginTransaction(spi_lcd_config);
    SPI.transfer(b);
    SPI.endTransaction();
    delayNanoseconds(1000);
    digitalWriteFast(local_lcd_write_pin, LOW);
    delayNanoseconds(1000);
    digitalWriteFast(local_lcd_write_pin, HIGH);
    delayNanoseconds(1000);
}

void lcd_send_command(uint8_t b) {
    digitalWriteFast(local_lcd_command_pin, HIGH); // command
    lcd_send(b);
}

void lcd_send_data(uint8_t b) {
    digitalWriteFast(local_lcd_command_pin, LOW); // data
    lcd_send(b);
}

void lcd_init(uint8_t write_pin, uint8_t command_pin) {
    local_lcd_command_pin = command_pin;
    local_lcd_write_pin = write_pin;

    pinMode(write_pin, OUTPUT);
    digitalWriteFast(write_pin, HIGH);

    pinMode(command_pin, OUTPUT);
    digitalWriteFast(command_pin, LOW); // data mode write // status mode read

    lcd_send_command(0x80); // set OR-MODE & set INTERNAL CGROM

    lcd_send_data(0x00);
    lcd_send_data(0x00);
    lcd_send_command(0x42); // set Graphic Home Address

    lcd_send_data(0x20);
    lcd_send_data(0x00);
    lcd_send_command(0x43); // set Graphic Area

    // lcd_send_command(0b10010000);    // display mode: Off

    // clear display
    lcd_send_data(0x00);
    lcd_send_data(0x00);
    lcd_send_command(0x24); // set adress

    lcd_send_command(0xB0); // auto write mode

    for (int i = 0; i < 32 * 64; i++) {
        lcd_send_data(0b00000000);
    }

    lcd_send_command(0xB2);       // stop auto write
    lcd_send_command(0b10011000); // display mode: Graphic
}
