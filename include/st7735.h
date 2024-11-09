// Setting include guards against multiple enabling
#ifndef ST7735_H
#define ST7735_H

// Required libraries  
#include "hardware/spi.h"
#include "hardware/clocks.h"

#include <stdio.h>
#include "pico/time.h"
#include "cmath"


// Display commands (CMD)
#define CMD_SFW_RESET        0x01 // SWRESET
#define CMD_SLEEP_IN         0x10 // SLPIN
#define CMD_SLEEP_OUT        0x11 // SLPOUT
#define CMD_COLOR_INV_OFF    0x20 // INVOFF
#define CMD_COLOR_INV_ON     0x21 // INVON
#define CMD_DISPLAY_OFF      0x28 // DISPOFF
#define CMD_DISPLAY_ON       0x29 // DISPON
#define CMD_COL_ADR_SET      0x2A // CASET
#define CMD_ROW_ADR_SET      0x2B // RASET
#define CMD_MEM_WRITE        0x2C // RAMWR
#define CMD_MEM_READ         0x2E // RAMRD
#define CMD_DISPLAY_ADR_DIR  0x36 // MADCTL
#define CMD_COLOR_MODE_SET   0x3A // COLMOD

// Public settings
#define SCREEN_WIDTH  160
#define SCREEN_HEIGHT  80

// Settings
#define ROWS_MODE     1
#define COLUMNS_MODE  0

#define X_MIRROR      1
#define NO_X_MIRROR   0
#define Y_MIRROR      1
#define NO_Y_MIRROR   0

// LCD Display Main Class
class ST7735_80x160 {
    private:
        const uint8_t _PIN_SCK; // Pin for synchronizing data transfer (SCL on board)
        const uint8_t _PIN_MOSI; // Pin for transfer data from RP2040 to LCD (SDA on board)
        const uint8_t _PIN_CS; // Pin for writing signals (on 0 - display receiving data, on - 1 display ignores data)
        const uint8_t _PIN_DC; // Pin for choosing signals you transfer (on 0 - commands, on 1 - data)
        const uint8_t _PIN_RST; // Pin for resetting display (RES on board)
        const uint8_t _PIN_BLK; // Pin for control brightness

        spi_inst_t* _SPI_PORT; // Spi port (spi0 or spi1, not const, but can't be modified)
        uint32_t _SPI_FREQ; // Spi baudrate (defines data transfer rate, default - 33MHZ)

        const uint8_t _OFFSET_X = 1; // Offset by X axis for display
        const uint8_t _OFFSET_Y = 26; // Offset by Y axis for display

        /* Buffer (array) of every pixel color and address for displaying next screen 
        (uses EMPTY (0x0000 color) pixels that not changed from _prev_screen_buffer 
        for optimization - we send only changed pixels to ST7735S) */
        uint16_t _screen_buffer[SCREEN_WIDTH][SCREEN_HEIGHT];
        // uint16_t _prev_screen_buffer[SCREEN_WIDTH][SCREEN_HEIGHT]; // Buffer of every pixel color before change for optimization

        /* Displaying with active zone column optimization
        Array for containing only changed columns zones: first value (_minY) is start of changed zone, 
        second value (_maxY) is end of changed zone (slowest case - pixels at least on {x, 0} and on {x, 79}, 
        fastest case - only one pixel anywhere in col) (full name: _changedColumnsArea[column][zone]) */
        uint8_t _cndCols[SCREEN_WIDTH][2];
        /* Array for containing only changed columns zones: first value (_minY) is start of changed zone, 
        second value (_maxY) is end of changed zone (slowest case - pixels at least on {x, 0} and on {x, 79}, 
        fastest case - only one pixel anywhere in col) (full name: _changedColumnsArea[column][zone]) */
        uint8_t _cndRows[SCREEN_HEIGHT][2];

        bool _COLOR_INVERSION = false; // Inverts colors on display (red -> cyan, green -> purple, blue -> yellow)
        bool _ROW_DISPLAYING = false; // Defines using of rows/columns displaying. It's optimize displaying in some cases


    // Private methods
    void _write_spi(uint8_t bus_data);
    void _send_data(uint8_t data);
    void _send_data16(uint16_t data16);
    void _send_command(uint8_t command);

    void _reset_display();
    void _spi_setup();
    void _cmd_setup();
    void _setAdressWindow(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2);


    public:

    const uint8_t W = SCREEN_WIDTH; // Max display width
    const uint8_t H = SCREEN_HEIGHT; // Max display height

    // Display class constructor
    ST7735_80x160(
        uint8_t PIN_SCK,
        uint8_t PIN_MOSI,
        uint8_t PIN_CS,
        uint8_t PIN_DC,
        uint8_t PIN_RST,
        uint8_t PIN_BLK,
        uint32_t SPI_FREQ = 33 * MHZ
    );

    // Public methods
    uint8_t getPinSCK() const;
    uint8_t getPinMOSI() const;
    uint8_t getPinCS() const;
    uint8_t getPinDC() const;
    uint8_t getPinRST() const;
    uint8_t getPinBLK() const;
    spi_inst_t* getSpiPort() const;
    uint32_t getSpiFreq() const;

    void init();
    void display(bool mirrorByX = false, bool mirrorByY = false);
    void clearDisplay(uint16_t color);

    void setColorInversion(bool inversion);
    void setDisplayingMode(bool displayingMode);

    void drawPixel(uint8_t x, uint8_t y, uint16_t color);
    void fillRect(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint16_t color);
    void fillScreen(uint16_t color);
};


// Colors
#define WHITE            0xFFFF
#define BLACK            0x0000
#define BLUE             0x001F
#define BRED             0XF81F
#define GRED             0XFFE0
#define GBLUE            0X07FF
#define RED              0xF800
#define MAGENTA          0xF81F
#define GREEN            0x07E0
#define CYAN             0x7FFF
#define YELLOW           0xFFE0
#define BROWN            0XBC40
#define BRRED            0XFC07
#define GRAY             0X8430

#define DARKBLUE         0X01CF
#define LIGHTBLUE        0X7D7C 
#define GRAYBLUE         0X5458

#define LIGHTGREEN       0X841F
#define LGRAY            0XC618

#define LGRAYBLUE        0XA651
#define LBBLUE           0X2B12

#define DARKGRAY         0x4208
#define DARKGREEN        0x03E0

#define DARKRED          0x7800

#endif /* ST7735_H */
