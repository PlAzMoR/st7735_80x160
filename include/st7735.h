// Setting include guards against multiple enabling
#ifndef ST7735_H
#define ST7735_H


// Required libraries  
#include "hardware/spi.h"
#include "hardware/clocks.h"

#include "pico/stdlib.h"
#include "cmath"


// Non-required libraries
// #include <stdio.h> // Input/output library
// #include <stdint.h>
// #include "pico/time.h" // Debug timer library



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


// Colors (full pallete - https://github.com/newdigate/rgb565_colors)
#define BUFFER_COLORS    0x0000 // Used as argument in _display() method

#define WHITE            0xFFFF
#define BLACK            0x0000

#define RED              0xF800
#define GREEN            0x07E0
#define BLUE             0x001F

#define CYAN             0x07FF
#define MAGENTA          0xF81F
#define YELLOW           0xFFE0


// Settings
#define SCREEN_WIDTH  160
#define SCREEN_HEIGHT  80

#define ROWS_MODE       1
#define COLUMNS_MODE    0

#define AZ_SIZE         2

#define X_MIRROR        1
#define NO_X_MIRROR     0
#define Y_MIRROR        1
#define NO_Y_MIRROR     0



// LCD Display Header Class
class ST7735_80x160 {
    private:
    /* Private properties */
        const uint8_t _PIN_SCK; // Pin for synchronizing data transfer (SCL on board)
        const uint8_t _PIN_MOSI; // Pin for transfering data from RP2040 to LCD (SDA on board)
        const uint8_t _PIN_CS; // Pin for writing signals (on 0 - display receiving data, on - 1 display ignores data)
        const uint8_t _PIN_DC; // Pin for choosing signals you transfer (on 0 - commands, on 1 - data)
        const uint8_t _PIN_RST; // Pin for resetting display (RES on board)
        const uint8_t _PIN_BLK; // Pin used to control brightness

        spi_inst_t* _SPI_PORT; // Spi port in RPi Pico board (spi0 or spi1)
        uint32_t _SPI_FREQ; // Spi baudrate (defines data transfer rate, default - 33MHZ)

        const uint8_t _OFFSET_X = 1; // Offset by X axis for display
        const uint8_t _OFFSET_Y = 26; // Offset by Y axis for display

        /* Screen buffer - array, we use to save graphics in it and show on LCD in display() call.
        It has a size of columns * rows amount = 12 800 pixels, contains color information about
        every screen pixel in uint8_t (unsigned char, 2 bytes), their address is index in the buffer.
        Total buffer size = 25 kB */
        uint16_t _screen_buffer[SCREEN_WIDTH][SCREEN_HEIGHT];

        /* Displaying with active zone column optimization (CAZ)
        Array for containing only changed column zones: first value (_minY) is start of changed zone, 
        second value (_maxY) is end of changed zone (slowest case - pixels at least on {x, 0} and on {x, 79}, 
        fastest case - only one pixel anywhere in col) (full name: _columnsActiveZone[column][activeZone]) */
        uint8_t _colsAZ[SCREEN_WIDTH][AZ_SIZE];
        /* Displaying with active zone row optimization (RAZ)
        Array for containing only changed row zones: first value (_minX) is start of changed zone, 
        second value (_maxX) is end of changed zone (slowest case - pixels at least on {0, y} and on {159, y}, 
        fastest case - only one pixel anywhere in row) (full name: _rowsActivezone[row][activeZone]) */
        uint8_t _rowsAZ[SCREEN_HEIGHT][AZ_SIZE];

        /* Similar to _colsAZ, but we fill it with argument color in clearDisplay() */
        uint8_t _cgdColsAZ[SCREEN_WIDTH][AZ_SIZE];
        /* Similar to _rowsAZ, but we fill it with argument color in clearDisplay() */
        uint8_t _cgdRowsAZ[SCREEN_HEIGHT][AZ_SIZE];

        bool _COLOR_INVERSION = false; // Inverts colors on display (red -> cyan, green -> purple, blue -> yellow)
        bool _ROW_DISPLAYING = false; // Defines using of rows/columns displaying. It's optimize displaying in some cases

    /* End of private properties */

    /* Private methods */


    // Writing info to ST7735 through SPI

    void _write_spi(uint8_t bus_data);
    void _send_data(uint8_t data);
    void _send_data16(uint16_t data16);
    void _send_command(uint8_t command);


    // Working with display

    void _reset_display();
    void _spi_setup();
    void _cmd_setup();
    void _setAdressWindow(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2);
    void _display(bool mirrorByX, bool mirrorByY, bool isClearing, uint16_t color);

    /* End of private methods */

    public:
    /* Public properties */
        const uint8_t W = SCREEN_WIDTH; // Max display width
        const uint8_t H = SCREEN_HEIGHT; // Max display height

    /* End of public properties */


    // ST7735_80x160 class constructor initializing

    ST7735_80x160(
        uint8_t PIN_SCK,
        uint8_t PIN_MOSI,
        uint8_t PIN_CS,
        uint8_t PIN_DC,
        uint8_t PIN_RST,
        uint8_t PIN_BLK,
        uint32_t SPI_FREQ = 33 * MHZ
    );


    /* Public methods */


    // --- Get data methods ---

    uint8_t getPinSCK() const;
    uint8_t getPinMOSI() const;
    uint8_t getPinCS() const;
    uint8_t getPinDC() const;
    uint8_t getPinRST() const;
    uint8_t getPinBLK() const;
    spi_inst_t* getSpiPort() const;
    uint32_t getSpiFreq() const;


    // --- General methods ---

    void init();
    void display(bool mirrorByX = false, bool mirrorByY = false);
    void clearDisplay(uint16_t color);
    void clearDisplayForce(uint16_t color = BLACK);


    // --- Settings methods ---

    void setColorInversion(bool inversion);
    void setDisplayingMode(bool displayingMode);


    //  --- Graphics methods ---

    void drawPixel(uint8_t x, uint8_t y, uint16_t color);
    void fillRect(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint16_t color);
    void fillScreen(uint16_t color);

    void drawVLine(uint8_t x, uint8_t y, uint8_t length, uint16_t color);
    void drawHLine(uint8_t x, uint8_t y, uint8_t length, uint16_t color);
    void drawRect(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint16_t color);

    void drawLine(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint16_t color);
    void drawTriangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t x3, uint8_t y3, uint16_t color);
    void fillTriangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t x3, uint8_t y3, uint16_t color);

    void drawCircle(uint8_t x0, uint8_t y0, uint8_t radius, uint16_t color);
    void fillCircle(uint8_t x0, uint8_t y0, uint8_t radius, uint16_t color);

    /* End of public methods */
};


#endif /* ST7735_H */

