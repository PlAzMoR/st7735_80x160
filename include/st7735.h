// Setting include guards against multiple enabling
#ifndef ST7735_H
#define ST7735_H

/* --- Libraries --- */

// Required hardware libraries  
#include "hardware/spi.h" // To connect display through SPI
#include "hardware/clocks.h"
#include "hardware/pwm.h" // To control brightness
#include "pico/stdlib.h" // Standard library for working with RPi Pico

// Other required libraries  
#include "cmath" // For drawing algorithms
#include "cstring" // For memcpy

// Non-required libraries
#include <stdio.h> // Serial input/output library

/* --- Libraries --- */



/* --- Private macroses --- */

// @skip Display settings
#define SCREEN_WIDTH    160 // Display width in pixels
#define SCREEN_HEIGHT    80 // Display height in pixels

// @skip Display commands (CMD)
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
#define CMD_ADDRESS_DIR      0x36 // MADCTL
#define CMD_COLOR_MODE_SET   0x3A // COLMOD

// @skip Brightness control
#define MAX_WRAP        255 // Wrap maximum value (other value: 65 535)
#define FREQ_CLKDIV    50.f // Value to control PWM frequency
#define GAMMA_FACTOR    2.2 // Value for smoothing linear brightness changing (from 1.8 to 2.5)

// @skip Active Zone settings
#define AZ_SIZE           2

// @skip Tile Displaying mode
#define CHANGED           1 // Tile was changed (for changed tiles array)
#define NOT_CHANGED       0 // Tile wasn't changed (for changed tiles array)

/* --- Private macroses --- */



/* --- Public macroses --- */

// @skip Class constructor
#define NO_PIN            0 // Used to ignore argument pin

// @skip 16-bit colors (full pallete - https://github.com/newdigate/rgb565_colors)
#define WHITE        0xFFFF
#define BLACK        0x0000
#define RED          0xF800
#define GREEN        0x07E0
#define BLUE         0x001F
#define CYAN         0x07FF
#define MAGENTA      0xF81F
#define YELLOW       0xFFE0

// @skip Brightness control
#define BRIGHTNESS_MAX  100 // Brightness: 100%
#define BRIGHTNESS_HALF  50 // Brightness: 50%
#define BRIGHTNESS_OFF    0 // Brightness: 0%
#define BRIGHTNESS_DEFAULT  BRIGHTNESS_MAX // Display brightness by default (100%)

// @skip Color inversion
#define INVERSION_OFF     0
#define INVERSION_ON      1

// @skip Displaying modes
#define FULLSCREEN_MODE   0
#define CAZ_MODE          1
#define RAZ_MODE          2
#define PAW_MODE          3
#define TILE_MODE         4
#define EXPAW_MODE        5

// @skip Tile Displaying mode
#define TILE_SIZE_XL     40
#define TILE_SIZE_L      16
#define TILE_SIZE_M       8
#define TILE_SIZE_S       4
#define TILE_SIZE_XS      2
#define TILE_SIZE_DEFAULT  TILE_SIZE_L

// @skip Screen mirroring
#define VER_MIRROR        1 // By Y
#define NO_VER_MIRROR     0 // By Y
#define HOR_MIRROR        1 // By X
#define NO_HOR_MIRROR     0 // By X

/* --- Public macroses --- */



// LCD Display Header Class
class ST7735_80x160 {
    private:
    /* Private properties */

        const uint8_t _PIN_SCK; // Pin for synchronizing data transfer (SC)
        const uint8_t _PIN_MOSI; // Pin for transfering data from MCU to display (SDA)
        const uint8_t _PIN_CS; // Pin for selecting SPI slave (0 -> display receives data, 1 -> ignores data)
        const uint8_t _PIN_DC; // Pin for selecting data to send (on 0 - commands, on 1 - data)
        const uint8_t _PIN_RST; // Pin for resetting display (RES)
        const uint8_t _PIN_BLK; // Pin for brightness controlling

        const uint8_t _OFFSET_X = 1; // Constant offset by X axis for address window
        const uint8_t _OFFSET_Y = 26; // Constant offset by Y axis for address window

        spi_inst_t* _SPI_PORT; // Spi port on Pico board (spi0 or spi1)
        uint32_t _SPI_FREQ; // Spi baudrate (default - 33MHZ)

        uint8_t _CUR_BRIGHTNESS; // Current display brightness percentage (if BLK pin is connected)

        bool _COLOR_INVERSION = false; // Color inversion boolean

        bool _MIRRORED_VER = NO_VER_MIRROR; // Reflects screen by Y axis (vertical) (0 -> disable, 1 -> enable)
        bool _MIRRORED_HOR = NO_HOR_MIRROR; // Reflects screen by X axis (horizontal) (0 -> disable, 1 -> enable)

        /* Screen buffer - array that used to save current frame and show on LCD in display() call.
        * It has size of columns * rows amount = 12 800 pixels (160x80), contains color information about
        * every screen pixel in uint16_t (unsigned char, 2 bytes), their address is index in the buffer.
        * 
        * Total buffer size = 25 kB (with 160x80 resolution)
        */
        uint16_t _screen_buffer[SCREEN_WIDTH][SCREEN_HEIGHT];

        /* Delta buffer - array, same as screen buffer, contains next frame that used to find difference 
        * between current frame and next one (drawed, but not displayed) to not re-drawing old pixels.
        * 
        * Total buffer size = 25 kB (same as SB, with 160x80 resolution)
        * 
        * Used by _displayPAW(), getPixelDelta()
        */
        uint16_t _delta_buffer[SCREEN_WIDTH][SCREEN_HEIGHT];



        /* Displaying modes */

        /* Displaying mode defines, how display() method will write data to screen (check docs)
        * Available modes:
        * +   0 - Fullscreen mode (very stable, default)
        * +   1 - Column Active Zone
        * +   2 - Row Active Zone
        * +   3 - Pixel Address Window
        * +   4 - Tile Displaying mode
        * +   5 - Expandable Address Window
        */
        int _DISPLAYING_MODE = FULLSCREEN_MODE;


        // CAZ & PAW mode
        /* Displaying with active zone column optimization (CAZ)
        * Array contains only changed column part start/end - active zones: first value (_minY) is start of AZ,
        * second (_maxY) is end. When currentY = _minY, _displayCAZ private method starts drawing every pixel in column
        * until currentY < _maxY, then moves to the next column. (full name: _columnsActiveZones[currentColumn][AZ start/end]).
        * Also used by PAW for acceleration.
        * 
        * Total array size = 0.3 kB
        */
        uint8_t _colsAZ[SCREEN_WIDTH][AZ_SIZE];


        // RAZ mode
        /* Displaying with active zone row optimization (RAZ)
        * Array contains only changed row part start/end - active zones: first value (_minX) is start of AZ,
        * second (_maxX) is end. When currentX = _minX, _displayRAZ private method starts drawing every pixel
        * in row until currentX < _maxX, then moves to the next row. (full name: _rowsActiveZones[currentRow][AZ start/end]).
        * 
        * Total array size = 0.16 kB
        */
        uint8_t _rowsAZ[SCREEN_HEIGHT][AZ_SIZE];


        // Tile Displaying mode
        /* Displaying by tiles (TILE)
        * Array contains bool values for every tile - was tile changed or not.
        * If value is true (CHANGED), _displayTiles private method draws current tile,
        * else - moves to the next one (full name: _changedTiles[tilesX][tilesY]).
        * 
        * Total array size = 4 B - 1.25 kB (maxTileSize - minTileSize, 80x160 resolution)
        */
        bool** _cgdTiles;
        // Current tile size by X (2 times more than Y with 160x80 resolution)
        uint8_t _tileSizeX = TILE_SIZE_DEFAULT * 2;
        // Current tile size by Y
        uint8_t _tileSizeY = TILE_SIZE_DEFAULT;

        
        // Expandable address window
        /* Expandable address window (ExpAW)
        * This mode contains 4 values for 2 extreme points of changed rectangle zone.
        * Lowest and highest points are XY axis positions. Pixels on {0, 0} and {W-1, H-1}
        * creates window with relevant lowest and highest positions, triangle with points
        * {30, 50}, {60, 70} and {115, 40} creates {30, 40} and {115, 70} address window.
        * 
        * Total mode size = 4 B
        */
        uint8_t _startX, _startY; // Lowest  (minimum by default) ExpAW position
        uint8_t _endX,   _endY;   // Highest (maximum by default) ExpAW position

        /* End of displaying modes */

    /* End of private properties */


    /* Private methods */

        // @skip Basic data writing by SPI
        void _write_spi(uint8_t bus_data);
        void _send_data(uint8_t data);
        void _send_data16(uint16_t data16);
        void _send_buffer();
        void _send_command(uint8_t command);


        // @skip Working with display
        void _reset_display();
        void _spi_setup();
        void _blk_setup();
        void _cmd_setup();
        void _tiles_setup(uint8_t tileSize);
        void _setAddressWindow(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2);


        // @skip Displaying modes
        void _displayFullscreen();
        void _displayCAZ();
        void _displayRAZ();
        void _displayPAW();
        void _displayTiles();
        void _displayExpAW();

    /* End of private methods */



    public:
    /* Public properties */

        const uint8_t W = SCREEN_WIDTH;  // Max display width in pixels (equal to SCREEN_WIDTH)
        const uint8_t H = SCREEN_HEIGHT; // Max display height in pixels (equal to SCREEN_HEIGHT)

    /* End of public properties */



    /* Class operators */

        // Constructor
        ST7735_80x160(
            uint8_t PIN_SCK,
            uint8_t PIN_MOSI,
            uint8_t PIN_CS,
            uint8_t PIN_DC,
            uint8_t PIN_RST,
            uint8_t PIN_BLK = 0,
            uint32_t SPI_FREQ = 33 * MHZ
        );

        // Destructor
        ~ST7735_80x160();

    /* End of class operators */



    /* Public methods */

    // @skip --- Get data methods ---
    uint8_t getPinSCK() const;
    uint8_t getPinMOSI() const;
    uint8_t getPinRST() const;
    uint8_t getPinDC() const;
    uint8_t getPinCS() const;
    uint8_t getPinBLK() const;

    spi_inst_t* getSpiPort() const;
    uint32_t getSpiFreq() const;

    uint8_t getBrightness() const;
    bool getColorInversion() const;
    int getDisplayingMode() const;
    uint8_t getTileSize() const;

    bool getMirrorVer() const;
    bool getMirrorHor() const;

    uint16_t getPixel(uint8_t x, uint8_t y) const;
    uint16_t getPixelDelta(uint8_t x, uint8_t y) const;


    // @skip --- Settings methods ---
    void setSpiFrequency(uint32_t freq);
    void setBrightness(uint8_t brightness);
    void setColorInversion(bool inversion);
    void setDisplayingMode(int displayingMode);
    void setScreenMirror(bool verMirror, bool horMirror);
    void setTileSize(uint8_t tileSize);


    // @skip --- General methods ---
    void init();
    void display();
    void clearDisplayForce(uint16_t color = BLACK);


    //  @skip --- GFX methods ---
    void clearDisplay(uint16_t color = BLACK);

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
