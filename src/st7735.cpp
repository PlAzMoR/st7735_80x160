// Including header with all dependences, like stdio
#include "st7735.h"


// Simple macroses for class fine working
// #define abs(val) { val < 0 ? -val : val } ERROR MACROS, NEED TO FIX!!! 
#define swap(val1, val2) { auto tmp = val2; val2 = val1; val1 = tmp; } // Swaps values (val1 <-> val2)
#define round3(num) (num + 500) / 1000 * 1000 // Rounds int value to thousands (1500 -> 2000)

// uint8_t abs(int val) { return val < 0 ? -val : val; } // For mirror display ability


// Class object constructor
ST7735_80x160::ST7735_80x160
(
    uint8_t PIN_SCK,
    uint8_t PIN_MOSI,
    uint8_t PIN_CS,
    uint8_t PIN_DC,
    uint8_t PIN_RST,
    uint8_t PIN_BLK,
    uint32_t SPI_FREQ)
    :   /* Define const (except _SPI_FREQ) private features */
    _PIN_SCK(PIN_SCK),
    _PIN_MOSI(PIN_MOSI),
    _PIN_CS(PIN_CS),
    _PIN_DC(PIN_DC),
    _PIN_RST(PIN_RST),
    _PIN_BLK(PIN_BLK),
    _SPI_FREQ(SPI_FREQ),
    _SPI_PORT((_PIN_SCK == 10 || _PIN_SCK == 14) ? spi1 : spi0)
{
    // _SPI_PORT = const_cast<spi_inst_t*>(_SPI_PORT);

    // Wrong pins error handler
}



/* Private methods */

// Writing info to ST7735 through SPI

/** 
 * @brief Sends 1 byte to display through SPI port
 * 
 * @param bus_data byte of any data that sends to controller
 * 
 * @since v1
*/
void ST7735_80x160::_write_spi(uint8_t bus_data) {

    // Activating writing mode by CS pin and sending byte of data to spi port
    gpio_put(_PIN_CS, 0);
    spi_write_blocking(_SPI_PORT, &bus_data, 1);
    gpio_put(_PIN_CS, 1);
}

/**
 * @brief Sends 1 data byte to display
 * 
 * @param data8 byte of data to send
 * 
 * @since v1
 */
void ST7735_80x160::_send_data(uint8_t data8) {

    // Activating data mode and writing 1 byte of data
    gpio_put(_PIN_DC, 1);
    _write_spi(data8);
}

/**
 * @brief Sends 2 data bytes to display (for color)
 * 
 * @param data16 2 bytes of data to send
 * 
 * @since v1
 */
void ST7735_80x160::_send_data16(uint16_t data16) {

    // Activating data mode and writing 2 bytes of data
    gpio_put(_PIN_DC, 1);
    _write_spi(data16 >> 8);
    _write_spi(data16 & 0xFF);
}

/**
 * @brief Sends 1 command byte to display
 * 
 * @param data16 byte of command to send
 * 
 * @since v1
 */
void ST7735_80x160::_send_command(uint8_t command) {

    // Activating command mode and writing 1 byte cmd
    gpio_put(_PIN_DC, 0);
    _write_spi(command);
}



// Working with display

/**
 * @brief Resets display to clean old configuration data
 * 
 * @since v1
 */
void ST7735_80x160::_reset_display() {

    // Putting short signal to reset pin
    gpio_put(_PIN_RST, 0);
    sleep_ms(50);
    gpio_put(_PIN_RST, 1);
    sleep_ms(50);
}

/**
 * @brief Setups SPI pins and channel to transfer data
 * 
 * @since v1
 */
void ST7735_80x160::_spi_setup() {

    // Pins configuration
    gpio_init(_PIN_SCK); gpio_set_function(_PIN_SCK, GPIO_FUNC_SPI);            // SCK pin
    gpio_init(_PIN_MOSI); gpio_set_function(_PIN_MOSI, GPIO_FUNC_SPI);          // MOSI pin

    gpio_init(_PIN_CS); gpio_set_dir(_PIN_CS, GPIO_OUT);                        // CS pin
    gpio_init(_PIN_DC); gpio_set_dir(_PIN_DC, GPIO_OUT);                        // DC pin
    gpio_init(_PIN_RST); gpio_set_dir(_PIN_RST, GPIO_OUT);                      // RST pin

    // Initializing SPI port of LCD with given frequency
    spi_init(_SPI_PORT, _SPI_FREQ);
}

/**
 * @brief Sets display configuration by commands
 * 
 * @since v1
 */
void ST7735_80x160::_cmd_setup() {

    // Commands for configuring display
    _send_command(CMD_SFW_RESET); // Software reset (to protect from errors)
    sleep_ms(50);

    _send_command(CMD_SLEEP_OUT); // Awaking display
    sleep_ms(200);

    if (_COLOR_INVERSION == false) { // Default colors in st7735 are inverted
        _send_command(CMD_COLOR_INV_ON); // If no inversion, return to normal colors
    }

    _send_command(CMD_DISPLAY_ADR_DIR); // Address direction, inverts display in some data values
    _send_data(0x08); // 0x08 - default order (RGB)

    _send_command(CMD_COLOR_MODE_SET); // Setting color mode
    _send_data(0x05); // 0x05 - 16 bit/pixel (RGB565 - standard)

    _send_command(CMD_DISPLAY_ON);
}

/**
 * @brief Sets rectangular address window on display to draw pixels in it
 * 
 * @param x1 Start point by X
 * @param y1 Start point by Y
 * @param x2 End point by X
 * @param y2 End point by Y
 * 
 * @since v1
 */
void ST7735_80x160::_setAdressWindow(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2) {

    // Commands and data for configuring address window
    _send_command(CMD_COL_ADR_SET); // Column address configuration
    _send_data(0x00); _send_data(y1 + _OFFSET_Y);
    _send_data(0x00); _send_data(y2 + _OFFSET_Y);

    _send_command(CMD_ROW_ADR_SET); // Row address configuration
    _send_data(0x00); _send_data(x1 + _OFFSET_X);
    _send_data(0x00); _send_data(x2 + _OFFSET_X);

    _send_command(CMD_MEM_WRITE); // Now we can write data
}

/**
 * @brief Private method that use public display()/clearDisplay() to show graphics
 * 
 * @param mirrorByX Mirrors screen by X
 * @param mirrorByY Mirrors screen by Y
 * @param isClearing When on, fills buffer and screen with color parameter
 * @param color Color that fills up screen if isClearing is on
 * 
 * @since v2
 * 
 * @deprecated
 */
void ST7735_80x160::_display(bool mirrorByX, bool mirrorByY, bool isClearing, uint16_t color) {

    // Initialing mirror offsets for AZ address window
    uint8_t _x_mirror_offset = 0;
    uint8_t _y_mirror_offset = 0;

    // Initializing active zone pointers (full name: _choosed---ActiveZone)
    uint8_t (*_chColsAZ)[AZ_SIZE] = nullptr;
    uint8_t (*_chRowsAZ)[AZ_SIZE] = nullptr;

    // Setting pointers, display() case
    if (!isClearing) {
        _chColsAZ = _colsAZ;
        _chRowsAZ = _rowsAZ;
    }
    else { // clearDisplay() case
        _chColsAZ = _cgdColsAZ;
        _chRowsAZ = _cgdRowsAZ;
    }

    // Value for right mirror order (0 - minAZ, 1 - maxAZ)
    bool _orderAZ = 0;

    // Debug timer
    // absolute_time_t st = to_us_since_boot(get_absolute_time());

    // If displaying by rows...
    if (_ROW_DISPLAYING) {

    // Mirror configuring
    if (mirrorByX) { _x_mirror_offset = H-1; }
    if (mirrorByY) { _y_mirror_offset = W-1; _orderAZ = 1;}

        // Choosing every row
        for (uint8_t y = 0; y < H; y++) {

            // If active zone existing (or not default - _minX < 160) - update row
            if (_chRowsAZ[y][0] < 160) {

                // Current row (at y) active zone address window set with mirror ability
                // (orders for y mirror: NXM - 0, 1; XM - 1, 0)
                _setAdressWindow(abs(_y_mirror_offset - _chRowsAZ[y][_orderAZ]),
                                abs(_x_mirror_offset - y), 
                                abs(_y_mirror_offset - _chRowsAZ[y][!_orderAZ]),
                                abs(_x_mirror_offset - y));
                
                for (uint8_t x = _chRowsAZ[y][0]; x <= _chRowsAZ[y][1]; x++) {
                    
                    // If called by display() - send _screen_buffer pixel
                    if (!isClearing) {
                        _send_data16(_screen_buffer[x][y]);
                    }
                    else { // In clearDisplay() case - send arg color and, especially, clearing buffer to arg color
                        _send_data16(color);
                        _screen_buffer[x][y] = color;
                    }
                }
            }

            // Setting RAZ to default
            _chRowsAZ[y][0] = 160;
            _chRowsAZ[y][1] = 0;
        }
    }
    else { // If we choosed displaying by columns AZ (default)

    // Mirror configuring
    if (mirrorByX) { _x_mirror_offset = H-1; _orderAZ = 1; }
    if (mirrorByY) { _y_mirror_offset = W-1;}

        // Choosing every column
        for (uint8_t x = 0; x < W; x++) {

            // If active zone existing (or not default - _minY < 80) - update column
            if (_chColsAZ[x][0] < 80) {
                
                // Current column (at x) active zone address window set with mirror ability
                // (orders for x mirror: NXM - 0, 1; XM - 1, 0)
                _setAdressWindow(abs(_y_mirror_offset - x),
                                abs(_x_mirror_offset - _chColsAZ[x][_orderAZ]),
                                abs(_y_mirror_offset - x),
                                abs(_x_mirror_offset - _chColsAZ[x][!_orderAZ]));

                for (uint8_t y = _chColsAZ[x][0]; y <= _chColsAZ[x][1]; y++) {
                    
                    // If called by display() - send _screen_buffer pixel
                    if (!isClearing) {
                        _send_data16(_screen_buffer[x][y]);
                    }
                    else { // In clearDisplay() case - send arg color and, especially, clearing buffer to arg color
                        _send_data16(color);
                        _screen_buffer[x][y] = color;
                    }
                }
            }

            // Setting CAZ to default
            _chColsAZ[x][0] = 80;
            _chColsAZ[x][1] = 0;
        }
    }
    // Print time in us
    // printf("Done for %llu us, isClearing - %d\n", to_us_since_boot(get_absolute_time()) - st, isClearing);
}

/* End of private methods */



/* Public methods */

// --- Get data methods ---

// Returns display SCK pin
uint8_t ST7735_80x160::getPinSCK()      const  { return _PIN_SCK;  }
// Returns display MOSI pin
uint8_t ST7735_80x160::getPinMOSI()     const  { return _PIN_MOSI; }
// Returns display CS pin
uint8_t ST7735_80x160::getPinCS()       const  { return _PIN_CS;   }
// Returns display DC pin
uint8_t ST7735_80x160::getPinDC()       const  { return _PIN_DC;   }
// Returns display RST pin
uint8_t ST7735_80x160::getPinRST()      const  { return _PIN_RST;  }
// Returns display BLK pin
uint8_t ST7735_80x160::getPinBLK()      const  { return _PIN_BLK;  }
// Returns display SPI port (spi0 or spi1)
spi_inst_t* ST7735_80x160::getSpiPort() const  { return _SPI_PORT; }
// Returns display SPI channel frequency (default - 33MHz)
uint32_t ST7735_80x160::getSpiFreq()    const  { return _SPI_FREQ; }




// --- General methods ---

/**
 * @brief Configures display settings (SPI and CMD)
 * 
 * @since v1
 */
void ST7735_80x160::init() {

    // LCD SPI setup
    _spi_setup();

    // Resetting display
    _reset_display();

    // LCD CMD setup
    _cmd_setup();
}

/**
 * @brief Show screen data on display
 * 
 * Method transfers screen buffer pixels address/color data to display,
 * 
 * buffer can be filled with drawPixel(), fillRect() or other graphics functions.
 * 
 * Also display() provides mirror ability (by X and Y axis),
 * 
 * for it better use X_MIRROR, Y_MIRROR macroses arguments.
 * 
 * @param mirrorByX Mirrors screen by X (default = NO_X_MIRROR = false)
 * @param mirrorByY Mirrors screen by Y (default = NO_Y_MIRROR = false)
 * 
 * @since v1
 */
void ST7735_80x160::display(bool mirrorByX, bool mirrorByY) {

    // Calling private _display() with display() arguments
    _display(mirrorByX, mirrorByY, 0, BUFFER_COLORS);
}

/**
 * @brief Clear display with color parameter
 * 
 * (Will be removed in v3 version because of unusability -> screen blinking)
 * 
 * @param color fills up display changed zones on clearDisplay() call
 * 
 * @since v2
 * 
 * @deprecated
 */
void ST7735_80x160::clearDisplay(uint16_t color) {

    // Calling private _display() with clearing arguments
    _display(NO_X_MIRROR, NO_Y_MIRROR, 1, color);
}

/**
 * @brief Forcefully clears display data
 * 
 * Fills display with parameter color forcefully - without a buffer.
 * 
 * Mainly called after init() to clean screen from noise.
 * 
 * @param color fills up cleared area pixels (default = BLACK)
 * 
 * @since v1
 */
void ST7735_80x160::clearDisplayForce(uint16_t color) {

    // Address window with screen size
    _setAdressWindow(0, 0, W-1, H-1);

    for (uint8_t x = 0; x < W; x++) {

        for (uint8_t y = 0; y < H; y++) {

            // Sending pixel without a buffer
            _send_data16(color);

            // Setting RAZ to default
            _rowsAZ[y][0] = 160; _cgdRowsAZ[y][0] = 160;
            _rowsAZ[y][1] = 0;  _cgdRowsAZ[y][1] = 0;
        }

        // Setting CAZ to default
        _colsAZ[x][0] = 80; _cgdColsAZ[x][0] = 80;
        _colsAZ[x][1] = 0;  _cgdColsAZ[x][1] = 0;
    }
}


// --- Settings methods ---

/**
 * @brief Sets colors inversion
 * 
 * When turned on, colors changes this way: RED (F800) -> CYAN, GREEN (07E0) -> PURPLE,
 * 
 * BLUE (001F) -> YELLOW, WHITE (FFFF) -> BLACK etc.
 * 
 * @param inversion on true - color inversion enables (false - disables)
 * 
 * @since v1
 */
void ST7735_80x160::setColorInversion(bool inversion) {

    // Sending inversion color command
    if (inversion) _send_command(CMD_COLOR_INV_OFF);
    else _send_command(CMD_COLOR_INV_ON);
}

/**
 * @brief Sets displaying mode
 * 
 * Displaying mode - way to show/update screen information.
 * 
 * Library currently has 2 ways - by rows and by columns, both is optimized by Active Zone (AZ) - 
 * 
 * updating all pixels in column/row from first changed to last changed.
 * 
 * Default - column AZ, but row AZ can be 500 times faster in extreme cases (2 V-lines at x on 0 and W-1)
 * 
 * @param displayingMode on true - row AZ, false - column AZ
 * 
 * @since v1
 */
void ST7735_80x160::setDisplayingMode(bool displayingMode) {

    // Changing private property _ROW_DISPLAYING, responsible for displaying mode
    _ROW_DISPLAYING = displayingMode;
}


//  --- Graphics methods ---

/**
 * @brief Draws pixel
 * 
 * Moves pixel address and color to screen buffer.
 * 
 * Method used by all other graphics methods.
 * 
 * For color better use macroses, like RED, GREEN, BLUE, WHITE etc.
 * 
 * @param x pixel X position
 * @param y pixel Y position
 * @param color pixel color
 * 
 * @since v1
 */
void ST7735_80x160::drawPixel(uint8_t x, uint8_t y, uint16_t color) {

    // Out of bounds error handler
    if (x > W-1 || y > H-1) return;

    // If new color was added...
    if (_screen_buffer[x][y] != color) {

        // Updating columns/rows active (changed) zone
        if (_colsAZ[x][0] > y) { _colsAZ[x][0] = y; _cgdColsAZ[x][0] = y; }
        if (_colsAZ[x][1] < y) { _colsAZ[x][1] = y; _cgdColsAZ[x][1] = y; }

        if (_rowsAZ[y][0] > x) { _rowsAZ[y][0] = x; _cgdRowsAZ[y][0] = x; }
        if (_rowsAZ[y][1] < x) { _rowsAZ[y][1] = x; _cgdRowsAZ[y][1] = x; }
    }
    
    // Moving color to buffer to show on display()
    _screen_buffer[x][y] = color;
}

/**
 * @brief Fills rectangle area with color
 * 
 * Moves rectangle information to screen buffer.
 * 
 * For color better use macroses, like RED, GREEN, BLUE, WHITE etc.
 * 
 * @param x1 area start X position
 * @param y1 area start Y position
 * @param x2 area end X position
 * @param y2 area end Y position
 * @param color area color
 * 
 * @since v1
 */
void ST7735_80x160::fillRect(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint16_t color) {

    // Swapping verticles if they inversed
    if (x1 > x2) swap(x1, x2);
    if (y1 > y2) swap(y1, y2);

    // Filling rectangle area with pixels
    for (uint8_t x = x1; x <= x2; x++) {

        for (uint8_t y = y1; y <= y2; y++) {

            drawPixel(x, y, color);
        }
    }
}

/**
 * @brief Fills screen with color
 * 
 * Moves rectangle area with size of screen to screen buffer.
 * 
 * For color better use macroses, like RED, GREEN, BLUE, WHITE etc.
 * 
 * @param color screen color
 * 
 * @since v1
 */
void ST7735_80x160::fillScreen(uint16_t color) {

    // Filling rectangle with screen size
    fillRect(0, 0, W-1, H-1, color);
}

/**
 * @brief Draws vertical line
 * 
 * Moves rectangle area with same X information to screen buffer.
 * 
 * @param x V-line start X position
 * @param y V-line start Y position
 * @param length line length in pixels
 * @param color line color
 * 
 * @since v2
 */
void ST7735_80x160::drawVLine(uint8_t x, uint8_t y, uint8_t length, uint16_t color) {

    // Actually filling rectangle with 1 pixel height
    fillRect(x, y, x, y + length - 1, color);
}

/**
 * @brief Draws horizontal line
 * 
 * Moves rectangle area with same Y information to screen buffer.
 * 
 * @param x H-line start X position
 * @param y H-line start Y position
 * @param length line length in pixels
 * @param color line color
 * 
 * @since v2
 */
void ST7735_80x160::drawHLine(uint8_t x, uint8_t y, uint8_t length, uint16_t color) {

    // Actually filling rectangle with 1 pixel width
    fillRect(x, y, x + length - 1, y, color);
}

/**
 * @brief Draws rectangle outline
 * 
 * Moves 2 V-lines and 2 H-lines information to screen buffer.
 * 
 * @param x1 outline start X position
 * @param y1 outline start Y position
 * @param x2 outline end X position
 * @param y2 outline end Y position
 * @param color outline color
 * 
 * @since v2
 */
void ST7735_80x160::drawRect(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint16_t color) {

    // Lines length definition
    uint8_t _vLen =  y2 - y1 + 1;
    uint8_t _hLen =  x2 - x1 - 1;

    // Drawing rectagle with 2 H-lines and 2 V-lines
    drawVLine(x1, y1, _vLen, color);
    drawVLine(x2, y1, _vLen, color);

    // Increment start x to clip pixels
    drawHLine(x1 + 1, y1, _hLen, color);
    drawHLine(x1 + 1, y2, _hLen, color);
}


/**
 * @brief Draws line from anywhere to anywhere
 * 
 * This method based on Bresenham's line algorithm - very fast way to calculate every line
 * 
 * pixel position, but without any smoothing. Uses "error" (or "delta") value as direction corrector.
 * 
 * @param x1 line start X position
 * @param y1 line start Y position
 * @param x2 line end X position
 * @param y2 line end Y position
 * @param color line color
 * 
 * @note Wikipedia opinion - https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm
 * 
 * @since v2
 */
void ST7735_80x160::drawLine(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint16_t color) {

    // Calculating differences between axis coordinates (actually cathets if line - hypotenuse)
    uint8_t _diffX = abs(x2 - x1);
    uint8_t _diffY = abs(y2 - y1);

    // Choosing drawing directions by X and Y axis
    int _dirX = (x2 > x1) ? 1 : -1;
    int _dirY = (y2 > y1) ? 1 : -1;

    // Initializing error - deviation from ideal line (algorithm base)
    int _error = _diffX - _diffY;

    // Infinite loop while we not reach last pixel {x2, y2}
    for (;;) {

        // Single drawing function
        drawPixel(x1, y1, color);

        // End line reaching check
        if (x1 == x2 && y1 == y2) break;

        // Double error, used to simplify arithmetics
        int _err2 = _error * 2;

        // If _err2 that big, we go along X to next pixel and decrease _diffY impact
        if (_err2 > -_diffY) {
            _error -= _diffY;
            x1 += _dirX;
        }
        
        // Same for Y axis
        if (_err2 < _diffX) {
            _error += _diffX;
            y1 += _dirY;
        }
    }
}

/**
 * @brief Draws triangle outline
 * 
 * Moves 3 Bresenham's lines from/to verticles information to screen buffer.
 * 
 * @param x1 first verticle X position
 * @param y1 first verticle Y position
 * @param x2 second verticle X position
 * @param y2 second verticle Y position
 * @param x3 third verticle X position
 * @param y3 third verticle Y position
 * @param color triangle outline color
 * 
 * @since v2
 */
void ST7735_80x160::drawTriangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t x3, uint8_t y3, uint16_t color) {

    // Actually drawing 3 lines with argument coordinates
    drawLine(x1, y1, x2, y2, color);
    drawLine(x2, y2, x3, y3, color);
    drawLine(x3, y3, x1, y1, color);
}

/**
 * @brief Fills triangle area with color
 * 
 * Splits triangle to the bottom part and top part and fills them with H-lines.
 * 
 * @param x1 first verticle X position
 * @param y1 first verticle Y position
 * @param x2 second verticle X position
 * @param y2 second verticle Y position
 * @param x3 third verticle X position
 * @param y3 third verticle Y position
 * @param color triangle area color
 * 
 * @since v2
 */
void ST7735_80x160::fillTriangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t x3, uint8_t y3, uint16_t color) {

    // Sorting verticles by Y (v1 <= v2 <= v3)
    if (y1 > y2) { swap(y1, y2); swap(x1, x2); }
    if (y2 > y3) { swap(y2, y3); swap(x2, x3); }
    if (y1 > y2) { swap(y1, y2); swap(x1, x2); }


    // Triangle edges smoothing + extreme verticles positions handler (requires more time to draw)
    drawTriangle(x1, y1, x2, y2, x3, y3, color);

    // One line triangle (y1 = y2 = y3) error handler (drawTriangle() draws H-line)
    if (y1 == y2 && y2 == y3) return;


    // Initializing multiplied by 1000 (to remove slow fractional arithmetics)
    // X-offsets to increase _xStart and _xEnd by them
    int _ofsX1 = (x2 - x1) * 1000 / (y2 - y1);
    int _ofsX2 = (x3 - x1) * 1000 / (y3 - y1);

    // Initializing triangle filling H-lines multiplied start/end
    int _xStart = x1 * 1000;
    int _xEnd = x1 * 1000;

    // _lastY - value for flat-botommed triangles handling (fixes last broken line)
    int _lastY = y2;
    // _h - height of current H-line
    int _h = y1;

    // Removing last line, if triangle is flat-botommed (thx to TFT_eSPI library :) )
    if (y1 == y2) _lastY -= 1;


    // Bottom part of triangle filling cycle
    for (/* h = y1 */; _h <= _lastY; _h++) {

        // H-line filling (fillRect() instead of drawHLine(), it's more convenient)
        fillRect(round3(_xStart) / 1000, _h, round3(_xEnd) / 1000, _h, color);

        // Move start/end with relevant offsets
        _xStart += _ofsX1;
        _xEnd += _ofsX2;
    }

    // Updating second X-offset and H-lines start point for next verticle
    _ofsX1 = (x3 - x2) * 1000 / (y3 - y2);
    _xStart = x2 * 1000;

    
    // Top part of triangle filling cycle
    for (/* h = lastY */; _h <= y3; _h++) {

        // H-line filling
        fillRect(round3(_xStart) / 1000, _h, round3(_xEnd) / 1000, _h, color);

        //Move start/end with relevant offsets
        _xStart += _ofsX1;
        _xEnd += _ofsX2;
    }
}

/**
 * @brief Draws circle outline
 * 
 * This method use Bresenham's (or Midpoint) algorithm for circles - it handle one 1/8 segment, but draws
 * 
 * all (by offsetting) - they are symmetric. Uses "error" value to correct every segment pixel height.
 * 
 * @param x0 circle center X position
 * @param y0 circle center Y position
 * @param radius circle outline radius
 * @param color circle outline color
 * 
 * @note Wikipedia opinion - https://en.wikipedia.org/wiki/Midpoint_circle_algorithm
 * 
 * @since v2
 */
void ST7735_80x160::drawCircle(uint8_t x0, uint8_t y0, uint8_t radius, uint16_t color) {

    // Zero radius error handler (else it draws circle with 255 radius)
    if (radius == 0) {
        drawPixel(x0, y0, color);
        return;
    }

    // Initializing coordinates from circle center for pixels in 1/8 symmetric segment
    uint8_t _x = 0;
    uint8_t _y = radius;

    // Error value, used to correct pixels height
    int _error = 1 - radius;


    // Cycle while we not finish 1/8 segments drawing
    while (_x <= _y) {

        // Drawing symmetric segments pixels using offsets from center
        drawPixel(x0 + _x, y0 + _y, color); // 1/8
        drawPixel(x0 - _x, y0 + _y, color); // 2/8
        drawPixel(x0 + _x, y0 - _y, color); // 5/8
        drawPixel(x0 - _x, y0 - _y, color); // 6/8

        drawPixel(x0 + _y, y0 + _x, color); // 3/8
        drawPixel(x0 + _y, y0 - _x, color); // 4/8
        drawPixel(x0 - _y, y0 + _x, color); // 7/8
        drawPixel(x0 - _y, y0 - _x, color); // 8/8


        // Correcting pixels height
        if (_error < 0) {
            _error += 2 * _x + 3;
        }
        else {
            _error += 2 * (_x - _y) + 5;
            _y--; // _y decrement as fast as _x getting more
        }

        // Always increasing X offset, segment can't have more than 1 pixel in X vector (column/row)
        _x++;
    }
}

/**
 * @brief Fills circle area with color
 * 
 * This method uses Bresenham's (or Midpoint) algorithm for circles, but
 * 
 * it filling up circle with 4 H-lines (2 at the edges and 2 in center).
 * 
 * @param x0 circle center X position
 * @param y0 circle center Y position
 * @param radius circle area radius
 * @param color circle area color
 * 
 * @since v2
 */
void ST7735_80x160::fillCircle(uint8_t x0, uint8_t y0, uint8_t radius, uint16_t color) {

    // Initializing pixels coordinates from circle center
    uint8_t _x = 0;
    uint8_t _y = radius;

    // Error value, used to correct pixels height
    int _error = 1 - radius;

    // Drawing central line to save one while cycle and remove double line check
    drawHLine(x0 - radius, y0, radius * 2 + 1, color);

    while (_x < _y) {
        
        // Always increment _x coordinate from start
        _x++;

        // Error correcting
        if (_error < 0) {
            _error += 2 * _x + 1;
        }
        else { // Getting down by _y (error correct height)
            _y--;
            _error += 2 * (_x - _y) + 1;
        }

        // Drawing 4 H-lines to fill circle
        drawHLine(x0 - _x, y0 + _y, 2 * _x + 1, color);
        drawHLine(x0 - _x, y0 - _y, 2 * _x + 1, color);

        drawHLine(x0 - _y, y0 + _x, 2 * _y + 1, color);
        drawHLine(x0 - _y, y0 - _x, 2 * _y + 1, color);
    }
}

/* End of public methods */

