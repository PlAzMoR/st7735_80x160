// Including header with all dependences, like stdio
#include "st7735.h"


// @skip Macroses for methods
#define swap(val1, val2) { auto tmp = val2; val2 = val1; val1 = tmp; } // Swaps values (val1 <-> val2)
#define round3(num) (num + 500) / 1000 * 1000 // Rounds int value to thousands (1500 -> 2000)



/** --- Class Constructor ---
 * 
 * @since v1
*/
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

/** --- Class Destructor ---
 * 
 * @since v4
*/
ST7735_80x160::~ST7735_80x160() {

    // Deleting tiles 2D dynamic array to release memory
    for (uint8_t x = 0; x < W / _tileSizeX; x++) delete[] _cgdTiles[x];
    delete[] _cgdTiles;
}



/* Private methods */

// Basic data writing by SPI

/** 
 * @brief Sends 1 byte to slave through SPI port
 * 
 * @param bus_data byte that sends to display controller
 * 
 * @since v1
*/
void ST7735_80x160::_write_spi(uint8_t bus_data) {

    // Activating writing mode by CS pin
    gpio_put(_PIN_CS, 0);
    // Writing bytes of data with 1 length by address to display SPI port
    spi_write_blocking(_SPI_PORT, &bus_data, 1);
    // Deactivating writing mode
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

    // Activating data mode and writing 1 byte
    gpio_put(_PIN_DC, 1);
    _write_spi(data8);
}

/**
 * @brief Sends 2 data bytes to display in Little Endian format (for color)
 * 
 * @param data16 2 bytes of data to send
 * 
 * @note Wikipedia opinion - https://en.wikipedia.org/wiki/Endianness
 * 
 * @since v1
 */
void ST7735_80x160::_send_data16(uint16_t data16) {

    // Activating data mode and writing 2 bytes of data
    gpio_put(_PIN_DC, 1);
    _write_spi(data16 & 0xFF); // First (low) byte
    _write_spi(data16 >> 8); // Second (high) byte
}

/**
 * @brief Sends 2D delta buffer to display
 * 
 * Method used by Fullscreen DM.
 * 
 * @since v4
 */
void ST7735_80x160::_send_buffer() {

    // Activating data mode by DC pin
    gpio_put(_PIN_DC, 1);
    // Activating writing mode by CS pin
    gpio_put(_PIN_CS, 0);

    // Choosing every buffer column and sending it as uint8_t (bytes)
    for (uint8_t x = 0; x < W; x++) spi_write_blocking(_SPI_PORT, (uint8_t*)_delta_buffer[x], H * sizeof(uint16_t));

    // Deactivating writing mode
    gpio_put(_PIN_CS, 1);
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

    // Initializing RST pin as OUTPUT
    gpio_init(_PIN_RST); gpio_set_dir(_PIN_RST, GPIO_OUT);

    // Sending short signal to pin
    gpio_put(_PIN_RST, 0);
    sleep_ms(50);
    gpio_put(_PIN_RST, 1);
    sleep_ms(50);
}

/**
 * @brief Configures SPI pins and port
 * 
 * @since v1
 */
void ST7735_80x160::_spi_setup() {

    // Pins configuration
    gpio_init(_PIN_SCK);  gpio_set_function(_PIN_SCK, GPIO_FUNC_SPI);     // SCK pin
    gpio_init(_PIN_MOSI); gpio_set_function(_PIN_MOSI, GPIO_FUNC_SPI);    // MOSI pin

    gpio_init(_PIN_CS);   gpio_set_dir(_PIN_CS, GPIO_OUT);                // CS pin
    gpio_init(_PIN_DC);   gpio_set_dir(_PIN_DC, GPIO_OUT);                // DC pin

    // Initializing SPI port of display with private frequency
    spi_init(_SPI_PORT, _SPI_FREQ);
}

/**
 * @brief Configures PWM and brightness control pin (BLK)
 * 
 * Current PWM frequency on Pico board - 9.7 kHz to minimize blinking.
 * 
 * PWM frequency can be speed up/slow down by decreasing/increasing clkdiv PWM value.
 * 
 * Possible frequencies: 2 - 20 kHz.
 * 
 * @since v3
 */
void ST7735_80x160::_blk_setup() {

    // BLK pin not connected handler
    if (!_PIN_BLK) return;

    // Initializing BLK pin as PWM
    gpio_init(_PIN_BLK); gpio_set_function(_PIN_BLK, GPIO_FUNC_PWM);

    // Taking BLK pin PWM slice (check RP2040 docs 4.6)
    uint8_t _slice_num = pwm_gpio_to_slice_num(_PIN_BLK);
    // Taking PWM default config to change
    pwm_config _pwm_config = pwm_get_default_config();

    // Setting PWM slice max value to 255 to speed up performance by lower PWM data
    pwm_config_set_wrap(&_pwm_config, MAX_WRAP);
    // Changing clock divider to 50 for slowing down PWM frequency
    pwm_config_set_clkdiv(&_pwm_config, FREQ_CLKDIV);

    // Re-initializing PWM with updated operating frequency = 
    // = MCU freq / (clkdiv * (wrap + 1)) = 125MHz / 50 * (255 + 1) = ~9.7kHz
    pwm_init(_slice_num, &_pwm_config, true);
    // Enabling PWM on BLK slice
    pwm_set_enabled(_slice_num, true);

    // Setting display brightness to default
    setBrightness(BRIGHTNESS_DEFAULT);
}

/**
 * @brief Sets display configuration by commands
 * 
 * @since v1
 */
void ST7735_80x160::_cmd_setup() {

    // Commands for configuring display
    _send_command(CMD_SFW_RESET); // Software reset (to protect from errors and old configuration data)
    sleep_ms(50);

    _send_command(CMD_SLEEP_OUT); // Awaking display
    sleep_ms(200);

    if (_COLOR_INVERSION == false) { // Default colors in st7735 are inverted
        _send_command(CMD_COLOR_INV_ON); // If no inversion, return to defaul colors (actually inverted)
    }

    _send_command(CMD_ADDRESS_DIR); // Address direction, inverts display in some data values
    _send_data(0x08); // 0x08 - default order (RGB)

    _send_command(CMD_COLOR_MODE_SET); // Setting color mode
    _send_data(0x05); // 0x05 - 16 bit/pixel (RGB565 - standard)

    _send_command(CMD_DISPLAY_ON);
}

/**
 * @brief Initializes tiles array used by Tiles DM
 * 
 * @param tileSize size of tiles by Y (2 times more by X)
 * 
 * @since v4
 */
void ST7735_80x160::_tiles_setup(uint8_t tileSize) {

    // Changing private tile size properties
    _tileSizeX = tileSize * 2;
    _tileSizeY = tileSize;

    // Creating 1D array of pointers (X axis part of dynamic 2D array)
    _cgdTiles = new bool*[W / _tileSizeX];

    // Creating 1D arrays from pointers (Y axis part of array)
    for (uint8_t x = 0; x < W / _tileSizeX; x++) _cgdTiles[x] = new bool[H / _tileSizeY];
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
void ST7735_80x160::_setAddressWindow(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2) {

    // Commands and data for configuring address window
    _send_command(CMD_COL_ADR_SET); // Column address configuration
    _send_data(0x00); _send_data(y1 + _OFFSET_Y);
    _send_data(0x00); _send_data(y2 + _OFFSET_Y);

    _send_command(CMD_ROW_ADR_SET); // Row address configuration
    _send_data(0x00); _send_data(x1 + _OFFSET_X);
    _send_data(0x00); _send_data(x2 + _OFFSET_X);

    _send_command(CMD_MEM_WRITE); // Now we can write data
}



// Displaying modes

/**
 * @brief Private displaying method used by display() wrapper
 * 
 * Displaying mode - Fullscreen mode
 * 
 * @since v3
 */
void ST7735_80x160::_displayFullscreen() {

    // Fullscreen address window set
    _setAddressWindow(0, 0, W-1, H-1);

    // Sending buffer
    _send_buffer();

    // Replacing screen buffer with delta buffer
    for (uint8_t x = 0; x < W; x++) memcpy(_screen_buffer[x], _delta_buffer[x], H * sizeof(uint16_t));
}

/**
 * @brief Private displaying method used by display() wrapper
 * 
 * Displaying mode - Column Active Zone
 * 
 * @since v3
 */
void ST7735_80x160::_displayCAZ() {

    // Column iterating
    for (uint8_t x = 0; x < W; x++) {

        // If current AZ exists (AZ not default) - update column
        if (_colsAZ[x][0] < H) {
                
            // Current column (at x) active zone address window set
            _setAddressWindow(x, _colsAZ[x][0], x, _colsAZ[x][1]);

            // Choosing every pixel in Active Zone
            for (uint8_t y = _colsAZ[x][0]; y <= _colsAZ[x][1]; y++) {
                    
                // Sending pixel from buffer to display
                _send_data16(_delta_buffer[x][y]);

                // Replacing screen buffer with delta buffer (updated zone)
                _screen_buffer[x][y] = _delta_buffer[x][y];
            }
                
            // Setting CAZ to default
            _colsAZ[x][0] = H;
            _colsAZ[x][1] = 0;
        }
    }
}

/**
 * @brief Private displaying method used by display() wrapper
 * 
 * Displaying mode - Row Active Zone
 * 
 * @since v3
 */
void ST7735_80x160::_displayRAZ() {

    // Row iterating
    for (uint8_t y = 0; y < H; y++) {

        // If current AZ exists (AZ not default) - update row
        if (_rowsAZ[y][0] < W) {

            // Current row (at y) active zone address window set
            _setAddressWindow(_rowsAZ[y][0], y, _rowsAZ[y][1], y);
            
            // Choosing every pixel in Active Zone
            for (uint8_t x = _rowsAZ[y][0]; x <= _rowsAZ[y][1]; x++) {

                // Sending pixel from buffer to display
                _send_data16(_delta_buffer[x][y]);

                // Replacing screen buffer with delta buffer (updated zone)
                _screen_buffer[x][y] = _delta_buffer[x][y];
            }
        }

        // Setting RAZ to default
        _rowsAZ[y][0] = W;
        _rowsAZ[y][1] = 0;
    }
}

/**
 * @brief Private displaying method used by display() wrapper
 * 
 * Displaying mode - Pixel Address Window
 * 
 * @since v3
 */
void ST7735_80x160::_displayPAW() {

    // Column iterating
    for (uint8_t x = 0; x < W; x++) {

        // If current AZ exists... (AZ not default)
        if (_colsAZ[x][0] < H) {
            
            // Choosing every pixel in Column Active Zone
            for (uint8_t y = _colsAZ[x][0]; y <= _colsAZ[x][1]; y++) {

                // If current pixel is new...
                if (_screen_buffer[x][y] != _delta_buffer[x][y]) {

                    // Putting one pixel address window
                    _setAddressWindow(x, y, x, y);

                    // Sending pixel from delta buffer to display
                    _send_data16(_delta_buffer[x][y]);

                    // Replacing screen buffer with delta buffer (updated zone)
                    _screen_buffer[x][y] = _delta_buffer[x][y];
                }
            }

            // Setting PAW support (CAZ) to default
            _colsAZ[x][0] = H;
            _colsAZ[x][1] = 0;
        }
    }
}

/**
 * @brief Private displaying method used by display() wrapper
 * 
 * Displaying mode - Tile Displaying mode
 * 
 * @since v3
 */
void ST7735_80x160::_displayTiles() {

    // Tiles iterating by X
    for (uint8_t tileX = 0; tileX < W / _tileSizeX; tileX++) {

        // Tiles iterating by Y
        for (uint8_t tileY = 0; tileY < H / _tileSizeY; tileY++) {

            // If tile was changed...
            if (_cgdTiles[tileX][tileY] == CHANGED) {

                // Initializing tile start position
                uint8_t _tileStartX = abs(tileX * _tileSizeX);
                uint8_t _tileStartY = abs(tileY * _tileSizeY);
                
                // Initializing tile end position (tileSize - 1, next tile starts on tileSize)
                uint8_t _tileEndX = abs(_tileStartX + (_tileSizeX - 1));
                uint8_t _tileEndY = abs(_tileStartY + (_tileSizeY - 1));


                // Putting tile address window
                _setAddressWindow(_tileStartX,
                                _tileStartY,
                                _tileEndX,
                                _tileEndY);


                // Selecting every column in tile
                for (uint8_t x = _tileStartX; x <= _tileEndX; x++) {

                    // Selecting every pixel in tile column
                    for (uint8_t y = _tileStartY; y <= _tileEndY; y++) {

                        // Drawing all tile pixels
                        _send_data16(_delta_buffer[x][y]);

                        // Replacing screen buffer with delta buffer (updated zone)
                        _screen_buffer[x][y] = _delta_buffer[x][y];
                    }
                }

                // Resetting changed tile to default
                _cgdTiles[tileX][tileY] = NOT_CHANGED;
            }
        }
    }
}

/**
 * @brief Private displaying method used by display() wrapper
 * 
 * Displaying mode - Expandable Address Window
 * 
 * @since v3
 */
void ST7735_80x160::_displayExpAW() {

    // Putting AW to updated rectangle area
    _setAddressWindow(_startX, _startY, _endX, _endY);


    // ExpAW Column iterating
    for (uint8_t x = _startX; x <= _endX; x++) {

        // Choosing every pixel in column
        for (uint8_t y = _startY; y <= _endY; y++) {

            // Sending pixel from buffer to display
            _send_data16(_delta_buffer[x][y]);

            // Replacing screen buffer with delta buffer (updated zone)
            _screen_buffer[x][y] = _delta_buffer[x][y];
        }
    }

    // Setting ExpAW positions to default
    _startX = W, _startY = H; // Lowest
    _endX = 0,   _endY = 0;   // Highest
}

/* End of private methods */



/* Public methods */

// --- Get data methods ---

/**
 * @brief Returns SCK (SCL) pin given in class constructor
 * 
 * @since v1
*/
uint8_t ST7735_80x160::getPinSCK()      const  { return _PIN_SCK;  }
/**
 * @brief Returns MOSI (SDA) pin given in class constructor
 * 
 * @since v1
*/
uint8_t ST7735_80x160::getPinMOSI()     const  { return _PIN_MOSI; }
/**
 * @brief Returns RST (RES) pin given in class constructor
 * 
 * @since v1
*/
uint8_t ST7735_80x160::getPinRST()      const  { return _PIN_RST;  }
/**
 * @brief Returns DC pin given in class constructor
 * 
 * @since v1
*/
uint8_t ST7735_80x160::getPinDC()       const  { return _PIN_DC;   }
/**
 * @brief Returns CS pin given in class constructor
 * 
 * @since v1
*/
uint8_t ST7735_80x160::getPinCS()       const  { return _PIN_CS;   }
/**
 * @brief Returns BLK pin given in class constructor
 * 
 * @since v1
*/
uint8_t ST7735_80x160::getPinBLK()      const  { return _PIN_BLK;  }

/**
 * @brief Returns display SPI port (*3987584 - spi0 / *4003968 - spi1)
 * 
 * @since v1
*/
spi_inst_t* ST7735_80x160::getSpiPort() const  { return _SPI_PORT; }
/**
 * @brief Returns current SPI channel frequency (default - 33MHz)
 * 
 * @since v1
*/
uint32_t ST7735_80x160::getSpiFreq()    const  { return _SPI_FREQ; }

/**
 * @brief Returns current display brightness percentage
 * 
 * @since v3
*/
uint8_t ST7735_80x160::getBrightness()  const  { return _CUR_BRIGHTNESS;  }
/**
 * @brief Returns current color inversion
 * 
 * @since v3
*/
bool ST7735_80x160::getColorInversion() const  { return _COLOR_INVERSION; }
/**
 * @brief Returns current displaying mode
 * 
 * @since v3
*/
int ST7735_80x160::getDisplayingMode()  const  { return _DISPLAYING_MODE; }
/**
 * @brief Returns current tiles size (Tiles Displaying mode)
 * 
 * @since v4
*/
uint8_t ST7735_80x160::getTileSize()    const  { return _tileSizeY;       }

/**
 * @brief Returns current vertical mirrorring
 * 
 * @since v3
*/
bool ST7735_80x160::getMirrorVer()      const  { return _MIRRORED_VER; }
/**
 * @brief Returns current horizontal mirrorring
 * 
 * @since v3
*/
bool ST7735_80x160::getMirrorHor()      const  { return _MIRRORED_HOR; }

/**
 *  @brief Returns pixel on screen in specified position
 * 
 * Returns pixel color in screen buffer, that fills only after display() method.
 * 
 * @since v3
*/
uint16_t ST7735_80x160::getPixel(uint8_t x, uint8_t y) const { return (_screen_buffer[x][y] << 8) | (_screen_buffer[x][y] >> 8); }
/**
 * @brief Returns pixel on screen before it's displayed
 * 
 * Returns pixel color in delta buffer, that fills before display() with GFX methods.
 * 
 * @since v3
*/
uint16_t ST7735_80x160::getPixelDelta(uint8_t x, uint8_t y) const { return (_delta_buffer[x][y] << 8) | (_delta_buffer[x][y] >> 8); }



// --- Settings methods ---

/**
 * @brief Sets SPI frequency
 * 
 * Reinitializes SPI port with stated baudrate.
 * 
 * @param baudrate data transfer rate in HZ 
*/
void ST7735_80x160::setSpiFrequency(uint32_t baudrate) {

    // Changing private frequency (or baudrate)
    _SPI_FREQ = baudrate;

    // Reinitializing SPI port with updated rate
    spi_init(_SPI_PORT, _SPI_FREQ);
}

/**
 * @brief Sets display brightness
 * 
 * Configures display PWM brightness only if BLK pin connected,
 * 
 * uses gamma correction to smooth default unlinear PWM levels.
 * 
 * Current PWM frequency on RPi Pico - 9.7 kHz to minimize blinking.
 * 
 * To choose brightness, use relevant BRIGHTNESS_* macroses.
 * 
 * @param brightness brightness rate from 0 to 100 percent
 * 
 * @note Wikipedia opinion - https://en.wikipedia.org/wiki/Gamma_correction
 * 
 * @since v3
 */
void ST7735_80x160::setBrightness(uint8_t brightness) {

    // If brightness control pin connected...
    if (_PIN_BLK) {
        
        // Setting private brightness percentage to new level
        _CUR_BRIGHTNESS = brightness;
        
        // Converting percentage to byte (0-100 -> 0-255)
        brightness = MAX_WRAP * brightness / 100;

        // Setting PWM brightness to given level with gamma correction for linear brightness appear
        // Gamma function: (brightness float / wrap) ^ GAMMA_FACTOR * wrap
        pwm_set_gpio_level(_PIN_BLK, (uint16_t)(
                            pow(float(brightness) / MAX_WRAP, GAMMA_FACTOR) * MAX_WRAP));
    }
}

/**
 * @brief Sets colors inversion
 * 
 * When turned on, colors changes this way: RED (F800) -> CYAN, GREEN (07E0) -> PURPLE,
 * 
 * BLUE (001F) -> YELLOW, WHITE (FFFF) -> BLACK etc.
 * 
 * To enable/disable, use relevant INVERSION_(ON/OFF) macroses.
 * 
 * @param inversion on true - color inversion enables, false - disables
 * 
 * @since v1
 */
void ST7735_80x160::setColorInversion(bool inversion) {

    // Setting private inversion
    _COLOR_INVERSION = inversion;

    // Sending color inversion command (paradoxically, but colors inverted by default and on/off commands inverted too)
    if (_COLOR_INVERSION) _send_command(CMD_COLOR_INV_OFF);
    else                  _send_command(CMD_COLOR_INV_ON);
}

/**
 * @brief Sets displaying mode
 * 
 * Displaying mode - way to update screen information (check docs).
 *
 * Available modes:
 * 
 * +   0 - Fullscreen mode (very stable, default) -> FULLSCREEN_MODE
 * 
 * +   1 - Column Active Zone -> CAZ_MODE
 * 
 * +   2 - Row Active Zone -> RAZ_MODE
 * 
 * +   3 - Pixel Address Window -> PAW_MODE
 * 
 * +   4 - Tile Displaying mode -> TILE_MODE
 * 
 * +   5 - Expandable address Window -> EXPAW_MODE
 * 
 * To choose them, use relevant *_MODE macroses.
 * 
 * @param displayingMode selected DM (value from 0 to 5)
 * 
 * @since v1
 */
void ST7735_80x160::setDisplayingMode(int displayingMode) {

    // Changing private _DISPLAYING_MODE used by display()
    _DISPLAYING_MODE = displayingMode;
}

/**
 * @brief Mirrors screen by horizontal/vertical directions
 * 
 * Sets screen XY axis (y - ver, x - hor) mirroring by 2 booleans (use mirror macroses down below).
 * 
 * To mirror, use relevant *_MIRROR macroses.
 * 
 * @param verMirror Y mirroring bool (NO_VER_MIRROR -> 0, VER_MIRROR -> 1)
 * @param horMirrror X mirroring bool (NO_HOR_MIRROR -> 0, HOR_MIRROR -> 1)
 * 
 * @since v3
 */
void ST7735_80x160::setScreenMirror(bool verMirror, bool horMirror) {

    // Changing private mirrors
    _MIRRORED_VER = verMirror; // By Y
    _MIRRORED_HOR = horMirror; // By X

    // Initializing address direction byte (0x08 or bit 3 for RGB colors)
    uint8_t _dir_byte = 0x08;

    // Assigning correct byte value
    if      (!verMirror && horMirror) _dir_byte = 0x48; // Horizontal (Bit 6)
    else if (verMirror && !horMirror) _dir_byte = 0x88; // Vertical (Bit 7)
    else if (verMirror && horMirror)  _dir_byte = 0xC8; // Hor + Ver (Bit 6 + 7)

    // Sending mirror command and data
    _send_command(CMD_ADDRESS_DIR);
    _send_data(_dir_byte);
}

/**
 * @brief Sets tile size used by Tile Displaying mode
 * 
 * To change tile size, use relevant TILE_SIZE_* macroses.
 * 
 * @param tileSize new tile size by Y (2 times more by X)
 * 
 * @since v4
 */
void ST7735_80x160::setTileSize(uint8_t tileSize) {

    // Deleting Y part of 2D dynamic array
    for (uint8_t x = 0; x < W / _tileSizeX; x++) delete[] _cgdTiles[x];

    // Deleting X part of array
    delete[] _cgdTiles;

    // Changing private tile size properties
    _tileSizeX = tileSize * 2;
    _tileSizeY = tileSize;

    // Reinitializing tiles array with new sizes
    _tiles_setup(tileSize);

    // Choosing every column
    for (uint8_t xTile = 0; xTile < W / _tileSizeX; xTile++) {

        // Choosing every tile
        for (uint8_t yTile = 0; yTile < H / _tileSizeY; yTile++) {

            // Setting 'CHANGED' value to re-draw all tiles next display() call
            _cgdTiles[xTile][yTile] = CHANGED;
        }
    }
}



// --- General methods ---

/**
 * @brief Configures display settings (SPI, BLK, CMD, Tile DM)
 * 
 * @since v1
 */
void ST7735_80x160::init() {

    // LCD SPI setup
    _spi_setup();

    // Display PWM brightness control pin setup
    _blk_setup();

    // Resetting display
    _reset_display();

    // LCD CMD setup
    _cmd_setup();

    // Tile Displaying mode setup
    _tiles_setup(TILE_SIZE_DEFAULT);
}

/**
 * @brief Show screen data on display
 * 
 * Method transfers screen buffer pixels address/color data to display,
 * 
 * buffer can be filled with drawPixel(), fillRect() or other graphics functions.
 * 
 * Displaying mode chooses by setDisplayingMode() method (check docs).
 * 
 * @since v1
 */
void ST7735_80x160::display() {

    // Calling private displaying methods choosed with current DM value
    switch (_DISPLAYING_MODE) {
        default:          _displayFullscreen(); break;  // Fullscreen mode
        case CAZ_MODE:    _displayCAZ();        break;  // Column Active Zone
        case RAZ_MODE:    _displayRAZ();        break;  // Row Active Zone
        case PAW_MODE:    _displayPAW();        break;  // Pixel Address Window address
        case TILE_MODE:   _displayTiles();      break;  // Tile Displaying mode
        case EXPAW_MODE:  _displayExpAW();      break;  // Expandable Address Window
    }
}

/**
 * @brief Forcefully clears display data
 * 
 * Fills display with parameter color forcefully - without a buffer,
 * 
 * and resets all Display Modes values to default.
 * 
 * Mainly called after init() to clean screen from noise and old data.
 * 
 * @param color fills up cleared area pixels (default - BLACK)
 * 
 * @since v1
 */
void ST7735_80x160::clearDisplayForce(uint16_t color) {

    // Putting address window with screen size
    _setAddressWindow(0, 0, W-1, H-1);

    // Converting color to Big Endian format
    color = (color << 8) | (color >> 8);

    // Choosing every column
    for (uint8_t x = 0; x < W; x++) {

        // Choosing every pixel in column
        for (uint8_t y = 0; y < H; y++) {

            // Sending pixel without a buffer
            _send_data16(color);

            // Clearing screen and delta buffers with given color
            _screen_buffer[x][y] = color;
            _delta_buffer[x][y] = color;

            // Setting RAZ to default (_minX -> SCREEN_WIDTH, _maxX -> 0)
            _rowsAZ[y][0] = W;
            _rowsAZ[y][1] = 0;
        }

        // Setting CAZ to default (_minY -> SCREEN_HEIGHT, _maxY -> 0)
        _colsAZ[x][0] = H;
        _colsAZ[x][1] = 0;
    }

    // Setting ExpAW positions to default
    _startX = W, _startY = H; // Lowest
    _endX = 0,   _endY = 0;   // Highest
}



//  --- Graphics methods ---

/**
 * @brief Clears display from any GFX
 * 
 * Wrapper method for fillScreen(), that replaces delta buffer with param color
 * 
 * @param color buffer replacement color (default - BLACK)
 * 
 * @since v4
 */
void ST7735_80x160::clearDisplay(uint16_t color) {

    // Filling screen with color
    fillScreen(color);
}

/**
 * @brief Draws pixel on screen
 * 
 * Moves pixel address and color to buffer.
 * 
 * Method used by all other graphics methods.
 * 
 * For color use relevant macroses, like RED, GREEN, BLUE, WHITE etc.
 * 
 * @param x pixel X position
 * @param y pixel Y position
 * @param color pixel color
 * 
 * @since v1
 */
void ST7735_80x160::drawPixel(int16_t x, int16_t y, uint16_t color) {

    // Out of bounds error handler
    if (x > W-1 || y > H-1 || x < 0 || y < 0) return;

    // Converting color to Big Endian format
    color = (color << 8) | (color >> 8);

    // If current pixel with new color...
    if (_screen_buffer[x][y] != color) {

        // CAZ updating
        if (_colsAZ[x][0] > y) _colsAZ[x][0] = y;
        if (_colsAZ[x][1] < y) _colsAZ[x][1] = y;

        // RAZ updating
        if (_rowsAZ[y][0] > x) _rowsAZ[y][0] = x;
        if (_rowsAZ[y][1] < x) _rowsAZ[y][1] = x;

        // Tiles updating
        _cgdTiles[x / _tileSizeX][y / _tileSizeY] = CHANGED;

        // ExpAW updating
        if (x < _startX) _startX = x; // Lowest position
        if (y < _startY) _startY = y;
        if (x > _endX)   _endX = x;   // Highest position
        if (y > _endY)   _endY = y;
    }

    // Moving color to delta buffer to update previous frame on display()
    _delta_buffer[x][y] = color;
}

/**
 * @brief Fills rectangle area with color
 * 
 * Moves rectangle information to buffer.
 * 
 * For color use macroses, like RED, GREEN, BLUE, WHITE etc.
 * 
 * @param x1 area start X position
 * @param y1 area start Y position
 * @param x2 area end X position
 * @param y2 area end Y position
 * @param color area color
 * 
 * @since v1
 */
void ST7735_80x160::fillRect(int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color) {

    // Swapping verticles if they inversed
    if (x1 > x2) swap(x1, x2);
    if (y1 > y2) swap(y1, y2);

    // Filling rectangle area with pixels
    for (int16_t x = x1; x <= x2; x++) {

        for (int16_t y = y1; y <= y2; y++) {

            // Drawing pixel
            drawPixel(x, y, color);
        }
    }
}

/**
 * @brief Fills screen with color
 * 
 * Moves rectangle area with size of screen to buffer.
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
 * Moves V-line to buffer.
 * 
 * @param x V-line start X position
 * @param y V-line start Y position
 * @param length line length in pixels
 * @param color line color
 * 
 * @since v2
 */
void ST7735_80x160::drawVLine(int16_t x, int16_t y, uint8_t length, uint16_t color) {

    // Actually filling rectangle with 1 pixel width
    fillRect(x, y, x, y + length - 1, color);
}

/**
 * @brief Draws horizontal line
 * 
 * Moves H-line to buffer.
 * 
 * @param x H-line start X position
 * @param y H-line start Y position
 * @param length line length in pixels
 * @param color line color
 * 
 * @since v2
 */
void ST7735_80x160::drawHLine(int16_t x, int16_t y, uint8_t length, uint16_t color) {

    // Actually filling rectangle with 1 pixel height
    fillRect(x, y, x + length - 1, y, color);
}

/**
 * @brief Draws rectangle outline
 * 
 * Moves rectangle outline to buffer.
 * 
 * @param x1 outline start X position
 * @param y1 outline start Y position
 * @param x2 outline end X position
 * @param y2 outline end Y position
 * @param color outline color
 * 
 * @since v2
 */
void ST7735_80x160::drawRect(int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color) {

    // Swapping verticles if they inversed
    if (x1 > x2) swap(x1, x2);
    if (y1 > y2) swap(y1, y2);

    // Lines length definition
    uint8_t _vLen =  y2 - y1 + 1;
    uint8_t _hLen =  x2 - x1 - 1;

    // Drawing rectagle by 2 H-lines and 2 V-lines
    drawVLine(x1, y1, _vLen, color);
    drawVLine(x2, y1, _vLen, color);

    // Increment start x to clip pixels
    drawHLine(x1 + 1, y1, _hLen, color);
    drawHLine(x1 + 1, y2, _hLen, color);
}


/**
 * @brief Draws line from any position
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
void ST7735_80x160::drawLine(int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color) {

    // Calculating differences between axis coordinates (actually cathets if line - hypotenuse)
    int16_t _diffX = abs(x2 - x1);
    int16_t _diffY = abs(y2 - y1);

    // Choosing drawing directions by X and Y axis
    int8_t _dirX = (x2 > x1) ? 1 : -1;
    int8_t _dirY = (y2 > y1) ? 1 : -1;

    // Initializing error - deviation from ideal line (algorithm base)
    int _error = _diffX - _diffY;

    // Infinite loop while we not reach last pixel {x2, y2}
    for (;;) {

        // Drawing current pixel of line
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
 * Moves triangle outline to buffer.
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
void ST7735_80x160::drawTriangle(int16_t x1, int16_t y1, int16_t x2, int16_t y2, int16_t x3, int16_t y3, uint16_t color) {

    // Actually drawing 3 lines in stated coordinates
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
void ST7735_80x160::fillTriangle(int16_t x1, int16_t y1, int16_t x2, int16_t y2, int16_t x3, int16_t y3, uint16_t color) {

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
 * This method use Bresenham's (or Midpoint) algorithm for circles - it handles one 1/8 segment, but draws
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
void ST7735_80x160::drawCircle(int16_t x0, int16_t y0, uint8_t radius, uint16_t color) {

    // Zero radius error handler (else it draws circle with 255 radius)
    if (radius == 0) {
        drawPixel(x0, y0, color);
        return;
    }

    // Initializing coordinates from circle center for pixels in 1/8 symmetric segment
    int16_t _x = 0;
    int16_t _y = radius;

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
void ST7735_80x160::fillCircle(int16_t x0, int16_t y0, uint8_t radius, uint16_t color) {

    // Initializing pixels coordinates from circle center
    int16_t _x = 0;
    int16_t _y = radius;

    // Error value, used to correct pixels height
    int _error = 1 - radius;

    // Drawing central line to save one while cycle and remove double line check
    drawHLine(x0 - radius, y0, radius * 2 + 1, color);


    // Drawing cycle
    while (_x < _y) {
        
        // Always increment _x coordinate from start
        _x++;

        // Error correcting
        if (_error < 0) {
            _error += 2 * _x + 1;
        }
        else { // Getting down by _y (correcting height by error)
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

