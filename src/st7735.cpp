// Including header with all dependences, like stdio
#include "st7735.h"



// Simple macroses for class fine working

#define swap(val1, val2) { auto tmp = val2; val2 = val1; val1 = tmp; } // Swaps values (val1 <-> val2)
#define round3(num) (num + 500) / 1000 * 1000 // Rounds int value to thousands (1500 -> 2000)

// #define abs(val) { val < 0 ? -val : val } ERROR MACROS, NEED TO FIX!!! 
// uint8_t abs(int val) { return val < 0 ? -val : val; } // For mirror display ability



// --- Class constructor ---

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
 * @param bus_data byte of any data that sends to display controller
 * 
 * @since v1
*/
void ST7735_80x160::_write_spi(uint8_t bus_data) {

    // Activating writing mode by CS pin
    gpio_put(_PIN_CS, 0);
    // Writing bytes of data with 1 length through address to display SPI port
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
    _write_spi(data16 >> 8); // First (high) byte
    _write_spi(data16 & 0xFF); // Second (low) byte
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

    // Initializing SPI port of LCD with private frequency property
    spi_init(_SPI_PORT, _SPI_FREQ);
}

/**
 * @brief Setups brightness control pin (BLK) with PWM
 * 
 * Current PWM frequency on RPi Pico - 9.7 kHz to minimize blinking.
 * 
 * PWM frequency can be speed up/down by decreasing/increasing clkdiv PWM value.
 * 
 * Possible frequencies: 2 - 20 kHz.
 * 
 * @since v3
 */
void ST7735_80x160::_blk_setup() {

    // If BLK pin connected...
    if (_PIN_BLK) {

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
        // = MCU freq / (clkdiv * (wrap + 1)) = 125MHz / 50 * (255 + 1) = ~ 9.7kHz
        pwm_init(_slice_num, &_pwm_config, true);
        // Enabling PWM on BLK slice
        pwm_set_enabled(_slice_num, true);

        // Setting display brightness to default
        setBrightness(BRIGHTNESS_DEFAULT);
    }
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



// Displaying modes

/**
 * @brief Private displaying method used by display() wrapper
 * 
 * Displaying mode - Fullscreen mode
 * 
 * @since v3
 */
void ST7735_80x160::_displayFullscreen() {

    // Debug timer
    // absolute_time_t st = to_us_since_boot(get_absolute_time());

    /* Mirror settings */

    // Initializing start position coordinates (increased by 1 to avoid uint8_t overflow)
    uint8_t _startY = 1;
    uint8_t _startX = 1;

    // Initializing last row/col coordinates (increased by 1 to avoid uint8_t overflow)
    uint8_t _lastY = H+1;
    uint8_t _lastX = W+1;

    // Initializing directions to move in buffer row/column order with mirroring
    int _dirY = 1;
    int _dirX = 1;

    // Mirror configuring if mirrorring mode enabled
    if (_MIRRORED_VER) { _startY = H; _lastY = 0; _dirY = -1; }
    if (_MIRRORED_HOR) { _startX = W; _lastX = 0; _dirX = -1; }

    /* End of mirror settings */



    // Fullscreen address window set
    _setAdressWindow(0, 0, W-1, H-1);

    // Choosing every column
    for (uint8_t x = _startX; x != _lastX; x += _dirX) {
        
        // Decrementing x to avoid uint8_t overflow
        x -= 1;

        // Choosing every pixel in column
        for (uint8_t y = _startY; y != _lastY; y += _dirY) {

            // Sending pixel from buffer to display (y decreased by 1 to avoid uint8_t overflow)
            _send_data16(_delta_buffer[x][y-1]);

            // Replacing screen buffer with delta buffer (updated zone)
            _screen_buffer[x][y-1] = _delta_buffer[x][y-1];
        }

        // Increasing x back
        x += 1;
    }

    // Debug timer
    // printf("Fullscreen done for %llu us\n", to_us_since_boot(get_absolute_time()) - st);
}

/**
 * @brief Private displaying method used by display() wrapper
 * 
 * Displaying mode - Column Active Zone
 * 
 * @since v3
 */
void ST7735_80x160::_displayCAZ() {

    // Debug timer
    // absolute_time_t st = to_us_since_boot(get_absolute_time());

    /* Mirror settings */

    // Initialing mirror offsets for AZ address window
    uint8_t _y_mirror_offset = 0;
    uint8_t _x_mirror_offset = 0;

    // Value for right mirror order (0 - _minAZ, 1 - _maxAZ)
    bool _orderAZ = 0;
    
    // Mirror configuring if mirrorring mode enabled (setting offsets and AZ order)
    if (_MIRRORED_VER) { _y_mirror_offset = H-1; _orderAZ = 1; }
    if (_MIRRORED_HOR) { _x_mirror_offset = W-1; }

    /* End of mirror settings */



    // Choosing every column
    for (uint8_t x = 0; x <= W-1; x++) {

        // If active zone existing (_minY < SCREEN_HEIGHT) - update column
        if (_colsAZ[x][0] < H) {
                
            // Current column (at x) active zone address window set with mirror ability
            // (orders for x mirror: NXM - 0, 1; XM - 1, 0)
            _setAdressWindow(abs(_x_mirror_offset - x),
                            abs(_y_mirror_offset - _colsAZ[x][_orderAZ]),
                            abs(_x_mirror_offset - x),
                            abs(_y_mirror_offset - _colsAZ[x][!_orderAZ]));

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

    // Debug timer
    // printf("CAZ done for %llu us\n", to_us_since_boot(get_absolute_time()) - st);
}

/**
 * @brief Private displaying method used by display() wrapper
 * 
 * Displaying mode - Row Active Zone
 * 
 * @since v3
 */
void ST7735_80x160::_displayRAZ() {

    // Debug timer
    // absolute_time_t st = to_us_since_boot(get_absolute_time());

    /* Mirror settings */

    // Initialing mirror offsets for AZ address window
    uint8_t _y_mirror_offset = 0;
    uint8_t _x_mirror_offset = 0;

    // Value for right mirror order (0 - minAZ, 1 - maxAZ)
    bool _orderAZ = 0;

    // Mirror configuring if mirrorring mode enabled (setting offsets and AZ order)
    if (_MIRRORED_VER) { _y_mirror_offset = H-1; }
    if (_MIRRORED_HOR) { _x_mirror_offset = W-1; _orderAZ = 1; }

    /* End of mirror settings */



    // Choosing every row
    for (uint8_t y = 0; y < H; y++) {

        // If active zone existing (_minX < SCREEN_WIDTH) - update row
        if (_rowsAZ[y][0] < W) {

            // Current row (at y) active zone address window set with mirror ability
            // (orders for y mirror: NXM - 0, 1; XM - 1, 0)
            _setAdressWindow(abs(_x_mirror_offset - _rowsAZ[y][_orderAZ]),
                            abs(_y_mirror_offset - y), 
                            abs(_x_mirror_offset - _rowsAZ[y][!_orderAZ]),
                            abs(_y_mirror_offset - y));
            
            // Choosing every pixel in active zone
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

    // Debug timer
    // printf("RAZ done for %llu us\n", to_us_since_boot(get_absolute_time()) - st);
}

/**
 * @brief Private displaying method used by display() wrapper
 * 
 * Displaying mode - Pixel Address Window
 * 
 * @since v3
 */
void ST7735_80x160::_displayPAW() {

    // Debug timer
    // absolute_time_t st = to_us_since_boot(get_absolute_time());

    /* Mirror settings */

    // Initializing mirror offsets by X and Y axis
    uint8_t _y_mirror_offset = 0;
    uint8_t _x_mirror_offset = 0;

    // Mirror configuring if mirrorring mode enabled (setting mirror offsets)
    if (_MIRRORED_VER) { _y_mirror_offset = H-1; }
    if (_MIRRORED_HOR) { _x_mirror_offset = W-1; }

    /* End of mirror settings */



    // Choosing every column
    for (uint8_t x = 0; x < W; x++) {

        // If column was changed by pixels... (_minY < SCREEN_HEIGHT)
        if (_colsAZ[x][0] < H) {
            
            // Choosing every pixel in Column Active Zone
            for (uint8_t y = _colsAZ[x][0]; y <= _colsAZ[x][1]; y++) {

                // If current pixel is new...
                if (_screen_buffer[x][y] != _delta_buffer[x][y]) {

                    // Setting one pixel address window with mirror offsets
                    _setAdressWindow(abs(x - _x_mirror_offset),
                                    abs(y - _y_mirror_offset),
                                    abs(x - _x_mirror_offset),
                                    abs(y - _y_mirror_offset));

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

    // Debug timer
    // printf("PAW done for %llu us\n", to_us_since_boot(get_absolute_time()) - st);
}

/**
 * @brief Private displaying method used by display() wrapper
 * 
 * Displaying mode - Tile displaying mode
 * 
 * @since v3
 */
void ST7735_80x160::_displayTiles() {

    // Debug timer
    // absolute_time_t st = to_us_since_boot(get_absolute_time());

    /* Mirror settings */

    // Initializing mirror offsets by X and Y axis
    uint8_t _y_mirror_offset = 0;
    uint8_t _x_mirror_offset = 0;

    // Initializing directions to move in buffer row/column order with mirroring
    int _dirY = 1;
    int _dirX = 1;

    // Mirror configuring if mirrorring mode enabled
    // (setting offsets and choosing opposite direction)
    if (_MIRRORED_VER) { _y_mirror_offset = H-1; _dirY = -1; }
    if (_MIRRORED_HOR) { _x_mirror_offset = W-1; _dirX = -1; }

    /* End of mirror settings */



    // Choosing every tile by X
    for (uint8_t tileX = 0; tileX < W / _tileSizeX; tileX++) {

        // Choosing every tile by Y
        for (uint8_t tileY = 0; tileY < H / _tileSizeY; tileY++) {

            // If tile was changed...
            if (_cgdTiles[tileX][tileY] == CHANGED) {

                // Initializing tile start position with mirror offset
                uint8_t _tileStartX = abs(tileX * _tileSizeX - _x_mirror_offset);
                uint8_t _tileStartY = abs(tileY * _tileSizeY - _y_mirror_offset);
                
                // Initializing tile end position (tileStart +- (tileSize - 1), becuse next tile starts on tileSize)
                uint8_t _tileEndX = abs(_tileStartX + ((_tileSizeX - 1) * _dirX));
                uint8_t _tileEndY = abs(_tileStartY + ((_tileSizeY - 1) * _dirY));

                // Sorting position for right address window and ordering
                if (_tileStartY > _tileEndY) swap(_tileStartY, _tileEndY); // if _MIRRORED_VER
                if (_tileStartX > _tileEndX) swap(_tileStartX, _tileEndX); // if _MIRRORED_HOR

                // Putting tile address window
                _setAdressWindow(_tileStartX,
                                _tileStartY,
                                _tileEndX,
                                _tileEndY);


                // Selecting every column in tile (!= let it moving in two directions and +1 for avoid uint8_t overflow by _dirX)
                for (uint8_t x = abs(_tileStartX - _x_mirror_offset)+1; x != abs(_tileEndX - _x_mirror_offset)+1 + _dirX; x += _dirX) {

                    // Decrementing x to avoid uint8_t overflow and minimize arithmetics
                    x -= 1;

                    // Selecting every pixel in tile column (with mirror offset, same as with columns)
                    for (uint8_t y = abs(_tileStartY - _y_mirror_offset)+1; y != abs(_tileEndY - _y_mirror_offset)+1 + _dirY; y += _dirY) {
                        
                        // Drawing all tile pixels with decremented position
                        _send_data16(_delta_buffer[x][y-1]);

                        // Replacing screen buffer with delta buffer (updated zone)
                        _screen_buffer[x][y-1] = _delta_buffer[x][y-1];
                    }

                    // Incrementing x back
                    x += 1;
                }

                // Resetting changed tile to default
                _cgdTiles[tileX][tileY] = NOT_CHANGED;
            }
        }
    }

    // Debug timer
    // printf("Tiles done for %llu us\n", to_us_since_boot(get_absolute_time()) - st);
}

/**
 * @brief Private displaying method used by display() wrapper
 * 
 * Displaying mode - Expandable Address Window
 * 
 * @since v3
 */
void ST7735_80x160::_displayExpAW() {

    // Debug timer
    // absolute_time_t st = to_us_since_boot(get_absolute_time());

    /* Mirror settings */

    // Initializing mirror offsets by X and Y axis
    uint8_t _y_mirror_offset = 0;
    uint8_t _x_mirror_offset = 0;

    // Initializing directions to move in buffer ExpAW part
    int _dirY = 1;
    int _dirX = 1;

    // Mirror configuring if mirrorring mode enabled
    // (setting offsets, choosing opposite direction and sorting verticles by swap)
    if (_MIRRORED_VER) { _y_mirror_offset = H-1; _dirY = -1; swap(_startY, _endY); }
    if (_MIRRORED_HOR) { _x_mirror_offset = W-1; _dirX = -1; swap(_startX, _endX); }

    /* End of mirror settings */



    // Putting AW to updated rectangle area with mirror offsets
    _setAdressWindow(abs(_startX - _x_mirror_offset),
                    abs(_startY - _y_mirror_offset),
                    abs(_endX - _x_mirror_offset),
                    abs(_endY - _y_mirror_offset));

    // Choosing every column in ExpAW (!= let cycle moving in two directions, +1 to avoid uint8_t overflow caused !=)
    for (uint8_t x = _startX+1; x != _endX+1 + _dirX; x += _dirX) {

        // Decrementing x to avoid uint8_t overflow
        x -= 1;

        // Choosing every pixel in column (same as x)
        for (uint8_t y = _startY+1; y != _endY+1 + _dirY; y += _dirY) {

            // Sending pixel from buffer to display
            _send_data16(_delta_buffer[x][y-1]);

            // Replacing screen buffer with delta buffer (updated zone)
            _screen_buffer[x][y-1] = _delta_buffer[x][y-1];
        }

        // Incrementing x back
        x += 1;
    }

    // Swapping verticles back (mirror wrong ExpAW positions fix)
    if (_MIRRORED_VER) swap(_startY, _endY);
    if (_MIRRORED_HOR) swap(_startX, _endX);

    // If screen changed without any mirroring... (mirror bad ExpAW positions fix)
    if (!_screenWasMirrored) {

        // Setting ExpAW positions to default
        _startX = W, _startY = H; // Lowest
        _endX = 0,   _endY = 0;   // Highest
    }
    
    // Setting screen mirrored bool to default
    // (sets ExpAW positions to default next time if mirror wasn't changed)
    _screenWasMirrored = false;

    // Debug timer
    // printf("ExpAW done for %llu us\n", to_us_since_boot(get_absolute_time()) - st);
}

/* End of private methods */



/* Public methods */

// --- Get data methods ---


/**
 * @brief Returns SCK (SCL on board) pin from class constructor
 * 
 * @since v1
*/
uint8_t ST7735_80x160::getPinSCK()      const  { return _PIN_SCK;  }
/**
 * @brief Returns MOSI (SDA on board) pin from class constructor
 * 
 * @since v1
*/
uint8_t ST7735_80x160::getPinMOSI()     const  { return _PIN_MOSI; }
/**
 * @brief Returns RST (RES on board) pin from class constructor
 * 
 * @since v1
*/
uint8_t ST7735_80x160::getPinRST()      const  { return _PIN_RST;  }
/**
 * @brief Returns DC pin from class constructor
 * 
 * @since v1
*/
uint8_t ST7735_80x160::getPinDC()       const  { return _PIN_DC;   }
/**
 * @brief Returns CS pin from class constructor
 * 
 * @since v1
*/
uint8_t ST7735_80x160::getPinCS()       const  { return _PIN_CS;   }
/**
 * @brief Returns BLK pin from class constructor
 * 
 * @since v1
*/
uint8_t ST7735_80x160::getPinBLK()      const  { return _PIN_BLK;  }

/**
 * @brief Returns display SPI port (*3987584 - spi0 or *4003968 - spi1)
 * 
 * @since v1
*/
spi_inst_t* ST7735_80x160::getSpiPort() const  { return _SPI_PORT; }
/**
 * @brief Returns current display channel frequency (default - 33MHz)
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
 * @brief Returns current vertical mirror mode
 * 
 * @since v3
*/
bool ST7735_80x160::getMirrorVer()      const  { return _MIRRORED_VER; }
/**
 * @brief Returns current horizontal mirror mode
 * 
 * @since v3
*/
bool ST7735_80x160::getMirrorHor()      const  { return _MIRRORED_HOR; }

/**
 *  @brief Returns pixel on screen in argument position
 * 
 * Returns pixel color in screen buffer, that fills only after display() method.
 * 
 * @since v3
*/
uint16_t ST7735_80x160::getPixel(uint8_t x, uint8_t y) const { return _screen_buffer[x][y]; }

/**
 * @brief Returns pixel on screen before it's displayed
 * 
 * Returns pixel color in delta buffer, that fills before display() with GFX methods.
 * 
 * @since v3
*/
uint16_t ST7735_80x160::getPixelDelta(uint8_t x, uint8_t y) const { return _delta_buffer[x][y]; }



// --- General methods ---

/**
 * @brief Configures display settings (SPI, BLK and CMD)
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
}

/**
 * @brief Show screen data on display
 * 
 * Method transfers screen buffer pixels address/color data to display,
 * 
 * buffer can be filled with drawPixel(), fillRect() or other graphics functions.
 * 
 * Displaying mode picks on with setDisplayingMode() method (check docs).
 * 
 * @since v1
 */
void ST7735_80x160::display() {

    // Calling private displaying methods choosed with current DM value
    switch (_DISPLAYING_MODE) {
        default:          _displayFullscreen(); break;  // Fullscreen mode
        case CAZ_MODE:    _displayCAZ();        break;  // Column Active Zone
        case RAZ_MODE:    _displayRAZ();        break;  // Row Active Zone
        case PAW_MODE:    _displayPAW();        break;  // Pixel Address Window
        case TILE_MODE:   _displayTiles();      break;  // Tile displaying mode
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
 * @param color fills up cleared area pixels (default = BLACK)
 * 
 * @since v1
 */
void ST7735_80x160::clearDisplay(uint16_t color) {

    // Address window with screen size
    _setAdressWindow(0, 0, W-1, H-1);

    // Choosing every column
    for (uint8_t x = 0; x < W; x++) {

        // Choosing every pixel in column
        for (uint8_t y = 0; y < H; y++) {

            // Sending pixel without a buffer
            _send_data16(color);

            // Clearing screen and delta buffers with arg color
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


// --- Settings methods ---

/**
 * @brief Sets SPI frequency
 * 
 * Reinitializes SPI port with argument baudrate.
 * 
 * @param baudrate data transfer rate in HZ 
*/
void ST7735_80x160::setSpiFrequency(uint32_t baudrate) {

    // Changing private frequency (or baudrate) property
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
 * @param brightness brightness rate from 0 to 100 percent
 * 
 * @note Wikipedia opinion - https://en.wikipedia.org/wiki/Gamma_correction
 * 
 * @since v3
 */
void ST7735_80x160::setBrightness(uint8_t brightness) {

    // If brightness control pin connected...
    if (_PIN_BLK) {
        
        // Setting private brightness percentage property to argument level
        _CUR_BRIGHTNESS = brightness;
        
        // Converting percentage to byte (0-100 -> 0-255)
        brightness = MAX_WRAP * brightness / 100;

        // Setting PWM brightness to argument level with gamma correction for linear brightness appear
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
 * @param inversion on true - color inversion enables, false - disables
 * 
 * @since v1
 */
void ST7735_80x160::setColorInversion(bool inversion) {

    // Setting private inversion property
    _COLOR_INVERSION = inversion;

    // Sending color inversion command (paradoxically, but colors inverted by default and on/off commands inverted too)
    if (_COLOR_INVERSION) _send_command(CMD_COLOR_INV_OFF);
    else                  _send_command(CMD_COLOR_INV_ON);
}

/**
 * @brief Sets displaying mode
 * 
 * Displaying mode - way to show/update screen information (check docs).
 *
 * Available modes:
 * 
 * +   0 - Fullscreen mode (stable, but slowest) (default) -> FULLSCREEN_MODE
 * 
 * +   1 - Column Active Zone (unstable, fastest in most cases) -> CAZ_MODE
 * 
 * +   2 - Row Active Zone (unstable, fastest in most cases) -> RAZ_MODE
 * 
 * +   3 - Pixel Address Window (quite unstable, fastest in most cases) -> PAW_MODE
 * 
 * +   4 - Tile displaying mode (quite unstable, fastest in few cases) -> TILE_MODE
 * 
 * +   5 - Expandable Adress Window (quite unstable, fastest in few cases) -> EXPAW_MODE
 * 
 * To choose them, use relevant *_MODE macroses.
 * 
 * @param displayingMode selected DM (macros from 0 to 4)
 * 
 * @since v1
 */
void ST7735_80x160::setDisplayingMode(int displayingMode) {

    // Changing private property _DISPLAYING_MODE used by display()
    _DISPLAYING_MODE = displayingMode;
}

/**
 * @brief Mirrors screen by horizontal/vertical directions
 * 
 * Sets screen XY axis (x - ver, y - hor) mirroring by 2 bool args (use mirror macroses down below).
 * 
 * @param verMirror X mirroring bool (NO_VER_MIRROR -> 0, VER_MIRROR -> 1)
 * @param horMirrror Y mirroring bool (NO_HOR_MIRROR -> 0, HOR_MIRROR -> 1)
 * 
 * @since v3
 */
void ST7735_80x160::setScreenMirror(bool verMirror, bool horMirror) {

    // If screen was mirrored - it available to update by display() method
    if (_MIRRORED_VER != verMirror || _MIRRORED_HOR != horMirror) { _screenWasMirrored = true; }

    // Changing private mirror properties used by display() wrapper
    _MIRRORED_VER = verMirror; // By X
    _MIRRORED_HOR = horMirror; // By Y
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
    if (x > W-1 || y > H-1 ) return;

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
 * Moves rectangle information to screen buffer.
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
void ST7735_80x160::fillRect(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint16_t color) {

    // Swapping verticles if they inversed
    if (x1 > x2) swap(x1, x2);
    if (y1 > y2) swap(y1, y2);

    // Filling rectangle area with pixels
    for (uint8_t x = x1; x <= x2; x++) {

        for (uint8_t y = y1; y <= y2; y++) {
            
            // Drawing pixel
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
 * Moves rectangle area with same X to screen buffer.
 * 
 * @param x V-line start X position
 * @param y V-line start Y position
 * @param length line length in pixels
 * @param color line color
 * 
 * @since v2
 */
void ST7735_80x160::drawVLine(uint8_t x, uint8_t y, uint8_t length, uint16_t color) {

    // Actually filling rectangle with 1 pixel width
    fillRect(x, y, x, y + length - 1, color);
}

/**
 * @brief Draws horizontal line
 * 
 * Moves rectangle area with same Y to screen buffer.
 * 
 * @param x H-line start X position
 * @param y H-line start Y position
 * @param length line length in pixels
 * @param color line color
 * 
 * @since v2
 */
void ST7735_80x160::drawHLine(uint8_t x, uint8_t y, uint8_t length, uint16_t color) {

    // Actually filling rectangle with 1 pixel height
    fillRect(x, y, x + length - 1, y, color);
}

/**
 * @brief Draws rectangle outline
 * 
 * Moves 2 V-lines and 2 H-lines at rectangle verticles to screen buffer.
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

