#include "st7735.h"

#include "pico/stdlib.h"


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

// uint8_t abs(int val) { return val < 0 ? -val : val; }s

void ST7735_80x160::_write_spi(uint8_t bus_data) {
    gpio_put(_PIN_CS, 0);

    spi_write_blocking(_SPI_PORT, &bus_data, 1);

    gpio_put(_PIN_CS, 1);
}

void ST7735_80x160::_send_data(uint8_t data8) {
    gpio_put(_PIN_DC, 1);

    _write_spi(data8);
}

void ST7735_80x160::_send_data16(uint16_t data16) {
    gpio_put(_PIN_DC, 1);

    _write_spi(data16 >> 8);
    _write_spi(data16 & 0xFF);
}

void ST7735_80x160::_send_command(uint8_t command) {
    gpio_put(_PIN_DC, 0);

    _write_spi(command);
}

void ST7735_80x160::_reset_display() {
    gpio_put(_PIN_RST, 0);
    sleep_ms(50);
    gpio_put(_PIN_RST, 1);
    sleep_ms(50);
}

void ST7735_80x160::_spi_setup() {

    // Pins configuration
    gpio_init(_PIN_SCK); gpio_set_function(_PIN_SCK, GPIO_FUNC_SPI);            // SCK pin
    gpio_init(_PIN_MOSI); gpio_set_function(_PIN_MOSI, GPIO_FUNC_SPI);          // MOSI pin

    gpio_init(_PIN_CS); gpio_set_dir(_PIN_CS, GPIO_OUT); gpio_put(_PIN_CS, 1);  // CS pin
    gpio_init(_PIN_DC); gpio_set_dir(_PIN_DC, GPIO_OUT);                        // DC pin
    gpio_init(_PIN_RST); gpio_set_dir(_PIN_RST, GPIO_OUT);                      // RST pin

    // Initializing SPI port of LCD
    spi_init(_SPI_PORT, _SPI_FREQ);
}

void ST7735_80x160::_cmd_setup() {
    _send_command(CMD_SFW_RESET);
    sleep_ms(150);

    _send_command(CMD_SLEEP_OUT);
    sleep_ms(500);

    if (_COLOR_INVERSION == false) { // Default colors in st7735 are inverted
        _send_command(CMD_COLOR_INV_ON); // If no inversion, we return to normal colors
    }

    _send_command(CMD_DISPLAY_ADR_DIR);
    _send_data(0x08); // 0x08 - default order (RGB)

    _send_command(CMD_COLOR_MODE_SET);
    _send_data(0x05); // 0x05 - 16 bit/pixel (RGB565)

    _send_command(CMD_DISPLAY_ON);
}

// Setting drawing area
void ST7735_80x160::_setAdressWindow(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2) {
    _send_command(CMD_COL_ADR_SET); // Column address configuration
    _send_data(0x00); _send_data(y1 + _OFFSET_Y);
    _send_data(0x00); _send_data(y2 + _OFFSET_Y);

    _send_command(CMD_ROW_ADR_SET); // Row address configuration
    _send_data(0x00); _send_data(x1 + _OFFSET_X);
    _send_data(0x00); _send_data(x2 + _OFFSET_X);

    _send_command(CMD_MEM_WRITE); // Now we can write data
}



uint8_t ST7735_80x160::getPinSCK() const { return _PIN_SCK; }
uint8_t ST7735_80x160::getPinMOSI() const { return _PIN_MOSI; }
uint8_t ST7735_80x160::getPinCS() const { return _PIN_CS; }
uint8_t ST7735_80x160::getPinDC() const { return _PIN_DC; }
uint8_t ST7735_80x160::getPinRST() const { return _PIN_RST; }
uint8_t ST7735_80x160::getPinBLK() const { return _PIN_BLK; }
spi_inst_t* ST7735_80x160::getSpiPort() const { return _SPI_PORT; }
uint32_t ST7735_80x160::getSpiFreq() const { return _SPI_FREQ; }



// Applying display settings
void ST7735_80x160::init() {

    // LCD SPI setup
    _spi_setup();

    // Resetting display
    _reset_display();

    // LCD CMD setup
    _cmd_setup();
    
    sleep_ms(50);
}

void ST7735_80x160::setColorInversion(bool inversion) {
    if (inversion) _send_command(CMD_COLOR_INV_OFF);
    else _send_command(CMD_COLOR_INV_ON);
}

void ST7735_80x160::setDisplayingMode(bool displayingMode) {
    _ROW_DISPLAYING = displayingMode;
}


void ST7735_80x160::drawPixel(uint8_t x, uint8_t y, uint16_t color) {

    // Out of bounds error handler
    if (x > 159 || y > 79) return;

    // Adding one pixel (color with address)
    if (_screen_buffer[x][y] != color) {

        // Updating columns active (changed) zone
        if (_cndCols[x][0] > y) _cndCols[x][0] = y;
        if (_cndCols[x][1] < y) _cndCols[x][1] = y;

        if (_cndRows[y][0] > x) _cndRows[y][0] = x;
        if (_cndRows[y][1] < x) _cndRows[y][1] = x;

        // printf("Col %d y - %d: active zone %d - %d, color 0x%x\n", x, y, _cndCols[x][0], _cndCols[x][1], color);
    }
    
    // Moving color to buffer
    _screen_buffer[x][y] = color;
}

void ST7735_80x160::fillRect(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint16_t color) {

    // Adding pixel colors and addresses
    for (uint8_t x = x1; x <= x2; x++) {
        for (uint8_t y = y1; y <= y2; y++) {
            // _send_data16(color);
            drawPixel(x, y, color);
        }
    }
}

void ST7735_80x160::fillScreen(uint16_t color) {

    // Filling rectangle with screen size
    fillRect(0, 0, W-1, H-1, color);
}

/*void ST7735_80x160::display() {

    absolute_time_t st = to_ms_since_boot(get_absolute_time());
    // Moving _screen_buffer to display
    for (int x = 0; x < W; x++) {

        // If active zone existing (_minY < 80) - update column
        if (_cndCols[x][0] < 80) {
            
            // Current column (at x) active zone address window set
            _setAdressWindow(x, _cndCols[x][0], x, _cndCols[x][1]);

            for (int y = _cndCols[x][0]; y <= _cndCols[x][1]; y++) {

                // Optimization change check
                // if (_screen_buffer[x][y] != _prev_screen_buffer[x][y]) {
                    
                    // Sending pixel from buffer
                    _send_data16(_screen_buffer[x][y]);
                    // printf("Pixel was sent: x %d y %d clr 0x%x\n", x, y, _screen_buffer[x][y]);

                    // Update previous buffer after send
                    // _prev_screen_buffer[x][y] = _screen_buffer[x][y];
                // }
                // else _send_data16(_prev_screen_buffer[x][y]);
            }
        }
        _cndCols[x][0] = 80;
        _cndCols[x][1] = 0;
    }
    printf("Done for %llu ms\n", to_ms_since_boot(get_absolute_time()) - st);
}*/

void ST7735_80x160::display(bool mirrorByX, bool mirrorByY) {

    uint8_t _x_mirror_offset = 0;
    uint8_t _y_mirror_offset = 0;

    bool _cgdOrder = 0;

    absolute_time_t st = to_ms_since_boot(get_absolute_time());

    // Use displaying by rows active zone...
    if (_ROW_DISPLAYING) {

    if (mirrorByX) { _x_mirror_offset = H-1; }
    if (mirrorByY) { _y_mirror_offset = W-1; _cgdOrder = 1;}

        // Choosing every row
        for (uint8_t y = 0; y < H; y++) {

            // If active zone existing (_minX < 160) - update column
            if (_cndRows[y][0] < 160) {

                // Current row (at y) active zone address window set
                _setAdressWindow(abs(_y_mirror_offset - _cndRows[y][_cgdOrder]),
                                abs(_x_mirror_offset - y), 
                                abs(_y_mirror_offset - _cndRows[y][!_cgdOrder]),
                                abs(_x_mirror_offset - y));

                printf("x1 - %d, x2 - %d\n", _cndRows[y][0], _cndRows[y][1]);
                
                for (uint8_t x = _cndRows[y][0]; x <= _cndRows[y][1]; x++) {

                    // Sending pixel from buffer to display
                    _send_data16(_screen_buffer[x][y]);
                }
            }

            // Set active zone to default values
            _cndRows[y][0] = 160;
            _cndRows[y][1] = 0;
        }
    }
    else { /* In case displaying by cols active zone */

    if (mirrorByX) { _x_mirror_offset = H-1; _cgdOrder = 1; }
    if (mirrorByY) { _y_mirror_offset = W-1;}

        // Choosing every column
        for (uint8_t x = 0; x < W; x++) {

            // If active zone existing (_minY < 80) - update column
            if (_cndCols[x][0] < 80) {
                
                /* Current column (at x) active zone address window set with mirror ability
                (orders for x mirror: NXM - 0, 1; XM - 1, 0) */
                _setAdressWindow(abs(_y_mirror_offset - x),
                                abs(_x_mirror_offset - _cndCols[x][_cgdOrder]),
                                abs(_y_mirror_offset - x),
                                abs(_x_mirror_offset - _cndCols[x][!_cgdOrder]));

                for (uint8_t y = _cndCols[x][0]; y <= _cndCols[x][1]; y++) {

                    // Sending pixel from buffer to display
                    _send_data16(_screen_buffer[x][y]);
                }
            }

            // Set active zone to default values
            _cndCols[x][0] = 80;
            _cndCols[x][1] = 0;
        }
    }
    printf("Done for %llu ms\n", to_ms_since_boot(get_absolute_time()) - st);
}

void ST7735_80x160::clearDisplay(uint16_t color) {

    _setAdressWindow(0, 0, W-1, H-1);

    for (uint8_t x = 0; x < W; x++) {

        for (uint8_t y = 0; y < H; y++) {
            _send_data16(color);

            _cndRows[y][0] = 160;
            _cndRows[y][1] = 0;
        }

        _cndCols[x][0] = 80;
        _cndCols[x][1] = 0;
    }
}

