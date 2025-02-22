/**
 * General ST7735 library example
 * 
 * Speedtest for different Displaying Modes
 * 
 * @since v3
*/

// Including ST7735 library header
#include "st7735.h"

// Including standard RPi Pico libraries
#include "pico/stdlib.h"



// Pins configuration
#define PIN_SCK    18 // SCL
#define PIN_MOSI   19 // SDA
#define PIN_RST    20 // RES
#define PIN_CS     17
#define PIN_DC     21
#define PIN_BLK    22

// Frequency configuration
#define SPI_FREQ   33 * MHZ



// Initializing display object
ST7735_80x160 lcd(PIN_SCK, PIN_MOSI, PIN_CS, PIN_DC, PIN_RST, PIN_BLK, SPI_FREQ);

// Colors that used in test
uint16_t colors[7] = {RED, GREEN, BLUE, CYAN, YELLOW, MAGENTA, WHITE};
// Global color value - index in colors[] array
int glColor = 0;

// Displaying modes array
int DM[6] = {FULLSCREEN_MODE, CAZ_MODE, RAZ_MODE, PAW_MODE, TILE_MODE, EXPAW_MODE};
const char* DM_str[6] = {"Fullscreen mode", "Column Active Zone", "Row Active Zone",
                        "Pixel Address Window", "Tile Displaying mode", "Expandable Window"};
// Global DM - index in DM[] array
int glDM = 0;


// --- Speedtest GFX functions ---

// Speedtest by one pixel at {0, 0}
void onePixelST(bool printTestName = 1, int times = 25) {

    // Initializing time accumulator for averaging execution time
    absolute_time_t timeAccum = 0;

    // Local color for updating pixel with new color
    int localColor = 0;

    if (printTestName) printf("One pixel at {0, 0} %d times speedtest\n", times);

    // Main speedtest cycle
    for (int c = 0; c < times; c++) {

        // Drawing only one pixel
        lcd.drawPixel(0, 0, colors[localColor]);

        // Timer start
        absolute_time_t start = to_us_since_boot(get_absolute_time());

        lcd.display();

        // Updating time accumulator
        timeAccum += to_us_since_boot(get_absolute_time()) - start;

        // Updating color
        localColor == 6 ? localColor = 0 : localColor++;
    }

    // Showing result
    printf("Average execution time: %llu us\n", timeAccum / times);

    // Clearing display
    lcd.fillScreen(BLACK);
}

// Speedtest by filled rectangle 20x20 with {30, 30} start point
void fillRectST(bool printTestName = 1, int times = 25) {

    // Initializing time accumulator for averaging execution time
    absolute_time_t timeAccum = 0;

    // Local color for updating pixel with new color
    int localColor = 0;

    if (printTestName) printf("Filled rectangle at {30, 30} with 20x20 size %d times speedtest\n", times);

    // Main speedtest cycle
    for (int c = 0; c < times; c++) {

        // Drawing filled rectangle
        lcd.fillRect(30, 30, 50, 50, colors[localColor]);

        // Timer start
        absolute_time_t start = to_us_since_boot(get_absolute_time());

        lcd.display();

        // Updating time accumulator
        timeAccum += to_us_since_boot(get_absolute_time()) - start;

        // Updating color
        localColor == 6 ? localColor = 0 : localColor++;
    }

    // Showing result
    printf("Average execution time: %llu us\n", timeAccum / times);

    // Clearing display
    lcd.fillScreen(BLACK);
}

// Speedtest by pixels on screen borders
void borderPixelsST(bool printTestName = 1, int times = 25) {

    // Initializing time accumulator for averaging execution time
    absolute_time_t timeAccum = 0;

    // Local color for updating pixel with new color
    int localColor = 0;

    if (printTestName) printf("Screen borders pixels %d times speedtest\n", times);

    // Main speedtest cycle
    for (int c = 0; c < times; c++) {

        // Drawing pixels on borders
        lcd.drawPixel(0,             0, colors[localColor]);
        lcd.drawPixel(lcd.W-1,       0, colors[localColor]);
        lcd.drawPixel(0,       lcd.H-1, colors[localColor]);
        lcd.drawPixel(lcd.W-1, lcd.H-1, colors[localColor]);

        // Timer start
        absolute_time_t start = to_us_since_boot(get_absolute_time());

        lcd.display();

        // Updating time accumulator
        timeAccum += to_us_since_boot(get_absolute_time()) - start;

        // Updating color
        localColor == 6 ? localColor = 0 : localColor++;
    }

    // Showing result
    printf("Average execution time: %llu us\n", timeAccum / times);

    // Clearing display
    lcd.fillScreen(BLACK);
}

// Speedtest by border pixels and filled rectangle
void pixelsRectST(bool printTestName = 1, int times = 25) {

    // Initializing time accumulator for averaging execution time
    absolute_time_t timeAccum = 0;

    // Local color for updating pixel with new color
    int localColor = 0;

    if (printTestName) printf("Screen borders pixels and filled rectangle 20x20 %d times speedtest\n", times);

    // Main speedtest cycle
    for (int c = 0; c < times; c++) {

        // Drawing pixels on borders
        lcd.drawPixel(0,             0, colors[localColor]);
        lcd.drawPixel(lcd.W-1,       0, colors[localColor]);
        lcd.drawPixel(0,       lcd.H-1, colors[localColor]);
        lcd.drawPixel(lcd.W-1, lcd.H-1, colors[localColor]);
        lcd.fillRect(30, 30, 50, 50, colors[localColor]);

        // Timer start
        absolute_time_t start = to_us_since_boot(get_absolute_time());

        lcd.display();

        // Updating time accumulator
        timeAccum += to_us_since_boot(get_absolute_time()) - start;

        // Updating color
        localColor == 6 ? localColor = 0 : localColor++;
    }

    // Showing result
    printf("Average execution time: %llu us\n", timeAccum / times);

    // Clearing display
    lcd.fillScreen(BLACK);
}

// Speedtest by filling screen
void fillScreenST(bool printTestName = 1, int times = 25) {

    // Initializing time accumulator for averaging execution time
    absolute_time_t timeAccum = 0;

    // Local color for updating pixel with new color
    int localColor = 0;

    if (printTestName) printf("Filling screen %d times speedtest\n", times);

    // Main speedtest cycle
    for (int c = 0; c < times; c++) {

        // Drawing pixels on borders
        lcd.fillScreen(colors[localColor]);

        // Timer start
        absolute_time_t start = to_us_since_boot(get_absolute_time());

        lcd.display();

        // Updating time accumulator
        timeAccum += to_us_since_boot(get_absolute_time()) - start;

        // Updating color
        localColor == 6 ? localColor = 0 : localColor++;
    }

    // Showing result
    printf("Average execution time: %llu us\n", timeAccum / times);

    // Clearing display
    lcd.fillScreen(BLACK);
}

// Speedtest by 2 H-lines on start and end
void HLinesST(bool printTestName = 1, int times = 25) {

    // Initializing time accumulator for averaging execution time
    absolute_time_t timeAccum = 0;

    // Local color for updating pixel with new color
    int localColor = 0;

    if (printTestName) printf("2 Width-sized H-lines %d times speedtest\n", times);

    // Main speedtest cycle
    for (int c = 0; c < times; c++) {

        // Drawing pixels on borders
        lcd.drawHLine(0,       0, lcd.W-1, colors[localColor]);
        lcd.drawHLine(0, lcd.H-1, lcd.W-1, colors[localColor]);

        // Timer start
        absolute_time_t start = to_us_since_boot(get_absolute_time());

        lcd.display();

        // Updating time accumulator
        timeAccum += to_us_since_boot(get_absolute_time()) - start;

        // Updating color
        localColor == 6 ? localColor = 0 : localColor++;
    }

    // Showing result
    printf("Average execution time: %llu us\n", timeAccum / times);

    // Clearing display
    lcd.fillScreen(BLACK);
}

// Speedtest by 2 V-lines on start and end
void VLinesST(bool printTestName = 1, int times = 25) {

    // Initializing time accumulator for averaging execution time
    absolute_time_t timeAccum = 0;

    // Local color for updating pixel with new color
    int localColor = 0;

    if (printTestName) printf("2 Height-sized V-lines %d times speedtest\n", times);

    // Main speedtest cycle
    for (int c = 0; c < times; c++) {

        // Drawing pixels on borders
        lcd.drawVLine(0,       0, lcd.H-1, colors[localColor]);
        lcd.drawVLine(lcd.W-1, 0, lcd.H-1, colors[localColor]);

        // Timer start
        absolute_time_t start = to_us_since_boot(get_absolute_time());

        lcd.display();

        // Updating time accumulator
        timeAccum += to_us_since_boot(get_absolute_time()) - start;

        // Updating color
        localColor == 6 ? localColor = 0 : localColor++;
    }

    // Showing result
    printf("Average execution time: %llu us\n", timeAccum / times);

    // Clearing display
    lcd.fillScreen(BLACK);
}

// --- End of speedtest GFX functions ---



// Executing cycle
int main() {

    // Initializing serial input/output
    stdio_init_all();

    // Display setup
    lcd.init();

    // Cleaning screen from noise and old data
    lcd.clearDisplayForce();

    // Infinite loop
    while (true) {

        // Setting & printing DM
        lcd.setDisplayingMode(DM[glDM]);
        printf("\nCurrent displaying mode: %s\n", DM_str[glDM]);

        // Test 1
        onePixelST(0);
        sleep_ms(1000);

        // Test 2
        fillRectST(0);
        sleep_ms(1000);

        // Test 3
        borderPixelsST(0);
        sleep_ms(1000);

        // Test 4
        pixelsRectST(0);
        sleep_ms(1000);

        // Test 5
        fillScreenST(0);
        sleep_ms(1000);

        // Test 6
        HLinesST(0);
        sleep_ms(1000);

        // Test 7
        VLinesST(0);
        sleep_ms(1000);

        // Moving to next global color
        glColor == 6 ? glColor = 0 : glColor++;
        // Moving to next DM
        glDM == 5 ? glDM = 0 : glDM++;
    }
}
