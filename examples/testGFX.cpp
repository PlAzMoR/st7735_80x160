/**
 * General ST7735 library example
 * 
 * GFX test includes lines, pongs and pointed mark.
 * 
 * @since v2
*/

// Including ST7735 library
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



// ----- GFX test functions -----

// Draws lines from center to all directions
void testLines(int step = 10, int breakpoint = 50) {

    // Current line color
    int c = 0;

    while (step <= breakpoint) {
        
        // Drawing lines from center with step
        for (int i = 0; i < lcd.W; i += step)  lcd.drawLine(79, 40, i, lcd.H-1, colors[c]);
        for (int i = 0; i <= lcd.H; i += step) lcd.drawLine(80, 40, lcd.W-1, i, colors[c]);
        for (int i = lcd.W; i > 0; i -= step)  lcd.drawLine(80, 39, i, 0, colors[c]);
        for (int i = lcd.H; i >= 0; i -= step) lcd.drawLine(79, 39, 0, i, colors[c]);

        // Increasing step
        step += 5;
        
        // Next color
        c == 6 ? c = 0 : c++;

        // Display lines
        lcd.display();

        // Delay after show
        sleep_ms(400);

        // Clean display
        lcd.clearDisplay();
    }
}

// Throws 4 colored pongs
void testPongs(int targetClCount = 100) {

    // Pong parameters
    struct Pong {
        int x; // Current X position (start - rad)
        int y; // Current Y position (start - rad)
        int rad; // Pong radius
        int dirX; // Incrementing direction by X (1 or -1)
        int dirY; // Incrementing direction by Y (1 or -1)
        uint16_t color; // Pong color
    };

    // Pongs amount
    int pongsAmount = 4;

    // Pongs radiuses
    int r1 = 10;
    int r2 = 6;
    int r3 = 3;
    int r4 = 14;

    // Pongs initializing
    Pong pong1 = {r1, r1, r1, 1, 1, YELLOW};
    Pong pong2 = {lcd.W-1-r2, lcd.H-1-r2, r2, -1, -1, RED};
    Pong pong3 = {r3, lcd.H-1-r3, r3, 1, -1, GREEN};
    Pong pong4 = {lcd.W-1-r4, r4, r4, -1, 1, MAGENTA};

    // Array of pongs
    Pong pongs[pongsAmount] = {pong1, pong2, pong3, pong4};
    
    // Collision counter to end test function
    int clCount = 0;

    while (clCount <= targetClCount) {

        // Drawing & collision handler
        for (int i = 0; i < pongsAmount; i++) {

            // Drawing pongs
            lcd.fillCircle(pongs[i].x, pongs[i].y, pongs[i].rad, pongs[i].color);

            // X collision check
            if (pongs[i].x == 0 + pongs[i].rad)             {pongs[i].dirX = 1; clCount++;}
            else if (pongs[i].x == lcd.W-1 - pongs[i].rad)  {pongs[i].dirX = -1; clCount++;}

            // Y collision check
            if (pongs[i].y == 0 + pongs[i].rad)             {pongs[i].dirY = 1; clCount++;}
            else if (pongs[i].y == lcd.H-1 - pongs[i].rad)  {pongs[i].dirY = -1; clCount++;}

            // Increasing position by dirs
            pongs[i].x += pongs[i].dirX;
            pongs[i].y += pongs[i].dirY;
        }

        // Display pongs
        lcd.display();

        // Clean screen
        lcd.clearDisplay();
    }
}

// Draws mark on center
void testMark(int breakpoint = 10) {

    // Current mark radius
    int rad = 25;

    // Counter that ends function
    int endCount = 0;

    while (endCount <= breakpoint) {
        
        // Filling pointing triangles
        lcd.fillTriangle(10, 20, 40, 40, 10, 60, colors[glColor]);
        lcd.fillTriangle(lcd.W-1-10, 20, lcd.W-1-40, 40, lcd.W-1-10, 60, colors[glColor]);
        
        // Drawing mark
        lcd.drawCircle(80, 40, rad, colors[glColor]);
        lcd.drawCircle(80, 40, rad+1, colors[glColor]);

        // Incrementing radius if it not reach nax value
        if (rad == 2) { rad = 25; endCount++; }
        else rad--;

        // Show mark on display
        lcd.display();

        // Delay after decreasing mark radius
        sleep_ms(10);

        // Cleaning display
        lcd.clearDisplay();
    }
}


// ----- End of GFX test functions -----



// Executing cycle
int main() {

    // Initializing serial input/output
    stdio_init_all();

    // Display setup
    lcd.init();
    // Cleaning screen from noise and old data
    lcd.clearDisplayForce(BLACK);


    // Configuring settings as default

    // Display Mode
    lcd.setDisplayingMode(FULLSCREEN_MODE);
    // Mirroring
    lcd.setScreenMirror(NO_VER_MIRROR, NO_HOR_MIRROR);
    // Brightness
    lcd.setBrightness(BRIGHTNESS_MAX);
    // Frequency
    lcd.setSpiFrequency(33 * MHZ);
    // Color inversion
    lcd.setColorInversion(INVERSION_OFF);

    // Infinite loop
    while (true) {
        
        // Lines test GFX
        testLines();

        // Pongs test GFX
        testPongs();

        // Mark test GFX
        testMark();

        // Moving to next global color
        glColor == 6 ? glColor = 0 : glColor++;
    }
}   

