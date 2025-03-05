# st7735_80x160
Free open-source GFX library on C++ for displays, controlled by RP2040/RP2350 MCU.

Supported displays: ST7735 80x160 IPS SPI.



## Installing 📥
Open your project and go to libraries directory (create if there's no libs):

```bash
mkdir libsDir
cd libsDir
git clone https://github.com/PlAzMoR/st7735_80x160.git
```

If you use CMake or Raspberry Pi Pico Project (VSCode Extension), add this lines to your `CMakeLists.txt` file:

```cmake
# Adding GFX subdirectory
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/libsDir/st7735_80x160 st7735_80x160)

# Adding libraries
target_link_libraries(yourUseableProject
    ...
    st7735_80x160
    ...
    )
```

... and include it to executable file (`main.cpp`):

```cpp
// Including GFX library header
#include "st7735.h"
...
```
After you define Display Class object by connected pins, initialize by *displayObj.init()*, configure by *displayObj.set**, and have fun!


## Testing ⚙️
There's some examples you can find in relevant `../examples` directory: GFX Test and DM Speedtest.


## License 📄
This OSS project protected by MIT License :)
