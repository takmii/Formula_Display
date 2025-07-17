// Setup for the ESP32 S2 with ILI9341 display
// Note SPI DMA with ESP32 S2 is not currently supported

// See SetupX_Template.h for all options available
#define ILI9342_DRIVER

#define TFT_WIDTH  320
#define TFT_HEIGHT 240

                    // Typical board default pins
#define TFT_CS   27 //     10 or 34

#define TFT_MOSI 13 //     11 or 35
#define TFT_SCLK 14 //     12 or 36
#define TFT_MISO 12 //     13 or 37

#define TFT_DC   26
#define TFT_RST  25

#define TOUCH_CS -1

//#define TOUCH_CS 16 // Optional for touch screen

#define LOAD_GLCD
#define LOAD_FONT2
#define LOAD_FONT4
#define LOAD_FONT6
#define LOAD_FONT7
#define LOAD_FONT8
//#define LOAD_GFXFF

//#define SMOOTH_FONT

// Optional DMA for pushImage and pushColors
#define SUPPORT_TRANSACTIONS
#define USE_HSPI_PORT

//#define SPI_FREQUENCY  27000000
#define SPI_FREQUENCY  27000000   // Maximum for ILI9341

#define SPI_READ_FREQUENCY  20000000

#define SPI_TOUCH_FREQUENCY 2500000

