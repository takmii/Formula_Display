#include <variables.h>

#define ILI9341_DRIVER



const __u8 TFT_CS=27;
//const __u8 TFT_SD_CS=25;    
const __u8 TFT_DC=26;
const __u8 TFT_RST=-1;
const __u8 TFT_MISO=12;
const __u8 TFT_MOSI=13;
const __u8 TFT_SCLK=14;

#define SPI_FREQUENCY  40000000
#define SPI_READ_FREQUENCY  20000000
#define SPI_TOUCH_FREQUENCY  2500000

// Optional DMA for pushImage and pushColors
#define SUPPORT_TRANSACTIONS
#define USE_HSPI_PORT