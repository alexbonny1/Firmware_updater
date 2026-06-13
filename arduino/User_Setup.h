#define USER_SETUP_INFO "User_Setup"

// ##################################################################################
// Section 1. Driver
// ##################################################################################

#define ILI9488_DRIVER

// ##################################################################################
// Section 2. Pins ESP32
// ##################################################################################

#define TFT_MISO 19
#define TFT_MOSI 23
#define TFT_SCLK 18
#define TFT_CS   15
#define TFT_DC    2
#define TFT_RST   4

#define TFT_BL   32
#define TFT_BACKLIGHT_ON HIGH

// ##################################################################################
// Section 3. Fonts
// ##################################################################################

#define LOAD_GLCD
#define LOAD_FONT2
#define LOAD_FONT4
#define LOAD_FONT6
#define LOAD_FONT7
#define LOAD_FONT8
#define LOAD_GFXFF
#define SMOOTH_FONT

// ##################################################################################
// Section 4. SPI frequency
// ##################################################################################

#define SPI_FREQUENCY  27000000
#define SPI_READ_FREQUENCY  20000000
#define SPI_TOUCH_FREQUENCY  2500000
