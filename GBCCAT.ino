
#include "loader.h"
#include "mem.h"
#include "rtc.h"
#include "emu.h"
#include "sys.h"

#include "sound.h"

#define INCLUDED_LIBRARIES true //better enable this feature!                           

#define LCD_ENABLED true
#define COMPOSITE_VIDEO_ENABLED true  //Do not disable! it also disable ADC.
#define KEYBOARD_ENABLED true
#define SOUND_ENABLED true
#define BLUETOOTH_ENABLED false //experimental.

#define DEBUG true       //Serial debugging enable.
#define DEBUGEXTRA false //Extra Serial debugging enable.

#define I2S_MODULE_INSTALLED false //we do not use I2S PCM5102A module (= PWM direct output)

//********************************************************************************

//KEY BUTTONS PINS:
#define PIN_UP     39  //SVN
#define PIN_DOWN   35  //IO35
#define PIN_LEFT   32 /// 36  //SVP
#define PIN_RIGHT  12  //TDI => Do not install 330R resistor!!!
#define PIN_A      2   //IO2
#define PIN_B      14  //TMS
#define PIN_START  15  //TDO
#define PIN_SELECT 13  //TCK

///!!! do not forget 1KOHM resistors
#define KEYBOARD_DATA 4  /// ---[ 1K ]--- // -D
#define KEYBOARD_CLK 0   /// ---[ 1K ]--- // +D

//COMPOSITE_VIDEO: - //DAC_GPIO25_CHANNEL or DAC_GPIO26_CHANNEL

#define VIDEO_OUT (DAC_GPIO26_CHANNEL)

//AUDIO DIRECT OUTPUT:
#define CONFIG_LEFT_CHANNEL_GPIO 25
#define CONFIG_RIGHT_CHANNEL_GPIO 27

//AUDIO_i2S:
#define I2S_DOUT (GPIO_NUM_25) //DIN
#define I2S_BCLK (GPIO_NUM_27) //BCK
#define I2S_LRC  (GPIO_NUM_32) //LCK
#define I2S_DI_IO -1           //Not Used

//LCD_ST7789:
#define TFT_CS   -1  // define chip select pin
#define TFT_DC    5  // define data/command pin
#define TFT_RST  19  
#define TFT_MOSI 23  // Data out (SDA) //better not change
#define TFT_SCLK 18  // Clock out (SCL) //better not change

//micro_SD_Card: //GPIO16 and GPIO17 can not use on WROVER
#define SOFTSD_MOSI_PIN 33
#define SOFTSD_MISO_PIN 22
#define SOFTSD_SCK_PIN  21
#define SD_CS_PIN -1     //not used

//Oscilloscope INPUT:
#define ADC_CHANNEL   ADC1_CHANNEL_6 // GPIO34

//********************************************************************************
#include "pwm_audio.h"

pwm_audio_config_t pac;
//********************************************************************************

//********************************************************************************
//MAIN LIBRARIES:

///#include "esp_wifi.h" 
///#include "esp_bt.h"

#include <esp_task_wdt.h>
#include "esp_types.h"
#include "esp_heap_caps.h"
#include "esp_attr.h"
#include "esp_intr_alloc.h"
#include "esp_err.h"
#include "soc/gpio_reg.h"
#include "soc/rtc.h"
#include "soc/soc.h"
#include "soc/i2s_struct.h"
#include "soc/i2s_reg.h"
#include "soc/ledc_struct.h"
#include "soc/rtc_io_reg.h"
#include "soc/io_mux_reg.h"
#include "rom/gpio.h"
#include "rom/lldesc.h"
#include "driver/periph_ctrl.h"
#include "driver/dac.h"
#include "driver/gpio.h"
#if SOUND_ENABLED
#include "driver/i2s.h"
#endif

//LIBRARIES:
///#include "Arduino.h"

//********************************************************************************
//********************************************************************************


//********************************************************************************
//VARIABLES:

//Allocated MEMORY variables:
uint8_t* SCREENMEMORY[256 + 1];  //256*256 bytes + 256 offset
uint16_t SCREENBUFFER[256];  //512 bytes

uint32_t PSRAMSIZE = 0;
uint8_t* PSRAM;


//COMPOSITE_VIDEO_OUT
#if COMPOSITE_VIDEO_ENABLED
#include "src/compositevideo/video.h"
#endif

//AUDIO_i2S:
//********************************************************************************
#if INCLUDED_LIBRARIES

//MODIFIED INCLUDED_LIBRARIES in "/src/" folder

//LCD_ST7789:
#include <SPI.h>
#include "src/Adafruit-GFX-Library/Adafruit_GFX.h"    // Core graphics library
#include "src/Adafruit-ST7735-Library/Adafruit_ST7789.h"    // Core graphics library

//micro_SD_Card:
#include "src/sdfat-beta/src/SdFat.h" // Use SDFAT-beta...

//AUDIO_i2S:
#if SOUND_ENABLED
///#include "driver/i2s.h"
#endif

//player LIBs
#include "src/ESP32-audioI2S/src/Audio.h" // Use SDFAT-beta...

//--------------------------------------------------------------------------------
///#include "arduinoFFT.h" // Standard Arduino FFT library
#include "src/arduinoFFT/src/arduinoFFT.h" // Standard Arduino FFT library

//********************************************************************************
#else
//********************************************************************************

//!!! or use MODIFIED LIBRARIES in "Arduino/libraries/" folder (need copy from /LIBRARIES/)

//LCD_ST7789:
#include <SPI.h>
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789

//micro_SD_Card:
#include <SdFat.h> // Use SDFAT-beta...

//player LIBs
///#include "Audio.h" //ESP32-audioI2S

#include "arduinoFFT.h" // Standard Arduino FFT library
//********************************************************************************
#endif

//********************************************************************************
//SETUP:

//USE SDFAT BETA!
SoftSpiDriver<SOFTSD_MISO_PIN, SOFTSD_MOSI_PIN, SOFTSD_SCK_PIN> softSpi;
#define SD_CONFIG SdSpiConfig(-1, DEDICATED_SPI, SD_SCK_MHZ(0), &softSpi)
SdFat SD;
File fp;

//LCD_ST7789
#define ST7789_DRIVER     // Configure all registers
#define TFT_WIDTH  240
#define TFT_HEIGHT 240
#define LOAD_GLCD   // Font 1. Original Adafruit 8 pixel font needs ~1820 bytes in FLASH
#define LOAD_FONT2  // Font 2. Small 16 pixel high font, needs ~3534 bytes in FLASH, 96 characters
#define LOAD_FONT4  // Font 4. Medium 26 pixel high font, needs ~5848 bytes in FLASH, 96 characters
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

//--------------------------------------------------------------------------------
//SPI FLASH MEMORY ACCESS:
#include "esp_spi_flash.h"

#define SPI_FLASH_ADDRESS 0x00300000 //0x00300000 working (empty flash area)
#define SPI_FLASH_SECTOR_SIZE 0x1000 //4kB = better not change

uint32_t FILE_ROM_SIZE = 0;
uint32_t FLASH_ROM_SIZE = 0;

//constant data pointer for direct access
const void *ROM;

spi_flash_mmap_handle_t handle1;

uint8_t flashdata[4096] = {0}; //4kB buffer

//--------------------------------------------------------------------------------
//DRAM MEMORY ACCESS:
//--------------------------------------------------------------------------------

char* ROMFILENAME; //ROM load File name
///unsigned char *rom = 0; //Actual ROM pointer

//********************************************************************************


//--------------------------------------------------------------------------------
//NES PALETTES 32bit & 16bit:

const uint32_t nes_32bit[64] = { //4colors Green-Yellow-Red-Blue
  0x2D253469, 0x000000D9, 0x000000A1, 0x0F111CA1, 0x001F3F70, 0x0024491C, 0x06264900, 0x061F3B00,
  0x12162300, 0x2D0B0000, 0x27090000, 0x21080000, 0x1806004D, 0x14141414, 0x14141414, 0x14141414,
  0x453950A1, 0x2D0B00D9, 0x210800D9, 0x181C2DD9, 0x002F5EAF, 0x0031624D, 0x153B6C00, 0x2139620E,
  0x2D304900, 0x45110000, 0x3F0F0000, 0x3F0F0038, 0x330C0077, 0x14141414, 0x14141414, 0x14141414,
  0x5D4D6CD9, 0x451D18D9, 0x33232DD9, 0x2D2C42D9, 0x2D416CD9, 0x213E6C85, 0x2D416C4D, 0x3C456C38,
  0x45476C00, 0x5D3F5015, 0x51272646, 0x5D2A2685, 0x571500BD, 0x2D253469, 0x14141414, 0x14141414,
  0x5D4D6CD9, 0x543846D9, 0x453950D9, 0x45405ED9, 0x45476CD9, 0x3C456CA8, 0x4E48699A, 0x544B6C93,
  0x514A6C69, 0x5D465E69, 0x5D3F50A1, 0x5D3F50BD, 0x5D1700D9, 0x514A6CD9, 0x14141414, 0x14141414,
};

const uint16_t nes_16bit[64] = {
  0x7BEF, 0x001F, 0x0017, 0x4157, 0x9010, 0xA804, 0xA880, 0x88A0,
  0x5180, 0x03C0, 0x0340, 0x02C0, 0x020B, 0x0000, 0x0000, 0x0000,
  0xBDF7, 0x03DF, 0x02DF, 0x6A3F, 0xD819, 0xE00B, 0xF9C0, 0xE2E2,
  0xABE0, 0x05C0, 0x0540, 0x0548, 0x0451, 0x0000, 0x0000, 0x0000,
  0xFFDF, 0x3DFF, 0x6C5F, 0x9BDF, 0xFBDF, 0xFAD3, 0xFBCB, 0xFD08,
  0xFDC0, 0xBFC3, 0x5ECA, 0x5FD3, 0x075B, 0x7BCF, 0x0000, 0x0000,
  0xFFFF, 0xA73F, 0xBDDF, 0xDDDF, 0xFDDF, 0xFD38, 0xF696, 0xFF15,
  0xFECF, 0xDFCF, 0xBFD7, 0xBFDB, 0x07FF, 0xFEDF, 0x0000, 0x0000,
};

const uint16_t gbc_16bit[64] = {
0x0,0x10,0x20,0x30,0x4000,0x4010,0x4020,0x4030,
0x8000,0x8010,0x8020,0x8030,0xC000,0xC010,0xC020,0xC030,
0x200,0x210,0x220,0x230,0x4200,0x4210,0x4220,0x4230,
0x8200,0x8210,0x8220,0x8230,0xC200,0xC210,0xC220,0xC230,
0x400,0x410,0x420,0x430,0x4400,0x4410,0x4420,0x4430,
0x8400,0x8410,0x8420,0x8430,0xC400,0xC410,0xC420,0xC430,
0x600,0x610,0x620,0x630,0x4600,0x4610,0x4620,0x4630,
0x8600,0x8610,0x8620,0x8630,0xC600,0xC610,0xC620,0xC630,
};

const uint32_t gbc_32bit[64] = {
0b00001111000011110000111100001111,
0x40,0x80,0xC0,0x4000,0x4040,0x4080,0x40C0,
0x408000,0x408040,0x408080,0x4080C0,0x40C000,0x40C040,0x40C080,0x40C0C0,
0x40000000,0x40000040,0x40000080,0x400000C0,0x40404000,0x40404040,0x40404080,0x404040C0,
0x40408000,0x40408040,0x40408080,0x404080C0,0x4080C000,0x4080C040,0x4080C080,0x4080C0C0,
0x80400000,0x80400040,0x80400080,0x804000C0,0x80404000,0x80404040,0x80404080,0x804040C0,
0x80808000,0x80808040,0x80808080,0x808080C0,0x8080C000,0x8080C040,0x8080C080,0x8080C0C0,
0xC0400000,0xC0400040,0xC0400080,0xC04000C0,0xC0804000,0xC0804040,0xC0804080,0xC08040C0,
0xC0808000,0xC0808040,0xC0808080,0xC08080C0,0x80C08000,0x80C08040,0x80C08080,0x80C080C0,
};

//--------------------------------------------------------------------------------
//for player...
bool PLAYING = false;
bool PAUSED = false;

//===============================================================================
//===============================================================================
//===============================================================================
//INPUT SYSTEM:
uint8_t JOY_UP = 0;
uint8_t JOY_DOWN = 0;
uint8_t JOY_LEFT = 0;
uint8_t JOY_RIGHT = 0;
uint8_t JOY_CROSS = 0;
uint8_t JOY_SQUARE = 0;
uint8_t JOY_CIRCLE = 0;
uint8_t JOY_TRIANGLE = 0;
uint8_t JOY_SHARE = 0; //(START)
uint8_t JOY_OPTIONS = 0; //(SELECT)

//================================================================================
bool buttonchange=false;
uint32_t badge_input_button_state = 0; 
//================================================================================

//--------------------------------------------------------------------------------
uint8_t scancode = 0;
boolean keyup = false;
byte keymap[256];
//--------------------------------------------------------------------------------
IRAM_ATTR void USB_KEYBOARD() {
#if KEYBOARD_ENABLED
  if (keymap[0x75] == 0) {
    JOY_UP = 1;
  } else JOY_UP = 0;
  if (keymap[0x72] == 0) {
    JOY_DOWN = 1;
  } else JOY_DOWN = 0;
  if (keymap[0x6B] == 0) {
    JOY_LEFT = 1;
  } else JOY_LEFT = 0;
  if (keymap[0x74] == 0) {
    JOY_RIGHT = 1;
  } else JOY_RIGHT = 0;

  if (keymap[0x11] == 0 || keymap[0x1A] == 0) JOY_CROSS = 1; //ALT or X
  else JOY_CROSS = 0;
  if (keymap[0x14] == 0 || keymap[0x22] == 0) JOY_SQUARE = 1; //CTRL or Z
  else JOY_SQUARE = 0;
  if (keymap[0x5A] == 0) JOY_SHARE = 1;
  else JOY_SHARE = 0;
  if (keymap[0x66] == 0) JOY_OPTIONS = 1;
  else JOY_OPTIONS = 0;

  if (keymap[0x76] == 0) {
    JOY_SHARE = 1;
    JOY_OPTIONS = 1;
  } else if (JOY_SHARE == 1 && JOY_OPTIONS == 1 ) {
    JOY_SHARE = 0;
    JOY_OPTIONS = 0;
  }

  if (DEBUGEXTRA) {
    if (JOY_UP) Serial.print("UP.");
    if (JOY_DOWN) Serial.print("DOWN.");
    if (JOY_LEFT) Serial.print("LEFT.");
    if (JOY_RIGHT) Serial.print("RIGHT.");
    if (JOY_SHARE) Serial.print("START.");
    if (JOY_OPTIONS) Serial.print("SELECT.");
    if (JOY_CROSS) Serial.print("A.");
    if (JOY_SQUARE) Serial.print("B.");
    Serial.println();
  }



if (JOY_CROSS==1) badge_input_button_state |= 1 << 6;
else badge_input_button_state &= 0b11111111111111111111111110111111;  

if (JOY_SQUARE==1) badge_input_button_state |= 1 << 7;
else badge_input_button_state &= 0b11111111111111111111111101111111;
 
if (JOY_SHARE==1) badge_input_button_state |= 1 << 9;
else badge_input_button_state &= 0b11111111111111111111110111111111;

if (JOY_OPTIONS==1)badge_input_button_state |= 1 << 8;
else badge_input_button_state &= 0b11111111111111111111111011111111;  

if (JOY_UP==1) badge_input_button_state |= 1 << 1;
else badge_input_button_state &= 0b11111111111111111111111111111101;  

if (JOY_DOWN==1) badge_input_button_state |= 1 << 2;
else badge_input_button_state &= 0b11111111111111111111111111111011;  

if (JOY_LEFT==1) badge_input_button_state |= 1 << 3;
else badge_input_button_state &= 0b11111111111111111111111111110111;  

if (JOY_RIGHT==1) badge_input_button_state |= 1 << 4;
else badge_input_button_state &= 0b11111111111111111111111111101111;

  buttonchange=true;



  
#endif
}
//--------------------------------------------------------------------------------
void IRAM_ATTR kb_interruptHandler(void) {
#if KEYBOARD_ENABLED
  static uint8_t bitcount = 0;
  uint8_t val;

  int clock = digitalRead(KEYBOARD_CLK);
  if (clock == 0) {
    return;
  }

  val = digitalRead(KEYBOARD_DATA);
  if (DEBUGEXTRA) Serial.print(".");
  if (DEBUGEXTRA) Serial.print(val);

  bitcount++;

  if (bitcount > 1 && bitcount < 10) { //8bits
    scancode |= ((val & 1) << (bitcount - 2));
  }

  if (bitcount > 10) {
    if (keyup == true) {
      keymap[scancode] = 1;

      if (DEBUGEXTRA) {
        Serial.print(" {");
        Serial.print(scancode, HEX);
        Serial.print("} ");
      }
      keyup = false;
    } else {
      keymap[scancode] = 0;
    }

    if (scancode  == 0xF0  ) {
      keyup = true;
    } else {
      keyup = false;
    }

    if (DEBUGEXTRA) {
      Serial.print("[");
      Serial.print(scancode, HEX);
      Serial.println("]");
    }

    USB_KEYBOARD();

    bitcount = 0;
    scancode = 0;
  }
#endif
}
//--------------------------------------------------------------------------------
void kb_begin() {
#if KEYBOARD_ENABLED
  pinMode(KEYBOARD_DATA, INPUT_PULLUP);
  pinMode(KEYBOARD_CLK, INPUT_PULLUP);
  digitalWrite(KEYBOARD_DATA, true);
  digitalWrite(KEYBOARD_CLK, true);
  attachInterrupt(digitalPinToInterrupt(KEYBOARD_CLK), kb_interruptHandler, RISING);
  memset(keymap, 1, sizeof(keymap));
#endif
}
//********************************************************************************
//================================================================================
//================================================================================
void IRAM_ATTR JOY_A_Handler(void) {
   if (digitalRead(PIN_A)==1) JOY_CROSS=1;
   else JOY_CROSS=0;
   buttonchange=true;
   if (JOY_CROSS==1) badge_input_button_state |= 1 << 6;
   else badge_input_button_state &= 0b11111111111111111111111110111111;  
   return;  
}  
//--------------------------------------------------------------------------------
void IRAM_ATTR JOY_B_Handler(void) {
   if (digitalRead(PIN_B)==1) JOY_SQUARE=1;
   else JOY_SQUARE=0;
   buttonchange=true;
   if (JOY_SQUARE==1) badge_input_button_state |= 1 << 7;
   else badge_input_button_state &= 0b11111111111111111111111101111111;
   return;  
}  
//--------------------------------------------------------------------------------
void IRAM_ATTR JOY_START_Handler(void) {
   if (digitalRead(PIN_START)==1) JOY_SHARE=1;
   else JOY_SHARE=0;
   buttonchange=true;
   if (JOY_SHARE==1) badge_input_button_state |= 1 << 9;
   else badge_input_button_state &= 0b11111111111111111111110111111111;
   return;  
}  
//--------------------------------------------------------------------------------
void IRAM_ATTR JOY_SELECT_Handler(void) {
   if (digitalRead(PIN_SELECT)==1) JOY_OPTIONS=1;
   else JOY_OPTIONS=0;
   buttonchange=true;
   if (JOY_OPTIONS==1) badge_input_button_state |= 1 << 8;
   else badge_input_button_state &= 0b11111111111111111111111011111111;  
   return;  
}  
//--------------------------------------------------------------------------------
void IRAM_ATTR JOY_UP_Handler(void) {
   if (digitalRead(PIN_UP)==1) JOY_UP=1;
   else JOY_UP=0;
   buttonchange=true;
   if (JOY_UP==1) badge_input_button_state |= 1 << 1;
   else badge_input_button_state &= 0b11111111111111111111111111111101;  
   return;  
}  
//--------------------------------------------------------------------------------
void IRAM_ATTR JOY_DOWN_Handler(void) {
   if (digitalRead(PIN_DOWN)==1) JOY_DOWN=1;
   else JOY_DOWN=0;
   buttonchange=true;
   if (JOY_DOWN==1) badge_input_button_state |= 1 << 2;
   else badge_input_button_state &= 0b11111111111111111111111111111011;  
   return;  
}  
//--------------------------------------------------------------------------------
void IRAM_ATTR JOY_LEFT_Handler(void) {
   if (digitalRead(PIN_LEFT)==1) JOY_LEFT=1;
   else JOY_LEFT=0;
   buttonchange=true;
   if (JOY_LEFT==1) badge_input_button_state |= 1 << 3;
   else badge_input_button_state &= 0b11111111111111111111111111110111;  
   return;  
}  
//--------------------------------------------------------------------------------
void IRAM_ATTR JOY_RIGHT_Handler(void) {
   if (digitalRead(PIN_RIGHT)==1) JOY_RIGHT=1;
   else JOY_RIGHT=0;
   buttonchange=true;
   if (JOY_RIGHT==1) badge_input_button_state |= 1 << 4;
   else badge_input_button_state &= 0b11111111111111111111111111101111;  
   return;  
}  
//================================================================================




#include <esp_task_wdt.h>

/*
extern "C" {
extern const unsigned char game_rom[];
extern const unsigned int game_rom_len;
extern void burnin();
};
*/


//===============================================================================
//INCLUDES:

//MAIN FONT
#include "Retro8x16.c" //used font...

//MENU ICON
///#include "icons.c" //used font...

//===============================================================================
//===============================================================================
//===============================================================================
//VIDEO SYSTEM:
QueueHandle_t vidQueue;

//VIDEO_SETUP
#define DEFAULT_WIDTH 240
#define DEFAULT_HEIGHT 240
//--------------------------------------------------------------------------------
//BUFFER TEXT DRAW FUNCTIONS
//--------------------------------------------------------------------------------
extern SemaphoreHandle_t fb_mutex;
//--------------------------------------------------------------------------------


void screenmemory_fillscreen(uint8_t COLOR) {
  for (uint16_t Ypos = 0; Ypos < DEFAULT_HEIGHT; Ypos++)
    for (uint16_t Xpos = 0; Xpos < DEFAULT_WIDTH; Xpos++) {
      SCREENMEMORY[Ypos][Xpos] = COLOR;
    }
///  if (LCD_ENABLED) xQueueSend(vidQueue, &SCREENMEMORY, 0); //refresh LCD
xSemaphoreGive(fb_mutex);
}
//--------------------------------------------------------------------------------
void screenmemory_drawpixel(uint16_t X, uint16_t Y, uint8_t COLOR) {
  if (Y < DEFAULT_HEIGHT && X < DEFAULT_WIDTH) SCREENMEMORY[Y][X] = COLOR;
}
//--------------------------------------------------------------------------------
void screenmemory_line(int startx, int starty, int endx, int endy, uint8_t color) {
  int t, distance;
  int xerr = 0, yerr = 0, delta_x, delta_y;
  int incx, incy;
  // compute the distances in both directions
  delta_x = endx - startx;
  delta_y = endy - starty;
  // Compute the direction of the increment,
  //   an increment of 0 means either a horizontal or vertical
  //   line.
  if (delta_x > 0) incx = 1;
  else if (delta_x == 0) incx = 0;
  else incx = -1;

  if (delta_y > 0) incy = 1;
  else if (delta_y == 0) incy = 0;
  else incy = -1;

  // determine which distance is greater
  delta_x = abs(delta_x);
  delta_y = abs(delta_y);
  if (delta_x > delta_y) distance = delta_x;
  else distance = delta_y;

  // draw the line
  for (t = 0; t <= distance + 1; t++) {
    screenmemory_drawpixel(startx, starty, color);

    xerr += delta_x;
    yerr += delta_y;
    if (xerr > distance) {
      xerr -= distance;
      startx += incx;
    }
    if (yerr > distance) {
      yerr -= distance;
      starty += incy;
    }
  }
}
//--------------------------------------------------------------------------------
void screenmemory_drawrectangle(int16_t X, int16_t Y, int16_t Width, int16_t Height, uint8_t COLOR) {
  screenmemory_line(X, Y, X + Width, Y, COLOR);
  screenmemory_line(X, Y, X, Y + Height, COLOR);
  screenmemory_line(X + Width, Y, X + Width, Y + Height, COLOR);
  screenmemory_line(X, Y + Height, X + Width, Y + Height, COLOR);
}
//--------------------------------------------------------------------------------
void screenmemory_drawfillrectangle(int16_t X, int16_t Y, int16_t Width, int16_t Height, uint8_t COLOR) {
  for (uint16_t Ypos = Y; Ypos < Y + Height; Ypos++)
    for (uint16_t Xpos = X; Xpos < X + Width; Xpos++) {
      screenmemory_drawpixel(Xpos, Ypos, COLOR);
    }
///  if (LCD_ENABLED) xQueueSend(vidQueue, &SCREENMEMORY, 0); //refresh LCD
xSemaphoreGive(fb_mutex);
}
//--------------------------------------------------------------------------------
uint8_t draw_char_xy(uint16_t Main_x, uint16_t Main_y, char Main_char, const char* font, uint8_t color = 48) {
  uint8_t XcharSize = font[0]; //x char size
  uint8_t YcharSize = font[1]; //y char size
  uint8_t CHAROFFSET = font[2]; //char start offset

  font[3]; //char count
  if (Main_char != '\n' && Main_char != '\r') for (uint16_t Ypos = 0; Ypos < YcharSize; Ypos++)
      for (uint16_t Xpos = 0; Xpos < XcharSize; Xpos += 8) {
        uint8_t CHARLINE = font[(Main_char - CHAROFFSET) * (YcharSize * (XcharSize / 8)) +   (Ypos) * (XcharSize / 8) + Xpos / 8  + 4];

        if ((Xpos + 0 < XcharSize) && (CHARLINE & 0b10000000) != 0) screenmemory_drawpixel(Main_x + Xpos + 0, Main_y + Ypos, color);
        else screenmemory_drawpixel(Main_x + Xpos + 0, Main_y + Ypos, 63);
        if ((Xpos + 1 < XcharSize) && (CHARLINE & 0b01000000) != 0) screenmemory_drawpixel(Main_x + Xpos + 1, Main_y + Ypos, color);
        else screenmemory_drawpixel(Main_x + Xpos + 1, Main_y + Ypos, 63);
        if ((Xpos + 2 < XcharSize) && (CHARLINE & 0b00100000) != 0) screenmemory_drawpixel(Main_x + Xpos + 2, Main_y + Ypos, color);
        else screenmemory_drawpixel(Main_x + Xpos + 2, Main_y + Ypos, 63);
        if ((Xpos + 3 < XcharSize) && (CHARLINE & 0b00010000) != 0) screenmemory_drawpixel(Main_x + Xpos + 3, Main_y + Ypos, color);
        else screenmemory_drawpixel(Main_x + Xpos + 3, Main_y + Ypos, 63);
        if ((Xpos + 4 < XcharSize) && (CHARLINE & 0b00001000) != 0) screenmemory_drawpixel(Main_x + Xpos + 4, Main_y + Ypos, color);
        else screenmemory_drawpixel(Main_x + Xpos + 4, Main_y + Ypos, 63);
        if ((Xpos + 5 < XcharSize) && (CHARLINE & 0b00000100) != 0) screenmemory_drawpixel(Main_x + Xpos + 5, Main_y + Ypos, color);
        else screenmemory_drawpixel(Main_x + Xpos + 5, Main_y + Ypos, 63);
        if ((Xpos + 6 < XcharSize) && (CHARLINE & 0b00000010) != 0) screenmemory_drawpixel(Main_x + Xpos + 6, Main_y + Ypos, color);
        else screenmemory_drawpixel(Main_x + Xpos + 6, Main_y + Ypos, 63);
        if ((Xpos + 7 < XcharSize) && (CHARLINE & 0b00000001) != 0) screenmemory_drawpixel(Main_x + Xpos + 7, Main_y + Ypos, color);
        else screenmemory_drawpixel(Main_x + Xpos + 7, Main_y + Ypos, 63);
      }
  return XcharSize;
}
//--------------------------------------------------------------------------------
uint16_t draw_string_xy(uint16_t x, uint16_t y, char *c, const char* font, uint8_t color = 48)
{
  uint8_t width;
  uint8_t XcharSize = font[0]; //x char size
  uint8_t YcharSize = font[1]; //y char size
  uint16_t textwidth = 0;
  while (*c) {
    width = draw_char_xy(x, y, *c, font, color);
    textwidth += (width);
    x += (width);
    c++;
  }
///  if (LCD_ENABLED) xQueueSend(vidQueue, &SCREENMEMORY, 0);
xSemaphoreGive(fb_mutex);
  return textwidth;
}
//--------------------------------------------------------------------------------
const char* DisplayFontSet = NULL;
uint16_t XPOS_CHAR = 0;
uint16_t YPOS_CHAR = 0;
//--------------------------------------------------------------------------------
void set_font(const char* font) {
  DisplayFontSet = font;
}
//--------------------------------------------------------------------------------
void set_font_XY(uint16_t x, uint16_t y) {
  XPOS_CHAR = x;
  YPOS_CHAR = y;
}
//--------------------------------------------------------------------------------
void draw_string(char *c, uint8_t color = 48) {
  if (c[strlen(c) - 1] == '\n') {
    draw_string_xy(XPOS_CHAR, YPOS_CHAR, c, DisplayFontSet, color);
    YPOS_CHAR += (uint8_t)(DisplayFontSet[1]);
    XPOS_CHAR = 0;
  } else {
    XPOS_CHAR += draw_string_xy(XPOS_CHAR, YPOS_CHAR, c, DisplayFontSet, color);
  }
}

//--------------------------------------------------------------------------------
extern "C" void pwm_audio_write_(uint8_t *inbuf, size_t len, size_t *bytes_written, TickType_t ticks_to_wait) {
   pwm_audio_write(inbuf, len, bytes_written, ticks_to_wait);
}
//--------------------------------------------------------------------------------
//--------------------------------------------------------------------------------
uint8_t DEFAULTPAL=0;

//--------------------------------------------------------------------------------
extern "C" int lcd_write_frame(const uint16_t x, const uint16_t y, const uint16_t width, const uint16_t height) {
  uint8_t n, l;
  uint16_t i, len;
  uint16_t w = TFT_WIDTH;
  uint16_t h = TFT_HEIGHT;
  uint16_t X_ = 0;

  for (i = 0; i < height; i++) {
///    uint16_t horizontal_offset = 8;
    for (X_ = 0; X_ < width; X_++) { //not 256 the 240
      if (DEFAULTPAL==0)
        SCREENBUFFER[X_] = nes_16bit[(0x3F & ((uint8_t *)SCREENMEMORY[i])[X_])];
      else 
        SCREENBUFFER[X_] = gbc_16bit[(0x3F & ((uint8_t *)SCREENMEMORY[i])[X_])];
    }



    ///PAL optimalisation in this case:
    tft.drawRGBBitmap(0, i, (uint16_t *)(SCREENBUFFER), 48, 1);
    tft.drawRGBBitmap(48, i, (uint16_t *)(SCREENBUFFER + 48), 48, 1);
    tft.drawRGBBitmap(96, i, (uint16_t *)(SCREENBUFFER + 96), 48, 1);
    tft.drawRGBBitmap(144, i, (uint16_t *)(SCREENBUFFER + 144), 48, 1);
    tft.drawRGBBitmap(192, i, (uint16_t *)(SCREENBUFFER + 192), 48, 1);
  }
}
//--------------------------------------------------------------------------------






//--------------------------------------------------------------------------------
char loadmessage[64];
//--------------------------------------------------------------------------------
uint8_t* load_file(char * filename, uint32_t* len) {
   uint8_t* data;
  
   fp = SD.open(filename);
   if (DEBUG) Serial.print("FILE SIZE: ");
   if (DEBUG) Serial.println(fp.size());
   *len = fp.size();

   uint16_t BLOCKCOUNT = (*len) / (1024*32);
   uint16_t BLOCKSIZEPX = 240 / BLOCKCOUNT;
 
   if (PSRAMSIZE > 0) {

      data = (uint8_t *)ps_malloc(fp.size());
      Serial.println("ps_malloc done.");
      uint32_t i = 0;
      for (i = 0; i < fp.size(); i++) {
         ((uint8_t*)data)[i] = fp.read();

         if (i%1024==0) {
            sprintf(loadmessage, " %d / %d kB", i/1024, (*len/1024));
            set_font_XY(8, 8);
            draw_string("Loaded:");
            draw_string(loadmessage);
            screenmemory_drawfillrectangle(((i / (1024*32)) - 0) * BLOCKSIZEPX, 24, BLOCKSIZEPX, 16, 57);
         }                  
      }           
   }
   fp.close();
   return data; 
}
//--------------------------------------------------------------------------------

//********************************************************************************

bool EXIT = false;

#define MAXFILES 512
#define MAXFILENAME_LENGTH 64
char* filename[MAXFILES];
char fileext[4];
#define FILESPERPAGE 8

SdFile dirFile;
SdFile file;

char* MAINPATH;
char textbuf[64] = {0};

//--------------------------------------------------------------------------------
void sortStrings(char* arr[], int n)
{
  char temp[MAXFILENAME_LENGTH];

  // Sorting strings using bubble sort
  for (int j = 0; j < n - 1; j++)
  {
    for (int i = j + 1; i < n; i++)
    {
      if (strcmp(arr[j], arr[i]) > 0)
      {
        strcpy(temp, arr[j]);
        strcpy(arr[j], arr[i]);
        strcpy(arr[i], temp);
      }
    }
  }
}
//--------------------------------------------------------------------------------
void secondsToHMS( const uint32_t seconds, uint16_t &h, uint8_t &m, uint8_t &s ) {
  uint32_t t = seconds;
  s = t % 60;
  t = (t - s) / 60;
  m = t % 60;
  t = (t - m) / 60;
  h = t;
}
//--------------------------------------------------------------------------------

uint8_t GBC_POWER = 1;


//--------------------------------------------------------------------------------
char* GBCEXPLORE(char* PATH) {
  uint8_t num = 0;
  uint8_t loadedFileNames = 0;


  //clear memory variables
  for (uint16_t tmp = 0; tmp < MAXFILES; tmp++) memset (filename[tmp], 0, sizeof(filename[tmp]));
  fileext[0] = 0;
  fileext[1] = 0;
  fileext[2] = 0;
  fileext[3] = 0;

  num = 0;
  loadedFileNames = 0;

  //LOAD FILENAMES INTO MEMORY...

  //Load List files in root directory.
  ///if (!dirFile.open("/", O_READ)) {
  if (!dirFile.open(PATH, O_READ)) {
    while (1) {};
  }
  while (num < MAXFILES && file.openNext(&dirFile, O_READ)) {

    // Skip hidden files.
    if (!file.isHidden()) {
      for (uint8_t i = strlen(filename[num]); i > 3; i--) filename[num][i] = 0;
      file.getName(filename[num], MAXFILENAME_LENGTH);
      if (file.isSubDir()) {
        sprintf(filename[num], "%s/", filename[num]);
        num++;
      } else {
        for (uint8_t i = strlen(filename[num]); i > 3; i--) {
          if (filename[num][i] != 0) {
            fileext[3] = '\0';
            fileext[2] = filename[num][i];
            fileext[1] = filename[num][i - 1];
            fileext[0] = filename[num][i - 2];
            break;
          }
        }
      }

      if (DEBUG) {
        ///Serial.println(fileext[num]);
        ///Serial.println(strlen(filename[num]));
      }

      //check GBC File extension, then increase index
      if ((fileext[0] == 'G' || fileext[0] == 'g')
          && (fileext[1] == 'B' || fileext[1] == 'b')
          && (fileext[2] == 'C' || fileext[2] == 'c')) {
        num++;
      }

    }
    loadedFileNames = num;
    file.close();
  }

  dirFile.close();

  if (DEBUG) {
    Serial.println("--------------------------------------");
    Serial.print("Count of loaded File Names:");
    Serial.println(loadedFileNames);
  }

  sortStrings(filename, loadedFileNames);

  //DRAW FILENAMES INTO BUFFER
  uint8_t CURSOR = 0;
  uint8_t PAGE = 0;
  bool NamesDisplayed = false;

  while (1) {



#if BLUETOOTH_ENABLED
    ///      hid_update();
    ///      PS4_JOY();
#endif
    PAGE = CURSOR / FILESPERPAGE;
    if (!NamesDisplayed) {
      screenmemory_fillscreen(63); //black color
      set_font_XY(16, 24 );
      draw_string(PATH);


      for (num = PAGE * FILESPERPAGE; num < ((PAGE + 1)*FILESPERPAGE) && num < loadedFileNames; num++) {
        set_font_XY(40, 48 + 20 * (num % FILESPERPAGE));
        ///draw_string(filename[num],48);

        if (filename[num][strlen(filename[num]) - 1] == '/') draw_string(filename[num], 23);
        else draw_string(filename[num], 48);

        delay(1);
      }
      NamesDisplayed = true;
    }

    //Draw Cursor
    set_font_XY(16, 48 + (20 * (CURSOR % FILESPERPAGE)));
    draw_string("->", 48);
    delay(200);

    //PROCESS CURSOR SELECTION
    while (JOY_CROSS == 0 && JOY_SQUARE == 0 && JOY_OPTIONS == 0 && JOY_SHARE == 0 && JOY_UP == 0 && JOY_DOWN == 0 && JOY_LEFT == 0 && JOY_RIGHT == 0) {
      if (digitalRead(PIN_A) == 1) {
        JOY_CROSS = 1;  //A
        delay(25);
      }
      if (digitalRead(PIN_B) == 1) {
        JOY_SQUARE = 1;   //B
        delay(25);
      }
      if (digitalRead(PIN_SELECT) == 1) {
        JOY_OPTIONS = 1;   //SELECT
        delay(25);
      }
      if (digitalRead(PIN_START) == 1) {
        JOY_SHARE = 1;   //START
        delay(25);
      }
      if (digitalRead(PIN_UP) == 1) {
        JOY_UP = 1;
        delay(25);
      }
      if (digitalRead(PIN_DOWN) == 1) {
        JOY_DOWN = 1;   //DOWN
        delay(25);
      }
      if (digitalRead(PIN_LEFT) == 1) {
        JOY_LEFT = 1;   //LEFT
        delay(25);
      }
      if (digitalRead(PIN_RIGHT) == 1) {
        JOY_RIGHT = 1;   //RIGHT
        delay(25);
      }
    }

    //Empty Cursor
    set_font_XY(16, 48 + (20 * (CURSOR % FILESPERPAGE)));
    draw_string("  ", 48);

///NCX: disabled for now:
/*
    if (JOY_SHARE == 1 && JOY_OPTIONS == 1) {
      JOY_SHARE = 0;
      JOY_OPTIONS = 0;
      EXIT = true;
      ///         PLAYING=false;
      GBC_POWER = 0;

      return MAINPATH;
    }
*/
    if (JOY_UP == 1 ) {
      if (CURSOR % FILESPERPAGE == 0) NamesDisplayed = false; //changed page
      if (CURSOR == 0 && loadedFileNames > 0) CURSOR = loadedFileNames - 1;
      else if (CURSOR > 0 && loadedFileNames > 0) CURSOR--;
      JOY_UP = 0;
    }
    if (JOY_DOWN == 1 ) {
      if (CURSOR % FILESPERPAGE == FILESPERPAGE - 1 || CURSOR == loadedFileNames - 1) NamesDisplayed = false; //changed page
      if (CURSOR == loadedFileNames - 1 && loadedFileNames > 0) CURSOR = 0;
      else if (CURSOR < loadedFileNames - 1 && loadedFileNames > 0) CURSOR++;
      JOY_DOWN = 0;
    }
    if (JOY_LEFT == 1) {
      if (CURSOR > FILESPERPAGE - 1) CURSOR -= FILESPERPAGE;
      NamesDisplayed = false;
      JOY_LEFT = 0;
    }
    if (JOY_RIGHT == 1) {
      if (CURSOR / FILESPERPAGE < loadedFileNames / FILESPERPAGE) CURSOR += FILESPERPAGE;
      if (CURSOR > loadedFileNames - 1) CURSOR = loadedFileNames - 1;
      NamesDisplayed = false;
      JOY_RIGHT = 0;
    }
    if (JOY_OPTIONS == 1) {
      //do nothing  = unused
      JOY_OPTIONS = 0;
    }
    if ((JOY_CROSS == 1 || JOY_SHARE == 1) && JOY_OPTIONS == 0) {
      dirFile.close();
      JOY_CROSS = 0;
      JOY_SHARE = 0;
      JOY_OPTIONS = 0;
      delay(25);

      ///         PLAYINGFILE=CURSOR;
      ///         TOTALFILES=loadedFileNames;

      sprintf(MAINPATH, "%s%s", PATH, filename[CURSOR]);
      if (DEBUG) Serial.println(MAINPATH);

      ///         sprintf(TRACKNAME, "%s", filename[CURSOR]);

      return MAINPATH ; //START //A
    }
    if ((JOY_SQUARE == 1 ) && JOY_OPTIONS == 0) {

      dirFile.close();
      JOY_SQUARE = 0;
      JOY_SHARE = 0;
      JOY_OPTIONS = 0;
      delay(25);

      if (DEBUG) Serial.println(PATH);
      if (DEBUG) Serial.println(strlen(PATH));

      sprintf(MAINPATH, "%s", PATH);

      if (strlen(MAINPATH) > 1) {
        MAINPATH[strlen(MAINPATH) - 1] = '\0';
        for (uint8_t strpos = strlen(MAINPATH) - 1; strpos > 0; strpos--) {
          if (MAINPATH[strpos] == '/') break;
          MAINPATH[strpos] = '\0';
        }
      }

      if (DEBUG) Serial.println(MAINPATH);
      if (DEBUG) Serial.println(strlen(MAINPATH));
      return MAINPATH ;
    }
  };
}
//################################################################################
//********************************************************************************
char* GBCBrowse(char* PATH) {
  if (PATH[strlen(PATH) - 1] != '/')
    if (strlen(PATH) > 1) {
      PATH[strlen(PATH) - 1] = '\0';
      for (uint8_t strpos = strlen(PATH) - 1; strpos > 0; strpos--) {
        if (PATH[strpos] == '/') break;
        PATH[strpos] = '\0';
      }
    }

  for (uint16_t tmp = 0; tmp < MAXFILES; tmp++) {
    ///    filename[tmp] = (char*)malloc(MAXFILENAME_LENGTH);
    ///    fileext[tmp] = (char*)malloc(4);
  }

  EXIT = false;
  //................................................................................
  while (EXIT == false && PATH[strlen(PATH) - 1] == '/')  {
    PATH =  GBCEXPLORE(PATH);
    Serial.println(PATH);
  }
  //................................................................................

  for (uint16_t tmp = 0; tmp < MAXFILES; tmp++) {
    ///      free(filename[tmp]);
    ///      free(fileext[tmp]);
  }
  return PATH;
}
//________________________________________________________________________________






















//********************************************************************************



unsigned char* game_rom;
unsigned int game_rom_len;

int GBC_EXIT=0;

void setup()
{
  Serial.begin(115200);
/// Serial.setDebugOutput(true);
   if (ESP.getPsramSize() > 0) {
      PSRAMSIZE = ESP.getPsramSize();
///      PSRAM = (uint8_t*)ps_malloc(2097152); //PSRAM malloc 2MB
   }
//--------------------------------------------------------------------------------
   //malloc MAINPATH for GBC browser
   MAINPATH = (char*)malloc(256);
//--------------------------------------------------------------------------------   
//--------------------------------------------------------------------------------
   // VIDEO MEMORY ALLOCATION (force)
   for (uint32_t tmp = 0; tmp < 240; tmp++) {
      SCREENMEMORY[tmp] = (uint8_t*)malloc(256 + 1 );
      memset(SCREENMEMORY[tmp], 0, 256);
   }
   
//--------------------------------------------------------------------------------
   // if the display has CS pin try with SPI_MODE0
   tft.init(240, 240, SPI_MODE2);    // Init ST7789 display 240x240 pixel
   // if the screen is flipped, remove this command
   tft.setRotation(3);
   tft.setSPISpeed(80000000);
   tft.fillScreen(ST77XX_BLACK); 

   tft.println("NCAT SYSTEM: MEOW!..."); //??? :D
//--------------------------------------------------------------------------------
   //SCREENMEMORY LCD DRAW INIT
   ///audiovideo_init();
//--------------------------------------------------------------------------------
   ///microSD CARD INIT
   if (!SD.begin(SD_CONFIG)) {
      Serial.println("SD error!");
   } else Serial.println("SD OK.");
//--------------------------------------------------------------------------------
   DEFAULTPAL=0;
   vid_init();
//--------------------------------------------------------------------------------
   //INTRO TEXT
   screenmemory_fillscreen(63); //black color
   set_font(Retro8x16); //Very important
   set_font_XY(32, 240 / 2 - 8);
   draw_string("NCat SYSTEM by Nathalis", 48);
   delay(500);
   screenmemory_fillscreen(63); //black color
//--------------------------------------------------------------------------------
//--------------------------------------------------------------------------------
   //Buttons Pins Input Init
   pinMode(PIN_A, INPUT);    //A
   pinMode(PIN_B, INPUT);   //B
   pinMode(PIN_SELECT, INPUT);   //SELECT
   pinMode(PIN_START, INPUT);   //START
   pinMode(PIN_UP, INPUT);    //UP
   pinMode(PIN_DOWN, INPUT);   //DOWN  //TCK
   pinMode(PIN_LEFT, INPUT);       //LEFT
   pinMode(PIN_RIGHT, INPUT);  //RIGHT
//--------------------------------------------------------------------------------
   digitalWrite(PIN_A, 0);
   digitalWrite(PIN_B, 0);
   digitalWrite(PIN_SELECT, 0);
   digitalWrite(PIN_START, 0);
   digitalWrite(PIN_UP, 0);
   digitalWrite(PIN_DOWN, 0);
   digitalWrite(PIN_LEFT, 0);
   digitalWrite(PIN_RIGHT, 0);
//--------------------------------------------------------------------------------
   attachInterrupt(digitalPinToInterrupt(PIN_A), JOY_A_Handler, CHANGE);
   attachInterrupt(digitalPinToInterrupt(PIN_B), JOY_B_Handler, CHANGE);
   attachInterrupt(digitalPinToInterrupt(PIN_START), JOY_START_Handler, CHANGE);
   attachInterrupt(digitalPinToInterrupt(PIN_SELECT), JOY_SELECT_Handler, CHANGE);
   attachInterrupt(digitalPinToInterrupt(PIN_UP), JOY_UP_Handler, CHANGE);
   attachInterrupt(digitalPinToInterrupt(PIN_DOWN), JOY_DOWN_Handler, CHANGE);
   attachInterrupt(digitalPinToInterrupt(PIN_LEFT), JOY_LEFT_Handler, CHANGE);
   attachInterrupt(digitalPinToInterrupt(PIN_RIGHT), JOY_RIGHT_Handler, CHANGE);
//--------------------------------------------------------------------------------

   //PS2/USB KEYBOARD SUPPORT
#if KEYBOARD_ENABLED
   kb_begin();
#endif
//--------------------------------------------------------------------------------
//--------------------------------------------------------------------------------

   

   ///pwm_audio_deinit();

   ///pwm_audio_config_t pac;
   pac.duty_resolution    = LEDC_TIMER_10_BIT;
   pac.gpio_num_left      = CONFIG_LEFT_CHANNEL_GPIO;
   pac.ledc_channel_left  = LEDC_CHANNEL_0;
   pac.gpio_num_right     = CONFIG_RIGHT_CHANNEL_GPIO;
   pac.ledc_channel_right = LEDC_CHANNEL_1;
   pac.ledc_timer_sel     = LEDC_TIMER_0;
   pac.tg_num             = TIMER_GROUP_0;
   pac.timer_num          = TIMER_0;
   pac.ringbuf_len        = 1024 * 8; /// default: 1024*8
   pwm_audio_init(&pac);

   ///  pwm_audio_set_param(wave_framerate, (ledc_timer_bit_t)wave_bits, wave_ch);
   //FOR MP3:
   ///  pwm_audio_set_param(46000, (ledc_timer_bit_t)16, 2);
   //FOR NES:
   pwm_audio_set_param(8000, (ledc_timer_bit_t)16, 1);
   pwm_audio_start();
   pwm_audio_set_volume(0);


   Serial.print("RATE: ");      
   Serial.println(8000);      

//--------------------------------------------------------------------------------
   // disable Core 0 WDT
   TaskHandle_t idle_0 = xTaskGetIdleTaskHandleForCPU(0);
   esp_task_wdt_delete(idle_0);


}


void loop()
{
DEFAULTPAL=0;
GBC_EXIT=0;

   screenmemory_fillscreen(63); //black color

   Serial.println("GBC_POWER: ##################");

   sprintf(MAINPATH, "/GBC/"); ///must be malloc(256);

   for (uint16_t tmp = 0; tmp < MAXFILES; tmp++) filename[tmp] = (char*)calloc(MAXFILENAME_LENGTH, sizeof(char));

      GBCBrowse(MAINPATH);
      delay(300);

      for (uint16_t tmp = 0; tmp < MAXFILES; tmp++) free(filename[tmp]);

///      if (EXIT) {
///        return; //return from GBCBrowse();
///      }

      screenmemory_fillscreen(63); //black color

      if (DEBUG) Serial.print("SELECTED GBC: ");
      if (DEBUG) Serial.println(MAINPATH);

//--------------------------------------------------------------------------------
//--------------------------------------------------------------------------------


///  vid_init();

//----------------------------------------
   uint32_t len;
///   game_rom=load_file("smb.gbc",&len);
   game_rom=load_file(MAINPATH,&len);

   
   game_rom_len=len;
//----------------------------------------

DEFAULTPAL=1;

   // setup the ROM image
   Serial.println("loading rom");
   rom_load(game_rom, game_rom_len);
   Serial.println("loaded!");

  // startup the emulator
   emu_reset();

   Serial.println("Starting emulator");
   emu_run();

   Serial.println("Terminated emulator");
   loader_unload();




}

int sys_elapsed(uint32_t * prev) {
   uint32_t now = sys_micros();
   uint32_t delta = now - *prev;
   *prev = now;
   //printf("elapsed: %d\n", delta);
   return delta;
}

void sys_sleep(int us) {
   if (us < 0) return;
   //printf("usleep(%d)\n", us);
   int start = sys_micros();
   while(sys_micros() - start < us) {
      doevents();
      delayMicroseconds(100);
   }
}

unsigned long sys_micros() {
  return micros();
}
