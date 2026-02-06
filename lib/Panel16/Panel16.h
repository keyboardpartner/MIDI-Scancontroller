/*
// #############################################################################
//       __ ________  _____  ____  ___   ___  ___
//      / //_/ __/\ \/ / _ )/ __ \/ _ | / _ \/ _ \
//     / ,< / _/   \  / _  / /_/ / __ |/ , _/ // /
//    /_/|_/___/_  /_/____/\____/_/_|_/_/|_/____/
//      / _ \/ _ | / _ \/_  __/ |/ / __/ _ \
//     / ___/ __ |/ , _/ / / /    / _// , _/
//    /_/  /_/ |_/_/|_| /_/ /_/|_/___/_/|_|
//
// #############################################################################
*/
// Class Lib for KBP MenuPanel, C. Meyer 1/2026
// Based on work from DFROBOT

#ifndef Panel16_h
#define Panel16_h

#include <inttypes.h>
#include "Print.h"
#include <Wire.h>

// LED-States:
// %00 = OFF, %01 = ON, %10 = PWM_0 (darker), %11= PWM_1 (brighter)
#define LED_OFF 0x00
#define LED_ON  0x01
#define LED_DIM_DARK 0x02
#define LED_DIM_BRIGHT 0x03

class Panel16 {
public:

  Panel16(uint8_t panel_Addr);
  void begin();
  void init();
  void setLEDdim(uint8_t dimlevel_bright, uint8_t dimlevel_dark);
  void setLEDsUpperByte(uint8_t row, uint8_t ledbyte);
  void setLEDsLowerByte(uint8_t row, uint8_t ledbyte);
  void setLEDsWord(uint8_t row, uint16_t ledword);
  void setLEDstate(uint8_t led, uint8_t ledstate); // set single LED 0..15 state LED_OFF, LED_ON, LED_DIM_DARK, LED_DIM_BRIGHT
  void toggleLEDstate(uint8_t led, uint8_t ledstate_if_on) ; // set single LED 0..15 state LED_OFF when on, LED_ON, LED_DIM_DARK, LED_DIM_BRIGHT when off
  uint8_t getButtonRow(uint8_t row); // get binary button states of given row as byte, 1 = pressed
  uint8_t getButtonRowWaitReleased(uint8_t row); // as above, wait for release of all buttons
  uint8_t buttonByteToNumber(uint8_t row, uint8_t buttonByte); // convert button byte to button number (0..7), 0xFF if none pressed

private:
  void init_priv();
  uint8_t _data[4];
  uint8_t _Addr;
  uint16_t _LEDsWord[2] = {0, 0};
  uint16_t _LEDsOnOff[2] = {0, 0};
};

#endif
