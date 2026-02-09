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
// Class Lib for KBP Panel16 I2C, Preset16 or Extend16, C. Meyer 1/2026

#ifndef Panel16_h
#define Panel16_h

#include <inttypes.h>
#include "Print.h"
#include <Wire.h>

class Panel16 {
public:
  Panel16(uint8_t panel_Addr);
  void begin();
  void init();
  void setLEDdim(uint8_t dimlevel_bright, uint8_t dimlevel_dark);
  void setLEDsUpperByte(uint8_t row, uint8_t ledbyte);
  void setLEDsLowerByte(uint8_t row, uint8_t ledbyte);
  void setLEDsWord(uint8_t row, uint16_t ledword);

  // LED States:
  // Bit 7 = Active/On, Bit 6 = Blinking, Bit 4,5 = OffState, Bit 2,3 = BlinkState, Bit 0,1 = OnState
  // mit State =%00 = OFF, %01 = ON, %10 = PWM_0 (darker), %11= PWM_1 (brighter)

  // LED-States Bit 0, 1: State if ON, led_off, led_on, led_dark, led_bright
  // LED-States Bit 2, 3: Alternative blinking state: led_off, led_on, led_dark, led_bright
  // LED-States Bit 6: led blinking if on/active
  // LED-States Bit 7: led active (on or blinking)
  void setLEDstate(uint8_t led, uint8_t ledstate); // set single LED 0..15 state led_off, led_on, led_dark or led_bright
  // %00 = OFF, %01 = ON, %10 = PWM_0 (darker), %11= PWM_1 (brighter)
  // Bit-Masken für steady-ON-Zustand
  const uint8_t led_off         = 0;
  const uint8_t led_hilight     = 0b00000001;
  const uint8_t led_dark        = 0b00000010;
  const uint8_t led_bright      = 0b00000011;
  // Bit-Masken für alternierenden Blink-Zustand
  const uint8_t led_alt_off     = 0;
  const uint8_t led_alt_hilight = 0b00000100;
  const uint8_t led_alt_dark    = 0b00001000;
  const uint8_t led_alt_bright  = 0b00001100;
  // Bit-Masken für Off-Zustand
  const uint8_t led_off_off     = 0; // Default-Off-Zustand, LED aus
  const uint8_t led_off_hilight = 0b00010000; // Off-Zustand, LED sehr hell, z.B. für Fehleranzeige
  const uint8_t led_off_dark    = 0b00100000; // Off-Zustand, LED sehr dunkel, z.B. für inaktive LEDs
  const uint8_t led_off_bright  = 0b00110000; // Off-Zustand, LED etwas heller als off_dark, z.B. für inaktive LEDs mit leichter Sichtbarkeit

  const uint8_t led_btn_on      = 0b10000000; // Bit 7: LED aktiv (on oder blinking) = Button-Zustand
  const uint8_t led_blink_ena   = 0b01000000; // Bit 6: LED blinking if on/active

  void setLEDonOff(uint8_t led, bool led_on); // set single LED 0..15 on or off, previous dim state is kept when switching on again

  uint8_t getLEDstate(uint8_t led); // get single LED 0..15 state led_off, led_on, led_dark or led_bright
  bool getLEDonOff(uint8_t led); // get single LED 0..15 button state, true = on, false = off (dim states are treated as on)
  void toggleLEDstate(uint8_t led) ; // set single LED 0..15 state led_off when on, otherwise led_on, led_dark, led_bright when off
  uint8_t getButtonRow(uint8_t row); // get binary button states of given row as byte, 1 = pressed
  uint8_t getButtonRowWaitReleased(uint8_t row); // as above, wait for release of all buttons
  void updateBlinkLEDs(); // toggle LEDs with blinking active, should be called periodically every few ms

  typedef void (*actionCallback)(uint8_t button);
  void setWaitCallback(actionCallback action) { _waitAction = action; } // Callback for wait loops, e.g. for button press handling, to avoid blocking calls

private:

  static void dummyAction(uint8_t button) { delay(10); } // debounce, in case user callback is not defined
  actionCallback _waitAction; // Callback for wait loops, e.g. for button press handling, to avoid blocking calls

  void init_priv();
  uint8_t _data[8];
  uint8_t _Addr;
  uint16_t _LEDsWord[2] = {0, 0};
  uint8_t _blinkToggle = 0;
  uint32_t _lastBlinkMillis = 0;

  // LED-States Bit 0, 1: State if ON, led_off, led_on, led_dark, led_bright
  // LED-States Bit 2, 3: Alternative blinking state: led_off, led_on, led_dark, led_bright
  // LED-States Bit 6: led blinking if on/active
  // LED-States Bit 7: led active (on or blinking)
  uint8_t _LEDstates[16];
};

#endif
