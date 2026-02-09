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

#include "Panel16.h"
#include <inttypes.h>
#include "Arduino.h"


Panel16::Panel16(uint8_t panel_Addr)
{
  _Addr = panel_Addr;
}

void Panel16::init() {
	init_priv();
}

void Panel16::init_priv() {
	begin();
}

void Panel16::begin() {
  _waitAction = dummyAction; // Pressed action callback auf default setzen, damit er nicht ins Leere läuft, falls er nicht definiert ist
	setLEDdim(0x28, 0x10); // Dim Settings: bright, dark
  setLEDsWord(0, 0); // Clear lower row
  setLEDsWord(1, 0); // Clear upper row
  for (int i = 0; i < 16; i++) {
    _LEDstates[i] = led_bright; // Default-Zustand für alle LEDs, Bit 6 und 7 = 0, also nicht aktiv
  }
}

void Panel16::setLEDdim(uint8_t dimlevel_bright, uint8_t dimlevel_dark) {
	_data[0] = 0x12; // PCA9532 Command, 2 = PWM Frequency, AutoIncrement on
	_data[1] = 0x00; // PWM Frequency = 152 / (n+1) Hz
	// PCA9532 Command, 3 = LED brightness
	_data[2] = dimlevel_dark; // darkest setting
  // PCA9532 Command, 4 = PWM Frequency
	_data[3] = 0x00;
	// PCA9532 Command, 5 = LED brightness
	_data[4] = dimlevel_bright; // dimmed
	// Send data
  Wire.beginTransmission(_Addr);
	Wire.write(_data, 5);   // 5 Bytes
	Wire.endTransmission();
}

void Panel16::setLEDsUpperByte(uint8_t row, uint8_t ledbyte) {
  // LEDs 4..7 einer Row setzen
  // %00 = OFF, %01 = ON, %10 = PWM_0 (darker), %11= PWM_1 (brighter)
  // row 0 = lower row, 1 = upper row
  _data[0] = 0x07 + row * 2; // PCA9532 Command, 7 = LED 4-7, AutoIncrement off
  _data[1] = ledbyte;
	// Send data
  Wire.beginTransmission(_Addr);
	Wire.write(_data, 2);   // 2 Bytes
	Wire.endTransmission();
}

void Panel16::setLEDsLowerByte(uint8_t row, uint8_t ledbyte) {
  // LEDs 0..3 einer Row setzen
  // %00 = OFF, %01 = ON, %10 = PWM_0 (darker), %11= PWM_1 (brighter)
  // row 0 = lower row, 1 = upper row
  _data[0] = 0x06 + row * 2; // PCA9532 Command, 6 = LED 0-3, AutoIncrement off
  _data[1] = ledbyte;
	// Send data
  Wire.beginTransmission(_Addr);
	Wire.write(_data, 2);   // 2 Bytes
	Wire.endTransmission();
}

void Panel16::setLEDsWord(uint8_t row, uint16_t ledword) {
  // %00 = OFF, %01 = ON, %10 = PWM_0 (darker), %11= PWM_1 (brighter)
  // row 0 = lower row, 1 = upper row
  _data[0] = 0x16 + row * 2; // PCA9532 Command, 6 = LED 0-3, AutoIncrement on
  _data[1] = (uint8_t)ledword;
  _data[2] = (uint8_t)(ledword >> 8);
	// Send data
  Wire.beginTransmission(_Addr);
	Wire.write(_data, 3);   // 3 Bytes
	Wire.endTransmission();
}

void Panel16::setLEDstate(uint8_t led, uint8_t ledstate) {
  // Bit 0, 1: State if ON, led_off, led_on, led_dark, led_bright
  // Bit 2, 3: Alternative blinking state: led_off, led_on, led_dark, led_bright
  // Bit 6: led blinking active
  // Bit 7: led active (on or blinking)
  // set led 0..15 to ledstate
  _LEDstates[led] = ledstate;
  uint8_t row = led / 8;
  uint8_t led_mod8 = (led % 8) * 2; // Position der 2 Bits für die LED in der Word
  uint8_t led_alt_state = (ledstate & 0b1100) >> 2; // Alternative Blink-LED-Zustände oder normaler LED-Zustand
  _LEDsWord[row] &= ~(0b11 << led_mod8); // Alte Bits löschen
  if (ledstate & B10000000) {
    // LED an, normaler LED-Zustand in Bit 0 und 1 nutzen, bei Blinken alternierend Blink-LED-Zustände in Bit 2 und 3
    if ((ledstate & B01000000) && _blinkToggle) {
      // Blinken aktiv, alternative Blink-LED-Zustände in Bit 2 und 3
      _LEDsWord[row] |= led_alt_state << led_mod8; // Blink-LED-Zustände setzen
    } else {
      // Kein Blinken, normaler LED-Zustand in Bit 0 und 1
      _LEDsWord[row] |= (ledstate & 0b11) << led_mod8; // LED-Zustand setzen
    }
  } else {
    // LED aus, Off-Zustand in Bit 4 und 5 nutzen, bei Blinken alternierend Blink-LED-Zustände in Bit 2 und 3
    if ((ledstate & B01000000) && _blinkToggle) {
      _LEDsWord[row] |= led_alt_state << led_mod8; // Blink-LED-Zustände setzen
    } else {
      uint8_t led_off_state = (ledstate & 0b00110000) >> 4; // Alternative Blink-LED-Zustände oder normaler LED-Zustand
      _LEDsWord[row] |= led_off_state << led_mod8; // Blink-LED-Zustände setzen, auch wenn LED aus ist
    }
  }
  // nur die nötigen Ports aktualisieren
  if (led_mod8 < 8) {
    setLEDsLowerByte(row, (uint8_t)_LEDsWord[row]);
  } else {
    setLEDsUpperByte(row, (uint8_t)(_LEDsWord[row] >> 8));
  }
}

uint8_t Panel16::getLEDstate(uint8_t led) {
  // Bit 0, 1: State if ON, led_off, led_on, led_dark, led_bright
  // Bit 2, 3: Alternative blinking state: led_off, led_on, led_dark, led_bright
  // Bit 6: led blinking active
  // Bit 7: led active (on or blinking)

  // led 0 = lower left, 15 = upper right
  // return ledstate %00 = OFF, %01 = ON, %10 = PWM_0 (darker), %11= PWM_1 (brighter)
  return _LEDstates[led];
}

void Panel16::updateBlinkLEDs() {
  // LEDs mit aktivem Blinken toggeln, sollte alle paar ms aufgerufen werden
  uint32_t currentMillis = millis();
  if (currentMillis - _lastBlinkMillis >= 350) { // Blink-Intervall von 500 ms
    _lastBlinkMillis = currentMillis;
    _blinkToggle = !_blinkToggle;
    for (uint8_t led = 0; led < 16; led++) {
      uint8_t led_state = _LEDstates[led]; // Alternative Blink-LED-Zustände oder normaler LED-Zustand
      if (led_state & led_blink_ena) {
        setLEDstate(led, led_state); // Blinken toggeln
      }
    }
  }
}

void Panel16::toggleLEDstate(uint8_t led) {
  // led 0 = lower left, 15 = upper right
  // ledstate %00 = OFF, %01 = ON, %10 = PWM_0 (darker), %11= PWM_1 (brighter)
  // set led 0..15 to ledstate
  uint8_t ledstate = _LEDstates[led];
  uint8_t ledstate_inactive = ledstate & 0b01111111; // Aktuellen LED-Zustand (ON, dim_dark, dim_bright) extrahieren
  if (ledstate & 0b10000000) {
    setLEDstate(led, ledstate_inactive);
  } else {
    setLEDstate(led, ledstate_inactive | 0b10000000); // LED einschalten, vorherigen Zustand beibehalten
  }
}

void Panel16::setLEDonOff(uint8_t led, bool led_on) {
  // led 0 = lower left, 15 = upper right
  // ledstate %00 = OFF, %01 = ON, %10 = PWM_0 (darker), %11= PWM_1 (brighter)
  // set led 0..15 to ledstate
  uint8_t ledstate_inactive = _LEDstates[led] & 0b01111111; // Aktuellen LED-Zustand (ON, dim_dark, dim_bright) extrahieren
  if (led_on) {
    setLEDstate(led, ledstate_inactive | 0b10000000); // LED einschalten, vorherigen Zustand beibehalten
  } else {
    setLEDstate(led, ledstate_inactive);
  }
}

bool Panel16::getLEDonOff(uint8_t led) {
  // led 0 = lower left, 15 = upper right
  // return true if LED (i.e. button) is on (including dim states), false if off
  return ((_LEDstates[led] & 0b10000000) != 0); // LED ist an, wenn Bit 7 gesetzt ist
}

uint8_t Panel16::getButtonRow(uint8_t row) {
  // return button number of given row or 0xFF if none pressed
  // row 0 = lower row, 1 = upper row
  setLEDsWord(row, 0); // Alle LEDs in der Reihe aus
  Wire.beginTransmission(_Addr);
	Wire.write(row);   // 1 Byte
  Wire.endTransmission();
  Wire.requestFrom(_Addr, 1);
  uint8_t _databyte = Wire.read();
  setLEDsWord(row, _LEDsWord[row]); // LEDs wiederherstellen
  _databyte = ~(_databyte); // Nur 8 Bits sind Tasten
	uint8_t _btnNumber;
  if (_databyte == 0) return 0xFF; // keine Taste gedrückt
  for (_btnNumber = 0; _btnNumber < 8; _btnNumber++) {
    if (_databyte & (1 << _btnNumber)) {
      return _btnNumber + row * 8;
    }
  }
}


uint8_t Panel16::getButtonRowWaitReleased(uint8_t row) {
  // return button number (0..7) of given row, wait until all buttons released
  // row 0 = lower row, 1 = upper row
  uint8_t button, last_button;
  last_button = 0xFF;
  do {
    button = getButtonRow(row);
    if (button != 0xFF) {
      last_button = button;
    }
    updateBlinkLEDs(); // LEDs mit aktivem Blinken weiter toggeln
    // Callback-Funktion für Panel16, liefert derzeit gedrückten Panel16-Button
    _waitAction(button);
  } while (button != 0xFF); // Warten bis alle Tasten losgelassen sind
  return last_button;
}