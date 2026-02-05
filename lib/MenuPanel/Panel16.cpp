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
// Class Lib for KBP Panel16 I2C C. Meyer 1/2026

#include "Panel16.h"
#include <inttypes.h>
#include "Arduino.h"


Panel16::Panel16(uint8_t panel_Addr)
{
  _Addr = panel_Addr;
}

void Panel16::init(){
	init_priv();
}

void Panel16::init_priv()
{
	begin();
}

void Panel16::begin() {
	setLEDdim(0x20, 0x08); // Dim Settings: bright, dark
  setLEDsWord(0, 0); // Clear lower row
  setLEDsWord(1, 0); // Clear upper row
}

void Panel16::setLEDdim(uint8_t dimlevel_bright, uint8_t dimlevel_dark) {
	_data[0] = 0x02; // PCA9532 Command, 2 = PWM Frequency
	_data[1] = 0x00;
		// Send data
  Wire.beginTransmission(_Addr);
	Wire.write(_data, 2);   // 2 Bytes
	Wire.endTransmission();

	_data[0] = 0x03; // PCA9532 Command, 3 = LED brightness
	_data[1] = dimlevel_dark; // darkest setting
	// Send data
  Wire.beginTransmission(_Addr);
	Wire.write(_data, 2);   // 2 Bytes
	Wire.endTransmission();

	_data[0] = 0x04; // PCA9532 Command, 4 = PWM Frequency
	_data[1] = 0x00;
	// Send data
  Wire.beginTransmission(_Addr);
	Wire.write(_data, 2);   // 2 Bytes
	Wire.endTransmission();

	_data[0] = 0x05; // PCA9532 Command, 5 = LED brightness
	_data[1] = dimlevel_bright; // dimmed
	// Send data
  Wire.beginTransmission(_Addr);
	Wire.write(_data, 2);   // 2 Bytes
	Wire.endTransmission();
}

void Panel16::setLEDsUpperByte(uint8_t row, uint8_t ledbyte) {
  // LEDs 4..7 einer Row setzen
  // %00 = OFF, %01 = ON, %10 = PWM_0 (darker), %11= PWM_1 (brighter)
  // row 0 = lower row, 1 = upper row
  _data[0] = 0x07 + row * 2; // PCA9532 Command, 7 = LED 4-7
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
  _data[0] = 0x06 + row * 2; // PCA9532 Command, 6 = LED 0-3
  _data[1] = ledbyte;
	// Send data
  Wire.beginTransmission(_Addr);
	Wire.write(_data, 2);   // 2 Bytes
	Wire.endTransmission();
}

void Panel16::setLEDsWord(uint8_t row, uint16_t ledword) {
  // %00 = OFF, %01 = ON, %10 = PWM_0 (darker), %11= PWM_1 (brighter)
  // row 0 = lower row, 1 = upper row
  setLEDsLowerByte(row, (uint8_t)ledword);
  setLEDsUpperByte(row, (uint8_t)(ledword >> 8));
}

void Panel16::setLEDstate(uint8_t led, uint8_t ledstate) {
  // led 0 = lower left, 15 = upper right
  // ledstate %00 = OFF, %01 = ON, %10 = PWM_0 (darker), %11= PWM_1 (brighter)
  // set led 0..15 to ledstate
  uint8_t row = led / 8;
  _LEDsWord[row] &= ~(0b11 << ((led % 8) * 2)); // Alte Bits löschen
  _LEDsWord[row] |= (ledstate & 0b11) << ((led % 8) * 2); // Neue Bits setzen
  _LEDsOnOff[row] &= ~(1 << led); // Alte On/Off Bits löschen
  if (ledstate) {
    _LEDsOnOff[row] |= (1 << led); // Neue On/Off Bits setzen
  }
  // nur die nötigen Ports aktualisieren
  if (led % 8 < 4) {
    setLEDsLowerByte(row, (uint8_t)_LEDsWord[row]);
  } else {
    setLEDsUpperByte(row, (uint8_t)(_LEDsWord[row] >> 8));
  }
}

void Panel16::toggleLEDstate(uint8_t led, uint8_t ledstate_if_on) {
  // led 0 = lower left, 15 = upper right
  // ledstate %00 = OFF, %01 = ON, %10 = PWM_0 (darker), %11= PWM_1 (brighter)
  // set led 0..15 to ledstate
  uint8_t row = led / 8;
  uint8_t currentState = _LEDsOnOff[row] & (1 << led);
  if (ledstate_if_on == 0) {
    // nur ausschalten
    setLEDstate(led, LED_OFF);
    return;
  }
  if (currentState) {
    setLEDstate(led, LED_OFF);
  } else {
    setLEDstate(led, ledstate_if_on);
  }
}

uint8_t Panel16::getButtonRow(uint8_t row) {
  // return button states of given row, 1 = pressed
  // row 0 = lower row, 1 = upper row
  setLEDsWord(row, 0); // Alle LEDs in der Reihe aus
	uint8_t _databyte;
  Wire.beginTransmission(_Addr);
	Wire.write(row);   // 1 Byte
  Wire.endTransmission();
  Wire.requestFrom(_Addr, (uint8_t)1);
  _databyte = Wire.read();
  setLEDsWord(row, _LEDsWord[row]); // LEDs wiederherstellen
  _databyte = ~(_databyte); // Nur 8 Bits sind Tasten
  return _databyte;
}

uint8_t Panel16::buttonByteToNumber(uint8_t row, uint8_t buttonByte) {
  // return button number (0..7) of given buttonByte in row, 0xFF if none pressed
  // row 0 = lower row, 1 = upper row
	uint8_t _btnNumber;
  if (buttonByte == 0) return 0xFF; // keine Taste gedrückt
  for (_btnNumber = 0; _btnNumber < 8; _btnNumber++) {
    if (buttonByte & (1 << _btnNumber)) {
      return _btnNumber + row * 8;
    }
  }
  return 0xFF;
}

uint8_t Panel16::getButtonRowWaitReleased(uint8_t row) {
  // return button number (0..7) of given row, wait until all buttons released
  // row 0 = lower row, 1 = upper row
  uint8_t buttons, last_buttons;
  last_buttons = 0;
  do {
    buttons = getButtonRow(row);
    if (buttons != 0) {
      last_buttons = buttons;
    }
    delay(10); // Entprellzeit
  } while (buttons != 0); // Warten bis alle Tasten losgelassen sind
  return last_buttons;
}