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
// Class Lib for KBP MenuPanel with rotary encoder and buttons, C. Meyer 1/2026
// Based on work from DFROBOT

#include "MenuPanel.h"
#include <inttypes.h>
#include "Arduino.h"

#define printIIC(args)	Wire.write(args)

inline size_t MenuPanel::write(uint8_t value) {
	send(value, RS_9555);
	_current_col++;
	return 1;
}

// When the display powers up, it is configured as follows:
//
// 1. Display clear
// 2. Function set:
//    DL = 1; 8-bit interface data
//    N = 0; 1-line display
//    F = 0; 5x8 dot character font
// 3. Display on/off control:
//    D = 0; Display off
//    C = 0; Cursor off
//    B = 0; Blinking off
// 4. Entry mode set:
//    I/D = 1; Increment by 1
//    S = 0; No shift
//
// Note, however, that resetting the Arduino doesn't reset the LCD, so we
// can't assume that its in that state when a sketch starts (and the
// LiquidCrystal constructor is called).

MenuPanel::MenuPanel(uint8_t lcd_Addr,uint8_t lcd_cols,uint8_t lcd_rows)
{
  _Addr = lcd_Addr;
  _cols = lcd_cols;
  _rows = lcd_rows;
}

void MenuPanel::init(){
	init_priv();
}

void MenuPanel::init_priv()
{
	begin(_cols, _rows);
}

bool MenuPanel::begin(uint8_t cols, uint8_t lines) {	// , uint8_t dotsize) {
	delay(50); // Wait for >40ms after power rises above 2.7V
  Wire.beginTransmission(_Addr); // Display I2C-Adresse
  if (Wire.endTransmission(true) != 0) {
    return false; // Kein Display gefunden
  }
	uint8_t data_[4];
	if (lines > 1) {
		_displayfunction = LCD_8BITMODE | LCD_2LINE;
	} else {
 	  _displayfunction = LCD_8BITMODE | LCD_1LINE;
	}
	_numlines = lines;
	// for some 1 line displays you can select a 10 pixel high font
	/*
	if ((dotsize != 0) && (lines == 1)) {
		_displayfunction |= LCD_5x10DOTS;
	}
	*/
	data_[0] = 0x06; // PCA9555 Command, 6 = DDR Port 0
	data_[1] = 0x00;
		// Send data
  Wire.beginTransmission(_Addr);
	Wire.write(data_, 2);   // 2 Bytes
	Wire.endTransmission();

	data_[0] = 0x07; // PCA9555 Command, 7 = DDR Port 1
	data_[1] = 0xF8; // PCA9555 Command, 7 = DDR Port 1
  Wire.beginTransmission(_Addr);
	Wire.write(data_, 2);   // 2 Bytes
	Wire.endTransmission();

	data_[0] = 0x02; // PCA9555 Command, 2 = Write to Port 0
	data_[1] = 0x00;
	data_[2] = 0xF8;
  Wire.beginTransmission(_Addr);
	Wire.write(data_, 3);   // 3 Bytes
	Wire.endTransmission();

	// SEE PAGE 45/46 FOR INITIALIZATION SPECIFICATION!
	// according to datasheet, we need at least 40ms after power rises above 2.7V
	// before sending commands. Arduino can turn on way before 4.5V so we'll wait 50
	// Now we pulled both RS and R/W low to begin commands

	delay(1);
  createChar(LCD_ARW_UP, arrowUp);
	createChar(LCD_ARW_DN, arrowDown);
	createChar(LCD_ARW_LT, arrowLeft);
	createChar(LCD_ARW_LT_GREY, arrowLeftGrey);
	createChar(LCD_ARW_UD, arrowUpDown);
	createChar(LCD_ARW_UD_GREY, arrowUpDownGrey);
	createChar(LCD_ARW_RT, arrowRight);
	createChar(LCD_ARW_RT_GREY, arrowRightGrey);

	// set # lines, font size, etc.
	command(LCD_FUNCTIONSET | _displayfunction); // 0x28
	// turn the display on with no cursor or blinking default
	_displaycontrol = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;
	display();
	// set the entry mode
	_displaymode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;
	command(LCD_ENTRYMODESET | _displaymode); //0x0C
	// clear it off
	clear();
	// Initialize to default text direction (for roman languages)
	home();
	return(true);
}

// #############################################################################
//
//     #        #####  ######
//     #       #     # #     #
//     #       #       #     #
//     #       #       #     #
//     #       #       #     #
//     #       #     # #     #
//     #######  #####  ######
//
// #############################################################################


/********** high level commands, for the user! */

void MenuPanel::clear(){
	command(LCD_CLEARDISPLAY);// clear display, set cursor position to zero
	delay(2);  // this command takes a long time!
	_current_col = 0; // for clrEOL
}

void MenuPanel::clearEOL(){
	for (uint8_t i = _current_col; i < _cols; i++) {
		write(' ');
	}
	_current_col = _cols; // for clrEOL
}

void MenuPanel::home(){
	command(LCD_RETURNHOME);  // set cursor position to zero
	delay(2);  // this command takes a long time!
	_current_col = 0; // for clrEOL
}

void MenuPanel::setCursor(uint8_t col, uint8_t row){
	int row_offsets[] = { 0x00, 0x40, 0x14, 0x54 };
	if ( row > _numlines ) {
		row = _numlines-1;    // we count rows starting w/0
	}
	command(LCD_SETDDRAMADDR | (col + row_offsets[row]));
	_current_col = col; // Update current column
}

// Alias for setCursor
void MenuPanel::cursorXY(uint8_t col, uint8_t row){
	setCursor(col, row);
}

// Turn the display on/off (quickly)
void MenuPanel::noDisplay() {
	_displaycontrol &= ~LCD_DISPLAYON;
	command(LCD_DISPLAYCONTROL | _displaycontrol);
}
void MenuPanel::display() {
	_displaycontrol |= LCD_DISPLAYON;
	command(LCD_DISPLAYCONTROL | _displaycontrol);
}

// Turns the underline cursor on/off
void MenuPanel::noCursor() {
	_displaycontrol &= ~LCD_CURSORON;
	command(LCD_DISPLAYCONTROL | _displaycontrol);
}
void MenuPanel::cursor() {
	_displaycontrol |= LCD_CURSORON;
	command(LCD_DISPLAYCONTROL | _displaycontrol);
}

// Turn on and off the blinking cursor
void MenuPanel::noBlink() {
	_displaycontrol &= ~LCD_BLINKON;
	command(LCD_DISPLAYCONTROL | _displaycontrol);
}
void MenuPanel::blink() {
	_displaycontrol |= LCD_BLINKON;
	command(LCD_DISPLAYCONTROL | _displaycontrol);
}

// These commands scroll the display without changing the RAM
void MenuPanel::scrollDisplayLeft(void) {
	command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVELEFT);
}
void MenuPanel::scrollDisplayRight(void) {
	command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVERIGHT);
}

// This will 'right justify' text from the cursor
void MenuPanel::autoscroll(void) {
	_displaymode |= LCD_ENTRYSHIFTINCREMENT;
	command(LCD_ENTRYMODESET | _displaymode);
}

// This will 'left justify' text from the cursor
void MenuPanel::noAutoscroll(void) {
	_displaymode &= ~LCD_ENTRYSHIFTINCREMENT;
	command(LCD_ENTRYMODESET | _displaymode);
}

// Allows us to fill the first 8 CGRAM locations
// with custom characters
void MenuPanel::createChar(uint8_t location, const uint8_t charmap[]) {
	location &= 0x7; // we only have 8 locations 0-7
	command(LCD_SETCGRAMADDR | (location << 3));
	for (int i=0; i<8; i++) {
		write(charmap[i]);
	}
}

/*********** mid level commands, for sending data/cmds */

inline void MenuPanel::command(uint8_t value) {
	send(value, 0);
}

/************ low level data pushing commands **********/

// write either command (mode = 0) or data (mode = RS_9555)
void MenuPanel::send(uint8_t value, uint8_t mode) {
	uint8_t data_[4];
	data_[0] = 0x02; // PCA9555 Command, 2 = Write to Port 0
	data_[1] = value;
	data_[2] = mode; // AutoInc to Port 1
	// Send data
  Wire.beginTransmission(_Addr);
	Wire.write(data_, 3);   // 3 Bytes
	Wire.endTransmission();

	// Pulse Enable pin
	data_[0] = 0x03; // PCA9555 Command, 3 = Write to Port 1
	data_[1] = mode | EN_9555 | 0xF8;
  Wire.beginTransmission(_Addr);
	Wire.write(data_, 2);   // 2 Bytes
	Wire.endTransmission();
	delayMicroseconds(5);

	data_[1] = mode | 0xF8;
  Wire.beginTransmission(_Addr);
	Wire.write(data_, 2);   // 2 Bytes
	Wire.endTransmission();
	delayMicroseconds(50);
}

// #############################################################################
//
//     ######  #     # ####### ####### ####### #     #  #####
//     #     # #     #    #       #    #     # ##    # #     #
//     #     # #     #    #       #    #     # # #   # #
//     ######  #     #    #       #    #     # #  #  #  #####
//     #     # #     #    #       #    #     # #   # #       #
//     #     # #     #    #       #    #     # #    ## #     #
//     ######   #####     #       #    ####### #     #  #####
//
// #############################################################################

// read a byte from PCA9555 port 1 (buttons)
// benötigt etwa 130 µs (inkl. I2C Overhead) bei 400 kHz
uint8_t MenuPanel::getButtons() {
	uint8_t data_byte;
  Wire.beginTransmission(_Addr);
	Wire.write(0x01);   // 1 Byte, Port 1 lesen
  Wire.endTransmission();
  Wire.requestFrom(_Addr, (uint8_t)1);
  data_byte = Wire.read();
	return ~(data_byte >>3) & 0x07; // Nur 3 Bits sind Tasten
}

uint8_t MenuPanel::getButtonsWaitReleased(uint16_t timeout_ms) {
	uint8_t buttons;
	uint32_t timeout_time = millis() + timeout_ms;
	do {
		buttons = getButtons();
	} while ((buttons != 0) && ((millis() < timeout_time) || (timeout_ms == 0))); // Warten bis alle Tasten losgelassen sind oder Timeout
	return buttons; // enthält bei Timeout die noch gedrückten Tasten, ansonsten 0
}

// #############################################################################
//
//     ####### #     #  #####  ####### ######  ####### ######
//     #       ##    # #     # #     # #     # #       #     #
//     #       # #   # #       #     # #     # #       #     #
//     #####   #  #  # #       #     # #     # #####   ######
//     #       #   # # #       #     # #     # #       #   #
//     #       #    ## #     # #     # #     # #       #    #
//     ####### #     #  #####  ####### ######  ####### #     #
//
// #############################################################################

int16_t MenuPanel::getEncoderDelta() {
  int16_t delta = 0;
  uint8_t currentState = (PINC & B00001100) >> 2; // Nur die beiden relevanten Bits lesen und nach rechts verschieben
  if (currentState != _lastState) {
    // Zustandsänderung erkannt, nur ganze Schritte zählen
    if ((_lastState == 0b00 && currentState == 0b10)) {
      delta = 1; // Vorwärts
    } else if ((_lastState == 0b00  && currentState == 0b01)) {
      delta = -1; // Rückwärts
    }
    _lastState = currentState;
  }
  return delta;
}


// #############################################################################
// Alias functions
// #############################################################################

void MenuPanel::cursor_on(){
	cursor();
}

void MenuPanel::cursor_off(){
	noCursor();
}

void MenuPanel::blink_on(){
	blink();
}

void MenuPanel::blink_off(){
	noBlink();
}

void MenuPanel::load_custom_character(uint8_t char_num, uint8_t *rows){
	createChar(char_num, rows);
}

void MenuPanel::printstr(const char c[]){
	//This function is not identical to the function used for "real" I2C displays
	//it's here so the user sketch doesn't have to be changed
	print(c);
}

