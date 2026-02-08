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

#ifndef MenuPanel_h
#define MenuPanel_h

#include <inttypes.h>
#include "Print.h"
#include <Wire.h>

// HD44780U defines
#define LCD_I2C_ADDR 0x20

// commands
#define LCD_CLEARDISPLAY 0x01
#define LCD_RETURNHOME 0x02
#define LCD_ENTRYMODESET 0x04
#define LCD_DISPLAYCONTROL 0x08
#define LCD_CURSORSHIFT 0x10
#define LCD_FUNCTIONSET 0x20
#define LCD_SETCGRAMADDR 0x40
#define LCD_SETDDRAMADDR 0x80

// flags for display entry mode
#define LCD_ENTRYRIGHT 0x00
#define LCD_ENTRYLEFT 0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00

// flags for display on/off control
#define LCD_DISPLAYON 0x04
#define LCD_DISPLAYOFF 0x00
#define LCD_CURSORON 0x02
#define LCD_CURSOROFF 0x00
#define LCD_BLINKON 0x01
#define LCD_BLINKOFF 0x00

// flags for display/cursor shift
#define LCD_DISPLAYMOVE 0x08
#define LCD_CURSORMOVE 0x00
#define LCD_MOVERIGHT 0x04
#define LCD_MOVELEFT 0x00

// flags for function set
#define LCD_8BITMODE 0x10
#define LCD_4BITMODE 0x00
#define LCD_2LINE 0x08
#define LCD_1LINE 0x00
#define LCD_5x10DOTS 0x04
#define LCD_5x8DOTS 0x00

#define EN_9555 B00000001  // Enable bit
#define RD_9555 B00000010  // Read data bit (unused)
#define RS_9555 B00000100  // Register select bit

// Define buttons bitmasks for KBP MenuPanel
#define LCD_BTNENTER_MASK 0x01
#define LCD_BTNUP_MASK 0x02
#define LCD_BTNDN_MASK 0x04
#define LCD_ALLBUTTONS_MASK (LCD_BTNENTER_MASK | LCD_BTNUP_MASK | LCD_BTNDN_MASK)
#define LCD_BTNENTER_BIT 0x00
#define LCD_BTNUP_BIT 0x01
#define LCD_BTNDN_BIT 0x02

#define LCD_ARW_UP char(0)
#define LCD_ARW_DN char(1)
#define LCD_ARW_DN char(1)
#define LCD_ARW_LT char(2)
#define LCD_ARW_LT_GREY char(3)
#define LCD_ARW_UD char(4)
#define LCD_ARW_UD_GREY char(5)
#define LCD_ARW_RT char(6)
#define LCD_ARW_RT_GREY char(7)

// Dieser Hack wird benötigt, um die Menü-Items im Flash-Speicher zu halten, da das RAM beim ATmega328P knapp ist.
// Menü-Items sind maximal 16 Zeichen lang, damit sie genau in eine LCD-Zeile passen, plus Nullterminator
typedef struct {
  char lcdText [16];
} lcdTextType;

class MenuPanel : public Print {
public:
  const uint8_t arrowUp[8] = {0x04, 0x0E, 0x15, 0x04, 0x04, 0x04, 0x04, 0x00};
  const uint8_t arrowDown[8] = {0x04,0x04,0x04,0x04,0x15,0x0E,0x04,0x00};
  const uint8_t arrowLeft[8] = {0x02, 0x06, 0x0E, 0x1E, 0x0E, 0x06, 0x02, 0x00}; // "<" Cursor filled (white}
  const uint8_t arrowLeftGrey[8] = {0x02, 0x04, 0x0A, 0x14, 0x0A, 0x04, 0x02, 0x00};  // "<" Cursor grey
  const uint8_t arrowUpDown[8] = {0x04, 0x0E, 0x1F, 0x00, 0x1F, 0x0E, 0x04, 0x00}; // Updown (white}
  const uint8_t arrowUpDownGrey[8] = {0x04, 0x0A, 0x15, 0x00, 0x15, 0x0A, 0x04, 0x00}; // Updown (grey}
  const uint8_t arrowRight[8] = {0x08, 0x0C, 0x0E, 0x0F, 0x0E, 0x0C, 0x08, 0x00}; // ">" Cursor filled (white}
  const uint8_t arrowRightGrey[8] = {0x08, 0x04, 0x0A, 0x04, 0x0A, 0x04, 0x08, 0x00}; // ">" Cursor grey


  MenuPanel(uint8_t lcd_Addr,uint8_t lcd_cols,uint8_t lcd_rows);
  void begin(uint8_t cols, uint8_t rows); // , uint8_t charsize = LCD_5x8DOTS );
  void clear();
  void clearEOL();
  void home();
  void noDisplay();
  void display();
  void noBlink();
  void blink();
  void noCursor();
  void cursor();
  void scrollDisplayLeft();
  void scrollDisplayRight();
  void autoscroll();
  void noAutoscroll();
  void createChar(uint8_t, const uint8_t[]);
  void setCursor(uint8_t, uint8_t);
  void cursorXY(uint8_t, uint8_t);
  virtual size_t write(uint8_t);
  void command(uint8_t);
  void init();
  uint8_t getButtons();
  uint8_t getButtonsWaitReleased(uint16_t timeout_ms); // as above, wait for release of all buttons
  int16_t getEncoderDelta();

  //compatibility API function aliases
  void blink_on();						// alias for blink()
  void blink_off();       					// alias for noBlink()
  void cursor_on();      	 					// alias for cursor()
  void cursor_off();      					// alias for noCursor()
  void load_custom_character(uint8_t char_num, uint8_t *rows);	// alias for createChar()
  void printstr(const char[]);

  void printProgmem(const lcdTextType* progmem_str) {
    // Kopiere MenuItem aus PROGMEM ins RAM, da lcd.print() nicht direkt aus PROGMEM lesen kann
    lcdTextType oneItem;
    memcpy_P (&oneItem, progmem_str, sizeof oneItem);
    print(oneItem.lcdText);
  }

private:
  void init_priv();
  void send(uint8_t data, uint8_t mode);
  uint8_t _Addr;
  uint8_t _displayfunction;
  uint8_t _displaycontrol;
  uint8_t _displaymode;
  uint8_t _numlines;
  uint8_t _cols;
  uint8_t _rows;
  uint8_t _current_col;
  uint8_t _lastState = B00000011; // Initialer Zustand der Encoder-Bits
  int16_t _encoderPosition = 0;
  int16_t _encoderDelta = 0;
};

#endif
