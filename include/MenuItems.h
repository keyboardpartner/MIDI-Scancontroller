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
// MIDI Controller Menu Item IDs


#ifndef MenuItems_h
#define MenuItems_h

#include <Wire.h>
#include "MenuPanel.h"
#include <EEPROM.h>

#define EEPROM_MENUDEFAULTS 16 // Startadresse im EEPROM für gespeicherte Werte

// Default MIDI Einstellungen
#define MIDI_BASE_UPR 36
#define MIDI_BASE_LWR 36
#define MIDI_BASE_PED 36

#define MIDI_CH_UPR 1
#define MIDI_CH_LWR 2
#define MIDI_CH_PED 3

#define MIDI_MINDYN 10
#define MIDI_DYNSLOPE 12
#define MIDI_MAXDYNADJ 5

// Menu System Variables

#define MENU_DRIVERCOUNT 4
enum {drv_sr61, drv_fatar1, drv_fatar2, drv_custom};
const lcdTextType DriverTypes[MENU_DRIVERCOUNT] PROGMEM = { 
  { "Scan16/61" }, 
  { "FatarScan1-61" }, 
  { "FatarScan2" }, 
  { "Custom" },
};


#define MENU_ITEMCOUNT 49

// ------------------------------------------------------------------------------
// Hier Daten aus Excel-Tabelle einfügen, die die Menüstruktur definiert. 
// Es müssen 1 enum-Liste und 5 Arrays mit gleicher Länge angelegt werden.
// MenuLink[MENU_ITEMCOUNT] definiert die Menüstruktur:
// 0 normaler Edit-Menüpunkt, der mit Encoder geändert werden kann
// >0 ist die Nummer des Submenüpunktes, zu dem verlinkt wird
// -1 Rücksprungmöglichkeit (Exit) zum Hauptmenü
// ------------------------------------------------------------------------------

enum {
  m_upper_channel, // #0 = Upper Channel
  m_lower_channel, // #1 = Lower Channel
  m_pedal_channel, // #2 = Pedal Channel
  m_kbd_subm, // #3 = Keyboard
  m_cc_subm, // #4 = Pot Assign
  m_button_subm, // #5 = Button Assign
  m_end, // #6 = (end)
  m_CC1, // #7 = Pot 1 CC
  m_CC2, // #8 = Pot 2 CC
  m_CC3, // #9 = Pot 3 CC
  m_CC4, // #10 = Pot 4 CC
  m_CC5, // #11 = Pot 5 CC
  m_CC6, // #12 = Pot 6 CC
  m_CC7, // #13 = Pot 7 CC
  m_CC8, // #14 = Pot 8 CC
  m_CC9, // #15 = Pot 9 CC
  m_CC10, // #16 = Pot 10 CC
  m_CC11, // #17 = Pot 11 CC
  m_CC12, // #18 = Pot 12 CC
  m_CC13, // #19 = Pot 13 CC
  m_CC14, // #20 = Pot 14 CC
  m_CC15, // #21 = Pot 15 CC
  m_CC16, // #22 = Pot 16 CC
  m_cc_back, // #23 = Pot Assign EXIT 
  m_btn1, // #24 = Button 1
  m_btn2, // #25 = Button 2
  m_btn3, // #26 = Button 3
  m_btn4, // #27 = Button 4
  m_btn5, // #28 = Button 5
  m_btn6, // #29 = Button 6
  m_btn7, // #30 = Button 7
  m_btn8, // #31 = Button 8
  m_btn9, // #32 = Button 9
  m_btn10, // #33 = Button 10
  m_btn11, // #34 = Button 11
  m_btn12, // #35 = Button 12
  m_btn13, // #36 = Button 13
  m_btn14, // #37 = Button 14
  m_btn15, // #38 = Button 15
  m_btn16, // #39 = Button 16
  m_btn_back, // #40 = Button Assign EXIT 
  m_driver_type, // #41 = Kbd Driver
  m_mindyn, // #42 = Velocity Min
  m_maxdynadj, // #43 = Velocity MaxAdj
  m_slope, // #44 = Velocity Slope
  m_upper_base, // #45 = Upper Base
  m_lower_base, // #46 = Lower Base
  m_pedal_base, // #47 = Pedal Base
  m_kbd_back, // #48 = Keyboard EXIT 
};

const lcdTextType MenuItems[MENU_ITEMCOUNT] PROGMEM = { 
  { "Upper Channel" },  // #0 
  { "Lower Channel" },  // #1 
  { "Pedal Channel" },  // #2 
  { "Keyboard" },  // #3 
  { "Pot Assign" },  // #4 
  { "Button Assign" },  // #5 
  { "(end)" },  // #6 
  { "Pot 1 CC" },  // #7 
  { "Pot 2 CC" },  // #8 
  { "Pot 3 CC" },  // #9 
  { "Pot 4 CC" },  // #10 
  { "Pot 5 CC" },  // #11 
  { "Pot 6 CC" },  // #12 
  { "Pot 7 CC" },  // #13 
  { "Pot 8 CC" },  // #14 
  { "Pot 9 CC" },  // #15 
  { "Pot 10 CC" },  // #16 
  { "Pot 11 CC" },  // #17 
  { "Pot 12 CC" },  // #18 
  { "Pot 13 CC" },  // #19 
  { "Pot 14 CC" },  // #20 
  { "Pot 15 CC" },  // #21 
  { "Pot 16 CC" },  // #22 
  { "Pot Assign" },  // #23  EXIT SUBM
  { "Btn 1 CC" },  // #24 
  { "Btn 2 CC" },  // #25 
  { "Btn 3 CC" },  // #26 
  { "Btn 4 CC" },  // #27 
  { "Btn 5 CC" },  // #28 
  { "Btn 6 CC" },  // #29 
  { "Btn 7 CC" },  // #30 
  { "Btn 8 CC" },  // #31 
  { "Btn 9 CC" },  // #32 
  { "Btn 10 CC" },  // #33 
  { "Btn 11 CC" },  // #34 
  { "Btn 12 CC" },  // #35 
  { "Btn 13 CC" },  // #36 
  { "Btn 14 CC" },  // #37 
  { "Btn 15 CC" },  // #38 
  { "Btn 16 CC" },  // #39 
  { "Button Assign" },  // #40  EXIT SUBM
  { "Kbd Driver" },  // #41 
  { "Velocity Min" },  // #42 
  { "Velocity MaxAdj" },  // #43 
  { "Velocity Slope" },  // #44 
  { "Upper Base" },  // #45 
  { "Lower Base" },  // #46 
  { "Pedal Base" },  // #47 
  { "Keyboard" },  // #48  EXIT SUBM
};

int8_t MenuValueMin[MENU_ITEMCOUNT] = {
  1, // #0 = Upper Channel
  1, // #1 = Lower Channel
  1, // #2 = Pedal Channel
  0, // #3 = Keyboard
  0, // #4 = Pot Assign
  0, // #5 = Button Assign
  0, // #6 = (end)
  -1, // #7 = Pot 1 CC
  -1, // #8 = Pot 2 CC
  -1, // #9 = Pot 3 CC
  -1, // #10 = Pot 4 CC
  -1, // #11 = Pot 5 CC
  -1, // #12 = Pot 6 CC
  -1, // #13 = Pot 7 CC
  -1, // #14 = Pot 8 CC
  -1, // #15 = Pot 9 CC
  -1, // #16 = Pot 10 CC
  -1, // #17 = Pot 11 CC
  -1, // #18 = Pot 12 CC
  -1, // #19 = Pot 13 CC
  -1, // #20 = Pot 14 CC
  -1, // #21 = Pot 15 CC
  -1, // #22 = Pot 16 CC
  0, // #23 = Pot Assign EXIT 
  -1, // #24 = Button 1
  -1, // #25 = Button 2
  -1, // #26 = Button 3
  -1, // #27 = Button 4
  -1, // #28 = Button 5
  -1, // #29 = Button 6
  -1, // #30 = Button 7
  -1, // #31 = Button 8
  -1, // #32 = Button 9
  -1, // #33 = Button 10
  -1, // #34 = Button 11
  -1, // #35 = Button 12
  -1, // #36 = Button 13
  -1, // #37 = Button 14
  -1, // #38 = Button 15
  -1, // #39 = Button 16
  0, // #40 = Button Assign EXIT 
  0, // #41 = Kbd Driver
  1, // #42 = Velocity Min
  1, // #43 = Velocity MaxAdj
  1, // #44 = Velocity Slope
  12, // #45 = Upper Base
  12, // #46 = Lower Base
  12, // #47 = Pedal Base
  0, // #48 = Keyboard EXIT 
};

int8_t MenuValueMax[MENU_ITEMCOUNT] = {
  16, // #0 = Upper Channel
  16, // #1 = Lower Channel
  16, // #2 = Pedal Channel
  0, // #3 = Keyboard
  0, // #4 = Pot Assign
  0, // #5 = Button Assign
  0, // #6 = (end)
  127, // #7 = Pot 1 CC
  127, // #8 = Pot 2 CC
  127, // #9 = Pot 3 CC
  127, // #10 = Pot 4 CC
  127, // #11 = Pot 5 CC
  127, // #12 = Pot 6 CC
  127, // #13 = Pot 7 CC
  127, // #14 = Pot 8 CC
  127, // #15 = Pot 9 CC
  127, // #16 = Pot 10 CC
  127, // #17 = Pot 11 CC
  127, // #18 = Pot 12 CC
  127, // #19 = Pot 13 CC
  127, // #20 = Pot 14 CC
  127, // #21 = Pot 15 CC
  127, // #22 = Pot 16 CC
  0, // #23 = Pot Assign EXIT 
  127, // #24 = Button 1
  127, // #25 = Button 2
  127, // #26 = Button 3
  127, // #27 = Button 4
  127, // #28 = Button 5
  127, // #29 = Button 6
  127, // #30 = Button 7
  127, // #31 = Button 8
  127, // #32 = Button 9
  127, // #33 = Button 10
  127, // #34 = Button 11
  127, // #35 = Button 12
  127, // #36 = Button 13
  127, // #37 = Button 14
  127, // #38 = Button 15
  127, // #39 = Button 16
  0, // #40 = Button Assign EXIT 
  drv_custom, // #41 = Kbd Driver
  40, // #42 = Velocity Min
  40, // #43 = Velocity MaxAdj
  30, // #44 = Velocity Slope
  60, // #45 = Upper Base
  60, // #46 = Lower Base
  60, // #47 = Pedal Base
  0, // #48 = Keyboard EXIT 
};

int8_t MenuValues[MENU_ITEMCOUNT] = {
  MIDI_CH_UPR, // #0 = Upper Channel
  MIDI_CH_LWR, // #1 = Lower Channel
  MIDI_CH_PED, // #2 = Pedal Channel
  drv_fatar1, // #3 = Keyboard
  0, // #4 = Pot Assign
  0, // #5 = Button Assign
  0, // #6 = (end)
  7, // #7 = Pot 1 CC
  10, // #8 = Pot 2 CC
  11, // #9 = Pot 3 CC
  91, // #10 = Pot 4 CC
  -1, // #11 = Pot 5 CC
  -1, // #12 = Pot 6 CC
  -1, // #13 = Pot 7 CC
  -1, // #14 = Pot 8 CC
  -1, // #15 = Pot 9 CC
  -1, // #16 = Pot 10 CC
  -1, // #17 = Pot 11 CC
  -1, // #18 = Pot 12 CC
  -1, // #19 = Pot 13 CC
  -1, // #20 = Pot 14 CC
  -1, // #21 = Pot 15 CC
  -1, // #22 = Pot 16 CC
  0, // #23 = Pot Assign EXIT 
  20, // #24 = Button 1
  21, // #25 = Button 2
  22, // #26 = Button 3
  23, // #27 = Button 4
  24, // #28 = Button 5
  25, // #29 = Button 6
  26, // #30 = Button 7
  27, // #31 = Button 8
  28, // #32 = Button 9
  29, // #33 = Button 10
  30, // #34 = Button 11
  31, // #35 = Button 12
  32, // #36 = Button 13
  33, // #37 = Button 14
  34, // #38 = Button 15
  35, // #39 = Button 16
  0, // #40 = Button Assign EXIT 
  drv_fatar1, // #41 = Kbd Driver
  MIDI_MINDYN, // #42 = Velocity Min
  MIDI_MAXDYNADJ, // #43 = Velocity MaxAdj
  MIDI_DYNSLOPE, // #44 = Velocity Slope
  MIDI_BASE_UPR, // #45 = Upper Base
  MIDI_BASE_LWR, // #46 = Lower Base
  MIDI_BASE_PED, // #47 = Pedal Base
  0, // #48 = Keyboard EXIT 
};

const int8_t MenuLink[MENU_ITEMCOUNT] = {
  0, // #0 = Upper Channel
  0, // #1 = Lower Channel
  0, // #2 = Pedal Channel
  41, // #3 = Keyboard
  7, // #4 = Pot Assign
  24, // #5 = Button Assign
  0, // #6 = (end)
  0, // #7 = Pot 1 CC
  0, // #8 = Pot 2 CC
  0, // #9 = Pot 3 CC
  0, // #10 = Pot 4 CC
  0, // #11 = Pot 5 CC
  0, // #12 = Pot 6 CC
  0, // #13 = Pot 7 CC
  0, // #14 = Pot 8 CC
  0, // #15 = Pot 9 CC
  0, // #16 = Pot 10 CC
  0, // #17 = Pot 11 CC
  0, // #18 = Pot 12 CC
  0, // #19 = Pot 13 CC
  0, // #20 = Pot 14 CC
  0, // #21 = Pot 15 CC
  0, // #22 = Pot 16 CC
  -1, // #23 = Pot Assign EXIT 
  0, // #24 = Button 1
  0, // #25 = Button 2
  0, // #26 = Button 3
  0, // #27 = Button 4
  0, // #28 = Button 5
  0, // #29 = Button 6
  0, // #30 = Button 7
  0, // #31 = Button 8
  0, // #32 = Button 9
  0, // #33 = Button 10
  0, // #34 = Button 11
  0, // #35 = Button 12
  0, // #36 = Button 13
  0, // #37 = Button 14
  0, // #38 = Button 15
  0, // #39 = Button 16
  -1, // #40 = Button Assign EXIT 
  0, // #41 = Kbd Driver
  0, // #42 = Velocity Min
  0, // #43 = Velocity MaxAdj
  0, // #44 = Velocity Slope
  0, // #45 = Upper Base
  0, // #46 = Lower Base
  0, // #47 = Pedal Base
  -1, // #48 = Keyboard EXIT 
};

// ------------------------------------------------------------------------------
                                   
const String Msg[] = {"FCK TRMP", "FCK AFD"};
int8_t MenuStart;  
int8_t MenuEnd;
int8_t MenuItemActive;
int8_t MenuItemReturn;   // speichert bei Untermenüs die Rücksprungposition 

MenuPanel lcd(LCD_I2C_ADDR, 16, 2);

bool menuInit() {
  // Initialisiere Menü, setze Start- und Endpunkt und zeige ersten Menüpunkt an
  MenuStart = 0;  
  MenuEnd = m_end - 1;
  MenuItemActive = m_upper_channel;
  MenuItemReturn = m_upper_channel; // Initiale Rücksprungposition auf ersten Menüpunkt setzen
  Wire.beginTransmission(LCD_I2C_ADDR); // Display I2C-Adresse
  if (Wire.endTransmission(true) == 0) {
    // Display gefunden
    lcd.begin(16, 2);
    delay(500); // Warte auf Display-Start
    lcd.setCursor(0, 0);
    lcd.print("ScanCtrl 0.9");
    lcd.setCursor(0, 1);
    lcd.print("C.Meyer 2026");
    lcd.createChar(LCD_ARW_UP, lcd.arrowUp);
    lcd.createChar(LCD_ARW_DN, lcd.arrowDown);
    lcd.createChar(LCD_ARW_LT, lcd.arrowLeft);
    lcd.createChar(LCD_ARW_LT_GREY, lcd.arrowLeftGrey);
    lcd.createChar(LCD_ARW_UD, lcd.arrowUpDown);
    lcd.createChar(LCD_ARW_UD_GREY, lcd.arrowUpDownGrey);
    lcd.createChar(LCD_ARW_RT, lcd.arrowRight);
    lcd.createChar(LCD_ARW_RT_GREY, lcd.arrowRightGrey);
    return true;  
  } else {
    // Kein Display gefunden
    return false;
  }
}

// Menu-Handling für LCD mit I2C-Interface

void displayMenuValue(uint8_t itemIndex) {
  lcd.setCursor(0, 1);
  int8_t item_value = MenuValues[itemIndex];
  switch (itemIndex) {
    case m_driver_type:
      // Kopiert Menu Text aus PROGMEM ins RAM, da lcd.print() nicht direkt aus PROGMEM lesen kann
      lcd.printProgmem(&DriverTypes[item_value]);
      lcd.clearEOL(); // Lösche evtl. alte Zeichen
      lcd.setCursor(13, 1);
      break;
    default:
      lcd.print(item_value);
      lcd.clearEOL(); // Lösche evtl. alte Zeichen
      lcd.setCursor(3, 1);
      break;
  }
  lcd.write(LCD_ARW_LT);
  if (item_value != (int8_t)EEPROM.read(itemIndex + EEPROM_MENUDEFAULTS)) {
    lcd.setCursor(15, 1);
    lcd.write('*'); // geänderte Werte mit Stern markieren
  }
  if (item_value < 0) {
    lcd.setCursor(5, 1);
    lcd.print("(unused)"); // negative Werte sind unbenutzt, entsprechend kennzeichnen
  }
}

void displayMenuItem(uint8_t itemIndex) {
  lcd.setCursor(0, 0);
  // Kopiert MenuItem aus PROGMEM ins RAM, da lcd.print() nicht direkt aus PROGMEM lesen kann
  lcd.printProgmem(&MenuItems[itemIndex]);
  lcd.clearEOL(); // Lösche evtl. alte Zeichen
  
  lcd.setCursor(15, 0);
  lcd.write(LCD_ARW_UD);
  int8_t menu_link = MenuLink[MenuItemActive];
  if (menu_link < 0) {
    lcd.setCursor(0, 1);
    lcd.print("Exit ");
    lcd.write(LCD_ARW_LT); // Untermenü-Ende mit Pfeil nach links markieren
    lcd.clearEOL(); // Lösche evtl. alte Zeichen
  } else if (menu_link > 0) {
    lcd.setCursor(0, 1);
    lcd.print("Settings ");
    lcd.write(LCD_ARW_RT); // Untermenü mit Pfeil nach rechts markieren
    lcd.clearEOL(); // Lösche evtl. alte Zeichen
  } else {
    displayMenuValue(itemIndex);
  }
}


#endif