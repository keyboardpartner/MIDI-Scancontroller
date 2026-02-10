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

#define MENU_BTNMODECOUNT 4
enum {btnmode_send_cc_val, btnmode_send_cc_evt, btnmode_send_prg_ch, btnmode_send_note};
const lcdTextType ButtonModes[MENU_BTNMODECOUNT] PROGMEM = {
  { "Send CC Val" }, // Sendet CC entsprechend der Zuweisung, bei ON 127, bei OFF 0
  { "Send CC Evt" }, // Sendet immer CC mit 127 bei ON und bei OFF
  { "Send Prg Ch" }, // Sendet ein MIDI Program Change mit der Nummer aus MenuValues[m_btn1 + bnt_number]
  { "Send Note" },   // Sendet MIDI Note On mit Velocity 64 wenn gedrückt und Note Off wenn losgelassen, Note Nummer aus MenuValues[m_btn1 + bnt_number]
};

#define MENU_ITEMCOUNT 69

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
  m_pitchwheel_pot, // #3 = Pitchwheel Pot
  m_modulation_pot, // #4 = Modulation Pot
  m_kbd_subm, // #5 = Keyboard
  m_cc_subm, // #6 = Pot Assign
  m_btncc_subm, // #7 = Button Assign
  m_btnmode_subm, // #8 = Button Mode
  m_end, // #9 = (end)
  m_CC1, // #10 = Pot 1 CC
  m_CC2, // #11 = Pot 2 CC
  m_CC3, // #12 = Pot 3 CC
  m_CC4, // #13 = Pot 4 CC
  m_CC5, // #14 = Pot 5 CC
  m_CC6, // #15 = Pot 6 CC
  m_CC7, // #16 = Pot 7 CC
  m_CC8, // #17 = Pot 8 CC
  m_CC9, // #18 = Pot 9 CC
  m_CC10, // #19 = Pot 10 CC
  m_CC11, // #20 = Pot 11 CC
  m_CC12, // #21 = Pot 12 CC
  m_CC13, // #22 = Pot 13 CC
  m_CC14, // #23 = Pot 14 CC
  m_CC15, // #24 = Pot 15 CC
  m_CC16, // #25 = Pot 16 CC
  m_cc_back, // #26 = Pot Assign EXIT
  m_btn1, // #27 = Btn 1 CC/Prg
  m_btn2, // #28 = Btn 2 CC/Prg
  m_btn3, // #29 = Btn 3 CC/Prg
  m_btn4, // #30 = Btn 4 CC/Prg
  m_btn5, // #31 = Btn 5 CC/Prg
  m_btn6, // #32 = Btn 6 CC/Prg
  m_btn7, // #33 = Btn 7 CC/Prg
  m_btn8, // #34 = Btn 8 CC/Prg
  m_btn9, // #35 = Btn 9 CC/Prg
  m_btn10, // #36 = Btn 10 CC/Prg
  m_btn11, // #37 = Btn 11 CC/Prg
  m_btn12, // #38 = Btn 12 CC/Prg
  m_btn13, // #39 = Btn 13 CC/Prg
  m_btn14, // #40 = Btn 14 CC/Prg
  m_btn15, // #41 = Btn 15 CC/Prg
  m_btn16, // #42 = Btn 16 CC/Prg
  m_btn_back, // #43 = Button Assign EXIT
  m_btnmode1, // #44 = Btn 1 Mode
  m_btnmode2, // #45 = Btn 2 Mode
  m_btnmode3, // #46 = Btn 3 Mode
  m_btnmode4, // #47 = Btn 4 Mode
  m_btnmode5, // #48 = Btn 5 Mode
  m_btnmode6, // #49 = Btn 6 Mode
  m_btnmode7, // #50 = Btn 7 Mode
  m_btnmode8, // #51 = Btn 8 Mode
  m_btnmode9, // #52 = Btn 9 Mode
  m_btnmode10, // #53 = Btn 10 Mode
  m_btnmode11, // #54 = Btn 11 Mode
  m_btnmode12, // #55 = Btn 12 Mode
  m_btnmode13, // #56 = Btn 13 Mode
  m_btnmode14, // #57 = Btn 14 Mode
  m_btnmode15, // #58 = Btn 15 Mode
  m_btnmode16, // #59 = Btn 16 Mode
  m_btnmode_back, // #60 = Button Mode EXIT
  m_driver_type, // #61 = Kbd Driver
  m_mindyn, // #62 = Velocity Min
  m_maxdynadj, // #63 = Velocity MaxAdj
  m_slope, // #64 = Velocity Slope
  m_upper_base, // #65 = Upper Base
  m_lower_base, // #66 = Lower Base
  m_pedal_base, // #67 = Pedal Base
  m_kbd_back, // #68 = Keyboard EXIT
};

const lcdTextType MenuItems[MENU_ITEMCOUNT] PROGMEM = {
  { "Upper Channel" },  // #0
  { "Lower Channel" },  // #1
  { "Pedal Channel" },  // #2
  { "Pitchwheel Pot" },  // #3
  { "Modulation Pot" },  // #4
  { "Keyboard" },  // #5
  { "Pot Assign" },  // #6
  { "Button Assign" },  // #7
  { "Button Mode" },  // #8
  { "(end)" },  // #9
  { "Pot 1 CC" },  // #10
  { "Pot 2 CC" },  // #11
  { "Pot 3 CC" },  // #12
  { "Pot 4 CC" },  // #13
  { "Pot 5 CC" },  // #14
  { "Pot 6 CC" },  // #15
  { "Pot 7 CC" },  // #16
  { "Pot 8 CC" },  // #17
  { "Pot 9 CC" },  // #18
  { "Pot 10 CC" },  // #19
  { "Pot 11 CC" },  // #20
  { "Pot 12 CC" },  // #21
  { "Pot 13 CC" },  // #22
  { "Pot 14 CC" },  // #23
  { "Pot 15 CC" },  // #24
  { "Pot 16 CC" },  // #25
  { "Pot Assign" },  // #26  EXIT SUBM
  { "Btn 1 CC/Prg" },  // #27
  { "Btn 2 CC/Prg" },  // #28
  { "Btn 3 CC/Prg" },  // #29
  { "Btn 4 CC/Prg" },  // #30
  { "Btn 5 CC/Prg" },  // #31
  { "Btn 6 CC/Prg" },  // #32
  { "Btn 7 CC/Prg" },  // #33
  { "Btn 8 CC/Prg" },  // #34
  { "Btn 9 CC/Prg" },  // #35
  { "Btn 10 CC/Prg" },  // #36
  { "Btn 11 CC/Prg" },  // #37
  { "Btn 12 CC/Prg" },  // #38
  { "Btn 13 CC/Prg" },  // #39
  { "Btn 14 CC/Prg" },  // #40
  { "Btn 15 CC/Prg" },  // #41
  { "Btn 16 CC/Prg" },  // #42
  { "Button Assign" },  // #43  EXIT SUBM
  { "Btn 1 Mode" },  // #44
  { "Btn 2 Mode" },  // #45
  { "Btn 3 Mode" },  // #46
  { "Btn 4 Mode" },  // #47
  { "Btn 5 Mode" },  // #48
  { "Btn 6 Mode" },  // #49
  { "Btn 7 Mode" },  // #50
  { "Btn 8 Mode" },  // #51
  { "Btn 9 Mode" },  // #52
  { "Btn 10 Mode" },  // #53
  { "Btn 11 Mode" },  // #54
  { "Btn 12 Mode" },  // #55
  { "Btn 13 Mode" },  // #56
  { "Btn 14 Mode" },  // #57
  { "Btn 15 Mode" },  // #58
  { "Btn 16 Mode" },  // #59
  { "Button Mode" },  // #60  EXIT SUBM
  { "Kbd Driver" },  // #61
  { "Velocity Min" },  // #62
  { "Velocity MaxAdj" },  // #63
  { "Velocity Slope" },  // #64
  { "Upper Base" },  // #65
  { "Lower Base" },  // #66
  { "Pedal Base" },  // #67
  { "Keyboard" },  // #68  EXIT SUBM
};

const int8_t MenuValueMin[MENU_ITEMCOUNT] = {
  1, // #0 = Upper Channel
  1, // #1 = Lower Channel
  1, // #2 = Pedal Channel
  -1, // #3 = Pitchwheel Pot
  -1, // #4 = Modulation Pot
  0, // #5 = Keyboard
  0, // #6 = Pot Assign
  0, // #7 = Button Assign
  0, // #8 = Button Mode
  0, // #9 = (end)
  -1, // #10 = Pot 1 CC
  -1, // #11 = Pot 2 CC
  -1, // #12 = Pot 3 CC
  -1, // #13 = Pot 4 CC
  -1, // #14 = Pot 5 CC
  -1, // #15 = Pot 6 CC
  -1, // #16 = Pot 7 CC
  -1, // #17 = Pot 8 CC
  -1, // #18 = Pot 9 CC
  -1, // #19 = Pot 10 CC
  -1, // #20 = Pot 11 CC
  -1, // #21 = Pot 12 CC
  -1, // #22 = Pot 13 CC
  -1, // #23 = Pot 14 CC
  -1, // #24 = Pot 15 CC
  -1, // #25 = Pot 16 CC
  0, // #26 = Pot Assign EXIT
  -1, // #27 = Btn 1 CC/Prg
  -1, // #28 = Btn 2 CC/Prg
  -1, // #29 = Btn 3 CC/Prg
  -1, // #30 = Btn 4 CC/Prg
  -1, // #31 = Btn 5 CC/Prg
  -1, // #32 = Btn 6 CC/Prg
  -1, // #33 = Btn 7 CC/Prg
  -1, // #34 = Btn 8 CC/Prg
  -1, // #35 = Btn 9 CC/Prg
  -1, // #36 = Btn 10 CC/Prg
  -1, // #37 = Btn 11 CC/Prg
  -1, // #38 = Btn 12 CC/Prg
  -1, // #39 = Btn 13 CC/Prg
  -1, // #40 = Btn 14 CC/Prg
  -1, // #41 = Btn 15 CC/Prg
  -1, // #42 = Btn 16 CC/Prg
  0, // #43 = Button Assign EXIT
  0, // #44 = Btn 1 Mode
  0, // #45 = Btn 2 Mode
  0, // #46 = Btn 3 Mode
  0, // #47 = Btn 4 Mode
  0, // #48 = Btn 5 Mode
  0, // #49 = Btn 6 Mode
  0, // #50 = Btn 7 Mode
  0, // #51 = Btn 8 Mode
  0, // #52 = Btn 9 Mode
  0, // #53 = Btn 10 Mode
  0, // #54 = Btn 11 Mode
  0, // #55 = Btn 12 Mode
  0, // #56 = Btn 13 Mode
  0, // #57 = Btn 14 Mode
  0, // #58 = Btn 15 Mode
  0, // #59 = Btn 16 Mode
  0, // #60 = Button Mode EXIT
  0, // #61 = Kbd Driver
  1, // #62 = Velocity Min
  1, // #63 = Velocity MaxAdj
  1, // #64 = Velocity Slope
  12, // #65 = Upper Base
  12, // #66 = Lower Base
  12, // #67 = Pedal Base
  0, // #68 = Keyboard EXIT
};

const int8_t MenuValueMax[MENU_ITEMCOUNT] = {
  16, // #0 = Upper Channel
  16, // #1 = Lower Channel
  16, // #2 = Pedal Channel
  31, // #3 = Pitchwheel Pot
  31, // #4 = Modulation Pot
  0, // #5 = Keyboard
  0, // #6 = Pot Assign
  0, // #7 = Button Assign
  0, // #8 = Button Mode
  0, // #9 = (end)
  127, // #10 = Pot 1 CC
  127, // #11 = Pot 2 CC
  127, // #12 = Pot 3 CC
  127, // #13 = Pot 4 CC
  127, // #14 = Pot 5 CC
  127, // #15 = Pot 6 CC
  127, // #16 = Pot 7 CC
  127, // #17 = Pot 8 CC
  127, // #18 = Pot 9 CC
  127, // #19 = Pot 10 CC
  127, // #20 = Pot 11 CC
  127, // #21 = Pot 12 CC
  127, // #22 = Pot 13 CC
  127, // #23 = Pot 14 CC
  127, // #24 = Pot 15 CC
  127, // #25 = Pot 16 CC
  0, // #26 = Pot Assign EXIT
  127, // #27 = Btn 1 CC/Prg
  127, // #28 = Btn 2 CC/Prg
  127, // #29 = Btn 3 CC/Prg
  127, // #30 = Btn 4 CC/Prg
  127, // #31 = Btn 5 CC/Prg
  127, // #32 = Btn 6 CC/Prg
  127, // #33 = Btn 7 CC/Prg
  127, // #34 = Btn 8 CC/Prg
  127, // #35 = Btn 9 CC/Prg
  127, // #36 = Btn 10 CC/Prg
  127, // #37 = Btn 11 CC/Prg
  127, // #38 = Btn 12 CC/Prg
  127, // #39 = Btn 13 CC/Prg
  127, // #40 = Btn 14 CC/Prg
  127, // #41 = Btn 15 CC/Prg
  127, // #42 = Btn 16 CC/Prg
  0, // #43 = Button Assign EXIT
  3, // #44 = Btn 1 Mode
  3, // #45 = Btn 2 Mode
  3, // #46 = Btn 3 Mode
  3, // #47 = Btn 4 Mode
  3, // #48 = Btn 5 Mode
  3, // #49 = Btn 6 Mode
  3, // #50 = Btn 7 Mode
  3, // #51 = Btn 8 Mode
  3, // #52 = Btn 9 Mode
  3, // #53 = Btn 10 Mode
  3, // #54 = Btn 11 Mode
  3, // #55 = Btn 12 Mode
  3, // #56 = Btn 13 Mode
  3, // #57 = Btn 14 Mode
  3, // #58 = Btn 15 Mode
  3, // #59 = Btn 16 Mode
  0, // #60 = Button Mode EXIT
  drv_custom, // #61 = Kbd Driver
  40, // #62 = Velocity Min
  40, // #63 = Velocity MaxAdj
  30, // #64 = Velocity Slope
  60, // #65 = Upper Base
  60, // #66 = Lower Base
  60, // #67 = Pedal Base
  0, // #68 = Keyboard EXIT
};

int8_t MenuValues[MENU_ITEMCOUNT] = {
  MIDI_CH_UPR, // #0 = Upper Channel
  MIDI_CH_LWR, // #1 = Lower Channel
  MIDI_CH_PED, // #2 = Pedal Channel
  -1, // #3 = Pitchwheel Pot
  -1, // #4 = Modulation Pot
  0, // #5 = Keyboard
  0, // #6 = Pot Assign
  0, // #7 = Button Assign
  0, // #8 = Button Mode
  0, // #9 = (end)
  7, // #10 = Pot 1 CC
  10, // #11 = Pot 2 CC
  11, // #12 = Pot 3 CC
  91, // #13 = Pot 4 CC
  -1, // #14 = Pot 5 CC
  -1, // #15 = Pot 6 CC
  -1, // #16 = Pot 7 CC
  -1, // #17 = Pot 8 CC
  -1, // #18 = Pot 9 CC
  -1, // #19 = Pot 10 CC
  -1, // #20 = Pot 11 CC
  -1, // #21 = Pot 12 CC
  -1, // #22 = Pot 13 CC
  -1, // #23 = Pot 14 CC
  -1, // #24 = Pot 15 CC
  -1, // #25 = Pot 16 CC
  0, // #26 = Pot Assign EXIT
  20, // #27 = Btn 1 CC/Prg
  21, // #28 = Btn 2 CC/Prg
  22, // #29 = Btn 3 CC/Prg
  23, // #30 = Btn 4 CC/Prg
  24, // #31 = Btn 5 CC/Prg
  25, // #32 = Btn 6 CC/Prg
  26, // #33 = Btn 7 CC/Prg
  27, // #34 = Btn 8 CC/Prg
  28, // #35 = Btn 9 CC/Prg
  29, // #36 = Btn 10 CC/Prg
  30, // #37 = Btn 11 CC/Prg
  31, // #38 = Btn 12 CC/Prg
  32, // #39 = Btn 13 CC/Prg
  33, // #40 = Btn 14 CC/Prg
  34, // #41 = Btn 15 CC/Prg
  35, // #42 = Btn 16 CC/Prg
  0, // #43 = Button Assign EXIT
  0, // #44 = Btn 1 Mode
  0, // #45 = Btn 2 Mode
  0, // #46 = Btn 3 Mode
  0, // #47 = Btn 4 Mode
  0, // #48 = Btn 5 Mode
  0, // #49 = Btn 6 Mode
  0, // #50 = Btn 7 Mode
  0, // #51 = Btn 8 Mode
  0, // #52 = Btn 9 Mode
  0, // #53 = Btn 10 Mode
  0, // #54 = Btn 11 Mode
  0, // #55 = Btn 12 Mode
  0, // #56 = Btn 13 Mode
  0, // #57 = Btn 14 Mode
  0, // #58 = Btn 15 Mode
  0, // #59 = Btn 16 Mode
  0, // #60 = Button Mode EXIT
  drv_fatar1, // #61 = Kbd Driver
  MIDI_MINDYN, // #62 = Velocity Min
  MIDI_MAXDYNADJ, // #63 = Velocity MaxAdj
  MIDI_DYNSLOPE, // #64 = Velocity Slope
  MIDI_BASE_UPR, // #65 = Upper Base
  MIDI_BASE_LWR, // #66 = Lower Base
  MIDI_BASE_PED, // #67 = Pedal Base
  0, // #68 = Keyboard EXIT
};

const int8_t MenuLink[MENU_ITEMCOUNT] = {
  0, // #0 = Upper Channel
  0, // #1 = Lower Channel
  0, // #2 = Pedal Channel
  0, // #3 = Pitchwheel Pot
  0, // #4 = Modulation Pot
  61, // #5 = Keyboard
  8, // #6 = Pot Assign
  27, // #7 = Button Assign
  44, // #8 = Button Mode
  0, // #9 = (end)
  0, // #10 = Pot 1 CC
  0, // #11 = Pot 2 CC
  0, // #12 = Pot 3 CC
  0, // #13 = Pot 4 CC
  0, // #14 = Pot 5 CC
  0, // #15 = Pot 6 CC
  0, // #16 = Pot 7 CC
  0, // #17 = Pot 8 CC
  0, // #18 = Pot 9 CC
  0, // #19 = Pot 10 CC
  0, // #20 = Pot 11 CC
  0, // #21 = Pot 12 CC
  0, // #22 = Pot 13 CC
  0, // #23 = Pot 14 CC
  0, // #24 = Pot 15 CC
  0, // #25 = Pot 16 CC
  -1, // #26 = Pot Assign EXIT
  0, // #27 = Btn 1 CC/Prg
  0, // #28 = Btn 2 CC/Prg
  0, // #29 = Btn 3 CC/Prg
  0, // #30 = Btn 4 CC/Prg
  0, // #31 = Btn 5 CC/Prg
  0, // #32 = Btn 6 CC/Prg
  0, // #33 = Btn 7 CC/Prg
  0, // #34 = Btn 8 CC/Prg
  0, // #35 = Btn 9 CC/Prg
  0, // #36 = Btn 10 CC/Prg
  0, // #37 = Btn 11 CC/Prg
  0, // #38 = Btn 12 CC/Prg
  0, // #39 = Btn 13 CC/Prg
  0, // #40 = Btn 14 CC/Prg
  0, // #41 = Btn 15 CC/Prg
  0, // #42 = Btn 16 CC/Prg
  -1, // #43 = Button Assign EXIT
  0, // #44 = Btn 1 Mode
  0, // #45 = Btn 2 Mode
  0, // #46 = Btn 3 Mode
  0, // #47 = Btn 4 Mode
  0, // #48 = Btn 5 Mode
  0, // #49 = Btn 6 Mode
  0, // #50 = Btn 7 Mode
  0, // #51 = Btn 8 Mode
  0, // #52 = Btn 9 Mode
  0, // #53 = Btn 10 Mode
  0, // #54 = Btn 11 Mode
  0, // #55 = Btn 12 Mode
  0, // #56 = Btn 13 Mode
  0, // #57 = Btn 14 Mode
  0, // #58 = Btn 15 Mode
  0, // #59 = Btn 16 Mode
  -1, // #60 = Button Mode EXIT
  0, // #61 = Kbd Driver
  0, // #62 = Velocity Min
  0, // #63 = Velocity MaxAdj
  0, // #64 = Velocity Slope
  0, // #65 = Upper Base
  0, // #66 = Lower Base
  0, // #67 = Pedal Base
  -1, // #68 = Keyboard EXIT
};


// ------------------------------------------------------------------------------

const String Msg[] = {"FCK TRMP", "FCK AFD"};
int8_t MenuStart;
int8_t MenuEnd;
int8_t MenuItemActive;
int8_t MenuItemReturn;   // speichert bei Untermenüs die Rücksprungposition

MenuPanel lcd(LCD_I2C_ADDR, 16, 2);

bool menuInit() {
  // Initialisiere Menü, setze Start- und Endpunkt und zeige Version an
  if (lcd.begin(16, 2)) {
    MenuStart = 0;
    MenuEnd = m_end - 1;
    MenuItemActive = m_upper_channel;
    MenuItemReturn = m_upper_channel; // Initiale Rücksprungposition auf ersten Menüpunkt setzen
    // Display gefunden, zeige Startbild
    lcd.setCursor(0, 0);
    lcd.print(VERSION);
    lcd.setCursor(0, 1);
    lcd.print(F("C.Meyer 2026"));
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
    case m_btnmode1 ... m_btnmode16:
      // Kopiert Menu Text aus PROGMEM ins RAM, da lcd.print() nicht direkt aus PROGMEM lesen kann
      if (item_value < MENU_BTNMODECOUNT) {
        lcd.printProgmem(&ButtonModes[item_value]);
        lcd.clearEOL(); // Lösche evtl. alte Zeichen
        lcd.setCursor(13, 1);
      }
      break;
    case m_driver_type:
      // Kopiert Menu Text aus PROGMEM ins RAM, da lcd.print() nicht direkt aus PROGMEM lesen kann
      if (item_value < MENU_DRIVERCOUNT) {
        lcd.printProgmem(&DriverTypes[item_value]);
        lcd.clearEOL(); // Lösche evtl. alte Zeichen
        lcd.setCursor(13, 1);
      }
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
    lcd.print(F("(unused)")); // negative Werte sind unbenutzt, entsprechend kennzeichnen
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
    lcd.print(F("Exit "));
    lcd.write(LCD_ARW_LT); // Untermenü-Ende mit Pfeil nach links markieren
    lcd.clearEOL(); // Lösche evtl. alte Zeichen
  } else if (menu_link > 0) {
    lcd.setCursor(0, 1);
    lcd.print(F("Settings "));
    lcd.write(LCD_ARW_RT); // Untermenü mit Pfeil nach rechts markieren
    lcd.clearEOL(); // Lösche evtl. alte Zeichen
  } else {
    displayMenuValue(itemIndex);
  }
}


#endif