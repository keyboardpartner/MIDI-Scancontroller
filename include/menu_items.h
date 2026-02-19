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


#ifndef Menu_Items_h
#define Menu_Items_h

#include "global_vars.h"

// Menu System Variables

// Menu System Variables

#define MENU_DRIVERCOUNT 5

enum {drv_sr61, drv_fatar1, drv_fatar2, drv_pulse6105, drv_none};

const lcdTextType DriverTypes[MENU_DRIVERCOUNT] PROGMEM = {
  { "Scan16/61" },
  { "FatarScan1-61" },
  { "FatarScan2" },
  { "Pulse 6105WF" },
  { "None" },
};

#define MENU_BTNMODECOUNT 4
enum {btnmode_send_cc_val, btnmode_send_cc_evt, btnmode_send_prg_ch, btnmode_send_note};
const lcdTextType ButtonModes[MENU_BTNMODECOUNT] PROGMEM = {
  { "Send CC Val" }, // Sendet CC entsprechend der Zuweisung, bei ON 127, bei OFF 0
  { "Send CC Evt" }, // Sendet immer CC mit 127 bei ON und bei OFF
  { "Send Prg Ch" }, // Sendet ein MIDI Program Change mit der Nummer aus MenuValues[m_btn1 + bnt_number]
  { "Send Note" },   // Sendet MIDI Note On mit Velocity 64 wenn gedrückt und Note Off wenn losgelassen, Note Nummer aus MenuValues[m_btn1 + bnt_number]
};

void setKbdDriver();

void setDynSlope();


// ------------------------------------------------------------------------------
// Hier Daten aus Excel-Tabelle einfügen, die die Menüstruktur definiert.
// Es müssen 1 enum-Liste und 5 Arrays mit gleicher Länge angelegt werden.
// MenuLink[MENU_ITEMCOUNT] definiert die Menüstruktur:
// 0 normaler Edit-Menüpunkt, der mit Encoder geändert werden kann
// >0 ist die Nummer des Submenüpunktes, zu dem verlinkt wird
// -1 Rücksprungmöglichkeit (Exit) zum Hauptmenü
//
// EditValuePtrs[MENU_ITEMCOUNT] enthält Zeiger auf die Werte, die bei 
// Änderung eines Menüeintrags geändert werden sollen, 
// NULL wenn kein Wert geändert werden soll.
// ------------------------------------------------------------------------------

enum {
  m_upper_ch,
  m_lower_ch,
  m_pedal_ch,
  m_pitchwheel,
  m_mod_pot,
  m_kbd_driver,
  m_pot_assign,
  m_btn_assign,
  m_btn_mode,
  m_main_end,
}; // alle vorhandenen Menü-Links


// Action-Routine über Tabelle
typedef void (*action)();

// Bei mehr als 127 Menüpunkten müssen die Datentypen in menuEntryType angepasst werden
typedef struct {
  char menuHeader[16];
  int8_t submenuLink;
  uint8_t* editValuePtr;
  action editAction;
  int8_t menuValueMin;
  int8_t menuValueMax;
} menuEntryType;

#define MENU_ITEMCOUNT 69

// Diese Tabelle enthält die Menüstruktur, die in der Excel-Tabelle HX35_menuItems.xlsx definiert ist
// Menü-Text, Link zu Untermenüs, Zeiger auf Werte, die bei Änderung geändert werden sollen, 
// Action-Routine bei Änderung, Min- und Maximalwerte für die Editierung

const menuEntryType MenuItems[MENU_ITEMCOUNT] PROGMEM = { 
  {"Upper Channel", m_upper_ch, &MenuValues[0], NULL, 1, 16},
  {"Lower Channel", m_lower_ch, &MenuValues[1], NULL, 1, 16},
  {"Pedal Channel", m_pedal_ch, &MenuValues[2], NULL, 1, 16},
  {"Pitchwheel Pot", m_pitchwheel, &MenuValues[3], NULL, -1, 127},
  {"Modulation Pot", m_mod_pot, &MenuValues[4], NULL, -1, 127},
  {"Keyboard", m_kbd_driver, NULL, NULL, -1, -1},
  {"Pot Assign", m_pot_assign, NULL, NULL, -1, -1},
  {"Button Assign", m_btn_assign, NULL, NULL, -1, -1},
  {"Button Mode", m_btn_mode, NULL, NULL, -1, -1},
  {"End", m_main_end, NULL, NULL, -1, -1},
  {"Pot 1 CC", m_pot_assign, &MenuValues[10], NULL, -1, 127},
  {"Pot 2 CC", m_pot_assign, &MenuValues[11], NULL, -1, 127},
  {"Pot 3 CC", m_pot_assign, &MenuValues[12], NULL, -1, 127},
  {"Pot 4 CC", m_pot_assign, &MenuValues[13], NULL, -1, 127},
  {"Pot 5 CC", m_pot_assign, &MenuValues[14], NULL, -1, 127},
  {"Pot 6 CC", m_pot_assign, &MenuValues[15], NULL, -1, 127},
  {"Pot 7 CC", m_pot_assign, &MenuValues[16], NULL, -1, 127},
  {"Pot 8 CC", m_pot_assign, &MenuValues[17], NULL, -1, 127},
  {"Pot 9 CC", m_pot_assign, &MenuValues[18], NULL, -1, 127},
  {"Pot 10 CC", m_pot_assign, &MenuValues[19], NULL, -1, 127},
  {"Pot 11 CC", m_pot_assign, &MenuValues[20], NULL, -1, 127},
  {"Pot 12 CC", m_pot_assign, &MenuValues[21], NULL, -1, 127},
  {"Pot 13 CC", m_pot_assign, &MenuValues[22], NULL, -1, 127},
  {"Pot 14 CC", m_pot_assign, &MenuValues[23], NULL, -1, 127},
  {"Pot 15 CC", m_pot_assign, &MenuValues[24], NULL, -1, 127},
  {"Pot 16 CC", m_pot_assign, &MenuValues[25], NULL, -1, 127},
  {"Pot CC", m_pot_assign, NULL, NULL, -1, -1},
  {"Btn 1 CC/Prg", m_btn_assign, &MenuValues[27], NULL, -1, 127},
  {"Btn 2 CC/Prg", m_btn_assign, &MenuValues[28], NULL, -1, 127},
  {"Btn 3 CC/Prg", m_btn_assign, &MenuValues[29], NULL, -1, 127},
  {"Btn 4 CC/Prg", m_btn_assign, &MenuValues[30], NULL, -1, 127},
  {"Btn 5 CC/Prg", m_btn_assign, &MenuValues[31], NULL, -1, 127},
  {"Btn 6 CC/Prg", m_btn_assign, &MenuValues[32], NULL, -1, 127},
  {"Btn 7 CC/Prg", m_btn_assign, &MenuValues[33], NULL, -1, 127},
  {"Btn 8 CC/Prg", m_btn_assign, &MenuValues[34], NULL, -1, 127},
  {"Btn 9 CC/Prg", m_btn_assign, &MenuValues[35], NULL, -1, 127},
  {"Btn 10 CC/Prg", m_btn_assign, &MenuValues[36], NULL, -1, 127},
  {"Btn 11 CC/Prg", m_btn_assign, &MenuValues[37], NULL, -1, 127},
  {"Btn 12 CC/Prg", m_btn_assign, &MenuValues[38], NULL, -1, 127},
  {"Btn 13 CC/Prg", m_btn_assign, &MenuValues[39], NULL, -1, 127},
  {"Btn 14 CC/Prg", m_btn_assign, &MenuValues[40], NULL, -1, 127},
  {"Btn 15 CC/Prg", m_btn_assign, &MenuValues[41], NULL, -1, 127},
  {"Btn 16 CC/Prg", m_btn_assign, &MenuValues[42], NULL, -1, 127},
  {"Btn CC", m_btn_assign, NULL, NULL, -1, -1},
  {"Btn 1 Mode", m_btn_mode, &MenuValues[44], NULL, -1, 127},
  {"Btn 2 Mode", m_btn_mode, &MenuValues[45], NULL, -1, 127},
  {"Btn 3 Mode", m_btn_mode, &MenuValues[46], NULL, -1, 127},
  {"Btn 4 Mode", m_btn_mode, &MenuValues[47], NULL, -1, 127},
  {"Btn 5 Mode", m_btn_mode, &MenuValues[48], NULL, -1, 127},
  {"Btn 6 Mode", m_btn_mode, &MenuValues[49], NULL, -1, 127},
  {"Btn 7 Mode", m_btn_mode, &MenuValues[50], NULL, -1, 127},
  {"Btn 8 Mode", m_btn_mode, &MenuValues[51], NULL, -1, 127},
  {"Btn 9 Mode", m_btn_mode, &MenuValues[52], NULL, -1, 127},
  {"Btn 10 Mode", m_btn_mode, &MenuValues[53], NULL, -1, 127},
  {"Btn 11 Mode", m_btn_mode, &MenuValues[54], NULL, -1, 127},
  {"Btn 12 Mode", m_btn_mode, &MenuValues[55], NULL, -1, 127},
  {"Btn 13 Mode", m_btn_mode, &MenuValues[56], NULL, -1, 127},
  {"Btn 14 Mode", m_btn_mode, &MenuValues[57], NULL, -1, 127},
  {"Btn 15 Mode", m_btn_mode, &MenuValues[58], NULL, -1, 127},
  {"Btn 16 Mode", m_btn_mode, &MenuValues[59], NULL, -1, 127},
  {"Btn Mode", m_btn_mode, NULL, NULL, -1, -1},
  {"Kbd Driver", m_kbd_driver, &MenuValues[61], &setKbdDriver, 0, 5},
  {"Velocity Min", m_kbd_driver, &MenuValues[62], NULL, 1, 30},
  {"Velocity MaxAdj", m_kbd_driver, &MenuValues[63], NULL, 0, 30},
  {"Velocity Slope", m_kbd_driver, &MenuValues[64], NULL, 5, 30},
  {"Upper Base", m_kbd_driver, &MenuValues[65], NULL, 12, 48},
  {"Lower Base", m_kbd_driver, &MenuValues[66], NULL, 12, 48},
  {"Pedal Base", m_kbd_driver, &MenuValues[67], NULL, 12, 48},
  {"Kbd Driver", m_kbd_driver, &MenuValues[68], NULL, -1, -1},
};

#define MENU_POT_CC 10
#define MENU_BTN_CC 27
#define MENU_BTN_MODE 44
#define MENU_KBD_DRIVER 61
#define MENU_MIN_DYN 62
#define MENU_MAX_DYNADJ 63
#define MENU_DYNSLOOPE 64
#define MENU_BASE_UPR 65
#define MENU_BASE_LWR 66
#define MENU_BASE_PED 67

const int8_t MenuDefaults[MENU_ITEMCOUNT] = {
  1,  // Upper Channel
  2,  // Lower Channel
  3,  // Pedal Channel
  -1,  // Pitchwheel Pot
  -1,  // Modulation Pot
  -1,  // Keyboard
  -1,  // Pot Assign
  -1,  // Button Assign
  -1,  // Button Mode
  -1,  // End
  12,  // Pot 1 CC
  13,  // Pot 2 CC
  14,  // Pot 3 CC
  15,  // Pot 4 CC
  -1,  // Pot 5 CC
  -1,  // Pot 6 CC
  -1,  // Pot 7 CC
  -1,  // Pot 8 CC
  -1,  // Pot 9 CC
  -1,  // Pot 10 CC
  -1,  // Pot 11 CC
  -1,  // Pot 12 CC
  -1,  // Pot 13 CC
  -1,  // Pot 14 CC
  -1,  // Pot 15 CC
  -1,  // Pot 16 CC
  -1,  // Pot CC
  40,  // Btn 1 CC/Prg
  41,  // Btn 2 CC/Prg
  42,  // Btn 3 CC/Prg
  43,  // Btn 4 CC/Prg
  -1,  // Btn 5 CC/Prg
  -1,  // Btn 6 CC/Prg
  -1,  // Btn 7 CC/Prg
  -1,  // Btn 8 CC/Prg
  -1,  // Btn 9 CC/Prg
  -1,  // Btn 10 CC/Prg
  -1,  // Btn 11 CC/Prg
  -1,  // Btn 12 CC/Prg
  -1,  // Btn 13 CC/Prg
  -1,  // Btn 14 CC/Prg
  -1,  // Btn 15 CC/Prg
  -1,  // Btn 16 CC/Prg
  -1,  // Btn CC
  0,  // Btn 1 Mode
  0,  // Btn 2 Mode
  0,  // Btn 3 Mode
  0,  // Btn 4 Mode
  0,  // Btn 5 Mode
  0,  // Btn 6 Mode
  0,  // Btn 7 Mode
  0,  // Btn 8 Mode
  0,  // Btn 9 Mode
  0,  // Btn 10 Mode
  0,  // Btn 11 Mode
  0,  // Btn 12 Mode
  0,  // Btn 13 Mode
  0,  // Btn 14 Mode
  0,  // Btn 15 Mode
  0,  // Btn 16 Mode
  -1,  // Btn Mode
  1,  // Kbd Driver
  10,  // Velocity Min
  10,  // Velocity MaxAdj
  20,  // Velocity Slope
  36,  // Upper Base
  36,  // Lower Base
  36,  // Pedal Base
  -1,  // Kbd Driver
};


// ------------------------------------------------------------------------------

menuEntryType currentMenuEntry; // extrahierter Menüpunkt

void getMenuEntry(uint8_t index) {
  // einen Menüpunkt aus PROGMEM lesen und lokal in currentMenuEntry speichern, 
  // damit wir die Werte daraus verwenden können
  if (index >= MENU_ITEMCOUNT) return;
  memcpy_P(&currentMenuEntry, &MenuItems[index], sizeof(menuEntryType));
}

uint8_t findSubMenuStartIndex(int8_t submenuLink) {
  // Hilfsfunktion, um den Startindex eines Untermenüs zu finden
  for (uint8_t i = m_main_end; i < MENU_ITEMCOUNT; i++) {
    getMenuEntry(i);
    if (currentMenuEntry.submenuLink == submenuLink) {
      return i; // In currentMenuEntry ist jetzt der gefundene Menüpunkt, wir können den Index zurückgeben
    }
  }
  return 0; // nicht gefunden, Rückfall auf Hauptmenü
}

uint8_t findSubMenuEndIndex(int8_t submenuLink) {
  // Hilfsfunktion, um den Endindex eines Untermenüs zu finden
  for (int8_t i = MENU_ITEMCOUNT-1; i > m_main_end; i--) {
    getMenuEntry(i);
    if (currentMenuEntry.submenuLink == submenuLink) {
      return i; // In currentMenuEntry ist jetzt der gefundene Menüpunkt, wir können den Index zurückgeben 
    }
  }
  return 0; // nicht gefunden, Rückfall auf Hauptmenü
}

bool isSubMenu(int8_t menuIdx) {
  // Hilfsfunktion, um zu prüfen, ob es ein Untermenü mit diesem Link gibt
  if (menuIdx > m_main_end) return true;
  return false; // nicht gefunden
}

// ------------------------------------------------------------------------------

const String Msg[] = {"FCK TRMP", "FCK AFD"};
int8_t MenuItemActive;
int8_t MenuItemReturn;   // speichert bei Untermenüs die Rücksprungposition



#endif