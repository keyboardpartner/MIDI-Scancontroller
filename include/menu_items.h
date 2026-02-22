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
#define MENU_ITEMCOUNT 58

int8_t MenuValues[MENU_ITEMCOUNT];

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

void setBtnMode();


// ------------------------------------------------------------------------------
// Hier Daten aus Excel-Tabelle "MIDI_menuItems.xlsx" einfügen
// ------------------------------------------------------------------------------
// <submenuLink> definiert die Menüstruktur:
// Innerhalb des Haupmenüs (submenuLink < m_main_end) 
// verlinken die Einträge auf Untermenüs (Suche nach gleichnamiger Gruppe).
// In den Untermenüs (submenuLink > m_main_end) zeigt subMenuLink die
// Zugehörigkeit zur jeweiligen Gruppe an.
// <editValuePtr> enthält Zeiger auf die Werte, die bei 
// Änderung eines Menüeintrags geändert werden sollen, 
// NULL wenn kein Wert geändert werden soll.
// <editAction> enthält Zeiger auf Routinen, die bei Änderung eines Menüeintrags
// ausgeführt werden sollen, z.B. um eine Textanzeige zu aktualisieren oder 
// um Werte zu berechnen.
// Ist <menuValueMax> < 0, kann im Hauptmenü ein Submenü aus <submenuLink> aufgerufen werden (<SETTINGS>) 
// oder, wenn bereits im Submenü, beendet werden (im Submenü <EXIT>)
// Ist <menuValueMax> > 0, wird der Wert als Integer zwischen min und max angezeigt und kann mit dem Encoder geändert werden.
// Ist <menuValueMax> = 0, wird kein Wert angezeigt, sondern nur auf Bestätigung <ENTER> gewartet.
// ------------------------------------------------------------------------------

// Alle vorhandenen Menü-Links und Gruppennamen müssen hier aufgelistet werden, 
// damit sie in der Tabelle <MenuItems> verwendet werden können:

enum {
  m_upper_ch,
  m_lower_ch,
  m_pedal_ch,
  m_swell,
  m_pitchwheel,
  m_kbd_driver,
  m_pot_assign,
  m_btn_assign,
  m_btn_mode,
  m_main_end,
};

// Action-Routine über Tabelle, ohne Parameter
typedef void (*action)();

// Bei mehr als 127 Menüpunkten müssen die Datentypen in menuEntryType angepasst werden
typedef struct {
  char menuHeader[16];
  int8_t submenuLink;
  int8_t* editValuePtr;
  action editAction; // function called when value changes
  int8_t menuValueMin;
  int8_t menuValueMax;
} menuEntryType;


// Diese Tabelle enthält die Menüstruktur, die in der Excel-Tabelle HX35_menuItems.xlsx definiert ist
// Menü-Text, Link zu Untermenüs, Zeiger auf Werte, die bei Änderung geändert werden sollen, 
// Action-Routine bei Änderung, Min- und Maximalwerte für die Editierung

const menuEntryType MenuItems[MENU_ITEMCOUNT] PROGMEM = { 
  {"Upper Channel", m_upper_ch, &MenuValues[0], NULL, 1, 16},
  {"Lower Channel", m_lower_ch, &MenuValues[1], NULL, 1, 16},
  {"Pedal Channel", m_pedal_ch, &MenuValues[2], NULL, 1, 16},
  {"Swell Ena/CC", m_swell, &MenuValues[3], NULL, -1, 127},
  {"Pitchwheel Pot", m_pitchwheel, &MenuValues[4], NULL, -1, 15},
  {"Keyboard", m_kbd_driver, NULL, NULL, -1, -1},
  {"Pot Assign", m_pot_assign, NULL, NULL, -1, -1},
  {"Button Assign", m_btn_assign, NULL, NULL, -1, -1},
  {"Button Mode", m_btn_mode, NULL, NULL, -1, -1},
  {"End", m_main_end, NULL, NULL, -1, -1},
  {"Pot 1 CC", m_pot_assign, &MenuValues[10], NULL, -1, 127},
  {"Pot 2 CC", m_pot_assign, &MenuValues[11], NULL, -1, 127},
  {"Pot 3 CC", m_pot_assign, &MenuValues[12], NULL, -1, 127},
  {"Pot 4 CC", m_pot_assign, &MenuValues[13], NULL, -1, 127},
  {"Pot CC", m_pot_assign, NULL, NULL, -1, -1},
  {"Btn 1 CC/Prg", m_btn_assign, &MenuValues[15], NULL, -1, 127},
  {"Btn 2 CC/Prg", m_btn_assign, &MenuValues[16], NULL, -1, 127},
  {"Btn 3 CC/Prg", m_btn_assign, &MenuValues[17], NULL, -1, 127},
  {"Btn 4 CC/Prg", m_btn_assign, &MenuValues[18], NULL, -1, 127},
  {"Btn 5 CC/Prg", m_btn_assign, &MenuValues[19], NULL, -1, 127},
  {"Btn 6 CC/Prg", m_btn_assign, &MenuValues[20], NULL, -1, 127},
  {"Btn 7 CC/Prg", m_btn_assign, &MenuValues[21], NULL, -1, 127},
  {"Btn 8 CC/Prg", m_btn_assign, &MenuValues[22], NULL, -1, 127},
  {"Btn 9 CC/Prg", m_btn_assign, &MenuValues[23], NULL, -1, 127},
  {"Btn 10 CC/Prg", m_btn_assign, &MenuValues[24], NULL, -1, 127},
  {"Btn 11 CC/Prg", m_btn_assign, &MenuValues[25], NULL, -1, 127},
  {"Btn 12 CC/Prg", m_btn_assign, &MenuValues[26], NULL, -1, 127},
  {"Btn 13 CC/Prg", m_btn_assign, &MenuValues[27], NULL, -1, 127},
  {"Btn 14 CC/Prg", m_btn_assign, &MenuValues[28], NULL, -1, 127},
  {"Btn 15 CC/Prg", m_btn_assign, &MenuValues[29], NULL, -1, 127},
  {"Btn 16 CC/Prg", m_btn_assign, &MenuValues[30], NULL, -1, 127},
  {"Btn CC", m_btn_assign, NULL, NULL, -1, -1},
  {"Btn 1 Mode", m_btn_mode, &MenuValues[32], &setBtnMode, 0, 3},
  {"Btn 2 Mode", m_btn_mode, &MenuValues[33], &setBtnMode, 0, 3},
  {"Btn 3 Mode", m_btn_mode, &MenuValues[34], &setBtnMode, 0, 3},
  {"Btn 4 Mode", m_btn_mode, &MenuValues[35], &setBtnMode, 0, 3},
  {"Btn 5 Mode", m_btn_mode, &MenuValues[36], &setBtnMode, 0, 3},
  {"Btn 6 Mode", m_btn_mode, &MenuValues[37], &setBtnMode, 0, 3},
  {"Btn 7 Mode", m_btn_mode, &MenuValues[38], &setBtnMode, 0, 3},
  {"Btn 8 Mode", m_btn_mode, &MenuValues[39], &setBtnMode, 0, 3},
  {"Btn 9 Mode", m_btn_mode, &MenuValues[40], &setBtnMode, 0, 3},
  {"Btn 10 Mode", m_btn_mode, &MenuValues[41], &setBtnMode, 0, 3},
  {"Btn 11 Mode", m_btn_mode, &MenuValues[42], &setBtnMode, 0, 3},
  {"Btn 12 Mode", m_btn_mode, &MenuValues[43], &setBtnMode, 0, 3},
  {"Btn 13 Mode", m_btn_mode, &MenuValues[44], &setBtnMode, 0, 3},
  {"Btn 14 Mode", m_btn_mode, &MenuValues[45], &setBtnMode, 0, 3},
  {"Btn 15 Mode", m_btn_mode, &MenuValues[46], &setBtnMode, 0, 3},
  {"Btn 16 Mode", m_btn_mode, &MenuValues[47], &setBtnMode, 0, 3},
  {"Btn Mode", m_btn_mode, NULL, NULL, -1, -1},
  {"Kbd Driver", m_kbd_driver, &MenuValues[49], &setKbdDriver, 0, 4},
  {"Velocity Min", m_kbd_driver, &MenuValues[50], &setDynSlope, 1, 30},
  {"Velocity MaxAdj", m_kbd_driver, &MenuValues[51], NULL, 0, 20},
  {"Velocity Slope", m_kbd_driver, &MenuValues[52], &setDynSlope, 1, 40},
  {"Upper Base", m_kbd_driver, &MenuValues[53], NULL, 12, 60},
  {"Lower Base", m_kbd_driver, &MenuValues[54], NULL, 12, 60},
  {"Pedal Base", m_kbd_driver, &MenuValues[55], NULL, 12, 60},
  {"Keyboard", m_kbd_driver, &MenuValues[56], NULL, -1, -1},
};
// Items with Action != NULL will need the editAction() to display lower LCD row value!
#define MENU_POT_CC 10
#define MENU_BTN_CC 15
#define MENU_BTN_MODE 32
#define MENU_KBD_DRIVER 49
#define MENU_MIN_DYN 50
#define MENU_MAX_DYNADJ 51
#define MENU_DYNSLOPE 52
#define MENU_BASE_UPR 53
#define MENU_BASE_LWR 54
#define MENU_BASE_PED 55

const int8_t MenuDefaults[MENU_ITEMCOUNT] = {
  1,  // Upper Channel
  2,  // Lower Channel
  3,  // Pedal Channel
  -1,  // Swell Ena/CC
  -1,  // Pitchwheel Pot
  -1,  // Keyboard
  -1,  // Pot Assign
  -1,  // Button Assign
  -1,  // Button Mode
  -1,  // End
  -1,  // Pot 1 CC
  -1,  // Pot 2 CC
  -1,  // Pot 3 CC
  -1,  // Pot 4 CC
  -1,  // Pot CC
  -1,  // Btn 1 CC/Prg
  -1,  // Btn 2 CC/Prg
  -1,  // Btn 3 CC/Prg
  -1,  // Btn 4 CC/Prg
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
  -1,  // Keyboard
};

// ------------------------------------------------------------------------------

const String Msg[] = {"FCK TRMP", "FCK AFD"};
int8_t MenuItemActiveIdx, SubmenuStartIdx, SubmenuEndIdx; // speichert die aktuelle Menüposition, Start- und Endindex des Untermenüs
int8_t MenuItemReturnIdx;   // speichert bei Untermenüs die Rücksprungposition


// ------------------------------------------------------------------------------


void initMenuValues() {
  // Initialisiere MenuValues mit Default-Werten, die in der Excel-Tabelle definiert sind
  for (uint8_t i = 0; i < MENU_ITEMCOUNT; i++) {
    MenuValues[i] = MenuDefaults[i]; // wenn Min-Wert definiert ist, nimm diesen als Default, sonst 0
  }
}

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
      SubmenuStartIdx = i; // speichere Startindex des Submenüs in globaler Variable, damit wir später prüfen können, ob wir uns im Submenü befinden
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
      SubmenuEndIdx = i; // speichere Endindex des Untermenüs in globaler Variable, damit wir später prüfen können, ob wir uns im Untermenü befinden
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


#endif