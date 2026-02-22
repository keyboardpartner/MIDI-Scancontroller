#ifndef MENU_SYSTEM_H
#define MENU_SYSTEM_H

// #############################################################################
//
//    ##     ## ######## ##    ## ##     ## 
//    ###   ### ##       ###   ## ##     ## 
//    #### #### ##       ####  ## ##     ## 
//    ## ### ## ######   ## ## ## ##     ## 
//    ##     ## ##       ##  #### ##     ## 
//    ##     ## ##       ##   ### ##     ## 
//    ##     ## ######## ##    ##  #######  
//
// #############################################################################

// Menu System für LCD mit I2C-Interface, basierend auf der Menu-Struktur aus Excel-Tabelle

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

#include <Wire.h>
#include <EEPROM.h>
#include "MenuPanel.h"
#include "global_vars.h"
#include "menu_items.h"



void callMenuAction() {
  // Hier wird entschieden, was bei einer Wertänderung eines Menüpunktes passieren soll
  // Bei Änderung von DBs oder Volume müssen die entsprechenden Werte an das FPGA oder MIDI gesendet werden
  if (currentMenuEntry.editAction != NULL) {
    currentMenuEntry.editAction();
  }
}

bool menuInit() {
  // Initialisiere Menü, setze Start- und Endpunkt und zeige Version an
  if (lcd.begin(16, 2)) {
    MenuItemActiveIdx = 0;
    MenuItemReturnIdx = 0; // Initiale Rücksprungposition auf ersten Menüpunkt setzen
    // Display gefunden, zeige Startbild
    lcd.setCursor(0, 0);
    lcd.print(F(VERSION));
    lcd.setCursor(0, 1);
    lcd.print(F(CREATOR));
    delay(1000);
    return true;
  } else {
    // Kein Display gefunden
    return false;
  }
}

// Menu-Handling für LCD mit I2C-Interface

void markEEPROMdifferent() {
  // Zeigt rechts unten einen Stern an, wenn der aktuelle Wert 
  // von dem in EEPROM gespeicherten Wert abweicht
  if (*currentMenuEntry.editValuePtr != (int8_t)EEPROM.read(MenuItemActiveIdx + EEPROM_MENUDEF_IDX)) {
    lcd.setCursor(15, 1);
    lcd.write('*'); // geänderte Werte mit Stern markieren
  }
}

void displayValueLine() {
  // Nur Wert, Pfeile und ggf. Änderungs-Stern anzeigen
  // Kann bei Menüeinträgen mit editAction verwendet werden, 
  // z.B. für Dynamikkurve
  lcd.setCursor(0, 1);
  int8_t item_value = *currentMenuEntry.editValuePtr;
  lcd.print(item_value);
  lcd.clearEOL(); // Lösche evtl. alte Zeichen
  markEEPROMdifferent();
  lcd.setCursor(3, 1);
  lcd.write(LCD_ARW_LT);
  if (item_value < 0) {
    lcd.setCursor(5, 1);
    lcd.print(F("(unused)")); // negative Werte sind unbenutzt, entsprechend kennzeichnen
  }
}


void displayMenuValue() {
  if (currentMenuEntry.editValuePtr == NULL) {  
    // kein Zeiger zum Ändern, sollte nicht passieren, aber sicherheitshalber prüfen
    lcd.setCursor(0, 1);
    lcd.print(F("(none)")); // negative Werte sind unbenutzt, entsprechend kennzeichnen
    lcd.clearEOL(); // Lösche evtl. alte Zeichen
    return;
  }
  if ((currentMenuEntry.editAction == NULL) ) {
    // Standard-Anzeige für Werte ohne spezielle editAction, z.B. MIDI-Kanal oder CC-Nummer
    displayValueLine();
  } else {
    // Spezielle Anzeige für Werte mit editAction, z.B. Treibername oder Dynamikkurve
    currentMenuEntry.editAction(); // muss auch die Anzeige der Werte aufrufen, z.B. Treibername oder Dynamikkurve
  } 
}

void displayMenuItem() {
  lcd.setCursor(0, 0);
  // Kopiert MenuItem aus PROGMEM ins RAM, da lcd.print() nicht direkt aus PROGMEM lesen kann
  lcd.print(currentMenuEntry.menuHeader);
  lcd.clearEOL(); // Lösche evtl. alte Zeichen

  lcd.setCursor(15, 0);
  lcd.write(LCD_ARW_UD);
  if (isSubMenu(MenuItemActiveIdx) && (currentMenuEntry.menuValueMax < 0)) {
    lcd.setCursor(0, 1);
    lcd.print(F("<EXIT>"));
    lcd.write(LCD_ARW_RT); // Untermenü-Ende mit Pfeil nach rechts markieren
    lcd.clearEOL(); // Lösche evtl. alte Zeichen
  } else if (currentMenuEntry.menuValueMax < 0) {
    lcd.setCursor(0, 1);
    lcd.print(F("<SETTINGS>"));
    lcd.write(LCD_ARW_RT); // Untermenü mit Pfeil nach rechts markieren
    lcd.clearEOL(); // Lösche evtl. alte Zeichen
  } else if (currentMenuEntry.menuValueMax > 0) {
    displayMenuValue();
  } else {
    // Kein Wert zu ändern, nur Bestätigung nötig
    lcd.setCursor(0, 1);
    lcd.print(F("<ENTER>"));
    lcd.clearEOL(); // Lösche evtl. alte Zeichen
  }
}

void handleMenuEncoderChange(int16_t encoderDelta) {
  // wird vom Callback der Encoderänderung aufgerufen
  // Menü-Handling bei Encoder-Änderungen: Wert ändern,
  if ((currentMenuEntry.menuValueMax <= 0) || (encoderDelta == 0)) 
    return; // kein Wert zu ändern

  // Encoder hat sich bewegt
  int16_t oldValue = *currentMenuEntry.editValuePtr;
  int16_t newValue = oldValue + encoderDelta; // Word, könnte sonst einen Überlauf geben
  int16_t minValue = (int16_t)currentMenuEntry.menuValueMin;
  int16_t maxValue = (int16_t)currentMenuEntry.menuValueMax;
  if (newValue < minValue) {
    newValue = minValue; // Unterlauf verhindern
  } else if (newValue > maxValue) {
    newValue = maxValue; // Maximalwert
  }
  if (currentMenuEntry.menuValueMax > 0) {
    // nur falls änderbar
    if (currentMenuEntry.editValuePtr == NULL) return; // kein Zeiger zum Ändern, sollte nicht passieren, aber sicherheitshalber prüfen
    *currentMenuEntry.editValuePtr = (int8_t)newValue;
    displayMenuValue(); // ruft auch editAction auf, falls vorhanden
  }
}

void menuEEPROMsave() {
  // speichert alle Menüwerte in EEPROM, z.B. vor einem Reset
  if (currentMenuEntry.editValuePtr != NULL) {
    EEPROM.update(MenuItemActiveIdx + EEPROM_MENUDEF_IDX, *currentMenuEntry.editValuePtr);
    displayMenuValue(); // aktualisiere Anzeige, um Stern zu entfernen
  }
}

void handleMenuButtons(int8_t buttons) {
  // Menü-Handling bei Button-Änderungen: Menupunkt wechseln oder Wert in EEPROM speichern
  if (buttons != 0) {
    if (buttons & LCD_BTNUP_MASK) {
      // Up-Taste mit Autorepeat
      uint16_t timeout = 750; // Startwert für getButtonsWaitReleased, wird nach erstem Durchlauf verkürzt für schnelleres Scrollen, wenn Taste gehalten wird
      do {
        if (isSubMenu(MenuItemActiveIdx)) {
          uint8_t submenu_start = findSubMenuStartIndex(currentMenuEntry.submenuLink);
          uint8_t submenu_end = findSubMenuEndIndex(currentMenuEntry.submenuLink);
          if (MenuItemActiveIdx > submenu_start) {
            MenuItemActiveIdx--;
            getMenuEntry(MenuItemActiveIdx); // Menüpunkt aus PROGMEM lesen, damit wir den Link für die Anzeige haben
          } else {
            MenuItemActiveIdx = submenu_end; // wrap around im Submenu
            getMenuEntry(MenuItemActiveIdx); // Menüpunkt aus PROGMEM lesen, damit wir den Link für die Anzeige haben
          }
        } else {
          if (MenuItemActiveIdx > 0) {
            MenuItemActiveIdx--;
            getMenuEntry(MenuItemActiveIdx); // Menüpunkt aus PROGMEM lesen, damit wir den Link für die Anzeige haben
          } else {
            MenuItemActiveIdx = m_main_end - 1; // wrap around
            getMenuEntry(MenuItemActiveIdx); // Menüpunkt aus PROGMEM lesen, damit wir den Link für die Anzeige haben
          }
        }
        displayMenuItem();
        buttons = lcd.waitReleased(timeout); // Warte bis losgelassen
        timeout = 250; // verkürze Wartezeit für schnelleres Scrollen, wenn Taste gehalten wird
      } while (buttons);
    }

    if (buttons & LCD_BTNDN_MASK) {
      // Down-Taste mit Autorepeat
      uint16_t timeout = 750; // Startwert für getButtonsWaitReleased, wird nach erstem Durchlauf verkürzt für schnelleres Scrollen, wenn Taste gehalten wird
      do {
        if (isSubMenu(MenuItemActiveIdx)) {
          uint8_t submenu_start = findSubMenuStartIndex(currentMenuEntry.submenuLink);
          uint8_t submenu_end = findSubMenuEndIndex(currentMenuEntry.submenuLink);
          if (MenuItemActiveIdx < submenu_end) {
            MenuItemActiveIdx++;
            getMenuEntry(MenuItemActiveIdx); // Menüpunkt aus PROGMEM lesen, damit wir den Link für die Anzeige haben
          } else {
            MenuItemActiveIdx = submenu_start; // wrap around im Submenu
            getMenuEntry(MenuItemActiveIdx); // Menüpunkt aus PROGMEM lesen, damit wir den Link für die Anzeige haben
          }
        } else {
          if (MenuItemActiveIdx < m_main_end - 1) {
            MenuItemActiveIdx++;
            getMenuEntry(MenuItemActiveIdx); // Menüpunkt aus PROGMEM lesen, damit wir den Link für die Anzeige haben
          } else {
            MenuItemActiveIdx = 0; // wrap around
            getMenuEntry(MenuItemActiveIdx); // Menüpunkt aus PROGMEM lesen, damit wir den Link für die Anzeige haben
          }
        }
        displayMenuItem();
        buttons = lcd.waitReleased(timeout); // Warte bis losgelassen
        timeout = 250; // verkürze Wartezeit für schnelleres Scrollen, wenn Taste gehalten wird
      } while (buttons);

    }

    if (buttons & LCD_BTNENTER_MASK) {
      // Enter-Taste, Wert in EEPROM speichern oder Submenu aufrufen
      if (isSubMenu(MenuItemActiveIdx)) {
        // Im Untermenü aufgerufen
        if (currentMenuEntry.menuValueMax < 0) {
          // Untermenü-Ende, Rücksprung zum Hauptmenü
          MenuItemActiveIdx = MenuItemReturnIdx; // Rücksprungposition wiederherstellen
          getMenuEntry(MenuItemActiveIdx); // Menüpunkt aus PROGMEM lesen, damit wir den Link für die Anzeige haben
          displayMenuItem();
        } else {
          // Wert in EEPROM speichern
          menuEEPROMsave();
        }
      } else if (currentMenuEntry.menuValueMax < 0) {
        // Im Hauptmenü, Submenu aufrufen
        MenuItemReturnIdx = MenuItemActiveIdx; // Rücksprungposition speichern
        MenuItemActiveIdx = findSubMenuStartIndex(currentMenuEntry.submenuLink); // zum Submenu springen
        getMenuEntry(MenuItemActiveIdx); // Menüpunkt aus PROGMEM lesen, damit wir den Link für die Anzeige haben
        displayMenuItem();
      } else {
        // Wert in EEPROM speichern
        menuEEPROMsave();
      } 
      lcd.waitReleased(0); // Warte bis losgelassen
    }
  }
}

#endif
