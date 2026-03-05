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
// MIDI Controller for FATAR, SR 4014 and PULSE 6105WF keybeds, C. Meyer 1/2026
// Banner Logos from
// https://patorjk.com/software/taag/#p=display&f=Banner&t=MAIN&x=cppComment&v=4&h=2&w=80&we=false
// 20 MHz Bootloaders:
// https://github.com/MCUdude/MiniCore/tree/master/avr/bootloaders/optiboot_flash/bootloaders
//

// Define used modules here, comment out unused modules to save program memory

#define LCD_I2C
//#define ANLG_MPX  // Für MPX-gestützte analoge Eingänge und Schweller
//#define PANEL16

#include <Arduino.h>
#include <EEPROM.h>
#include <TimerOne.h>
#include "midi_io.h"
#include "global_vars.h"

uint8_t UpperKeyState[KEY_ARR_SIZE]; // Zustand der Tasten
uint8_t LowerKeyState[KEY_ARR_SIZE]; // Zustand der Tasten
uint8_t CommonKeyState[KEY_ARR_SIZE]; // Zustand der Tasten Upper und Lower

uint8_t PedalContactState[PEDAL_ARR_SIZE]; // Zustand der Pedaltasten
uint8_t AnyKeyPressed = false;

uint8_t UpperKeyTimer[KEY_ARR_SIZE]; // Timer für jede Taste
uint8_t LowerKeyTimer[KEY_ARR_SIZE]; // Timer für jede Taste

uint8_t UpperPresetState[PRESET_ARR_SIZE]; // Zustand für jede Taste
uint8_t LowerPresetState[PRESET_ARR_SIZE]; // Zustand für jede Taste

volatile uint8_t Timer1Semaphore = 0;
volatile uint8_t Timer1RoundRobin = 0;

bool lcdPresent = false;


#ifdef PANEL16
  // Für LCD mit I2C-Interface
  #include "Panel16.h"
  #define PANEL16_I2C_ADDR 0x62
  Panel16 panel16(PANEL16_I2C_ADDR);
#endif
bool panel16Present = false;

#ifdef ANLG_MPX
  // Für MPX-gestützte analoge Eingänge und Schweller
  #define ANLG_INPUTS 4 // Analoge Eingänge für MIDI-CC-Potentiometer
  #include "MpxPots.h"
  MPXpots mpxPots(ANLG_INPUTS, MPX_ACTIVE_TIMEOUT, MPX_INTEGRATOR_FACTOR);
#endif

#ifdef LCD_I2C
  // Für MenuPanel mit LCD I2C-Interface
  #include "menu_system.h"
#else
  // Ohne LCD, z.B. für reinen MIDI-Controller ohne Display
  #include "menu_items.h"
#endif

// #############################################################################
//
//      #####  #######    #    ####### #######  #####
//     #     #    #      # #      #    #       #     #
//     #          #     #   #     #    #       #
//      #####     #    #     #    #    #####    #####
//           #    #    #######    #    #             #
//     #     #    #    #     #    #    #       #     #
//      #####     #    #     #    #    #######  #####
//
// #############################################################################

// State Machines für Upper und Lower bei FATARSCAN_NEW und FATARSCAN_OLD
enum {t_idle, t_forward, t_pressed};

void UpperCheckstate(uint8_t scankey, uint8_t mk, uint8_t br) {
  switch (UpperKeyState[scankey]) {
  case t_idle:
    if (br || mk) {
      // Taste wurde gerade gedrückt
      UpperKeyTimer[scankey] = TIMER_MAX; // Timer auf Startwert
      UpperKeyState[scankey] = t_forward;
    }
    break;
  case t_forward:
    // Taste ist nicht mehr in Ruhestellung, Timer dekrementieren
    if (mk) {
      // Make-Kontakt geschlossen, Taste voll gedrückt
      // MIDI senden und Zustand auf t_pressed setzen
      UpperKeyState[scankey] = t_pressed;
      uint16_t tval = UpperKeyTimer[scankey] + MenuValues[MENU_MAX_DYNADJ];
      if (tval > 255) tval = 255;
      uint8_t mididyn = TimeToDyn[tval];
      MidiSendNoteOn(MenuValues[m_upper_ch], MenuValues[MENU_BASE_UPR] + scankey, mididyn);  // sende MIDI NoteOn mit Dynamikwert
    } else if (br) {
      // Break-Kontakt noch geschlossen, Timer dekrementieren
      uint8_t tval = UpperKeyTimer[scankey];
      if (tval > 0) {
        tval--;
        UpperKeyTimer[scankey] = tval;
      }
    } else {
      // Break-Kontakt offen, Taste wieder losgelassen
      UpperKeyState[scankey] = t_idle;
      MidiSendNoteOff(MenuValues[m_upper_ch], MenuValues[MENU_BASE_LWR] + scankey);// sende MIDI NoteOFF
    }
    break;
  case t_pressed:
    if (!(mk || br)) {
      // Make- und Break-Kontakt offen, Taste losgelassen
      UpperKeyState[scankey] = t_idle;
      MidiSendNoteOff(MenuValues[m_upper_ch], MenuValues[MENU_BASE_LWR] + scankey);// sende MIDI NoteOFF
    };
    break;
  }
}


// #############################################################################

void LowerCheckstate(uint8_t scankey, uint8_t mk, uint8_t br) {
  switch (LowerKeyState[scankey]) {
  case t_idle:
    if (br || mk) {
      // Taste wurde gerade gedrückt
      LowerKeyTimer[scankey] = TIMER_MAX; // Timer auf Startwert
      LowerKeyState[scankey] = t_forward;
    }
    break;
  case t_forward:
    // Taste ist nicht mehr in Ruhestellung, Timer dekrementieren
    if (mk) {
      // Make-Kontakt geschlossen, Taste voll gedrückt
      // MIDI senden und Zustand auf t_pressed setzen
      LowerKeyState[scankey] = t_pressed;
      uint16_t tval = LowerKeyTimer[scankey] + MenuValues[MENU_MAX_DYNADJ];
      if (tval > 255) tval = 255;
      uint8_t mididyn = TimeToDyn[tval];
      MidiSendNoteOn(MenuValues[m_lower_ch], MenuValues[MENU_BASE_LWR] + scankey, mididyn);  // sende MIDI NoteOn mit Dynamikwert
    } else if (br) {
      // Break-Kontakt noch geschlossen, Timer dekrementieren
      uint8_t tval = LowerKeyTimer[scankey];
      if (tval > 0) {
        tval--;
        LowerKeyTimer[scankey] = tval;
      }
    } else {
      // Break-Kontakt offen, Taste wieder losgelassen
      LowerKeyState[scankey] = t_idle;
      MidiSendNoteOff(MenuValues[m_lower_ch], MenuValues[MENU_BASE_LWR] + scankey);// sende MIDI NoteOFF
    }
    break;
  case t_pressed:
    if (!(mk || br)) {
      // Make- und Break-Kontakt offen, Taste losgelassen
      LowerKeyState[scankey] = t_idle;
      MidiSendNoteOff(MenuValues[m_lower_ch], MenuValues[MENU_BASE_LWR] + scankey);// sende MIDI NoteOFF
    };
    break;
  }
}


// #############################################################################
//
//     ######  ####### ######     #    #
//     #     # #       #     #   # #   #
//     #     # #       #     #  #   #  #
//     ######  #####   #     # #     # #
//     #       #       #     # ####### #
//     #       #       #     # #     # #
//     #       ####### ######  #     # #######
//
// #############################################################################

void ScanPedal() {
  // Pedal mit BASS25 scannen, Zeit für 25 Pedaltasten etwa 25 us bei 20 MHz Takt
  uint8_t scankey, mk_ped, mk_old; // aktuelle Taste
  _SET_SR_LOAD;
  _NOP_DLY;
  _NOP_DLY;
  _SET_SR_CLK;
  _NOP_DLY;
  _NOP_DLY;
  _CLR_SR_CLK;
  _CLR_SR_LOAD; // Load LOW
  for (scankey = 0; scankey < scanParams.pedalKeys; scankey++) {
    mk_ped = PINB & (1 << SR_PED); // Make-Kontakt Pedal lesen, active LOW
    // Pedal hat nur Make-Kontakt, deshalb keine State Machine
    mk_old = PedalContactState[scankey];
    if (mk_ped != mk_old) {
      // Zustand hat sich geändert
      PedalContactState[scankey] = mk_ped;
      if (mk_ped == 0) {
        // Pedal gedrückt, NoteOn mit fester Dynamik
        MidiSendNoteOnNoDyn(MenuValues[m_pedal_ch], MenuValues[MENU_BASE_PED] + scankey);
        ledTimerStart(20); // Board-LED für 20 Timer-Zyklen einschalten
      } else {
        // Pedal losgelassen
        MidiSendNoteOff(MenuValues[m_pedal_ch], MenuValues[MENU_BASE_PED] + scankey);
      }
    }
    _SET_SR_CLK;
    _NOP_DLY;
    _NOP_DLY;
    _CLR_SR_CLK;
  }
}

// #############################################################################
//
//     #     #    #    #     # #     #    #    #        #####
//     ##   ##   # #   ##    # #     #   # #   #       #     #
//     # # # #  #   #  # #   # #     #  #   #  #       #
//     #  #  # #     # #  #  # #     # #     # #        #####
//     #     # ####### #   # # #     # ####### #             #
//     #     # #     # #    ## #     # #     # #       #     #
//     #     # #     # #     #  #####  #     # #######  #####
//
// #############################################################################

void ScanManualsFatar1() {
  // Upper und Lower scannen, auch für PULSE 6105WF mit gleicher Pinbelegung wie FATARSCAN,
  // aber mit prePulses und anderem scankey-Startwert
  // Zeitbedarf für Abfrage beider 61er Manuale: 
  // FatarScan1-73: etwa 220µs bei 20 MHz Takt, 270µs bei 16 MHz
  AnyKeyPressed = false;
  _SET_TEST; // Test Pin für Debugging, z.B. mit Oszilloskop
  delayMicroseconds(1);
  _CLR_TEST;

  uint8_t portd_idle = (PORTD & ~(1 << FT_CLK)) | (1 << FT_LOAD); // Set bit 3 LOW, 4 HIGH for idle
  PORTD = portd_idle;  // zurück zu idle
  uint8_t first_key = scanParams.groupOffset * scanParams.keysPerGroup + scanParams.keyOffset; // erste spielbare Taste, nach Preset-Tasten
  uint8_t first_playable_key = first_key + scanParams.presetKeys; // erste spielbare Taste, nach Preset-Tasten
  for (uint8_t tdrive = 0; tdrive < scanParams.keysPerGroup; tdrive++) {
    PORTB = (PORTB & B11111000) | (tdrive ^ scanParams.invertMask); // nur unterste 3 Bits, als T-Drive an Decoder
    delayMicroseconds(2); // kurze Pause, Settle time
    uint8_t matrixkey = 0;  // erste Taste, ggf. mit Offset
    // SRs einer Gruppe laden, an FT_UPR und FT_LWR steht danach das erste Bit an
    _CLR_FT_LOAD; // FT_LOAD auf LOW, aktiv!
    for (uint8_t pulsecount = 0; pulsecount < scanParams.keyGroups; pulsecount++) {
      // MK/BR0..10-Bits einlesen
      _SET_FT_CLK;  // FT_CLK auf HIGH
      PORTD = portd_idle; // FT_CLK auf LOW, FT_LOAD auf HIGH für idle
      uint8_t scankey = matrixkey + tdrive;
      _NOP_DLY;
      // jetzt liegt das MK-Bit an FT_UPR und FT_LWR an
      uint8_t contacts_mk = PIND & FT_CONT_MASK1; // Make-Kontakte Upper und Lower lesen
      _SET_FT_CLK;  // FT_CLK auf HIGH
      PORTD = portd_idle; // FT_CLK auf LOW, FT_LOAD auf HIGH für idle
      if (scankey >= first_key) {  // nur gültige Tasten scannen      
        if (scankey >= first_playable_key) {  // nur gültige Tasten scannen
          // jetzt liegt das BR-Bit an FT_UPR und FT_LWR an
          scankey -= first_playable_key;
          uint8_t contacts_br = PIND & FT_CONT_MASK1; // Break-Kontakte Upper und Lower lesen
          // Nur gültige Tasten scannen, abhängig von groupOffset, scankeyEnd und presetKeys
          // Alle Tasten über groupOffset + presetKeys bis scankeyEnd - 1 sind spielbar
          AnyKeyPressed = AnyKeyPressed | contacts_mk;
          // Binäres ODER ist deutlich schneller als logisches ODER, deshalb hier mit Bitmasken arbeiten
          // Pointer statt Array-Zugriff ist dagegen NICHT schneller!
          if ((CommonKeyState[scankey] != (contacts_mk | contacts_br)) | contacts_br | contacts_mk) {
            uint8_t mk = contacts_mk & (1 << FT_UPR); // Make-Kontakt Upper lesen
            uint8_t br = contacts_br & (1 << FT_UPR); // Break-Kontakt Upper lesen
            UpperCheckstate(scankey, mk, br);
            mk = contacts_mk & (1 << FT_LWR); // Make-Kontakt Lower lesen
            br = contacts_br & (1 << FT_LWR); // Break-Kontakt Lower lesen
            LowerCheckstate(scankey, mk, br);
            CommonKeyState[scankey] = contacts_mk | contacts_br; // Zustand der Taste in beiden Manuals

          }
        } else {
          // Preset-Tasten 0..11
          scankey -= first_key;
          uint8_t mk = contacts_mk & (1 << FT_UPR); // Make-Kontakt Upper lesen
          if (UpperPresetState[scankey] != mk) {
            UpperPresetState[scankey] = mk;
            if (mk) {
              // Preset-Taste gedrückt
              MidiSendProgramChange(MenuValues[m_upper_ch], scankey); // CC mit Wert 127 senden
            }
          }
          mk = contacts_mk & (1 << FT_LWR); // Make-Kontakt Lower lesen
          if (LowerPresetState[scankey] != mk) {
            LowerPresetState[scankey] = mk;
            if (mk) {
              // Preset-Taste gedrückt
              MidiSendProgramChange(MenuValues[m_lower_ch], scankey); // CC mit Wert 127 senden
            }
          }
        }
      }
      matrixkey += scanParams.keysPerGroup; // erhöht sich mit jedem Pulspaar um die Anzahl der Tasten pro Gruppe
    }
  }
}

// #############################################################################

void ScanManualsFatar2() {
  // Upper und Lower scannen, altes FatarScan2-Board
  // Zeitbedarf für Abfrage beider 61er Manuale:
  // etwa 155µs bei 20 MHz Takt, 195µs bei 16 MHz
  _SET_TEST; // Test Pin für Debugging, z.B. mit Oszilloskop
  delayMicroseconds(1);
  _CLR_TEST;
  AnyKeyPressed = false;
  uint8_t mk, br, pin_d;
  uint8_t scankey = 0; // aktuelle Taste
  // Reset des 4017 T-Drive Counters
  // Reset des 4024 Sense Counters
  _SET_FT_TDRV_RST;
  _SET_FT_SENSE_RST;
  _NOP_DLY;
  _NOP_DLY;
  _CLR_FT_TDRV_RST;
  _CLR_FT_SENSE_RST;
  for (uint8_t tdrive = 0; tdrive < 8; tdrive++) {
    delayMicroseconds(1); // kurze Pause nach Setzen von T Drive, Settle time
    for (uint8_t pulsecount = 0; pulsecount < scanParams.keysPerGroup; pulsecount++) {
      if (scankey >= scanParams.playableKeys) scankey = scankey - scanParams.playableKeys; // Modulo bis zur maximalen Tastenzahl
      // scankey = scankey & 0x3F; // ginge auch, aber nur für 61er Tastaturen
      pin_d = (PIND & FT_CONT_MASK2) | (PINB & (1 << BR_UPR)); // relevante Pins von Port B und D lesen
      // Increment Sense Counter
      // Überlauf ist schon für nächste Gruppe, da der 4024 weiter durchläuft
      _SET_FT_SENSE_INC;
      AnyKeyPressed = AnyKeyPressed | pin_d; // Pulsdauer für 4024 Increment nutzen
      _CLR_FT_SENSE_INC;
      // Settle Time nutzen zur Auswertung der Kontakte
      if ((CommonKeyState[scankey] != pin_d) | pin_d) {
        mk = pin_d & (1 << MK_UPR); // Make-Kontakt Upper lesen
        br = pin_d & (1 << BR_UPR); // Break-Kontakt Upper lesen, hier Port B!
        UpperCheckstate(scankey, mk, br);
        mk = pin_d & (1 << MK_LWR); // Make-Kontakt Lower lesen
        br = pin_d & (1 << BR_LWR); // Break-Kontakt Lower lesen
        LowerCheckstate(scankey, mk, br);
        CommonKeyState[scankey] = pin_d; // Zustand der Taste in beiden Manuals
      }
      scankey += scanParams.keysPerGroup;
    }
    // Letzte Taste in der Sense-Gruppe, danach T-Drive inkrementieren
    _SET_FT_TDRV_INC;  // FT_CLK auf HIGH
    _NOP_DLY;
    _NOP_DLY;
    _CLR_FT_TDRV_INC; // FT_CLK auf LOW
    scankey++;
  }
}

// #############################################################################

void ScanManualsSR61() {
  // Manual mit 4014 SR scannen, Zeit für 61 Manualtasten etwa 81 us bei 20 MHz Takt
  _SET_TEST; // Test Pin für Debugging, z.B. mit Oszilloskop
  delayMicroseconds(1);
  _CLR_TEST;
  AnyKeyPressed = false; //Timing hier nicht wichtig
  uint8_t scankey, scankey_w_offs; // aktuelle Taste
  uint8_t mk_upr, mk_upr_old;
  uint8_t mk_lwr, mk_lwr_old;
  _SET_SR_LOAD;
  _NOP_DLY;
  _NOP_DLY;
  _SET_SR_CLK;
  _NOP_DLY;
  _NOP_DLY;
  _CLR_SR_CLK;
  _CLR_SR_LOAD; // Load LOW
  for (scankey = 0; scankey < scanParams.playableKeys; scankey++) {
    mk_upr = PINB & (1 << SR_UPR); // Make-Kontakt Pedal lesen, active LOW
    // Manual hat nur Make-Kontakt, deshalb keine State Machine
    mk_upr_old = UpperKeyState[scankey];
    scankey_w_offs = scankey - scanParams.keyOffset;
    if (mk_upr != mk_upr_old) {
      // Zustand hat sich geändert
      UpperKeyState[scankey] = mk_upr;
      if (mk_upr == 0) {
        // Pedal gedrückt
        MidiSendNoteOnNoDyn(MenuValues[m_upper_ch], MenuValues[MENU_BASE_UPR] + scankey_w_offs); // Upper NoteOn mit fester Dynamik
      } else {
        // Pedal losgelassen
        MidiSendNoteOff(MenuValues[m_upper_ch], MenuValues[MENU_BASE_UPR] + scankey_w_offs); // Upper NoteOff
      }
    }
    mk_lwr = PINB & (1 << SR_LWR); // Make-Kontakt Pedal lesen, active LOW
    // Manual hat nur Make-Kontakt, deshalb keine State Machine
    mk_lwr_old = LowerKeyState[scankey];
    if (mk_lwr != mk_lwr_old) {
      // Zustand hat sich geändert
      LowerKeyState[scankey] = mk_lwr;
      if (mk_lwr == 0) {
        // Pedal gedrückt
        MidiSendNoteOnNoDyn(MenuValues[m_lower_ch], MenuValues[MENU_BASE_LWR] + scankey_w_offs); // Lower NoteOn mit fester Dynamik
      } else {
        // Pedal losgelassen
        MidiSendNoteOff(MenuValues[m_lower_ch], MenuValues[MENU_BASE_LWR] + scankey_w_offs); // Lower NoteOff
      }
    }
    _SET_SR_CLK;
    _NOP_DLY;
    _NOP_DLY;
    _CLR_SR_CLK;
  }
}

// ------------------------------------------------------------------------------

void scanKeybeds() {
  // Alle Manuale und Pedale scannen, MIDI-Events senden, MIDI-Merge durchführen
  ScanPedal();    // 25 us bei 20 MHz
  switch (MenuValues[MENU_KBD_DRIVER]) {
    case drv_pulse6105:
      //ScanManualsPulse6105(); // 270 us bei 20 MHz
      //break;
    case drv_fatar1_61:
    case drv_fatar1_73:
      ScanManualsFatar1(); // 270 us bei 20 MHz
      break;
    case drv_fatar2:
      ScanManualsFatar2();   // 250 us bei 20 MHz
      break;
    case drv_sr61:
      ScanManualsSR61();     // 81 us bei 20 MHz
      break;
    default:
      break;
  }
  MidiMerge();    // nicht der Rede wert, falls nichts anliegt
}

// #############################################################################

void configurePorts(uint8_t driverType) {
  DDRC =  B00000011; // Encoder-Eingänge PC2 und PC3, MPX Data PC0 und MPX-Clk PC1 als Ausgänge
  PORTC = B00001100; // Pull-ups für Encoder-Eingänge PC2 und PC3 aktivieren
  if (driverType == drv_sr61) {
    for (uint8_t i = 0; i < KEY_ARR_SIZE; i++) {
      UpperKeyState[i] = (1 << SR_UPR);
      LowerKeyState[i] = (1 << SR_LWR);
    }
  } else {
    // Für alle anschlagdynamischen Tastaturen
    for (uint8_t i = 0; i < KEY_ARR_SIZE; i++) {
      UpperKeyState[i] = t_idle;
      LowerKeyState[i] = t_idle;
    }
  }
  // Initialisierung der Pedal-Kontaktzustände
  for (uint8_t i = 0; i < PEDAL_ARR_SIZE; i++) {
    PedalContactState[i] = (1 << SR_PED);
  }
  // Initialisierung der Timer für alle Tasten
  for (uint8_t i = 0; i < KEY_ARR_SIZE; i++) {
    UpperKeyTimer[i] = TIMER_MAX;
    LowerKeyTimer[i] = TIMER_MAX;
    CommonKeyState[i] = 0;
  }
  // Initialisierung der Preset-Kontaktzustände
  for (uint8_t i = 0; i < PRESET_ARR_SIZE; i++) {
    UpperPresetState[i] = 0;
    LowerPresetState[i] = 0;
  }
  scanParams.groupOffset  = 0;  // MK/BR-Gruppe der ersten Manual- oder Preset-Taste
  scanParams.keyOffset    = 0;  // Offset der ersten Taste innerhalb der Gruppe
  scanParams.presetKeys   = 0;  // werden für NoteOn/Off übersprungen
  scanParams.keyGroups    = 11; // Anzahl der MK/BR Paare
  scanParams.keysPerGroup = 8;  // Anzahl der Tasten pro MK/BR Gruppe
  scanParams.invertMask   = 0;  // BUGFIX für v02: 3 für FATAR 1-73, 0 für alle anderen Treiber
  Timer1.setPeriod(500);  // Timer1 auf 500 us einstellen
  switch (driverType) {
    case drv_fatar1_61:
      // FATAR Scan Controller 1 (NEU)
      // beginnt mit MK/BR0, T=0, 8 Tasten pro Gruppe, 8 Gruppen, keine Preset-Tasten
      DDRB =  B00000111; // PB0..PB2 als Ausgänge
      PORTB = B00111000; // Pull-ups für SR61- und BASS25-Eingänge aktivieren
      DDRD =  B00111110; // Keine Pullups an Eingängen PIND6 und PIND7!
      PORTD = B00010110; // Pull-ups für Eingänge aktivieren, FT_CLK low, LED PD2 off (high!)
      scanParams.keyGroups = 8; // Anzahl der MK/BR Paare, hier nur 2 SR HC166!
      break;
    case drv_fatar1_73:
      // FATAR Scan Controller 1 (NEU)
      // beginnt mit MK/BR1, T=7, 8 Tasten pro Gruppe, 11 Gruppen, 12 Preset-Tasten
      DDRB =  B00000111; // PB0..PB2 als Ausgänge
      PORTB = B00111000; // Pull-ups für SR61- und BASS25-Eingänge aktivieren
      DDRD =  B00111110; // Keine Pullups an Eingängen PIND6 und PIND7!
      PORTD = B00010110; // Pull-ups für Eingänge aktivieren, FT_CLK low, LED PD2 off (high!)
      scanParams.groupOffset = 1; // Erste Taste in Gruppe MK1/BR1
      scanParams.keyOffset  = 7;  // Erste Taste bei T=7
      scanParams.presetKeys = 12;
      scanParams.invertMask = 3;  // BUGFIX für v02
      break;
    case drv_pulse6105:
      // Pulse 6105WF Scan Controller
      // beginnt mit MK/BR3, T=3, 8 Tasten pro Gruppe, 8 Gruppen, keine Preset-Tasten
      DDRB =  B00000111; // PB0..PB2 als Ausgänge
      PORTB = B00111000; // Pull-ups für SR61- und BASS25-Eingänge aktivieren
      DDRD =  B00111110; // Keine Pullups an Eingängen PIND6 und PIND7!
      PORTD = B00010110; // Pull-ups für Eingänge aktivieren, FT_CLK low, LED PD2 off (high!)
      scanParams.groupOffset = 3; // Erste Taste in Gruppe MK3/BR3
      scanParams.keyOffset = 3;   // Erste Taste bei T=3
      break;
    case drv_fatar2:
      // FATAR Scan Controller 2, festgelegte Gruppen, keine Preset-Tasten
      DDRB =  B00000011; // PB0..PB1 als Ausgänge
      PORTB = B00111000; // Pull-ups für SR61- und BASS25-Eingänge aktivieren
      DDRD =  B00110110; // Keine Pullups an Eingängen PIND6 und PIND7!
      PORTD = B00000110; // Pull-ups für Eingänge aktivieren, LED PD2 off (high!)
      break;
    case drv_sr61:
    default:
      // SR61 Scan Controller
      DDRB =  B00000011; // PB0..PB1 als Ausgänge
      PORTB = B00111000; // Pull-ups für SR61- und BASS25-Eingänge aktivieren
      DDRD =  B00000110; // Nur Txd/Rxd und LED benutzt
      PORTD = B11111110; // Pull-ups für Eingänge aktivieren, LED PD2 off (high!)
      Timer1.setPeriod(1000); // Timer1 auf 1000 us einstellen
  }
}

void setKbdDriver() {
  // Hier wird die Portkonfiguration je nach ausgewähltem Treiber gesetzt
  #ifdef LCD_I2C
    int8_t item_value = MenuValues[MENU_KBD_DRIVER];
    lcd.setCursor(0, 1);
    if (item_value >= 0 && item_value < MENU_DRIVERCOUNT) {
      lcd.printProgmem(&DriverTypes[item_value]);
      lcd.clearEOL(); // Lösche evtl. alte Zeichen
    } else {
      lcd.print(F("(invalid)")); // ungültige Werte entsprechend kennzeichnen
      lcd.clearEOL(); // Lösche evtl. alte Zeichen
    }
    markEEPROMdifferent();
    lcd.setCursor(13, 1);
    lcd.write(LCD_ARW_LT);
  #endif
  configurePorts(MenuValues[MENU_KBD_DRIVER]);
}

void setDynSlope() {
  // Hier wird die Portkonfiguration je nach ausgewähltem Treiber gesetzt
  CreateDynTable(MenuValues[MENU_MIN_DYN], MenuValues[MENU_DYNSLOPE]);
  #ifdef LCD_I2C
    displayValueLine(); // STandard-Anzeigeroutine für Integer-Wert
  #endif
}


void setBtnMode(){
  #ifdef LCD_I2C
    int8_t item_value = MenuValues[MenuItemActiveIdx];
    lcd.setCursor(0, 1);
    if (item_value >= 0 && item_value < MENU_BTNMODECOUNT) {
      lcd.printProgmem(&ButtonModes[item_value]);
      lcd.clearEOL(); // Lösche evtl. alte Zeichen
    } else {
      lcd.print(F("(invalid)")); // ungültige Werte entsprechend kennzeichnen
      lcd.clearEOL(); // Lösche evtl. alte Zeichen
    }
    markEEPROMdifferent();
    lcd.setCursor(13, 1);
    lcd.write(LCD_ARW_LT);
    #ifdef PANEL16
      panel16.setLEDstate(MenuItemActiveIdx - MENU_BTN_MODE, panel16.btnModeToLED[item_value]); // LEDs anhand ButtonMode setzen
    #endif
  #endif
}

// #############################################################################
//
//      #####     #    #       #       ######     #     #####  #    #  #####
//     #     #   # #   #       #       #     #   # #   #     # #   #  #     #
//     #        #   #  #       #       #     #  #   #  #       #  #   #
//     #       #     # #       #       ######  #     # #       ###     #####
//     #       ####### #       #       #     # ####### #       #  #         #
//     #     # #     # #       #       #     # #     # #     # #   #  #     #
//      #####  #     # ####### ####### ######  #     #  #####  #    #  #####
//
// #############################################################################

#ifdef ANLG_MPX
// #############################################################################
// Callback für MPX-Eingänge, hier können die MIDI-CC-Werte gesendet werden
// Muss in setup() mit "mpxPots.setChangeAction(onMPXChange)" registriert werden
// #############################################################################

void onMPXChange(uint8_t inputIndex, uint8_t value) {
  if (inputIndex == MenuValues[m_pitchwheel]) {
    // Pot ist Pitchwheel, sendet Pitch Bend Change, Wert 0..127 wird auf -8192..8191 gemappt
    int16_t pitch_value = ((int16_t)value - 64) * 128; // Wert von 0..127 auf -8192..8191 mappen
    MidiSendPitchBend(MenuValues[m_upper_ch], pitch_value);
  } else if (MenuValues[MENU_POT_CC + inputIndex] >= 0) {
    // andere Potentiometer senden MIDI CC-Nummer aus MenuValues, Wert 0..127 direkt senden
    // nur senden, wenn CC zugewiesen ist, bei -1 ist kein CC zugewiesen
   ledTimerStart(20); // Board-LED für 50 Timer-Zyklen einschalten
   MidiSendController(MenuValues[m_upper_ch], MenuValues[MENU_POT_CC + inputIndex], value); // Volume Upper
  }
}
#endif

#ifdef PANEL16
// #############################################################################
// Callback-Funktion für Panel16, liefert derzeit gedrückten Button
// Wird von der Panel16-Library aufgerufen, während ein Panel16-Button gedrückt
// und auf Loslassen gewartet wird. Dadurch können die Manuale weiter
// gescannt werden, ohne die Scan-Funktion zu blockieren
// #############################################################################

void onPanel16releaseWait() {
  scanKeybeds();
  yield();
  tickerObject.update();
}

void onPanel16press(uint8_t bnt_number) {
  ledTimerStart(50); // Board-LED für 50 Timer-Zyklen einschalten
  int8_t btn_onoff = panel16.getLEDonOff(bnt_number) ? 0 : 127;
  int8_t btn_cc = MenuValues[MENU_BTN_CC + bnt_number];
  switch (MenuValues[MENU_BTN_MODE + bnt_number]) {
    case btnmode_send_cc_val:
      // Button Mode 0 = Toggle, sendet MIDI-CC mit 127 bei ON und 0 bei OFF
      MidiSendController(MenuValues[m_upper_ch], btn_cc, btn_onoff);
      panel16.toggleLEDstate(bnt_number);
      break;
    case btnmode_send_cc_evt:
      // Button Mode 1 = Event, sendet immer MIDI-CC mit 127 bei ON und bei OFF
      MidiSendController(MenuValues[m_upper_ch], btn_cc, 127);
      break;
    case btnmode_send_prg_ch:
      // Button Mode 2 = Program Change, sendet ein MIDI Program Change mit der Nummer aus MenuValues[m_btn1 + bnt_number]
      MidiSendProgramChange(MenuValues[m_upper_ch], btn_cc);
      for (uint8_t i = 0; i < 16; i++) {
        if (MenuValues[MENU_BTN_MODE + i] == btnmode_send_prg_ch) {
          panel16.setLEDonOff(i, false); // alle Program Change LEDs ausschalten
          panel16.setLEDblink(i, false); // sicherheitshalber auch den LED-Zustand zurücksetzen
        }
      }
      panel16.setLEDonOff(bnt_number, true); // aktuelle LED bei Program Change immer an
      panel16.setLEDblink(bnt_number, true); // aktuelle LED bei Program Change immer blinken
      break;
    case btnmode_send_note:
      // Button Mode 3 = Note On/Off, sendet MIDI Note On mit Velocity 64 bei ON und Note Off bei OFF, Note Nummer aus MenuValues[m_btn1 + bnt_number]
      MidiSendNoteOnNoDyn(MenuValues[m_upper_ch], btn_cc);
      panel16.waitReleased();
      MidiSendNoteOff(MenuValues[m_upper_ch], btn_cc);
      break;
  }
  panel16.waitReleased();
}
#endif

#ifdef LCD_I2C
void onMenuButton(uint8_t button) {
  // Callback-Funktion für MenuPanel-Button, liefert gedrückten Button
  handleMenuButtons(button);
}

void onMenuEncoder(int16_t delta) {
  // Callback-Funktion für MenuPanel-Encoder, liefert Bewegungsdelta
  handleMenuEncoderChange(delta);
}
#endif



// #############################################################################
//
//     #     #    #    ### #     #
//     ##   ##   # #    #  ##    #
//     # # # #  #   #   #  # #   #
//     #  #  # #     #  #  #  #  #
//     #     # #######  #  #   # #
//     #     # #     #  #  #    ##
//     #     # #     # ### #     #
//
// #############################################################################

void timer1SemaphoreISR() {
  // Timer1 Interrupt Service Routine, setzt Semaphore für Timer-basiertes Ausführen
  // der Scan- und MIDI-Merge-Funktionen im Hauptprogramm
  Timer1Semaphore++;
  Timer1RoundRobin++;
  Timer1RoundRobin &= 0x0F; // nur die unteren 4 Bits behalten = 16 Durchläufe bis zum nächsten vollen Zyklus
    // Alle 16 Durchläufe, also alle 8 ms bei 500 us Timer-Intervall, wird die LCD-Button-Abfrage ausgelöst
  if (lcdPresent) {
    // Timer-Interrupt erledigt auch das Scannen des MenuPanel-Encoders
    lcd.encoderISR();
  }
}

// ------------------------------------------------------------------------------

void setup() {
  Serial.begin(31250);
  pinMode(LED_PIN, OUTPUT);
  
  Timer1.attachInterrupt(timer1SemaphoreISR); // timer1SemaphoreISR to run every 0.5 milliseconds
  Timer1.initialize(500); // Timer1 auf 500 us einstellen
  
  Wire.begin();
  Wire.setClock(400000UL);  // 400kHz
  
  #ifdef LCD_I2C
    if (menuInit()) {
      lcdPresent = true;
      MenuItemActiveIdx = 0; // erstes Menü-Item aktivieren
      // Callback-Funktion für MenuPanel-Button-Handling registrieren
      lcd.setButtonCallback(onMenuButton); 
      // Callback-Funktion für MenuPanel-Encoder-Handling registrieren
      lcd.setEncoderCallback(onMenuEncoder);
    }
  #endif
  
  blinkLED(3);
  initMenuValues(); // Defaults-Werte laden, falls EEPROM ungültig oder nicht programmiert
  // Defaults aus EEPROM lesen
  if (EEPROM.read(EEPROM_VERSION_IDX) != FIRMWARE_VERSION) {
    // EEPROM veraltet oder nicht programmiert, auf Defaults zurücksetzen
    #ifdef LCD_I2C
      if (lcdPresent) {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(F("EEPROM invalid,"));
        lcd.setCursor(0, 1);
        lcd.print(F("reset to defaults"));
        delay(1500);
        blinkLED(2);
      }
    #else 
      delay(1000);
      blinkLED(2);
    #endif
    for (uint8_t i = 0; i < MENU_ITEMCOUNT; i++) {   
      EEPROM.update(i + EEPROM_MENUDEF_IDX, MenuDefaults[i]);
    }
    EEPROM.update(EEPROM_VERSION_IDX, FIRMWARE_VERSION);
  } else {
    // gültige Werte, in MenuValues laden
    for (uint8_t i = 0; i < MENU_ITEMCOUNT; i++) {
      MenuValues[i] = EEPROM.read(i + EEPROM_MENUDEF_IDX);
    }
  }
  #ifdef LCD_I2C
    if (lcdPresent) {
      getMenuEntry(MenuItemActiveIdx);
      displayMenuItem(); // Hauptmenü anzeigen
    }
  #endif
  configurePorts(MenuValues[MENU_KBD_DRIVER]); // Port Initialisierung je nach Treibertyp
  CreateDynTable(MenuValues[MENU_MIN_DYN], MenuValues[MENU_DYNSLOPE]);

  #ifdef PANEL16
    Wire.beginTransmission(PANEL16_I2C_ADDR); // Panel I2C-Adresse
    if (Wire.endTransmission(true) == 0) {
      panel16Present = true;
      panel16.begin();
      panel16.setPressCallback(onPanel16press); 
      panel16.setWaitCallback(onPanel16releaseWait); // Callback-Funktion für Button-Handling registrieren
      for (uint8_t i = 0; i < 16; i++) {
        panel16.setLEDstate(i, panel16.btnModeToLED[MenuValues[MENU_BTN_MODE + i]]); // LEDs anhand ButtonMode setzen
      }
      bool is_first_prg_ch = true; // Flag, um die erste gefundene Program Change LED einzuschalten
      for (uint8_t i = 0; i < 16; i++) {
        if (MenuValues[MENU_BTN_MODE + i] == btnmode_send_prg_ch) {
          panel16.setLEDonOff(i, is_first_prg_ch); // alle anderen Program Change LEDs ausschalten
          panel16.setLEDblink(i, is_first_prg_ch); // alle anderen Program Change LEDs ausschalten
          is_first_prg_ch = false; // nur die erste gefundene Program Change LED einschalten
        }
      }
    }
  #endif

  #ifdef ANLG_MPX
    mpxPots.setChangeAction(onMPXChange); // MPX-gestützte analoge Eingänge initialisieren, Callback-Funktion für Änderungen übergeben
    mpxPots.resetMPX(); // MPX-SR 74HC164 zurücksetzen
  #endif
  MidiSendController(MenuValues[m_upper_ch], 123, 0); // All Notes Off on Channel 1
  MidiSendController(MenuValues[m_lower_ch], 123, 0); // All Notes Off on Channel 2
  MidiSendController(MenuValues[m_pedal_ch], 123, 0); // All Notes Off on Channel 3
  MidiSendProgramChange(MenuValues[m_upper_ch], 0); // Program Change on Channel 1
  MidiSendProgramChange(MenuValues[m_lower_ch], 0); // Program Change on Channel 2
  MidiSendProgramChange(MenuValues[m_pedal_ch], 0); // Program Change on Channel 3
}


// #############################################################################

void loop() {
  static uint8_t old_swell = 0;
  tickerObject.update();

  while (Timer1Semaphore) {
    if (ledTimeout > 0) {
      ledTimeout--;
      if (ledTimeout == 0) {
        _LED_OFF; // LED aus
      }
    } 
    // wird alle 500µs neu gesetzt durch Timer1 ISR, hier wird die eigentliche Arbeit erledigt
    Timer1Semaphore--;

    scanKeybeds(); // Manuale und Pedale scannen, MIDI-Events generieren, etwa 300 us bei 20 MHz Takt

    #ifdef LCD_I2C
      if (lcdPresent) {
        lcd.checkEncoder(); // ruft Callback für Encoder-Bewegung auf
        if (Timer1RoundRobin == 0) {
          lcd.getButtons(); // benötigt etwa 130 µs für Button-Abfrage bei 400 kHz, ruft callbacks für gedrückte Buttons auf
        }
      }
    #endif

    #ifdef PANEL16
      // Test für Panel16 Button-Abfrage
      if (panel16Present) {
        if (Timer1RoundRobin == 8) {
          panel16.checkRow(0); // aus Zeitgründen in zwei Hälften aufteilen
        }
        if (Timer1RoundRobin == 12) {
          panel16.checkRow(1); // aus Zeitgründen in zwei Hälften aufteilen
        }
      }
    #endif

    #ifdef ANLG_MPX
      if (Timer1RoundRobin == 2) {
        mpxPots.handleMPX(); // muss regelmäßig aufgerufen werden, um Änderungen aller Potis zu erkennen
      }
     if (Timer1RoundRobin == 6) {
        if (MenuValues[m_swell] >= 0) {
          uint8_t swell = mpxPots.getSwell() / 2; // aktueller Wert des Swell-Pedals, 0..127
          if (swell != old_swell) {
            MidiSendController(MenuValues[m_upper_ch], MenuValues[m_swell], swell); // sendet MIDI
            old_swell = swell;
          }
        }
      }
    #endif
  }
}

