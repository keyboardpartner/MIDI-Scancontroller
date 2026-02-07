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
// MIDI Controller for FATAR keybeds, C. Meyer 1/2026
// Banner Logos from
// https://patorjk.com/software/taag/#p=display&f=Banner&t=MAIN&x=cppComment&v=4&h=2&w=80&we=false
// 20 MHz Bootloaders: 
// https://github.com/MCUdude/MiniCore/tree/master/avr/bootloaders/optiboot_flash/bootloaders

#include <Arduino.h>
#include <EEPROM.h>
#include <TimerOne.h>
#include "midi_io.h"

#define LED_PIN 2 // Pin für LED
#define EEPROM_MENUDEFAULTS 16 // Startadresse im EEPROM für gespeicherte Werte

// FATAR 1-61 Scan-Controller NEU Pinbelegung
#define FT_TDRV_A   PORTB0
#define FT_TDRV_B   PORTB1
#define FT_TDRV_C   PORTB2
#define FT_CLK   PORTD3
#define FT_LOAD  PORTD4
// #define FT_KBDPOL PORTD5
#define FT_TEST  PORTD5
#define FT_UPR   PIND6
#define FT_LWR   PIND7
// Fast port bit manipulation macros
#define _SET_FT_CLK  asm volatile("sbi %0,%1 " : : "I" (_SFR_IO_ADDR(PORTD)), "I" (FT_CLK))
#define _CLR_FT_CLK  asm volatile("cbi %0,%1 " : : "I" (_SFR_IO_ADDR(PORTD)), "I" (FT_CLK))
#define _SET_FT_LOAD  asm volatile("sbi %0,%1 " : : "I" (_SFR_IO_ADDR(PORTD)), "I" (FT_LOAD))
#define _CLR_FT_LOAD  asm volatile("cbi %0,%1 " : : "I" (_SFR_IO_ADDR(PORTD)), "I" (FT_LOAD))

// FATAR 2 Scan-Controller ALT Pinbelegung
#define FT_SENSE_INC   PORTB0
#define FT_SENSE_RST   PORTB1
#define BR_UPR   PORTB2
#define MK_UPR   PORTD3
#define FT_TDRV_INC  PORTD4
#define FT_TDRV_RST  PORTD5
#define BR_LWR   PIND6
#define MK_LWR   PIND7
// Fast port bit manipulation macros
#define _SET_FT_SENSE_INC  asm volatile("sbi %0,%1 " : : "I" (_SFR_IO_ADDR(PORTB)), "I" (FT_SENSE_INC))
#define _CLR_FT_SENSE_INC  asm volatile("cbi %0,%1 " : : "I" (_SFR_IO_ADDR(PORTB)), "I" (FT_SENSE_INC))
#define _SET_FT_SENSE_RST  asm volatile("sbi %0,%1 " : : "I" (_SFR_IO_ADDR(PORTB)), "I" (FT_SENSE_RST))
#define _CLR_FT_SENSE_RST  asm volatile("cbi %0,%1 " : : "I" (_SFR_IO_ADDR(PORTB)), "I" (FT_SENSE_RST))
#define _SET_FT_TDRV_INC  asm volatile("sbi %0,%1 " : : "I" (_SFR_IO_ADDR(PORTD)), "I" (FT_TDRV_INC))
#define _CLR_FT_TDRV_INC  asm volatile("cbi %0,%1 " : : "I" (_SFR_IO_ADDR(PORTD)), "I" (FT_TDRV_INC))
#define _SET_FT_TDRV_RST  asm volatile("sbi %0,%1 " : : "I" (_SFR_IO_ADDR(PORTD)), "I" (FT_TDRV_RST))
#define _CLR_FT_TDRV_RST  asm volatile("cbi %0,%1 " : : "I" (_SFR_IO_ADDR(PORTD)), "I" (FT_TDRV_RST))

// BASS und SR61 4014 Scan-Controller Pinbelegung
#define SR_CLK   PORTB0 // auf Prototyp V01 ändern!
#define SR_LOAD  PORTB1 // auf Prototyp V01 ändern!
#define SR_UPR   PINB3
#define SR_LWR   PINB4
#define SR_PED   PINB5
// Fast port bit manipulation Macros
#define _SET_SR_CLK  asm volatile("sbi %0,%1 " : : "I" (_SFR_IO_ADDR(PORTB)), "I" (SR_CLK))
#define _CLR_SR_CLK  asm volatile("cbi %0,%1 " : : "I" (_SFR_IO_ADDR(PORTB)), "I" (SR_CLK))
#define _SET_SR_LOAD  asm volatile("sbi %0,%1 " : : "I" (_SFR_IO_ADDR(PORTB)), "I" (SR_LOAD))
#define _CLR_SR_LOAD  asm volatile("cbi %0,%1 " : : "I" (_SFR_IO_ADDR(PORTB)), "I" (SR_LOAD))

#define _NOP_DLY asm volatile ("nop")

#define KEYS_PER_GROUP 8
#define KEYS (KEYS_PER_GROUP * 8) // 8 Treibergruppen à 8 Tasten = 64 Tasten pro Manual

#define TIMER_MAX 255
#define TIMER_DIV ((TIMER_MAX * 2) + MIDI_MINDYN)

#define PEDALKEYS 25
#define MANUALKEYS 61
#define ANLG_INPUTS 4 // Analoge Eingänge für MIDI-CC-Potentiometer, 0 für keine analogen Eingänge

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

uint8_t UpperKeyState[KEYS]; // Zustand der Tasten
uint8_t LowerKeyState[KEYS]; // Zustand der Tasten
uint8_t PedalContactState[PEDALKEYS]; // Zustand der Pedaltasten
uint8_t AnyKeyPressed = false;

uint8_t UpperKeyTimer[KEYS]; // Timer für jede Taste
uint8_t LowerKeyTimer[KEYS]; // Timer für jede Taste


volatile uint8_t Timer1Semaphore = 0;
volatile uint8_t Timer1RoundRobin = 0;

#define LCD_I2C
#define ANLG_MPX
#define PANEL16

#ifdef LCD_I2C
  // Für LCD mit I2C-Interface
  #include "MenuPanel.h"
  #define LCD_I2C_ADDR 0x20
  MenuPanel lcd(LCD_I2C_ADDR, 16, 2);
#endif
bool lcdPresent = false;

#ifdef PANEL16
  // Für LCD mit I2C-Interface
  #include "Panel16.h"
  #define PANEL16_I2C_ADDR 0x62
  Panel16 panel16(PANEL16_I2C_ADDR);
#endif
bool panel16Present = false;

#ifdef ANLG_MPX
  // Für MPX-gestützte analoge Eingänge
  #include "MpxPots.h"
  MPXpots mpxPots(ANLG_INPUTS, MPX_ACTIVE_TIMEOUT, MPX_INTEGRATOR_FACTOR);
#endif



// Menu System Variables

#define MENU_ITEMCOUNT 14
enum {m_upper_channel, m_lower_channel, m_pedal_channel, m_driver_type, 
      m_upper_base, m_lower_base, m_pedal_base, m_mindyn, m_maxdynadj, m_slope,
      m_CC1, m_CC2, m_CC3, m_CC4};
const lcdTextType MenuItems[MENU_ITEMCOUNT] PROGMEM = { 
  { "Upper Channel" }, 
  { "Lower Channel" }, 
  { "Pedal Channel" }, 
  { "Kbd Driver" }, 
  { "Upper Base" }, 
  { "Lower Base" }, 
  { "Pedal Base" }, 
  { "Velocity Min" }, 
  { "Velocity MaxAdj" }, // maximale String-Länge 15 Zeichen
  { "Velocity Slope" },
  { "CC1 Number" }, 
  { "CC2 Number" }, 
  { "CC3 Number" }, 
  { "CC4 Number" }, 
};

#define MENU_DRIVERCOUNT 4
enum {drv_sr61, drv_fatar1, drv_fatar2, drv_custom};
const lcdTextType DriverTypes[MENU_DRIVERCOUNT] PROGMEM = { 
  { "Scan16/61" }, 
  { "FatarScan1-61" }, 
  { "FatarScan2" }, 
  { "Custom" },
};

uint8_t MenuItemActive = m_upper_channel;
const int8_t MenuValueMin[] = {1, 1, 1, 0, 12, 12, 12, 0, 0, 0, 0, 0, 0, 0};
const int8_t MenuValueMax[] = {16, 16, 16, MENU_DRIVERCOUNT - 1, 60, 60, 60, 40, 40, 20, 127, 127, 127, 127};
const int8_t MenuValueDefaults[] = {MIDI_CH_UPR, MIDI_CH_LWR, MIDI_CH_PED, drv_fatar1, MIDI_BASE_UPR, MIDI_BASE_LWR, MIDI_BASE_PED, MIDI_MINDYN, MIDI_MAXDYNADJ, MIDI_DYNSLOPE, 7, 10, 11, 91};
const String Msg[] = {"FCK TRMP", "FCK AFD"};
int8_t MenuValues[MENU_ITEMCOUNT];


// #############################################################################
//
//     #     # ######  #     #      #    #     # #        #####
//     ##   ## #     #  #   #      # #   ##    # #       #     #
//     # # # # #     #   # #      #   #  # #   # #       #
//     #  #  # ######     #      #     # #  #  # #       #  ####
//     #     # #         # #     ####### #   # # #       #     #
//     #     # #        #   #    #     # #    ## #       #     #
//     #     # #       #     #   #     # #     # #######  #####
//
// #############################################################################

// #############################################################################
// Callback für MPX-Eingänge, hier können die MIDI-CC-Werte gesendet werden
// #############################################################################

#ifdef ANLG_MPX

// Callback-Funktion für Änderungen der MPX-gestützten analogen Eingänge, hier können die MIDI-CC-Werte gesendet werden
// Muss in setup() mit "mpxPots.setChangeAction(onMPXChange)" registriert werden
void onMPXChange(uint8_t inputIndex, uint8_t value){
  if (inputIndex < ANLG_INPUTS) {
    MidiSendController(MenuValues[m_upper_channel], MenuValues[m_CC1 + inputIndex], value); // Volume Upper
  }
}

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
enum {t_idle, t_forward, t_pressed, t_reverse};

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
      uint16_t tval = UpperKeyTimer[scankey] + MenuValues[m_maxdynadj];
      if (tval > 255) tval = 255;
      uint8_t mididyn = TimeToDyn[tval];
      MidiSendNoteOn(MenuValues[m_upper_channel], MenuValues[m_upper_base] + scankey, mididyn);  // sende MIDI NoteOn mit Dynamikwert
    } else if (br) {
      // Break-Kontakt noch geschlossen, Timer dekrementieren
      uint8_t tval = UpperKeyTimer[scankey];
      if (tval > 0) {
        tval--;
        UpperKeyTimer[scankey] = tval;
      } else {
        // sehr langsames Drücken oder verschmutzt, Maximalwert erreicht
        UpperKeyState[scankey] = t_pressed;
        MidiSendNoteOn(MenuValues[m_upper_channel], MenuValues[m_upper_base] + scankey, MenuValues[m_mindyn]);  // sende MIDI NoteOn mit Dynamikwert
      }
    } else {
      // Break-Kontakt offen, Taste wieder losgelassen
      UpperKeyState[scankey] = t_reverse;
    }
    break;
  case t_pressed:
    if (!mk) {
      // Make-Kontakt offen, Taste bewegt sich zurück
      UpperKeyState[scankey] = t_reverse;
    }
    break;
  case t_reverse:
    if (!(mk || br)) {
      // Make- und Break-Kontakt offen, Taste losgelassen
      UpperKeyState[scankey] = t_idle;
      MidiSendNoteOff(MenuValues[m_upper_channel], MenuValues[m_upper_base] + scankey);// sende MIDI NoteOFF
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
      uint16_t tval = LowerKeyTimer[scankey] + MenuValues[m_maxdynadj];
      if (tval > 255) tval = 255;
      uint8_t mididyn = TimeToDyn[tval];
      MidiSendNoteOn(MenuValues[m_lower_channel], MenuValues[m_lower_base] + scankey, mididyn);  // sende MIDI NoteOn mit Dynamikwert
    } else if (br) {
      // Break-Kontakt noch geschlossen, Timer dekrementieren
      uint8_t tval = LowerKeyTimer[scankey];
      if (tval > 0) {
        tval--;
        LowerKeyTimer[scankey] = tval;
      } else {
        // sehr langsames Drücken oder verschmutzt, Maximalwert erreicht
        LowerKeyState[scankey] = t_pressed;
        MidiSendNoteOn(MenuValues[m_lower_channel], MenuValues[m_lower_base] + scankey, MenuValues[m_mindyn]);  // sende MIDI NoteOn mit Dynamikwert
     }
    } else {
      // Break-Kontakt offen, Taste wieder losgelassen
      LowerKeyState[scankey] = t_reverse;
    }
    break;
  case t_pressed:
    if (!mk) {
      // Make-Kontakt offen, Taste bewegt sich zurück
      LowerKeyState[scankey] = t_reverse;
    }
    break;
  case t_reverse:
    if (!(mk || br)) {
      // Make- und Break-Kontakt offen, Taste losgelassen
      LowerKeyState[scankey] = t_idle;
      MidiSendNoteOff(MenuValues[m_lower_channel], MenuValues[m_lower_base] + scankey);// sende MIDI NoteOFF
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
  for (scankey = 0; scankey < PEDALKEYS; scankey++) {
    mk_ped = PINB & (1 << SR_PED); // Make-Kontakt Pedal lesen, active LOW
    // Pedal hat nur Make-Kontakt, deshalb keine State Machine
    mk_old = PedalContactState[scankey];
    if (mk_ped != mk_old) {
      // Zustand hat sich geändert
      PedalContactState[scankey] = mk_ped;
      if (mk_ped == 0) {
        // Pedal gedrückt, NoteOn mit fester Dynamik
        MidiSendNoteOnNoDyn(MenuValues[m_pedal_channel], MenuValues[m_pedal_base] + scankey);
      } else {
        // Pedal losgelassen
        MidiSendNoteOff(MenuValues[m_pedal_channel], MenuValues[m_pedal_base] + scankey);
      }
    }
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

void ScanManualsFatar1_61() {
  // Upper und Lower scannen
  // Zeitbedarf für ZWEI 61er Manuale etwa 200 us bei 20 MHz Takt, 250 us bei 16 MHz
  // Zeitbedarf für EIN 61er Manual etwa 150 us bei 20 MHz Takt, 190 us bei 16 MHz
  AnyKeyPressed = false;
  uint8_t scankey = 0; // aktuelle Taste
  uint8_t mk_upr, br_upr, mk_lwr, br_lwr;
  uint8_t portd_idle = (PORTD & ~(1 << FT_CLK)) | (1 << FT_LOAD); // Set bit 3 LOW, 4 HIGH for idle
  PORTD = portd_idle;  // zurück zu idle
  for (uint8_t tdrive = 0; tdrive < 8; tdrive++) {
    PORTB = (PORTB & B11111000) | tdrive; // nur unterste 3 Bits, als T-Drive an Decoder
    delayMicroseconds(1); // kurze Pause, Settle time
    // SRs einer Gruppe laden, an FT_UPR und FT_LWR steht danach das erste Bit an
    _CLR_FT_LOAD; // FT_LOAD auf LOW
    _SET_FT_CLK;  // FT_CLK auf HIGH
    PORTD = portd_idle;  // zurück zu idle
    for (uint8_t pulsecount = 0; pulsecount < KEYS_PER_GROUP; pulsecount++) {
      if (pulsecount != 0) {
        // nur nächstes Bit
        _SET_FT_CLK;  // FT_CLK auf HIGH
        _CLR_FT_CLK;  // FT_CLK auf LOW
      }
      // jetzt liegt das MK-Bit an FT_UPR und FT_LWR an
      // Berechnung ist auch eine kleine Pause zum Settle des Eingangspins:
      if (scankey >= KEYS) scankey = scankey - KEYS; // Modulo bis zur maximalen Tastenzahl
      // scankey = scankey & 0x3F; // ginge auch, aber nur für 61er Tastaturen
      mk_upr = PIND & (1 << FT_UPR); // Make-Kontakt Upper lesen
      mk_lwr = PIND & (1 << FT_LWR); // Make-Kontakt Lower lesen
      // BR-Bit(s) einlesen
      _SET_FT_CLK;  // FT_CLK auf HIGH
      _CLR_FT_CLK;  // FT_CLK auf LOW
      AnyKeyPressed = AnyKeyPressed | br_lwr | br_upr; // kleine Pause zum Settle, WICHTIG!
      // asm volatile ("nop");
      br_upr = PIND & (1 << FT_UPR); // Break-Kontakt Upper lesen
      br_lwr = PIND & (1 << FT_LWR); // Break-Kontakt Lower lesen
      UpperCheckstate(scankey, mk_upr, br_upr);
      LowerCheckstate(scankey, mk_lwr, br_lwr);
      scankey += KEYS_PER_GROUP;
    }
    scankey++;
  }
}

// #############################################################################

void ScanManualsFatar2() {
  // Upper und Lower scannen, altes FatarScan2-Board
  // Zeitbedarf für ZWEI 61er Manuale etwa 200 us bei 20 MHz Takt, 250 us bei 16 MHz
  // Zeitbedarf für EIN 61er Manual etwa 150 us bei 20 MHz Takt, 190 us bei 16 MHz
  AnyKeyPressed = false;
  uint8_t mk_upr, br_upr, mk_lwr, br_lwr;
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
    for (uint8_t pulsecount = 0; pulsecount < KEYS_PER_GROUP; pulsecount++) {
      if (scankey >= KEYS) scankey = scankey - KEYS; // Modulo bis zur maximalen Tastenzahl
      // scankey = scankey & 0x3F; // ginge auch, aber nur für 61er Tastaturen
      mk_upr = PIND & (1 << MK_UPR); // Make-Kontakt Upper lesen
      mk_lwr = PIND & (1 << MK_LWR); // Make-Kontakt Lower lesen
      br_upr = PINB & (1 << BR_UPR); // Break-Kontakt Upper lesen, hier Port B!
      br_lwr = PIND & (1 << BR_LWR); // Break-Kontakt Lower lesen
      // Increment Sense Counter
      // Überlauf ist schon für nächste Gruppe, da der 4024 weiter durchläuft
      _SET_FT_SENSE_INC;
      AnyKeyPressed = AnyKeyPressed | br_lwr | br_upr; // Pulsdauer für 4024 Increment
      _CLR_FT_SENSE_INC;
      // Settle Time nutzen zur Auswertung der Kontakte
      UpperCheckstate(scankey, mk_upr, br_upr);
      LowerCheckstate(scankey, mk_lwr, br_lwr);
      scankey += KEYS_PER_GROUP;
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
  AnyKeyPressed = false; //Timing hier nicht wichtig
  uint8_t scankey; // aktuelle Taste
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
  for (scankey = 0; scankey < MANUALKEYS; scankey++) {
    mk_upr = PINB & (1 << SR_UPR); // Make-Kontakt Pedal lesen, active LOW
    // Manual hat nur Make-Kontakt, deshalb keine State Machine
    mk_upr_old = UpperKeyState[scankey];
    if (mk_upr != mk_upr_old) {
      // Zustand hat sich geändert
      UpperKeyState[scankey] = mk_upr;
      if (mk_upr == 0) {
        // Pedal gedrückt
        MidiSendNoteOnNoDyn(MenuValues[m_upper_channel], MIDI_BASE_UPR + scankey); // Upper NoteOn mit fester Dynamik
      } else {
        // Pedal losgelassen
        MidiSendNoteOff(MenuValues[m_upper_channel], MIDI_BASE_UPR + scankey); // Upper NoteOff
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
        MidiSendNoteOnNoDyn(MenuValues[m_lower_channel], MIDI_BASE_LWR + scankey); // Lower NoteOn mit fester Dynamik
      } else {
        // Pedal losgelassen
        MidiSendNoteOff(MenuValues[m_lower_channel], MIDI_BASE_LWR + scankey); // Lower NoteOff
      }
    }
    _SET_SR_CLK;
    _NOP_DLY;
    _NOP_DLY;
    _CLR_SR_CLK;
  }
}

// #############################################################################

void configurePorts(uint8_t driverType) {
  DDRC =  B00000011; // Encoder-Eingänge PC2 und PC3, MPX Data PC0 und MPX-Clk PC1 als Ausgänge
  PORTC = B00001100; // Pull-ups für Encoder-Eingänge PC2 und PC3 aktivieren
  switch (driverType) {
    case drv_fatar1:
      // FATAR Scan Controller 1 (NEU)
      DDRB =  B00000111; // PB0..PB2 als Ausgänge
      PORTB = B00111000; // Pull-ups für SR61- und BASS25-Eingänge aktivieren
      DDRD =  B00111110; // Keine Pullups an Eingängen PIND6 und PIND7!
      PORTD = B00010110; // Pull-ups für Eingänge aktivieren, FT_CLK low, LED PD2 off (high!)
      // Initialisierung der State Machines für FATAR Scan-Controller, anschlagdynamisch
      for (uint8_t i = 0; i < KEYS; i++) {
        UpperKeyState[i] = t_idle;
        LowerKeyState[i] = t_idle;
      }
      break;
    case drv_fatar2:
      // FATAR Scan Controller 2
      DDRB =  B00000011; // PB0..PB1 als Ausgänge
      PORTB = B00111000; // Pull-ups für SR61- und BASS25-Eingänge aktivieren
      DDRD =  B00110110; // Keine Pullups an Eingängen PIND6 und PIND7!
      PORTD = B00000110; // Pull-ups für Eingänge aktivieren, LED PD2 off (high!)
      // Initialisierung der State Machines für FATAR Scan-Controller, anschlagdynamisch
      for (uint8_t i = 0; i < KEYS; i++) {
        UpperKeyState[i] = t_idle;
        LowerKeyState[i] = t_idle;
      }
      break;
    case drv_sr61:
    default:
      // SR61 Scan Controller
      DDRB =  B00000011; // PB0..PB1 als Ausgänge
      PORTB = B00111000; // Pull-ups für SR61- und BASS25-Eingänge aktivieren
      DDRD =  B00000110; // Nur Txd/Rxd und LED benutzt
      PORTD = B11111110; // Pull-ups für Eingänge aktivieren, LED PD2 off (high!)
      // Initialisierung der State Machines für SR61 Scan-Controller, nicht anschlagdynamisch
      for (uint8_t i = 0; i < KEYS; i++) {
        UpperKeyState[i] = (1 << SR_UPR);
        LowerKeyState[i] = (1 << SR_LWR);
      }
  }
  for (uint8_t i = 0; i < KEYS; i++) {
    UpperKeyTimer[i] = 255;
    LowerKeyTimer[i] = 255;
  }
  // Initialisierung der Pedal-Kontaktzustände
  for (uint8_t i = 0; i < PEDALKEYS; i++) {
    PedalContactState[i] = (1 << SR_PED);
  }
}


// #############################################################################
//
//     #     # ####### #     # #     #
//     ##   ## #       ##    # #     #
//     # # # # #       # #   # #     #
//     #  #  # #####   #  #  # #     #
//     #     # #       #   # # #     #
//     #     # #       #    ## #     #
//     #     # ####### #     #  #####
//
// #############################################################################


void blinkLED(uint8_t times) {
  for (uint8_t i=0; i<times; i++) {
    digitalWrite(LED_PIN, LOW); // sets the LED on
    delay(150);
    digitalWrite(LED_PIN, HIGH);  // sets the LED off
    delay(150);
  }
}

#ifdef LCD_I2C

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
  lcd.print(LCD_ARW_LT);
  if (item_value != EEPROM.read(itemIndex + EEPROM_MENUDEFAULTS)) {
    lcd.setCursor(15, 1);
    lcd.print('*'); // geänderte Werte mit Stern markieren
  }
}

void displayMenuItem(uint8_t itemIndex) {
  lcd.setCursor(0, 0);
  // Kopiert MenuItem aus PROGMEM ins RAM, da lcd.print() nicht direkt aus PROGMEM lesen kann
  lcd.printProgmem(&MenuItems[itemIndex]);
  lcd.clearEOL(); // Lösche evtl. alte Zeichen
  lcd.setCursor(15, 0);
  lcd.print(LCD_ARW_UD);
  displayMenuValue(itemIndex);
}

void handleEncoder(int16_t encoderDelta, bool forceDisplay) {
  // Menü-Handling bei Encoder-Änderungen: Wert ändern, 
  // bei Änderung des Treibertyps Ports neu konfigurieren, Dynamiktabelle neu erstellen
  if ((encoderDelta != 0) || forceDisplay) {
    // Encoder hat sich bewegt
    int8_t oldValue = MenuValues[MenuItemActive];
    if (oldValue + encoderDelta < MenuValueMin[MenuItemActive]) {
      MenuValues[MenuItemActive] = MenuValueMin[MenuItemActive]; // Unterlauf verhindern
    } else if (oldValue + encoderDelta > MenuValueMax[MenuItemActive]) {
      MenuValues[MenuItemActive] = MenuValueMax[MenuItemActive]; // Maximalwert
    } else {
      MenuValues[MenuItemActive] = oldValue + encoderDelta;
    }
    displayMenuValue(MenuItemActive);
    if (MenuItemActive == m_driver_type) {
      // PortD neu konfigurieren
      configurePorts(MenuValues[m_driver_type]);
      if (MenuValues[m_driver_type] >= drv_fatar1) {
        Timer1.setPeriod(500);  // Timer1 auf 500 us einstellen
      } else {
        Timer1.setPeriod(1000); // Timer1 auf 1000 us einstellen
      }
    }
    if ((MenuItemActive == m_mindyn ) || (MenuItemActive == m_slope)) {
      CreateDynTable(MenuValues[m_mindyn], MenuValues[m_slope]);
    }
  }
}

void handleButtons() {
  // Menü-Handling bei Button-Änderungen: Menupunkt wechseln oder Wert in EEPROM speichern
  uint8_t buttons = lcd.getButtons(); // benötigt etwa 130 µs (inkl. I2C Overhead) bei 400 kHz
  if (buttons != 0) {
    displayMenuItem(MenuItemActive);
    if (buttons & LCD_BTNUP_MASK) {
      // Up-Taste
      if (MenuItemActive > 0) {
        MenuItemActive--;
      } else {
        MenuItemActive = MENU_ITEMCOUNT - 1; // wrap around
      }
      displayMenuItem(MenuItemActive);
    }
    if (buttons & LCD_BTNDN_MASK) {
      // Down-Taste
      if (MenuItemActive < MENU_ITEMCOUNT - 1) {
        MenuItemActive++;
      } else {
        MenuItemActive = 0; // wrap around
      }
      displayMenuItem(MenuItemActive);
    }
    if (buttons & LCD_BTNENTER_MASK) {
      // Enter-Taste, Wert in EEPROM speichern
      EEPROM.update(MenuItemActive + EEPROM_MENUDEFAULTS, MenuValues[MenuItemActive]);
      displayMenuItem(MenuItemActive);
      // Kurzes Blinken als Bestätigung
      blinkLED(1);
    }
    lcd.getButtonsWaitReleased(); // Warte bis losgelassen
  }
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
  Timer1RoundRobin &= 0x0F; // nur die unteren 4 Bits behalten
}

void setup() {
  Serial.begin(31250);
  // set led port as output
  pinMode(LED_PIN, OUTPUT);
  // Defaults aus EEPROM lesen
  for (uint8_t i = 0; i < MENU_ITEMCOUNT; i++) {
    uint8_t eep_val = EEPROM.read(i + EEPROM_MENUDEFAULTS);
    if ((eep_val < MenuValueMin[i]) || (eep_val > MenuValueMax[i])) {
      // ungültiger Wert, auf default zurücksetzen
      eep_val = MenuValueDefaults[i];
      EEPROM.update(i + EEPROM_MENUDEFAULTS, eep_val);
    }
    MenuValues[i] = eep_val;
  }
  configurePorts(MenuValues[m_driver_type]); // Port Initialisierung je nach Treibertyp
  CreateDynTable(MenuValues[m_mindyn], MenuValues[m_slope]);

  Timer1.attachInterrupt(timer1SemaphoreISR); // timer1SemaphoreISR to run every 0.5 milliseconds
  if (MenuValues[m_driver_type] >= drv_fatar1) {
    Timer1.setPeriod(500);  // Timer1 auf 500 us einstellen
  } else {
    Timer1.setPeriod(1000); // Timer1 auf 1000 us einstellen
  }
  Timer1.initialize(500); // Timer1 auf 500 us einstellen

  MidiSendController(MenuValues[m_upper_channel], 123, 0); // All Notes Off on Channel 1
  MidiSendController(MenuValues[m_lower_channel], 123, 0); // All Notes Off on Channel 1
  MidiSendController(MenuValues[m_pedal_channel], 123, 0); // All Notes Off on Channel 1

  Wire.begin();
  Wire.setClock(400000UL);  // 400kHz

  #ifdef LCD_I2C
    Wire.beginTransmission(LCD_I2C_ADDR); // Display I2C-Adresse
    if (Wire.endTransmission(true) == 0) {
      // Display gefunden
      lcdPresent = true;
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
      blinkLED(5);
      displayMenuItem(MenuItemActive);
    } else {
      // Kein Display gefunden
      blinkLED(3);
    }
  #else
    // Kein Display-Support
    blinkLED(3);
  #endif

  #ifdef PANEL16
    // just a test for Panel16 library
    Wire.beginTransmission(PANEL16_I2C_ADDR); // Panel I2C-Adresse
    if (Wire.endTransmission(true) == 0) {
      panel16Present = true;
      panel16.begin();
      panel16.setLEDstate(2, panel16.led_hilight | panel16.led_alt_dark | panel16.led_blink_ena); // einzelne LED in lower row
      panel16.setLEDstate(3, panel16.led_dark | panel16.led_alt_dark | panel16.led_btn_on); // einzelne LED in lower row
      panel16.setLEDstate(4, 0b10001001); // einzelne LED in lower row, direkte Bitmask, entspricht hilight, alt_bright, off_dark, blink_ena
      panel16.setLEDstate(8, panel16.led_hilight | panel16.led_alt_bright | panel16.led_off_dark | panel16.led_blink_ena); // einzelne LED in lower row
      panel16.setLEDstate(13, panel16.led_dark | panel16.led_btn_on); // einzelne LED in upper row
    }
  #endif

  #ifdef ANLG_MPX
    mpxPots.setChangeAction(onMPXChange); // MPX-gestützte analoge Eingänge initialisieren, Callback-Funktion für Änderungen übergeben
    mpxPots.resetMPX(); // MPX-SR 74HC164 zurücksetzen
  #endif
}

// #############################################################################

void loop() {
  while (Timer1Semaphore) {
    // wird alle 500µs neu gesetzt durch Timer1 ISR, hier wird die eigentliche Arbeit erledigt
    Timer1Semaphore--;
    ScanPedal();    // 25 us bei 20 MHz
    switch (MenuValues[m_driver_type]) {
      case drv_fatar1:
        ScanManualsFatar1_61(); // 270 us bei 20 MHz
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
    #ifdef LCD_I2C
      if (lcdPresent) {
        handleEncoder(lcd.getEncoderDelta(), false);
        if ((Timer1RoundRobin == 0) && (AnyKeyPressed == 0)) {
          handleButtons(); // benötigt etwa 130 µs für Button-Abfrage bei 400 kHz
        }
      }
    #endif

    #ifdef PANEL16
      // Test für Panel16 Button-Abfrage
      if (panel16Present && (AnyKeyPressed == 0)) {
        if (Timer1RoundRobin == 4) {
          panel16.updateBlinkLEDs();
        }
        if (Timer1RoundRobin == 8) {
          uint8_t bnt_number = panel16.getButtonRow(0); // benötigt etwa 550 µs für Button-Abfrage bei 400 kHz
          if (bnt_number != 0xFF) {
            lcd.setCursor(0, 1);
            lcd.print("Btn: ");
            lcd.print(bnt_number, DEC);
            panel16.toggleLEDstate(bnt_number);
            panel16.getButtonRowWaitReleased(0);
            #ifdef LCD_I2C
              if (lcdPresent) displayMenuItem(MenuItemActive);
            #endif
          }; 
        }      
        if (Timer1RoundRobin == 12) {
          uint8_t bnt_number = panel16.getButtonRow(1); // benötigt etwa 550 µs für Button-Abfrage bei 400 kHz
          if (bnt_number != 0xFF) {
            lcd.setCursor(0, 1);
            lcd.print("Btn: ");
            lcd.print(bnt_number, DEC);
            panel16.toggleLEDstate(bnt_number);
            panel16.getButtonRowWaitReleased(1);
            #ifdef LCD_I2C
              if (lcdPresent) displayMenuItem(MenuItemActive);
            #endif
          }; 
        }
      }
    #endif

    #ifdef ANLG_MPX
      if (Timer1RoundRobin == 4) {
        mpxPots.handleMPX(); // muss regelmäßig aufgerufen werden, um Änderungen aller Potis zu erkennen
      }
    #endif
  }
}

