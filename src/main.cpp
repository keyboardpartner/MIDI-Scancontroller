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

#include <Arduino.h>
#include <EEPROM.h>
#include <TimerOne.h>

#define LED_PIN 2 // Pin für LED
#define EEPROM_MENUDEFAULTS 16 // Startadresse im EEPROM für gespeicherte Werte

//#define FATARSCAN_NEW // Neue FATAR Scan-Controller Platine
//#define FATARSCAN_OLD // Alte FATAR Scan-Controller Platine
#define SCAN_SR61 // Klassische 4014-Scan-Platine Scan16, Scan61 ggf. plus BASS25

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

// FATAR 2 Scan-Controller ALT Pinbelegung
#define FT_SENSE_INC   PORTB0
#define FT_SENSE_RST   PORTB1
#define BR_UPR   PORTB2
#define MK_UPR   PORTD3
#define FT_TDRV_INC  PORTD4
#define FT_TDRV_RST  PORTD5
#define BR_LWR   PIND6
#define MK_LWR   PIND7


#define KEYS_PER_GROUP 8
#define KEYS (KEYS_PER_GROUP * 8) // 8 Treibergruppen à 8 Tasten = 64 Tasten pro Manual

#define TIMER_MAX 255
#define TIMER_DIV ((TIMER_MAX * 2) + MIDI_MINDYN)

// BASS und SR61 4014 Scan-Controller Pinbelegung
#define SR_CLK   PORTB0 // auf Prototyp V01 ändern!
#define SR_LOAD  PORTB1 // auf Prototyp V01 ändern!
#define SR_UPR   PINB3
#define SR_LWR   PINB4
#define SR_PED   PINB5

#define PEDALKEYS 25
#define MANUALKEYS 61

// Default MIDI Einstellungen
#define MIDI_BASE_UPR 36
#define MIDI_BASE_LWR 36
#define MIDI_BASE_PED 36

#define MIDI_CH_UPR 1
#define MIDI_CH_LWR 2
#define MIDI_CH_PED 3

#define MIDI_MINDYN 10
#define MIDI_DYNSLOPE 10

uint8_t UpperKeyState[128]; // Zustand der Tasten
uint8_t LowerKeyState[128]; // Zustand der Tasten
uint8_t PedalContactState[PEDALKEYS]; // Zustand der Pedaltasten

uint8_t UpperKeyTimer[128]; // Timer für jede Taste
uint8_t LowerKeyTimer[128]; // Timer für jede Taste

#define LCD_I2C

#ifdef LCD_I2C
  // Für LCD mit I2C-Interface
  #include "MenuPanel.h"
  #define LCD_I2C_ADDR 0x20
  MenuPanel lcd(LCD_I2C_ADDR, 16, 2);
#endif
bool lcdPresent = false;

// Menu System Variables

#define MENU_ITEMCOUNT 9
#define MENU_DRIVERCOUNT 4

uint8_t TimeToDyn[256]; // Lookup-Tabelle Zeitwert -> Dynamikwert

enum {drv_sr61, drv_fatar1, drv_fatar2, drv_custom, drv_fcktrmp};
const String Msg = {"FCK TRMP + FCK AFD"};
const String DriverTypes[] = {"Scan16/61", "FatarScan1-61", "FatarScan2", "Custom"};

enum {m_upper_channel, m_lower_channel, m_pedal_channel, m_driver_type, m_upper_base, m_lower_base, m_pedal_base, m_mindyn, m_cubicblend};
uint8_t MenuItemActive = m_upper_channel;
const String MenuItems[MENU_ITEMCOUNT] = {"Upper Channel", "Lower Channel", "Pedal Channel", "Kbd Driver", "Upper Base", "Lower Base", "Pedal Base", "Velocity Min.", "Velocity Slope"};
const int8_t MenuValueMin[MENU_ITEMCOUNT] = {1, 1, 1, 0, 12, 12, 12, 0, 0};
const int8_t MenuValueMax[MENU_ITEMCOUNT] = {16, 16, 16, MENU_DRIVERCOUNT - 1, 60, 60, 60, 40, 20};
const int8_t MenuValueDefaults[MENU_ITEMCOUNT] = {MIDI_CH_UPR, MIDI_CH_LWR, MIDI_CH_PED, drv_fatar1, MIDI_BASE_UPR, MIDI_BASE_LWR, MIDI_BASE_PED, MIDI_MINDYN, MIDI_DYNSLOPE};
int8_t MenuValues[MENU_ITEMCOUNT];


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

uint8_t lastState = B00000011; // Initialer Zustand der Encoder-Bits
int16_t encoderPosition = 0;
int16_t encoderDelta = 0;

int16_t GetEncoderDelta(uint8_t &lastState) {
  int16_t delta = 0;
  uint8_t currentState = (PINC & B00001100) >> 2; // Nur die beiden relevanten Bits lesen und nach rechts verschieben
  if (currentState != lastState) {
    // Zustandsänderung erkannt, nur ganze Schritte zählen
    if ((lastState == 0b00 && currentState == 0b10)) {
      delta = 1; // Vorwärts
    } else if ((lastState == 0b00  && currentState == 0b01)) {
      delta = -1; // Rückwärts
    }
    lastState = currentState;
  }
  return delta;
}

// #############################################################################
//
//     #     # ####### ######   #####  #######
//     ##   ## #       #     # #     # #
//     # # # # #       #     # #       #
//     #  #  # #####   ######  #  #### #####
//     #     # #       #   #   #     # #
//     #     # #       #    #  #     # #
//     #     # ####### #     #  #####  #######
//
// #############################################################################

// Simple MIDI-Merge-Funktion mit Running Status Unterstützung
// Wartet kompletten MIDI-Befehl einschließlich Datenbytes ab und
// sendet diese, bevor es zurückkehrt


uint8_t MidiDataCounter = 0;
uint8_t MdatPending = 0; // Zu erwartende Datenbytes nach Statusbyte
uint8_t LastRunningStatusSent = 0xFF; // ungültiger Wert zum Initialisieren
uint8_t LastRunningStatusReceived = 0xFF; // ungültiger Wert zum Initialisieren


bool SerialReadTimeout(uint8_t &data, unsigned long timeout_ms) {
  unsigned long startTime = millis();
  while (Serial.available() == 0) {
    if (millis() - startTime >= timeout_ms)
      return false; // Timeout
  }
  data = Serial.read();
  return true; // Daten gelesen, erfolgreich
}

void MIDIdiscardSysEx() {
  uint8_t midi_in;
  bool midi_timeout = false;
  do {
    midi_in = 0xFF;
    // midi_timeout = !SerInp_TO1(midi_in, 25);
    midi_timeout = !SerialReadTimeout(midi_in, 25);
  } while ((midi_in != 0xF7) && !midi_timeout); // SysEx beendet oder Timeout
}

bool MidiMerge() {
  uint8_t  midi_in;
  bool midi_handled, databytes_handled, midi_timeout;

  if (Serial.available()) {
    // eingehende MIDI-Daten vorhanden
    do {
      // Abwarten, bis eine eingehende MIDI-Übertragung vollständig ist.
      // Danach können wir selbst senden, ggf. auch ActiveSensing
      midi_timeout = !SerialReadTimeout(midi_in, 50);
      if (midi_in >= 0x80) {
        LastRunningStatusReceived = midi_in; // neuer Running Status, Default
        MidiDataCounter = 0;
        switch (midi_in) {
          case 0xF0: // System Exclusive ausfiltern
            MIDIdiscardSysEx();
            MdatPending = 0;
            databytes_handled = true;
            LastRunningStatusSent = 0; // Ungültig
            LastRunningStatusReceived = 0xF7;
            break;
          case 0xF4 ... 0xF7:
          case 0xF8:
          case 0xFA ... 0xFE:
            // Tune Request etc ohne Datenbyte
            MdatPending = 0;
            databytes_handled = true; // nicht benötigt
            break;
          case 0x80 ... 0x9F: // 2-Byte-Befehle, Note OFF ($8x), Note ON ($9x)
            MdatPending = 2;
            Serial.write(midi_in);  // Status senden
            LastRunningStatusSent = midi_in; // neuer Out-Status
            break;
          case 0xA0 ... 0xAF: // 2-Byte-Befehle, Polyphonic Aftertouch
            MdatPending = 2;
            Serial.write(midi_in);  // Running Status senden
            LastRunningStatusSent = midi_in; // neuer Out-Status
            break;
          case 0xB0 ... 0xBF: // 2-Byte-Befehle, CCs
            MdatPending = 2;
            Serial.write(midi_in);  // Running Status senden
            LastRunningStatusSent = midi_in; // neuer Out-Status
            break;
          case 0xC0 ... 0xCF: // 1-Byte-Befehle, Program Change
            MdatPending = 1;
            Serial.write(midi_in);  // Running Status senden
            LastRunningStatusSent = midi_in; // neuer Out-Status
            break;
          case 0xD0 ... 0xDF: // 1-Byte-Befehle, Channel Aftertouch
            MdatPending = 1;
            Serial.write(midi_in);  // Running Status senden
            LastRunningStatusSent = midi_in; // neuer Out-Status
            break;
          case 0xE0 ... 0xEF: // 2-Byte-Befehle, Pitch Bend (LSB, MSB)
            MdatPending = 2;
            Serial.write(midi_in);  // Running Status senden
            LastRunningStatusSent = midi_in; // neuer Out-Status
            break;
          case 0xF2: // 2-Byte-Befehl, Song Position Pointer
            MdatPending = 2;
            break;
          case 0xF1: case 0xF3: // 1-Byte-Befehle, MIDI Time Code, Song Select
            MdatPending = 1;
            break;
          default:
            // andere Statusbytes
            MdatPending = 0;
            break;
        }
      } else {
        // Datenbytes werden normal weitergereicht
        if (MdatPending > 0) {
          MidiDataCounter++;
          if (LastRunningStatusSent != LastRunningStatusReceived) {
            Serial.write(LastRunningStatusReceived);  // Running Status senden
            LastRunningStatusSent = LastRunningStatusReceived; // neuer Out-Status
          }
          Serial.write(midi_in);  // Datenbyte gleich wieder loswerden
          // erforderliche 1 oder 2 Datenbytes eingetroffen?
          if (MidiDataCounter % MdatPending == 0) {
            databytes_handled = true;
            MidiDataCounter = 0;
          }
        }
      }
      midi_handled = true; // MIDI war eingetroffen
      if (databytes_handled || midi_timeout) {
        break;
      }
    } while (true);
  } else {
    midi_handled = false;
  }
  return(midi_handled);
}

// #############################################################################
//
//     #     # ### ######  ###   ### #######
//     ##   ##  #  #     #  #     #  #     #
//     # # # #  #  #     #  #     #  #     #
//     #  #  #  #  #     #  #     #  #     #
//     #     #  #  #     #  #     #  #     #
//     #     #  #  #     #  #     #  #     #
//     #     # ### ######  ###   ### #######
//
// #############################################################################

void MidiSendNoteOn(uint8_t channel, uint8_t note, uint8_t timerval) {
  // Dynamikwert aus Timerwert berechnen, je kleiner der Timerwert, desto größer die Dynamik
  uint8_t cmd_byte = 0x90 + channel - 1;
  uint8_t mididyn = TimeToDyn[timerval];
  if (cmd_byte != LastRunningStatusSent) {
    Serial.write(cmd_byte);
    LastRunningStatusSent = cmd_byte;
  }
  Serial.write(note);  // Key
  Serial.write(mididyn);   // Dynamik
}

void MidiSendNoteOnNoDyn(uint8_t channel, uint8_t note) {
  uint8_t cmd_byte = 0x90 + channel - 1;
  if (cmd_byte != LastRunningStatusSent) {
    Serial.write(cmd_byte);
    LastRunningStatusSent = cmd_byte;
  }
  Serial.write(note);  // Key
  Serial.write(64);   // Dynamik
}

void MidiSendNoteOff(uint8_t channel, uint8_t note) {
  uint8_t cmd_byte = 0x80 + channel - 1;
  if (cmd_byte != LastRunningStatusSent) {
    Serial.write(cmd_byte);
    LastRunningStatusSent = cmd_byte;
  }
  Serial.write(note);  // Key
  Serial.write(64);   // Off-Dynamik
}

void MidiSendController(uint8_t channel, uint8_t cc, uint8_t value) {
  // Dynamikwert aus Timerwert berechnen, je kleiner der Timerwert, desto größer die Dynamik
  uint8_t cmd_byte = 0xB0 + channel - 1;
  if (cmd_byte != LastRunningStatusSent) {
    Serial.write(cmd_byte);
    LastRunningStatusSent = cmd_byte;
  }
  Serial.write(cc);  // Controller number
  Serial.write(value);   // Dynamik
}

void blinkLED(uint8_t times) {
  for (uint8_t i=0; i<times; i++) {
    digitalWrite(LED_PIN, LOW); // sets the LED on
    delay(150);
    digitalWrite(LED_PIN, HIGH);  // sets the LED off
    delay(150);
  }
}

void CreateDynTable(uint8_t mindyn, uint8_t slope) {
  // Erstelle inverse Lookup-Tabelle Timerwert -> Dynamikwert
  // Tastenanschlagzeit 0..255 (255 = extrem schnell)
  // mindyn: minimaler Dynamikwert 0..127
  // slope: 1 = nahezu linear, 20 = stark 1/t-ähnlich
  digitalWrite(LED_PIN, LOW); // sets the LED on
  if (slope < 1) slope = 1;
  float inv_slope = 1.0f / (float)slope;
  for (uint16_t t = 0; t <= 255; t++) {
    float norm = (float)t / 255.0f;  // normalisierter Zeitwert 0..1
    norm = (inv_slope * norm) / ((1 + inv_slope) - norm);
    norm *= (float)(127 - mindyn);
    TimeToDyn[t] = (uint8_t)norm + mindyn;
  }
  digitalWrite(LED_PIN, HIGH);  // sets the LED off
}

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
  uint8_t tval;
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
      tval = UpperKeyTimer[scankey];
      MidiSendNoteOn(MenuValues[m_upper_channel], MenuValues[m_upper_base] + scankey, tval);  // sende MIDI NoteOn mit Dynamikwert
    } else if (br) {
      // Break-Kontakt noch geschlossen, Timer dekrementieren
      tval = UpperKeyTimer[scankey];
      if (tval > 0) {
        tval--;
        UpperKeyTimer[scankey] = tval;
      } else {
        // sehr langsames Drücken oder verschmutzt, Maximalwert erreicht
        UpperKeyState[scankey] = t_pressed;
        MidiSendNoteOn(MenuValues[m_upper_channel], MenuValues[m_upper_base] + scankey, 1);  // sende MIDI NoteOn mit Dynamikwert
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
  uint8_t tval;
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
      tval = LowerKeyTimer[scankey];
      MidiSendNoteOn(MenuValues[m_lower_channel], MenuValues[m_lower_base] + scankey, tval);  // sende MIDI NoteOn mit Dynamikwert
    } else if (br) {
      // Break-Kontakt noch geschlossen, Timer dekrementieren
      tval = LowerKeyTimer[scankey];
      if (tval > 0) {
        tval--;
        LowerKeyTimer[scankey] = tval;
      } else {
        // sehr langsames Drücken oder verschmutzt, Maximalwert erreicht
        LowerKeyState[scankey] = t_pressed;
        MidiSendNoteOn(MenuValues[m_lower_channel], MenuValues[m_lower_base] + scankey, 1);  // sende MIDI NoteOn mit Dynamikwert
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
  uint8_t scankey; // aktuelle Taste
  uint8_t portb_idle = (PORTB | B00111000) & ~((1 << SR_CLK) | (1 << SR_LOAD)); // Set SR_CLK and SR_LOAD LOW for idle
  uint8_t portb_load = portb_idle | (1 << SR_LOAD);
  uint8_t portb_loadclk = portb_load | (1 << SR_CLK);
  uint8_t portb_clk = portb_idle | (1 << SR_CLK);
  uint8_t mk_ped, mk_old;

  PORTB = portb_load;
  asm volatile ("nop"); // Setup-Time für CMOS-4014
  asm volatile ("nop");
  asm volatile ("nop");
  PORTB = portb_loadclk;
  asm volatile ("nop"); // Clock-Dauer für CMOS-4014
  asm volatile ("nop");
  asm volatile ("nop");
  PORTB = portb_idle;   // zurück zu idle
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
    PORTB = portb_clk;
    asm volatile ("nop"); // Clock-Dauer für CMOS-4014
    asm volatile ("nop");
    asm volatile ("nop");
    PORTB = portb_idle;   // zurück zu idle
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
  uint8_t portd_idle = (PORTD & ~(1 << FT_CLK)) | (1 << FT_LOAD); // Set bit 3 LOW, 4 HIGH for idle
  uint8_t portd_load = portd_idle & ~(1 << FT_LOAD); // Set bit 4 LOW for load
  uint8_t portd_loadclk = portd_load | (1 << FT_CLK); // Set bit 3 HIGH for clock
  uint8_t portd_clk = portd_idle | (1 << FT_CLK); // Set bit 3 HIGH for clock
  uint8_t mk_upr, br_upr, mk_lwr, br_lwr;
  uint8_t scankey = 0; // aktuelle Taste
  PORTD = portd_idle;  // zurück zu idle
  for (uint8_t tdrive = 0; tdrive < 8; tdrive++) {
    PORTB = (PORTB & B11111000) | tdrive; // nur unterste 3 Bits, als T-Drive an Decoder
    delayMicroseconds(1); // kurze Pause, Settle time
    // SRs einer Gruppe laden, an FT_UPR und FT_LWR steht danach das erste Bit an
    PORTD = portd_load;    // FT_LOAD auf LOW
    PORTD = portd_loadclk; // FT_CLK auf HIGH
    PORTD = portd_idle;    // FT_LOAD auf HIGH, FT_CLK auf LOW
    for (uint8_t pulsecount = 0; pulsecount < KEYS_PER_GROUP; pulsecount++) {
      if (pulsecount != 0) {
        // nur nächstes Bit
        PORTD = portd_clk;  // FT_CLK auf HIGH
        PORTD = portd_idle; // FT_CLK auf LOW
      }
      // jetzt liegt das MK-Bit an FT_UPR und FT_LWR an
      // Berechnung ist auch eine kleine Pause zum Settle des Eingangspins:
      if (scankey >= KEYS) scankey = scankey - KEYS; // Modulo bis zur maximalen Tastenzahl
      // scankey = scankey & 0x3F; // ginge auch, aber nur für 61er Tastaturen
      mk_upr = PIND & (1 << FT_UPR); // Make-Kontakt Upper lesen
      mk_lwr = PIND & (1 << FT_LWR); // Make-Kontakt Lower lesen
      // BR-Bit(s) einlesen
      PORTD = portd_clk;  // FT_CLK auf HIGH
      PORTD = portd_idle; // FT_CLK auf LOW
      asm volatile ("nop"); // kleine Pause zum Settle, WICHTIG!
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
  uint8_t portd_tdrive_idle = PORTD & B00000111;
  uint8_t portd_tdrive_inc = portd_tdrive_idle | (1 << FT_TDRV_INC);
  uint8_t portd_tdrive_rst = portd_tdrive_idle | (1 << FT_TDRV_RST);
  uint8_t portb_sense_idle = PORTB & B11111000;
  uint8_t portb_sense_inc = portb_sense_idle | (1 << FT_SENSE_INC);
  uint8_t portb_sense_rst = portb_sense_idle | (1 << FT_SENSE_RST);
  uint8_t mk_upr, br_upr, mk_lwr, br_lwr;
  uint8_t scankey = 0; // aktuelle Taste
  // Reset des 4017 T-Drive Counters
  // Reset des 4024 Sense Counters
  PORTD = portd_tdrive_rst;
  PORTB = portb_sense_rst;
  asm volatile ("nop"); // Pulsdauer für Reset
  asm volatile ("nop");
  PORTD = portd_tdrive_idle;
  PORTB = portb_sense_idle;
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
      PORTB = portb_sense_inc;
      asm volatile ("nop"); // Pulsdauer für 4024 Increment
      asm volatile ("nop");
      PORTB = portb_sense_idle;
      // Settle Time nutzen zur Auswertung der Kontakte
      UpperCheckstate(scankey, mk_upr, br_upr);
      LowerCheckstate(scankey, mk_lwr, br_lwr);
      scankey += KEYS_PER_GROUP;
    }
    // Letzte Taste in der Sense-Gruppe, danach T-Drive inkrementieren
    PORTD = portd_tdrive_inc;  // FT_CLK auf HIGH
    asm volatile ("nop"); // Pulsdauer für 4017 Increment
    asm volatile ("nop");
    PORTD = portd_tdrive_idle; // FT_CLK auf LOW
    scankey++;
  }
}

// #############################################################################

void ScanManualsSR61() {
  // Manual mit 4014 SR scannen, Zeit für 61 Manualtasten etwa 81 us bei 20 MHz Takt
  uint8_t scankey; // aktuelle Taste
  uint8_t portb_idle = (PORTB | B00111000) & ~((1 << SR_CLK) | (1 << SR_LOAD)); // Set SR_CLK and SR_LOAD LOW for idle
  uint8_t portb_load = portb_idle | (1 << SR_LOAD);
  uint8_t portb_loadclk = portb_load | (1 << SR_CLK);
  uint8_t portb_clk = portb_idle | (1 << SR_CLK);
  uint8_t mk_upr, mk_upr_old;
  uint8_t mk_lwr, mk_lwr_old;

  PORTB = portb_load;
  asm volatile ("nop"); // Setup-Time für CMOS-4014
  asm volatile ("nop");
  asm volatile ("nop");
  PORTB = portb_loadclk;
  asm volatile ("nop"); // Clock-Dauer für CMOS-4014
  asm volatile ("nop");
  asm volatile ("nop");
  PORTB = portb_idle;   // zurück zu idle
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
    PORTB = portb_clk;
    asm volatile ("nop"); // Clock-Dauer für CMOS-4014
    asm volatile ("nop");
    asm volatile ("nop");
    PORTB = portb_idle;   // zurück zu idle
  }
}

// #############################################################################

void configurePorts(uint8_t driverType) {
  DDRC =  B00000000; // Encoder-Eingänge PC0 und PC1
  PORTC = B00001100; // Pull-ups für Encoder-Eingänge aktivieren
  switch (driverType) {
    case drv_fatar1:
      // FATAR Scan Controller 1 (NEU)
      DDRB =  B00000111; // PB0..PB2 als Ausgänge
      PORTB = B00111000; // Pull-ups für SR61- und BASS25-Eingänge aktivieren
      DDRD =  B00111110; // Keine Pullups an Eingängen PIND6 und PIND7!
      PORTD = B00010110; // Pull-ups für Eingänge aktivieren, FT_CLK low, LED PD2 off (high!)
      // Initialisierung der State Machines für FATAR Scan-Controller, anschlagdynamisch
      for (uint8_t i = 0; i < MANUALKEYS; i++) {
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
      for (uint8_t i = 0; i < MANUALKEYS; i++) {
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
  for (uint8_t i = 0; i < MANUALKEYS; i++) {
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

#ifdef LCD_I2C

void displayMenuValue(uint8_t itemIndex) {
  int8_t item_value = MenuValues[itemIndex];
  lcd.setCursor(0, 1);
  switch (itemIndex) {
    case m_driver_type:
      lcd.print(DriverTypes[item_value]);
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
  lcd.print(MenuItems[itemIndex]);
  lcd.clearEOL(); // Lösche evtl. alte Zeichen
  lcd.setCursor(15, 0);
  lcd.print(LCD_ARW_UD);
  displayMenuValue(itemIndex);
}

void handleMenu() {
  // Menü-Handling hier
  encoderDelta = GetEncoderDelta(lastState);
  if (encoderDelta != 0) {
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
    if ((MenuItemActive == m_mindyn ) || (MenuItemActive == m_cubicblend)) {
      CreateDynTable(MenuValues[m_mindyn], MenuValues[m_cubicblend]);
    }
  }
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

volatile uint8_t Timer1Semaphore = 0;
volatile uint8_t Timer1RoundRobin = 0;


void timer1SemaphoreISR() {
  // Timer1 Interrupt Service Routine, setzt Semaphore für Timer-basiertes Ausführen
  // der Scan- und MIDI-Merge-Funktionen im Hauptprogramm
  Timer1Semaphore++;
  Timer1RoundRobin++;
}

void setup() {
  Serial.begin(31250);
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
  MidiSendController(MenuValues[m_upper_channel], 123, 0); // All Notes Off on Channel 1
  MidiSendController(MenuValues[m_lower_channel], 123, 0); // All Notes Off on Channel 1
  MidiSendController(MenuValues[m_pedal_channel], 123, 0); // All Notes Off on Channel 1  Timer1.initialize(500); // Timer1 auf 500 us einstellen
  Timer1.attachInterrupt(timer1SemaphoreISR); // timer1SemaphoreISR to run every 0.5 milliseconds
  if (MenuValues[m_driver_type] >= drv_fatar1) {
    Timer1.setPeriod(500);  // Timer1 auf 500 us einstellen
  } else {
    Timer1.setPeriod(1000); // Timer1 auf 1000 us einstellen
  }
  CreateDynTable(MenuValues[m_mindyn], MenuValues[m_cubicblend]);

#ifdef LCD_I2C
  Wire.begin();
  Wire.setClock(400000UL);  // 400kHz
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

}

// #############################################################################

void loop() {
  // put your main code here, to run repeatedly:
  while (Timer1Semaphore) {
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
  }

#ifdef LCD_I2C
  if ((Timer1RoundRobin & 0x07) == 0) {
    if (lcdPresent) {
      handleMenu();   // benötigt etwa 130 µs für Button-Abfrage bei 400 kHz
    }
  }
#endif
}

