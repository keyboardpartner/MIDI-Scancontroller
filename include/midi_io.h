
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

#ifndef MIDI_IO_H
#define MIDI_IO_H

#include <Arduino.h>

uint8_t TimeToDyn[256]; // Lookup-Tabelle Zeitwert -> Dynamikwert

void CreateDynTable(uint8_t mindyn, uint8_t slope) {
  // Erstelle inverse Lookup-Tabelle Timerwert -> Dynamikwert
  // Index: gemessene Tastenanschlagzeit 255..0 (255 = extrem schnell)
  // mindyn: minimaler Dynamikwert 0..40
  // slope: 1 = nahezu linear, 20 = stark 1/t-ähnlich
  if (slope < 1) slope = 1;
  float inv_slope = 1.0f / (float)slope;
  for (uint16_t t = 0; t <= 255; t++) {
    float norm = (float)t / 255.0f;  // normalisierter Zeitwert 0..1
    norm = (inv_slope * norm) / ((1 + inv_slope) - norm);
    norm *= (float)(127 - mindyn);
    TimeToDyn[t] = (uint8_t)norm + mindyn;
  }
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


bool  MidiReadByteTimeout(uint8_t &data, unsigned long timeout_ms) {
  unsigned long startTime = millis();
  while (Serial.available() == 0) {
    if (millis() - startTime >= timeout_ms)
      return false; // Timeout
  }
  data = Serial.read();
  return true; // Daten gelesen, erfolgreich
}

void MidiDiscardSysEx() {
  uint8_t midi_in;
  bool midi_timeout = false;
  do {
    midi_in = 0xFF;
    // midi_timeout = !SerInp_TO1(midi_in, 25);
    midi_timeout = ! MidiReadByteTimeout(midi_in, 25);
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
      midi_timeout = ! MidiReadByteTimeout(midi_in, 50);
      if (midi_in >= 0x80) {
        LastRunningStatusReceived = midi_in; // neuer Running Status, Default
        MidiDataCounter = 0;
        switch (midi_in) {
          case 0xF0: // System Exclusive ausfiltern
            MidiDiscardSysEx();
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


void MidiSendNoteOn(uint8_t channel, uint8_t note, uint8_t mididyn) {
  // Dynamikwert aus Timerwert berechnen, je kleiner der Timerwert, desto größer die Dynamik
  uint8_t cmd_byte = 0x90 + channel - 1;
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
  // CC senden
  uint8_t cmd_byte = 0xB0 + channel - 1;
  if (cmd_byte != LastRunningStatusSent) {
    Serial.write(cmd_byte);
    LastRunningStatusSent = cmd_byte;
  }
  Serial.write(cc);  // Controller number
  Serial.write(value & 0x7F);   // Wert
}

void MidiSendProgramChange(uint8_t channel, uint8_t value) {
  // Program Change senden
  uint8_t cmd_byte = 0xC0 + channel - 1;
  if (cmd_byte != LastRunningStatusSent) {
    Serial.write(cmd_byte);
    LastRunningStatusSent = cmd_byte;
  }
  Serial.write(value & 0x7F);   // Programm number
}

void MidiSendPitchBend(uint8_t channel, int16_t value) {
  // Pitch Bend senden, Wert -8192..8191 auf 0..16383 mappen
  uint16_t bend_value = (uint16_t)(value + 8192);
  uint8_t cmd_byte = 0xE0 + channel - 1;
  if (cmd_byte != LastRunningStatusSent) {
    Serial.write(cmd_byte);
    LastRunningStatusSent = cmd_byte;
  }
  Serial.write(bend_value & 0x7F);   // LSB
  Serial.write((bend_value >> 7) & 0x7F);   // MSB
}

#endif