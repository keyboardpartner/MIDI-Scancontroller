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

#ifndef MPX_POTS_H
#define MPX_POTS_H

#include <Arduino.h>
#include "global_vars.h"

// MPX Functions for reading analog inputs via MPX and 74HC164 shift register

#ifdef HX35_BOARD
  #define ANLG_MPX // MPX-gestützte analoge Eingänge für MIDI-CC-Potis aktivieren, wenn HX3.5 Board definiert ist
  // MPX Port Definitions and Fast Manipulation Macros
  #define MPX_DATA PORTD4
  #define MPX_CLK  PORTD5
  // Fast port bit manipulation macros
  #define _SET_MPX_DATA asm volatile("sbi %0,%1 " : : "I" (_SFR_IO_ADDR(PORTD)), "I" (MPX_DATA))
  #define _CLR_MPX_DATA asm volatile("cbi %0,%1 " : : "I" (_SFR_IO_ADDR(PORTD)), "I" (MPX_DATA))
  #define _SET_MPX_CLK  asm volatile("sbi %0,%1 " : : "I" (_SFR_IO_ADDR(PORTD)), "I" (MPX_CLK))
  #define _CLR_MPX_CLK  asm volatile("cbi %0,%1 " : : "I" (_SFR_IO_ADDR(PORTD)), "I" (MPX_CLK))
#else
  // MPX Port Definitions and Fast Manipulation Macros
  #define MPX_DATA PORTC0
  #define MPX_CLK  PORTC1
  // Fast port bit manipulation macros
  #define _SET_MPX_DATA asm volatile("sbi %0,%1 " : : "I" (_SFR_IO_ADDR(PORTC)), "I" (MPX_DATA))
  #define _CLR_MPX_DATA asm volatile("cbi %0,%1 " : : "I" (_SFR_IO_ADDR(PORTC)), "I" (MPX_DATA))
  #define _SET_MPX_CLK  asm volatile("sbi %0,%1 " : : "I" (_SFR_IO_ADDR(PORTC)), "I" (MPX_CLK))
  #define _CLR_MPX_CLK  asm volatile("cbi %0,%1 " : : "I" (_SFR_IO_ADDR(PORTC)), "I" (MPX_CLK))
#endif

#define MPX_ACTIVE_TIMEOUT 10 // Anzahl der kompletten Durchläufe, die ein geänderter Potentiometerwert als aktiv gilt, höher = länger aktiv
#define MPX_INTEGRATOR_FACTOR 4 // Faktor für die Integration der MPX-Werte, höher = stärker geglättet, aber auch träger


// Pro MPX-Eingang werden 5 Byte benötigt: integrierter Wert, alter Wert, Timer für Aktivität, CC-Nummer, CC-Kanal
// RAM kann bei ATmega328P knapp sein, daher hier die Anzahl der MPX-Eingänge auf 16 begrenzt

class MPXpots {

public:
  MPXpots(uint8_t mpx_input_count = ANLG_INPUTS, uint8_t active_timeout = MPX_ACTIVE_TIMEOUT, int16_t integrator_factor = MPX_INTEGRATOR_FACTOR);
  void init(uint8_t mpx_input_count = ANLG_INPUTS, uint8_t active_timeout = MPX_ACTIVE_TIMEOUT, int16_t integrator_factor = MPX_INTEGRATOR_FACTOR);
  void handleMPX(); // in main loop aufrufen, um MPX-Werte zu lesen und MIDI-CCs zu senden
  void resetMPX();

  typedef void (*actionCallback)(uint8_t inputIndex, uint8_t value);
  void setChangeAction(actionCallback action) { _changeAction = action; }

  uint8_t getAnalogValue(uint8_t index) {
    // aktueller Wert eines analogen Eingangs, 0..127, erst nach einigen handleMPX() gültig, da dort die Werte gelesen und verarbeitet werden
    return (index < ANLG_INPUTS) ? _analogInputs[index] : 0;
  }

private:
  void clockMPX();
  void startMPX();

  // Callback bei Änderungen
  static void dummyAction(uint8_t inputIndex, uint8_t value) { }; // In case user callback is not defined!
  actionCallback _changeAction; // Analog Changed action callback

  uint8_t _analogInputsIntegrated[ANLG_INPUTS]; // Geglättete/integrierte Werte der analogen Eingänge
  uint8_t _analogInputsOld[ANLG_INPUTS]; // Aktuelle Werte der analogen Eingänge
  uint8_t _analogInputTimers[ANLG_INPUTS]; // Zählt nach Änderung des integrierten Werts auf 0 herunter, um bei längerem Stillstand der Potis die Integration zurückzusetzen
  uint8_t _analogInputs[ANLG_INPUTS]; // Aktuelle Werte der analogen Eingänge, 0..127, werden nach Integration und Mapping auf MIDI-Bereich in handleMPX() aktualisiert

  uint8_t _analogMPXinputCount = ANLG_INPUTS; // Anzahl der tatsächlich genutzten analogen Eingänge, 0..ANLG_INPUTS, kann über setMPXinputCount() angepasst werden
  uint8_t _analogMPXactiveTimeout = MPX_ACTIVE_TIMEOUT;
  int16_t _analogMPXintegrator = MPX_INTEGRATOR_FACTOR;
  uint8_t _analogInputSelect = 0; // derzeit ausgewählter analoger Eingang, wird intern hochgezählt, 0.._analogMPXinputCount-1
};

MPXpots::MPXpots(uint8_t mpx_input_count, uint8_t active_timeout, int16_t integrator_factor) {
  init(mpx_input_count, active_timeout, integrator_factor);
}

void MPXpots::init(uint8_t mpx_input_count, uint8_t active_timeout, int16_t integrator_factor) {
  _changeAction = dummyAction; // Pressed action callback auf default setzen, damit er nicht ins Leere läuft, falls er nicht definiert ist
  _analogMPXinputCount = mpx_input_count;
  _analogMPXactiveTimeout = active_timeout;
  _analogMPXintegrator = integrator_factor;
  for (uint8_t i = 0; i < ANLG_INPUTS; i++) {
    _analogInputsIntegrated[i] = 0;
    _analogInputsOld[i] = 0;
    _analogInputTimers[i] = 0;
  }
  resetMPX();
 }


void MPXpots::clockMPX() {
  // MPX Data PC0 und MPX-Clk PC1 als Ausgänge
  // PORTC = mpx_clk; // Clock auf HIGH, Datenbit wird übernommen
  _SET_MPX_CLK;
  _NOP_DLY; // kurze Pause für Clock-Dauer
  _CLR_MPX_CLK;
  // Am _analogeingang ADC7 liegt jetzt der Wert des nächsten MPX-Potentiometers an
}

void MPXpots::startMPX() {
  // MPX Data PC0 und MPX-Clk PC1 als Ausgänge
  // Schiebe eine 1 in das Schieberegister 74HC164
  _SET_MPX_DATA; // Data PORTC Bit 0 auf HIGH
  _SET_MPX_CLK; // Clk PORTC Bit 1 auf HIGH
  _NOP_DLY; // kurze Pause für Clock-Dauer
  _CLR_MPX_CLK;
  _CLR_MPX_DATA;
  _NOP_DLY; // kurze Pause zum Settle des ADC-Eingangs
  _NOP_DLY;
  // Am _analogeingang ADC7 liegt jetzt der Wert des ersten MPX-Potentiometers an
}

void MPXpots::resetMPX() {
  // MPX Data PC0 und MPX-Clk PC1 als Ausgänge, einmal in setup() aufrufen
  ADMUX = (1<<REFS0)|(1<<ADLAR)|(0x07); //select AVCC as reference and set MUX to pin 7
  ADCSRA = (1<<ADEN)|(1<<ADPS1)|(1<<ADPS0); // enable ADC and set prescaler to 8
  ADCSRA |= (1 << ADSC); // Starte erste Wandlung
  _CLR_MPX_DATA; // LOWs einschieben, kein Poti aktiv
  for (uint8_t i = 0; i < _analogMPXinputCount; i++) {
    // Zeit zur ersten Wandlung nutzen
    clockMPX();
  }
  while (ADCSRA & (1 << ADSC)); // warte bis erste Wandlung abgeschlossen
  startMPX(); // erste 1 schieben
  _analogInputSelect = 0;
}

void MPXpots::handleMPX() {
  // Muss regelmäßig aufgerufen werden, um die _analogen Eingänge zu scannen und MIDI-CC-Updates zu senden, wenn sich Werte geändert haben
  // Read _analog input ADC7 and shift next MPX bit, send CC if value changed
  // Bei Prescaler 8 und 20 MHz Takt ist die ADC-Wandlung nach etwa 13 ADC-Takten (etwa 5 µs) abgeschlossen
  ADCSRA |= (1 << ADSC); // Starte nächste Wandlung
  // Wandlungszeit nutzen
  int16_t oldValue = (int16_t)_analogInputsIntegrated[_analogInputSelect] * (_analogMPXintegrator - 1); // Integration mit Faktor 4, alter Wert wird mit 3 gewichtet;
  uint8_t timerValue = _analogInputTimers[_analogInputSelect];
  bool isActive = false;
  if (timerValue > 0) {
    timerValue--;
    _analogInputTimers[_analogInputSelect] = timerValue;
    isActive = true;
    digitalWrite(LED_PIN, LOW);  // Timer läuft, Potentiometer aktiv
  } else {
    // Prüfe, ob noch Potis aktiv sind, wenn nicht, LED ausschalten
    bool anyActive = false;
    for (uint8_t i = 0; i < _analogMPXinputCount; i++) {
      if (_analogInputTimers[i] > 0) anyActive = true;
    }
    if (!anyActive) {
      digitalWrite(LED_PIN, HIGH);  // Timer abgelaufen, kein Potentiometer aktiv
    }
  }
  while (ADCSRA & (1 << ADSC)); // warte bis Wandlung abgeschlossen
  // ADC-Rohwert integrieren, um Rauschen zu reduzieren, Integration mit Faktor 4,
  // neuer Wert wird mit 1 gewichtet, alter Wert mit 3 gewichtett, damit schnelle Änderungen trotzdem relativ schnell durchkommen
  int16_t newValue = (int16_t)ADCH;
  newValue =  (newValue + oldValue) / _analogMPXintegrator; // skalierten Wert im Array speichern
  _analogInputsIntegrated[_analogInputSelect] = (uint8_t)newValue;

  // Hysterese: MIDI-CC-Update nur bei Änderung des integrierten Werts, um Rauschen zu reduzieren und unnötige MIDI-Updates zu vermeiden
  oldValue = _analogInputsOld[_analogInputSelect];
  if (abs(newValue - oldValue) > 1)  {
    isActive = true; // Wert hat sich geändert, Potentiometer ist aktiv
    _analogInputTimers[_analogInputSelect] = _analogMPXactiveTimeout;
  }
  if (isActive) { // Hysterese von 2 (0..255) für _analoge Eingänge, falls nicht aktiv
    _analogInputsOld[_analogInputSelect] = (uint8_t)newValue;
    // ADC-Wert 77 % von 0..255, um bei 5V AVR-Referenzspannung
    // und 3,3V Versorgungsspannung der Potis den vollen MIDI-Wertebereich 0..127 abzudecken
    newValue = (newValue * 77) / 100;
    if (newValue > 127) newValue = 127; // Überlauf verhindern

    // Change Action Callback aufrufen, damit Nutzer eigene Aktionen bei Änderungen definieren kann,
    // z.B. Anzeige auf LCD aktualisieren oder MIDI-CC senden
    if ((uint8_t)newValue != _analogInputs[_analogInputSelect]) {
      _changeAction(_analogInputSelect, (uint8_t)newValue);
      _analogInputs[_analogInputSelect] = (uint8_t)newValue; // integrierten Wert in aktuelles Array schreiben
    }
  }
  clockMPX(); // nächsten MPX-Wert vorbereiten
  _analogInputSelect++;
  if (_analogInputSelect >= _analogMPXinputCount) {
    _analogInputSelect = 0;
    startMPX(); // MPX zurücksetzen, damit der nächste Durchlauf wieder mit dem ersten MPX-Potentiometer beginnt
  }
}

#endif