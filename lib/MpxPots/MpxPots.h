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
#include "midi_io.h"

#define LED_PIN 2 // Pin für LED

// MPX Functions for reading _analog inputs via MPX and 74HC164 shift register

// MPX Port Definitions and Fast Manipulation Macros
#define MPX_DATA PORTC0
#define MPX_CLK  PORTC1
// Fast port bit manipulation macros
#define _SET_MPX_DATA asm volatile("sbi %0,%1 " : : "I" (_SFR_IO_ADDR(PORTC)), "I" (MPX_DATA))
#define _CLR_MPX_DATA asm volatile("cbi %0,%1 " : : "I" (_SFR_IO_ADDR(PORTC)), "I" (MPX_DATA))
#define _SET_MPX_CLK  asm volatile("sbi %0,%1 " : : "I" (_SFR_IO_ADDR(PORTC)), "I" (MPX_CLK))
#define _CLR_MPX_CLK  asm volatile("cbi %0,%1 " : : "I" (_SFR_IO_ADDR(PORTC)), "I" (MPX_CLK))

#define _NOP_MPX asm volatile ("nop")

#define MPX_ACTIVE_TIMEOUT 10 // Anzahl der kompletten Durchläufe, die ein geänderter Potentiometerwert als aktiv gilt, höher = länger aktiv
#define MPX_INTEGRATOR_FACTOR 4 // Faktor für die Integration der MPX-Werte, höher = stärker geglättet, aber auch träger

#ifndef ANLG_INPUTS
  #define ANLG_INPUTS 4
#endif
#define ANLG_INPUTS_MAX 16

class MPXpots {
public:
  MPXpots(uint8_t mpx_input_count = ANLG_INPUTS, uint8_t active_timeout = MPX_ACTIVE_TIMEOUT, int16_t integrator_factor = MPX_INTEGRATOR_FACTOR);
  void init(uint8_t mpx_input_count = ANLG_INPUTS, uint8_t active_timeout = MPX_ACTIVE_TIMEOUT, int16_t integrator_factor = MPX_INTEGRATOR_FACTOR);
  void handleMPX(); // in main loop aufrufen, um MPX-Werte zu lesen und MIDI-CCs zu senden
  void setMPXccNumber(uint8_t inputIndex, uint8_t ccNumber);
  void setMPXccChannel(uint8_t inputIndex, uint8_t ccChannel);
  void resetMPX();

private:
  void clockMPX();
  void startMPX();

  // uint8_t _analogInputs[ANLG_INPUTS]; // Aktuelle Werte der _analogen Eingänge, hier nicht benötigt, da direkt in handleMPX gelesen und verarbeitet wird
  uint8_t _analogInputsIntegrated[ANLG_INPUTS_MAX]; // Geglättete/integrierte Werte der _analogen Eingänge
  uint8_t _analogInputsOld[ANLG_INPUTS_MAX]; // Aktuelle Werte der _analogen Eingänge
  uint8_t _analogInputTimer[ANLG_INPUTS_MAX]; // Zählt nach Änderung des integrierten Werts auf 0 herunter, um bei längerem Stillstand der Potis die Integration zurückzusetzen
  uint8_t _analogCCnumbers[ANLG_INPUTS_MAX] = {7, 10, 11, 91}; // MIDI CC-Nummern für die _analogen Eingänge (z.B. Volume, Pan, Expression, Reverb Depth)
  uint8_t _analogCCchannels[ANLG_INPUTS_MAX];   // MIDI CC-Kanäle für die _analogen Eingänge (z.B. Volume, Pan, Expression, Reverb Depth)

  uint8_t _analogMPXinputCount = ANLG_INPUTS; // Anzahl der tatsächlich genutzten _analogen Eingänge, 0..ANLG_INPUTS, kann über setMPXinputCount() angepasst werden
  uint8_t _analogMPXactiveTimeout = MPX_ACTIVE_TIMEOUT; // Anzahl der tatsächlich genutzten _analogen Eingänge, 0..ANLG_INPUTS, kann über setMPXinputCount() angepasst werden
  int16_t _analogMPXintegrator = MPX_INTEGRATOR_FACTOR; // Anzahl der tatsächlich genutzten _analogen Eingänge, 0..ANLG_INPUTS, kann über setMPXinputCount() angepasst werden
  uint8_t _analogInputSelect = 0; // derzeit ausgewählter _analoger Eingang, wird intern hochgezählt, 0.._analogMPXinputCount-1
};

MPXpots::MPXpots(uint8_t mpx_input_count = ANLG_INPUTS, uint8_t active_timeout = MPX_ACTIVE_TIMEOUT, int16_t integrator_factor = MPX_INTEGRATOR_FACTOR) {
  init(mpx_input_count, active_timeout, integrator_factor);
}

void MPXpots::init(uint8_t mpx_input_count = ANLG_INPUTS, uint8_t active_timeout = MPX_ACTIVE_TIMEOUT, int16_t integrator_factor = MPX_INTEGRATOR_FACTOR) {
  _analogMPXinputCount = mpx_input_count;
  _analogMPXactiveTimeout = active_timeout;
  _analogMPXintegrator = integrator_factor;
  for (uint8_t i = 0; i < _analogMPXinputCount; i++) {
    _analogInputsIntegrated[i] = 0;
    _analogInputsOld[i] = 0;
    _analogInputTimer[i] = 0;
    _analogCCchannels[i] = 1; // default MIDI-Kanal 1
    if (i >= 4)  _analogCCnumbers[i] = 7; // default MIDI CC 7 (Volume) für weitere Potis über 4
  }
  resetMPX();
}


void MPXpots::setMPXccNumber(uint8_t inputIndex, uint8_t ccNumber) {
  if (inputIndex < ANLG_INPUTS_MAX && ccNumber <= 127) {
    _analogCCnumbers[inputIndex] = ccNumber;
  }
}

void MPXpots::setMPXccChannel(uint8_t inputIndex, uint8_t ccChannel) {
  if (inputIndex < ANLG_INPUTS_MAX && ccChannel <= 15) {
    _analogCCchannels[inputIndex] = ccChannel;
  }
}

void MPXpots::clockMPX() {
  // MPX Data PC0 und MPX-Clk PC1 als Ausgänge
  // PORTC = mpx_clk; // Clock auf HIGH, Datenbit wird übernommen
  _SET_MPX_CLK;
  _NOP_MPX; // kurze Pause für Clock-Dauer
  _CLR_MPX_CLK;
  // Am _analogeingang ADC7 liegt jetzt der Wert des nächsten MPX-Potentiometers an
}

void MPXpots::startMPX() {
  // MPX Data PC0 und MPX-Clk PC1 als Ausgänge
  // Schiebe eine 1 in das Schieberegister 74HC164
  _SET_MPX_DATA; // Data PORTC Bit 0 auf HIGH
  _SET_MPX_CLK; // Clk PORTC Bit 1 auf HIGH
  _NOP_MPX; // kurze Pause für Clock-Dauer
  _CLR_MPX_CLK;
  _CLR_MPX_DATA;
  _NOP_MPX; // kurze Pause zum Settle des ADC-Eingangs
  _NOP_MPX;
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
  uint8_t timerValue = _analogInputTimer[_analogInputSelect];
  bool isActive = false;
  if (timerValue > 0) {
    timerValue--;
    _analogInputTimer[_analogInputSelect] = timerValue;
    isActive = true;
    digitalWrite(LED_PIN, LOW);  // sets the LED on
  } else {
    // Prüfe, ob noch Potis aktiv sind, wenn nicht, LED ausschalten
    bool anyActive = false;
    for (uint8_t i = 0; i < _analogMPXinputCount; i++) {
      if (_analogInputTimer[i] > 0) anyActive = true;
    }
    if (!anyActive) {
      digitalWrite(LED_PIN, HIGH);  // sets the LED off
    }
    // Timer abgelaufen, Integration zurücksetzen, damit schnelle Änderungen der Potis trotzdem relativ schnell durchkommen
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
    _analogInputTimer[_analogInputSelect] = _analogMPXactiveTimeout;
  }
  if (isActive) { // Hysterese von 2 (0..255) für _analoge Eingänge, falls nicht aktiv
    _analogInputsOld[_analogInputSelect] = (uint8_t)newValue;
    // ADC-Wert 77 % von 0..255, um bei 5V AVR-Referenzspannung
    // und 3,3V Versorgungsspannung der Potis den vollen MIDI-Wertebereich 0..127 abzudecken
    newValue = (newValue * 77) / 100;
    if (newValue > 127) newValue = 127; // Überlauf verhindern
    MidiSendController(_analogCCchannels[_analogInputSelect], _analogCCnumbers[_analogInputSelect], newValue); // Volume Upper
    // _analogInputs[_analogInputSelect] = (uint8_t)newValue; // integrierten Wert in aktuelles Array schreiben
  }
  clockMPX(); // nächsten MPX-Wert vorbereiten
  _analogInputSelect++;
  if (_analogInputSelect >= _analogMPXinputCount) {
    _analogInputSelect = 0;
    startMPX(); // MPX zurücksetzen, damit der nächste Durchlauf wieder mit dem ersten MPX-Potentiometer beginnt
  }
}

#endif