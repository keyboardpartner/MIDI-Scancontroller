#ifndef global_vars_h
#define global_vars_h

// #############################################################################
//
//      #####  #       ####### ######     #    #        #####  
//     #     # #       #     # #     #   # #   #       #     # 
//     #       #       #     # #     #  #   #  #       #       
//     #  #### #       #     # ######  #     # #        #####  
//     #     # #       #     # #     # ####### #             # 
//     #     # #       #     # #     # #     # #       #     # 
//      #####  ####### ####### ######  #     # #######  #####  
//                                                             
// #############################################################################

#include <Arduino.h>
#include "MenuPanel.h"

#define VERSION "ScanCtrl v0.10"
#define CREATOR "C.Meyer 2/2026"

#define FIRMWARE_VERSION 0x02 // Vergleichswert für EEPROM, um veraltete Versionen zu erkennen
#define PRESET_VERSION 60 
#define EEPROM_VERSION_IDX 0x08 // Adresse des Vergleichwerts
#define EEPROM_MENUDEF_IDX 0x10 // Startadresse im EEPROM für gespeicherte Werte#define VERSION "ScanCtrl 0.9"

#define LED_PIN 2 // Pin für LED

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

// Fast port bit manipulation macros, Test Pin für Debugging, z.B. mit Oszilloskop
#define _SET_TEST  asm volatile("sbi %0,%1 " : : "I" (_SFR_IO_ADDR(PORTD)), "I" (FT_TEST))
#define _CLR_TEST  asm volatile("cbi %0,%1 " : : "I" (_SFR_IO_ADDR(PORTD)), "I" (FT_TEST))

// Fast port bit manipulation macros
#define _SET_FT_CLK   asm volatile("sbi %0,%1 " : : "I" (_SFR_IO_ADDR(PORTD)), "I" (FT_CLK))
#define _CLR_FT_CLK   asm volatile("cbi %0,%1 " : : "I" (_SFR_IO_ADDR(PORTD)), "I" (FT_CLK))
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
#define _SET_FT_TDRV_INC   asm volatile("sbi %0,%1 " : : "I" (_SFR_IO_ADDR(PORTD)), "I" (FT_TDRV_INC))
#define _CLR_FT_TDRV_INC   asm volatile("cbi %0,%1 " : : "I" (_SFR_IO_ADDR(PORTD)), "I" (FT_TDRV_INC))
#define _SET_FT_TDRV_RST   asm volatile("sbi %0,%1 " : : "I" (_SFR_IO_ADDR(PORTD)), "I" (FT_TDRV_RST))
#define _CLR_FT_TDRV_RST   asm volatile("cbi %0,%1 " : : "I" (_SFR_IO_ADDR(PORTD)), "I" (FT_TDRV_RST))

// BASS und SR61 4014 Scan-Controller Pinbelegung
#define SR_CLK   PORTB0 // auf Prototyp V01 ändern!
#define SR_LOAD  PORTB1 // auf Prototyp V01 ändern!
#define SR_UPR   PINB3
#define SR_LWR   PINB4
#define SR_PED   PINB5
// Fast port bit manipulation Macros
#define _SET_SR_CLK   asm volatile("sbi %0,%1 " : : "I" (_SFR_IO_ADDR(PORTB)), "I" (SR_CLK))
#define _CLR_SR_CLK   asm volatile("cbi %0,%1 " : : "I" (_SFR_IO_ADDR(PORTB)), "I" (SR_CLK))
#define _SET_SR_LOAD  asm volatile("sbi %0,%1 " : : "I" (_SFR_IO_ADDR(PORTB)), "I" (SR_LOAD))
#define _CLR_SR_LOAD  asm volatile("cbi %0,%1 " : : "I" (_SFR_IO_ADDR(PORTB)), "I" (SR_LOAD))

#define _NOP_DLY asm volatile ("nop")

#define KEYS_PER_GROUP 8
#define KEYS (KEYS_PER_GROUP * 8) // 8 Treibergruppen à 8 Tasten = 64 Tasten pro Manual

#define TIMER_MAX 255
#define TIMER_DIV ((TIMER_MAX * 2) + MIDI_MINDYN)

#define PEDALKEYS 25
#define MANUALKEYS 61

#define MIDI_MINDYN 10
#define MIDI_DYNSLOPE 12
#define MIDI_MAXDYNADJ 5

// ATMEL ATMEGA644P

#define LED_PIN PIN_PD2  // Pin für LED
#define PWR_GOOD PIN_PD7 // Pin für DSP-Reset

#define _NOP_DLY asm volatile ("nop")

#ifdef DEBUG
#define DPRINT(...)    Serial.print(__VA_ARGS__)
//OR, #define DPRINT(args...)    Serial.print(args)
#define DPRINTLN(...)  Serial.println(__VA_ARGS__)
#define DPRINTF(...)    Serial.print(F(__VA_ARGS__))
#define DPRINTLNF(...) Serial.println(F(__VA_ARGS__)) //printing text using the F macro
#else
#define DPRINT(...)     //blank line
#define DPRINTLN(...)   //blank line
#define DPRINTF(...)    //blank line
#define DPRINTLNF(...)  //blank line
#define DBEGIN(...)     //blank line
#endif


// Default MIDI Einstellungen
#define MIDI_BASE_UPR 36
#define MIDI_BASE_LWR 36
#define MIDI_BASE_PED 36

#define MIDI_CH_UPR 1
#define MIDI_CH_LWR 2
#define MIDI_CH_PED 3

MenuPanel lcd(LCD_I2C_ADDR, 16, 2);

// #############################################################################
//
//    ########   #######     ###    ########  ########  
//    ##     ## ##     ##   ## ##   ##     ## ##     ## 
//    ##     ## ##     ##  ##   ##  ##     ## ##     ## 
//    ########  ##     ## ##     ## ########  ##     ## 
//    ##     ## ##     ## ######### ##   ##   ##     ## 
//    ##     ## ##     ## ##     ## ##    ##  ##     ## 
//    ########   #######  ##     ## ##     ## ########  
//
// #############################################################################

#ifndef LCD_I2C

uint8_t MenuValues[100];

void initMenuValues() {
  // Initialisiere MenuValues mit Default-Werten, die in der Excel-Tabelle definiert sind
  for (uint8_t i = 0; i < 100; i++) {
    MenuValues[i] = 0; // wenn Min-Wert definiert ist, nimm diesen als Default, sonst 0
  }
}

#endif

struct {
  int8_t keyOffset = 0;
  int8_t prePulses = 0;
  int8_t keysPerGroup = KEYS_PER_GROUP; // noch nicht in Benutzung, da bei allen Treibern 8 Tasten pro Gruppe, aber könnte für zukünftige Erweiterungen nützlich sein
  int8_t keys = KEYS;
} scanParams;
#endif