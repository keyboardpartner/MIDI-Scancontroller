# MIDI Scan Controller

![Controller Pic](https://github.com/keyboardpartner/MIDI-Scancontroller/blob/main/docs/ScanFatarContr_02_small.jpg)

### ATmega328 Project for PlatformIO or Arduino IDE

Fast AVR ATmega328 based scan controller with velocity dynamic scanning for FATAR and PULSE keybeds or non-dynamic scanning of single-contact keybeds with MOS 4014 shift registers. Comes with velocity slope control and menu system (if equipped with KeyboardPartner MenuPanel). Achieves min. 2 kHz scan rate, scanning of 2 keybeds (upper and lower manual) takes approx. 280 µs plus 80 µs for 25-note bass pedal.

Designed for KeyboardPartner Scan61, FatarScan2 or new FatarScan1-61 and PulseScan interface boards or DIY equivalents, see schematics in */docs* and on http://updates.keyboardpartner.de/Files/index.php?dir=Schematics%20%28Schaltbilder%29.


In case of Arduino IDE, you may have to copy *include* and *lib* directory contects into sketch directory. Anyway, we **highly recommend** using PlatformIO with VSCode for development!

**Version Info:**
 * New approach for velocity table with variable slope:

```cpp
void CreateDynTable(uint8_t mindyn, uint8_t slope) {
  // Create lookup table, converts key timer value -> MIDI dynamic
  // Table index: key velocity timer 0..255 (255 = extremely fast key press)
  // mindyn: minimal MIDI dynamic value, 1..40
  // slope: 1 = nearly linear, 20 = strong 1/t chracteristic
  if (slope < 1) slope = 1;
  float inv_slope = 1.0f / (float)slope;
  for (uint16_t t = 0; t <= 255; t++) {
    float norm = (float)t / 255.0f;  // normalisierter Zeitwert 0..1
    norm = (inv_slope * norm) / ((1 + inv_slope) - norm);
    norm *= (float)(127 - mindyn);
    TimeToDyn[t] = (uint8_t)norm + mindyn;
  }
}
```

 * Added support for analog inputs with KeyboardPartner MPX boards (SR-based multiplexer)
 * Faster bit manipulation on ports by inline assembler macros
 * Menu Strings moved to progmem, editable CC numbers
 * Improved support for Panel16 LED Button board, some examples in *main.cpp*


C. Meyer 2/2026
