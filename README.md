# MIDI Scan Controller

![Controller Pic](https://github.com/keyboardpartner/MIDI-Scancontroller/blob/main/docs/ScanFatarContr_02_small.jpg)

### Project for PlatformIO and Arduino IDE

AVR based scan controller for Fatar and SR4014 keybeds with velocity slope control and menu system (if equipped with KeyboardPartner MenuPanel). Achieves min. 2 kHz scan rate, scanning of 2 keybeds (upper and lower manual) takes approx. 260 µs plus 80 µs for 25-not pedal.

Designed for KeyboardPartner Scan61, FatarScan2 or new FatarScan1-61 interface boards, see schematics in /docs.



New approach for velocity table:

```cpp
void CreateDynTable(uint8_t mindyn, uint8_t slope) {
  // Erstelle inverse Lookup-Tabelle Timerwert -> Dynamikwert
  // Index: gemessene Tastenanschlagzeit 255..0 (255 = extrem schnell)
  // mindyn: minimaler Dynamikwert 0..40
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
```

C. Meyer 2/2026
