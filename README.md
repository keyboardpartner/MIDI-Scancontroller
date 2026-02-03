# MIDI-Scancontroller
AVR based scan controller for Fatar and SR4014 keybeds with velocity slope control and menu system (if eqipped with KeyboardPartner MenuPanel)

Project for PlatformIO and Arduino IDE

New approach for velocity table:

```javascript
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
```

C. Meyer 2/2026
