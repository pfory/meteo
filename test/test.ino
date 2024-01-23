#include <avr/dtostrf.h>

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  char temp[200];
  float temperature = -24.625;
  float humidity=96.5;
  float pressure=102378;
  float srazkyOdPulnoci=1.4;

  
  //|11.09.11|21:17|22.9|69|1010.2|1.3|207|0.0|2.1|0.0|, 
  //|datum|čas|teplota|vlhkost|tlak (přepočtený na hladinu moře, relativní)|rychlost větru v m/s (maximální náraz za 30 minut)
  //|směr ve stupních|dnešní srážky (od půlnoci)|průměrná rychlost větru za 10 minut v m/s|aktuální intenzita deště v mm/h).
  
  snprintf(temp, 200, "<html><head><meta charset='UTF-8'></head><body>\
            |%2.1f|%3.1f|%4.2f|%3.1f|</body></html>",
            temperature,
            humidity,
            pressure / 100,
            srazkyOdPulnoci
  );
  Serial.print(temp);
}

void loop()
{}