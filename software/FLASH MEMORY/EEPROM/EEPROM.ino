#include <EEPROM.h>

#define DATA_SIZE 30
#define EEPROM_SIZE 512

byte value;

uint16_t data[DATA_SIZE];

void setup() {
  Serial.begin(115200);
  EEPROM.begin(512);
  delay(3000);

  for (int i=0; i<DATA_SIZE; i++) {
    data[i] = i%2;
  }
  read_eeprom();
  write_eeprom();
  read_eeprom();
}

void read_eeprom() {
  for(int addr=0; addr<EEPROM_SIZE; addr++) {
    Serial.print(addr);
    Serial.print(": ");
    Serial.println(EEPROM.read(addr));
  }
}

void write_eeprom() {
  for(int addr=0; addr<DATA_SIZE; addr++) {
    EEPROM.write(addr, data[addr]);
  }
  if (EEPROM.commit()) {
    Serial.println("Wrote to EEPROM successfully");
  } else {
    Serial.println("Error writting to EEPROM!");
  }
}

void loop() {
}
