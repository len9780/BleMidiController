// SN74HC595-ESP32 WROOM
// MR-VCC
// OE-GND

#include <stdint.h>
#define SCK_GPIO_PIN 13
#define RCK_GPIO_PIN 14
#define SDA_GPIO_PIN 12

void _74hc595_init();
void HC595_Save();

void HC595_Send_Byte(uint8_t byte);

void HC595_Send_Multi_Byte(uint8_t* byte, uint16_t len);
void HC595_SCK_Low();
void HC595_SCK_High();
void HC595_RCK_Low();
void HC595_RCK_High();
void HC595_DATA_Low();
void HC595_DATA_High();

