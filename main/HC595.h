// SN74HC595-ESP32 WROOM
// MR-VCC
// OE-GND

#include <stdint.h>
//#define SCK_GPIO_PIN 27  // SHCP
//#define RCK_GPIO_PIN 25  // STCP
//#define SDA_GPIO_PIN 26  // DS
#define SCK_GPIO_PIN 13  // SHCP
#define RCK_GPIO_PIN 14  // STCP
#define SDA_GPIO_PIN 12  // DS

#define row1Pin_In 33
#define row2Pin_In 32
#define row3Pin_In 15
#define row4Pin_In 23
#define row5Pin_In 22
#define row6Pin_In 33

typedef struct HC595_ctrl_pin_set {
  int ic_id;
  int SCK;
  int RCK;
  int SDA;
  struct HC595_ctrl_pin_set *next;
} HC595_ctrl_pin_set;

void _74hc595_init();
void HC595_Save();

void HC595_Send_Byte(HC595_ctrl_pin_set *hc595, uint8_t byte, uint8_t dir);

void HC595_Send_Multi_Byte(uint8_t* byte, uint16_t len);
void HC595_SCK_Low(HC595_ctrl_pin_set *pin);
void HC595_SCK_High(HC595_ctrl_pin_set *pin);
void HC595_RCK_Low(HC595_ctrl_pin_set *pin);
void HC595_RCK_High(HC595_ctrl_pin_set *pin);
void HC595_DATA_Low(HC595_ctrl_pin_set *pin);
void HC595_DATA_High(HC595_ctrl_pin_set *pin);
void midi_init();
void HC595_pin_register(HC595_ctrl_pin_set pin_set,HC595_ctrl_pin_set**record);
void dump_HC595_pin_list(HC595_ctrl_pin_set * pin);
void gpio_pin_test(HC595_ctrl_pin_set*pin);
