#include "HC595.h"
#include <inttypes.h>
#include <stdint.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"

#define row1Pin_In 18
#define row2Pin_In 19
#define row3Pin_In 20
#define row4Pin_In 21
#define row5Pin_In 22
#define row6Pin_In 23
#define GATTS_TABLE_TAG ""
#define ESP_INTR_FLAG_DEFAULT 0
static QueueHandle_t gpio_evt_queue = NULL;
static void input_sensor_thread(void* arg) {
  uint32_t io_num;
  for (;;) {
    if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
      printf("GPIO[%" PRIu32 "] intr, val: %d\n", io_num,
             gpio_get_level(io_num));
    }
  }
}
static void IRAM_ATTR gpio_isr_handler(void* arg) {
  uint32_t gpio_num = (uint32_t)arg;
  xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}
void _74hc595_init() {
  gpio_config_t io_conf = {
      .intr_type = GPIO_INTR_DISABLE,
      .mode = GPIO_MODE_OUTPUT,
      .pin_bit_mask = (1ULL << SCK_GPIO_PIN) | (1ULL << RCK_GPIO_PIN) |
                      (1ULL << SDA_GPIO_PIN),
      .pull_down_en = 1,
      .pull_up_en = 0,
  };
  gpio_config(&io_conf);
  HC595_SCK_Low();
  HC595_RCK_Low();
  HC595_DATA_Low();

  io_conf.intr_type = GPIO_INTR_POSEDGE;
  io_conf.mode = GPIO_MODE_INPUT;
  io_conf.pin_bit_mask = (1ULL << row1Pin_In) | (1ULL << row2Pin_In) |
                         (1ULL << row3Pin_In) | (1ULL << row4Pin_In) |
                         (1ULL << row5Pin_In) | (1ULL << row6Pin_In);
  io_conf.pull_up_en = 0;

  gpio_config(&io_conf);
  gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
  gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
  // hook isr handler for specific gpio pin
  gpio_isr_handler_add(row1Pin_In, gpio_isr_handler, (void*)row1Pin_In);
  gpio_isr_handler_add(row2Pin_In, gpio_isr_handler, (void*)row1Pin_In);
  gpio_isr_handler_add(row3Pin_In, gpio_isr_handler, (void*)row1Pin_In);
  gpio_isr_handler_add(row4Pin_In, gpio_isr_handler, (void*)row1Pin_In);
  gpio_isr_handler_add(row5Pin_In, gpio_isr_handler, (void*)row1Pin_In);
  gpio_isr_handler_add(row6Pin_In, gpio_isr_handler, (void*)row1Pin_In);
  xTaskCreate(input_sensor_thread, "input_sensor_thread", 2048, NULL, 10, NULL);
}
void HC595_Save(void) {
  HC595_RCK_Low();
  vTaskDelay(pdMS_TO_TICKS(10));
  HC595_RCK_High();
}

void HC595_Send_Byte(uint8_t byte) {
  uint8_t i;
  for (i = 0; i < 8; i++) {
    HC595_SCK_Low();
    if (byte & 0x80) {
      HC595_DATA_High();
    } else {
      HC595_DATA_Low();
    }
    byte <<= 1;

    HC595_SCK_High();
  }
}

void HC595_Send_Multi_Byte(uint8_t* data, uint16_t len) {
  uint8_t i;
  for (i = 0; i < len; i++) {
    HC595_Send_Byte(data[i]);
  }
  HC595_Save();
}
void HC595_RCK_Low() {
  gpio_set_level(SCK_GPIO_PIN, 0);
}

void HC595_SCK_Low() {
  gpio_set_level(RCK_GPIO_PIN, 0);
}
void HC595_DATA_Low() {
  gpio_set_level(SDA_GPIO_PIN, 0);
}
void HC595_RCK_High() {
  gpio_set_level(RCK_GPIO_PIN, 1);
}

void HC595_SCK_High() {
  gpio_set_level(SCK_GPIO_PIN, 1);
}
void HC595_DATA_High() {
  gpio_set_level(SDA_GPIO_PIN, 1);
}
