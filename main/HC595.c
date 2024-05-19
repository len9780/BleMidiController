#include "HC595.h"
#include <inttypes.h>
#include <esp_check.h>
#include <string.h>
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

#define TAG "BleMidiController"
#define ESP_INTR_FLAG_DEFAULT 0

extern uint8_t* midi_dat;
HC595_ctrl_pin_set * pin_set_r = NULL;

uint8_t midi_note_data[12][3] = {{0x90, 0x3C, 0x40},
                                 // byte0->0x90(0x90:Note On,0x80:NoteOff)
                                 // byte1->0x3C->C3
                                 // byte2->0x40->velocity

                                 {0x90, 0x3C, 0x40},
                                 {0x90, 0x3C, 0x40},
                                 {0x90, 0x3C, 0x40},
                                 {0x90, 0x3C, 0x40},
                                 {0x90, 0x3C, 0x40},
                                 {0x90, 0x3C, 0x40},
                                 {0x90, 0x3C, 0x40},
                                 {0x90, 0x3C, 0x40},
                                 {0x90, 0x3C, 0x40},
                                 {0x90, 0x3C, 0x40},
                                 {0x90, 0x3C, 0x40}};

// based on MIDI Tutorial Part 3
typedef enum midi_note { Note_On = 0x90, Note_Off = 0x80 } midi_note;

typedef enum midi_pitch {
  C_pitch = 0x3C,
  CS_pitch,
  D_pitch,
  DS_pitch,
  E_pitch,
  F_pitch,
  FS_pitch,
  G_pitch,
  GS_pitch,
  A_pitch,
  AS_pitch,
  B_pitch,
} midi_pitch;
uint8_t midi_pitch_data_array[] = {C_pitch,  CS_pitch, D_pitch,  DS_pitch,
                                   E_pitch,  F_pitch,  FS_pitch, G_pitch,
                                   GS_pitch, A_pitch,  AS_pitch, B_pitch};

uint8_t out_position = 0x01;
static QueueHandle_t gpio_evt_queue = NULL;
static void input_sensor_thread(void* arg) {
  uint32_t io_num;
  for (;;) {
    if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
      printf("\nGPIO[%lu] intr, val: %d,out_position:%d", io_num,
             gpio_get_level(io_num), out_position);
      // if ((out_position - 1) & 0x01) {
      //   if (io_num == row1Pin_In) {
      //     printf("\ntrigger pin:%lu,level:%d", io_num,
      //     gpio_get_level(io_num)); midi_dat = midi_note_data[0];
      //   }
      //   if (io_num == row2Pin_In) {
      //     printf("\ntrigger pin:%lu,level:%d", io_num,
      //     gpio_get_level(io_num)); midi_dat = midi_note_data[1];
      //   }
      //   if (io_num == row3Pin_In) {
      //     printf("\ntrigger pin:%lu,level:%d", io_num,
      //     gpio_get_level(io_num)); midi_dat = midi_note_data[2];
      //   }
      //   if (io_num == row4Pin_In) {
      //     printf("\ntrigger pin:%lu,level:%d", io_num,
      //     gpio_get_level(io_num)); midi_dat = midi_note_data[3];
      //   }
      //   if (io_num == row5Pin_In) {
      //     printf("\ntrigger pin:%lu,level:%d", io_num,
      //     gpio_get_level(io_num)); midi_dat = midi_note_data[4];
      //   }
      //   // if (io_num == row6Pin_In) {
      //   //   printf("\ntrigger pin:%lu,level:%d", io_num,
      //   gpio_get_level(io_num));
      //   //   midi_dat = midi_note_data[5];
      //   // }
      // }
      // if ((out_position - 1) & 0x02) {
      //   if (io_num == row1Pin_In) {
      //     printf("trigger pin:%lu,level:%d", io_num, gpio_get_level(io_num));
      //     midi_dat = midi_note_data[6];
      //   }
      //   if (io_num == row2Pin_In) {
      //     printf("trigger pin:%lu,level:%d", io_num, gpio_get_level(io_num));
      //     midi_dat = midi_note_data[7];
      //   }
      //   if (io_num == row3Pin_In) {
      //     printf("trigger pin:%lu,level:%d", io_num, gpio_get_level(io_num));
      //     midi_dat = midi_note_data[8];
      //   }
      //   if (io_num == row4Pin_In) {
      //     printf("trigger pin:%lu,level:%d", io_num, gpio_get_level(io_num));
      //     midi_dat = midi_note_data[9];
      //   }
      //   if (io_num == row5Pin_In) {
      //     printf("trigger pin:%lu,level:%d", io_num, gpio_get_level(io_num));
      //     midi_dat = midi_note_data[10];
      //   }
      //   // if (io_num == row6Pin_In) {
      //   //   printf("trigger pin:%lu,level:%d", io_num,
      //   gpio_get_level(io_num));
      //   //   midi_dat = midi_note_data[11];
      //   // }
      // }
    }
  }
}
static void IRAM_ATTR gpio_isr_handler(void* arg) {
  uint32_t gpio_num = (uint32_t)arg;
  xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

void HC595_pin_register(HC595_ctrl_pin_set pin_set,HC595_ctrl_pin_set**record){
  // ESP_LOGI(TAG,"HC595_pin_register first element addr:%p",*record);
  HC595_ctrl_pin_set * show_add_addr;
  int ic_index = 0;
  // if(*record==NULL){
  //   (*record) = (HC595_ctrl_pin_set*)malloc(sizeof(HC595_ctrl_pin_set));
  //   (*record)->RCK = pin_set.RCK;
  //   (*record)->SCK = pin_set.SCK;
  //   (*record)->SDA= pin_set.SDA;
  //   (*record)->next= NULL;
  //   show_add_addr = (*record);
  // }
  // else{
  // while((*record)->next!=NULL){
  //   *record =  (*record)->next;
  // }
  
  //   (*record)->next = (HC595_ctrl_pin_set*)malloc(sizeof(HC595_ctrl_pin_set));
  //   (*record)->next->RCK = pin_set.RCK;
  //   (*record)->next->SCK = pin_set.SCK;
  //   (*record)->next->SDA= pin_set.SDA;
  //   (*record)->next->next=NULL;
  //   show_add_addr = (*record)->next;
  // }
  while((*record)!=NULL){
    record =  &((*record)->next);
    ic_index+=1;
  }

    (*record) = (HC595_ctrl_pin_set*)malloc(sizeof(HC595_ctrl_pin_set));
    (*record)->ic_id = ic_index;
    (*record)->RCK = pin_set.RCK;
    (*record)->SCK = pin_set.SCK;
    (*record)->SDA= pin_set.SDA;
    (*record)->next= NULL;
    show_add_addr = (*record);
    // ESP_LOGI(TAG,"HC595_pin_register add new node:%p",show_add_addr);
    gpio_config_t io_conf = {
      .intr_type = GPIO_INTR_DISABLE,
      .mode = GPIO_MODE_OUTPUT,
      .pin_bit_mask = (1ULL << (*record)->SCK) | (1ULL << (*record)->RCK) |
                      (1ULL << (*record)->SDA),
      .pull_down_en = 1,
      .pull_up_en = 0,
  };
    esp_err_t err = gpio_config(&io_conf);
    ESP_LOGI(TAG,"HC595_pin_register,[addr]:%p[data]%d,%d,%d,%s",show_add_addr,pin_set.RCK,pin_set.SCK,pin_set.SDA,esp_err_to_name(err));
    HC595_SCK_Low((*record));
    HC595_RCK_Low((*record));
    HC595_DATA_Low((*record));
  // *record = head_tmp;
}
void dump_HC595_pin_list(HC595_ctrl_pin_set * pin){
  ESP_LOGI(TAG,"dump_HC595_pin_list head %p",pin);
  while(pin!=NULL){
    ESP_LOGI(TAG,"dump_HC595_pin_list:%p,%d,%d,%d,%d",pin,pin->ic_id,pin->RCK,pin->SCK,pin->SDA);
    pin = pin->next;
  }
}
void gpio_pin_test(HC595_ctrl_pin_set*pin){
  while(pin!=NULL){
    ESP_LOGI(TAG,"pin no[SCK,RCK,SDA]:[%d,%d,%d]",pin->SCK,pin->RCK,pin->SDA);
    gpio_set_level(pin->SCK, 1);
    ESP_LOGI(TAG,"pin no:%d,level:%d",pin->SCK,1);
    vTaskDelay(pdMS_TO_TICKS(5000));
    gpio_set_level(pin->RCK, 1);
    ESP_LOGI(TAG,"pin no:%d,level:%d",pin->RCK,1);
    vTaskDelay(pdMS_TO_TICKS(5000));
    gpio_set_level(pin->SDA, 1);
    ESP_LOGI(TAG,"pin no:%d,level:%d",pin->SDA,1);
    vTaskDelay(pdMS_TO_TICKS(5000));

    gpio_set_level(pin->SCK, 0);
    ESP_LOGI(TAG,"pin no:%d,level:%d",pin->SCK,0);
    vTaskDelay(pdMS_TO_TICKS(5000));
    gpio_set_level(pin->RCK, 0);
    ESP_LOGI(TAG,"pin no:%d,level:%d",pin->RCK,0);
    vTaskDelay(pdMS_TO_TICKS(5000));
    gpio_set_level(pin->SDA, 0);
    ESP_LOGI(TAG,"pin no:%d,level:%d",pin->SDA,0);
    vTaskDelay(pdMS_TO_TICKS(5000));
    pin = pin->next;
  }

}
void _74hc595_init() {
  // gpio_config_t io_conf = {
  //     .intr_type = GPIO_INTR_DISABLE,
  //     .mode = GPIO_MODE_OUTPUT,
  //     .pin_bit_mask = (1ULL << SCK_GPIO_PIN) | (1ULL << RCK_GPIO_PIN) |
  //                     (1ULL << SDA_GPIO_PIN),
  //     .pull_down_en = 1,
  //     .pull_up_en = 0,
  // };
  // gpio_config(&io_conf);
  gpio_config_t io_conf ;
  memset(&io_conf, 0, sizeof(gpio_config_t));
  io_conf.intr_type = GPIO_INTR_POSEDGE;
  io_conf.mode = GPIO_MODE_INPUT;
  io_conf.pin_bit_mask = (1ULL << row1Pin_In) | (1ULL << row2Pin_In) |
                         (1ULL << row3Pin_In) | (1ULL << row4Pin_In) |
                         (1ULL << row5Pin_In);  // | (1ULL << row6Pin_In);
  io_conf.pull_down_en = 1;

  ESP_LOGI(TAG,"input io pin init:%s",esp_err_to_name(gpio_config(&io_conf)));

  gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
  gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
  // hook isr handler for specific gpio pin
  gpio_isr_handler_add(row1Pin_In, gpio_isr_handler, (void*)row1Pin_In);
  gpio_isr_handler_add(row2Pin_In, gpio_isr_handler, (void*)row2Pin_In);
  gpio_isr_handler_add(row3Pin_In, gpio_isr_handler, (void*)row3Pin_In);
  gpio_isr_handler_add(row4Pin_In, gpio_isr_handler, (void*)row4Pin_In);
  gpio_isr_handler_add(row5Pin_In, gpio_isr_handler, (void*)row5Pin_In);
  // gpio_isr_handler_add(row6Pin_In, gpio_isr_handler, (void*)row6Pin_In);
  xTaskCreate(input_sensor_thread, "input_sensor_thread", 2048, NULL, 10, NULL);
  HC595_pin_register((HC595_ctrl_pin_set){0,13,14,12,NULL},&pin_set_r);
  HC595_pin_register((HC595_ctrl_pin_set){1,26,27,25,NULL},&pin_set_r);
  dump_HC595_pin_list(pin_set_r);
  gpio_pin_test(pin_set_r);
  // HC595_SCK_Low(pin_set_r);
  // HC595_RCK_Low(pin_set_r);
  // HC595_DATA_Low(pin_set_r);
  ESP_LOGI(TAG,"_74hc595_init done");
}
void HC595_Save(void) {
  HC595_RCK_Low(pin_set_r);
  vTaskDelay(pdMS_TO_TICKS(10));
  HC595_RCK_High(pin_set_r);
}

void HC595_Send_Byte(HC595_ctrl_pin_set *hc595, uint8_t byte, uint8_t dir) {
  uint8_t i;
  for (i = 0; i < 8; i++) {
    HC595_SCK_Low(hc595);
    if (dir == 1) {
      if (byte & 0x80) {
        HC595_DATA_High(hc595);
      } else {
        HC595_DATA_Low(hc595);
      }
      byte = byte << 1;
    } else {
      if (byte & 0x01) {
        HC595_DATA_High(hc595);
      } else {
        HC595_DATA_Low(hc595);
      }
      byte = byte >> 1;
    }

    HC595_SCK_High(pin_set_r);
  }
}

void HC595_Send_Multi_Byte(uint8_t* data, uint16_t len) {
  uint8_t index = 1;
  HC595_ctrl_pin_set ** pin_set_r_tmp = &pin_set_r;
  ESP_LOGI(TAG,"HC595_Send_Multi_Byte start");
while(*pin_set_r_tmp!=NULL){
  index = 1;
  while(index!=0){
  HC595_RCK_Low(*pin_set_r_tmp);
  HC595_Send_Byte(*pin_set_r_tmp,index, 1);
  HC595_RCK_High( *pin_set_r_tmp);
  ESP_LOGI(TAG,"HC595_Send_Multi_Byte:%d,%d,%d,%d,%p,%d",(*pin_set_r_tmp)->ic_id,(*pin_set_r_tmp)->RCK,(*pin_set_r_tmp)->SCK,(*pin_set_r_tmp)->SDA,*pin_set_r_tmp,index);
  vTaskDelay(pdMS_TO_TICKS(3000));
  index = index << 1;
  }
  pin_set_r_tmp = &((*pin_set_r_tmp)->next);
  len--;
}
  //  HC595_Save();
}
void HC595_RCK_Low(HC595_ctrl_pin_set *pin) {
  // gpio_set_level(RCK_GPIO_PIN, 0);
  // ESP_LOGI(TAG,"HC595_RCK_Low");
  gpio_set_level(pin->RCK, 0);
}

void HC595_SCK_Low(HC595_ctrl_pin_set *pin) {
  // gpio_set_level(SCK_GPIO_PIN, 0);
  // ESP_LOGI(TAG,"HC595_SCK_Low");
  gpio_set_level(pin->SCK, 0);
}
void HC595_DATA_Low(HC595_ctrl_pin_set *pin) {
  // gpio_set_level(SDA_GPIO_PIN, 0);
  // ESP_LOGI(TAG,"HC595_DATA_Low");
  gpio_set_level(pin->SDA, 0);
}
void HC595_RCK_High(HC595_ctrl_pin_set *pin) {
  // gpio_set_level(RCK_GPIO_PIN, 1);
  // ESP_LOGI(TAG,"HC595_RCK_High");
  gpio_set_level(pin->RCK, 1);
}

void HC595_SCK_High(HC595_ctrl_pin_set *pin) {
  // gpio_set_level(SCK_GPIO_PIN, 1);
  // ESP_LOGI(TAG,"HC595_SCK_High");
  gpio_set_level(pin->SCK, 1);
}
void HC595_DATA_High(HC595_ctrl_pin_set *pin) {
  // gpio_set_level(SDA_GPIO_PIN, 1);
  // ESP_LOGI(TAG,"HC595_DATA_High");
  gpio_set_level(pin->SDA, 1);
}
void midi_init() {
  printf("length:%d\n", sizeof(midi_note_data) / 3);
  for (int i = 0; i < sizeof(midi_note_data) / 3; i++) {
    midi_note_data[i][1] = midi_pitch_data_array[i];
  }
}
