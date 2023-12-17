#include "utils.h"
#include "cJSON.h"
#include "esp_log.h"

#define TAG "utils"
void cmd_parse(char* cmd) {
  cJSON* root;
  ESP_LOGI(TAG, "cmd:%s", cmd);
  root = cJSON_Parse(cmd);
  if (root != NULL) {
    ESP_LOGI(TAG, "cmd:%s",
             cJSON_GetObjectItemCaseSensitive(root, "cmd")->valuestring);

  } else {
    ESP_LOGI(TAG, "data parse fail");
  }
}
