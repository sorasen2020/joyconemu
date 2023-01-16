#pragma once

#include <string.h>
#include "esp_bt.h"
#include "esp_bt_device.h"
#include "esp_bt_main.h"
#include "esp_err.h"
#include "esp_gap_bt_api.h"
#include "esp_hidd_api.h"
#include "esp_random.h"
#include "esp_mac.h"
#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "freertos/semphr.h"
#include "pokecon.h"

#define BTDEVICE_OK     (0)
#define BTDEVICE_ERROR  (1)

int btdevice_init(void);
esp_err_t btdevice_sendreport(pokecon_report_input_t pc_report);
