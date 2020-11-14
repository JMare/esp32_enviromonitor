#ifndef UI_H
#define UI_H
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "tft.h"
#include "bme280_defs.h"

#define UI_STACK_SIZE 2048
#define UI_UPDATE_RATE_MS 1000

#define SPI_BUS TFT_HSPI_HOST
#define TFT_WIDTH 320
#define TFT_HEIGHT 240

// UI AREA DEFINES
#define HEADER_HEIGHT 25

extern QueueHandle_t qButton;
extern QueueHandle_t qSensor;
extern QueueHandle_t qNetwork;

typedef struct{
  unsigned int soc;
  unsigned int voltage;
  int current;
  unsigned int full_capacity;
  unsigned int capacity;
  bool is_chg;
  bool is_dsg;
  int temp;
} BattStatus;

typedef struct{
  float temp;
  float press;
  float humid;
} EnviroData;

typedef struct{
  BattStatus batt;
  EnviroData enviro;
} SensorData;

typedef struct{
  bool connected;
  char ip_addr[50];
} NetworkData;

#define BUTTON_QUEUE_SIZE 10

// functions to draw screens
void draw_screen();

uint8_t switch_screen(uint8_t current_screen, int8_t dir);
void draw_header();
void draw_header_background();
void draw_body_background();
void init_tft();
void ui_task(void *pvParameters);

#endif
