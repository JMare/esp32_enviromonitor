#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"

#include "ui.h"
#include "bme280_idf.h"
#include "bme280_defs.h"

static const char *TAG = "[MAIN]";

#define BL_IO               26
#define I2C_SCL_IO          22        /*!< gpio number for I2C master clock */
#define I2C_SDA_IO          21        /*!< gpio number for I2C master data  */
#define I2C_FREQ_HZ         400000    /*!< I2C master clock frequency */
#define I2C_PORT_NUM        I2C_NUM_0 /*!< I2C port number for master dev */
#define I2C_TX_BUF_DISABLE  0         /*!< I2C master do not need buffer */
#define I2C_RX_BUF_DISABLE  0         /*!< I2C master do not need buffer */
#define ACK_CHECK_EN        0x1       /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS       0x0       /*!< I2C master will not check ack from slave */
#define BQ27441_I2C_ADDRESS 0x55

static void i2c_master_init(void)
{
  i2c_config_t conf = {};
  conf.mode = I2C_MODE_MASTER;
  conf.sda_io_num = (gpio_num_t)I2C_SDA_IO;
  conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
  conf.scl_io_num = (gpio_num_t)I2C_SCL_IO;
  conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
  conf.master.clk_speed = I2C_FREQ_HZ;
  i2c_param_config(I2C_PORT_NUM, &conf);
  ESP_ERROR_CHECK(i2c_driver_install(I2C_PORT_NUM, conf.mode, I2C_RX_BUF_DISABLE, I2C_TX_BUF_DISABLE, 0));
  i2c_set_timeout(I2C_PORT_NUM,0xFFFFF);
}

void app_main()
{
  ESP_LOGI(TAG,"esp32_enviromonitor Main Started");

  i2c_master_init();

  vTaskDelay(100/portTICK_PERIOD_MS);

  // Init BME280
  ESP_ERROR_CHECK(bme280_begin(I2C_PORT_NUM,0x77));

  gpio_num_t bl_pin = (gpio_num_t)BL_IO;
  gpio_pad_select_gpio(bl_pin);
  gpio_set_direction(bl_pin, GPIO_MODE_OUTPUT);
  gpio_set_level(bl_pin,1);

  //Initialize the TFT
  init_tft();
  // Start the UI task
  TaskHandle_t uiHandle = NULL;
  xTaskCreate(ui_task,"UI", UI_STACK_SIZE,NULL,tskIDLE_PRIORITY, &uiHandle);

  while (1) {
      vTaskDelay(100/portTICK_PERIOD_MS);

      SensorData data;

      struct bme280_data comp_data;
      bme280_get_forced_data(&comp_data);

      data.enviro.temp = comp_data.temperature;
      data.enviro.press = 0.01 * comp_data.pressure;
      data.enviro.humid = comp_data.humidity;

      xQueueOverwrite(qSensor,&data);
  }
}
