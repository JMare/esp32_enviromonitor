#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sys.h"


#include "ui.h"
#include "credentials.h"
#include "bme280_idf.h"
#include "bme280_defs.h"

static const char *TAG = "[MAIN]";

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

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

static int s_retry_num = 0;

static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < 5) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        NetworkData ndata;
        ndata.connected = true;
        sprintf(ndata.ip_addr,IPSTR,IP2STR(&event->ip_info.ip));
        xQueueOverwrite(qNetwork,&ndata);
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            /* Setting a password implies station will connect to all security modes including WEP/WPA.
             * However these modes are deprecated and not advisable to be used. Incase your Access point
             * doesn't support WPA2, these mode can be enabled by commenting below line */
	     .threshold.authmode = WIFI_AUTH_WPA2_PSK,

            .pmf_cfg = {
                .capable = true,
                .required = false
            },
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 WIFI_SSID, WIFI_PASS);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 WIFI_SSID, WIFI_PASS);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }

    /* The event will not be processed after unregister */
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
    vEventGroupDelete(s_wifi_event_group);
}

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
  vTaskDelay(100/portTICK_PERIOD_MS);

  NetworkData ndata = {};
  ndata.connected = false;
  xQueueOverwrite(qNetwork,&ndata);

  //Initialize NVS
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
  wifi_init_sta();

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
