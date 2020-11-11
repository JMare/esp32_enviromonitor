#include "ui.h"

uint8_t currScreen = 2;
void (*draw_functions[NUM_SCREENS]) () = {draw_screenData,draw_screenSettings,draw_screenDebug1,draw_screenDebug2};
QueueHandle_t qButton;
QueueHandle_t qSensor;

static const char *TAG ="[UI]";

void ui_task(void *pvParameters)
{
  qButton = xQueueCreate(1,sizeof(uint8_t));
  qSensor = xQueueCreate(1,sizeof(SensorData));

  if(qButton == NULL)
    ESP_LOGE(TAG,"Error creating the button queue");

  draw_header_background();
  draw_header();
  draw_body_background();
  draw_functions[currScreen]();
  while(1)
  {
    vTaskDelay(200/portTICK_PERIOD_MS);
    uint8_t byte;
    if(pdTRUE == xQueueReceive(qButton,&byte,0))
    {

      if(byte == 'a')
        currScreen = switch_screen(currScreen,-1);
      else if(byte == 'd')
        currScreen = switch_screen(currScreen,1);
    }
    draw_functions[currScreen]();
  }
}

uint8_t switch_screen(uint8_t current_screen, int8_t dir)
{
  current_screen += dir;
  if(current_screen == 4)
    return 0;

  if(current_screen > 4)
    return 3;

  return current_screen;
}

void draw_screenData()
{
  TFT_setclipwin(0,HEADER_HEIGHT+1,TFT_WIDTH,TFT_HEIGHT);
  tft_bg = TFT_DARKGREY;
  TFT_fillWindow(tft_bg);
  tft_fg = TFT_WHITE;
  TFT_print("Main Data Display",5,50);
}

void draw_screenSettings()
{
  TFT_setclipwin(0,HEADER_HEIGHT+1,TFT_WIDTH,TFT_HEIGHT);
  tft_bg = TFT_DARKGREY;
  TFT_fillWindow(tft_bg);
  tft_fg = TFT_WHITE;
  TFT_print("Settings Display",5,50);
}

void draw_screenDebug1()
{

  SensorData data;
  if(pdTRUE == xQueueReceive(qSensor,&data,0))
    {
      tft_font_transparent = 0;
      TFT_setclipwin(5,HEADER_HEIGHT+1,TFT_WIDTH,TFT_HEIGHT);
      tft_bg = TFT_DARKGREY;
      tft_fg = TFT_WHITE;
      static int lastlen = 0;

      //TFT_fillWindow(tft_bg);

      // Print battery stuff
      char buf[50];
      int len = sprintf(buf,"%dmA %dV %d%% ",data.batt.current,data.batt.voltage,data.batt.soc);
      if(len != lastlen)
        TFT_fillWindow(tft_bg);

      lastlen = len;

      TFT_print("BQ27441",0,10);
      TFT_print(buf,0,30);
      if(data.batt.current > 0)
        TFT_print("CHG",LASTX,30);
      else if(data.batt.current < 0)
        TFT_print("DSG",LASTX,30);

      float temp = (data.batt.temp/10)-273.15;
      sprintf(buf,"%imAh/%imAh %.0f°C",data.batt.capacity,data.batt.full_capacity, temp);
      TFT_print(buf,0,50);

      // Print Sesnor Data
      sprintf(buf,"BME280\n%.2f deg C %.2f hPa \n%.2f%% Humidity",data.enviro.temp,data.enviro.press,data.enviro.humid);
      TFT_print(buf,0,100);
    }
}

void draw_screenDebug2()
{
  TFT_setclipwin(0,HEADER_HEIGHT+1,TFT_WIDTH,TFT_HEIGHT);
  tft_bg = TFT_DARKGREY;
  TFT_fillWindow(tft_bg);
  tft_fg = TFT_WHITE;
  TFT_print("Debug Display2",5,50);
}


void draw_header_background()
{
  TFT_setclipwin(0,0,TFT_WIDTH,HEADER_HEIGHT);
  tft_bg = TFT_LIGHTGREY;
  TFT_fillWindow(tft_bg);
}

void draw_body_background()
{
  TFT_setclipwin(0,HEADER_HEIGHT+1,TFT_WIDTH,TFT_HEIGHT);
  tft_bg = TFT_DARKGREY;
  TFT_fillWindow(tft_bg);
}

void draw_header()
{
  TFT_setclipwin(0,0,TFT_WIDTH,HEADER_HEIGHT);
  tft_fg = TFT_BLACK;
  TFT_print("SkI2c v0.01",CENTER,CENTER);
}

void draw_body()
{
}

void init_tft()
{
  esp_err_t ret;
  TFT_PinsInit();
  spi_lobo_device_handle_t spi;
  spi_lobo_bus_config_t buscfg = {};
  buscfg.mosi_io_num=PIN_NUM_MOSI;
  buscfg.miso_io_num=PIN_NUM_MISO;
  buscfg.sclk_io_num=PIN_NUM_CLK;
  buscfg.quadwp_io_num=-1;
  buscfg.quadhd_io_num=-1;
  buscfg.max_transfer_sz = 6*1024;

  spi_lobo_device_interface_config_t devcfg = {};
  devcfg.clock_speed_hz=40000000;                // Initial clock out at 40MHz
  devcfg.mode=0;                                // SPI mode 0
  devcfg.spics_io_num=-1;                       // we will use external CS pin
  devcfg.spics_ext_io_num=PIN_NUM_CS;           // external CS pin
  devcfg.flags=LB_SPI_DEVICE_HALFDUPLEX;        // ALWAYS SET  to HALF DUPLEX MODE!! for display spi

  ESP_LOGI(TAG,"Pins used: miso=%d, mosi=%d, sck=%d, cs=%d", PIN_NUM_MISO, PIN_NUM_MOSI, PIN_NUM_CLK, PIN_NUM_CS);

	// ==== Initialize the SPI bus and attach the LCD to the SPI bus ====
	ret=spi_lobo_bus_add_device(SPI_BUS, &buscfg, &devcfg, &spi);
  if(ret != ESP_OK)
    ESP_LOGE(TAG, "Failed to attach TFT to SPI bus");
  else
    ESP_LOGI(TAG,"SPI: display device added to spi bus (%d)", SPI_BUS);

	tft_disp_spi = spi;

	// ==== Test select/deselect ====
	ret = spi_lobo_device_select(spi, 1);
  if(ret != ESP_OK)
    ESP_LOGE(TAG, "Failed to select TFT");
  assert(ret==ESP_OK);
	ret = spi_lobo_device_deselect(spi);
  if(ret != ESP_OK)
    ESP_LOGE(TAG, "Failed to deselect TFT");
  assert(ret==ESP_OK);

  ESP_LOGI(TAG,"SPI: attached display device, speed=%u", spi_lobo_get_speed(spi));
	ESP_LOGI(TAG,"SPI: bus uses native pins: %s", spi_lobo_uses_native_pins(spi) ? "true" : "false");

  TFT_display_init();

  ESP_LOGI(TAG,"TFT init completed");

  TFT_invertDisplay(1);
  TFT_setGammaCurve(DEFAULT_GAMMA_CURVE);
	tft_font_transparent = 0;
  tft_bg = TFT_DARKGREY;
  tft_fg = TFT_LIGHTGREY;
	TFT_setRotation(LANDSCAPE_FLIP);
	TFT_setFont(UBUNTU16_FONT, NULL);
	TFT_resetclipwin();
}
