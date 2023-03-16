#pragma once

/*DEBUG*/
#define WIFI_SSID             "ssid"
#define WIFI_PASSWORD         "password"


#define WIFI_CONNECT_WAIT_MAX (30 * 1000)

#define NTP_SERVER1           "pool.ntp.org"
#define NTP_SERVER2           "time.nist.gov"
#define GMT_OFFSET_SEC        (3600 * 8)
#define DAY_LIGHT_OFFSET_SEC  0

#define LV_SCREEN_WIDTH       320
#define LV_SCREEN_HEIGHT      170
#define LV_BUF_SIZE           (LV_SCREEN_WIDTH * LV_SCREEN_HEIGHT)
/*ESP32S3*/
#define PIN_POWER_ON          46

#define PIN_IIC_SDA           18
#define PIN_IIC_SCL           8

#define PIN_APA102_CLK        45
#define PIN_APA102_DI         42

#define PIN_ENCODE_A          2
#define PIN_ENCODE_B          1
#define PIN_ENCODE_BTN        0

#define PIN_LCD_BL            15
#define PIN_LCD_DC            13
#define PIN_LCD_CS            10
#define PIN_LCD_CLK           12
#define PIN_LCD_MOSI          11
#define PIN_LCD_RES           9

#define PIN_BAT_VOLT          4

#define PIN_IIS_BCLK          7
#define PIN_IIS_WCLK          5
#define PIN_IIS_DOUT          6

#define PIN_ES7210_BCLK       47
#define PIN_ES7210_LRCK       21
#define PIN_ES7210_DIN        14
#define PIN_ES7210_MCLK       48

#define PIN_SD_CS             39
#define PIN_SD_SCK            40
#define PIN_SD_MOSI           41
#define PIN_SD_MISO           38

#define SAMPLES                  512
//#define SAMPLE_BLOCK             64
#define SAMPLE_BLOCK             1024
#define SAMPLE_FREQ              16000

#define I2S_PORT I2S_NUM_0
#define I2S_SAMPLE_BITS   (16)
#define I2S_READ_LEN      (16 * 1024)
#define RECORD_TIME       (20) //Seconds
#define I2S_CHANNEL_NUM   (1)
#define RECORD_SIZE (I2S_CHANNEL_NUM * SAMPLE_FREQ * I2S_SAMPLE_BITS / 8 * RECORD_TIME)

#define NUM_LEDS 7
#define DATA_PIN 42
#define CLOCK_PIN 45

#define PIN_IN1 2
#define PIN_IN2 1

#define color1 TFT_WHITE
#define color2  0x8410
#define color3 TFT_ORANGE
#define color4 TFT_BLACK  //background
#define color5 0xC638
