#pragma once

/*DEBUG*/
#define WIFI_SSID             "ssid"
#define WIFI_PASSWORD         "password"


#define WIFI_CONNECT_WAIT_MAX (30 * 1000)

#define WIFI_CONNECT_WAIT_MAX (30 * 1000)

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

#define PIN_IN1 2
#define PIN_IN2 1

#define ESP32_RTOS

#define VAD_SAMPLE_RATE_HZ              16000
#define VAD_FRAME_LENGTH_MS             30
#define VAD_BUFFER_LENGTH               (VAD_FRAME_LENGTH_MS * VAD_SAMPLE_RATE_HZ / 1000)
#define I2S_CH                          I2S_NUM_1
#define SAMPLE_BUFFER_SIZE 512
