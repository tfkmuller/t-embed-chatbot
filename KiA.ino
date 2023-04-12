/*
 * ChatGPT Client For ESP32 on Lilygo t-embed
 * Description: For HTTPS connection using WiFiClientSecure
 * Author: Tom Mueller based on ThatProject code
 * Date: 03-08-2023
 */

#include <Arduino.h>
#include <driver/i2s.h>
#include <esp_vad.h>
#include "es7210.h"
#include "pin_config.h"
#include "OTA.h"
#include <Battery18650Stats.h>
#include <DigitalRainAnim.h>
#include <TFT_eSPI.h>
#include <RotaryEncoder.h>
#include <FastLED.h>
#include "FS.h"
#include "SD_MMC.h"
#include <JPEGDecoder.h>
#include "Audio.h"
#include "OneButton.h"
#include "create_wav.h"

#define color1 TFT_WHITE
#define color2  0x8410
#define color3 TFT_ORANGE
#define color4 TFT_BLACK  //background
#define color5 0xC638

size_t          bytes_read;
CRGB leds[7];

int16_t         *vad_buff;
vad_handle_t    vad_inst;

unsigned short colorsF[3] = {TFT_WHITE, TFT_BLACK, TFT_WHITE};
unsigned short colorsB[3] = {TFT_BLACK, color5, TFT_BLACK};
unsigned short colorsL[3] = {TFT_RED, TFT_BLACK, TFT_RED};
unsigned short colorsD[3] = {TFT_WHITE, TFT_BLACK, TFT_RED};

int playing = 0;

File file;

TFT_eSPI tft = TFT_eSPI();
TFT_eSprite img = TFT_eSprite(&tft);

RotaryEncoder encoder(PIN_IN1, PIN_IN2, RotaryEncoder::LatchMode::TWO03);
OneButton button(16, true);

DigitalRainAnim digitalRainAnim = DigitalRainAnim();
Audio audio;
Battery18650Stats battery(PIN_BAT_VOLT);

void setup()
{
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  //pinMode(0, INPUT_PULLUP);
  Serial.begin(115200);
  pinMode(PIN_POWER_ON, OUTPUT);
  digitalWrite(PIN_POWER_ON, HIGH);
  pinMode(15, OUTPUT);
  digitalWrite(15, HIGH);

  button.attachClick(click1);
  button.attachDoubleClick(doubleclick1);
  button.attachLongPressStart(longPressStart1);
  button.attachLongPressStop(longPressStop1);
  button.attachDuringLongPress(longPress1);

  FastLED.addLeds<APA102, PIN_APA102_DI, PIN_APA102_CLK, RGB>(leds, 7);
  FastLED.clear();
  FastLED.show();

  Serial.printf("psram size : %d kb\r\n", ESP.getPsramSize() / 1024);
  Serial.printf("FLASH size : %d kb\r\n", ESP.getFlashChipSize() / 1024);

  Wire.begin(PIN_IIC_SDA, PIN_IIC_SCL);

  uint32_t ret_val = ESP_OK;

  audio_hal_codec_config_t cfg = {
    .adc_input = AUDIO_HAL_ADC_INPUT_ALL,
    .codec_mode = AUDIO_HAL_CODEC_MODE_ENCODE,
    .i2s_iface =
    {
      .mode = AUDIO_HAL_MODE_SLAVE,
      .fmt = AUDIO_HAL_I2S_NORMAL,
      .samples = AUDIO_HAL_16K_SAMPLES,
      .bits = AUDIO_HAL_BIT_LENGTH_16BITS,
    },
  };

  ret_val |= es7210_adc_init(&Wire, &cfg);
  ret_val |= es7210_adc_config_i2s(cfg.codec_mode, &cfg.i2s_iface);
  ret_val |= es7210_adc_set_gain(
               (es7210_input_mics_t)(ES7210_INPUT_MIC1 | ES7210_INPUT_MIC2),
               (es7210_gain_value_t)GAIN_0DB);
  ret_val |= es7210_adc_set_gain(
               (es7210_input_mics_t)(ES7210_INPUT_MIC3 | ES7210_INPUT_MIC4),
               (es7210_gain_value_t)GAIN_37_5DB);
  ret_val |= es7210_adc_ctrl_state(cfg.codec_mode, AUDIO_HAL_CTRL_START);



  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = VAD_SAMPLE_RATE_HZ,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ALL_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = 1000,
    .use_apll = true,
    .tx_desc_auto_clear = true,
    .fixed_mclk = 0,
    .mclk_multiple = I2S_MCLK_MULTIPLE_256,
    .bits_per_chan = I2S_BITS_PER_CHAN_16BIT,
    .chan_mask = (i2s_channel_t)(I2S_TDM_ACTIVE_CH0 | I2S_TDM_ACTIVE_CH1),
  };

  i2s_pin_config_t pin_config = {0};
  pin_config.bck_io_num = PIN_ES7210_BCLK;
  pin_config.ws_io_num = PIN_ES7210_LRCK;
  pin_config.data_in_num = PIN_ES7210_DIN;
  pin_config.mck_io_num = PIN_ES7210_MCLK;
  i2s_driver_install(I2S_CH, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_CH, &pin_config);
  i2s_zero_dma_buffer(I2S_CH);
  // i2s_start(I2S_NUM_1);


  vad_inst = vad_create(VAD_MODE_0);
  vad_buff = (int16_t *)malloc(VAD_BUFFER_LENGTH * sizeof(short));
  if (vad_buff == NULL) {
    while (1) {
      Serial.println("Memory allocation failed!");
      delay(1000);
    }
  }

  setupOTA("TEmbed", WIFI_SSID, WIFI_PASSWORD);

  Serial.print("Connecting to WiFi network: ");
  Serial.print(WIFI_SSID);
  Serial.println("'...");

  tft.begin();
  tft.writecommand(0x11);
  tft.setRotation(3);
  tft.setSwapBytes(true);
  tft.fillScreen(TFT_BLACK);
  tft.setTextDatum(4);

  img.setSwapBytes(true);
  img.createSprite(320, 170);
  digitalRainAnim.init(&tft, 3, 20, 3, 20, 60);

  SD_init();
  
  audio.setPinout(PIN_IIS_BCLK , PIN_IIS_WCLK , PIN_IIS_DOUT);
  audio.connecttoFS(SD_MMC, "/book.mp3");
  audio.setVolume(21);

}
void loop()
{
  button.tick();

  encoder.tick();

#ifdef defined(ESP32_RTOS) && defined(ESP32)
#else // If you do not use FreeRTOS, you have to regulary call the handle method.
  ArduinoOTA.handle();
#endif


  static int pos = 0;

  digitalRainAnim.loop();

  int newPos = encoder.getPosition();

  if (pos != newPos) {
    if (pos > newPos)
    {
      // Serial.println("forward");
      tft.fillScreen(TFT_BLACK);
      tft.setTextSize(1);
      digitalRainAnim.resume();
      //drawSdJpeg("/peter.jpg", 0, 0);
    
    }
    if (pos < newPos)
    {
      //Serial.println("back");
      digitalRainAnim.pause();
      tft.fillScreen(TFT_BLACK);
      drawSdJpeg("/tomuelle.jpg", 0, 0);
      //drawSdJpeg("/kids.jpg", 0, 0);
    }
    pos = newPos;

  }

  if (playing == 1) {
    audio.loop();

  }

  //delay(5);
}

void SD_init() {
  pinMode(PIN_SD_CS, OUTPUT);
  digitalWrite(PIN_SD_CS, 1);
  SD_MMC.setPins(PIN_SD_SCK, PIN_SD_MOSI, PIN_SD_MISO);
  if (!SD_MMC.begin("/sdcard", true)) {
    Serial.println("Card Mount Failed");
    return;
  }
  uint8_t cardType = SD_MMC.cardType();

  if (cardType == CARD_NONE) {
    Serial.println("No SD card attached");
    return;
  }
  delay(500);
}

void readtext()
{
  File textfile = SD_MMC.open("/response.txt");
  if (!textfile) {
    Serial.println("Failed to open file for reading");
    return;
  }

  Serial.println("File Content:");
  while (textfile.available()) {
    Serial.write(textfile.read());
  }
  textfile.close();
}

void drawSdJpeg(const char *filename, int xpos, int ypos) {

  // Open the named file (the Jpeg decoder library will close it)
  File jpegFile = SD_MMC.open( filename, FILE_READ);  // or, file handle reference for SD library

  if ( !jpegFile ) {
    Serial.print("ERROR: File \""); Serial.print(filename); Serial.println ("\" not found!");
    return;
  }

  Serial.println("===========================");
  Serial.print("Drawing file: "); Serial.println(filename);
  Serial.println("===========================");

  // Use one of the following methods to initialise the decoder:
  bool decoded = JpegDec.decodeSdFile(jpegFile);  // Pass the SD file handle to the decoder,
  //bool decoded = JpegDec.decodeSdFile(filename);  // or pass the filename (String or character array)

  if (decoded) {
    // print information about the image to the serial port
    // jpegInfo();
    // render the image onto the screen at given coordinates
    jpegRender(xpos, ypos);
  }
  else {
    Serial.println("Jpeg file format not supported!");
  }
}

//####################################################################################################
// Draw a JPEG on the TFT, images will be cropped on the right/bottom sides if they do not fit
//####################################################################################################
// This function assumes xpos,ypos is a valid screen coordinate. For convenience images that do not
// fit totally on the screen are cropped to the nearest MCU size and may leave right/bottom borders.
void jpegRender(int xpos, int ypos) {

  //jpegInfo(); // Print information from the JPEG file (could comment this line out)

  uint16_t *pImg;
  uint16_t mcu_w = JpegDec.MCUWidth;
  uint16_t mcu_h = JpegDec.MCUHeight;
  uint32_t max_x = JpegDec.width;
  uint32_t max_y = JpegDec.height;

  bool swapBytes = tft.getSwapBytes();
  tft.setSwapBytes(true);

  // Jpeg images are draw as a set of image block (tiles) called Minimum Coding Units (MCUs)
  // Typically these MCUs are 16x16 pixel blocks
  // Determine the width and height of the right and bottom edge image blocks
  uint32_t min_w = jpg_min(mcu_w, max_x % mcu_w);
  uint32_t min_h = jpg_min(mcu_h, max_y % mcu_h);

  // save the current image block size
  uint32_t win_w = mcu_w;
  uint32_t win_h = mcu_h;

  // record the current time so we can measure how long it takes to draw an image
  uint32_t drawTime = millis();

  // save the coordinate of the right and bottom edges to assist image cropping
  // to the screen size
  max_x += xpos;
  max_y += ypos;

  // Fetch data from the file, decode and display
  while (JpegDec.read()) {    // While there is more data in the file
    pImg = JpegDec.pImage ;   // Decode a MCU (Minimum Coding Unit, typically a 8x8 or 16x16 pixel block)

    // Calculate coordinates of top left corner of current MCU
    int mcu_x = JpegDec.MCUx * mcu_w + xpos;
    int mcu_y = JpegDec.MCUy * mcu_h + ypos;

    // check if the image block size needs to be changed for the right edge
    if (mcu_x + mcu_w <= max_x) win_w = mcu_w;
    else win_w = min_w;

    // check if the image block size needs to be changed for the bottom edge
    if (mcu_y + mcu_h <= max_y) win_h = mcu_h;
    else win_h = min_h;

    // copy pixels into a contiguous block
    if (win_w != mcu_w)
    {
      uint16_t *cImg;
      int p = 0;
      cImg = pImg + win_w;
      for (int h = 1; h < win_h; h++)
      {
        p += mcu_w;
        for (int w = 0; w < win_w; w++)
        {
          *cImg = *(pImg + w + p);
          cImg++;
        }
      }
    }

    // calculate how many pixels must be drawn
    uint32_t mcu_pixels = win_w * win_h;

    // draw image MCU block only if it will fit on the screen
    if (( mcu_x + win_w ) <= tft.width() && ( mcu_y + win_h ) <= tft.height())
      tft.pushImage(mcu_x, mcu_y, win_w, win_h, pImg);
    else if ( (mcu_y + win_h) >= tft.height())
      JpegDec.abort(); // Image has run off bottom of screen so abort decoding
  }

  tft.setSwapBytes(swapBytes);

}

void click1()
{

  Serial.println("Button pressed");
  leds[0] = CRGB::Green;
  leds[1] = CRGB::Green;
  leds[2] = CRGB::Green;
  leds[3] = CRGB::Green;
  leds[4] = CRGB::Green;
  leds[5] = CRGB::Green;
  leds[6] = CRGB::Green;
  FastLED.show();
  delay(150);
  FastLED.clear();
  FastLED.show();

  digitalRainAnim.pause();
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_GREEN);
  tft.setTextSize(2);

  tft.setCursor(0, 0);
  tft.println("Battery charge:");
  tft.setCursor(200, 0);
  tft.println(battery.getBatteryChargeLevel());

  playing = 0;
}

void doubleclick1()
{
  listFiles("/");
  microphone_record("/rec1.wav", 8);
}
void longPressStart1()
{}
void longPressStop1()
{
  playing = 1;
}
void longPress1()
{
  //Serial.println("Pressed");
  speechdetected();
  digitalRainAnim.pause();
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_BLUE);
  tft.setTextSize(2);
  tft.setCursor(0, 0);
  tft.println("Long click");
  leds[0] = CRGB::Red;
  leds[1] = CRGB::Red;
  leds[2] = CRGB::Red;
  leds[3] = CRGB::Red;
  leds[4] = CRGB::Red;
  leds[5] = CRGB::Red;
  leds[6] = CRGB::Red;
  FastLED.show();
  delay(150);
  FastLED.clear();
  FastLED.show();
  //microphone_record("/rec1.wav", 8);
}

void listFiles(String dirName) {
  File root = SD_MMC.open(dirName);

  if (!root) {
    Serial.println("Failed to open directory!");
    return;
  }

  if (!root.isDirectory()) {
    Serial.println("Not a directory!");
    return;
  }

  File file = root.openNextFile();
  while (file) {
    if (file.isDirectory()) {
      Serial.print("  DIR : ");
      Serial.println(file.name());
      listFiles(file.name());
    } else {
      Serial.print("  FILE: ");
      //      tft.setTextColor(TFT_GREEN);
      //  tft.setCursor(0, 0);
      //      tft.println(file.name());
      Serial.print(file.name());
      //

      //      tft.println(file.size());
      Serial.print("  SIZE: ");
      Serial.println(file.size());
    }
    file = root.openNextFile();
    //    tft.setCursor(0, 20);
  }
}

void speechdetected()
{

  i2s_read(I2S_CH, (char *)vad_buff, VAD_BUFFER_LENGTH * sizeof(short), &bytes_read, portMAX_DELAY);
  // Feed samples to the VAD process and get the result
  vad_state_t vad_state = vad_process(vad_inst, vad_buff, VAD_SAMPLE_RATE_HZ, VAD_FRAME_LENGTH_MS);
  if (vad_state == VAD_SPEECH) {
    //Serial.print(millis());
    Serial.println("Speech detected");
    //microphone_record("/rec1.wav", millis()/1000);
    //    microphone_record("/rec1.wav", 8);
  }
}

void microphone_record(const char* song_name, uint32_t duration) {
  // Add wav header to the file so we can play it from PC later
  if (!create_wav_file(song_name, duration, 1, VAD_SAMPLE_RATE_HZ, I2S_BITS_PER_SAMPLE_16BIT)) {
    Serial.println("Error during wav header creation");
    return;
  }

  // Initiate SD card to save data from microphone
  if (!SD_MMC.begin()) {
    Serial.println("Card Mount Failed");
    return;
  }

  // Buffer to receive data from microphone
  const size_t BUFFER_SIZE = 500;
  uint8_t* buf = (uint8_t*)malloc(BUFFER_SIZE);

  // Open created .wav file in append+binary mode to add PCM data
  File audio_file = SD_MMC.open(song_name, FILE_APPEND);
  if (audio_file == NULL) {
    Serial.println("Failed to create file");
    return;
  }

  // data size in bytes - > this amount of data should be recorded from microphone
  uint32_t data_size = VAD_SAMPLE_RATE_HZ * I2S_BITS_PER_SAMPLE_16BIT * duration / 8;

  // Record until "file_size" bytes have been read from mic.
  uint32_t counter = 0;
  uint32_t bytes_written;
  Serial.println("Recording started");
  while (counter != data_size) {
    // Check for file size overflow
    if (counter > data_size) {
      Serial.println("File is corrupted. data_size must be multiple of BUFFER_SIZE. Please modify BUFFER_SIZE");
      break;
    }

    // Read data from microphone
    if (i2s_read(I2S_CH, buf, BUFFER_SIZE, &bytes_written, portMAX_DELAY) != ESP_OK) {
      Serial.println("i2s_read() error");
    }

    if (bytes_written != BUFFER_SIZE) {
      Serial.println("Bytes written error");
    }

    // Save data to SD card
    audio_file.write( buf, BUFFER_SIZE);

    // Increment the counter
    counter += BUFFER_SIZE;
  }
  Serial.println("Recording finished");

  audio_file.close();
  free(buf);
  //SD_MMC.end();
}

