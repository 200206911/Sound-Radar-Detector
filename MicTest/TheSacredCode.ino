#include <driver/i2s.h>

#define BCLK 4  //SCK
#define LRCL 5  //WS
#define DOUT 6  //SD 

void setup() {
  Serial.begin(115200);
  delay(200);


  i2s_config_t cfg = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = 16000,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = 128,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };

  i2s_pin_config_t pins = {
    .bck_io_num = BCLK,
    .ws_io_num = LRCL,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = DOUT
  };

  Serial.println("Installing I2S...");
  
  Serial.println(i2s_driver_install(I2S_NUM_0, &cfg, 0, NULL));
  Serial.println("Setting pins...");
  Serial.println(i2s_set_pin(I2S_NUM_0, &pins));
}

void loop() {
  int32_t sample;
  size_t bytes_read = 0;

  esp_err_t result = i2s_read(I2S_NUM_0, &sample, sizeof(sample), &bytes_read, 100);

  int32_t clean = sample >> 14; // normalize

  Serial.print("raw=");
  Serial.print(sample);
  Serial.print(" clean=");
  Serial.println(clean);

  delay(50);
}

