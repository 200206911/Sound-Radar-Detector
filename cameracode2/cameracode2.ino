#include <SPI.h>
#include <TFT_eSPI.h>       
#include <TJpg_Decoder.h>   
#include "esp_camera.h"

// =================== CONFIGURATION & PIN DEFINITIONS ===================

// Camera Pins (AI THINKER Model)
#define PWDN_GPIO_NUM    32
#define RESET_GPIO_NUM   -1
#define XCLK_GPIO_NUM    0
#define SIOD_GPIO_NUM    26
#define SIOC_GPIO_NUM    27
#define Y9_GPIO_NUM      35
#define Y8_GPIO_NUM      34
#define Y7_GPIO_NUM      39
#define Y6_GPIO_NUM      36
#define Y5_GPIO_NUM      21
#define Y4_GPIO_NUM      19
#define Y3_GPIO_NUM      18
#define Y2_GPIO_NUM      5
#define VSYNC_GPIO_NUM   25
#define HREF_GPIO_NUM    23
#define PCLK_GPIO_NUM    22

// Control Pins
#define FLASH_LED_PIN    4      // On-board white LED (Flash)
#define START_VIDEO_PIN  12     // GPIO 12 for external start/stop trigger

// Thresholds
#define LIMITE_ESCURIDAO 40     // Darkness limit (0 to 255)

// =======================================================================

TFT_eSPI tft = TFT_eSPI(); 

// Variables for brightness calculation
long somaBrilho = 0;
long totalPixels = 0;
bool isRecording = false; // Flag to control video capture

// This function is called automatically by TJpg_Decoder to draw the image
bool tft_output(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t* bitmap) {
    
    // Check if drawing area is off-screen
    if (y >= tft.height()) return 0;
    tft.pushImage(x, y, w, h, bitmap);

    // --- BRIGHTNESS CALCULATION ---
    // Samples pixels to estimate image brightness
    for(int i = 0; i < w * h; i += 10) { 
      uint16_t color = bitmap[i];
      
      // Extract Green component from RGB565 color (G is the 6 bits in the middle)
      int g = (color >> 5) & 0x3F; 
      
      // Normalize G component (0-63) to approx 0-255
      somaBrilho += (g * 4);
      totalPixels++;
    }

    return 1;
}

void setup() {
    Serial.begin(115200);
    Serial.println("\nInitializing...");

    // Configure the video control pin
    pinMode(START_VIDEO_PIN, INPUT_PULLDOWN);

    // Configure the Flash LED
    pinMode(FLASH_LED_PIN, OUTPUT);
    digitalWrite(FLASH_LED_PIN, LOW);

    // Initialize the Display (TFT_eSPI setup must be in User_Setup.h)
    tft.init();
    tft.setRotation(1); 
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextSize(2);
    tft.setCursor(0, 0);
    tft.println("Camera TFT");
    delay(1000);

    // Initialize the JPEG Decoder
    TJpgDec.setJpgScale(1);
    TJpgDec.setSwapBytes(true);
    TJpgDec.setCallback(tft_output);

    // Camera Configuration
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sscb_sda = SIOD_GPIO_NUM;
    config.pin_sscb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_JPEG;
    config.frame_size = FRAMESIZE_QQVGA; 
    config.jpeg_quality = 10; 
    config.fb_count = 1;

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
      tft.fillScreen(TFT_RED);
      tft.println("Camera Error!");
      Serial.printf("Camera init failed with error 0x%x", err);
      return;
    }
}

void loop() {
  // 1. Logic to check the input pin (GPIO 12) and update the isRecording flag
  int pinState = digitalRead(START_VIDEO_PIN);
  
  if (pinState == HIGH && isRecording == false) {
    // Pin went HIGH: Start recording
    isRecording = true;
    tft.fillScreen(TFT_BLACK);
    Serial.println("Pin HIGH: Video Started");
  } else if (pinState == LOW && isRecording == true) {
    // Pin went LOW: Stop recording
    isRecording = false;
    tft.fillScreen(TFT_BLACK); 
    tft.setCursor(0, 0);
    tft.println("Video Stopped");
    Serial.println("Pin LOW: Video Stopped");
  }
  
  // 2. Check if the other ESP32 sent a command via Serial (Optional control)
  if (Serial.available() > 0) {
    char command = Serial.read();
    
    if (command == '1') {
      isRecording = true;
      Serial.println("Serial Command: Video Started");
    } 
    else if (command == '0') {
      isRecording = false;
      Serial.println("Serial Command: Video Stopped");
      tft.fillScreen(TFT_BLACK); 
    }
  }

  // 3. THE IF STATEMENT (Video capture loop)
  if (isRecording == true) {
      
      // Reset brightness variables
      totalPixels = 0;

      // Capture 
      camera_fb_t * fb = esp_camera_fb_get();
      if (!fb) return;

      // Draw (This calls tft_output)
      TJpgDec.drawJpg(0, 20, fb->buf, fb->len);
      
      // Release 
      esp_camera_fb_return(fb);
      
  }
}