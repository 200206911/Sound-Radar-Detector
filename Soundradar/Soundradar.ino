#include <driver/i2s.h>
#include <ESP32Servo.h>

Servo myServo;

#define SAMPLE_RATE     32000
#define I2S_PORT        I2S_NUM_0
#define BUFFER_SAMPLES  256

// Safe ESP32-C6 pins
#define I2S_BCLK  2
#define I2S_LRCLK 3
#define I2S_DATA  6
#define BUTTON_PIN    12 
#define servoPin    9  

int32_t i2s_buffer[BUFFER_SAMPLES*2]; // stereo buffer: L + R

#define tmax 0.00029154518
#define pi 3.141592653589
#define SoS 343
#define SoundThreshold 10

int buttonState;
int lastButtonState = LOW;
int state = 1;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;

double maxL, maxR;
int Li, Ri;


double tri(int a, int b) {
  double t = (b - a) / (double)SAMPLE_RATE;
  if (t > tmax) t = tmax;
  if (t < -tmax) t = -tmax;
  return asin(t / tmax) * (180.0 / pi);
}

bool buttonPressed() {
  int reading = digitalRead(BUTTON_PIN);
  bool pressed = false;
  if (reading != lastButtonState) lastDebounceTime = millis();
  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != buttonState) {
      buttonState = reading;
      if (buttonState == HIGH) pressed = true;
    }
  }
  lastButtonState = reading;
  return pressed;
}


void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println(">>> Stereo INMP441 Test <<<");

  // I2S configuration
  i2s_config_t cfg = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT, // stereo
    .communication_format = I2S_COMM_FORMAT_I2S,
    .intr_alloc_flags = 0,
    .dma_buf_count = 4,
    .dma_buf_len = 512,  // bigger buffer for stereo
    .use_apll = false
  };

  i2s_pin_config_t pins = {
    .bck_io_num = I2S_BCLK,
    .ws_io_num = I2S_LRCLK,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_DATA
  };

  // Install and start I2S
  i2s_driver_install(I2S_PORT, &cfg, 0, NULL);
  i2s_set_pin(I2S_PORT, &pins);

  Serial.println("I2S initialized! Reading both mics...");
}

void loop() {
 switch (state){
  case 1;

  if(buttonPressed()){
    state = 2;
    break;
  }

  case 2;
 
  size_t bytes_read = 0;
  maxL = maxR = 0;
  Li = Ri = 0;
  // Non-blocking read (50ms timeout)
  esp_err_t res = i2s_read(I2S_PORT, i2s_buffer, sizeof(i2s_buffer), &bytes_read, 50);
  if(res != ESP_OK) {
    Serial.print("I2S read error: "); Serial.println(res);
    delay(50);
    return;
  }

  if(bytes_read == 0) {
    Serial.println("No data yet from microphones");
    delay(50);
    return;
  }

  int frames = bytes_read / 8; // 4 bytes left + 4 bytes right
  for(int i = 0; i < frames; i++) {
    int32_t L = i2s_buffer[i*2] >> 14;     // Left mic
    int32_t R = i2s_buffer[i*2 + 1] >> 14; // Right mic
   // Serial.print("Frame "); Serial.print(i);
   // Serial.print(": L="); Serial.print(L);
   // Serial.print(" R="); Serial.println(R);
    if (abs(L) > maxL && abs(L) > SoundThreshold) { maxL = abs(L); Li = i; }
    if (abs(R) > maxR && abs(R) > SoundThreshold) { maxR = abs(R); Ri = i; }
  }
    double angle = tri(Li, Ri);
//  myServo.write(angle);

  
if(angle < 89 && angle > -89){  Serial.print("L peak: "); Serial.print(maxL);
  Serial.print("  R peak: "); Serial.print(maxR);
  Serial.print("  Angle: "); Serial.println(90+angle);}
  delay(50);

    if(buttonPressed()){
    state = 1;
    break;
                      }
 }

   
}
