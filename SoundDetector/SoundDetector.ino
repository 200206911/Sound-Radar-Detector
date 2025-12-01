#include <driver/i2s.h>
#include <ESP32Servo.h>

Servo myServo;

//////////////////////////////////////////////////////////////////////////////////////////////
// Settings
#define tmax 0.00029154518
#define pi 3.141592653589
#define SoS 343
#define SoundThreshold 2000

#define SAMPLE_RATE     32000
#define I2S_PORT        I2S_NUM_0
#define BUFFER_SAMPLES  256

// Safe pins for ESP32-C6
#define I2S_BCLK  2
#define I2S_LRCLK 3
#define I2S_DATA  6

#define BUTTON_PIN      12    
const int servoPin = 9;

int buttonState;
int lastButtonState = LOW;
int state = 1;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;

double maxL, maxR;
int Li, Ri;

// stereo buffer
int32_t i2s_buffer[BUFFER_SAMPLES * 2];

//////////////////////////////////////////////////////////////////////////////////////////////
double tri(int a, int b) {
  double t = (b - a) / (double)SAMPLE_RATE;
  if (t > tmax) t = tmax;
  if (t < -tmax) t = -tmax;
  return asin(t / tmax) * (180.0 / pi);
}

//////////////////////////////////////////////////////////////////////////////////////////////
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

//////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println(">>> BOOT OK <<<");

  myServo.setPeriodHertz(50);   
  myServo.attach(servoPin, 500, 2400); // min/max pulse width
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // I2S initialization (from working test code)
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT, // stereo
    .communication_format = I2S_COMM_FORMAT_I2S,
    .intr_alloc_flags = 0,
    .dma_buf_count = 4,
    .dma_buf_len = 512,
    .use_apll = false
  };

  i2s_pin_config_t pin_config = { .bck_io_num = I2S_BCLK,
    .ws_io_num = I2S_LRCLK,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_DATA };

  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_PORT, &pin_config);

  Serial.println("Stereo I2S microphones ready!");
}

//////////////////////////////////////////////////////////////////////////////////////////////
void loop() {

  size_t bytes_read = 0;
  maxL = maxR = 0;
  Li = Ri = 0;

  // Read stereo samples
  esp_err_t res = i2s_read(I2S_PORT, i2s_buffer, sizeof(i2s_buffer), &bytes_read, 50);
  if (bytes_read == 0) {
    Serial.println("No data yet from microphones");
    delay(50);
    return;
  }

  int samples_read = bytes_read / 8; // 4 bytes per channel

  for (int i = 0; i < samples_read; i++) {
    int32_t rawL = i2s_buffer[i * 2];
    int32_t rawR = i2s_buffer[i * 2 + 1];

    int16_t L = rawL >> 14;
    int16_t R = rawR >> 14;

    if (abs(L) > maxL && abs(L) > SoundThreshold) { maxL = abs(L); Li = i; }
    if (abs(R) > maxR && abs(R) > SoundThreshold) { maxR = abs(R); Ri = i; }
  }

  // Triangulation
  double angle = tri(Li, Ri);
  myServo.write(angle);

  Serial.print("L peak: "); Serial.print(Li);
  Serial.print("  R peak: "); Serial.print(Ri);
  Serial.print("  Angle: "); Serial.println(angle);

  // Optional: check button
  if (buttonPressed()) {
    Serial.println("Button pressed");
    state = 1;
  }
}
