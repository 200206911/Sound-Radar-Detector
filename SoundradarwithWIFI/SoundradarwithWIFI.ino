#include <driver/i2s.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define SAMPLE_RATE     32000
#define I2S_PORT        I2S_NUM_0
#define BUFFER_SAMPLES  256
// ESP32-C6 pins////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define I2S_BCLK  2     //sck
#define I2S_LRCLK 3     //ws
#define I2S_DATA  6     //sd
#define BUTTON_PIN    5
#define servoPin    4
#define Led        14
#define TriggerOutPin 7
Servo myServo;
int32_t i2s_buffer[BUFFER_SAMPLES*2]; // stereo buffer: L + R
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define MIC_DISTANCE 0.145
#define SoS 343
#define tmax (MIC_DISTANCE / SoS)
#define pi 3.141592653589
#define LSoundThreshold 50
#define RSoundThreshold 50
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
const char* ssid = "TNT";
const char* password = "12345678";
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

const char* html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<title>ESP32 Live Data</title>
<style>
  body { font-family: Arial, sans-serif; text-align: center; margin-top: 50px; }
  .value { font-size: 24px; margin: 10px 0; }
</style>
<script>
let socket;

function init() {
  socket = new WebSocket("ws://" + window.location.hostname + "/ws");
  
  socket.onopen = function() {
    console.log("WebSocket connected");
  };

  socket.onmessage = function(event) {
    const values = event.data.split(",");  // expects "maxL,maxR,angle"
    document.getElementById('a').innerText = "Left Mic: " + values[0];
    document.getElementById('b').innerText = "Right Mic: " + values[1];
    document.getElementById('c').innerText = "Angle: " + values[2];
  };

  socket.onclose = function() {
    console.log("WebSocket disconnected. Refresh the page to reconnect.");
  };
}
</script>
</head>
<body onload="init()">
<h1>ESP32 Live Data</h1>
<div id="a" class="value">Left Mic: 0</div>
<div id="b" class="value">Right Mic: 0</div>
<div id="c" class="value">Angle: 0</div>
</body>
</html>
)rawliteral";
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float currentAngle = 0;       // current approximate angle
int n=0;
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int buttonState;
int lastButtonState = LOW;
int state = 1;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
double maxL, maxR;
int Li, Ri;
double angle;
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
double tri(int a, int b) {
  double t = (b - a) / (double)SAMPLE_RATE;
  if (t > tmax) t = tmax;
  if (t < -tmax) t = -tmax;
  return asin(t / tmax) * (180.0 / pi);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  while(WiFi.status() != WL_CONNECTED && n < 20){
    delay(500);
    Serial.print(".");
    n++;
  }
  Serial.println(WiFi.localIP());
  // Serve webpage
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/html", html);
  });
  // Start WebSocket
  server.addHandler(&ws);
  server.begin();

  myServo.setPeriodHertz(50);
  myServo.attach(servoPin, 500, 2400);
  Serial.begin(115200);
  delay(500);
  Serial.println(">>> Stereo INMP441 Test <<<");
  pinMode(TriggerOutPin, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLDOWN);
  pinMode(Led, OUTPUT);
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
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
  static unsigned long lastSend = 0;
 switch (state){
  case 1:
   digitalWrite(Led, LOW);
    delay(400);
  if(buttonPressed()){
    digitalWrite(TriggerOutPin, HIGH);
    myServo.write(0);
    delay(2000);
    state = 2;
  }
   break;
  case 2:
      if(buttonPressed()){
           digitalWrite(TriggerOutPin, LOW);
    state = 1;
    break;}
 // myServo.write(0);
 digitalWrite(Led, HIGH);
  size_t bytes_read = 0;
  maxL = maxR = 0;
  Li = Ri = 0;
  angle = 0;
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
    if (abs(L) > maxL && abs(L) > LSoundThreshold) { maxL = abs(L); Li = i; }
    if (abs(R) > maxR && abs(R) > RSoundThreshold) { maxR = abs(R); Ri = i; }
  }
  if((int)maxL == 0 || (int)maxR == 0){
  break;}
  if((int)maxL != 0 || (int)maxR != 0){
       angle = tri(Li, Ri);
  }
   if((int)angle != 90 && (int)angle != -90 && (int)angle != 0){
  Serial.print("L peak: "); Serial.print(maxL);
    Serial.print("L index: "); Serial.print(Li);
  Serial.print("  R peak: "); Serial.print(maxR);
  Serial.print("R index: "); Serial.print(Ri);

  Serial.print("  Angle: "); Serial.println(90-angle);
 myServo.write(90 + angle);
  // digitalWrite(TriggerOutPin, HIGH);
  angle = 90  + angle;
 String msg = String(maxL) + "," + String(maxR) + "," + String(angle);
  ws.textAll(msg);
 delay(3000);}


 }

   
}
