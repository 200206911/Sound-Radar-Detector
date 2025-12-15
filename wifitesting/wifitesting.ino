#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>

const char* ssid = "TNT";
const char* password = "12345678";

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// Your 3 integers
int a = 1;
int b = 2;
int c = 3;

const char* html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<title>ESP32 Live Data</title>
<script>
let socket;

function init() {
  socket = new WebSocket("ws://" + window.location.hostname + "/ws");
  socket.onmessage = function(event) {
    // event.data contains something like: "12,47,89"
    const values = event.data.split(",");
    document.getElementById('a').innerText = values[0];
    document.getElementById('b').innerText = values[1];
    document.getElementById('c').innerText = values[2];
  };
}
</script>
</head>
<body onload="init()">
<h1>Live ESP32 Data</h1>
<p>A: <span id="a">0</span></p>
<p>B: <span id="b">0</span></p>
<p>C: <span id="c">0</span></p>
</body>
</html>
)rawliteral";

void setup(){
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  while(WiFi.status() != WL_CONNECTED){
    delay(500);
    Serial.print(".");
  }
  Serial.println(WiFi.localIP());

  // Serve webpage
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/html", html);
  });

  // Start WebSocket
  server.addHandler(&ws);
  server.begin();
}

void loop(){
  // Example logic: update values every second
  static unsigned long lastSend = 0;
  if (millis() - lastSend > 1000) {
    lastSend = millis();
    a++;  
    b += 2;
    c += 3;

    // Make message string
    String msg = String(a) + "," + String(b) + "," + String(c);
    
    // Push data to all connected browsers
    ws.textAll(msg);
  }
}
