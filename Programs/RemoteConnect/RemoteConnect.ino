#include <WiFi.h>

const char* WIFI_SSID = "yourSSID";
const char* WIFI_PASS = "yourPassword";

WiFiServer telnetServer(23);
WiFiClient telnetClient;

void logPrint(const String &msg) {
  Serial.print(msg);
  if (telnetClient && telnetClient.connected()) {
    telnetClient.print(msg);
  }
}
void logPrintln(const String &msg) {
  Serial.println(msg);
  if (telnetClient && telnetClient.connected()) {
    telnetClient.println(msg);
  }
}

void setup() {
  Serial.begin(921600);

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting to WiFi");
  unsigned long wifiStart = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - wifiStart < 8000) {
    delay(250);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.print("Connected! IP address: ");
    Serial.println(WiFi.localIP());
    telnetServer.begin();
  } else {
    Serial.println();
    Serial.println("WiFi failed to connect - continuing without telemetry.");
  }

  IBusSerial.begin(115200, SERIAL_8N1, IBUS_RX_PIN, -1);
}

void loop() {
  if (telnetServer.hasClient()) {
    if (telnetClient && telnetClient.connected()) {
      telnetServer.available().stop(); // reject new client if one's already attached
    } else {
      telnetClient = telnetServer.available();
      Serial.println("Telnet client connected.");
    }
}
