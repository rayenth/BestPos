#include <WiFi.h>
#include <PubSubClient.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>

/* ===================== CONFIG ===================== */

// WiFi credentials
const char* WIFI_SSID = "YOUR_WIFI";
const char* WIFI_PASS = "YOUR_PASSWORD";

// MQTT broker
const char* MQTT_BROKER = "192.168.100.122";
const int   MQTT_PORT   = 1883;
const char* MQTT_TOPIC  = "owntracks/esp32/filtered";

// GPS UART pins (UART2)
#define GPS_RX 16
#define GPS_TX 17

/* ===================== OBJECTS ===================== */

WiFiClient espClient;
PubSubClient mqttClient(espClient);
TinyGPSPlus gps;
HardwareSerial gpsSerial(2);

/* ===================== KALMAN FILTER ===================== */

class KalmanFilter {
  public:
    float estimate;
    float errorEstimate;
    float errorMeasure;
    float q;

    KalmanFilter(float q, float r, float p, float initial) {
      this->q = q;
      this->errorMeasure = r;
      this->errorEstimate = p;
      this->estimate = initial;
    }

    float update(float measurement) {
      errorEstimate += q;
      float gain = errorEstimate / (errorEstimate + errorMeasure);
      estimate += gain * (measurement - estimate);
      errorEstimate *= (1 - gain);
      return estimate;
    }
};

// Kalman filters for coordinates
KalmanFilter kalmanLat(0.0001, 0.01, 1, 0);
KalmanFilter kalmanLon(0.0001, 0.01, 1, 0);
KalmanFilter kalmanAlt(0.1, 1, 1, 0);

/* ===================== WIFI + MQTT ===================== */

void connectWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
}

void connectMQTT() {
  while (!mqttClient.connected()) {
    mqttClient.connect("esp32-gps-kalman");
    delay(1000);
  }
}

/* ===================== WIFI SCAN ===================== */

int getAverageRSSI() {
  int n = WiFi.scanNetworks();
  int rssiSum = 0;

  for (int i = 0; i < n; i++) {
    rssiSum += WiFi.RSSI(i);
  }

  return (n > 0) ? rssiSum / n : -100;
}

/* ===================== SETUP ===================== */

void setup() {
  Serial.begin(115200);

  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);

  connectWiFi();
  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
}

/* ===================== LOOP ===================== */

void loop() {

  // Read GPS data
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }

  if (gps.location.isUpdated()) {

    float rawLat = gps.location.lat();
    float rawLon = gps.location.lng();
    float rawAlt = gps.altitude.meters();

    // Wi-Fi signal used as stability indicator
    int avgRSSI = getAverageRSSI();

    // Adapt Kalman noise dynamically
    kalmanLat.q = (avgRSSI > -60) ? 0.00005 : 0.0002;
    kalmanLon.q = (avgRSSI > -60) ? 0.00005 : 0.0002;

    float filtLat = kalmanLat.update(rawLat);
    float filtLon = kalmanLon.update(rawLon);
    float filtAlt = kalmanAlt.update(rawAlt);

    connectMQTT();

    char payload[256];
    snprintf(payload, sizeof(payload),
      "{"
      "\"id\":\"person1\","
      "\"lat\":%.6f,"
      "\"lon\":%.6f,"
      "\"alt\":%.2f,"
      "\"wifi_rssi\":%d"
      "}",
      filtLat, filtLon, filtAlt, avgRSSI
    );

    mqttClient.publish(MQTT_TOPIC, payload);

    Serial.println(payload);
  }

  mqttClient.loop();
  delay(1000);
}
