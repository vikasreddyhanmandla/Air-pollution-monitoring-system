
#include <WiFi.h>
#include <HTTPClient.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <MQUnifiedsensor.h>

// Define Wi-Fi credentials and ThingSpeak API
const char* ssid = "Your_WiFi_SSID";
const char* password = "Your_WiFi_Password";
String thingSpeakAPIKey = "YOUR_THINGSPEAK_API_KEY";
String thingSpeakServer = "https://api.thingspeak.com/update";

// Sensor pins and constants
#define DHTPIN 4              // GPIO4 for DHT11/DHT22 data pin
#define DHTTYPE DHT22         // DHT sensor type (DHT11 or DHT22)
#define MQ135Pin 34           // Analog input pin for MQ135
#define PM2_5_RX_PIN 16       // RX pin for PMS5003

DHT dht(DHTPIN, DHTTYPE);

// Initialize sensors
MQUnifiedsensor MQ135("ESP32", 3.3, 4095, MQ135Pin, "MQ-135");
HardwareSerial PMSerial(2);

// Wi-Fi connection setup
void setupWiFi() {
  Serial.print("Connecting to Wi-Fi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("Connected!");
}

// Initialize all sensors
void setup() {
  Serial.begin(115200);
  dht.begin();
  setupWiFi();

  // Initialize MQ135 sensor
  MQ135.setRegressionMethod(1);     // Set regression method
  MQ135.init();
  MQ135.setRL(10);                  // Load resistance in kilo ohms

  // PMS5003 setup
  PMSerial.begin(9600, SERIAL_8N1, PM2_5_RX_PIN, -1);
}

// Function to read PM2.5 data
float readPM25() {
  if (PMSerial.available()) {
    byte buffer[32];
    PMSerial.readBytes(buffer, 32);
    float pm25 = buffer[12] << 8 | buffer[13];
    return pm25 / 10.0;
  }
  return -1;  // If reading fails
}

// Function to upload data to ThingSpeak
void uploadData(float temp, float hum, float co2, float pm25) {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    String url = thingSpeakServer + "?api_key=" + thingSpeakAPIKey
                 + "&field1=" + String(temp)
                 + "&field2=" + String(hum)
                 + "&field3=" + String(co2)
                 + "&field4=" + String(pm25);

    http.begin(url);
    int httpResponseCode = http.GET();
    if (httpResponseCode > 0) {
      Serial.println("Data sent successfully!");
    } else {
      Serial.println("Error sending data.");
    }
    http.end();
  }
}

void loop() {
  // Read DHT sensor data
  float temp = dht.readTemperature();
  float hum = dht.readHumidity();

  // Read CO2 levels from MQ135
  MQ135.update();
  float co2 = MQ135.readSensor();

  // Read PM2.5 data from PMS5003
  float pm25 = readPM25();

  // Print values
  Serial.print("Temp: "); Serial.print(temp); Serial.print("°C ");
  Serial.print("Humidity: "); Serial.print(hum); Serial.print("% ");
  Serial.print("CO2: "); Serial.print(co2); Serial.print("ppm ");
  Serial.print("PM2.5: "); Serial.print(pm25); Serial.println("µg/m³");

  // Send data to ThingSpeak
  uploadData(temp, hum, co2, pm25);

  // Wait 15 seconds before the next reading
  delay(15000);
}
