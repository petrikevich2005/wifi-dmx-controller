#include <WiFi.h>
#include <ArtnetWifi.h>
#include <HardwareSerial.h>
#include <ArduinoOTA.h>
#include "esp_timer.h"

const char* ssid = "DMX - controller";
const char* password = "password";

IPAddress local_IP(192, 168, 1, 100);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress primaryDNS(8, 8, 8, 8);
IPAddress secondaryDNS(8, 8, 4, 4);

ArtnetWifi artnet;
HardwareSerial DMXSerial(2);

#define DMX_PIN 17   // TX на SP3485
#define ENABLE_PIN 4 // Управление передачей
#define WIFI_RECONNECT_TIMEOUT 60000 // Переподключение Wi-Fi через 60 секунд

unsigned long lastPacketTime = 0;
unsigned long lastReconnectAttempt = 0;

void setup() {
    Serial.begin(115200);
    if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
        Serial.println("Ошибка настройки статического IP");
    }
    WiFi.begin(ssid, password);
    connectWiFi();

    artnet.begin();
    artnet.setArtDmxCallback(onArtnetData);

    DMXSerial.begin(250000, SERIAL_8N2, -1, DMX_PIN);
    pinMode(ENABLE_PIN, OUTPUT);
    digitalWrite(ENABLE_PIN, LOW);
    
    ArduinoOTA.setPasswordHash("5f4dcc3b5aa765d61d8327deb882cf99");
    ArduinoOTA.begin();
}

void loop() {
    unsigned long currentTime = millis();
    
    if (artnet.read()) {
        lastPacketTime = currentTime;
    }

    if (currentTime - lastPacketTime > 10000) {
        Serial.println("Art-Net данные не поступают!");
    }
    
    checkWiFi();
    ArduinoOTA.handle();
}

void connectWiFi() {
    Serial.print("Подключение к Wi-Fi...");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWi-Fi подключён");
    Serial.print("IP-адрес: ");
    Serial.println(WiFi.localIP());
}

void checkWiFi() {
    if (WiFi.status() != WL_CONNECTED) {
        unsigned long currentMillis = millis();
        if (currentMillis - lastReconnectAttempt >= WIFI_RECONNECT_TIMEOUT) {
            Serial.println("Wi-Fi потерян! Переподключение...");
            WiFi.disconnect();
            delay(1000);
            WiFi.reconnect();
            lastReconnectAttempt = currentMillis;
        }
    }
}

void onArtnetData(uint16_t universe, uint16_t length, uint8_t sequence, uint8_t* data) {
    if (!data || length == 0) {
        Serial.println("Ошибка: Пустой пакет DMX!");
        return;
    }
    if (length > 512) {
        Serial.println("Предупреждение: Длина DMX пакета превышает 512 байт, обрезаем...");
        length = 512;
    }
    
    digitalWrite(ENABLE_PIN, HIGH);
    esp_delay_us(88);
    DMXSerial.write(0);
    DMXSerial.write(data, length);
    esp_delay_us(44);
    digitalWrite(ENABLE_PIN, LOW);
}
