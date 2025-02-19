#include <WiFi.h>
#include <ArtnetWifi.h>
#include <HardwareSerial.h>
#include <ArduinoOTA.h>
#include "esp_timer.h"

const char* ssid = "DMX - controller";
const char* password = "password";

// Настройки статического IP
IPAddress local_IP(192, 168, 1, 100);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress primaryDNS(8, 8, 8, 8);
IPAddress secondaryDNS(8, 8, 4, 4);

ArtnetWifi artnet;
HardwareSerial DMXSerial(2); // UART2 для MAX485

#define DMX_PIN 17   // TX на MAX485
#define ENABLE_PIN 4 // Управление передачей

unsigned long lastPacketTime = 0;

void setup() {
    Serial.begin(115200);
    
    // Настройка статического IP
    if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
        Serial.println("Ошибка настройки статического IP");
    }
    
    WiFi.begin(ssid, password);
    unsigned long startAttemptTime = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 10000) {
        delay(500);
        Serial.print(".");
    }
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("\nWiFi failed! Restarting...");
        ESP.restart();
    }
    
    Serial.println("\nWiFi Connected");
    Serial.print("IP-адрес: ");
    Serial.println(WiFi.localIP());
    
    artnet.begin();
    artnet.setArtDmxCallback(onArtnetData);

    DMXSerial.begin(250000, SERIAL_8N2, -1, DMX_PIN); // DMX: 250000 бод, 8 бит, 2 стоп-бита
    pinMode(ENABLE_PIN, OUTPUT);
    digitalWrite(ENABLE_PIN, LOW); // Включаем приём
    
    // Инициализация OTA с паролем
    ArduinoOTA.setPasswordHash("5f4dcc3b5aa765d61d8327deb882cf99"); // MD5-хеш пароля "password"
    ArduinoOTA.begin();
}

void loop() {
    unsigned long currentTime = millis();
    
    if (artnet.read()) {
        lastPacketTime = currentTime;
    }

    if (currentTime - lastPacketTime > 10000) { // Если данных нет более 10 сек
        Serial.println("Art-Net данные не поступают!");
    }
    
    checkWiFi();
    ArduinoOTA.handle();
}

void checkWiFi() {
    static unsigned long lastCheck = 0;
    static int reconnectAttempts = 0;
    
    if (millis() - lastCheck > 10000) { // Проверяем раз в 10 секунд
        lastCheck = millis();
        
        if (WiFi.status() != WL_CONNECTED) {
            Serial.println("Соединение потеряно! Переподключение...");
            WiFi.disconnect();
            delay(1000);
            WiFi.reconnect();
            
            reconnectAttempts++;
            if (reconnectAttempts >= 5) { // После 5 неудачных попыток увеличиваем интервал
                Serial.println("Wi-Fi не восстановлено. Ждем 60 сек перед повторной попыткой...");
                delay(60000);
                reconnectAttempts = 0;
            }
        } else {
            reconnectAttempts = 0; // Сбрасываем счетчик при успешном подключении
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
    
    digitalWrite(ENABLE_PIN, HIGH); // Включаем передачу
    esp_delay_us(88); // Оптимизированная задержка
    DMXSerial.write(0); // Старт-код DMX
    DMXSerial.write(data, length); // Отправка данных
    delay(1); // Короткая задержка для корректного завершения передачи
    digitalWrite(ENABLE_PIN, LOW); // Отключаем передачу
}

// TODO: Добавить проверку уровня напряжения питания ESP32 через ADC
