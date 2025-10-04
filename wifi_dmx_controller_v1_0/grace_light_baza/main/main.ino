#include <WiFi.h>
#include <WiFiUdp.h>
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_system.h"
#include "esp_timer.h"
#include <EEPROM.h>
#include "SPIFFS.h"
#include <WebServer.h>
#include <ArduinoJson.h>

// TIMER INIT
esp_timer_handle_t dmx_timer;

// EEPROM
#define EEPROM_SIZE 512
#define RESET_PIN GPIO_NUM_12  // пин сброса EEPROM
struct Settings {
  char ssid[32];
  char password[32];
  uint16_t universe;
};
Settings settings;

// LED SETTINGS
#define STATUS_LED_PIN GPIO_NUM_2
bool ledState = false;
unsigned long lastLedUpdate = 0;

// DMX SETTINGS
#define DMX_UART_NUM     UART_NUM_1
#define DMX_TX_PIN       GPIO_NUM_17
#define DMX_DE_RE_PIN    GPIO_NUM_4
#define DMX_CHANNELS     512
#define DMX_PACKET_SIZE  (1 + DMX_CHANNELS)  // старт-код + 512 каналов

//ART_NET SETTINGS
#define ART_NET_PORT     6454
#define ART_NET_ID       "Art-Net\0"
#define ART_DMX_OPCODE   0x5000

uint8_t dmx_data[DMX_PACKET_SIZE] = {0};
WiFiUDP Udp;

// WEB SERVER
WebServer server(80);

// ==== Загрузка настроек из EEPROM ====
void loadSettings() {
  EEPROM.get(0, settings);

  // Проверка: если пусто или битые данные → дефолт
  if (strlen(settings.ssid) == 0 || strlen(settings.password) == 0) {
    setDefaultSettings();
  }
}

// ==== Сохранение настроек ====
void saveSettings() {
  EEPROM.put(0, settings);
  EEPROM.commit();
}

// ==== Настройки по умолчанию ====
void setDefaultSettings() {
  strcpy(settings.ssid, "wifi-dmx-device");
  strcpy(settings.password, "grace_dmx512");
  settings.universe = 0;
  saveSettings();
}

// === TIMER CALLBACK ===
void IRAM_ATTR dmx_timer_callback(void* arg) {
  send_dmx();
}

// ==== DMX ПЕРЕДАЧА ====
void send_dmx() {
  // Включаем передачу через SP3485
  gpio_set_level(DMX_DE_RE_PIN, 1);  

  // === BREAK ===
  gpio_set_direction(DMX_TX_PIN, GPIO_MODE_OUTPUT); // отключаем UART
  gpio_set_level(DMX_TX_PIN, 0);                     // логический 0
  delayMicroseconds(110);                            // длительность BREAK (>88 мкс)

  // === MAB (MARK AFTER BREAK) ===
  gpio_set_level(DMX_TX_PIN, 1);                     // логическая 1
  delayMicroseconds(12);                             // длительность MAB (≥8 мкс)

  // === Передача пакета ===
  uart_set_pin(DMX_UART_NUM, DMX_TX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE); // возвращаем UART
  uart_write_bytes(DMX_UART_NUM, (const char*)dmx_data, DMX_PACKET_SIZE);
  uart_wait_tx_done(DMX_UART_NUM, pdMS_TO_TICKS(10)); // ждём завершения передачи

  // === Межкадровая пауза (Inter-frame gap) ===
  delayMicroseconds(44); // минимум 44 мкс между фреймами по DMX-стандарту

  // Выключаем передачу
  gpio_set_level(DMX_DE_RE_PIN, 0);  
}

// ==== Создание точки доступа с фиксированным IP ====
void startAccessPoint() {
  WiFi.mode(WIFI_AP);

  // Настраиваем статический IP для AP интерфейса
  IPAddress local_ip(192,168,0,1);
  IPAddress gateway(192,168,0,1);
  IPAddress subnet(255,255,255,0);
  WiFi.softAPConfig(local_ip, gateway, subnet);

  bool apOk = WiFi.softAP(settings.ssid, settings.password);
  if (!apOk) {
    esp_restart();
  }
}

// === TIMER SETUP ===
void timerSetup() {
    // ==== Настройка таймера отправки DMX ====  
  const esp_timer_create_args_t dmx_timer_args = {
    .callback = &dmx_timer_callback,
    .arg = nullptr,
    .dispatch_method = ESP_TIMER_TASK,  // работает в фоне, безопасен с delayMicroseconds()
    .name = "dmx_timer"
  };

  esp_err_t res = esp_timer_create(&dmx_timer_args, &dmx_timer);
  if (res != ESP_OK) {
    esp_restart();  // или обработать иначе
  }

  // Запуск таймера с периодом 25000 мкс (25 мс = 40 Гц)
  // Запуск таймера с периодом 33000 мкс (33 мс = 30 Гц)
  esp_timer_start_periodic(dmx_timer, 33000);
}

// ==== SETUP ====
void setup() {
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, HIGH);

  // === Reset pin check ===
  pinMode(RESET_PIN, INPUT_PULLUP);
  EEPROM.begin(EEPROM_SIZE);
  if (digitalRead(RESET_PIN) == LOW) {
    setDefaultSettings();
    for (int i=0; i<3; i++)
      digitalWrite(STATUS_LED_PIN, LOW);
      delay(200);
      digitalWrite(STATUS_LED_PIN, HIGH);
    ESP.restart();
  }

  delay(5);

  // Загружаем настройки
  loadSettings();

  // Поднимаем точку
  startAccessPoint();

  // ArtNet UDP
  Udp.begin(ART_NET_PORT);

  // RS-485 управление
  pinMode(DMX_DE_RE_PIN, OUTPUT);
  digitalWrite(DMX_DE_RE_PIN, LOW);

  // TX pin по умолчанию HIGH
  pinMode(DMX_TX_PIN, OUTPUT);
  digitalWrite(DMX_TX_PIN, HIGH);

  // UART конфиг
  uart_config_t config = {
    .baud_rate = 250000,
    .data_bits = UART_DATA_8_BITS,
    .parity    = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_2,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
  };
  uart_driver_install(DMX_UART_NUM, 1024, 0, 0, NULL, 0);
  uart_param_config(DMX_UART_NUM, &config);
  uart_set_pin(DMX_UART_NUM, DMX_TX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

  // DMX старт-код
  dmx_data[0] = 0x00;

  // WEB SERVER
  SPIFFS.begin(true);
  server.on("/", HTTP_GET, [](){
    File f = SPIFFS.open("/index.html", "r");
    server.streamFile(f, "text/html");
    f.close();
  });

  server.on("/saveSettings", HTTP_POST, [](){
    // читаем JSON, проверяем update code, сохраняем EEPROM

  });

  server.on("/restart", HTTP_POST, [](){
    server.send(200, "application/json", "{\"status\":\"ok\",\"msg\":\"Restarting\"}");
    delay(500);
    ESP.restart();
  });

  // остальные обработчики: /restart, /setMode, /saveScene, /loadScene

  server.begin();

  digitalWrite(STATUS_LED_PIN, LOW);
  delay(2);
  timerSetup();
  delay(2);
}

// === GET ART-NET DATA ===
void getArtnetData() {
  int packetSize = Udp.parsePacket();
  if (packetSize > 0) {
    uint8_t buffer[600];
    int len = Udp.read(buffer, sizeof(buffer));
    if (len >= 18 && memcmp(buffer, ART_NET_ID, 8) == 0) {
      uint16_t opcode = buffer[8] | (buffer[9] << 8);
      if (opcode == ART_DMX_OPCODE) {
        uint16_t universe = buffer[14] | (buffer[15] << 8);
        if (universe == settings.universe) {
          uint16_t dmxLen = (buffer[16] << 8) | buffer[17];
          if (dmxLen > DMX_CHANNELS) dmxLen = DMX_CHANNELS;
          memcpy(&dmx_data[1], &buffer[18], dmxLen);  // dmx_data[0] = старт-код
        }
      }
    }
  }
}

// ==== LOOP ====
void loop() {
  server.handleClient();
  getArtnetData();
}