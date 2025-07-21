#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_system.h"
#include <EEPROM.h>
#include <WebServer.h>

// ==== НАСТРОЙКИ ====
#define EEPROM_SIZE 512 // размер EEPROM
#define RESET_PIN GPIO_NUM_12

//WIFI SETTINGS
bool apMode = false;
unsigned long wifiLostTime = 0;

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
#define ART_DMX_OPCODE   0x5000

uint8_t dmx_data[DMX_PACKET_SIZE] = {0};
WiFiUDP Udp;

int loopCounter = 0;

WebServer server(80);

// reset settings
void checkResetButton() {
  pinMode(RESET_PIN, INPUT_PULLUP);
  if (digitalRead(RESET_PIN) == LOW) {
    delay(500);  // защита от случайного срабатывания
    if (digitalRead(RESET_PIN) == LOW) {
      for (int i = 0; i < EEPROM_SIZE; i++) EEPROM.write(i, 0xFF);
      EEPROM.commit();
      ESP.restart();
    }
  }
}


// структура данных для сохранения в EEPROM
struct Config {
  char ssid[32];
  char password[32];
  uint16_t artnet_port;
  char artnet_id[12];
  uint16_t universe;
  uint32_t loop_check_point;
  uint32_t wifiRetryWindow;
  uint32_t startTimeoutConnetion;
  IPAddress local_ip;
  IPAddress gateway;
  IPAddress subnet;
  uint8_t device_index;
};

Config config;

// загружаем данные
void loadConfig() {
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.get(0, config);
  if (strlen(config.ssid) == 0 || config.ssid[0] == 0xFF) {
    // Устанавливаем значения по умолчанию
    strcpy(config.ssid, "grace_light");
    strcpy(config.password, "grace_dmx512");
    config.artnet_port = 6454;
    strcpy(config.artnet_id, "Art-Net\0");
    config.universe = 0;
    config.loop_check_point = 50;
    config.wifiRetryWindow = 10000;
    config.startTimeoutConnetion = 4000;
    config.local_ip = IPAddress(192, 168, 0, 1);
    config.gateway = IPAddress(192, 168, 0, 1);
    config.subnet = IPAddress(255, 255, 255, 0);
    config.device_index = 1;
    saveConfig();
  }
}

// запись в EEPROM
void saveConfig() {
  EEPROM.put(0, config);
  EEPROM.commit();
}

// ==== DMX ПЕРЕДАЧА ====
void send_dmx() {
  gpio_set_level(DMX_DE_RE_PIN, 1);  // Включаем передачу

  // BREAK (LOW >88 мкс)
  gpio_set_direction(DMX_TX_PIN, GPIO_MODE_OUTPUT);
  delayMicroseconds(110);

  // MAB (MARK AFTER BREAK)
  delayMicroseconds(12);

  // Назад в UART
  uart_set_pin(DMX_UART_NUM, DMX_TX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  uart_write_bytes(DMX_UART_NUM, (const char*)dmx_data, DMX_PACKET_SIZE);
  uart_wait_tx_done(DMX_UART_NUM, pdMS_TO_TICKS(10));

  gpio_set_level(DMX_DE_RE_PIN, 0);  // Выключаем передачу
}

// ==== Функция подключения к WiFi с таймаутом 6 секунд ====
bool connectToWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(config.ssid, config.password);

  unsigned long timeout = config.startTimeoutConnetion * config.device_index;
  unsigned long startAttemptTime = millis();

  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < timeout) {
    delay(200);
  }

  return WiFi.status() == WL_CONNECTED;
}

// ==== Создание точки доступа с фиксированным IP ====
void startAccessPoint() {

  WiFi.mode(WIFI_OFF);
  WiFi.mode(WIFI_AP);

  // Настраиваем статический IP для AP интерфейса
  IPAddress local_ip(192,168,0,1);
  IPAddress gateway(192,168,0,1);
  IPAddress subnet(255,255,255,0);
  WiFi.softAPConfig(local_ip, gateway, subnet);

  bool apOk = WiFi.softAP(config.ssid, config.password);
  if (apOk) {
    apMode = true;
  } else {
    esp_restart();
  }
}

// =====  WiFi UPDATE  =====
void checkWiFiStatus() {
  wl_status_t status = WiFi.status();

  if (status == WL_CONNECTED) {
    wifiLostTime = 0;  // всё ок
    return;
  }

  if (apMode) return;  // если уже AP, ничего не делаем

  // если впервые потеряли соединение — запоминаем время
  if (wifiLostTime == 0) {
    wifiLostTime = millis();
    return;
  }

  // прошло больше 10 секунд — пробуем снова подключиться
  if (millis() - wifiLostTime >= config.wifiRetryWindow) {
    digitalWrite(STATUS_LED_PIN, LOW);

    WiFi.disconnect();
    WiFi.mode(WIFI_STA);
    WiFi.begin(config.ssid, config.password);
    
    send_dmx();

    unsigned long retryStart = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - retryStart < config.startTimeoutConnetion) {
      delay(100);
      send_dmx();
    }

    if (WiFi.status() != WL_CONNECTED) {
      startAccessPoint();
    } else {
      wifiLostTime = 0;
    }
  }
}

// страничка
void handleRoot() {
  String html = "<html><head><title>ESP Settings</title></head><body>";
  html += "<h2>Настройки устройства</h2>";
  html += "<form method='POST' action='/save'>";
  html += "Device Index: <input type='number' name='device_index' value='" + String(config.device_index) + "'><br>";
  html += "SSID: <input type='text' name='ssid' value='" + String(config.ssid) + "'><br>";
  html += "Password: <input type='text' name='password' value='" + String(config.password) + "'><br>";
  html += "Universe: <input type='number' name='universe' value='" + String(config.universe) + "'><br>";
  html += "ArtNet Port: <input type='number' name='artnet_port' value='" + String(config.artnet_port) + "'><br>";
  html += "Loop Checkpoint: <input type='number' name='loop_check_point' value='" + String(config.loop_check_point) + "'><br>";
  html += "WiFi Retry (ms): <input type='number' name='wifiRetryWindow' value='" + String(config.wifiRetryWindow) + "'><br>";
  html += "Connect Timeout (ms): <input type='number' name='startTimeoutConnetion' value='" + String(config.startTimeoutConnetion) + "'><br>";
  html += "Local IP: <input type='text' name='local_ip' value='" + config.local_ip.toString() + "'><br>";
  html += "Gateway: <input type='text' name='gateway' value='" + config.gateway.toString() + "'><br>";
  html += "Subnet: <input type='text' name='subnet' value='" + config.subnet.toString() + "'><br>";
  html += "<br><input type='submit' value='Save & Reboot'>";
  html += "</form>";
  html += "</body></html>";
  server.send(200, "text/html", html);
}

// сохранить
void handleSave() {
  if (server.hasArg("device_index")) config.device_index = server.arg("device_index").toInt();
  if (server.hasArg("ssid")) strncpy(config.ssid, server.arg("ssid").c_str(), sizeof(config.ssid));
  if (server.hasArg("password")) strncpy(config.password, server.arg("password").c_str(), sizeof(config.password));
  if (server.hasArg("universe")) config.universe = server.arg("universe").toInt();
  if (server.hasArg("artnet_port")) config.artnet_port = server.arg("artnet_port").toInt();
  if (server.hasArg("loop_check_point")) config.loop_check_point = server.arg("loop_check_point").toInt();
  if (server.hasArg("wifiRetryWindow")) config.wifiRetryWindow = server.arg("wifiRetryWindow").toInt();
  if (server.hasArg("startTimeoutConnetion")) config.startTimeoutConnetion = server.arg("startTimeoutConnetion").toInt();

  if (server.hasArg("local_ip")) config.local_ip.fromString(server.arg("local_ip"));
  if (server.hasArg("gateway")) config.gateway.fromString(server.arg("gateway"));
  if (server.hasArg("subnet")) config.subnet.fromString(server.arg("subnet"));

  saveConfig();
  server.send(200, "text/html", "<html><body><h2>Настройки сохранены</h2><p>Перезагрузка...</p></body></html>");
  delay(1000);
  ESP.restart();
}


// ===== LED CONTROL =====
void updateStatusLed() {
  unsigned long now = millis();

  // Меняем поведение в зависимости от состояния
  if (WiFi.status() == WL_CONNECTED) {
    // ✅ STA подключен — постоянный свет
    digitalWrite(STATUS_LED_PIN, HIGH);
  }
  else if (apMode) {
    // 📡 В режиме AP — мигает медленно (раз в 1 секунду)
    if (now - lastLedUpdate > 1000) {
      ledState = !ledState;
      digitalWrite(STATUS_LED_PIN, ledState);
      lastLedUpdate = now;
    }
  }
  else {
    // ❌ Нет соединения и не AP — мигает быстро
    if (now - lastLedUpdate > 200) {
      ledState = !ledState;
      digitalWrite(STATUS_LED_PIN, ledState);
      lastLedUpdate = now;
    }
  }
}

// ==== SETUP ====
void setup() {
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, HIGH);

  delay(200);

  loadConfig();

  // Подключение к WiFi
  bool connected = connectToWiFi();
  
  if (!connected) {
    startAccessPoint();
  }

  // OTA
  ArduinoOTA.setHostname(("esp32-artnet-" + String(config.device_index)).c_str());
  ArduinoOTA.setPasswordHash("3f483ade2441b07818408b62709274e2");
  ArduinoOTA.begin();

  // ArtNet UDP
  Udp.begin(config.artnet_port);

  // RS-485 управление
  pinMode(DMX_DE_RE_PIN, OUTPUT);
  digitalWrite(DMX_DE_RE_PIN, LOW);

  // TX pin по умолчанию HIGH
  pinMode(DMX_TX_PIN, OUTPUT);
  digitalWrite(DMX_TX_PIN, HIGH);

  // UART конфиг
  uart_config_t uart_config = {
    .baud_rate = 250000,
    .data_bits = UART_DATA_8_BITS,
    .parity    = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_2,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
  };
  uart_driver_install(DMX_UART_NUM, 1024, 0, 0, NULL, 0);  // буфер увеличен
  uart_param_config(DMX_UART_NUM, &uart_config);
  uart_set_pin(DMX_UART_NUM, DMX_TX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

  // DMX старт-код
  dmx_data[0] = 0x00;

  server.on("/", handleRoot);
  server.on("/save", HTTP_POST, handleSave);
  server.begin();

  digitalWrite(STATUS_LED_PIN, LOW);
  delay(200);
}

// ==== LOOP ====
void loop() {
  updateStatusLed();
  loopCounter++;
  if (loopCounter >= config.loop_check_point) {
    ArduinoOTA.handle();
    checkWiFiStatus();
    server.handleClient();
    loopCounter = 0;
  }

  int packetSize = Udp.parsePacket();
  if (packetSize > 0) {
    uint8_t buffer[600];  // достаточно для ArtDMX 512
    int len = Udp.read(buffer, sizeof(buffer));
    if (len >= 18 && strncmp((char*)buffer, config.artnet_id, 8) == 0) {
      uint16_t opcode = buffer[8] | (buffer[9] << 8);
      if (opcode == ART_DMX_OPCODE) {
        uint16_t universe = buffer[14] | (buffer[15] << 8);
        if (universe == config.universe) {
          uint16_t dmxLen = (buffer[16] << 8) | buffer[17];
          if (dmxLen > DMX_CHANNELS) dmxLen = DMX_CHANNELS;
          memcpy(&dmx_data[1], &buffer[18], dmxLen);  // dmx_data[0] = старт-код
        }
      }
    }
  }

  send_dmx();
}