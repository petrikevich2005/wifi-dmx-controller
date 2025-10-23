// DMX_UART_NUM 1, DMX_TX_PIN 17, DMX_DE_RE_PIN 4, STATUS_LED_PIN 2, MODE_SWITCH_PIN 32, RECOVERY_PIN 12

#include <WiFi.h>
#include <WiFiUdp.h>
#include <WebServer.h>
#include <Preferences.h>
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_system.h"
#include "esp_timer.h"

// ===================== ПИНЫ И КОНСТАНТЫ =====================
#define DMX_UART_NUM     UART_NUM_1
#define DMX_TX_PIN       GPIO_NUM_17
#define DMX_DE_RE_PIN    GPIO_NUM_4

#define STATUS_LED_PIN   GPIO_NUM_2
#define MODE_SWITCH_PIN  GPIO_NUM_32
#define RECOVERY_PIN GPIO_NUM_12

#define DMX_CHANNELS     512
#define DMX_PACKET_SIZE  (1 + DMX_CHANNELS)  // старт-код + 512 каналов

#define ART_NET_PORT     6454
const char ART_NET_ID[] = "Art-Net\0";
#define ART_DMX_OPCODE   0x5000

// тайминги и таймеры
#define WIFI_CONNECT_TIMEOUT_MS 15000UL
#define DMX_PERIOD_US           33000UL  // 33 ms ~30Hz

// защита сохранения веба
#define ACCESS_CODE "1736"

// ===================== ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ =====================
uint8_t dmx_data[DMX_PACKET_SIZE] = {0};
WiFiUDP Udp;
esp_timer_handle_t dmx_timer;

WebServer server(80);
Preferences prefs;

bool isAPMode = false;
bool recovery_mode = false;
bool use_static_ip = false;

unsigned long lastLedUpdate = 0;
bool ledState = false;

uint16_t configured_universe = 0;
String saved_ssid = "WIFI-DMX-Controller";
String saved_pass = "123456789";

IPAddress saved_local_ip(192,168,0,50);
IPAddress saved_gateway(192,168,0,1);
IPAddress saved_subnet(255,255,255,0);

// ===================== ПРОТОТИПЫ =====================
void startRecoveryAccessPoint();
void startAccessPoint();
void startStationMode();
void timerSetup();
void send_dmx();
void IRAM_ATTR dmx_timer_callback(void* arg);
void handleRoot();
void handleSave();
void handleRestart();
void handleGetSettings();
void loadSettings();
void setupUART();
void updateLedPattern();
void getArtnetData();
void checkConnection();

// ===================== SETUP =====================
void setup() {
  // Сохранённые настройки
  prefs.begin("wifidmx", false);
  loadSettings();

  // начальная индикация
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, HIGH);

  // кнопки
  pinMode(MODE_SWITCH_PIN, INPUT_PULLUP);
  pinMode(RECOVERY_PIN, INPUT_PULLUP);

  // если RECOVERY нажата при старте -> RECOVERY_MODE
  if (digitalRead(RECOVERY_PIN) == LOW) {
    recovery_mode = true;
  }

  // recovery_mode
  if (recovery_mode) {
    // startRecoveryAccessPoint
    startRecoveryAccessPoint();
    // WebServer роуты (работают как в AP, так и в STA)
    server.on("/", HTTP_GET, handleRoot);
    server.on("/save", HTTP_POST, handleSave);
    server.on("/restart", HTTP_POST, handleRestart);
    server.on("/settings", HTTP_GET, handleGetSettings);
    server.begin();
  } else {
    // MODE switch: если зажат при старте - AP
    if (digitalRead(MODE_SWITCH_PIN) == LOW) {
      isAPMode = true;
    } else {
      isAPMode = false;
    }

    // UART / RS485 DE/RE
    pinMode(DMX_DE_RE_PIN, OUTPUT);
    digitalWrite(DMX_DE_RE_PIN, LOW);

    // TX pin по умолчанию HIGH (idle)
    pinMode(DMX_TX_PIN, OUTPUT);
    digitalWrite(DMX_TX_PIN, HIGH);

    setupUART();

    // DMX старт-код
    dmx_data[0] = 0x00;

    // WiFi
    if (isAPMode) {
      startAccessPoint();
    } else {
      startStationMode();
    }

    // UDP
    Udp.begin(ART_NET_PORT);

    // таймер DMX
    timerSetup();
  }

  digitalWrite(STATUS_LED_PIN, LOW);
}

// ===================== UART / DMX =====================
void setupUART() {
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
}

// === TIMER CALLBACK ===
void IRAM_ATTR dmx_timer_callback(void* arg) {
  send_dmx();
}

// ==== DMX ПЕРЕДАЧА ====
void send_dmx() {
  // выставляем DE/RE
  gpio_set_level((gpio_num_t)DMX_DE_RE_PIN, 1);

  // === BREAK ===
  gpio_set_direction((gpio_num_t)DMX_TX_PIN, GPIO_MODE_OUTPUT); // отключаем UART pin
  gpio_set_level((gpio_num_t)DMX_TX_PIN, 0);                     // логический 0
  ets_delay_us(110); // delayMicroseconds — не безопасен в IRAM, используем ets_delay_us

  // === MAB ===
  gpio_set_level((gpio_num_t)DMX_TX_PIN, 1);
  ets_delay_us(12);

  // === Включаем UART и отправляем пакет ===
  uart_set_pin(DMX_UART_NUM, DMX_TX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  uart_write_bytes(DMX_UART_NUM, (const char*)dmx_data, DMX_PACKET_SIZE);
  uart_wait_tx_done(DMX_UART_NUM, pdMS_TO_TICKS(10));

  // Межкадровая пауза
  ets_delay_us(44);

  // Выключаем DE/RE
  gpio_set_level((gpio_num_t)DMX_DE_RE_PIN, 0);
}

// ==== ТАЙМЕР SETUP ====
void timerSetup() {
  const esp_timer_create_args_t dmx_timer_args = {
    .callback = &dmx_timer_callback,
    .arg = nullptr,
    .dispatch_method = ESP_TIMER_TASK,
    .name = "dmx_timer"
  };
  esp_err_t res = esp_timer_create(&dmx_timer_args, &dmx_timer);
  if (res != ESP_OK) {
    esp_restart();
  }
  esp_timer_start_periodic(dmx_timer, DMX_PERIOD_US);
}

// ===================== WiFi MODEs =====================
void startRecoveryAccessPoint() {
  WiFi.mode(WIFI_AP);
  IPAddress local_ip(192,168,0,1);
  IPAddress gateway(192,168,0,1);
  IPAddress subnet(255,255,255,0);
  WiFi.softAPConfig(local_ip, gateway, subnet);
  WiFi.softAP("ESP_WIFI_DMX_RECOVERY", "12345678");
}

void startAccessPoint() {
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(saved_local_ip, saved_gateway, saved_subnet);
  WiFi.softAP(saved_ssid.c_str(), saved_pass.c_str());
}

void startStationMode() {
  WiFi.mode(WIFI_STA);

  if (use_static_ip) {
    if (!WiFi.config(saved_local_ip, saved_gateway, saved_subnet)) {
      digitalWrite(STATUS_LED_PIN, HIGH);
      delay(200);
      digitalWrite(STATUS_LED_PIN, LOW);
      delay(200);
      digitalWrite(STATUS_LED_PIN, HIGH);
      delay(200);
      digitalWrite(STATUS_LED_PIN, LOW);
      delay(10);
      esp_restart();
    }
  }

  WiFi.begin(saved_ssid.c_str(), saved_pass.c_str());
  unsigned long startAttemptTime = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(200);
    updateLedPattern();
    if (millis() - startAttemptTime > WIFI_CONNECT_TIMEOUT_MS) {
      // не подключились — перезагрузка
      esp_restart();
    }
  }
}

// ===================== Web handlers =====================
void handleRoot() {
  String page = "<!doctype html><html><head><meta charset='utf-8'><title>WIFI DMX Controller</title></head><body>";
  page += "<h2>WIFI-DMX Controller</h2>";
  page += "<form method='POST' action='/save'>";
  page += "SSID:<br><input name='ssid' value='" + saved_ssid + "'><br>";
  page += "Password:<br><input name='pass' value='" + saved_pass + "'><br>";
  page += "Universe:<br><input name='universe' value='" + String(configured_universe) + "'><br>";
  page += "<h3>WiFi Settings</h3>";
  page += "Mode:<br><select name='ip_mode'>";
  page += "<option value='dynamic'" + String(use_static_ip ? "" : " selected") + ">Dynamic (DHCP)</option>";
  page += "<option value='static'" + String(use_static_ip ? " selected" : "") + ">Static</option>";
  page += "</select><br><br>";

  page += "Local IP:<br><input name='local_ip' value='" + saved_local_ip.toString() + "'><br>";
  page += "Gateway:<br><input name='gateway' value='" + saved_gateway.toString() + "'><br>";
  page += "Subnet:<br><input name='subnet' value='" + saved_subnet.toString() + "'><br>";
  page += "Access code (required to save):<br><input name='code' value=''><br><br>";
  page += "<button type='submit'>Save & Restart</button>";
  page += "</form>";
  page += "<form method='POST' action='/restart' style='margin-top:16px;'><button type='submit'>Restart</button></form>";
  page += "</body></html>";
  server.send(200, "text/html", page);
}

void handleGetSettings() {
  String json = "{";
  json += "\"ssid\":\"" + saved_ssid + "\",";
  json += "\"universe\":" + String(configured_universe);
  json += "}";
  server.send(200, "application/json", json);
}

void handleSave() {
  if (!server.hasArg("ssid") || !server.hasArg("pass") || !server.hasArg("universe") || !server.hasArg("code")) {
    server.send(400, "text/plain", "Bad request");
    return;
  }
  String code = server.arg("code");
  if (code != ACCESS_CODE) {
    server.send(401, "text/plain", "Access denied");
    return;
  }
  saved_ssid = server.arg("ssid");
  saved_pass = server.arg("pass");
  configured_universe = (uint16_t)atoi(server.arg("universe").c_str());

  // сохраняем
  prefs.putString("ssid", saved_ssid);
  prefs.putString("pass", saved_pass);
  prefs.putUShort("univ", configured_universe);

  bool staticFlag = server.arg("ip_mode") == "static";

  prefs.putBool("static_ip", staticFlag);
  prefs.putString("local_ip", server.arg("local_ip"));
  prefs.putString("gateway", server.arg("gateway"));
  prefs.putString("subnet", server.arg("subnet"));

  server.send(200, "text/plain", "Saved, restarting...");
  delay(200);
  esp_restart();
}

void handleRestart() {
  server.send(200, "text/plain", "Restarting...");
  delay(200);
  esp_restart();
}

// ===================== SETTINGS =====================
void loadSettings() {
  saved_ssid = prefs.getString("ssid", saved_ssid);
  saved_pass = prefs.getString("pass", saved_pass);
  configured_universe = prefs.getUShort("univ", configured_universe);
  use_static_ip = prefs.getBool("static_ip", false);

  saved_local_ip.fromString(prefs.getString("local_ip", "192.168.0.50"));
  saved_gateway.fromString(prefs.getString("gateway", "192.168.0.1"));
  saved_subnet.fromString(prefs.getString("subnet", "255.255.255.0"));
}

// ===================== ArtNet парсинг =====================
void getArtnetData() {
  int packetSize = Udp.parsePacket();
  if (packetSize > 0) {
    uint8_t buffer[600];
    int len = Udp.read(buffer, sizeof(buffer));
    if (len >= 18 && memcmp(buffer, ART_NET_ID, 8) == 0) {
      uint16_t opcode = buffer[8] | (buffer[9] << 8);
      if (opcode == ART_DMX_OPCODE) {
        uint16_t universe = buffer[14] | (buffer[15] << 8);
        uint16_t dmxLen = (buffer[16] << 8) | buffer[17];
        if (dmxLen > DMX_CHANNELS) dmxLen = DMX_CHANNELS;
        // фильтр по universe (если хотите принимать только настроенный)
        if (universe == configured_universe) {
          memcpy(&dmx_data[1], &buffer[18], dmxLen);
        }
      }
    }
  }
}

// ===================== LED PATTERNS =====================
void updateLedPattern() {
  unsigned long now = millis();
  // режимы:
  if (isAPMode) {
    // AP: длинный blink 2000ms on / 2000ms off
    if (now - lastLedUpdate >= 2000) {
      ledState = !ledState;
      digitalWrite(STATUS_LED_PIN, ledState ? HIGH : LOW);
      lastLedUpdate = now;
    }
  } else {
    // STA: короткие blinks 500ms on / 500ms off
    if (ledState) {
      if (now - lastLedUpdate >= 500) {
        ledState = false;
        digitalWrite(STATUS_LED_PIN, LOW);
        lastLedUpdate = now;
      }
    } else {
      if (now - lastLedUpdate >= 500) {
        ledState = true;
        digitalWrite(STATUS_LED_PIN, HIGH);
        lastLedUpdate = now;
      }
    }
  }
}

// ===================== CONNECTION / WATCHDOG =====================
void checkConnection() {
  // Если в STA и не подключены -> пробуем переподключиться небольшой цикл
  if (!isAPMode) {
    if (WiFi.status() != WL_CONNECTED) {
      unsigned long start = millis();
      while (WiFi.status() != WL_CONNECTED && millis() - start < WIFI_CONNECT_TIMEOUT_MS) {
        delay(200);
        updateLedPattern();
      }
      if (WiFi.status() != WL_CONNECTED) {
        WiFi.disconnect();
        WiFi.reconnect();
      }
    }
  }
}

// ===================== LOOP =====================
void loop() {
  if (recovery_mode) {
    // web server handling
    server.handleClient();
  } else {
    // ArtNet приём
    getArtnetData();
    // проверка соединения
    checkConnection();
    // обновление индикатора
    updateLedPattern();
  }
}
