#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_system.h"

// ==== НАСТРОЙКИ ====
#define DEVICE_INDEX 1 // номер устройства  |  влияет на приоритет STA / AP

//WIFI SETTINGS
const char* ssid = "grace_light";
const char* password = "grace_dmx512";
bool apMode = false;
unsigned long wifiLostTime = 0;
const unsigned long wifiRetryWindow = 10000;  // 10 секунд
const unsigned long startTimeoutConnetion = 4000;

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

#define LOOP_CHECK_POINT 50
int loopCounter = 0;

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
  WiFi.begin(ssid, password);

  unsigned long timeout = startTimeoutConnetion * DEVICE_INDEX;
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

  bool apOk = WiFi.softAP(ssid, password);
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
  if (millis() - wifiLostTime >= wifiRetryWindow) {
    digitalWrite(STATUS_LED_PIN, LOW);

    WiFi.disconnect();
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    
    send_dmx();

    unsigned long retryStart = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - retryStart < startTimeoutConnetion) {
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

  // Подключение к WiFi
  bool connected = connectToWiFi();
  
  if (!connected) {
    startAccessPoint();
  }

  // OTA
  ArduinoOTA.setHostname(("esp32-artnet-" + String(DEVICE_INDEX)).c_str());
  ArduinoOTA.setPasswordHash("3f483ade2441b07818408b62709274e2");
  ArduinoOTA.begin();

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
  uart_driver_install(DMX_UART_NUM, 1024, 0, 0, NULL, 0);  // буфер увеличен
  uart_param_config(DMX_UART_NUM, &config);
  uart_set_pin(DMX_UART_NUM, DMX_TX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

  // DMX старт-код
  dmx_data[0] = 0x00;

  digitalWrite(STATUS_LED_PIN, LOW);
  delay(200);
}

// ==== LOOP ====
void loop() {
  updateStatusLed();
  loopCounter++;
  if (loopCounter >= LOOP_CHECK_POINT) {
    ArduinoOTA.handle();
    checkWiFiStatus();
    loopCounter = 0;
  }

  int packetSize = Udp.parsePacket();
  if (packetSize > 0) {
    uint8_t buffer[600];  // достаточно для ArtDMX 512
    int len = Udp.read(buffer, sizeof(buffer));
    if (len >= 18 && memcmp(buffer, ART_NET_ID, 8) == 0) {
      uint16_t opcode = buffer[8] | (buffer[9] << 8);
      if (opcode == ART_DMX_OPCODE) {
        uint16_t universe = buffer[14] | (buffer[15] << 8);
        if (universe == 0) {
          uint16_t dmxLen = (buffer[16] << 8) | buffer[17];
          if (dmxLen > DMX_CHANNELS) dmxLen = DMX_CHANNELS;
          memcpy(&dmx_data[1], &buffer[18], dmxLen);  // dmx_data[0] = старт-код
        }
      }
    }
  }

  send_dmx();
}