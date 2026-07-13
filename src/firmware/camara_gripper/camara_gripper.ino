/*
 * camara_gripper.ino — Cámara de teleoperación + servo del gripper
 * Proyecto 1MTR52 — Lab. Procesos Industriales, PUCP
 *
 * Basado en el ejemplo oficial CameraWebServer de Espressif (esp32-camera).
 * Modificaciones propias del proyecto:
 *   1. Credenciales WiFi del router del laboratorio e IP estática 192.168.1.14.
 *   2. Control del servomotor del gripper (ESP32Servo) en el pin SERVO_PIN = 13:
 *        SERVO_OPEN = 0°  (gripper abierto) | SERVO_CLOSE = 180° (gripper cerrado)
 *   3. Nuevo endpoint HTTP GET /gripper?action=open|close registrado en
 *      app_httpd.cpp (gripper_handler) — server.py lo invoca desde la interfaz web.
 *
 * Endpoints principales:
 *   http://192.168.1.14/              -> interfaz de la cámara (Espressif)
 *   http://192.168.1.14:81/stream    -> stream MJPEG usado en el dashboard
 *   http://192.168.1.14/gripper?action=open|close -> apertura/cierre del gripper
 *
 * Placa: ESP32-CAM (AI-Thinker) — seleccionar en board_config.h.
 * Archivos auxiliares (base Espressif): app_httpd.cpp, board_config.h,
 * camera_index.h, camera_pins.h, partitions.csv.
 */

#include <Arduino.h>
#include "esp_camera.h"
#include <WiFi.h>
#include <ESP32Servo.h>

#include "board_config.h"

const char *ssid = "TP-Link_95E0";
const char *password = "20375010";




#define SERVO_PIN 13

#define SERVO_OPEN   0
#define SERVO_CLOSE  180



Servo gripper;

void startCameraServer();
void setupLedFlash();



void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 10000000;
  config.frame_size = FRAMESIZE_QVGA;
  config.pixel_format = PIXFORMAT_JPEG;
  config.grab_mode = CAMERA_GRAB_LATEST;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 15;
  config.fb_count = 2;

  if (config.pixel_format == PIXFORMAT_JPEG) {
    if (psramFound()) {
      config.jpeg_quality = 15;
      config.fb_count = 2;
      config.grab_mode = CAMERA_GRAB_LATEST;
    } else {
      config.frame_size = FRAMESIZE_QQVGA;
      config.fb_location = CAMERA_FB_IN_DRAM;
      config.jpeg_quality = 18;
      config.fb_count = 1;
    }
  } else {
    config.frame_size = FRAMESIZE_240X240;

#if CONFIG_IDF_TARGET_ESP32S3
    config.fb_count = 2;
#endif
  }

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t *s = esp_camera_sensor_get();

  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);
    s->set_brightness(s, 1);
    s->set_saturation(s, -2);
  }

  if (config.pixel_format == PIXFORMAT_JPEG) {
    s->set_framesize(s, FRAMESIZE_QVGA);
  }

#if defined(CAMERA_MODEL_M5STACK_WIDE) || defined(CAMERA_MODEL_M5STACK_ESP32CAM)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif

#if defined(CAMERA_MODEL_ESP32S3_EYE)
  s->set_vflip(s, 1);
#endif

#if defined(LED_GPIO_NUM)
  setupLedFlash();
#endif

  ESP32PWM::allocateTimer(1);
  gripper.setPeriodHertz(50);
  gripper.attach(SERVO_PIN, 500, 2400);
  gripper.write(SERVO_OPEN);
  delay(500);
  gripper.write(SERVO_CLOSE);

  //IP estática para el ESP32-CAM
  IPAddress local_IP(192, 168, 1, 14);   // IP que quieres asignar
  IPAddress gateway(192, 168, 1, 1);      // IP del router
  IPAddress subnet(255, 255, 255, 0);     
  IPAddress primaryDNS(8, 8, 8, 8);
  IPAddress secondaryDNS(8, 8, 4, 4);

//Configurar IP estática
 if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
    Serial.println("Error configurando IP estática");
  }

  WiFi.begin(ssid, password);
  WiFi.setSleep(false);

  Serial.print("WiFi connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");

  startCameraServer();

  Serial.print("Camera Ready! Use 'http://");
  Serial.print(WiFi.localIP());
  Serial.println("' to connect");

  Serial.print("Gripper open: http://");
  Serial.print(WiFi.localIP());
  Serial.println("/gripper?action=open");

  Serial.print("Gripper close: http://");
  Serial.print(WiFi.localIP());
  Serial.println("/gripper?action=close");
}

void loop() {
  delay(10000);
}