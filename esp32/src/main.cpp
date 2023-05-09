/* clang-format off */
#include <Arduino.h>
#include <WiFi.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "esp_camera.h"
#include <HTTPClient.h>
/* clang-format on */

#define GREEN 13
#define RED 14
#define LOCK 2
#define FLASH 4

const char* ssid = "fathomless";
const char* password = "theviper46";
// String server_name = "192.168.29.98";
String server_name = "13.233.251.246";
String server_url = "http://" + server_name + ":4000/validate";
String server_path = "/validate";
const int server_port = 4000;

WiFiClient client;

// CAMERA_MODEL_AI_THINKER
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27

#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22

const int timerInterval = 10;      // time between each HTTP POST image
unsigned long previousMillis = 0;  // last time image was sent

String checkPhoto() {
  String getAll;
  String getBody;

  camera_fb_t* fb = NULL;

  Serial.println("Capturing Camera");
  fb = esp_camera_fb_get();

  while (!fb) {
    Serial.println("Camera capture failed!");
    delay(500);
    Serial.println("Capturing Camera Again!");
    fb = esp_camera_fb_get();
  }

  Serial.println("Connecting to server: " + server_name);

  if (client.connect(server_name.c_str(), server_port)) {
    Serial.println("Connection successful!");
    String head =
        "--RandomNerdTutorials\r\nContent-Disposition: form-data; "
        "name=\"imageFile\"; filename=\"esp32-cam.jpg\"\r\nContent-Type: "
        "image/jpeg\r\n\r\n";
    String tail = "\r\n--RandomNerdTutorials--\r\n";

    uint32_t imageLen = fb->len;
    uint32_t extraLen = head.length() + tail.length();
    uint32_t totalLen = imageLen + extraLen;
    Serial.printf("Message Size: %dB, %dKB\n", totalLen, totalLen / 1024);

    client.println("POST " + server_path + " HTTP/1.1");
    client.println("Host: " + server_name);
    client.println("Content-Length: " + String(totalLen));
    client.println(
        "Content-Type: multipart/form-data; boundary=RandomNerdTutorials");
    client.println();
    client.print(head);

    uint8_t* fbBuf = fb->buf;
    size_t fbLen = fb->len;
    for (size_t n = 0; n < fbLen; n = n + 1024) {
      if (n + 1024 < fbLen) {
        client.write(fbBuf, 1024);
        fbBuf += 1024;
      } else if (fbLen % 1024 > 0) {
        size_t remainder = fbLen % 1024;
        client.write(fbBuf, remainder);
      }
    }
    client.print(tail);

    esp_camera_fb_return(fb);

    int timoutTimer = 10000;
    long startTimer = millis();
    boolean state = false;

    while ((startTimer + timoutTimer) > millis()) {
      Serial.print(".");
      delay(100);
      while (client.available()) {
        char c = client.read();
        if (c == '\n') {
          if (getAll.length() == 0) {
            state = true;
          }
          getAll = "";
        } else if (c != '\r') {
          getAll += String(c);
        }
        if (state == true) {
          getBody += String(c);
        }
        startTimer = millis();
      }
      if (getBody.length() > 0) {
        break;
      }
    }
    Serial.println();
    client.stop();
    // Serial.println(getBody);
  } else {
    getBody = "Connection to " + server_name + " failed.";
    Serial.println(getBody);
  }
  return getBody;
}

void setupPorts() {
  pinMode(RED, OUTPUT);
  pinMode(GREEN, OUTPUT);
  pinMode(LOCK, OUTPUT);
  pinMode(FLASH, OUTPUT);

  digitalWrite(FLASH, LOW);
  digitalWrite(LOCK, LOW);

  for (size_t i = 0; i != 10; i++) {
    digitalWrite(RED, i % 2 ? HIGH : LOW);
    digitalWrite(GREEN, i % 2 ? LOW : HIGH);
    delay(100);
  }

  digitalWrite(RED, HIGH);
  digitalWrite(GREEN, LOW);

  /* for (size_t i = 0; i != 1; i++) {
    digitalWrite(LOCK, LOW);
    delay(1000);
    digitalWrite(LOCK, HIGH);
    delay(1000);
  } */

  digitalWrite(LOCK, LOW);
  digitalWrite(FLASH, LOW);
}

void Lock() {
  digitalWrite(LOCK, LOW);
  digitalWrite(RED, HIGH);
  digitalWrite(GREEN, LOW);
  Serial.println("Closed Lock");
}

void UnLock() {
  digitalWrite(LOCK, HIGH);
  digitalWrite(RED, LOW);
  digitalWrite(GREEN, HIGH);
  Serial.println("Opened Lock");
}

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  Serial.begin(9600);
  Serial.setDebugOutput(true);
  Serial.println();
  Serial.println("Performing Setup");

  setupPorts();

  WiFi.mode(WIFI_STA);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println();
  Serial.print("ESP32-CAM IP Address: ");
  Serial.println(WiFi.localIP());

  Serial.println("SETTING UP WIFI :DONE:");

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
  // config.pin_sscb_sda = SIOD_GPIO_NUM;
  // config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  // init with high specs to pre-allocate larger buffers
  if (psramFound()) {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 10;  // 0-63 lower number means higher quality
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_CIF;
    config.jpeg_quality = 12;  // 0-63 lower number means higher quality
    config.fb_count = 1;
  }

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    delay(1000);
    ESP.restart();
  }
}

void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= timerInterval) {
    previousMillis = currentMillis;

    String result = checkPhoto();
    result.toUpperCase();
    // String result = "UNLOCK";
    Serial.printf("Server Return: %s\n", result.c_str());

    if (result.indexOf("UNLOCK") != -1)
      UnLock();
    else
      Lock();

    // if (result.compareTo("UNLOCK")) UnLock();
    // if (result.compareTo("LOCK")) Lock();
  }
}

// 192.168.29.98