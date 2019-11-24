/*
  // Define in user_setup.h in the library directory in TFT_eSPI
  #define TFT_MISO 22
  #define TFT_MOSI 23
  #define TFT_SCLK 18
  #define TFT_CS   21  // Chip select control pin
  #define TFT_DC    19  // Data Command control pin
  #define TFT_RST   5 // Reset pin (could connect to RST pin)
  #define TOUCH_CS 2     // Chip select pin (T_CS) of touch screen

  // Defned in this file
  #define LED_PIN 4

  T_DO / TFT_MISO
  T_DIN  / TFT_MOSI
  T_CLK / SCLK

   Touch caib settings
  uint16_t calData[5] = { 423, 3334, 446, 3168, 7 };
  tft.setTouch(calData);

*/


#include <WiFi.h>
#include <analogWrite.h>
#include "AsyncUDP.h"
#include "network_config.h"
#include <TFT_eSPI.h> // Hardware-specific library
#include <SPI.h>
#include <TimeLib.h>
#include "GfxUi.h"          // Attached to this sketch
#include "SPIFFS_Support.h" // Attached to this sketch


#define CENTER 35
#define MAX_WIFI_RETRIES 2
#define DEBUG_MSG true

// Pin definition
#define X_PIN 32
#define Y_PIN 33
#define S_PIN 25

//TFT Definitions
#define LED_BRIGHT 255
#define CHAR_LEN 255
#define LED_DIM 20
#define LED_PIN 4
#define PRESS_DEBOUNCE 1
#define STATUS_MESSAGE_TIME 5           // Seconds an status message can be displayed
#define TOUCH_CALIBRATION { 423, 3334, 446, 3168, 7 }
#define CAR_MESSAGE_TIMEOUT 1000

AsyncUDP udp;
TFT_eSPI tft = TFT_eSPI();
GfxUi ui = GfxUi(&tft);

// Global definitions
char frontDistance [CHAR_LEN];
char frontObsticle [CHAR_LEN];
char backDistance [CHAR_LEN];
char backObsticle [CHAR_LEN];
char statusMessage[CHAR_LEN];
char RPM[CHAR_LEN];
bool carConnected = false;
bool statusMessageUpdated = false;
int carSpeed = 0;
int carDirection = 0;
unsigned long lastCarMessage = 0;
int auto_speed = 0;
bool auto_accel = true;

void setup() {
  if (DEBUG_MSG) {
    Serial.begin(115200);
  }
  SPIFFS.begin();
  pinMode(LED_PIN, OUTPUT);
  xTaskCreatePinnedToCore( tft_output_t, "TFT Update", 8192 , NULL, 10, NULL, 0 ); // Highest priorit on this cpu to avoid coms errors
  //xTaskCreatePinnedToCore( tft_read_t, "TFT Update", 8192 , NULL, 9, NULL, 0 );

  pinMode(X_PIN, INPUT);
  pinMode(Y_PIN, INPUT);

  network_connect();
}

void tft_draw_string_centre(const char* message, int leftx, int rightx, int y, int font) {

  tft.drawString(message, leftx + (rightx - leftx - tft.textWidth(message, font)) / 2 , y , font);

}

void tft_read_t(void * pvParameters ) {

  uint16_t x = 0, y = 0;
  int speed;
  int direction;
  boolean pressed;
  time_t lastPressed = 0;

  //uint16_t calData[5] = TOUCH_CALIBRATION;
  //tft.setTouch(calData);
  delay(1000);
  while (true) {
    delay(100);

    pressed = tft.getTouch(&x, &y);
    if (pressed) {
      lastPressed = now();
      speed = map(y, 0, 230, -255, 255);
      
      if ((direction > 200) || (direction < -200)) {
        speed = 0;
      }

      direction = map(x, 320, 0, -255, 255);
      if (DEBUG_MSG) {
        Serial.printf("Press detected at x:%i, y:%i, speed %i, direction %i\n", x, y, speed, direction);
      }
    }
    else {
      speed = 0;
      direction = 0;
    }
    output_upd(speed, direction);
  }
}

void tft_output_t(void * pvParameters ) {

  int TITLE_LEFT = 0;
  int TITLE_RIGHT = 320;
  int TITLE_TOP = 0;
  int TITLE_BOTTOM = 20;

  int STATUS_LEFT = 0;
  int STATUS_RIGHT = 320;
  int STATUS_TOP = 216;
  int STATUS_BOTTOM = 240;

  int CAR_STATUS_LEFT = 0;
  int CAR_STATUS_RIGHT = 160;
  int CAR_STATUS_TOP = 30;
  int CAR_STATUS_BOTTOM = 200;
  int CAR_STATUS_GAP = 20;


  time_t statusChangeTime = 0;
  bool statusMessageDisplayed = false;
  tft.init();
  uint16_t calData[5] = TOUCH_CALIBRATION;
  tft.setTouch(calData);
  tft.fillScreen(TFT_RED);
  analogWrite(LED_PIN, LED_BRIGHT);
  tft.setRotation(3);
  //ui.drawJpeg("/images/logo.jpg", 0, 0);
  delay(2000);
  tft.fillScreen(TFT_BLACK);

  strcpy(frontDistance, "N/A");
  strcpy(frontObsticle, "N/A");
  strcpy(backDistance, "N/A");
  strcpy(backObsticle, "N/A");
  strcpy(RPM, "N/A");

  tft.fillRect(TITLE_LEFT, TITLE_TOP, TITLE_RIGHT - TITLE_LEFT, TITLE_BOTTOM - TITLE_TOP, TFT_GREEN);
  tft.setTextColor(TFT_BLACK, TFT_GREEN);
  tft_draw_string_centre(" The Car Tramsmitter V0.1", TITLE_LEFT, TITLE_RIGHT, TITLE_TOP, 2);


  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString("Front distance", CAR_STATUS_LEFT, CAR_STATUS_TOP , 2);
  tft.drawString("Front obsticle", CAR_STATUS_LEFT, CAR_STATUS_TOP + CAR_STATUS_GAP, 2);
  tft.drawString("Back distance", CAR_STATUS_LEFT, CAR_STATUS_TOP + CAR_STATUS_GAP * 2, 2);
  tft.drawString("Back obsticle", CAR_STATUS_LEFT, CAR_STATUS_TOP + CAR_STATUS_GAP * 3, 2);
  tft.drawString("Car RPM", CAR_STATUS_LEFT, CAR_STATUS_TOP + CAR_STATUS_GAP * 4, 2);
  tft.drawString("Car speed", CAR_STATUS_LEFT, CAR_STATUS_TOP + CAR_STATUS_GAP * 5, 2);
  tft.drawString("Car direction", CAR_STATUS_LEFT, CAR_STATUS_TOP + CAR_STATUS_GAP * 6, 2);
  tft.drawString("Car connected", CAR_STATUS_LEFT, CAR_STATUS_TOP + CAR_STATUS_GAP * 7, 2);


  while (true) {
    delay(50);

    // Remove old status messages
    if (statusChangeTime + STATUS_MESSAGE_TIME < now() && statusMessageDisplayed) {
      tft.fillRect(STATUS_LEFT, STATUS_TOP, STATUS_RIGHT - STATUS_LEFT, STATUS_BOTTOM - STATUS_TOP, TFT_BLACK);
      statusMessageDisplayed = false;
    }

    //Serial.printf("lastCarMessage is %i, millis is %i\n", lastCarMessage, millis());
    if (lastCarMessage + CAR_MESSAGE_TIMEOUT < millis())
    {
      carConnected = false;
      strcpy(frontDistance, "N/A          ");
      strcpy(frontObsticle, "N/A          ");
      strcpy(backDistance, "N/A          ");
      strcpy(backObsticle, "N/A          ");
      strcpy(RPM, "N/A          ");
    }
    else {
      carConnected = true;
    }


    if (statusMessageUpdated) {
      statusMessageUpdated = false;
      tft.fillRect(STATUS_LEFT, STATUS_TOP, STATUS_RIGHT - STATUS_LEFT, STATUS_BOTTOM - STATUS_TOP, TFT_BLACK);
      tft.setTextColor(TFT_CYAN, TFT_BLACK);
      tft_draw_string_centre(statusMessage, STATUS_LEFT, STATUS_RIGHT, STATUS_TOP + 4, 2);
      statusMessageDisplayed = true;
      statusChangeTime = now();
    }


    // Car status message

    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawString(String(frontDistance) + "     ", CAR_STATUS_LEFT + 105, CAR_STATUS_TOP , 2);
    tft.drawString(String(frontObsticle) + "     ", CAR_STATUS_LEFT + 105, CAR_STATUS_TOP + CAR_STATUS_GAP, 2);
    tft.drawString(String(backDistance) + "     ", CAR_STATUS_LEFT + 105, CAR_STATUS_TOP + CAR_STATUS_GAP * 2 , 2);
    tft.drawString(String(backObsticle) + "     ", CAR_STATUS_LEFT + 105, CAR_STATUS_TOP + CAR_STATUS_GAP * 3 , 2);
    tft.drawString(String(RPM) + "     ", CAR_STATUS_LEFT + 105, CAR_STATUS_TOP + CAR_STATUS_GAP * 4 , 2);
    tft.drawString(String(carSpeed) + "     ", CAR_STATUS_LEFT + 105, CAR_STATUS_TOP + CAR_STATUS_GAP * 5 , 2);


    tft.drawString(String(carDirection) + "   ", CAR_STATUS_LEFT + 105, CAR_STATUS_TOP + CAR_STATUS_GAP * 6 , 2);
    if (carConnected) {
      tft.drawString("Yes   ", CAR_STATUS_LEFT + 105, CAR_STATUS_TOP + CAR_STATUS_GAP * 7 , 2);
    }
    else {
      tft.setTextColor(TFT_RED, TFT_BLACK);
      tft.drawString("No   " , CAR_STATUS_LEFT + 105, CAR_STATUS_TOP + CAR_STATUS_GAP * 7 , 2);
      tft.setTextColor(TFT_WHITE, TFT_BLACK);
    }
  }

  yield();

}


void network_connect() {

  int retryLoops = 0;
  if (DEBUG_MSG) {
    Serial.print("Connect to WPA SSID: ");
    Serial.println(WIFI_SSID);
  }
  strncpy(statusMessage, "Waiting for ", CHAR_LEN);
  strncat(statusMessage, WIFI_SSID, CHAR_LEN);
  statusMessageUpdated = true;

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (int retcode = WiFi.status() != WL_CONNECTED) {
    delay(1000);
    if (DEBUG_MSG) {
      Serial.println("WiFi return code is: " + String(retcode));
    }
    retryLoops++;
    if (retryLoops > MAX_WIFI_RETRIES) {
      strncpy(statusMessage, "Error with WiFi, reboot ", CHAR_LEN);
      statusMessageUpdated = true;
      delay(2000);
      ESP.restart();
    }
  }

  if (udp.listen(UPD_PORT)) {
    if (DEBUG_MSG) {
      Serial.print("UDP Listening on IP: ");
      Serial.println(WiFi.localIP());
    }
    udp.onPacket(udpHandle);
  }

  strncpy(statusMessage, "Connected to ", CHAR_LEN);
  strncat(statusMessage, WIFI_SSID, CHAR_LEN);
  statusMessageUpdated = true;
  Serial.printf("Input_speed,RPM\n");

}

void udpHandle (AsyncUDPPacket packet) {
  char serialNum[4] = {SERIAL_NUMBER};
  char receivedPacket[255];
  int speed;
  int direction;
  char message_type;

  if (packet.length() < 5 ) {
    if (DEBUG_MSG) {
      Serial.println("Invalid length for us");
    }
    return;
  }


  memcpy(receivedPacket, packet.data(), packet.length());
  for (int i = 0; i < sizeof(serialNum); i++) {
    if (receivedPacket[i] != serialNum[i]) {
      if (DEBUG_MSG) {
        Serial.println("Invalid serial number");
      }
      return;
    }
  }

  lastCarMessage = millis();

  message_type = receivedPacket[4];
  switch (message_type) {
    case MSG_TYPE_STATUS:

      strncpy(statusMessage, receivedPacket + 4 , packet.length() - 4);
      statusMessage[packet.length() - 4] = 0;
      statusMessageUpdated = true;
      break;

    case MSG_TYPE_FRONT_DISTANCE:
      strncpy(frontDistance, receivedPacket + 4 , packet.length() - 4);
      frontDistance[packet.length() - 4] = 0;
      break;

    case MSG_TYPE_FRONT_OBSTICLE:
      strncpy(frontObsticle, receivedPacket + 4 , packet.length() - 4);
      frontObsticle[packet.length() - 4] = 0;
      break;

    case MSG_TYPE_RPM:
      strncpy(RPM, receivedPacket + 5 , packet.length() - 5);
      RPM[packet.length() - 5] = 0;
      break;

    case MSG_TYPE_NETWORK:

    case MSG_TYPE_REAR_OBSTICLE:
    case MSG_TYPE_REAR_DISTANCE:
      break;
  }
}

void output_upd(int speed, int direction) {
  static char serialNum[4] = {SERIAL_NUMBER};
  unsigned char transmitPacket[8];

  carSpeed = speed;
  carDirection = direction;
  memcpy(transmitPacket, serialNum, sizeof(serialNum));

  if (speed < 0) {
    speed = -speed;
    transmitPacket[4] = 0x00;
  }
  else {
    transmitPacket[4] = 0x01;
  }
  if (direction < 0) {
    direction = -direction;
    transmitPacket[6] = 0x00;
  }
  else {
    transmitPacket[6] = 0x01;
  }

  transmitPacket[5] = (char) speed;
  transmitPacket[7] = (char) direction;
  udp.broadcastTo((uint8_t *) transmitPacket, sizeof(transmitPacket), 1234);

}


void loop() {

  if (WiFi.status() != WL_CONNECTED) {
    network_connect();
  }
  if (carConnected) {
    if (auto_accel) {
      auto_speed++;
      if (auto_speed > 254)
      {
        auto_accel = false;
      }
    }
    else {
      auto_speed--;
      if (auto_speed < 1)
      {
        auto_accel = true;

      }
    }
    output_upd(auto_speed, 0);
    Serial.printf("%i,%s\n", auto_speed, RPM);
  }
  else {
    //Serial.printf("Waiting for car\n");
  }
  delay(1000);
}
