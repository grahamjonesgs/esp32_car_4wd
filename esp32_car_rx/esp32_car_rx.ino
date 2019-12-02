/*
  Wheel order
  front_left;
  front_right;
  back_left;
  back_right;


*/

#include <analogWrite.h>
#include <WiFi.h>
#include "AsyncUDP.h"
#include "network_config.h"


// Structures
struct DirectionPin {
  int one;
  int two;
};

struct Velocity {
  int speed;
  int rotation;  // Possitve right
};

// Definitions
#define MAX_WIFI_RETRIES 2
#define TX_MESSAGE_LEN 250

// Pin Definitions
#define FRONT_LEFT_EN 16
#define FRONT_LEFT_1 5
#define FRONT_LEFT_2 18
#define FRONT_LEFT_DIR false
#define FRONT_RIGHT_1 21
#define FRONT_RIGHT_2 19
#define FRONT_RIGHT_EN 23
#define FRONT_RIGHT_DIR false
#define BACK_LEFT_EN 17
#define BACK_LEFT_1 5
#define BACK_LEFT_2 18
#define BACK_LEFT_DIR false
#define BACK_RIGHT_1 21
#define BACK_RIGHT_2 19
#define BACK_RIGHT_EN 22
#define BACK_RIGHT_DIR false
#define LED_PIN  LED_BUILTIN
#define RIGHT_ENCODER_PIN 15
#define LEFT_ENCODER_PIN 2
#define VOLTAGE_DIVIDER_PIN 34
#define VOLTAGE_DIVIDER_RATIO 5.06 *3.3 // Calulated from actuals
#define VOLTAGE_MESSAGE_DELAY 5000 // Time between voltage messages

#define DEBUG_MSG true

// Ultra sonic definitions
#define FRONT_TRIG_PIN 35 // define TrigPin
#define FRONT_ECHO_PIN 36 // define EchoPin.
#define MAX_DISTANCE 300 // Maximum sensor distance is rated at 400-500cm.
#define MAX_FRONT_DIST 30.0 // closest we want to come
#define MAX_FRONT_DIST_FAST 60.0 // closest we want to come when fast
#define HEARTBEAT_TIMEOUT 200  // delay millis before sending heartbeat 
float timeOut = MAX_DISTANCE * 60;
int soundVelocity = 340; // define sound speed=340m/s


// RPM definitions
unsigned int rightRpmCounter = 0;
unsigned int leftRpmCounter = 0;
float rightRpm;
float leftRpm;


// Global Variables
int speedPin[4] = {FRONT_LEFT_EN, FRONT_RIGHT_EN, BACK_LEFT_EN, BACK_RIGHT_EN};
bool wheelDirection[4] = {FRONT_LEFT_DIR, FRONT_RIGHT_DIR, BACK_LEFT_DIR, BACK_RIGHT_DIR};  // Define if wheels wired backwards
DirectionPin directionPin[4] {{FRONT_LEFT_1, FRONT_LEFT_2}, {FRONT_RIGHT_1, FRONT_RIGHT_2}, {BACK_LEFT_1, BACK_LEFT_2}, {BACK_RIGHT_1, BACK_RIGHT_2}};
Velocity velocity;
int wheelOutput[4];
int wheelOutputPwm[4] = {0, 0, 0, 0};
bool velocity_update = true;
bool forward_obsticle = false;
bool forward_obsticle_fast = false;
static uint32_t lastMessageTime = 0;
static uint32_t lastVoltageMessageTime = 0;

AsyncUDP udp;

void setup() {

  if (DEBUG_MSG) {
    Serial.begin(115200); // Default speed of esp32
  }

  for (int i = 0; i < 4; i++) {
    pinMode(directionPin[i].one, OUTPUT);
    pinMode(directionPin[i].two, OUTPUT);
    pinMode(speedPin[i], OUTPUT);
  }
  pinMode(LED_PIN, OUTPUT);
  pinMode(FRONT_TRIG_PIN, OUTPUT); // set TRIG_PIN to output mode
  pinMode(FRONT_ECHO_PIN, INPUT); // set ECHO_PIN to input mode
  pinMode(RIGHT_ENCODER_PIN, INPUT); // set ECHO_PIN to input mode
  velocity.speed = 0;
  velocity.rotation = 0;
  network_connect();
  message_tx("Car connected to WiFi", MSG_TYPE_STATUS);
  //xTaskCreatePinnedToCore( ultra_sonic_t, "TFT Update", 8192 , NULL, 9, NULL, 0 );

  attachInterrupt(RIGHT_ENCODER_PIN, rightCountPulse, RISING);
  attachInterrupt(LEFT_ENCODER_PIN, leftCountPulse, RISING);
}

void rightCountPulse() {
  rightRpmCounter++;
}

void leftCountPulse() {
  leftRpmCounter++;
}


void ultra_sonic_t(void * pvParameters ) {

  float distance;
  while (true) {

    delay(100); // Wait 100ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
    distance = get_front_sonar();
    message_tx(String(distance), MSG_TYPE_FRONT_DISTANCE);
    if (distance < MAX_FRONT_DIST) {
      forward_obsticle = true;
      set_wheel_speed();
      velocity_update = true;
      message_tx("Yes", MSG_TYPE_FRONT_OBSTICLE);
    }
    else {
      if (distance < MAX_FRONT_DIST_FAST) {
        forward_obsticle_fast = true;
        set_wheel_speed();
        velocity_update = true;
        message_tx("Fast", MSG_TYPE_FRONT_OBSTICLE);
      }

      else {
        forward_obsticle = false;
        forward_obsticle_fast = false;
        message_tx("No", MSG_TYPE_FRONT_OBSTICLE);
      }
    }
  }
}

void message_tx (String message, char message_type) {
  char full_message[TX_MESSAGE_LEN + 5];
  static char serialNum[4] = {SERIAL_NUMBER};

  if (message.length() > TX_MESSAGE_LEN) {
    return;
  }
  for (int i = i; i < sizeof(full_message); i++) {
    full_message[i] = 0;
  }
  memcpy(full_message, serialNum, sizeof(serialNum));
  full_message[4] = (char) message_type;
  message.toCharArray(full_message + 5, message.length() + 1);
  udp.broadcastTo(full_message, UPD_PORT);
  lastMessageTime = millis();
}

void set_wheel_speed () {
  //Set individual wheel speed based on Velocity
  if (velocity.speed == 0) {
    wheelOutput[0] = - velocity.rotation;
    wheelOutput[1] = velocity.rotation;
    wheelOutput[2] = - velocity.rotation;
    wheelOutput[3] = velocity.rotation;
  }
  else {
    if (velocity.speed == 0) {
      wheelOutput[0] = 0;
      wheelOutput[1] = 0;
      wheelOutput[2] = 0;
      wheelOutput[3] = 0;
    }
    else {
      if (velocity.speed > 0) {
        if (velocity.rotation > 0) {
          wheelOutput[0] = velocity.speed - velocity.rotation;
          wheelOutput[1] = velocity.speed;
          wheelOutput[2] = velocity.speed - velocity.rotation;
          wheelOutput[3] = velocity.speed;
        }
        else {
          wheelOutput[0] = velocity.speed;
          wheelOutput[1] = velocity.speed + velocity.rotation;
          wheelOutput[2] = velocity.speed;
          wheelOutput[3] = velocity.speed + velocity.rotation;
        }
      }

      else {
        if (velocity.rotation > 0) {
          wheelOutput[0] = velocity.speed + velocity.rotation;
          wheelOutput[1] = velocity.speed;
          wheelOutput[2] = velocity.speed + velocity.rotation;
          wheelOutput[3] = velocity.speed;
        }
        else {
          wheelOutput[0] = velocity.speed;
          wheelOutput[1] = velocity.speed - velocity.rotation;
          wheelOutput[2] = velocity.speed;
          wheelOutput[3] = velocity.speed - velocity.rotation;
        }
      }
    }
  }
  for (int i = 0; i < 4; i++) {
    if (DEBUG_MSG) {
      //Serial.printf(" Wheel % i, speed % i. ", i, wheelOutput[i]);
    }
  }
  if (DEBUG_MSG) {
    //Serial.println();
  }
}

void motor_control () {

  /*for (int i = 0; i < 4; i++) {
    if (wheelOutputPwm[i] < MIN_OUTPUT) {

      wheelOutputPwm[i] = MIN_OUTPUT;
    }
    if (rightRpm < map(wheelOutput[i], 0, 255, 0, MAX_RPM)) {
      wheelOutputPwm[i] += 16;
      if (wheelOutputPwm[i] > 255) {
        wheelOutputPwm[i] = 255;
      }
    }
    else {
      wheelOutputPwm[i] -= 16;
      if (wheelOutputPwm[i] < MIN_OUTPUT) {
        wheelOutputPwm[i] = MIN_OUTPUT;
      }
    }

    }*/


  for (int i = 0; i < 4; i++) {
    //analogWrite(speedPin[i], abs(wheelOutputPwm[i])); used for RMP based control
    analogWrite(speedPin[i], abs(wheelOutput[i]));
    if (DEBUG_MSG) {
      //Serial.printf("Wheel % i, pin % i, speed % i ", i, speedPin[i], abs(wheelOutput[i]));
    }



    if ((wheelOutput[i] > 0 && wheelDirection[i]) || (wheelOutput[i] < 0 && !wheelDirection[i])) {
      if (DEBUG_MSG) {
        //Serial.printf(" first direction, pin % i high, pin % i low\n", directionPin[i].one, directionPin[i].two);
      }
      digitalWrite(directionPin[i].one, HIGH);
      digitalWrite(directionPin[i].two, LOW);
    }
    else {
      if (DEBUG_MSG) {
        //Serial.printf(" second direction, pin % i low, pin % i high\n", directionPin[i].one, directionPin[i].two);
      }
      digitalWrite(directionPin[i].one, LOW);
      digitalWrite(directionPin[i].two, HIGH);
    }
  }
  if (DEBUG_MSG) {
    //Serial.println();
  }
}

void udpHandle (AsyncUDPPacket packet) {
  char serialNum[4] = {SERIAL_NUMBER};
  char receivedPacket[8];
  int speed;
  int direction;

  if (packet.length() != 8 ) {
    if (DEBUG_MSG) {
      Serial.println("Invalid length for us");
    }
    return;
  }

  memcpy(receivedPacket, packet.data(), sizeof(receivedPacket));
  for (int i = 0; i < sizeof(serialNum); i++) {
    if (receivedPacket[i] != serialNum[i]) {
      if (DEBUG_MSG) {
        Serial.println("Invalid serial number");
      }
      return;
    }
  }

  if ((receivedPacket[4] != 0 && receivedPacket[4] != 1) || (receivedPacket[6] != 0 && receivedPacket[6] != 1)) {
    if (DEBUG_MSG) {
      Serial.println("Invalid direction flags");
    }
    return;
  }

  speed = (int) receivedPacket[5];
  direction = (int) receivedPacket[7];
  if (receivedPacket[4] == 0) {
    speed = - speed;
  }
  if (receivedPacket[6] == 0) {
    direction = - direction;
  }
  velocity.speed = speed;
  velocity.rotation = direction;
  if (DEBUG_MSG) {
    //Serial.printf("Valid message, speed % i, direction % i\n", speed, direction);
  }

  if (speed > 0 && forward_obsticle == true) {
    velocity.speed = 0;
  }
  if (speed > 0 && forward_obsticle_fast == true) {
    velocity.speed = velocity.speed / 2;
  }
  //digitalWrite(LED_PIN, HIGH);
  //delay(50);
  //digitalWrite(LED_PIN, LOW);
  //Serial.print(speed);
  set_wheel_speed();
  velocity_update = true;


}


void network_connect() {

  int retryLoops = 0;
  if (DEBUG_MSG) {
    Serial.print("Connect to WPA SSID : ");
    Serial.println(WIFI_SSID);
  }

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (int retcode = WiFi.status() != WL_CONNECTED) {
    delay(1000);
    if (DEBUG_MSG) {
      Serial.println("WiFi return code is : " + String(retcode));
    }

    retryLoops++;
    if (retryLoops > MAX_WIFI_RETRIES) {
      delay(2000);
      ESP.restart();
    }
  }

  if (udp.listen(UPD_PORT)) {
    if (DEBUG_MSG) {
      Serial.print("UDP Listening on IP : ");
      Serial.println(WiFi.localIP());
    }
    udp.onPacket(udpHandle);
    for (int i = 0; i < 3; i++) {
      digitalWrite(LED_PIN, HIGH);
      delay(500);
      digitalWrite(LED_PIN, LOW);
      delay(500);
    }
  }
}

float get_front_sonar() {
  unsigned long pingTime;
  float distance;
  digitalWrite(FRONT_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(FRONT_TRIG_PIN, LOW);
  pingTime = pulseIn(FRONT_ECHO_PIN, HIGH, timeOut);
  distance = (float)pingTime * soundVelocity / 2 / 10000;
  if (distance > 0) {
    return distance;
  }
  else {
    return 999;
  }
}

void loop() {
  static uint32_t previousRpmMillis = 0;
  int voltageInput = 0;

  if (WiFi.status() != WL_CONNECTED) {
    network_connect();
  }

  //if (velocity_update) {
  //velocity_update = false;
  //motor_control();
  //}

  if (millis() - previousRpmMillis >= 200) {
    rightRpm = ((float)(rightRpmCounter) / 20.0) * 300.0;
    rightRpmCounter = 0;
    leftRpm = ((float)(leftRpmCounter) / 20.0) * 300.0;
    leftRpmCounter = 0;
    previousRpmMillis = millis();
    message_tx(String(rightRpm, 1), MSG_TYPE_RIGHT_RPM);
    message_tx(String(leftRpm, 1), MSG_TYPE_LEFT_RPM);
  }

  if (millis() - lastVoltageMessageTime >= VOLTAGE_MESSAGE_DELAY) {
    voltageInput = analogRead(VOLTAGE_DIVIDER_PIN);
    message_tx(String(voltageInput * VOLTAGE_DIVIDER_RATIO / 4096.0, 1), MSG_TYPE_VOLTAGE);
    lastVoltageMessageTime = millis();
  }

  if (millis() - lastMessageTime >= HEARTBEAT_TIMEOUT) {
    message_tx("", MSG_TYPE_HEARTBEAT);
  }
  delay(5);

}
