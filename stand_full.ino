#include "WiFi.h"
#include "DNSServer.h"
#include "WebServer.h"
#include "index.h"
#include "HX711.h"

#define DT  16
#define SCK 4
#define INA 18
#define INB 19
#define OP  15


TaskHandle_t Task1;


WebServer server(80);
HX711 scale;


const float calibration_factor = 76.30;
float units;


const char* ssid = "callibration_stand";
const char* pass = "";
IPAddress APIP(192, 168, 4, 1);
DNSServer dns;
const char *server_name = "stand.cc";
const byte DNS_PORT = 53;


uint8_t power = 0;
uint8_t motorSpeedRange = 0;
uint8_t motorSecondSpeedRange = 0;
uint8_t rotationVector = 1;
uint8_t workMode = 0;
float motorSpeed = 0;
float gramms = 0;

const uint8_t nominalSpeed = 48; // номинальная скорсть
const uint8_t maxSpeed = 80; // ВРЕМЕННО! установка максимально скорости мотора в оборотах в секунду


const float h = 6.04; // длина плеча в сантиметрах
const float g = 9.81;     // ускорение свободного падения
const uint8_t gearRatio = 3; // передаточное отношение


uint8_t prev = 1; // предыдущий сигнал с оптопары
uint8_t now = 0;  // текущий сигнал с оптопары
unsigned long count = 0; // счет оборотов
unsigned long micPrev = 0; // предыдущее значение микросекунд
const uint8_t spaces = 60; // кол-во делений в диске






void handleRoot() {
  server.send(200, "text/html", index_html);
}

void control() {
  switch(server.arg("block").toInt()) {
    case 0:
      setPower(server.arg("details").toInt());
      break;
    case 1:
      set255MotorSpeedRange(server.arg("details").toInt(), 0);
      break;
    case 2:
      setRotationVector(server.arg("details").toInt());
      break;
    case 3:
      setWorkMode(server.arg("details").toInt());
      break;
  }
}

void getAllData() {
  sendMessageToServer(String(power) + " " + 
                      String(motorSpeedRange) + " " + 
                      String(rotationVector) + " " + 
                      String(workMode));
}

void getMotorSpeed() {
  sendMessageToServer(String(motorSpeed, 4));
}

void getMotorMoment() {
    sendMessageToServer(String(scale.get_units()*
                               0.035274/1000*g*h*
                               gearRatio, 4));
}



void setPower(uint8_t p) {
  power = p;
  if(power == 0) { // выключаем
    goToZeroPoint();
  }
  if(power == 1) { // включаем
    setRotationVector();
  }
}

void setMotorSpeedRange(uint8_t sr, bool s) {
  if(s == 1) motorSecondSpeedRange = motorSpeedRange;
  motorSpeedRange = 255/(maxSpeed/nominalSpeed*100)*sr;
  setRotationVector();
}
void set255MotorSpeedRange(uint8_t sr, bool s) {
  if(s == 1) motorSecondSpeedRange = motorSpeedRange;
  motorSpeedRange = sr;
  setRotationVector();
}

void setRotationVector(uint8_t v) {
  rotationVector = v;
  setRotationVector();
}
void setRotationVector() {
  if(power == 1) {
    if(rotationVector == 0) { // реверс
      turnReverse();    
    }
    if(rotationVector == 1) { // аверс
      turnAverse();
    }
  } else {
    stop();
  }
}

void setWorkMode(uint8_t m) {
  workMode = m;
  goToZeroPoint();
  count = 0;
  Serial.println("mode: " + String(workMode));
  delay(500);
  switch(workMode) {
    case 0: 
      mode0();
      break;
    case 1: 
      mode1();
      break;
    case 2: 
      mode2();
      break;
    case 3: 
      mode3();
      break;
  }  
}








void speedCalculate(void * pvParameters) {
  for(;;) {
    now = digitalRead(OP);
    if(now != prev) {
      prev = now;
      if(now == 1) {
        micPrev = micros();
      }
      if(now == 0) {
        motorSpeed = (((double)360/spaces)/
                     ((micros() - micPrev)/
                     1000000.0000000000))/360;
        count++;     
      }
    }
  }
}






void sendMessageToServer(String message) {
  server.send(200, "text/plane", message);
}

void turnReverse() {
  ledcWrite(1, 0);
  ledcWrite(2, motorSpeedRange);
}
void turnAverse() {
  ledcWrite(1, motorSpeedRange);
  ledcWrite(2, 0);
}
void stop() {
  ledcWrite(1, 0);
  ledcWrite(2, 0);
}
void speedToZero() {
  motorSpeedRange = 0;
  stop();
}
void goToZeroPoint() {  
  set255MotorSpeedRange(80, 1);
  if(rotationVector == 0) turnReverse();
  if(rotationVector == 1) turnAverse();
  while(count % (spaces*3) != 0) {
    Serial.println("L " + String(count));
    delay(1);
  }
  stop();
  motorSpeedRange = motorSecondSpeedRange;
}






// режимы
void mode0() {
  speedToZero();
  goToZeroPoint();
}
void mode1() {
  setRotationVector(1);
  while(count%(20*3) != 0) {}
  speedToZero();
  stop();
}
void mode2() {
  setRotationVector(0);
  while(count%(20*3) != 0) {}
  speedToZero();
  stop();
}
void mode3() {
  setRotationVector(1);
  while(count%(10*3) != 0) {}
  setRotationVector(0);
  while(count%(20*3) != 0) {}
  speedToZero();
  stop();
}

void setup() {
  Serial.begin(9600);

  WiFi.mode(WIFI_AP);
  if(WiFi.softAP(ssid, pass)) {
    WiFi.softAPConfig(APIP, APIP, IPAddress(255, 255, 255, 0));
    
    Serial.println();
    Serial.println("WiFi started at:");
    Serial.println("| ssid: " + String(ssid));
    if(String(pass).length() > 0) Serial.println("| pass: " + String(pass));
    Serial.println("| ip addr: " + WiFi.softAPIP().toString());
    Serial.println("| server: " + String(server_name));
    Serial.println("Не забудьте подключиться к данной сети WiFi, прежде чем заходить в панель управления.");

    dns.start(DNS_PORT, server_name, APIP);

    server.begin();
    server.on("/", handleRoot);
    server.on("/control", control);
    server.on("/getdata", getAllData);
    server.on("/getspeed", getMotorSpeed);
    server.on("/getgramms", getMotorMoment);


    scale.begin(DT, SCK);
    scale.set_scale(calibration_factor);
    scale.tare();

    ledcAttachPin(INA, 1);
    ledcAttachPin(INB, 2);
    ledcSetup(1, 12000, 8);
    ledcSetup(2, 12000, 8);

    xTaskCreatePinnedToCore(
      speedCalculate,
      "Task1",
      10000,
      NULL,
      0,
      &Task1,
      0
    );
  } else {
    Serial.println("failed to create AP");
  }
}

void loop() {
  dns.processNextRequest();
  server.handleClient();
}

