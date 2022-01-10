#include <esp_now.h>
#include <WiFi.h>

uint8_t controllerAddress[] = {0xE0, 0xE2, 0xE6, 0xB2, 0xFC, 0xB8};

const int leftMotorPin = 17;
const int rightMotorPin = 16;
const int batteryMonitorPin = 34;
float throttleMultiplier = 0;
int rightMotorPower = 0;
int leftMotorPower = 0;
float leftMotorMulti = 1.0;
float rightMotorMulti = 1.0;

bool connectionTest = false;

const int pwmFreq = 1000;
const int leftMotorChannel = 1;
const int rightMotorChannel = 2;
const int pwmResolution = 8;

typedef struct controller_message {
  int throttle;
  int steering;
} controller_message;

typedef struct vessel_message {
  float batteryLevel;
} vessel_message;

controller_message cm;
vessel_message vm;

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&cm, incomingData, sizeof(cm));
  connectionTest = true;
  return;
}
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if(status == ESP_NOW_SEND_SUCCESS){
    connectionTest = true;
    //Serial.println("Send Success");
  }
  else{
    connectionTest = false;
    throttleMultiplier = 0;
    ledcWrite(leftMotorChannel,0);
    ledcWrite(rightMotorChannel,0);
    //Serial.println("Failed Message");
  }
}
 
void setup() {
  //Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  pinMode(leftMotorPin,OUTPUT);
  pinMode(rightMotorPin,OUTPUT);
  pinMode(batteryMonitorPin,INPUT);

  ledcAttachPin(leftMotorPin, leftMotorChannel);
  ledcAttachPin(rightMotorPin, rightMotorChannel);

  ledcSetup(leftMotorChannel, pwmFreq, pwmResolution);
  ledcSetup(rightMotorChannel, pwmFreq, pwmResolution);

  ledcWrite(leftMotorChannel,0);
  ledcWrite(rightMotorChannel,0);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    //Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_peer_info_t controllerInfo;
  memcpy(controllerInfo.peer_addr, controllerAddress, 6);
  controllerInfo.channel = 0;  
  controllerInfo.encrypt = false;

  if (esp_now_add_peer(&controllerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_send_cb(OnDataSent);
}
 
void loop() {
  
  if(cm.throttle <= 1100 && connectionTest == true){
    throttleMultiplier = map(cm.throttle,0,1100,255,30);
    throttleMultiplier = throttleMultiplier / 100;
  } 
  else if(cm.throttle > 1100){
    throttleMultiplier = 0.0;
  }
  // Max 100 for steering multi
  if(cm.steering < 1300){
    int tempSteering = map(cm.steering, 1300,0,0,100);
    leftMotorMulti = (100 - tempSteering);
  } 
  else if (cm.steering > 2500){
    int tempSteering = map(cm.steering, 1800,4095,0,100);
    rightMotorMulti = (100 - tempSteering);
  }
  else{
    rightMotorMulti = 100;
    leftMotorMulti = 100;
  }
     
  rightMotorPower = throttleMultiplier * rightMotorMulti;
  leftMotorPower = throttleMultiplier * leftMotorMulti;

  ledcWrite(leftMotorChannel,leftMotorPower);
  ledcWrite(rightMotorChannel,rightMotorPower);
  
  //Serial.println("Left Motor Power : ");
  //Serial.println(leftMotorPower);
  //Serial.println(" Right Motor Power : ");
  //Serial.println(rightMotorPower);

  vm.batteryLevel = analogRead(batteryMonitorPin);

  esp_err_t result = esp_now_send(controllerAddress, (uint8_t *) &vm, sizeof(vm)); 
  
  if (result == ESP_OK){
  }

  else{
    //Serial.println("Error Sending the Data");
    ledcWrite(leftMotorChannel,0);
    ledcWrite(rightMotorChannel,0);
  }
  delay(50);
}