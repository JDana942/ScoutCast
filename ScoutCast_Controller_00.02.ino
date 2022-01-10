#include <esp_now.h>
#include <WiFi.h>

uint8_t vesselAddress[] = {0xE0, 0xE2, 0xE6, 0xB2, 0xE7, 0x2C};

const int steeringPin = 34;
const int throttlePin = 35;
const int connectStatusPin = 21;

// Structure example to receive data
// Must match the sender structure
typedef struct controller_message {
  int throttle;
  int steering;
} controller_message;

typedef struct vessel_message {
  float batteryLevel;
} vessel_message;

controller_message cm;
vessel_message vm;

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&vm, incomingData, sizeof(vm));  
  return;
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if(status == ESP_NOW_SEND_SUCCESS){
    //Serial.println("Send Success");
    digitalWrite(connectStatusPin, 1);
  }
  else{
    digitalWrite(connectStatusPin,0);
    //Serial.println("Failed Message");
  }
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  pinMode(throttlePin,INPUT);
  pinMode(steeringPin,INPUT);
  pinMode(connectStatusPin,OUTPUT);

  digitalWrite(connectStatusPin,0);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register peer
  esp_now_peer_info_t vesselInfo;
  memcpy(vesselInfo.peer_addr, vesselAddress, 6);
  vesselInfo.channel = 0;  
  vesselInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&vesselInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_send_cb(OnDataSent);
}

void loop() {
  // Set values to send
  cm.throttle = analogRead(throttlePin);
  cm.steering = analogRead(steeringPin);

  Serial.println("Throttle : ");
  Serial.println(cm.throttle);
  Serial.println(" Steering : ");
  Serial.println(cm.steering);
 
  Serial.println("Battery Level : ");
  int tempBattery = constrain(vm.batteryLevel,3000,4095);
  int batteryPercentage = map(tempBattery,3000,4095,0,100);
  Serial.println(batteryPercentage);
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(vesselAddress, (uint8_t *) &cm, sizeof(cm));
  
  if (result == ESP_OK) {
  }
  else {
    Serial.println("Error sending the data");
  }
  delay(50);
}