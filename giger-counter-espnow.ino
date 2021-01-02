#define DEBUG 1

const int led = 2;
int ledStatus = 0;
char payload[32];
unsigned long lastTime = 0;
unsigned long timerDelay = 1000;
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

#define SENSOR_PIN 5
#define MEASUREMENT_SECS 60

volatile unsigned int newTicks = 0;
unsigned int ticksLog[MEASUREMENT_SECS];
unsigned int tickCountsLog[MEASUREMENT_SECS];
int logsIndex = -1;
unsigned int ticksCount = 0;
unsigned int ticksCountPerSec = 0;
unsigned long tickCountsLogSum = 0;
unsigned long lastTickLogsUpdateTime = 0;
unsigned long lastVoltageReadTime = 0;

String xxx;

#include <ESP8266WiFi.h>
#include <espnow.h>

// callback function that will be executed when data is received
void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len) {

//  memcpy(&payload, incomingData, sizeof(payload));
//
//  char macStr[18];
//  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
//
//  Serial.print(">");
//  Serial.print(macStr);
//  Serial.print(",");
//  Serial.print(payload);
//  Serial.println("<");
//
//  if (ledStatus == 0) {
//    digitalWrite(led, HIGH);
//    ledStatus = 1;
//  } else {
//    digitalWrite(led, LOW);
//    ledStatus = 0;
//  }

}

// callback function that will be executed when data is sent
void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus) {

}

void ICACHE_RAM_ATTR sensorISR() {
  newTicks++;
}

void setup() {

  // led on
  pinMode(led, OUTPUT);
  digitalWrite(led, HIGH);
  delay(2000);

  // led off
  digitalWrite(led, LOW);

  pinMode(SENSOR_PIN,INPUT_PULLUP);

  // serial
  Serial.begin(115200);
  delay(random(1000,5000));

  // set device as a Wi-Fi station
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  // init ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("error initializing ESP-NOW");
    delay(1000);
    ESP.restart();
  }

  // once ESP-NOW is successfully initalized, register callbacks
  esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
  esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_send_cb(OnDataSent);

  // register peer(s)
  esp_now_add_peer(broadcastAddress, ESP_NOW_ROLE_COMBO, 1, NULL, 0);

  // led off
  digitalWrite(led, HIGH);

  // rad interrupt
  attachInterrupt(digitalPinToInterrupt(SENSOR_PIN), sensorISR, FALLING);

}

void loop() {

  unsigned int _newTicks = newTicks;
  newTicks = 0;
  
  updateTicks(_newTicks); 
  updateTickLogs();

  boolean hasNewTicks = (_newTicks > 0);


  if ((millis() - lastTime) > timerDelay) {
    
    
    strcpy(payload, "PING,RADD");
    //esp_now_send(broadcastAddress, (uint8_t *) &payload, sizeof(payload));

    xxx = String("CPM,") + ticksCount;
    Serial.println(xxx);

    Serial.print(" CPM: ");
    Serial.println(ticksCount);

    
    
    lastTime = millis();
  }
    
}

void updateTicks(unsigned int newValue) {
  ticksCountPerSec += newValue;
  ticksCount += newValue;

#if DEBUG
  if(newValue > 0) {
    Serial.print(" CPM: ");
    Serial.println(ticksCount);
  }
#endif
}

void updateTickLogs() {
  if(!isTimeOut(lastTickLogsUpdateTime, 1000)) {
    return;
  }
      
  if(logsIndex < (MEASUREMENT_SECS - 1)) {
    logsIndex++;
  } else {
    ticksCount -= ticksLog[0];
    tickCountsLogSum -= tickCountsLog[0];
    
    for(byte i = 1; i <= logsIndex; i++) {
      ticksLog[i - 1] = ticksLog[i];
      tickCountsLog[i - 1] = tickCountsLog[i];
    }
  }

  ticksLog[logsIndex] = ticksCountPerSec;
  ticksCountPerSec = 0;
  
  tickCountsLog[logsIndex] = ticksCount;
  
  tickCountsLogSum += ticksCount;
}

boolean isTimeOut(unsigned long& lastTimestamp, unsigned long duration) {
  unsigned long currentTimestamp = millis();
  
  if((currentTimestamp - lastTimestamp) < duration) {
    return false;
  }
  
  lastTimestamp = currentTimestamp;

  return true;
}
