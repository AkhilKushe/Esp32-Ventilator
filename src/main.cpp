#include <Arduino.h>
#include "sensors.h"
#include "ESPAsyncWebServer.h"
#include "SPIFFS.h"

// #define spi_cs 25
// #define spi_clk 26
// #define spi_mosi 27
// #define spi_miso 14

#define i2c_sda 21
#define i2c_scl 22

#define valve_1 26               //26
#define valve_2 27              //PEEP valve, i.e. valve 2

#define ADS1115ADDRESS 0x48
#define flowAddress 0x40
#define pressureAddr 0x28
#define BlowerAddress 0x00
// Replace with your network credentials
const char* ssid = "abcd";
const char* password = "qwertyui";

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

SemaphoreHandle_t mutex;
QueueHandle_t s_data;
TaskHandle_t flow_handle = NULL;
bool flow_flag = false;


typedef struct sensor_data_t{
  float flow;
  float volume;
  float pressure;
  float oxygen;
}sensor_data_t; 


oxy_config_t oxy_config = {
  .address = ADS1115ADDRESS,
  .port = 1,
};

Valve v1(valve_1);
Valve v2(valve_2);
Flow_sensor flow(flowAddress);
Oxy_sensor oxy(oxy_config);
Pressure_sensor P(pressureAddr);


void all_sensor_get(void* pvParameters){
  sensor_begin(i2c_sda, i2c_scl);
  flow.flow_begin();
  uint16_t vol = 0;
  unsigned long mt = millis();
  sensor_data_t sensor_data;
  while(1){

    sensor_data.flow = flow.getRAWReading();
    sensor_data.oxygen = oxy.getRAWReading();
    sensor_data.pressure = P.getRAWReading();

    float new_flow = sensor_data.flow;
    unsigned long mt_delta = millis() - mt;
    mt = millis();
    vol += (new_flow/60)*mt_delta;
    vol = constrain(vol, 0, 10000);
    sensor_data.volume = vol;
    delay(10);
    xQueueOverwrite(s_data, (void*)&sensor_data);
  }
}

sensor_data_t all_sensor_read(){
  sensor_data_t s_read;
  if(xQueuePeek(s_data, (void*)&s_read, 0)==pdTRUE){
    //Serial.println("Data recieved");
  }
  // Serial.print("Flow : ");
  // Serial.println(s_read.flow);
  // Serial.print("Volume : ");
  // Serial.println(s_read.volume);
  // Serial.print("Pressure : ");
  // Serial.println(s_read.pressure);
  // Serial.print("Oxygen conc. : ");
  // Serial.println(s_read.oxygen);
  return s_read;
}

void maintain_flow(void* pvParameter){
  Blower_i2c blower(BlowerAddress);
  int* inputs = (int*)pvParameter;
  int req_flow = *inputs;
  sensor_data_t s_read;
  uint16_t rpm = 0; 
  //Set initial flow,   
  while(1){
    xQueuePeek(s_data, (void*)&s_read, 0);
    if((s_read.flow - req_flow) < -1 ) blower.setRPM(++rpm);                 //set tolerance level (-1, 1)
    else if((s_read.flow - req_flow) > 1) blower.setRPM(--rpm);
    else break;
  }     
  // set flow flag to indicate initial flow achieved
  flow_flag = true;
  while(1){
    xQueuePeek(s_data, (void*)&s_read, 0);
    if((s_read.flow - req_flow) < -1 ) blower.setRPM(++rpm);                 //set tolerance level (-1, 1)
    else if((s_read.flow - req_flow) > 1) blower.setRPM(--rpm);
  }
}

void volume_control(void* pvParameter){
  int* inputs = (int*)pvParameter;
  
}

char v1_state, v2_state;

void setup() {  
  //Sensor Setup
  Serial.begin(115200);
  //Serial2.begin(115200, SERIAL_8N1, 16, 17);                            //Use for external usb uart

  //initialize file system
  if(!SPIFFS.begin(true)){
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }

  s_data = xQueueCreate(1, sizeof(sensor_data_t));    // Queue for sharing sensor data
  //Create multiple tasks here
  xTaskCreate(all_sensor_get, "get_sensor_d", 2048, NULL, 5, NULL);

  //Web Server setup
  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi..");
  }

  // Print ESP32 Local IP Address
  Serial.println(WiFi.localIP());

  // Route for root / web page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/index.html");
  });
  server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/style.css", "text/css");
  });

  server.on("/flow", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", String(all_sensor_read().flow, 3u).c_str());
  });
  server.on("/volume", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", String(all_sensor_read().volume, 3u).c_str());
  });
  server.on("/pressure", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", String(all_sensor_read().pressure, 3u).c_str());
  });
  server.on("/oxygen", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", String(all_sensor_read().oxygen, 3u).c_str());
  });


  server.on("/valve_1", HTTP_GET, [](AsyncWebServerRequest *request){
    v1.toggle();
    Serial.println("valve 1 toggled");
    request->send_P(200, "text/plain", String(digitalRead(valve_1)).c_str());
  });
  server.on("/valve_2", HTTP_GET, [](AsyncWebServerRequest *request){
    v2.toggle();
    Serial.println("Valve 2 toggles");
    request->send_P(200, "text/plain", String(digitalRead(valve_2)).c_str());
  });


  // Start server
  server.begin();
  
} 


//long unsigned int start = millis();
void loop() {
  // delay(500);
  // if (millis()-start > 8000){
  //   v1.toggle();
  //   v2.toggle();
  //   start = millis();
  //   Serial.println("Toggled");
  // }
}