/*  
    Copyright (C) 2023, Network Research Lab at the University of Toronto.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License along
    with this program; if not, write to the Free Software Foundation, Inc.,
    51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.

*/


// Include necessary libraries
#include <HardwareSerial.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include <driver/adc.h>
#include <esp_wifi.h>
#include <ModbusMaster.h>

/**************************************************************************************************************************************************************/
//                                                          GLOBAL DECLARATIONS AND INITIALIZATIONS
/**************************************************************************************************************************************************************/

// Pin Definitions and other macros
#define RS485_DE 4
#define RS485_RE 15
#define ONE_WIRE_BUS 22
#define SENSOR_EN 25
#define RS485_GRD 26
#define ccWakeUp 27
#define ccReqData 5
#define MAX_BATCH_SIZE 50
#define ccRx 14
#define ccTx 12
#define RS485_Rx 16
#define RS485_Tx 17
#define actuateWakeUp 33
#define SAMPLE_SIZE 32 //size of the datapacket to be sent at once, so that data is not lost in fragmentation
#define AIR_TEMP 1
#define ACTUATE_INTENT 0xFF

//Timeout Configuration
#define uS_TO_S_MULTIPLIER 1000000  
#define TIMEOUT_LIGHT_SLEEP  60 //change TIMEOUT_LIGHT_SLEEP for setting timeout in secs 
#define SENSOR_TIMEOUT 15000


// Create instances of OneWire and DallasTemperature classes
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);


// Global variables
uint8_t dataBufferSize = 0;     // Size of the packet
volatile bool isCollectionPhase = false;   // Flag indicating to make data collection active
QueueHandle_t msg_queue = xQueueCreate(15, sizeof(float));   // Message queue for storing data
volatile bool isTransmitPhase = false;  // Flag indicating to make data transmission active
unsigned int collectionCounter = 2;  // Counter for data collection, change to UINT_MAX for continuous collection
HardwareSerial SerialPort(1);
ModbusMaster node;


xTaskHandle xCollectionTask;   // Task handle for collecting data
xTaskHandle xTransmissionTask;   // Task handle for transmiting data

// Task Declarations
void TransmitLocalData(void *pvParameters);
void CollectLocalData(void *pvParameters);
void actuatePump();

/**************************************************************************************************************************************************************/
//                                                                  End of Declarations
/**************************************************************************************************************************************************************/





/**************************************************************************************************************************************************************/
//                                                                    HELPER FUNCTIONS
/**************************************************************************************************************************************************************/
void actuatePump() {
  Serial.println("Actuating Pump");
  pinMode(32, OUTPUT);
  digitalWrite(32, HIGH);
  delay(3000);
  digitalWrite(32, LOW);
}

// This function will convert any datatype to byte packets to transmit using UART
void send_bytes(void *val,int packet_size)
{
    
    byte* byte_ptr=(byte*)val;
    // For Debugging Purposes
    uint8_t i=0;
    while(i<packet_size){
      Serial.print(*(byte_ptr+i),HEX);
      Serial.print(".");
      i++;
    }
    Serial.println();
    SerialPort.write(byte_ptr,packet_size);
    SerialPort.flush();

}

// Sensor and IC control for power management
void disableMAX485()
{
    pinMode(RS485_RE,INPUT_PULLUP);
    pinMode(RS485_GRD,INPUT);
    pinMode(RS485_DE,INPUT);
}

void enableMAX485()
{
    pinMode(RS485_RE,INPUT_PULLDOWN);
    pinMode(RS485_GRD,INPUT_PULLUP); // should be pullup, for some reason this code is PD
    pinMode(RS485_DE,INPUT_PULLDOWN);
}

void enableSensor()
{
    pinMode(ONE_WIRE_BUS,INPUT_PULLUP);
    pinMode(SENSOR_EN,INPUT_PULLUP);
}

void disableSensor()
{
    pinMode(SENSOR_EN,INPUT_PULLDOWN);
}

void send_esp_to_deep_sleep()
{
    disableSensor();
    disableMAX485();
    pinMode(ccTx,INPUT);
    pinMode(ccRx,INPUT);
    esp_deep_sleep_start();
}

void esp_sleep_config_low_power()
{
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH,ESP_PD_OPTION_ON);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_OFF);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_OFF);
    esp_sleep_pd_config(ESP_PD_DOMAIN_XTAL, ESP_PD_OPTION_OFF);
    esp_sleep_pd_config(ESP_PD_DOMAIN_VDDSDIO, ESP_PD_OPTION_OFF);
    
    gpio_pulldown_en((gpio_num_t)ccWakeUp);
    gpio_pulldown_en((gpio_num_t)ccReqData);
    gpio_pulldown_en((gpio_num_t)actuateWakeUp);

    gpio_wakeup_enable((gpio_num_t)ccReqData, GPIO_INTR_HIGH_LEVEL);
    esp_sleep_enable_ext1_wakeup(GPIO_SEL_33 | GPIO_SEL_27, ESP_EXT1_WAKEUP_ANY_HIGH);


    esp_sleep_enable_gpio_wakeup();
 
}

void send_esp_to_light_sleep()
{
    disableSensor();
    disableMAX485();
    esp_sleep_enable_timer_wakeup(TIMEOUT_LIGHT_SLEEP*uS_TO_S_MULTIPLIER);
    esp_light_sleep_start();
    check_wakeup_reason();
}

// User Defined function to collect data from sensors
uint8_t datacollect(float *humidity, float *soilTemperature, float *conductivity,float *temperature, float *pH, float *nitrogen, float *potassium, float *phosphorus) 
{
  // enabling the sensor and IC
  enableMAX485();
  delay(300);
  enableSensor();
  delay(300);

  if(AIR_TEMP) {
    // Collecting Dallas OneWire Temperature Sensor Data
      sensors.requestTemperatures(); 
    *temperature=((float)sensors.getTempCByIndex(0));
  }
  else {
    *temperature = 0;
  }
  


  // Collect Soil Sensor Data
  uint8_t result;

  Serial.println(F("Read MAX485 Node Registers"));
  unsigned long start=millis();
  while((unsigned long)(millis()-start)<SENSOR_TIMEOUT)//check if the sensor has booted and has collected the data or timeout
  {
    result = node.readHoldingRegisters(0x0000, 7);
    if (result == node.ku8MBSuccess)
    {
      *humidity = ((float)node.getResponseBuffer(0x00)) * ((float)0.1);
      *soilTemperature = ((float)node.getResponseBuffer(0x01)) * ((float)0.1);
      *conductivity = ((float)node.getResponseBuffer(0x02));
      *pH = ((float)node.getResponseBuffer(0x03))/10;
      *nitrogen = ((float)node.getResponseBuffer(0x04));
      *phosphorus = ((float)node.getResponseBuffer(0x05));
      *potassium = ((float)node.getResponseBuffer(0x06));
      
      if(*soilTemperature>0)
      {
        Serial.println(F("Success, Received data"));
        break;
      }
      else delay(500);//wait for the sensor to bootup and load the values
    }
    else{
      Serial.println(result);
      Serial.println(F("Sensor didn't respond!"));
      Serial.flush();
      delay(500);//wait for the sensor to turn on
    }
  }

  disableSensor();
  delay(500);
  disableMAX485();
  if(result==0)return 1;
  else return 0;
}

// For performing the wakeup stub
void check_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;
  uint64_t wakeupBit;
  wakeup_reason = esp_sleep_get_wakeup_cause();
  Serial.println("Checking wakeup reason");

  switch(wakeup_reason)
  { 
    case ESP_SLEEP_WAKEUP_EXT1 : 
      Serial.println(F("Waking up the ESP from deep sleep!")); 
      Serial.flush();
      
      wakeupBit = esp_sleep_get_ext1_wakeup_status();
      if (wakeupBit & GPIO_SEL_33) {
        Serial.println("Selected pin 33");
        actuatePump();
      }
      else if (wakeupBit & GPIO_SEL_27) {
        Serial.println("collection phase TRUE");
        isCollectionPhase=true;
      }     
      break;
    case ESP_SLEEP_WAKEUP_TIMER :
      Serial.println(F("Timed out, Going to deep sleep")); 
      Serial.flush();
      esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
      send_esp_to_deep_sleep();
      break;
    
    case ESP_SLEEP_WAKEUP_GPIO:
      Serial.println(F("Waking up the ESP from Light Sleep")); 
      Serial.flush();
      esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
      break;

    default : 
      Serial.println(F("Going to deep sleep\n"));
      Serial.flush();
      send_esp_to_deep_sleep(); 
      break;
  }
}


void sensor_init()
{
    Serial2.begin(4800, SERIAL_8N1, RS485_Rx, RS485_Tx);
    node.begin(1, Serial2);
    node.preTransmission(preTransmission);
    node.postTransmission(postTransmission); 
    sensors.begin();
}

void preTransmission()
{
  Serial.println("DE_HIGH");
  pinMode(RS485_DE, OUTPUT);
  digitalWrite(RS485_DE, HIGH);
  delay(100);
  // pinMode(RS485_DE, INPUT_PULLUP);
  pinMode(RS485_RE, INPUT_PULLUP);
  
}

void postTransmission()
{
  pinMode(RS485_RE, OUTPUT);
  digitalWrite(RS485_RE, LOW);
  pinMode(RS485_DE, INPUT_PULLDOWN);

  delay(500);
  
}



/**************************************************************************************************************************************************************/
//                                                                 End of Helper Functions
/**************************************************************************************************************************************************************/




/**************************************************************************************************************************************************************/
//                                                                     Main Programme
/**************************************************************************************************************************************************************/

void setup() {
  SerialPort.begin(9600, SERIAL_8N1, ccRx, ccTx);
  pinMode(32, OUTPUT); // Actuation transistor output
  // delay(100);
  Serial.begin(115200);
  unsigned long startTime = millis();
  uint8_t myIntent;
  Serial.println("Before Reading Intent");
  while((unsigned long)(millis()-startTime) < 5000) {
    // Serial.print("time: ");
    // Serial.println(millis()-startTime);
    if(SerialPort.available()) {
      myIntent = (byte)SerialPort.read();
      Serial.print("Intent: ");
      Serial.println(myIntent, HEX);
      break;
    }
    delay(10);
  }


  delay(300);
  sensor_init();
  delay(100);
  esp_sleep_config_low_power();
  check_wakeup_reason();

  // if(myIntent == ACTUATE_INTENT) {
  //   actuatePump();
  // }

  // should disable max chip on startup;
  disableMAX485();
  
  //If u get the freertos error for out of stack memory increase the stack size here
  xTaskCreatePinnedToCore(TransmitLocalData,"Communication",2048,NULL,1,&xTransmissionTask,0);
  xTaskCreatePinnedToCore(CollectLocalData,"Read_Sensor_Data",2048,NULL,1,&xCollectionTask,1);

}


void TransmitLocalData(void * pvParameters)
{
  
  while(1)
  {
    float value=0;
    bool isDataBufferEmpty=false;
    if(isTransmitPhase){
      while(!isDataBufferEmpty){
        if(dataBufferSize<=MAX_BATCH_SIZE)
        {
          uint8_t packetSize=dataBufferSize;
          send_bytes(&packetSize,1);   
          while(xQueueReceive(msg_queue, &value, (TickType_t)5)==pdPASS){
            send_bytes(&value,4);
            dataBufferSize=dataBufferSize-4;
          }
          Serial.print(packetSize);
          Serial.println(" packets sent");
          isDataBufferEmpty=true;
          break;
        }
        else
        {
          uint8_t packetSize=(MAX_BATCH_SIZE/SAMPLE_SIZE)*(SAMPLE_SIZE);
          send_bytes(&packetSize,1);
          for(int i=0;i<packetSize;i+=4)
          {
            if(xQueueReceive(msg_queue, &value, (TickType_t)5)==pdPASS)
            {
              send_bytes(&value,4);
              dataBufferSize=dataBufferSize-4;
            }
            else
            {
              dataBufferSize=0;
              isDataBufferEmpty=true;
              break;
            }
          }
        }
        if(!isDataBufferEmpty)send_esp_to_light_sleep(); 
      }

      if(collectionCounter<=0){
        Serial.println(F("Message Sent Going to deep sleep now"));
        Serial.flush();
        send_esp_to_deep_sleep();
      }
      else{
        isTransmitPhase=false;
        isCollectionPhase=true;
        vTaskResume(xCollectionTask);

      }
    }

    else vTaskSuspend(xTransmissionTask);

  }
  
}


void CollectLocalData(void * pvParameters)
{
    dataBufferSize=0;
    while(1){
          if(collectionCounter==0)
          {
            Serial.println(F("No more sensor data to collect, going to sleep"));
            Serial.flush();
            send_esp_to_deep_sleep(); 
          }
          else if(collectionCounter==UINT_MAX);
          else collectionCounter--;
        
          unsigned long start=millis();
          while((unsigned long)(millis()-start)<SENSOR_TIMEOUT)
          {
            if(isCollectionPhase){
            
                float hum,temp,cond,stemp,pH,nitrogen, phosphorus, potassium;
                hum=0;
                temp=0;
                stemp=0;
                cond=0;
                pH=0;
                nitrogen=0;
                phosphorus=0;
                potassium=0;

                if(datacollect(&hum, &stemp, &cond, &temp, &pH, &nitrogen, &phosphorus, &potassium)){
                    Serial.print("Sending Sensor Data to queue [H sT C T pH N P K]: ");
                    Serial.print(hum);
                    Serial.print(" ");
                    Serial.print(stemp);
                    Serial.print(" ");
                    Serial.print(cond);
                    Serial.print(" ");
                    Serial.print(temp);
                    Serial.print(" ");
                    Serial.print(pH);
                    Serial.print(" ");
                    Serial.print(nitrogen);
                    Serial.print(" ");
                    Serial.print(phosphorus);
                    Serial.print(" ");
                    Serial.println(potassium);
                    Serial.flush();
                    xQueueSendToBack(msg_queue, &hum, (TickType_t)0);
                    xQueueSendToBack(msg_queue, &stemp, (TickType_t)0);
                    xQueueSendToBack(msg_queue, &cond, (TickType_t)0);
                    xQueueSendToBack(msg_queue, &temp, (TickType_t)0);
                    xQueueSendToBack(msg_queue, &pH, (TickType_t)0);
                    xQueueSendToBack(msg_queue, &nitrogen, (TickType_t)0);
                    xQueueSendToBack(msg_queue, &phosphorus, (TickType_t)0);
                    xQueueSendToBack(msg_queue, &potassium, (TickType_t)0);
                    dataBufferSize+=32;
                    break;
                }
            
            }
            
            else break;
          
          }

          isCollectionPhase=false;
          send_esp_to_light_sleep();
          isTransmitPhase=true;
          vTaskResume(xTransmissionTask);
          vTaskSuspend(xCollectionTask);
    }
}

void loop() {

}
/**************************************************************************************************************************************************************/
//                                                                 End of Main Programme
/**************************************************************************************************************************************************************/
