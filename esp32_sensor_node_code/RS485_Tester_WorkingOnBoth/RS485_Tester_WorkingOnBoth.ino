#include <ModbusMaster.h>

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
#define RS485_Rx 16 // ic recvr output 
#define RS485_Tx 17 // ic input
#define SAMPLE_SIZE 16// Sensor and IC control for power management
#define SENSOR_TIMEOUT 30000

ModbusMaster node;
  

void disableMAX485()
{
    pinMode(RS485_RE,INPUT_PULLUP);
    pinMode(RS485_GRD,INPUT);
    pinMode(RS485_DE,INPUT);

}

void enableMAX485()
{
    pinMode(RS485_RE,INPUT_PULLDOWN);
    pinMode(RS485_GRD,INPUT_PULLUP); //mosfet ctrl -- shouldn't this be pull UP?
    pinMode(RS485_DE,INPUT_PULLDOWN); // driver output en
}

void readRS485(){
  float humidity; 
  float soilTemperature;
  float conductivity;
  float temperature;
  float pH;
  float nitrogen;
  float phosphorus;
  float potassium;

  float pHOffset;

  uint8_t result;

  Serial.println(F("Read MAX485 Node Registers"));
  unsigned long start=millis();
  while((unsigned long)(millis()-start)<SENSOR_TIMEOUT)//check if the sensor has booted and has collected the data or timeout
  { 
      // ask for 7x 16-bit words starting at register address 0x0000
  uint8_t result = node.readHoldingRegisters( 0x0000, 7 );  
  
  if (result == node.ku8MBSuccess)
  {
    Serial.print("Reply:: ");
    Serial.print(node.getResponseBuffer( 0 ));
    Serial.print(" ");
    Serial.print(node.getResponseBuffer( 1 ));
    Serial.print(" ");
    Serial.print(node.getResponseBuffer( 2 ));
    Serial.print(" ");
    Serial.print(node.getResponseBuffer( 3 ));
    Serial.print(" ");
    Serial.print(node.getResponseBuffer( 4 ));
    Serial.print(" ");
    Serial.print(node.getResponseBuffer( 5 ));
    Serial.print(" ");
    Serial.print(node.getResponseBuffer( 6 ));
    Serial.println("");
  } else {
    // printModbusError( result );
  }
    // result = node.readHoldingRegisters(0x0000, 7);
    // if (result == node.ku8MBSuccess)
    // {
    //    humidity = ((float)node.getResponseBuffer(0x00)) * ((float)0.1);
    //    soilTemperature = ((float)node.getResponseBuffer(0x01)) * ((float)0.1);
    //    conductivity = ((float)node.getResponseBuffer(0x02));

    //    pH = ((float)node.getResponseBuffer(0x03))/10;
    //   //  pHOffset = ((float)node.getResponseBuffer(0x53)); // these values are messed up.`

    //    nitrogen = ((float)node.getResponseBuffer(0x04));
    //    phosphorus = ((float)node.getResponseBuffer(0x05));
    //    potassium = ((float)node.getResponseBuffer(0x06));
    //    float phOffset = 0; 

    //     // int resultMain = node.readInputRegisters(0x53, 1);
    //     //  if (resultMain == node.ku8MBSuccess) {
    //     //   Serial.println("Reading pH Offset");
    //     //   phOffset = node.getResponseBuffer(0x00);
    //     // }



      
    //   if(soilTemperature>0)
    //   {
    //     Serial.println(F("Success, Received data"));
    //     Serial.print("Humidity ");
    //     Serial.println(humidity);
    //     Serial.print("Temperature ");
    //     Serial.println(soilTemperature);
    //     Serial.print("Conductivity ");
    //     Serial.println(conductivity);
    //     Serial.print("pH ");
    //     Serial.println(pH);
    //     Serial.print("pH offset");
    //     Serial.println(pHOffset);
        
    //     Serial.print("Nitrogen ");
    //     Serial.println(nitrogen);
    //     Serial.print("Phosphorus ");
    //     Serial.println(phosphorus);
    //     Serial.print("Potassium ");
    //     Serial.println(potassium);


    //     break;
    //   }
    //   else delay(500);//wait for the sensor to bootup and load the values
    // }
    // else{
    //   Serial.println(result);
    //   Serial.println(F("Sensor didn't respond!"));
    //   Serial.flush();
    //   delay(500);//wait for the sensor to turn on
    // }
  }
  disableMAX485();
}


void sensor_init()
{
    Serial2.begin(4800, SERIAL_8N1, RS485_Rx, RS485_Tx);
    node.begin(1, Serial2);
    node.preTransmission(preTransmission);
    node.postTransmission(postTransmission); 
}

void preTransmission()
{
  Serial.println("DE_HIGH");
  pinMode(RS485_DE, OUTPUT);
  digitalWrite(RS485_DE, HIGH);
  // digitalWrite(RS485_RE, HIGH);

  delay(100);
  // pinMode(4, INPUT_PULLUP);
  // pinMode(RS485_DE, INPUT_PULLUP);
  pinMode(RS485_RE, INPUT_PULLUP);
  
}

void postTransmission()
{
  pinMode(RS485_DE, INPUT_PULLDOWN);
  pinMode(RS485_RE, OUTPUT);
  digitalWrite(RS485_RE, LOW);


  delay(500);
  
}

void setup() {
  pinMode(SENSOR_EN,INPUT_PULLUP); // shouldn't be a problem

  Serial.begin(115200);
  delay(300);
  sensor_init();    
  
  // put your setup code here, to run once:
  enableMAX485();
  delay(1000);
  Serial.print("Reading sensor\n");
  while(1) {
    readRS485();
    delay(1000);
  }
}

void loop() {
  // put your main code here, to run repeatedly:

}
