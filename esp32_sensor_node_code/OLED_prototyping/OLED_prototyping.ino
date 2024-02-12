#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define NUM_BULLETS 200
#define BUTTON_PIN 4

// declare an SSD1306 display object connected to I2C
Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
//oled is the name of the OLED object we just constructed

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); // initialize how many bits/s get communicated to the Serial monitor
  pinMode(BUTTON_PIN, INPUT);
  
  // initialize OLED display with address 0x3C for 128x64
  if (!oled.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    while (true);
  } 

}

//Example: increment from N to 0 if a push button is pushed
void loop() {
  // put your main code here, to run repeatedly:
  // initialize push button
  Serial.println("TEST!");
  int bulletCount = NUM_BULLETS;

  //delay(2000); //wait for initialization
  oled.clearDisplay(); //clear display

  oled.setTextSize(4);          // text size
  oled.setTextColor(WHITE);     // text color
  oled.setCursor(30, 10);        // position to display

  Serial.println(digitalRead(BUTTON_PIN)); //testing
  delay(100);

  while(bulletCount > 0) {
    oled.setCursor(30, 10);        // position to display
    oled.println(bulletCount); // text to display
    oled.display();
    
    // if(digitalRead(BUTTON_PIN)) {
    //   delay(500);
    //   bulletCount--;
    // }      
    delay(100);
    bulletCount--;
    oled.clearDisplay(); //remember to clear screen

  }
}
