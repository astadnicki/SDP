#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
const int BUFFER_SIZE = 50;
char buf[BUFFER_SIZE];

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
  delay(2000);
  display.clearDisplay();
  delay(100);
}

void loop() {
  // put your main code here, to run repeatedly:
  //Serial.begin(9600);
  Serial.begin(115200);
  while (!Serial)   // 1/10 of a meter for set distance to go forward
    delay(0.1); // will pause Zero, Leonardo, etc until serial console opens // INITIALLY DELAY 10

  delay(10);
  
  int serialInput = Serial.read();
  //deserializeJson(doc, Serial);
  //String serialInput = doc["dir"];

  //driveForward(99);
  //  stopMoving();
  //if (Serial.readBytes(buf, BUFFER_SIZE) != 0){
  String inputString = String(serialInput);


  display.clearDisplay();

  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  // Display static text
  //display.println(s);
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  // Display static text
  display.display(); 
  if (inputString != NULL){
    display.println(inputString); 
  }
  else{
    display.println("didn't work");
  }
  delay(100);
  
}
