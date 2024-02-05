#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

int Left = 2;
int LeftDirection = 10;
int Right = 3;
int RightDirection = 11;

#define S0 4
#define S1 5
#define S2 6
#define S3 7
#define sensorOut 8

// Stores frequency read by the photodiodes
int redFrequency = 0;
int greenFrequency = 0;
int blueFrequency = 0;

void setup(void) {

  pinMode(Left, OUTPUT);  // Left
  pinMode(Right, OUTPUT);  // Right
  pinMode(LeftDirection, OUTPUT); // Left Direction
  pinMode(RightDirection, OUTPUT); // Right Direction

  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);

  // Setting the sensorOut as an input
  pinMode(sensorOut, INPUT);
  
  // Setting frequency scaling to 20%
  digitalWrite(S0,HIGH);
  digitalWrite(S1,LOW);

  Serial.begin(9600);
  /*
  while (!Serial)   // 1/10 of a meter for set distance to go forward
    delay(0.1); // will pause Zero, Leonardo, etc until serial console opens // INITIALLY DELAY 10
  */
  //Serial.println("Adafruit MPU6050 test!");
  
  // Try to initialize!
  if (!mpu.begin()) {
    //Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(0.1); // INITIALLY DELAY 10
    }
  }
  //Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);

  //Serial.println("");

  //driveLeft(8,50);
  //stopMoving();

  //CORRECT LOOP SEQUENCE
  //digitalWrite(2, HIGH);
  /*
  driveForward(99);
  stopMoving();
  driveRight(99,50);
  stopMoving();
  driveForward(99);
  stopMoving();
  driveRight(99,50);
  stopMoving();
  driveForward(99);
  stopMoving();
  driveRight(99,50);
  stopMoving();
  driveForward(99);
  stopMoving();
  driveRight(99,50);
  stopMoving();
*/
  //driveForward(99);
  //driveRight(99,50);  // 50% of pi (90 degree) angle
  //driveLeft(99,50);  // 50% of pi (90 degree) angle
  //stopMoving();
  
  delay(100);
}

void loop() {

  //analogWrite(2, 255);
  // 50-99: Positive speed (50 is 0 or just weak, 99 is highest)
  // 0-49: Negative speed (0 is 0 or just week, 49 is highest)
  // Commands: A is forward, B is right, C is left, D is nothing (empty) can drop the rest of the characters
  // Characters (space in between 1st, 2nd, 3rd)
  // 1st: Command (1 character), 2nd: Speed (2 characters), 3rd: Angle (2 characters)

  //driveForward(99);
  //delay(500);
  //driveRight(99,50);  // 50% of pi (90 degree) angle
  //delay(500);
  //driveLeft(99,50);  // 50% of pi (90 degree) angle
  //delay(500);
  //stopMoving();
  //delay(400);

  delay(10);
  
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    if (data.substring(0,1) == "A") {
      driveForward(99, data.substring(6,7));
      stopMoving();
      Serial.print("AA 99 00");
    } else if (data.substring(0,1) == "B") {
      driveLeft(99, 50);
      Serial.print("BB 99 50");
      stopMoving();
    } else if (data.substring(0,1) == "C") {
      driveRight(99, 50);
      Serial.print("CC 99 50");
      stopMoving();
    } else if (data.substring(0,1) == "D") {
      stopMoving();
      Serial.print("DD 00 00");
    }
    //stopMoving();
    //Serial.println("Ack");
  }
  
  /*
  if (serialInput.substring(0,1) == "A") {
    driveForward(serialInput.substring(2,4).toInt());
  } else if (serialInput.substring(0,1) == "B") {
    driveLeft(serialInput.substring(2,4).toInt(), serialInput.substring(5,7).toInt());
  } else if (serialInput.substring(0,1) == "C") {
    driveRight(serialInput.substring(2,4).toInt(), serialInput.substring(5,7).toInt());
  } else if (serialInput.substring(0,1) == "D") {
    stopMoving();
  }
  */

  // Setting RED (R) filtered photodiodes to be read
  digitalWrite(S2,LOW);
  digitalWrite(S3,LOW);
  
  // Reading the output frequency
  redFrequency = pulseIn(sensorOut, LOW);
  
   // Printing the RED (R) value
  Serial.print("R = ");
  Serial.print(redFrequency);
  delay(100);
  
  // Setting GREEN (G) filtered photodiodes to be read
  digitalWrite(S2,HIGH);
  digitalWrite(S3,HIGH);
  
  // Reading the output frequency
  greenFrequency = pulseIn(sensorOut, LOW);
  
  // Printing the GREEN (G) value  
  Serial.print(" G = ");
  Serial.print(greenFrequency);
  delay(100);
 
  // Setting BLUE (B) filtered photodiodes to be read
  digitalWrite(S2,LOW);
  digitalWrite(S3,HIGH);
  
  // Reading the output frequency
  blueFrequency = pulseIn(sensorOut, LOW);
  
  // Printing the BLUE (B) value 
  Serial.print(" B = ");
  Serial.println(blueFrequency);
  delay(100);

  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  /* Print out the values */
  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");

  Serial.print("Rotation X: ");
  Serial.print(g.gyro.x);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y);
  Serial.print(", Z: ");
  Serial.print(g.gyro.z);
  Serial.println(" rad/s");

  Serial.print("Temperature: ");
  Serial.print(temp.temperature);
  Serial.println(" degC");
  
}

int driveForward(int speed){  // 23 inches moved for 350ms (IMU says on average 0.7)
  int pulse = speed;
  int error = 1;
  int timeDelay = 350;
  if (speed >= 50) {
    pulse = ((speed+100)/200) * 255;
    if (pulse < 130) {
      pulse = 130;
    }
    // HIGH all wheels forward
    digitalWrite(LeftDirection, LOW);
    digitalWrite(RightDirection, LOW);
    analogWrite(Left, 255); // pulse
    analogWrite(Right, 255); // pulse
    delay(500); // timeDelay
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    float dx2 = (a.acceleration.x - 0.44)*timeDelay*timeDelay/1000000 + 0.5*(a.acceleration.x - 0.44)*timeDelay*timeDelay/1000000;
    float dy2 = (a.acceleration.y + 0.66)*timeDelay*timeDelay/1000000 + 0.5*(a.acceleration.y + 0.66)*timeDelay*timeDelay/1000000;
    float d = sqrt(pow(dx2, 2) + pow(dy2, 2));

    // Reading color outputs

    // Setting RED (R) filtered photodiodes to be read
    digitalWrite(S2,LOW);
    digitalWrite(S3,LOW);
    // Reading the output frequency
    redFrequency = pulseIn(sensorOut, LOW);

    // Setting GREEN (G) filtered photodiodes to be read
    digitalWrite(S2,HIGH);
    digitalWrite(S3,HIGH);
    // Reading the output frequency
    greenFrequency = pulseIn(sensorOut, LOW);
    
    if ((redFrequency >= 65) && (redFrequency <= 80) && (greenFrequency >= 35) && (greenFrequency >= 40) && (blueFrequency >= 30) && (blueFrequency <= 50)) {    //we see blue sticker
      //checking for rotations
    }
    /*
    Serial.print("Acceleration X: ");
    Serial.print(a.acceleration.x);
    Serial.print(", Y: ");
    Serial.print(a.acceleration.y);
    Serial.print(", Z: ");
    Serial.print(a.acceleration.z);
    Serial.println("distance x:");
    Serial.println(dx2);
    Serial.println("distance y:");
    Serial.println(dy2);
    Serial.println("total distance:");
    Serial.println(d);
    */
    while (error == 1) {
      if (d < 0.2) {
        error = 0;
        //error = 1;
        //timeDelay = 50; // continue running function for another 50ms
        //delay(timeDelay);
      } else {
        error = 0;
      }
      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);
      dx2 = dx2 + ((a.acceleration.x - 0.44)*timeDelay*timeDelay/1000000 + 0.5*(a.acceleration.x - 0.44)*timeDelay*timeDelay/1000000);
      dy2 = dy2 + ((a.acceleration.y + 0.66)*timeDelay*timeDelay/1000000 + 0.5*(a.acceleration.y + 0.66)*timeDelay*timeDelay/1000000);
      d = sqrt(pow(dx2, 2) + pow(dy2, 2));
      /*
      Serial.println("total distance:");
      Serial.println(d);
      */
    }
  }
  if (speed < 50) {
    pulse = (((speed*2)+100)/200) * 255;
    if (pulse < 130) {
      pulse = 130;
    }
    // HIGH all wheels reverse
    digitalWrite(LeftDirection, HIGH);
    digitalWrite(RightDirection, HIGH);
    analogWrite(Left, pulse);
    analogWrite(Right, pulse);
    delay(500); // timeDelay
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    float dx2 = (a.acceleration.x - 0.44)*timeDelay*timeDelay/1000000 + 0.5*(a.acceleration.x - 0.44)*timeDelay*timeDelay/1000000;
    float dy2 = (a.acceleration.y + 0.66)*timeDelay*timeDelay/1000000 + 0.5*(a.acceleration.y + 0.66)*timeDelay*timeDelay/1000000;
    float d = sqrt(pow(dx2, 2) + pow(dy2, 2));
    //Serial.println(dx1);
    //Serial.println(dy1);
    /*
    Serial.print("Acceleration X: ");
    Serial.print(a.acceleration.x);
    Serial.print(", Y: ");
    Serial.print(a.acceleration.y);
    Serial.print(", Z: ");
    Serial.print(a.acceleration.z);
    Serial.println("distance x:");
    Serial.println(dx2);
    Serial.println("distance y:");
    Serial.println(dy2);
    Serial.println("total distance:");
    Serial.println(d);
    */
    while (error == 1) {
      if (d < 0.2) {
        error = 1;
        timeDelay = 50; // continue running function for another 50ms
        delay(timeDelay);
      } else {
        error = 0;
      }
      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);
      dx2 = dx2 + ((a.acceleration.x)*timeDelay*timeDelay/1000000 + 0.5*(a.acceleration.x)*timeDelay*timeDelay/1000000);
      dy2 = dy2 + ((a.acceleration.y)*timeDelay*timeDelay/1000000 + 0.5*(a.acceleration.y)*timeDelay*timeDelay/1000000); // Add distance to prior distance traveled
      d = sqrt(pow(dx2, 2) + pow(dy2, 2));
      /*
      Serial.println("total distance:");
      Serial.println(d);
      */
    }
    //Serial.write("done");
    return 0;
  }
  Serial.flush();
}

int driveLeft(int speed, float deg) {
  float percentAngle = deg/100;
  int timeDelay = percentAngle * 420; // 290 is the time for a 90 degree turn to occur if pulse is 580
  int pulse = 130;
  if (pulse < 130) {
    pulse = 130;
  }
  int error = 1;
  digitalWrite(RightDirection, HIGH);
  digitalWrite(LeftDirection, LOW);
  analogWrite(Right, 255);
  analogWrite(Left, 255);
  delay(500); // timeDelay
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  float testedAngleEnd = sqrt(pow(g.gyro.x + 0.06, 2) + pow(g.gyro.y - 0.01, 2)) * timeDelay / 1000; // Math for angles might be wrong, way to low for #s
  /*
  Serial.print("Rotation X: ");
  Serial.print(g.gyro.x);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y);
  Serial.print("testedAngleEnd");
  Serial.println(testedAngleEnd);
  */
  while (error == 1) {
    if (testedAngleEnd < 0.03) {
      error = 0;
      //error = 1;
      //timeDelay = 50; // continue running function for 50ms to see if angle is fixed
      //delay(timeDelay);
    } else {
      error = 0;
    }
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    testedAngleEnd = testedAngleEnd + (sqrt(pow(g.gyro.x + 0.06, 2) + pow(g.gyro.y - 0.01, 2)) * timeDelay / 1000);
    /*
    Serial.print("testedAngleEnd CHECK");
    Serial.println(testedAngleEnd);
    */
  }
  //Serial.write("done");
  return 0;
  Serial.flush();
}

int driveRight(int speed, float deg) { // 100% means 90 degree turn
  float percentAngle = deg/100;
  int timeDelay = percentAngle * 420; // 290 is the time for a 90 degree turn to occur if pulse is 130, 180 degrees for 590 or 100% deg
  int pulse = 130;
  if (pulse < 130) {
    pulse = 130;
  }
  int error = 1;
  digitalWrite(RightDirection, LOW);
  digitalWrite(LeftDirection, HIGH);
  analogWrite(Right, 255); // pulse would be duty cycle in analogWrite here
  analogWrite(Left, 255);
  delay(500); // timeDelay
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  float testedAngleEnd = sqrt(pow(g.gyro.x + 0.06, 2) + pow(g.gyro.y - 0.01, 2)) * timeDelay / 1000;
  /*
  Serial.print("Rotation X: ");
  Serial.print(g.gyro.x);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y);
  Serial.print("testedAngleEnd");
  Serial.println(testedAngleEnd);
  */
  while (error == 1) {
    if (testedAngleEnd < 0.03) {
      error = 0;
      //error = 1;
      //timeDelay = 50; // continue running function for 50ms to see if angle is fixed
      //delay(timeDelay);
    } else {
      error = 0;
    }
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    testedAngleEnd = testedAngleEnd + (sqrt(pow(g.gyro.x + 0.06, 2) + pow(g.gyro.y - 0.01, 2)) * timeDelay / 1000);
    /*
    Serial.print("testedAngleEnd CHECK");
    Serial.println(testedAngleEnd);
    */
  }
  //Serial.write("done");
  return 0;
  Serial.flush();
}

int stopMoving() {
   digitalWrite(RightDirection, LOW);
   digitalWrite(LeftDirection, LOW);
   analogWrite(Right, 0);
   analogWrite(Left, 0);
   delay(1);
   //Serial.write("done");
   Serial.flush();
}

void delay_us_10us(int ms) {
  while(0 < ms) {
    delay(10); // allows robot to execute instruction for approximately one second
    ms -= 1;
  }
}
