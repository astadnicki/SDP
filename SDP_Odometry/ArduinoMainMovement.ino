#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

int Right = 5; // 5 drives the right motor when going forward
int RightDirection = 10; // controls the direction of the right motor
int Left = 6; // was 3 // 6 drives the left motor when going forward
int LeftDirection = 11; // controls the direction of the left motor

#define S0 3 // was 4
#define S1 2
#define S2 4
#define S3 7
#define sensorOut 8

// Stores frequency read by the photodiodes
int redFrequency = 0;
int greenFrequency = 0;
int blueFrequency = 0;

void setup(void) {

  pinMode(Left, OUTPUT);  // Left motor
  pinMode(Right, OUTPUT);  // Right motor
  pinMode(LeftDirection, OUTPUT); // Left motor Direction
  pinMode(RightDirection, OUTPUT); // Right motor Direction

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

  //Simple Test Sequence
  //driveForward(99);
  //delay(500);
  //stopMoving();
  /*
  delay(1000);
  driveRight(99,50);  // 50% of pi (90 degree) angle
  delay(500);
  stopMoving();
  delay(1000);
  driveLeft(99,50);  // 50% of pi (90 degree) angle
  delay(500);
  stopMoving();
  */
  
  delay(100);
}

int counter = 0;

void loop() {

  Serial.flush();

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

  //delay(10);

  //float testedAngleEnd = 0;

      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);
      //testedAngleEnd = sqrt(pow(g.gyro.x + 0.07, 2) + pow(g.gyro.y - 0.01, 2)) * 1 / 1000; // 1 is for time delay
      // if needed: https://robotics.stackexchange.com/questions/21988/determining-the-turn-of-robot-using-gyroscope-and-accelerometer-data

    /*Serial.print("Rotation X: ");
    Serial.print(g.gyro.x);
    Serial.print(", Y: ");
    Serial.print(g.gyro.y);
    Serial.print(", Z: ");
    Serial.print(g.gyro.z);
    Serial.println("");*/
/*
    Serial.print("Counter");
    Serial.print(counter);
    Serial.println("");*/
    
  
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    if (data.substring(0,1) == "A") {
      driveForward(data.substring(3,5).toInt()); // initial forward command
      //Serial.print("AA 99 00");
    } else if (data.substring(0,1) == "B") {
      driveLeft(99, 50);
      //Serial.print("BB 99 50");
    } else if (data.substring(0,1) == "C") {
      driveRight(99, 50);
      //Serial.print("CC 99 50");
    } else if (data.substring(0,1) == "D") {
      //stopMoving();
      analogWrite(Right, 0);
      analogWrite(Left, 0);
      //Serial.print("DD 00 00");
    }
    //stopMoving();
    //Serial.println("Ack");
  }







/*
  
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

  // Get new sensor events with the readings 
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Print out the values 
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
  
*/









  
}

int driveForward(int speed){  // 23 inches moved for 350ms (IMU says on average 0.7)
  //Serial.println("GOING FORWARD");
  //THINGS TO DO
  // 50-99 speed increase/decrease, 0-49 speed increase/decrease
  // slow start after stopMoving command
  int pulse = speed;
  int error = 1;
  if (speed >= 50) {
    pulse = ((speed+100)/200) * 255;
    if (pulse < 130) {
      pulse = 130;
    }
    // HIGH all wheels forward
    digitalWrite(LeftDirection, LOW);
    digitalWrite(RightDirection, LOW);
    //analogWrite(Right, 220); // was 220
    analogWrite(Right, 243); // 2443
    analogWrite(Left, 255); // 255
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    
    if (g.gyro.z + 0.02 < -0.2) {  // negative z means right
      //Serial.println("TURNED SLIGHTLY RIGHT");
      // briefly drive slightly left to correct
      analogWrite(Right, 255);
      analogWrite(Left, 255 - (((g.gyro.z+0.02)*0.1)*5)); // pulse // g.gyro.z+0.02)*0.1 is the radians travelled from destination
    } else if (g.gyro.z + 0.02 > 0.2) { // positive z means left
      //Serial.println("TURNED SLIGHTLY LEFT");
      // briefly drive slightly right to correct
      analogWrite(Right, 255 - (((g.gyro.z+0.02)*0.1)*5)); // pulse // *10 is to account for strength of change in motor speed
      analogWrite(Left, 255); // pulse // g.gyro.z+0.02)*0.1 is the radians travelled from destination
    }
    
    float d = 5; // random number lol (to get past error for debugging) (eventually from encoder)
    
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
    
    while (error == 1) {
      if (d < 0.2) {
        error = 0;
        //error = 1;
        //timeDelay = 50; // continue running function for another 50ms
        //delay(timeDelay);
      } else {
        error = 0;
      }
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
    analogWrite(Left, 255); // pulse
    analogWrite(Right, 255); // pulse
    float d = 5; // random number lol (to get past error for debugging)
    
    while (error == 1) {
      if (d < 0.2) {
        error = 1;
        //timeDelay = 50; // continue running function for another 50ms
        //delay(timeDelay);
      } else {
        error = 0;
      }
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
  digitalWrite(RightDirection, LOW);
  digitalWrite(LeftDirection, HIGH);
  analogWrite(Right, 255);
  analogWrite(Left, 255);
  /*
  while (error == 1) {
    if (testedAngleEnd < 0.03) {
      error = 0;
      //error = 1;
      //timeDelay = 50; // continue running function for 50ms to see if angle is fixed
      //delay(timeDelay);
    } else {
      error = 0;
    }
    
  }*/
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
  digitalWrite(RightDirection, HIGH);
  digitalWrite(LeftDirection, LOW);
  analogWrite(Right, 255); // pulse would be duty cycle in analogWrite here
  analogWrite(Left, 255);
  /*
  while (error == 1) {
    if (testedAngleEnd < 0.03) {
      error = 0;
      //error = 1;
      //timeDelay = 50; // continue running function for 50ms to see if angle is fixed
      //delay(timeDelay);
    } else {
      error = 0;
    }
    
  }*/
  //Serial.write("done");
  return 0;
  Serial.flush();
}

int stopMoving() {
   digitalWrite(RightDirection, LOW);
   digitalWrite(LeftDirection, LOW);
   analogWrite(Right, 0);
   analogWrite(Left, 0);
   //delay(1);
   //Serial.write("done");
   Serial.flush();
}

void delay_us_10us(int ms) {
  while(0 < ms) {
    delay(10); // allows robot to execute instruction for approximately one second
    ms -= 1;
  }
}
