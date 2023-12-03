// Basic demo for accelerometer readings from Adafruit MPU6050

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

void setup(void) {

  // Driving initialization
  // Forward driving
  pinMode(2, OUTPUT);  // Rear Left
  pinMode(3, OUTPUT);  // Front Left
  pinMode(4, OUTPUT);  // Rear Right
  pinMode(5, OUTPUT);  // Front Right
  // Reverse driving
  pinMode(8, OUTPUT);  // Rear Left
  pinMode(9, OUTPUT);  // Front Left
  pinMode(10, OUTPUT); // Rear Right
  pinMode(11, OUTPUT); // Front Right

  // 0: 

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

  //Serial.println("");
  //Serial.println("");

  // 50-99: Positive speed (50 is 0 or just weak, 99 is highest)
  // 0-49: Negative speed (0 is 0 or just week, 49 is highest)
  // Commands: A is forward, B is right, C is left, D is nothing (empty) can drop the rest of the characters
  // Characters (space in between 1st, 2nd, 3rd)
  // 1st: Command (1 character), 2nd: Speed (2 characters), 3rd: Angle (2 characters)

  delay(10);
  
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    if (data.substring(0,1) == "A") {
      driveForward(99);
    } else if (data.substring(0,1) == "B") {
      driveLeft(99, 50);
    } else if (data.substring(0,1) == "C") {
      driveRight(99, 00);
    } else if (data.substring(0,1) == "D") {
    stopMoving();
    }
    stopMoving();
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
    analogWrite(8, 0);
    analogWrite(9, 0);
    analogWrite(10, 0);
    analogWrite(11, 0);
    analogWrite(2, pulse);
    analogWrite(3, pulse);
    analogWrite(4, pulse);
    analogWrite(5, pulse);
    delay(timeDelay);
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    float dx2 = (a.acceleration.x - 0.44)*timeDelay*timeDelay/1000000 + 0.5*(a.acceleration.x - 0.44)*timeDelay*timeDelay/1000000;
    float dy2 = (a.acceleration.y + 0.66)*timeDelay*timeDelay/1000000 + 0.5*(a.acceleration.y + 0.66)*timeDelay*timeDelay/1000000;
    float d = sqrt(pow(dx2, 2) + pow(dy2, 2));
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
      dx2 = dx2 + ((a.acceleration.x - 0.44)*timeDelay*timeDelay/1000000 + 0.5*(a.acceleration.x - 0.44)*timeDelay*timeDelay/1000000);
      dy2 = dy2 + ((a.acceleration.y + 0.66)*timeDelay*timeDelay/1000000 + 0.5*(a.acceleration.y + 0.66)*timeDelay*timeDelay/1000000);
      d = sqrt(pow(dx2, 2) + pow(dy2, 2));
      Serial.println("total distance:");
      Serial.println(d);
    }
  }
  if (speed < 50) {
    pulse = (((speed*2)+100)/200) * 255;
    if (pulse < 130) {
      pulse = 130;
    }
    // HIGH all wheels reverse
    analogWrite(2, 0);
    analogWrite(3, 0);
    analogWrite(4, 0);
    analogWrite(5, 0);
    analogWrite(8, pulse);
    analogWrite(9, pulse);
    analogWrite(10, pulse);
    analogWrite(11, pulse);
    delay(timeDelay);
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    float dx2 = (a.acceleration.x - 0.44)*timeDelay*timeDelay/1000000 + 0.5*(a.acceleration.x - 0.44)*timeDelay*timeDelay/1000000;
    float dy2 = (a.acceleration.y + 0.66)*timeDelay*timeDelay/1000000 + 0.5*(a.acceleration.y + 0.66)*timeDelay*timeDelay/1000000;
    float d = sqrt(pow(dx2, 2) + pow(dy2, 2));
    //Serial.println(dx1);
    //Serial.println(dy1);
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
      Serial.println("total distance:");
      Serial.println(d);
    }
    
    Serial.write("done");
    return 0;
  }
}

//float angle
int driveLeft(int speed, float deg) {
  float percentAngle = deg/100;
  int timeDelay = percentAngle * 500; // 290 is the time for a 90 degree turn to occur if pulse is 580
  int pulse = 130;
  if (pulse < 130) {
    pulse = 130;
  }
  int error = 1;
  analogWrite(2, 0);
  analogWrite(3, 0);
  analogWrite(10, 0);
  analogWrite(11, 0);
  analogWrite(4, pulse);
  analogWrite(5, pulse);
  analogWrite(8, pulse);
  analogWrite(9, pulse);
  delay(timeDelay);
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  float testedAngleEnd = sqrt(pow(g.gyro.x + 0.06, 2) + pow(g.gyro.y - 0.01, 2)) * timeDelay / 1000; // Math for angles might be wrong, way to low for #s
  Serial.print("Rotation X: ");
  Serial.print(g.gyro.x);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y);
  Serial.print("testedAngleEnd");
  Serial.println(testedAngleEnd);
  while (error == 1) {
    if (testedAngleEnd < 0.03) {
      error = 1;
      timeDelay = 50; // continue running function for 50ms to see if angle is fixed
      delay(timeDelay);
    } else {
      error = 0;
    }
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    testedAngleEnd = testedAngleEnd + (sqrt(pow(g.gyro.x + 0.06, 2) + pow(g.gyro.y - 0.01, 2)) * timeDelay / 1000);
    Serial.print("testedAngleEnd CHECK");
    Serial.println(testedAngleEnd);
  }
  Serial.write("done");
  return 0;
}

int driveRight(int speed, float deg) { // 100% means 90 degree turn
  float percentAngle = deg/100;
  int timeDelay = percentAngle * 500; // 290 is the time for a 90 degree turn to occur if pulse is 130, 180 degrees for 590 or 100% deg
  int pulse = 130;
  if (pulse < 130) {
    pulse = 130;
  }
  int error = 1;
  analogWrite(4, 0);
  analogWrite(5, 0);
  analogWrite(8, 0);
  analogWrite(9, 0);
  analogWrite(2, pulse);  // pulse would be duty cycle in analogWrite here
  analogWrite(3, pulse);
  analogWrite(10, pulse);
  analogWrite(11, pulse);
  delay(timeDelay);
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  float testedAngleEnd = sqrt(pow(g.gyro.x + 0.06, 2) + pow(g.gyro.y - 0.01, 2)) * timeDelay / 1000;
  Serial.print("Rotation X: ");
  Serial.print(g.gyro.x);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y);
  Serial.print("testedAngleEnd");
  Serial.println(testedAngleEnd);
  while (error == 1) {
    if (testedAngleEnd < 0.03) {
      error = 1;
      timeDelay = 50; // continue running function for 50ms to see if angle is fixed
      delay(timeDelay);
    } else {
      error = 0;
    }
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    testedAngleEnd = testedAngleEnd + (sqrt(pow(g.gyro.x + 0.06, 2) + pow(g.gyro.y - 0.01, 2)) * timeDelay / 1000);
    Serial.print("testedAngleEnd CHECK");
    Serial.println(testedAngleEnd);
  }
  Serial.write("done");
  return 0;
}

int stopMoving() {
   analogWrite(2, 0);
   analogWrite(3, 0);
   analogWrite(4, 0);
   analogWrite(5, 0);
   analogWrite(8, 0);
   analogWrite(9, 0);
   analogWrite(10, 0);
   analogWrite(11, 0);
   delay(500);
   Serial.write("done");
}

void delay_us_10us(int ms) {
  while(0 < ms) {
    delay(10); // allows robot to execute instruction for approximately one second
    ms -= 1;
  }
}
