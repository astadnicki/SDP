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

  //Serial.begin(115200);
  Serial.begin(9600);
  //Serial.begin(115200);
  while (!Serial)   // 1/10 of a meter for set distance to go forward
    delay(0.1); // will pause Zero, Leonardo, etc until serial console opens // INITIALLY DELAY 10

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
  //Serial.print("Accelerometer range set to: ");
  /*
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }*/

  //Serial.println("");

  //CORRECT LOOP SEQUENCE
  driveForward(99);
  stopMoving();
  driveRight(10,50);
  stopMoving();
  driveForward(99);
  stopMoving();
  driveRight(10,50);
  stopMoving();
  driveForward(99);
  stopMoving();
  driveRight(10,50);
  stopMoving();
  driveForward(99);
  stopMoving();
  driveRight(10,50);
  stopMoving();

  //driveRight(10,50);
  //stopMoving();
  
  //driveRight(99, 50); // 50% of pi (90 degree) angle
  
  delay(100);
}

void loop() {

  Serial.println("");
  Serial.println("");

  // 50-99: Positive speed (50 is 0 or just weak, 99 is highest)
  // 0-49: Negative speed (0 is 0 or just week, 49 is highest)
  // Commands: A is forward, B is right, C is left, D is nothing (empty) can drop the rest of the characters
  // Characters (space in between 1st, 2nd, 3rd)
  // 1st: Command (1 character), 2nd: Speed (2 characters), 3rd: Angle (2 characters)

  delay(10);
  
  String serialInput = Serial.readString();
  
  //driveForward(99); // purely based on time delay how long it runs for!
  //stopMoving();
  //delay(10000);

  //driveForward(99);
  //delay(1500);
  //driveRight(99, 50); // 50% of pi (90 degree) angle
  //delay(2000);
  //driveForward(99); // purely based on time delay how long it runs for!
  //delay(2000);
  //driveRight(99, 50);
  //delay(2000);
  //stopMoving();
  
  /*
  if (serialInput.substring(0,1) == "A") {
    driveForward(serialInput.substring(2,4).toInt());
  } else if (serialInput.substring(0,1) == "B") {
    driveLeft(serialInput.substring(2,4).toInt(), serialInput.substring(5,7).toInt());
  } else if (serialInput.substring(0,1) == "C") {
    driveRight(serialInput.substring(2,4).toInt(), serialInput.substring(5,7).toInt());
  } else if (serialInput.substring(0,1) == "D") {
    stopMoving();
  }*/
  
}

int driveForward(int speed){  // IMU says 5 distance for 1 second (full instruction time), we travel exactly 37 inches for each forward with 1 second. 14 inches with 350ms.
  int pulse = speed;
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  float dx1 = (a.acceleration.x-0.4)*0.35*0.35 + 0.5*(a.acceleration.x-0.4)*0.35*0.35;
  float dy1 = (a.acceleration.y+2.5)*0.35*0.35 + 0.5*(a.acceleration.y+2.5)*0.35*0.35;
  int error = 1;
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
    analogWrite(4, pulse);//digitalWrite(4, HIGH);
    analogWrite(5, pulse);
    delay(350);
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    float dx2 = (a.acceleration.x-0.4)*0.35*0.35 + 0.5*(a.acceleration.x-0.4)*0.35*0.35;
    float dy2 = (a.acceleration.y+2.5)*0.35*0.35 + 0.5*(a.acceleration.y+2.5)*0.35*0.35; // 1 second movement for robot is 1 meter in distance, hence we use time = 1
    float d = sqrt(pow(dx2 - dx1, 2) + pow(dy2 - dy1, 2));
    Serial.println(dx2);
    Serial.println(dx1);
    Serial.println("distance:");
    Serial.println(dy2);
    Serial.println(dy1);
    Serial.println(d);
    while (error == 1) {
      if (d < 0.2) { //On average it goes about 1 unit of distance at least at 99 speed. // TYPICALLY AROUND 0.7
        //error = 1;
        error = 1;
        delay(100); // continue running function
      } else {
        error = 0;
      }
      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);
      float dx2 = (a.acceleration.x-0.4)*0.35*0.35 + 0.5*(a.acceleration.x-0.4)*0.35*0.35;
      float dy2 = (a.acceleration.y+2.5)*0.35*0.35 + 0.5*(a.acceleration.y+2.5)*0.35*0.35; // 1 second movement for robot is 1 meter in distance, hence we use time = 1
      float d = sqrt(pow(dx2 - dx1, 2) + pow(dy2 - dy1, 2));
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
    delay(350);
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    float dx2 = (a.acceleration.x-0.4)*1*1 + 0.5*(a.acceleration.x-0.4)*1*1;
    float dy2 = (a.acceleration.y+2.5)*1*1 + 0.5*(a.acceleration.y+2.5)*1*1;
    float d = sqrt(pow(dx2 - dx1, 2) + pow(dy2 - dy1, 2));
    Serial.println(dx2);
    Serial.println(dx1);
    Serial.println("distance:");
    Serial.println(dy2);
    Serial.println(dy1);
    Serial.println(d);
    while (error == 1) {
      if (d < 3) { //On average it goes about 0.2 meters at least. TYPICALLY AROUND 3.77
        //error = 1;
        error = 1;
        delay(100); // continue running function
      } else {
        error = 0;
      }
      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);
      float dx2 = (a.acceleration.x-0.4)*0.35*0.35 + 0.5*(a.acceleration.x-0.4)*0.35*0.35;
      float dy2 = (a.acceleration.y+2.5)*0.35*0.35 + 0.5*(a.acceleration.y+2.5)*0.35*0.35; // 1 second movement for robot is 1 meter in distance, hence we use time = 1
      float d = sqrt(pow(dx2 - dx1, 2) + pow(dy2 - dy1, 2));
    }
    

    return 0;
  }
}

//float angle
int driveLeft(int speed, float deg) {
  float percentAngle = deg/100;
  int timeDelay = percentAngle * 730; // 365 is the time for a 90 degree turn to occur if pulse is 130
  int pulse = 130;
  if (pulse < 130) {
    pulse = 130;
  }
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  float testedAngleStart = sqrt(pow(g.gyro.x, 2) + pow(g.gyro.y, 2)) * 1;
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
  while (error == 1) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    float testedAngleEnd = sqrt(pow(g.gyro.x, 2) + pow(g.gyro.y, 2)) * 1; // Math for angles might be wrong, way to low for #s
    float testedAngle = testedAngleStart - testedAngleEnd;
    Serial.println(testedAngleStart);
    Serial.println(testedAngleEnd);
    Serial.println(testedAngle);
    if (testedAngle < -0.1) { // TYPICALLY AROUND -0.4
      //error = 1;
      error = 1;
      delay(100); // continue running function
    } else {
      error = 0;
    }
  }
  
  return 0;
}

int driveRight(int speed, float deg) {
  float percentAngle = deg/100;
  int timeDelay = percentAngle * 730; // 365 is the time for a 90 degree turn to occur if pulse is 130
  int pulse = 130;
  if (pulse < 130) {
    pulse = 130;
  }
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  float testedAngleStart = sqrt(pow(g.gyro.x, 2) + pow(g.gyro.y, 2)) * 1;
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
  while (error == 1) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    float testedAngleEnd = sqrt(pow(g.gyro.x, 2) + pow(g.gyro.y, 2)) * 1;
    float testedAngle = testedAngleStart - testedAngleEnd;
    Serial.println(testedAngleStart);
    Serial.println(testedAngleEnd);
    Serial.println(testedAngle);
    if (testedAngle > -0.1) { // TYPICALLY AROUND -0.4
      //error = 1;
      error = 1;
      delay(100); // continue running function
    } else {
      error = 0;
    }
  }
  
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
}

void delay_us_10us(int ms) {
  while(0 < ms) {
    delay(10); // allows robot to execute instruction for approximately one second
    ms -= 1;
  }
}
