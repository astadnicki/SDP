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
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  //Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    //Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
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
  delay(100);
}

void loop() {

    /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  /* Print out the values */
  /*
  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);  //Serial.print("Acceleration X: ");
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);  //Serial.print(", Y: ");
  //Serial.print(", Z: ");
  //Serial.print(a.acceleration.z);  //Serial.print(", Z: ");
  Serial.println(" m/s^2");

  Serial.print("Rotation X: ");
  Serial.print(g.gyro.x);  //Serial.print("Rotation X: ");
  Serial.print(", Y: ");
  Serial.print(g.gyro.y);  //Serial.print(", Y: ");
  //Serial.print(", Z: ");
  //Serial.print(g.gyro.z);  //Serial.print(", Z: ");
  Serial.println(" rad/s");
  */

  //Serial.print("Temperature: ");
  //Serial.print(temp.temperature);
  //Serial.println(" degC");

  Serial.println("");
  //Serial.println("");
  //delay(500);

  // 50-99: Positive speed (50 is 0 or just weak, 99 is highest)
  // 0-49: Negative speed (0 is 0 or just week, 49 is highest)
  // Commands: A is forward, B is right, C is left, D is nothing (empty) can drop the rest of the characters
  // Characters (space in between 1st, 2nd, 3rd)
  // 1st: Command (1 character), 2nd: Speed (2 characters), 3rd: Angle (2 characters)

  delay(10);
  
  String serialInput = Serial.readString();
  //Serial.println(serialInput);
  //Serial.println(serialInput.substring(0,1));
  
  //driveForward(60, a.acceleration.x, a.acceleration.y);
  //delay(1000);
  //driveLeft(60, 5, g.gyro.x, g.gyro.y);
  //delay(1000);
  driveRight(60, 20, g.gyro.x, g.gyro.y);
  //if (serialInput.substring(0,1) == "A") {
  if (serialInput.substring(0,1) == "A") {
    driveForward(serialInput.substring(2,4).toInt(), a.acceleration.x, a.acceleration.y);
  //} else if (serialInput.substring(0,1) == "B") {
  } else if (serialInput.substring(0,1) == "B") {
    //Serial.println("TESTING");
    //Serial.println(serialInput.substring(2,4).toInt());
    //Serial.println(serialInput.substring(5,7).toInt());
    driveLeft(serialInput.substring(2,4).toInt(), serialInput.substring(5,7).toInt(), g.gyro.x, g.gyro.y);
  //} else if (serialInput.substring(0,1) == "C") {
  } else if (serialInput.substring(0,1) == "C") {
    driveRight(serialInput.substring(2,4).toInt(), serialInput.substring(5,7).toInt(), g.gyro.x, g.gyro.y);
  } else if (serialInput.substring(0,1) == "D") {
      analogWrite(2, 0);
      analogWrite(3, 0);
      analogWrite(4, 0);
      analogWrite(5, 0);
      analogWrite(8, 0);
      analogWrite(9, 0);
      analogWrite(10, 0);
      analogWrite(11, 0);
  }

  //TESTING
  //driveForward(70, a.acceleration.x, a.acceleration.y);
  //driveRight(3.14, 70, g.gyro.x, g.gyro.y);
  //driveLeft(3.14, 70, g.gyro.x, g.gyro.y);
  //Serial.println("DRIVE TEST");
  //delay(1000);
  
}

int driveForward(int speed, float xacc, float yacc){
  int pulse = speed;
  float dx1 = (xacc-0.4)*1*1 + 0.5*(xacc-0.4)*1*1;
  float dy1 = (yacc+2.5)*1*1 + 0.5*(yacc+2.5)*1*1;
  int error = 1;
  if (speed >= 50) {
    pulse = ((speed+100)/200) * 255;
    if (pulse < 130) {
      pulse = 130;
    }
    while (error == 1) {
      // HIGH all wheels forward
      analogWrite(8, 0);
      analogWrite(9, 0);
      analogWrite(10, 0);
      analogWrite(11, 0);
      analogWrite(2, pulse);
      analogWrite(3, pulse);
      analogWrite(4, pulse);//digitalWrite(4, HIGH);
      analogWrite(5, pulse);
      //delay_us_10us(100 - pulse);
      //Serial.println(xacc);
      //Serial.println(yacc);
      float dx2 = (xacc-0.4)*1*1 + 0.5*(xacc-0.4)*1*1;
      float dy2 = (yacc+2.5)*1*1 + 0.5*(yacc+2.5)*1*1;
      float d = sqrt(pow(dx2 - dx1, 2) + pow(dy2 - dy1, 2));
      //Serial.println("distance:");
      //Serial.println(d);
      if (d < 0.2) { //On average it goes about 0.2 meters at least.
        //error = 1;
        error = 0;
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
    while (error == 1) {
      // HIGH all wheels reverse
      analogWrite(2, 0);
      analogWrite(3, 0);
      analogWrite(4, 0);
      analogWrite(5, 0);
      analogWrite(8, pulse);
      analogWrite(9, pulse);
      analogWrite(10, pulse);
      analogWrite(11, pulse);
      //Serial.println(xacc);
      //Serial.println(yacc);
      //It tests this way too fast
      float dx2 = (xacc-0.4)*1*1 + 0.5*(xacc-0.4)*1*1;
      float dy2 = (yacc+2.5)*1*1 + 0.5*(yacc+2.5)*1*1;
      float d = sqrt(pow(dx2 - dx1, 2) + pow(dy2 - dy1, 2));
      //Serial.println("distance:");
      //Serial.println(d);
      if (d < 0.2) { //On average it goes about 0.2 meters at least.
        //error = 1;
        error = 0;
      } else {
        error = 0;
      }
    }

    return 0;
  }
}

//float angle
int driveLeft(int speed, float deg, float xrot, float yrot) {
  float angleRadians = deg/100 * 3.14; // angle sent as a percentage of pi
  //float percentTurn = (3.14 - angleRadians)/3.14;   // how far to turn
  //int pulse = speed*angleRadians;  // uses angle to reduce turn time
  int pulse = ((speed+100)/200) * 255 * angleRadians;
  if (pulse < 130) {
    pulse = 130;
  }
  float testedAngleStart = sqrt(pow(xrot, 2) + pow(yrot, 2)) * pulse;
  int error = 1;
  while(error == 1) {
    // HIGH right wheels forward
    analogWrite(2, 0);
    analogWrite(3, 0);
    analogWrite(10, 0);
    analogWrite(11, 0);
    analogWrite(4, pulse);
    analogWrite(5, pulse);
    // HIGH left wheels reverse
    analogWrite(8, pulse);
    analogWrite(9, pulse);
    float testedAngleEnd = sqrt(pow(xrot, 2) + pow(yrot, 2)) * pulse;
    float testedAngle = testedAngleStart - testedAngleEnd;
    if (testedAngle < angleRadians) {
      //error = 1;
      error = 0;
    } else {
      error = 0;
    }
  }
  return 0;
}

int driveRight(int speed, float deg, float xrot, float yrot) {
  float angleRadians = deg/100 * 3.14; // angle sent as a percentage of pi
  //float percentTurn = (3.14 - angleRadians)/3.14;   // how far to turn
  //int pulse = speed*angleRadians;  // uses angle to reduce turn time
  int pulse = ((speed+100)/200) * 255 * angleRadians;
  if (pulse < 130) {
    pulse = 130;
  }
  float testedAngleStart = sqrt(pow(xrot, 2) + pow(yrot, 2)) * pulse;
  int error = 1;
  while(error == 1) {
      // HIGH right wheels forward
      analogWrite(4, 0);
      analogWrite(5, 0);
      analogWrite(8, 0);
      analogWrite(9, 0);
      analogWrite(2, pulse);
      analogWrite(3, pulse);
      // HIGH left wheels reverse
      analogWrite(10, pulse);
      analogWrite(11, pulse);
      float testedAngleEnd = sqrt(pow(xrot, 2) + pow(yrot, 2)) * 1;
      float testedAngle = testedAngleStart - testedAngleEnd;
      if (testedAngle < angleRadians) {
        //error = 1;
        error = 0;
      } else {
        error = 0;
      }
  }
  return 0;
}

void delay_us_10us(int ms) {
  while(0 < ms) {
    delay(10); // allows robot to execute instruction for approximately one second
    ms -= 1;
  }
}
