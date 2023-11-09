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

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
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
  }

  Serial.println("");
  delay(100);
}

void loop() {

    /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  /* Print out the values */
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

  //Serial.print("Temperature: ");
  //Serial.print(temp.temperature);
  //Serial.println(" degC");

  // Serial read takes in data from ROS as a string
  Serial.read(); // comes in from USB conenction to arduino

  Serial.println("");
  //delay(500);

  // 50-99: Positive speed (50 is 0 or just weak, 99 is highest)
  // 0-49: Negative speed (0 is 0 or just week, 49 is highest)
  // Commands: A is forward, B is right, C is left, D is nothing (empty) can drop the rest of the characters

  // Characters (space in between 1st, 2nd, 3rd)
  // 1st: Command (1 character), 2nd: Speed (2 characters), 3rd: Angle (2 characters)

  // First float (32 bits, 4 bytes), Serial read is one byte
  // command angle speed
  /*
  for (int i=0; i <= 6; i++) {
    if (i != 1 || i != ) {
      serialInput[i] = Serial.read();
    }
  }*/

  String serialInput = Serial.readString();

  /*
  if (serialInput.substring(0,1) == 'A') {
    driveForward(serialInput.substring(2,3).toInt());
  } else if (serialInput.substring(0,1) == 'B') {
    driveLeft(serialInput.substring(2,3).toInt(), serialInput.substring(5,6).toInt());
  } else if (serialInput.substring(0,1) == 'C') {
    driveRight(serialInput.substring(2,3).toInt(), serialInput.substring(5,6).toInt());
  }*/

  //TESTING
  driveForward(70, a.acceleration.x, a.acceleration.y);
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
    while (error == 1) {
      // HIGH all wheels forward
      digitalWrite(2, HIGH);
      digitalWrite(3, HIGH);
      digitalWrite(4, HIGH);
      digitalWrite(5, HIGH);
      delay_us_10us(pulse);
      // Set pins low to mimic pulse width modulation
      digitalWrite(2, LOW);
      digitalWrite(3, LOW);
      digitalWrite(4, LOW);
      digitalWrite(5, LOW);
      delay_us_10us(100 - pulse);
      Serial.println(xacc);
      Serial.println(yacc);
      float dx2 = (xacc-0.4)*1*1 + 0.5*(xacc-0.4)*1*1;
      float dy2 = (yacc+2.5)*1*1 + 0.5*(yacc+2.5)*1*1;
      float d = sqrt(pow(dx2 - dx1, 2) + pow(dy2 - dy1, 2));
      Serial.println("distance:");
      Serial.println(d);
      if (d < 0.2) { //On average it goes about 0.2 meters at least.
        //error = 1;
        error = 0;
      } else {
        error = 0;
      }
    }
  }
  if (speed < 50) {
    int pulse;
    pulse = speed;
    while (error == 1) {
      // HIGH all wheels reverse
      digitalWrite(8, HIGH);
      digitalWrite(9, HIGH);
      digitalWrite(10, HIGH);
      digitalWrite(11, HIGH);
      delay_us_10us(pulse);
      // Set pins low to mimic pulse width modulation
      digitalWrite(8, LOW);
      digitalWrite(9, LOW);
      digitalWrite(10, LOW);
      digitalWrite(11, LOW);
      delay_us_10us(100 - pulse);
      Serial.println(xacc);
      Serial.println(yacc);
      //It tests this way too fast
      float dx2 = (xacc-0.4)*1*1 + 0.5*(xacc-0.4)*1*1;
      float dy2 = (yacc+2.5)*1*1 + 0.5*(yacc+2.5)*1*1;
      float d = sqrt(pow(dx2 - dx1, 2) + pow(dy2 - dy1, 2));
      Serial.println("distance:");
      Serial.println(d);
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
int driveLeft(float deg, int speed, float xrot, float yrot) {
  float angleRadians = deg/100 * 3.14; // angle sent as a percentage of pi
  float percentTurn = (3.14 - angleRadians)/3.14;   // how far to turn
  int pulse = speed*percentTurn;  // uses angle to reduce turn time
  float testedAngleStart = sqrt(pow(xrot, 2) + pow(yrot, 2)) * pulse;
  int error = 0;
  while(error == 1) {
      // HIGH right wheels forward
      digitalWrite(4, HIGH);
      digitalWrite(5, HIGH);
      // HIGH left wheels reverse
      digitalWrite(8, HIGH);
      digitalWrite(9, HIGH);
      delay_us_10us(pulse);
      // Set pins low to mimic pulse width modulation
      digitalWrite(4, LOW);
      digitalWrite(5, LOW);
      digitalWrite(8, LOW);
      digitalWrite(9, LOW);
      delay_us_10us(100 - pulse);
      //i++;
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

int driveRight(int deg, int speed, float xrot, float yrot) {
  float angleRadians = deg/100 * 3.14; // angle sent as a percentage of pi
  float percentTurn = (3.14 - angleRadians)/3.14;   // how far to turn
  int pulse = speed*percentTurn;  // uses angle to reduce turn time
  float testedAngleStart = sqrt(pow(xrot, 2) + pow(yrot, 2)) * pulse;
  int error = 1;
  while(error == 1) {
      // HIGH right wheels forward
      digitalWrite(2, HIGH);
      digitalWrite(3, HIGH);
      // HIGH left wheels reverse
      digitalWrite(10, HIGH);
      digitalWrite(11, HIGH);
      delay_us_10us(pulse);
      // Set pins low to mimic pulse width modulation
      digitalWrite(2, LOW);
      digitalWrite(3, LOW);
      digitalWrite(10, LOW);
      digitalWrite(11, LOW);
      delay_us_10us(100 - pulse);
      //i++;
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
