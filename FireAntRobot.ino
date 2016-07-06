#include <ZumoMotors.h>
#include <QTRSensors.h>
#include <ZumoReflectanceSensorArray.h>
#include <L3G.h>
#include <Wire.h>
#include <Pushbutton.h>

#include <LSM303.h>
#include <ShirriffIRremote.h>
#include <Triangulation.h>
#include "Signal.h"

//#include <ZumoBuzzer.h>


/*
* ------------------------------------------------
* | PROPERTY OF ELIZABETH WANG                   |
* | @author: Elizabeth Wang                      |
* |                                              |
* | @version: 12/22/14 -                         |
* |                                              |
* | Description: Robot runs randomly until it    |
* |             reaches a black spot. Then, it   |
* |            finds the angle, triangulates,    |
* |             and sends the coordinates to     |
* |             the second robot                 |
* -----------------------------------------------
*/
#define rf_pin  0           //  Arduino RX pin to read RF input from Wixel pin P1_6 (TX pin)
#define l_pin  10          // pin to direct Left wheel servo
#define r_pin  11          // pin to direct Right wheel servo


#define ToRad(x) ((x) * 0.01745329252)  // *pi/180
#define ToDeg(x) ((x) * 57.2957795131)  // *180/pi
#define PI_FLOAT     3.14159265f
#define PIBY2_FLOAT  1.5707963f
#define GYRO_SCALE 0.07f
#define betaDef		0.08f
#define WIDTH 24
#define HEIGHT 36

//#define DEBUG
//#define SHOW_RESULT


#define compassXMax 216.0f
#define compassXMin -345.0f
#define compassYMax 210.0f
#define compassYMin -347.0f
#define compassZMax 249.0f
#define compassZMin -305.0f
#define inverseXRange (float)(2.0 / (compassXMax - compassXMin))
#define inverseYRange (float)(2.0 / (compassYMax - compassYMin))
#define inverseZRange (float)(2.0 / (compassZMax - compassZMin))


#define IR_SIGNAL_TYPE 2
#define KEY_1_HEX_VALUE 0x10
#define KEY_2_HEX_VALUE 0x810
#define KEY_3_HEX_VALUE 0x410
#define KEY_4_HEX_VALUE 0xC10

#define FAST true
#define SLOW false

// parameters for tuning
#define WHEEL_SPEED 100
#define ROTATION_TIME_STEP 20
#define TIME_TO_TRY_IN_MICRO_SECONDS  18000000
#define NUM_OF_RESETS_TO_TRY 3
#define RECV_PIN  3 // ir stuff

/*
// Best so far
#define WHEEL_SPEED 100
#define ROTATION_TIME_STEP 20
#define TIME_TO_TRY_IN_MICRO_SECONDS  18000000
*/

#define MOVE_SPEED 180
#define DELAY_TIME 50
#define SENSOR_SIZE 6

// this might need to be tuned for different lighting conditions, surfaces, etc.
#define QTR_THRESHOLD  1700 // microseconds

// these might need to be tuned for different motor types
#define TURN_SPEED        200
#define FORWARD_SPEED     200
#define REVERSE_DURATION  200 // ms
#define TURN_DURATION     300 // ms

#define BLUE_MIN 250
#define BLUE_MAX 400

// Assign Servos/Wixel I/O to Arduino  pins
#define rf_pin  0           //  Arduino RX pin to read RF input from Wixel pin P1_6 (TX pin)
#define l_pin  10          // pin to direct Left wheel servo
#define r_pin  11          // pin to direct Right wheel servo

L3G gyro;
LSM303 compass;

long timer;
float G_Dt;

float q0;
float q1;
float q2;
float q3;
float beta;

float magnitude;

float pitch, roll, yaw;

float offSetX, offSetY, offSetZ;

float floatMagX, floatMagY, floatMagZ;
float smoothAccX, smoothAccY, smoothAccZ;
float accToFilterX, accToFilterY, accToFilterZ;

int i;
long startingTime;
int resetCounter = 0;

IRrecv irrecv(RECV_PIN);
ZumoMotors motors;

//ZumoBuzzer buzzer;

unsigned int sensor_values[SENSOR_SIZE];

ZumoReflectanceSensorArray reflectanceSensors;

boolean blackSpotFound = false;

int angle1Counter = 0; // the angle between tower 1 and tower 2
int angle2Counter = 0; // the angle between tower 2 and tower 3
int angle3Counter = 0; // the angle between tower 3 and tower 4
int angle4Counter = 0; // the angle between tower 4 and tower 1

float angle1 = 0.0;
float angle2 = 0.0;
float angle3 = 0.0;
float angle4 = 0.0;

int timeToTry = 8;
int speedFactor = 1;

bool tower1Found = false;
bool tower2Found = false;
bool tower3Found = false;
bool tower4Found = false;

bool coordsCalculatable = true;
bool coordsCalculated = false;

int numOfFoundTowers = 0;

void setup()
{
  pinMode(rf_pin, INPUT);    // set Arduino RX pin for input from Wixel's TX pin
  pinMode(l_pin, OUTPUT);
  pinMode(r_pin, OUTPUT);

  delay(10);
  Serial.begin(115200); // *** NOTE: change this to the baud rate you set on your Wixel
  delay(10);

  // Serial.println("START setup");
  pinMode(rf_pin, INPUT);    // set Arduino RX pin for input from Wixel's TX pin
  pinMode(l_pin, OUTPUT);
  pinMode(r_pin, OUTPUT);

  Wire.begin(); // for gyroscope

  Pushbutton button(ZUMO_BUTTON); // pushbutton on pin 12
  button.waitForButton();

  reflectanceSensors.init();
  delay(1000);
  int i;
  for (i = 0; i < 80; i++)
  {
    if ((i > 10 && i <= 30) || (i > 50 && i <= 70))
      motors.setSpeeds(-200, 200);
    else
      motors.setSpeeds(200, -200);
    reflectanceSensors.calibrate();
    delay(20);
  }
  motors.setSpeeds(0, 0);

#ifdef DEBUG
  Serial.println("Start testing!");
#endif

  delay(1000);
  motors.setSpeeds(0, 0);


#ifdef DEBUG
  Serial.println("Keeping the device still and level during startup will yeild the best results");
#endif

  TWBR = ((F_CPU / 400000) - 16) / 2;//set the I2C speed to 400KHz
  IMUinit();
  timer = micros();

  irrecv.enableIRIn(); // Start the receiver

#ifdef DEBUG
  Serial.println("I'm trying to read");
#endif

  startingTime = micros();

  button.waitForButton();

}

void loop()
{
  // update gyroscope tracking
  if (micros() - timer >= 5000) {
    //this runs in 4ms on the MEGA 2560
    G_Dt = (micros() - timer) / 1000000.0;
    timer = micros();
    compass.read();
    floatMagX = ((float)compass.m.x - compassXMin) * inverseXRange - 1.0;
    floatMagY = ((float)compass.m.y - compassYMin) * inverseYRange - 1.0;
    floatMagZ = ((float)compass.m.z - compassZMin) * inverseZRange - 1.0;
    Smoothing((float*)&compass.a.x, &smoothAccX);
    Smoothing((float*)&compass.a.y, &smoothAccY);
    Smoothing((float*)&compass.a.z, &smoothAccZ);
    accToFilterX = smoothAccX;
    accToFilterY = smoothAccY;
    accToFilterZ = smoothAccZ;
    gyro.read();
    AHRSupdate(&G_Dt);
  }

  if (!blackSpotFound)
  {
    motors.setSpeeds (MOVE_SPEED, MOVE_SPEED); // goes straight
    delay (DELAY_TIME);

    stayAwayFromLine ();


    motors.setSpeeds ((int) random (-MOVE_SPEED, MOVE_SPEED), (int) random (-MOVE_SPEED, MOVE_SPEED)); // random turn
    //    motors.setSpeeds ((pow (-1, (int) random (0, 2))) * MOVE_SPEED, (pow (-1, (int) random (0, 2))) * MOVE_SPEED); // random turn
    delay ((int) (2 * DELAY_TIME * random (0, 100) / 100.0));
    stayAwayFromLine ();

    blackSpotFound = lookForBlackSpot (); // look for black spot

#ifdef DEBUG
    if (blackSpotFound) {
      logStr("I got the blackspot!!!!", "");
      logStr("blackSpotFound = ", blackSpotFound + "");
    }
    logStr("inside if", "");
#endif

    motors.setSpeeds (0, 0); // random turn
  }

  else
  {

    if ((!(tower1Found && tower2Found && tower3Found && tower4Found)) && (micros() - startingTime <= TIME_TO_TRY_IN_MICRO_SECONDS)) {
      motors.setSpeeds(WHEEL_SPEED / speedFactor, - (WHEEL_SPEED / speedFactor));
      delay(ROTATION_TIME_STEP);
      motors.setSpeeds(0, 0);
      findTowerSignalAngle();
    } else {
      if (!coordsCalculated && coordsCalculatable) {
        numOfFoundTowers = 0;
        if (tower1Found) {
          numOfFoundTowers ++;
        }
        if (tower2Found) {
          numOfFoundTowers ++;
        }
        if (tower3Found) {
          numOfFoundTowers ++;
        }
        if (tower4Found) {
          numOfFoundTowers ++;
        }

        if (numOfFoundTowers >= 3) {
          motors.setSpeeds(0, 0);

#ifdef DEBUG
          // try trianglation
          Serial.print("Number of found towers = ");
          Serial.print(numOfFoundTowers);
          Serial.print("  (tower1, tower2, tower3, tower4) = (");
          Serial.print(angle1);
          Serial.print(", ");
          Serial.print(angle2);
          Serial.print(", ");
          Serial.print(angle3);
          Serial.print(", ");
          Serial.print(angle4);
          Serial.print(") ");
          Serial.println("   Trying to do triangulation");
#endif

          int numOfCalculatedNegativeAngle = 0;

          float angle21 = 0.0;
          float angle32 = 0.0;
          float angle43 = 0.0;
          float angle14 = 0.0;

          if (tower1Found && tower2Found) {
            angle21 = angle2 - angle1;
          }
          if (tower2Found && tower3Found) {
            angle32 = angle3 - angle2;
          }
          if (tower3Found && tower4Found) {
            angle43 = angle4 - angle3;
          }
          if (tower4Found && tower1Found) {
            angle14 = angle1 - angle4;
          }

          if (angle21 < 0) {
            angle21 += 360;
            numOfCalculatedNegativeAngle++;
          }

          if (angle32 < 0) {
            angle32 += 360;
            numOfCalculatedNegativeAngle++;
          }

          if (angle43 < 0) {
            angle43 += 360;
            numOfCalculatedNegativeAngle++;
          }

          if (angle14 < 0) {
            angle14 += 360;
            numOfCalculatedNegativeAngle++;
          }

          if (numOfCalculatedNegativeAngle > 1) {
#ifdef DEBUG
            Serial.println("The measurement crossed 360 degree more than once. Something is wrong!");
#endif
            coordsCalculatable = false;
          } else {
            COORDS coords;
            if (tower1Found && tower2Found && tower3Found) {
              //              coords = trianglate123(angle32, angle21);
              Triangulation triangulation(WIDTH, HEIGHT, ToRad(angle32), ToRad(angle21));
              coords = triangulation.triangulate();
              coords.x = WIDTH - coords.x;
              coords.y = HEIGHT - coords.y;;
#ifdef SHOW_RESULT
              Serial.print("Result of trianglate123  ====>  ");
              printCoords(coords);
#endif
            } else if (tower1Found && tower3Found && tower4Found) {
              //              coords = trianglate134(angle14, angle43);
              Triangulation triangulation(WIDTH, HEIGHT, ToRad(angle14), ToRad(angle43));
              coords = triangulation.triangulate();

#ifdef SHOW_RESULT
              Serial.print("Result of trianglate134  ====>  ");
              printCoords(coords);
#endif
            } else if (tower1Found && tower2Found && tower4Found) {
              //              coords = trianglate124(angle21, angle14);
              Triangulation triangulation(HEIGHT, WIDTH, ToRad(angle21), ToRad(angle14));
              coords = triangulation.triangulate();
              float yprime = coords.y;
              coords.y = HEIGHT - coords.x;;
              coords.x = yprime;
#ifdef SHOW_RESULT
              Serial.print("Result of trianglate124  ====>  ");
              printCoords(coords);
#endif
            } else if (tower2Found && tower3Found && tower4Found) {
              //              coords = trianglate234(angle43, angle32);
              Triangulation triangulation(HEIGHT, WIDTH, ToRad(angle43), ToRad(angle32));
              coords = triangulation.triangulate();
              float yprime = coords.y;
              coords.y = coords.x;;
              coords.x = WIDTH - yprime;
#ifdef SHOW_RESULT
              Serial.print("Result of trianglate234  ====>  ");
              printCoords(coords);
#endif
            }
            coordsCalculated = true;

#ifdef SHOW_RESULT
            Serial.println("Trying to validate the result: ");
            printCoords(coords);
            Serial.print("resetCounter = ");
            Serial.println(resetCounter);
#endif

            //            Serial.print("(X, Y) = (");


            if ((coords.x <= 0 || coords.x >= WIDTH || coords.y <= 0 || coords.y >= HEIGHT) && resetCounter < NUM_OF_RESETS_TO_TRY) {
              reset();
            } else {
              Serial.println(coords.x);
              Serial.println(coords.y);
            }

          }
        } else {
          coordsCalculatable = false;
        }

      } else if (!coordsCalculatable && resetCounter < NUM_OF_RESETS_TO_TRY) {

#ifdef DEBUG
        Serial.print("Number of found towers = ");
        Serial.print(numOfFoundTowers);
        Serial.print("  (tower1, tower2, tower3, tower4) = (");
        Serial.print(angle1);
        Serial.print(", ");
        Serial.print(angle2);
        Serial.print(", ");
        Serial.print(angle3);
        Serial.print(", ");
        Serial.print(angle4);
        Serial.print(") ");

        Serial.println("The coordinate cannot be found at this point. Resetting");
        // move a small ramdom distance and do the trianglation again.
#endif
        int randomSpeed1 = (int) random (-90, 90);
        int randomSpeed2 = (int) random (-90, 90);

        motors.setSpeeds (randomSpeed1, randomSpeed2);

        int randomDelay = (int) random (0, 20);
        //      delay (randomDelay);
        delay (300);

        motors.setSpeeds (0, 0);

        reset ();
      }
    }
  }
  /*
    Serial.println("writing:");
  int bytes = Serial.available();
  char buffer[bytes + 1];
  for (int i = 0; i < bytes; i++) {
    buffer[i] = Serial.read();
  }
  buffer[bytes] = '\0';
  Serial.println(buffer);
  */
}

boolean lookForBlackSpot ()
{
  unsigned int sensors[SENSOR_SIZE];
  reflectanceSensors.readCalibrated (sensors); // method is void but sensor values are returned in array

  for (int i = 0; i < SENSOR_SIZE; i++) {
    if (sensors[i] >= 990) {
#ifdef DEBUG
      logStr("i = ", i + "");
      logStr("sensors[i] = ", sensors[i] + "");
      Serial.println ("Found black spot!");
#endif

      return true;
    }
  }
  return false;
}

void stayAwayFromLine()
{
  motors.setSpeeds(0, 0);
  reflectanceSensors.read(sensor_values);

  int sum = 0;

  for (int i = 0; i < 6; i++) {
    sum += sensor_values[i];
  }

  int average = sum / 6;

  if (sensor_values[0] == 2000 || sensor_values[1] == 2000 ||
      sensor_values[2] == 2000 || sensor_values[3] == 2000 ||
      sensor_values[4] == 2000 || sensor_values[5] == 2000) {
  }

  if (sensor_values[0] > BLUE_MIN && sensor_values[0] < BLUE_MAX) // leftmost
  {
    motors.setSpeeds(-MOVE_SPEED, -MOVE_SPEED);
    delay(REVERSE_DURATION);

    motors.setSpeeds(TURN_SPEED, -TURN_SPEED);
    delay(TURN_DURATION);

    motors.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);
#ifdef DEBUG

    Serial.println("Blue is at the left most  ");
#endif
  }
  else if (sensor_values[5] > BLUE_MIN && sensor_values[5] < BLUE_MAX)
  {
    motors.setSpeeds(-MOVE_SPEED, -MOVE_SPEED);
    delay(REVERSE_DURATION);

    motors.setSpeeds(-TURN_SPEED, TURN_SPEED);
    delay(TURN_DURATION);

    motors.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);
#ifdef DEBUG
    Serial.println("Blue is at the right most  ");
#endif
  }

#ifdef DEBUG
  Serial.print("0 read: ");
  Serial.println(sensor_values[0]);

  Serial.print("1 read: ");
  Serial.println(sensor_values[1]);

  Serial.print("2 read: ");
  Serial.println(sensor_values[2]);

  Serial.print("3 read: ");
  Serial.println(sensor_values[3]);

  Serial.print("4 read: ");
  Serial.println(sensor_values[4]);

  Serial.print("5 read: ");
  Serial.println(sensor_values[5]);
#endif

  delay(20);
}

void reset () {
  angle1 = 0.0; // the angle for tower 1
  angle2 = 0.0; // the angle for tower 2
  angle3 = 0.0; // the angle for tower 3
  angle4 = 0.0; // the angle for tower 4

  angle1Counter = 0; // the angle counter for tower 1
  angle2Counter = 0; // the angle counter for tower 2
  angle3Counter = 0; // the angle counter for tower 3
  angle4Counter = 0; // the angle counter for tower 4

  tower1Found = false;
  tower2Found = false;
  tower3Found = false;
  tower4Found = false;
  numOfFoundTowers = 0;
  /*
    angle21 = 0.0;
    angle32 = 0.0;
    angle43 = 0.0;
    angle14 = 0.0;
  */

  coordsCalculatable = true;
  coordsCalculated = false;

  startingTime = micros();
  resetCounter ++;

}

Signal readIRSensor() {
  Signal sig;
  decode_results results;

  if (irrecv.decode(&results)) {
#ifdef DEBUG
    Serial.println("Got signal");
    Serial.println(results.decode_type);
#endif
    sig.type = results.decode_type;
#ifdef DEBUG
    Serial.println(results.value, HEX); // binary but with base 16 instead of base 2
#endif
    sig.value = results.value;

    irrecv.resume(); // Receive the next value
  }

  return sig;
}

void findTowerSignalAngle() {
#ifdef DEBUG
  Serial.println("In find tower signal angle:");
#endif

  Signal sig = readIRSensor();
  if (isSignalMatching(sig, IR_SIGNAL_TYPE, KEY_1_HEX_VALUE)) {
    tower1Found = true;
    float newAngle = getYawAngle();
    angle1 = (angle1 * angle1Counter + newAngle) / (angle1Counter + 1);
    angle1Counter ++;
#ifdef DEBUG
    Serial.print("Tower 1 found with angle = ");
    Serial.println(angle1);
#endif
  } else if (isSignalMatching(sig, IR_SIGNAL_TYPE, KEY_2_HEX_VALUE)) {
    tower2Found = true;
    float newAngle = getYawAngle();
    angle2 = (angle2 * angle2Counter + newAngle) / (angle2Counter + 1);
    angle2Counter ++;
#ifdef DEBUG
    Serial.print("Tower 2 found with angle = ");
    Serial.println(angle2);
#endif
  } else if (isSignalMatching(sig, IR_SIGNAL_TYPE, KEY_3_HEX_VALUE)) {
    tower3Found = true;
    float newAngle = getYawAngle();
    angle3 = (angle3 * angle3Counter + newAngle) / (angle3Counter + 1);
    angle3Counter ++;
#ifdef DEBUG
    Serial.print("Tower 3 found with angle = ");
    Serial.println(angle3);
#endif
  } else if (isSignalMatching(sig, IR_SIGNAL_TYPE, KEY_4_HEX_VALUE)) {
    tower4Found = true;
    float newAngle = getYawAngle();
    angle4 = (angle4 * angle4Counter + newAngle) / (angle4Counter + 1);
    angle4Counter ++;
#ifdef DEBUG
    Serial.print("Tower 4 found with angle = ");
    Serial.println(angle4);
#endif
  }
}



bool isSignalMatching(Signal sig, int signalType, int signalValue) {
  return sig.type == signalType && sig.value == signalValue;
}


void IMUinit() {

  float gyroSumX, gyroSumY, gyroSumZ;

  //init devices
  compass.init();
  gyro.init();

  gyro.writeReg(L3G_CTRL_REG1, 0xCF);
  gyro.writeReg(L3G_CTRL_REG2, 0x00);
  gyro.writeReg(L3G_CTRL_REG3, 0x00);
  gyro.writeReg(L3G_CTRL_REG4, 0x20); //
  gyro.writeReg(L3G_CTRL_REG5, 0x02);
  /*
    compass.writeAccReg(LSM303_CTRL_REG1_A, 0x77);//400hz all enabled
    compass.writeAccReg(LSM303_CTRL_REG4_A, 0x20);//+/-8g 4mg/LSB

    compass.writeMagReg(LSM303_CRA_REG_M, 0x1C);
    compass.writeMagReg(LSM303_CRB_REG_M, 0x60);
    compass.writeMagReg(LSM303_MR_REG_M, 0x00);
  */

  compass.writeAccReg(LSM303::CTRL_REG1_A, 0x77);//400hz all enabled
  compass.writeAccReg(LSM303::CTRL_REG4_A, 0x20);//+/-8g 4mg/LSB

  compass.writeMagReg(LSM303::CRA_REG_M, 0x1C);
  compass.writeMagReg(LSM303::CRB_REG_M, 0x60);
  compass.writeMagReg(LSM303::MR_REG_M, 0x00);

  beta = betaDef;
  //calculate initial quaternion
  //take an average of the gyro readings to remove the bias

  for (i = 0; i < 500; i++) {
    gyro.read();
    compass.read();
    Smoothing((float*)&compass.a.x, &smoothAccX);
    Smoothing((float*)&compass.a.y, &smoothAccY);
    Smoothing((float*)&compass.a.z, &smoothAccZ);
    delay(3);
  }
  gyroSumX = 0;
  gyroSumY = 0;
  gyroSumZ = 0;
  for (i = 0; i < 500; i++) {
    gyro.read();
    compass.read();
    Smoothing((float*)&compass.a.x, &smoothAccX);
    Smoothing((float*)&compass.a.y, &smoothAccY);
    Smoothing((float*)&compass.a.z, &smoothAccZ);
    gyroSumX += (gyro.g.x);
    gyroSumY += (gyro.g.y);
    gyroSumZ += (gyro.g.z);
    delay(3);
  }
  offSetX = gyroSumX / 500.0;
  offSetY = gyroSumY / 500.0;
  offSetZ = gyroSumZ / 500.0;

  do {
    compass.read();

    //calculate the initial quaternion
    //these are rough values. This calibration works a lot better if the device is kept as flat as possible
    //find the initial pitch and roll
    pitch = ToDeg(fastAtan2(compass.a.x, sqrt(compass.a.y * compass.a.y + compass.a.z * compass.a.z)));
    roll = ToDeg(fastAtan2(-1 * compass.a.y, sqrt(compass.a.x * compass.a.x + compass.a.z * compass.a.z)));


    if (compass.a.z > 0) {
      if (compass.a.x > 0) {
        pitch = 180.0 - pitch;
      }
      else {
        pitch = -180.0 - pitch;
      }
      if (compass.a.y > 0) {
        roll = -180.0 - roll;
      }
      else {
        roll = 180.0 - roll;
      }
    }

    floatMagX = (compass.m.x - compassXMin) * inverseXRange - 1.0;
    floatMagY = (compass.m.y - compassYMin) * inverseYRange - 1.0;
    floatMagZ = (compass.m.z - compassZMin) * inverseZRange - 1.0;
    //tilt compensate the compass
    float xMag = (floatMagX * cos(ToRad(pitch))) + (floatMagZ * sin(ToRad(pitch)));
    float yMag = -1 * ((floatMagX * sin(ToRad(roll))  * sin(ToRad(pitch))) + (floatMagY * cos(ToRad(roll))) - (floatMagZ * sin(ToRad(roll)) * cos(ToRad(pitch))));

    yaw = ToDeg(fastAtan2(yMag, xMag));

    if (yaw < 0) {
      yaw += 360;
    }
#ifdef DEBUG
    Serial.println(pitch);
    Serial.println(roll);
    Serial.println(yaw);
#endif
  } while (isnan(pitch) || isnan(roll) || isnan(yaw));

  //calculate the rotation matrix
  float cosPitch = cos(ToRad(pitch));
  float sinPitch = sin(ToRad(pitch));

  float cosRoll = cos(ToRad(roll));
  float sinRoll = sin(ToRad(roll));

  float cosYaw = cos(ToRad(yaw));
  float sinYaw = sin(ToRad(yaw));

  //need the transpose of the rotation matrix
  float r11 = cosPitch * cosYaw;
  float r21 = cosPitch * sinYaw;
  float r31 = -1.0 * sinPitch;

  float r12 = -1.0 * (cosRoll * sinYaw) + (sinRoll * sinPitch * cosYaw);
  float r22 = (cosRoll * cosYaw) + (sinRoll * sinPitch * sinYaw);
  float r32 = sinRoll * cosPitch;

  float r13 = (sinRoll * sinYaw) + (cosRoll * sinPitch * cosYaw);
  float r23 = -1.0 * (sinRoll * cosYaw) + (cosRoll * sinPitch * sinYaw);
  float r33 = cosRoll * cosPitch;



  //convert to quaternion
  q0 = 0.5 * sqrt(1 + r11 + r22 + r33);
  q1 = (r32 - r23) / (4 * q0);
  q2 = (r13 - r31) / (4 * q0);
  q3 = (r21 - r12) / (4 * q0);


}

void IMUupdate(float *dt) {
  static float gx;
  static float gy;
  static float gz;
  static float ax;
  static float ay;
  static float az;

  static float recipNorm;
  static float s0, s1, s2, s3;
  static float qDot1, qDot2, qDot3, qDot4;
  static float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 , _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

  gx = ToRad((gyro.g.x - offSetX) * GYRO_SCALE);
  gy = ToRad((gyro.g.y - offSetY) * GYRO_SCALE);
  gz = ToRad((gyro.g.z - offSetZ) * GYRO_SCALE);

  ax = -1.0 * compass.a.x;
  ay = -1.0 * compass.a.y;
  az = -1.0 * compass.a.z;
  // Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

  magnitude = sqrt(ax * ax + ay * ay + az * az);
  if ((magnitude > 384) || (magnitude < 128)) {
    ax = 0;
    ay = 0;
    az = 0;
  }

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

    // Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Auxiliary variables to avoid repeated arithmetic
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _4q0 = 4.0f * q0;
    _4q1 = 4.0f * q1;
    _4q2 = 4.0f * q2;
    _8q1 = 8.0f * q1;
    _8q2 = 8.0f * q2;
    q0q0 = q0 * q0;
    q1q1 = q1 * q1;
    q2q2 = q2 * q2;
    q3q3 = q3 * q3;

    // Gradient decent algorithm corrective step
    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // Apply feedback step
    qDot1 -= beta * s0;
    qDot2 -= beta * s1;
    qDot3 -= beta * s2;
    qDot4 -= beta * s3;
  }

  // Integrate rate of change of quaternion to yield quaternion
  q0 += qDot1 **dt;
  q1 += qDot2 **dt;
  q2 += qDot3 **dt;
  q3 += qDot4 **dt;

  // Normalise quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
}

void AHRSupdate(float *dt) {
  static float gx;
  static float gy;
  static float gz;
  static float ax;
  static float ay;
  static float az;
  static float mx;
  static float my;
  static float mz;


  static float recipNorm;
  static float s0, s1, s2, s3;
  static float qDot1, qDot2, qDot3, qDot4;
  static float hx, hy;
  static float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

  gx = ToRad((gyro.g.x - offSetX) * GYRO_SCALE);
  gy = ToRad((gyro.g.y - offSetY) * GYRO_SCALE);
  gz = ToRad((gyro.g.z - offSetZ) * GYRO_SCALE);

  ax = -1.0 * compass.a.x;
  ay = -1.0 * compass.a.y;
  az = -1.0 * compass.a.z;

  mx = floatMagX;
  my = floatMagY;
  mz = floatMagZ;
  // Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

  magnitude = sqrt(ax * ax + ay * ay + az * az);

  if ((magnitude > 384) || (magnitude < 128)) {
    ax = 0;
    ay = 0;
    az = 0;
  }

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {


    // Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;
    // Normalise magnetometer measurement
    recipNorm = invSqrt(mx * mx + my * my + mz * mz);
    mx *= recipNorm;
    my *= recipNorm;
    mz *= recipNorm;
    // Auxiliary variables to avoid repeated arithmetic
    _2q0mx = 2.0f * q0 * mx;
    _2q0my = 2.0f * q0 * my;
    _2q0mz = 2.0f * q0 * mz;
    _2q1mx = 2.0f * q1 * mx;
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _2q0q2 = 2.0f * q0 * q2;
    _2q2q3 = 2.0f * q2 * q3;
    q0q0 = q0 * q0;
    q0q1 = q0 * q1;
    q0q2 = q0 * q2;
    q0q3 = q0 * q3;
    q1q1 = q1 * q1;
    q1q2 = q1 * q2;
    q1q3 = q1 * q3;
    q2q2 = q2 * q2;
    q2q3 = q2 * q3;
    q3q3 = q3 * q3;



    // Reference direction of Earth's magnetic field
    hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
    hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
    _2bx = sqrt(hx * hx + hy * hy);
    _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
    _4bx = 2.0f * _2bx;
    _4bz = 2.0f * _2bz;

    // Gradient decent algorithm corrective step
    s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // Apply feedback step
    qDot1 -= beta * s0;
    qDot2 -= beta * s1;
    qDot3 -= beta * s2;
    qDot4 -= beta * s3;
  }

  // Integrate rate of change of quaternion to yield quaternion
  q0 += qDot1 **dt;
  q1 += qDot2 **dt;
  q2 += qDot3 **dt;
  q3 += qDot4 **dt;

  // Normalise quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
}

void GetEuler(void) {
  roll = ToDeg(fastAtan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2)));
  pitch = ToDeg(asin(2 * (q0 * q2 - q3 * q1)));
  yaw = ToDeg(fastAtan2(2 * (q0 * q3 + q1 * q2) , 1 - 2 * (q2 * q2 + q3 * q3)));
  if (yaw < 0) {
    yaw += 360;
  }

}
float fastAtan2( float y, float x)
{
  static float atan;
  static float z;
  if ( x == 0.0f )
  {
    if ( y > 0.0f ) return PIBY2_FLOAT;
    if ( y == 0.0f ) return 0.0f;
    return -PIBY2_FLOAT;
  }
  z = y / x;
  if ( fabs( z ) < 1.0f )
  {
    atan = z / (1.0f + 0.28f * z * z);
    if ( x < 0.0f )
    {
      if ( y < 0.0f ) return atan - PI_FLOAT;
      return atan + PI_FLOAT;
    }
  }
  else
  {
    atan = PIBY2_FLOAT - z / (z * z + 0.28f);
    if ( y < 0.0f ) return atan - PI_FLOAT;
  }
  return atan;
}

float invSqrt(float number) {
  volatile long i;
  volatile float x, y;
  volatile const float f = 1.5F;

  x = number * 0.5F;
  y = number;
  i = * ( long * ) &y;
  i = 0x5f375a86 - ( i >> 1 );
  y = * ( float * ) &i;
  y = y * ( f - ( x * y * y ) );
  return y;
}

void Smoothing(float *raw, float *smooth) {
  *smooth = (*raw * (0.15)) + (*smooth * 0.85);
}
/*
void Smoothing(int *raw, float *smooth) {
  *smooth = (*raw * (0.15)) + (*smooth * 0.85);
}
*/
float getYawAngle() {
  GetEuler();
#ifdef DEBUG
  Serial.print("Yaw Angle =  ");
  Serial.println(yaw);
#endif
  return yaw;
}

#ifdef SHOW_RESULT
void printCoords(COORDS coords) {
  Serial.print("(X, Y) = (");
  Serial.print(coords.x);
  Serial.print(", ");
  Serial.print(coords.y);
  Serial.println(")");
}
#endif

char move_to(char dir){
  switch (dir) { // scan ahead
  case 'f':  
    motors.setSpeeds (MOVE_SPEED, MOVE_SPEED); // goes straight
    delay (DELAY_TIME);

    stayAwayFromLine ();
    
    motors.setSpeeds ((int) random (-MOVE_SPEED, MOVE_SPEED), (int) random (-MOVE_SPEED, MOVE_SPEED)); // random turn
    //    motors.setSpeeds ((pow (-1, (int) random (0, 2))) * MOVE_SPEED, (pow (-1, (int) random (0, 2))) * MOVE_SPEED); // random turn
    delay ((int) (2 * DELAY_TIME * random (0, 100) / 100.0));
    stayAwayFromLine ();

    blackSpotFound = lookForBlackSpot (); // look for black spot

    //Serial.println("move_to>> Turn  forward");
    break;
  }
}

