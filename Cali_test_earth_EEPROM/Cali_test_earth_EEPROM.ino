/* This .ino is functional for calibration the IMU and store the data into flash
 *  address of 0x00. 
 *  To calibrate the IMU, it should first place flat and enter letter 'c' by serial 
 *  port to activate the calibration funtion. It will take 50 samples for 6 direction
 *  wihch +x, +y, +z, -x, -y, -z total of 300 samples. 
 *  The user should face each direction down when the monitor mention "Flip IMU to next 
 *  axis"
                Another library: Wire.h
   Port connection: MPU9250 --- ESP32
                      VCC   --- 3V3
                      GND   --- GND
                      SCL   --- 22
                      SDA   --- 21
   Debug use: by send defferent char by serial to esp32, it will output current states:
              "a" :calibrated acceleration data, unit: G
              "A" :Calibrate acceleration data
              "v" :Return calibrated body Velocity data, unit: m/s
              "V" :Return calibrated earth Velocity data, unit: m/s
              "p" :Return calibrated body position data, unit: m
              "P" :Return calibrated earth position data, unit: m
              "r" :Rest position data to oringe(xyz=0,0,0).
              "o" :return compared data before and after calibration, 
                   a/y/z and abs acc.
              "c" :Do calibration.

*/
#include "MPU9250.h"
#include "BasicLinearAlgebra.h"
#include <TinyGPS++.h>
#include <EEPROM.h>

//#define debug 1;

using namespace BLA;
MPU9250 mpu;

float a[3] = {0, 0, 0};
float a_org[3] = {0, 0, 0};
float v[3] = {0, 0, 0};
float p[3] = {0, 0, 0};
float ae[3] = {0, 0, 0};
float ve[3] = {0, 0, 0};
float pe[3] = {0, 0, 0};
char print_ = 'a';
int calibration[1] = {2};
int count = 0;
float fix[20] = {};
float m_ = 1, b_ = 0;
//float lms_mb[6] = {1.0015, -0.0123, 1.0038, 0.0041, 0.9730, -0.01071};
float lms_mb[6] = {1, 0, 1, 0, 1, 0};
float error = 0.01;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  delay(2000);

  if (!mpu.setup(0x68)) {  // change to your own address
    while (1) {
      Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
      delay(5000);
    }
  }

  // calibrate anytime you want to
  Serial.println("Accel Gyro calibration will start in 5sec.");
  Serial.println("Please leave the device still on the flat plane.");
  mpu.verbose(true);
  delay(5000);
  mpu.calibrateAccelGyro();

  Serial.println("Mag calibration will start in 5sec.");
  Serial.println("Please Wave device in a figure eight until done.");
  //delay(5000);
  //mpu.calibrateMag();
  mpu.setMagBias(24.68, 418.01, -487.37);

  print_calibration();
  mpu.verbose(false);

  EEPROM.begin(50);
#ifdef debug
  float lms[6] = {1.0015, -0.0123, 1.0038, 0.0041, 0.9730, -0.01071};
  //float lms[6] = {22, 11, 1, 10, 44, 33};
  for (int i = 0; i < 6 * sizeof(float); i += sizeof(float)) {
    EEPROM.writeFloat(i, lms[i / sizeof(float)]);
    Serial.println(lms[i / sizeof(float)]);
  }
  EEPROM.commit();
#endif
  for (int i = 0; i < 6 * sizeof(float); i += sizeof(float)) {
    lms_mb[i / sizeof(float)] = EEPROM.readFloat(i);
    Serial.println(EEPROM.readFloat(i), 6);
  }

}


void loop() {
  if (mpu.update()) {
    static uint32_t prev_ms = millis();
    if (millis() > prev_ms + 100) {
      float time_ = (millis() - prev_ms);
      a_org[0] = mpu.getAccX(); a_org[1] = mpu.getAccY(); a_org[2] = mpu.getAccZ();
      a[0] = lms_mb[0] * mpu.getAccX() + lms_mb[1];
      a[1] = lms_mb[2] * mpu.getAccY() + lms_mb[3];
      a[2] = lms_mb[4] * mpu.getAccZ() + lms_mb[5];
      measure_imu_data();
      for (int i = 0; i < 3; i++) v[i] += a[i] * time_ / 1000 * 9.81;
      for (int i = 0; i < 3; i++) p[i] += v[i] * time_ / 1000 * 0.5;
      for (int i = 0; i < 3; i++) ve[i] += ae[i] * time_ / 1000 * 9.81;
      for (int i = 0; i < 3; i++) pe[i] += ve[i] * time_ / 1000 * 0.5;
      //Check stational for IMU
      fix[0] = abs(get_sqre(ae, 3) - 1);
      for (int i = 0; i < 20; i++) fix[i + 1] = fix[i];
      int stationary = 0;
      for (int i = 0; i < 15; i++) {
        if (fix[i] <= error) stationary += 1;
      }
      if (stationary >= 15) {
        for (int i = 0; i < 3; i++) v[i] = 0;
        for (int i = 0; i < 3; i++) ve[i] = 0;
        for (int i = 0; i < 20; i++) fix[i] = 1;
      }
      //Physical reset button
      //      Serial << "Read: " << digitalRead(32)
      //             << " ReadA: " << analogRead(32)<< '\n';
      //if (analogRead(32) >= 200) for (int i = 0; i < 6; i++) p[i] = 0;
      prev_ms = millis();

      print_data(print_);

      //Serial.println(calibration[0]);

      //Run Calibration
      if (calibration[0] == 1) {
        int n = 50;
        float accx[6 * n] = {};
        float accy[6 * n] = {};
        float accz[6 * n] = {};
        Serial.println(" Get 100 samples \n");
        for (int j = 0; j < 6; j++) {
          Serial << "Press k to cuntinue\n";
          while (1) {
            if (Serial.available()) {
              char c = Serial.read();
              if (c == 'k') break;
            }
          }

          for (int i = (j * 50); i < (j * 50 + n); i++) { //Get n samples of acceleration
            if (mpu.update()) {
              accx[i] = mpu.getAccX();
              accy[i] = mpu.getAccY();
              accz[i] = mpu.getAccZ();
              Serial << "Count: " << i
                     << " x: " << accx[i]
                     << " y: " << accy[i]
                     << " z: " << accz[i]
                     << '\n';
              delay(50);
            }
          }
          Serial << "Flip IMU to next axis\n";
        }

        Serial << "get all data done \n";
        //Write into matrix, contains m & b for x, y, z;
        LMS_para(accx, 6 * n);
        lms_mb[0] = m_; lms_mb[1] = b_;
        LMS_para(accy, 6 * n);
        lms_mb[2] = m_; lms_mb[3] = b_;
        LMS_para(accz, 6 * n);
        lms_mb[4] = m_; lms_mb[5] = b_;
        //Write data into Flash
        //EEPROM.begin(24);
        int count_lms = 0;
        for (int i = 0; i < (6 * sizeof(float)); i += sizeof(float)) {
          EEPROM.writeFloat(i, lms_mb[i / sizeof(float)]);
          //Serial.println(lms_mb[i]);
          Serial.println(EEPROM.readFloat(i / sizeof(float)));
        }
        EEPROM.commit();
        Serial.println("Store flash done.");
        Serial << "para:";
        for (int i = 0; i < 3; i++) {
          Serial << " m" << i + 1 << ": ";
          Serial.print(lms_mb[i * 2], 5);
          Serial << " b" << i + 2 << ": ";
          Serial.print(lms_mb[i * 2 + 1], 5);
        }
        Serial << '\n';
        calibration[0] = 0;
      }

      if (Serial.available()) {
        char c = Serial.read();
        delay(200);

        if (c == 'c') {
          Serial << "Run Calibration: ----------------------\n";
          calibration[0] = 1;
        } else if (c == 'r') {
          Serial << "Reset position\n";
          for (int i = 0; i < 3; i++) {
            p[i] = 0;
            v[i] = 0;
            pe[i] = 0;
            ve[i] = 0;
          }
        } else if (c == 'a' | c == 'v' | c == 'p' | c == 'o' | c == 'A' | c == 'V' | c == 'P')print_ = c;
      }
    }
  }
}


void print_data(char print_) {
  if (print_ == 'a') {
    Serial << "Ax: " << a[0]
           << " Ay: " << a[1]
           << " Az_e: " << a[2] << " ";
    Serial.println(get_sqre(ae, 3), 6);
  } else if (print_ == 'v') {
    Serial << "Vx: " << v[0]
           << " Vy: " << v[1]
           //<< " Vz: " << v[2]
           << '\n';
  } else if (print_ == 'p') {
    Serial << "Px: " << p[0]
           << " Py: " << p[1]
           //<< " Pz: " << p[2]
           << '\n';
  }
  else if (print_ == 'o') {
    Serial << "[x,y,z] Before: ["
           << a_org[0] << ',' << a_org[1] << ',' << a_org[2]
           << "] ";
    Serial.print(get_sqre(a_org, 3), 6);
    Serial << "After: ["
           << a[0] << ',' << a[1] << ',' << a[2]
           << "] " ;
    Serial.println(get_sqre(a, 3), 6);
  } else if (print_ == 'A') {
    Serial << "Ax_e: " << ae[0]
           << " Ay_e: " << ae[1]
           << " Az_e: " << ae[2] << " ";
    Serial.println(get_sqre(ae, 3), 6);
  } else if (print_ == 'V') {
    Serial << "Vex: " << ve[0]
           << " Vey: " << ve[1]
           //<< " Vez: " << ve[2]
           << '\n';
  } else if (print_ == 'P') {
    Serial << "Pex: " << pe[0]
           << " Pey: " << pe[1]
           //<< " Pez: " << pe[2]
           << '\n';
  }
}

void LMS_para(float x[], int n) {
  float sumx = 0, sumx2 = 0, sumxy = 0, sumy = 0, sumy2 = 0;
  float y[n] = {};
  Serial << "Create y array, -1, 0, 1\n";
  //Sum loop
  for (int i = 0; i < n; i++) {
    //Create y array, -1, 0, 1
    if (x[i] < -0.5) {
      y[i] = -1;
    } else if (x[i] > 0.5) {
      y[i] = 1;
    } else {
      y[i] = 0;
    }
    Serial << x[i] << "," << y[i] << "  ";
    sumx += x[i];
    sumx2 += x[i] * x[i];
    sumxy += x[i] * y[i];
    sumy += y[i];
    sumy2 += y[i] * y[i];
  }
  Serial << "Loop done\n";
  float denom = (n * sumx2 - sumx * sumx);
  if (denom == 0) {
    // singular matrix. can't solve the problem.
    Serial.println("No solution\n");
    m_ = 1;
    b_ = 0;
  } else {
    m_ = (n * sumxy - sumx * sumy) / denom;
    b_ = (sumy * sumx2 - sumx * sumxy) / denom;
    Serial << "Solution get, m= " << m_ << " b= " << b_ << '\n' ;
  }
}

float get_sqre(float x[], int n) {
  float sqar = 0;
  for (int i = 0; i < n; i++) {
    sqar += x[i] * x[i];
  }
  return sqrt(sqar);
}

//Rotate frame to earth frame
void measure_imu_data() {
  float acc_the = mpu.getRoll() / 180.f * PI;
  float acc_fin = mpu.getPitch() / 180.f * PI;
  float acc_psi = mpu.getYaw() / 180.f * PI;

  //rotate from x-axis
  float acc_y__ = a[1] * cos(acc_the) - a[2] * sin(acc_the);
  float acc_z__ = a[2] * cos(acc_the) + a[1] * sin(acc_the);
  //rotate from y-axis
  float acc_x_ = a[0] * cos(acc_fin) - acc_z__ * sin(acc_fin);
  float acc_z_ = acc_z__ * cos(acc_fin) + a[0] * sin(acc_fin);
  //rotate from z-axis
  ae[0] = acc_x_ * cos(acc_psi) + acc_y__ * sin(acc_psi);
  ae[1] = acc_y__ * cos(acc_psi) - acc_x_ * sin(acc_psi);
  ae[2] = acc_z_;
#ifdef DEBUG
  Serial << " Acc: " << acc << " Yew[0]: " << float(mpu.getYaw() / 180.f * PI)
         << " pitch: " << float(mpu.getPitch() / 180.f * PI)
         << " Roll: " << float(mpu.getRoll() / 180.f * PI) << " \n ";
#endif
}



void print_calibration() {
  Serial.println("< calibration parameters >");
  Serial.println("accel bias [g]: ");
  Serial.print(mpu.getAccBiasX() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
  Serial.print(", ");
  Serial.print(mpu.getAccBiasY() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
  Serial.print(", ");
  Serial.print(mpu.getAccBiasZ() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
  Serial.println();
  Serial.println("gyro bias [deg/s]: ");
  Serial.print(mpu.getGyroBiasX() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
  Serial.print(", ");
  Serial.print(mpu.getGyroBiasY() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
  Serial.print(", ");
  Serial.print(mpu.getGyroBiasZ() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
  Serial.println();
  Serial.println("mag bias [mG]: ");
  Serial.print(mpu.getMagBiasX());
  Serial.print(", ");
  Serial.print(mpu.getMagBiasY());
  Serial.print(", ");
  Serial.print(mpu.getMagBiasZ());
  Serial.println();
  Serial.println("mag scale []: ");
  Serial.print(mpu.getMagScaleX());
  Serial.print(", ");
  Serial.print(mpu.getMagScaleY());
  Serial.print(", ");
  Serial.print(mpu.getMagScaleZ());
  Serial.println();
}
