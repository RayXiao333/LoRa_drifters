/* This ino code performs the MPU9250 IMU to predict the position
 * The system is runing at 40Hz, the front Y-axis is pointing to West (Yew = 0 degree)
 * MPU library: https://github.com/hideakitai/MPU9250
 *              Another library: Wire.h
 * Port connection: MPU9250 --- ESP32
 *                    VCC   --- 3V3
 *                    GND   --- GND
 *                    SCL   --- 22
 *                    SDA   --- 21
 * Debug use: by send defferent char by serial to esp32, it will output current states:
 *            "a" :Raw acceleration data, unit: G
 *            "A" :Processed acceleration data on Earth Frame, unit: m/s2
 *            "R" :Yew, Pitch, Roll data(Y axis point to West), unit: degree
 *            "v" :Return Velocity data, unit: m/s
 *            "p" :Return Position data, unit: cm
 *            "m" :Return Magnetometer data, unit: µTesla
 *            "u" :Increase threshold value for stationary by 0.001
 *            "d" :Increase threshold value for stationary by 0.001
 *            "r" :Rest position data to oringe(xyz=0,0,0).
*/
#include "MPU9250.h"

MPU9250 mpu;
int sample_period = 25; //in ms
int Freq_acc = 1000/sample_period; //40Hz
char in123= 'a';          // char for input debug
char incomingByte = 'a';
int count=0;              //Count for drift velocity elimination

//Define global variable
float acc[3] = {0, 0, 0};
float Acc_mag[2] = {0, 0};
float Acc[3][2] = {{}, {}};
float Vel[3][2] = {{}, {}}; //Vel( 3 raws, 2 cols   )
float Pos[3][2] = {{}, {}};
float thread_G = 0.01;
int stationary = 1;
int count_S = 0;

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
    delay(2000);
    mpu.calibrateMag();

    print_calibration();
    mpu.verbose(false);

    
}

void loop() {
    if (mpu.update()) {
        static uint32_t prev_ms = millis();
        if (millis() > prev_ms + sample_period) {
            Position_calulation();
            prev_ms = millis();
        }
        Print_Data();
    }
}


//Process Acceleration data to earth frame
void Get_globle_acc() {
  float acc_x___ = mpu.getAccX(); //get from mpu
  float acc_y___ = mpu.getAccY();
  float acc_z___ = mpu.getAccZ();
  float acc_the = mpu.getRoll()/180.f*PI;
  float acc_fin = mpu.getPitch()/180.f*PI;
  float acc_psi = mpu.getYaw()/180.f*PI;

  //Set Max Acc
  if (acc_x___ >= 2) acc_x___ = 2;
  if (acc_y___ >= 2) acc_y___ = 2;
  if (acc_z___ >= 2) acc_z___ = 2;

  //rotate from x-axis
  float acc_x__ = acc_x___;
  float acc_y__ = acc_y___ * cos(acc_the) - acc_z___ * sin(acc_the);
  float acc_z__ = acc_z___ * cos(acc_the) + acc_y___ * sin(acc_the);
  //rotate from y-axis
  float acc_x_ = acc_x__ * cos(acc_fin) - acc_z__ * sin(acc_fin);
  float acc_y_ = acc_y__;
  float acc_z_ = acc_z__ * cos(acc_fin) + acc_x__ * sin(acc_fin);
  //rotate from z-axis
  acc[0] = acc_x_ * cos(acc_psi) + acc_y_ * sin(acc_psi);
  acc[1] = acc_y_ * cos(acc_psi) - acc_x_ * sin(acc_psi);
  acc[2] = acc_z_;
}


//Get Acceleration on mm/s
void Get_Acc_e() {
  Get_globle_acc();
  //remove G and set unit as m/s^2
  Acc[0][0] = acc[0] * 9.806;
  Acc[1][0] = acc[1] * 9.806;
  Acc[2][0] = (acc[2] - 1.05) * 9.806;
  
  //Set station to reset speed to 0 m/s
  Acc_mag[1]=Acc_mag[0];
  Acc_mag[0] = sqrt(acc[0]*acc[0]+acc[1]*acc[1]+(acc[2])*(acc[2]));
  if(abs(Acc_mag[0] - Acc_mag[1]) < thread_G) count_S++;
  else count_S=0;
  if(count_S >= 3) stationary = 1;
  else stationary = 0;
  
}

//Get position prediction
void Position_calulation() {
  //Push data
  for (int j = 0; j < 3; j++) {
    for (int i = 1; i < 2; i++) {
      Acc[j][i] = Vel[j][i - 1];
      Vel[j][i] = Vel[j][i - 1];
      Pos[j][i] = Pos[j][i - 1];
    }
  }
  Get_Acc_e();

  //Get New speed
  for (int j = 0; j < 3; j++) {
    //if (stationary == 1) Acc[j] = 0;
    Vel[j][0] = Vel[j][1] + (Acc[j][0]+Acc[j][1])/2 * sample_period / 1000; //Unit m/s
    if (stationary == 1) Vel[j][0] = 0;
    //Pos[j][0] = Pos[j][1] + Vel[j][0] * sample_period / 1000 * 100;   //Unit cm
    Pos[j][0] = Pos[j][1] + (Vel[j][1] + (Acc[j][0]+Acc[j][1])/4 * sample_period / 1000) * sample_period / 1000 * 100;   //Unit cm
  }
  //drift_eliminator();
}

void Print_Data(){
  static uint32_t prev_ms_print = millis();
  if (millis() > prev_ms_print + 100) {
    //For debug display by serial port
    if (Serial.available() > 0) {
      incomingByte = Serial.read();
      //Serial.print("I reeceived: ");
      if (incomingByte == 'v' | incomingByte == 'a' | incomingByte == 'A' |
          incomingByte == 'p' | incomingByte == 'r' | incomingByte == 'm' |
          incomingByte == 'R' | incomingByte == 'g' | incomingByte == 'u' |
          incomingByte == 'd') {
        in123 = incomingByte;
      }
    }
    if (in123 == 'p') {
      Serial.print("Position e-frame (x,y,z):");
      Serial.print(Pos[0][0]);
      Serial.print("   ");
      Serial.print(Pos[1][0]);
      Serial.print("   ");
      Serial.println(Pos[2][0]);
    } else if (in123 == 'v') {
      Serial.print("Velocity e-frame (x,y,z):");
      Serial.print(Vel[0][0]);
      Serial.print("   ");
      Serial.print(Vel[1][0]);
      Serial.print("   ");
      Serial.print(Vel[2][0]);
      Serial.print("   ");
      Serial.println(abs(Acc_mag[0] - Acc_mag[1]));
    } else if (in123 == 'A') {
      Serial.println("Acceleration e-frame (x,y,z):");
      Serial.print(Acc[0][0]);
      Serial.print("   ");
      Serial.print(Acc[1][0]);
      Serial.print("   ");
      Serial.println(Acc[2][0]);
    } else if (in123 == 'a') {
      Serial.println("Acceleration in g (x,y,z):");
      Serial.print(acc[0]);
      Serial.print(" ");
      Serial.print(acc[1]);
      Serial.print(" ");
      Serial.println(acc[2]); 
      Serial.print("Resultant g: ");
      Serial.println(sqrt(acc[0]*acc[0]+acc[1]*acc[1]+(acc[2])*(acc[2])));
    } else if (in123 == 'g') {
//      Serial.println("Gyroscope data in degrees/s: ");
//      Serial.print(gyr.x);
//      Serial.print("   ");
//      Serial.print(gyr.y);
//      Serial.print("   ");
//      Serial.println(gyr.z);
      delay(500);
    } else if (in123 == 'R') {
      Serial.print("Yaw, Pitch, Roll: ");
      Serial.print(mpu.getYaw(), 2);
      Serial.print(", ");
      Serial.print(mpu.getPitch(), 2);
      Serial.print(", ");
      Serial.println(mpu.getRoll(), 2);
    } else if (in123 == 'r') {
      for (int j = 0; j < 3; j++) {
        for (int i = 0; i < 2; i++) {
          Pos[j][i] = 0;
          Vel[j][i] = 0;
        }
      }
      in123 = 'p';
    } else if (in123 == 'm') {
      Serial.print("x, y, z: ");
      Serial.print(mpu.getMagX(), 2);
      Serial.print(", ");
      Serial.print(mpu.getMagY(), 2);
      Serial.print(", ");
      Serial.println(mpu.getMagZ(), 2);
      //delay(200);
    }else if(in123 == 'u'){
      thread_G +=0.001;
      in123 = 'l';
    }else if(in123 == 'd'){
      thread_G -=0.001;
      in123 = 'l';
    }else{
      Serial.print("thread_G= ");
      Serial.println(thread_G, 5);
    }
    prev_ms_print = millis();
  }
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
