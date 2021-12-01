/* This code performs the MPU9250 IMU to predict the position
   The system is runing at 40Hz, the front Y-axis is pointing to West (Yew = 0 degree)
   MPU library: https://github.com/hideakitai/MPU9250
                Another library: Wire.h
   Port connection: MPU9250 --- ESP32
                      VCC   --- 3V3
                      GND   --- GND
                      SCL   --- 22
                      SDA   --- 21
   Debug use: by send defferent char by serial to esp32, it will output current states:
              "a" :Raw acceleration data, unit: G
              "A" :Processed acceleration data on Earth Frame, unit: m/s2
              "R" :Yew, Pitch, Roll data(Y axis point to West), unit: degree
              "v" :Return Velocity data, unit: m/s
              "p" :Return Position data, unit: cm
              "m" :Return Magnetometer data, unit: µTesla
              "u" :Increase threshold value for stationary by 0.001
              "d" :Increase threshold value for stationary by 0.001
              "r" :Rest position data to oringe(xyz=0,0,0).
*/
#ifndef IMU_H
#define IMU_H

#include "MPU9250.h"
#include "BasicLinearAlgebra.h"
#include <TinyGPS++.h>

using namespace BLA;
MPU9250 mpu;
int sample_period = 50; //in ms
float T_ = sample_period / 1000; //in Sec
// int Freq_acc = 1000 / sample_period; //40Hz
// char in123 = 'a';         // char for input debug
// char incomingByte = 'a';
int count = 0;            //Count for drift velocity elimination

// The TinyGPS++ object
extern TinyGPSPlus gps;

//Define global variables
BLA::Matrix<3> acc = {0.f, 0.f, 0.f};
// float Acc_mag[2] = {0.f, 0.f};
// float Acc[3][2] = {{}, {}};
// float Vel[3][2];
// float Pos[3][2] = {{}, {}};
// float Acc_bias[3] = {0.f, 0.f, 0.f};
float Yaw[2] = {0.f, 0.f};
// float thread_G = 0.01f;
// int stationary = 1;
// int count_S = 0, count_v = 0;

float Lat_o, Lng_o;

BLA::Matrix<3> U_INS = {0, 0, 0};
// BLA::Matrix<3> Y_INS = {0, 0, 0};
BLA::Matrix<5> X_INS = {0, 0, 0, 0, 0};
// BLA::Matrix<3> N_INS = {0, 0, 0};
BLA::Matrix<3> Y_GPS = {0, 0, 0};
// BLA::Matrix<3> N_GPS = {0, 0, 0};
// BLA::Matrix<3> Bias = {0, 0, 0};
BLA::Matrix<3> Bias_Predic = {0, 0, 0};
// BLA::Matrix<3> U_E = {0, 0, 0};
BLA::Matrix<3> Y_E = {0, 0, 0};
BLA::Matrix<8> X_E_Predic;
BLA::Matrix<8, 8> P;
BLA::Matrix<8, 8, Diagonal<8, float>> P_0;
BLA::Matrix<8, 3> G_k;
BLA::Matrix<3, 8> H;
BLA::Matrix<3, 3, Diagonal<3, float>> R;
BLA::Matrix<3, 3, Diagonal<3, float>> Q;
BLA::Matrix<8, 8> A_E;
BLA::Matrix<8, 3> B_E;
#define BETA 0.005

void Initial_Kalman() {
  //Form matrix reference.
  big_zero.Fill(0);
  big_I.Fill(1);
  big_diag.Fill(1);

  //Setup initial Guess:
  beta = 0.005;
  float P_0_[8] = {10, 10, 10, 10, 90 * PI / 180, 5, 5, 25 * PI / 180};
  float R_[3] = {0.1, 0.1, 0.1 * PI / 180};
  float Q_[3] = {1, 1, 1 * PI / 180};
  for (int i = 0; i < 8; i++) P_0(i, i) =  P_0_[i];
  for (int i = 0; i < 3; i++) R(i, i) =  R_[i];
  for (int i = 0; i < 3; i++) Q(i, i) =  Q_[i];
  Bias_Predic = {0, 0, 0};

  P = P_0;    //Initial The cov of error.
  H.Fill(0);
  //form: H = {0,0,1,1,1,0,0,0, 0,0,1,1,1,0,0,0, 0,0,1,1,1,0,0,0};
  H = (big_zero.Submatrix<3, 2>(1, 1) || big_I.Submatrix<3, 3>(1, 1) || big_zero.Submatrix<3, 3>(1, 1));
  X_E_Predic.Fill(0);

  //Para matrix for IMU(U) and GPS(Y)
  Y_E.Fill(0);
  G_k.Fill(0);
  X_E_Predic.Fill(0);
  U_INS.Fill(0);
  X_INS.Fill(0);
  Y_GPS.Fill(0);

#ifdef DEBUG_MODE
  Serial << "Initial Kal: \n"
         << "P_0" << P_0 << "\n"
         << "R" << R << "\n"
         << "Q" << Q << "\n"
         << "H" << H << "\n"
         << "X_E_Predic" << X_E_Predic << "\n"

         << "A_E" << A_E << "\n"
         << "B_E" << B_E << "\n";
#endif
}

void Update_Kalman() {
  //Parameters update
  C_ = {cos(X_INS(4)), sin(X_INS(4)), cos(X_INS(4)), -sin(X_INS(4))};
  A_E = (big_zero.Submatrix<2, 2>(0, 0) && big_I.Submatrix<2, 2>(0, 0) && big_zero.Submatrix<4, 2>(0, 0)) ||
        big_zero.Submatrix<8, 3>(0, 0) ||
        (C_ && big_zero.Submatrix<6, 2>(0, 0)) ||
        (big_zero.Submatrix<4, 1>(0, 0) && big_I.Submatrix<1, 1>(0, 0) && big_zero.Submatrix<3, 1>(0, 0));
  B_E = (C_ && big_zero.Submatrix<6, 2>(0, 0)) ||
        (big_zero.Submatrix<4, 1>(0, 0) && big_I.Submatrix<1, 1>(0, 0) && big_zero.Submatrix<3, 1>(0, 0));


  //Optimized output X and cov matrix.
  //Update from input
  BLA::Matrix<3> U_pre = {acc(0) * 9.81, acc(1) * 9.81, (Yaw[0] - Yaw[1]) / T_};
  U_INS = U_pre - Bias_Predic;
  X_INS = {X_INS(0) + T_ * U_INS(0),
           X_INS(1) + T_ * U_INS(1),
           0.5 * T_ * X_INS(0) + X_INS(2),
           0.5 * T_ * X_INS(1) + X_INS(3),
           X_INS(4) + T_ * U_INS(2)
          };
  P = A_E * P * ~A_E + B_E * Q * ~B_E + P_0 * beta;

  //Update the kalman para
  for (int i = 0; i < 3; i++ )Y_E(i) = X_INS(2 + i) - Y_GPS(i);
  BLA::Matrix<3, 3> dom = H * P * ~H + R;
  BLA::Matrix<3, 3> dom1 = Invert(dom);
  G_k = P * ~H * dom1;
  P = (big_diag - G_k * H) * P;
  BLA::Matrix<8>Bias_0; Bias_0.Fill(0);
  for (int i = 0; i < 3; i++) Bias_0(i + 5) = Bias_Predic(i);
  X_E_Predic = Bias_0 + G_k * Y_E;

  X_INS -= X_E_Predic.Submatrix<5, 1>(0, 0);
  Bias_Predic = X_E_Predic.Submatrix<3, 1>(5, 0);
//  #ifdef DEBUG_MODE
//  Serial << "Y_E: " << Y_E << "\n"
//         << "H: "   << H   << "\n"
//         << "G_k: " << G_k << "\n"
//         << "X_E_Predic: " << X_E_Predic << "\n"
//         << "Bias_Predic: " << Bias_Predic << "\n"
//         << "U_pre: [" << U_pre(0) << "; " << U_pre(1) << "; " << U_pre(2) << "] "
//         << "U_INS: " << U_INS << "\n"
//         << "Y_GPS: " << Y_GPS << "\n"
//         << "X_INS: " << X_INS << "\n"
//         << "P: " <<  P << "\n"
//         << "Yaw" << Yaw[0] << " C_" << C_ << "\n"
//         << "----------------------------------------------------\n";
//  #endif
}

static void update_ref_location() {
  if(gps.location.isValid()) {
    Lat_o = gps.location.lat();
    Lng_o = gps.location.lng();
  }
}

float get_diff_dist(const float oringe, const float update_) {
  return 6372795 * PI / 180 * (update_ - oringe);
}

void get_current_location(float & lat, float & lng) {
  lat = X_INS(2) * 180 / (6372795 * PI) + Lat_o;
  lng = X_INS(3) * 180 / (6372795 * PI) + Lng_o;
}

//Record GPS data
void measure_gps_data() {
  if(gps.location.isValid()) {
    //Get GPS measure in meter from ref point.
    Y_GPS = {get_diff_dist(Lat_o, gps.location.lat()),
             get_diff_dist(Lng_o, gps.location.lng()),
             mpu.getYaw() / 180.f * PI};
  } else Serial.println(F("Location: INVALID"));
#ifdef DEBUG_MODE
  Serial << "Lat: " <<  Y_GPS(0)
         << " Lon: " << Y_GPS(1)
         << " angle: " << Y_GPS(2) << "\n";
  // Serial << "Lat: " <<  gps.location.lat()
  //        << " Lon: " << gps.location.lng() << "\n";
#endif
}

//Process Acceleration data to earth frame
void measure_imu_data() {
  float acc_x___ = mpu.getAccX() / 1.02; //get from mpu
  float acc_y___ = mpu.getAccY();
  float acc_z___ = mpu.getAccZ() / 1.045;
  const float acc_the = mpu.getRoll() / 180.f * PI;
  const float acc_fin = mpu.getPitch() / 180.f * PI;
  const float acc_psi = mpu.getYaw() / 180.f * PI;

  //Set Max Acc
  if(acc_x___ >= 2) acc_x___ = 2;
  if(acc_y___ >= 2) acc_y___ = 2;
  if(acc_z___ >= 2) acc_z___ = 2;

  //rotate from x-axis
  float acc_y__ = acc_y___ * cos(acc_the) - acc_z___ * sin(acc_the);
  float acc_z__ = acc_z___ * cos(acc_the) + acc_y___ * sin(acc_the);
  //rotate from y-axis
  float acc_x_ = acc_x___ * cos(acc_fin) - acc_z__ * sin(acc_fin);
  float acc_z_ = acc_z__ * cos(acc_fin) + acc_x___ * sin(acc_fin);
  //rotate from z-axis
  acc(0) = acc_x_ * cos(acc_psi) + acc_y__ * sin(acc_psi);
  acc(1) = acc_y__ * cos(acc_psi) - acc_x_ * sin(acc_psi);
  acc(3) = acc_z_;
  Yaw[1] = Yaw[0];
  Yaw[0] = acc_psi;
#ifdef DEBUG_MODE
  Serial << " Acc: " << acc << " Yew[0]: " << float(mpu.getYaw() / 180.f * PI)
         << " pitch: " << float(mpu.getPitch() / 180.f * PI)
         << " Roll: " << float(mpu.getRoll() / 180.f * PI) << " \n ";
#endif
}

void initIMU() {
  Wire.begin();
  delay(1000);

  //Detect if MPU setup correctly
  if(!mpu.setup(0x68)) {  // change to your own address
    while(1) {
      Serial << "MPU connection failed. Please check your connection with `connection_check` example.\n";
      delay(1000);
// #undef USING_IMU
//       break;
    }
  } else Serial << "MPU is connected. \n";

  // MPU calibrate anytime you want to
  Serial << "Accel Gyro calibration will start in 5sec.\n";
  Serial << "Please leave the device still on the flat plane.\n";
  mpu.verbose(true);
  delay(5000);
  mpu.calibrateAccelGyro();
  mpu.setMagBias(-89.91, 446.60, -375.83);  //Set by Observation
  mpu.verbose(false);

  //Initial Kalman and GPS
  Initial_Kalman(); //Initial parameters

  // A sample NMEA stream.
  const char *gpsStream =
    "$GPRMC,045103.000,A,3014.1984,N,09749.2872,W,0.67,161.46,030913,,,A*7C\r\n"
    "$GPGGA,045104.000,3014.1985,N,09749.2873,W,1,09,1.2,211.6,M,-22.5,M,,0000*62\r\n"
    "$GPRMC,045200.000,A,3014.3820,N,09748.9514,W,36.88,65.02,030913,,,A*77\r\n"
    "$GPGGA,045201.000,3014.3864,N,09748.9411,W,1,10,1.2,200.8,M,-22.5,M,,0000*6C\r\n"
    "$GPRMC,045251.000,A,3014.4275,N,09749.0626,W,0.51,217.94,030913,,,A*7D\r\n"
    "$GPGGA,045252.000,3014.4273,N,09749.0628,W,1,09,1.3,206.9,M,-22.5,M,,0000*6F\r\n";

  while(*gpsStream) {
    if(gps.encode(*gpsStream++))
      update_ref_location(); //Set first reference location
  }
}

#endif //IMU_H
