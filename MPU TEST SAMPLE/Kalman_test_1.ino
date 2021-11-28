/* This ino code performs the MPU9250 IMU to predict the position
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
#include "MPU9250.h"
#include "BasicLinearAlgebra.h"
#include <TinyGPS++.h>

using namespace BLA;
MPU9250 mpu;
int sample_period = 50; //in ms
float T_ = sample_period / 1000; //in Sec
int Freq_acc = 1000 / sample_period; //40Hz
char in123 = 'a';         // char for input debug
char incomingByte = 'a';
int count = 0;            //Count for drift velocity elimination

// A sample NMEA stream.
const char *gpsStream =
  "$GPRMC,045103.000,A,3014.1984,N,09749.2872,W,0.67,161.46,030913,,,A*7C\r\n"
  "$GPGGA,045104.000,3014.1985,N,09749.2873,W,1,09,1.2,211.6,M,-22.5,M,,0000*62\r\n"
  "$GPRMC,045200.000,A,3014.3820,N,09748.9514,W,36.88,65.02,030913,,,A*77\r\n"
  "$GPGGA,045201.000,3014.3864,N,09748.9411,W,1,10,1.2,200.8,M,-22.5,M,,0000*6C\r\n"
  "$GPRMC,045251.000,A,3014.4275,N,09749.0626,W,0.51,217.94,030913,,,A*7D\r\n"
  "$GPGGA,045252.000,3014.4273,N,09749.0628,W,1,09,1.3,206.9,M,-22.5,M,,0000*6F\r\n";

// The TinyGPS++ object
TinyGPSPlus gps;

//Define global variable
BLA::Matrix<3> acc = {0, 0, 0};
float Acc_mag[2] = {0, 0};
float Acc[3][2] = {{}, {}};
float Vel[3][20]; //Vel( 3 raws, 2 cols   )
float Pos[3][2] = {{}, {}};
float Acc_bias[3] = {0, 0, 0};
float Yaw[2] = {0, 0};
float thread_G = 0.01;
int stationary = 1;
int count_S = 0, count_v = 0;

float Lat, Lng, Lat_o, Lng_o;

BLA::Matrix<3> U_INS = {0, 0, 0};
BLA::Matrix<3> Y_INS = {0, 0, 0};
BLA::Matrix<5> X_INS = {0, 0, 0, 0, 0};
BLA::Matrix<3> N_INS = {0, 0, 0};
BLA::Matrix<3> Y_GPS = {0, 0, 0};
BLA::Matrix<3> N_GPS = {0, 0, 0};
BLA::Matrix<3> Bias = {0, 0, 0};
BLA::Matrix<3> Bias_Predic = {0, 0, 0};
BLA::Matrix<3> U_E = {0, 0, 0};
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
BLA::Matrix<2, 2> C_;
float beta;

BLA::Matrix<8, 8> big_zero;
BLA::Matrix<8, 8> big_I;
BLA::Matrix<8, 8, Diagonal<8, float>> big_diag;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  delay(1000);

  //Detect if MPU setup correctly
  if (!mpu.setup(0x68)) {  // change to your own address
    while (1) {
      Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
      delay(1000);
    }
  } else Serial << "MPU is connected. \n";

  // MPU calibrate anytime you want to
  Serial.println("Accel Gyro calibration will start in 5sec.");
  Serial.println("Please leave the device still on the flat plane.");
  mpu.verbose(true);
  delay(5000);
  mpu.calibrateAccelGyro();
  mpu.setMagBias(-89.91, 446.60, -375.83);  //Set by Observation
  mpu.verbose(false);

  //Initial Kalman and GPS
  Initial_Kalman(); //Initial parameters
  while (*gpsStream)
    if (gps.encode(*gpsStream++))
      update_ref_location(); //Set first reference location

}
void loop() {
  if (mpu.update()) {
    static uint32_t prev_ms = millis();
    if (millis() > prev_ms + sample_period) {
      Serial << "-------------------------------- \n";
      
      //Get GPS data
      while (*gpsStream)
        if (gps.encode(*gpsStream++))
          measure_gps_data();
      
      //Run Kalman with fusion IMU and GPS
      
      Update_Kalman();
      Serial << "K_Px: " << X_INS(2) << " K_Py: " << X_INS(3)
             << " K_Vx: " << X_INS(0) << " K_Vy: " << X_INS(1)
             << " K_Yew: " << X_INS(5) << "\n";
      if (mpu.update()) measure_imu_data();
      //Ouput current location in Lat and Lng
      get_current_location();
      Serial << "Lat: " << Lat << " Lng: " << Lng << "\n";
      
#ifdef DEBUG
      Serial << " Acc: " << acc << " Yew[0]: " << float(mpu.getYaw() / 180.f * PI)
             << " pitch: " << float(mpu.getPitch() / 180.f * PI)
             << " Roll: " << float(mpu.getRoll() / 180.f * PI) << " \n ";
#endif
      Serial << "-------------------------------- \n";
      prev_ms = millis();
    }
  }

}

void Initial_Kalman() {
  //Form matrix reference.
  big_zero.Fill(0);
  big_I.Fill(1);
  big_diag.Fill(1);

  //Setup initial Guess:
  beta = 0.005;
  float P_0_[8] = {10, 10, 10, 10, 90 * PI / 180, 5, 5, 25 * PI / 180};
  float R_[3] = {1, 1, PI / 180};
  float Q_[3] = {1, 1, 0.1 * PI / 180};
  for (int i = 0; i < 8; i++) P_0(i, i) =  P_0_[i];
  for (int i = 0; i < 3; i++) R(i, i) =  R_[i];
  for (int i = 0; i < 3; i++) Q(i, i) =  Q_[i];

  P.Fill(0.1);    //Initial The cov of error.
  H.Fill(0);
  //form: H = {0,0,1,1,1,0,0,0, 0,0,1,1,1,0,0,0, 0,0,1,1,1,0,0,0};
  H = (big_zero.Submatrix<3, 2>(1, 1) || big_I.Submatrix<3, 3>(1, 1) || big_zero.Submatrix<3, 3>(1, 1));
  X_E_Predic.Fill(0);

  //Para matrix for IMU(U) and GPS(Y)
  C_ = {cos(X_INS(5)), sin(X_INS(5)), cos(X_INS(5)), -sin(X_INS(5))};
  A_E = (big_zero.Submatrix<2, 2>(0, 0) && big_I.Submatrix<2, 2>(0, 0) && big_zero.Submatrix<4, 2>(0, 0)) ||
        big_zero.Submatrix<8, 3>(0, 0) ||
        (C_ && big_zero.Submatrix<6, 2>(0, 0)) ||
        (big_zero.Submatrix<4, 1>(0, 0) && big_I.Submatrix<1, 1>(0, 0) && big_zero.Submatrix<3, 1>(0, 0));
  B_E = (C_ && big_zero.Submatrix<6, 2>(0, 0)) ||
        (big_zero.Submatrix<4, 1>(0, 0) && big_I.Submatrix<1, 1>(0, 0) && big_zero.Submatrix<3, 1>(0, 0));
#ifdef DEBUG_MODE
  Serial << "Initial Kal: \n"
         << "P_0" << P_0 << "\n"
         << "R" << R << "\n"
         << "Q" << Q << "\n"
         << "H" << H << "\n"
         << "X_E_Predic" << X_E_Predic << "\n"
         << "C_" << C_ << "\n"
         << "A_E" << A_E << "\n"
         << "B_E" << B_E << "\n";
#endif
}

void Update_Kalman() {
  //BLA::Matrix<3> X_35 = {X_INS(3), X_INS(4), X_INS(5)};
  //Y_E = X_35 - Y_GPS;

  //Update the kalman para
  for (int i = 0; i < 3; i++ )Y_E(i) = X_INS(2 + i) - Y_GPS(i);
  BLA::Matrix<3, 3> dom = H * P * ~H + R;
  BLA::Matrix<3, 3> dom1 = Invert(dom);
  G_k = P * ~H * dom1;
  BLA::Matrix<8, 8> I_8;
  I_8.Fill(1);
  P = (I_8 - G_k * H) * P;
  BLA::Matrix<8>Bias_0; Bias_0.Fill(0); Bias_0.Submatrix<1, 5>(0, 0);
  X_E_Predic = Bias_0 + G_k * Y_E;

  X_INS -= X_E_Predic.Submatrix<5, 1>(0, 0);
  Bias_Predic = X_E_Predic.Submatrix<3, 1>(5, 0);

  //Update from input
  BLA::Matrix<3> U_pre = {acc(0) * 9.81, acc(1) * 9.81, (Yaw[0] - Yaw[1]) / T_};
  U_INS = U_pre - Bias_Predic;

  //Optimized output X and cov matrix.
  X_INS = {X_INS(0) + T_ * U_INS(0),
           X_INS(1) + T_ * U_INS(1),
           1 / 2 * T_ * X_INS(0) + X_INS(2),
           1 / 2 * T_ * X_INS(1) + X_INS(3),
           X_INS(4) + T_ * U_INS(3)
          };
  P = A_E * P * ~A_E + B_E * Q * ~B_E + P_0 * beta;

/*
 * X = {[V_x],
 *      [V_y],
 *      [P_x],
 *      [P_y],
 *      [Yaw]}
 */
#ifdef DEBUG_MODE
  Serial << "Y_E: " << Y_E << "\n"
         << "H: "   << H   << "\n"
         << "G_k: " << G_k << "\n"
         << "X_E_Predic: " << X_E_Predic << "\n"
         << "Bias_Predic: " << Bias_Predic << "\n"
         << "U_INS: " << U_INS << "\n"
         << "X_INS: " << X_INS << "\n"
         << "P: " <<  P << "\n";
#endif
  //#ifdef DEBUG_MODE
  Serial  << "U_pre: " << U_pre
          << "U_INS: " << U_INS << "\n";
  //#endif


}

//Record GPS data
void measure_gps_data() {
  if (gps.location.isValid()) {
    //Get GPS measure in meter from ref point.
    Y_GPS = {get_diff_dist(Lat_o, gps.location.lat()),
             get_diff_dist(Lng_o, gps.location.lng()),
             mpu.getYaw() / 180.f * PI
            };
  } else Serial.println(F("Location: INVALID"));
#ifdef DEBUG_MODE
  Serial << "Lat: " <<  Y_GPS(0)
         << " Lon: " << Y_GPS(1)
         << " angle: " << Y_GPS(2) << "\n";
#endif
#ifdef DEBUG_MODE
  Serial << "Lat: " <<  gps.location.lat()
         << " Lon: " << gps.location.lng() << "\n";
#endif
}

//Process Acceleration data to earth frame
void measure_imu_data() {
  float acc_x___ = mpu.getAccX() / 1.02; //get from mpu
  float acc_y___ = mpu.getAccY();
  float acc_z___ = mpu.getAccZ() / 1.045;
  float acc_the = mpu.getRoll() / 180.f * PI;
  float acc_fin = mpu.getPitch() / 180.f * PI;
  float acc_psi = mpu.getYaw() / 180.f * PI;

  //Set Max Acc
  if (acc_x___ >= 2) acc_x___ = 2;
  if (acc_y___ >= 2) acc_y___ = 2;
  if (acc_z___ >= 2) acc_z___ = 2;

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
#ifdef DEBUG
  Serial << " Acc: " << acc << " Yew[0]: " << float(mpu.getYaw() / 180.f * PI)
         << " pitch: " << float(mpu.getPitch() / 180.f * PI)
         << " Roll: " << float(mpu.getRoll() / 180.f * PI) << " \n ";
#endif
}

void update_ref_location() {
  if (gps.location.isValid()) {
    Lat_o = gps.location.lat();
    Lng_o = gps.location.lng();
  }
}

float get_diff_dist(float oringe, float update_) {
  float dist = 6372795 * PI / 180 * (update_ - oringe);
  return dist;
}

void get_current_location() {
  Lat = X_INS(2) * 180 / (6372795 * PI) + Lat_o;
  Lng = X_INS(3) * 180 / (6372795 * PI) + Lng_o;
}
