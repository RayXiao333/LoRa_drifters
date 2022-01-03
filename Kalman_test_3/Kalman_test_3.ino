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

#define DEBUG_MODE 1
#define Calibration_imu 1

#ifdef Calibration_imu
#include <EEPROM.h>
float lms_mb[6] = {1, 0, 1, 0, 1, 0};
#define ACC_cali_para_addr 0
#define Mag_cali_para_addr 25
/***  do not modify  ***********************************************/
template< typename T, size_t NumberOfSize >
size_t MenuItemsSize(T (&) [NumberOfSize]) {
  return NumberOfSize;
}
#endif

using namespace BLA;
MPU9250 mpu;
int sample_period = 100; //in ms
float T_ = 0.1; //in Sec
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
//  mpu.setMagBias(-89.91, 446.60, -375.83);  //Set by Observation
  mpu.verbose(false);

#ifdef Calibration_imu
  EEPROM.begin(50);
  read_imu_cali_para(ACC_cali_para_addr, lms_mb, MenuItemsSize(lms_mb));
  float mag_bias_[3]={};
  read_imu_cali_para(Mag_cali_para_addr, mag_bias_, MenuItemsSize(mag_bias_));
  mpu.setMagBias(mag_bias_[0], mag_bias_[1], mag_bias_[2]);  //Set by Observation
#endif

  //Initial Kalman and GPS
  Initial_Kalman(); //Initial parameters
  while (*gpsStream)
    if (gps.encode(*gpsStream++))
      update_ref_location(); //Set first reference location
}

int reset__ = 0;
void loop() {
  if (mpu.update()) {
    static uint32_t prev_ms = millis();
    static uint32_t imu_ms = millis(); //For reset
    if (millis() > prev_ms + sample_period) {
      //Serial << "-------------------------------- \n";
      if ((millis() > imu_ms + 20000) & (reset__ != 1)) {
        update_ref_location(); //Rest data when 50sec.
        imu_ms = millis();
        reset__ = 1;
      }
      //Get GPS data
      while (*gpsStream)
        if (gps.encode(*gpsStream++))
          measure_gps_data();

      //Run Kalman with fusion IMU and GPS
      Update_Kalman();
      if (mpu.update()) measure_imu_data();
#ifdef Calibration_imu
      read_Serial_input();
      //check_stable_imu();
#endif
      //Ouput current location in Lat and Lng
      get_current_location();
      Serial << "Px: " << X_INS(2) << " Py: " << X_INS(3)
             << " Vx:" << X_INS(0) << " Vy: " << X_INS(1)
             //<< " Lat: " << Lat << " Lng: " << Lng
             << " ||Y: " << Y_GPS << "\n";
#ifdef DEBUG
      Serial << " Acc: " << acc << " Yew[0]: " << float(mpu.getYaw() / 180.f * PI)
             << " pitch: " << float(mpu.getPitch() / 180.f * PI)
             << " Roll: " << float(mpu.getRoll() / 180.f * PI) << " \n ";
#endif
      //Serial << "-------------------------------- \n";
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
  float R_[3] = {0.5, 0.5, 0.5 * PI / 180};
  float Q_[3] = {0.1, 0.1, 0.1 * PI / 180};
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
#ifdef DEBUG_MODE
  Serial << "Y_E: " << Y_E << "\n"
         << "H: "   << H   << "\n"
         << "G_k: " << G_k << "\n"
         << "X_E_Predic: " << X_E_Predic << "\n"
         << "Bias_Predic: " << Bias_Predic << "\n"
         << "U_pre: [" << U_pre(0) << "; " << U_pre(1) << "; " << U_pre(2) << "] "
         << "U_INS: " << U_INS << "\n"
         << "Y_GPS: " << Y_GPS << "\n"
         << "X_INS: " << X_INS << "\n"
         << "P: " <<  P << "\n"
         << "Yaw" << Yaw[0] << " C_" << C_ << "\n"
         << "----------------------------------------------------\n";
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
  acc(2) = acc_z_;
  Yaw[1] = Yaw[0];
  Yaw[0] = acc_psi;
#ifdef Calibration_imu
  for (int i = 0; i < 3; i++) acc(i) = lms_mb[2 * i] * acc(i) + lms_mb[2 * i + 1];
#endif Calibration_imu
#ifdef DEBUG
  Serial << " Acc: " << acc << " Yew[0]: " << float(mpu.getYaw() / 180.f * PI)
         << " pitch: " << float(mpu.getPitch() / 180.f * PI)
         << " Roll: " << float(mpu.getRoll() / 180.f * PI) << " \n ";
#endif
}

//Record GPS data
void measure_gps_data() {
  if (gps.location.isValid()) {
    //Get GPS measure in meter from ref point.
    Y_GPS = {get_diff_dist(Lat_o, gps.location.lat()),
             get_diff_dist(Lng_o, gps.location.lng()),
             Yaw[0]
            };
    Y_GPS = {gps.location.lat(),
             gps.location.lng(),
             Yaw[0]
            };
  } else Serial.println(F("Location: INVALID"));
#ifdef DEBUG_MODE
  Serial << "Lat: " <<  Y_GPS(0)
         << " Lon: " << Y_GPS(1)
         << " angle: " << Y_GPS(2) << "\n";
#endif
  //#ifdef DEBUG_MODE
  //  Serial << "Lat: " <<  gps.location.lat()
  //         << " Lon: " << gps.location.lng() << "\n";
  //#endif
}



void update_ref_location() {
  if (gps.location.isValid()) {
    Lat_o = gps.location.lat();
    Lng_o = gps.location.lng();
  }
  X_INS.Fill(0);
}

float get_diff_dist(float oringe, float update_) {
  float dist = 6372795 * PI / 180 * (update_ - oringe);
  return dist;
}

void get_current_location() {
  Lat = X_INS(2) * 180 / (6372795 * PI) + Lat_o;
  Lng = X_INS(3) * 180 / (6372795 * PI) + Lng_o;
}

/*-----------------------------Calibration functions---------------------*/
#ifdef Calibration_imu
void LMS_para(float x[], int n, float *m_, float *b_) {
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
    *m_ = 1;
    *b_ = 0;
  } else {
    *m_ = (n * sumxy - sumx * sumy) / denom;
    *b_ = (sumy * sumx2 - sumx * sumxy) / denom;
    Serial << "Solution get, m= " << *m_ << " b= " << *b_ << '\n' ;
  }
}

float get_sqre(BLA::Matrix<3> x, int n) {
  float sqar = 0;
  for (int i = 0; i < n; i++) {
    sqar += x(i) * x(i);
  }
  return sqrt(sqar);
}

void calibration_mag() {
  Serial.println("Mag calibration will start in 5sec.");
  Serial.println("Please Wave device in a figure eight until done.");
  delay(5000);
  mpu.calibrateMag();
}

void calibration_imu() {
  int n = 50;
  //  float accx[6 * n] = {};
  //  float accy[6 * n] = {};
  //  float accz[6 * n] = {};
  float acc_cal_temp[3][6 * n] = {{}, {}, {}};
  float m_;
  float b_;
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
        acc_cal_temp[0][i] = mpu.getAccX();
        acc_cal_temp[1][i] = mpu.getAccY();
        acc_cal_temp[2][i] = mpu.getAccZ();
        Serial << "Count: " << i
               << " x: " << acc_cal_temp[0][i]
               << " y: " << acc_cal_temp[1][i]
               << " z: " << acc_cal_temp[2][i]
               << '\n';
        delay(50);
      }
    }
    Serial << "Flip IMU to next axis\n";
  }

  Serial << "get ACC data done \n";
  //Write into matrix, contains m & b for x, y, z;
  for (int i = 0; i < 3; i++) {
    float acc_temp[6 * n] = {};
    for (int j = 0; j < 6 * n; j++) acc_temp[j] = acc_cal_temp[i][j];
    LMS_para(acc_temp, 6 * n, &lms_mb[i * 2], &lms_mb[i * 2 + 1]);
  }
  //Write data into Flash
  //EEPROM.begin(24);
  int count_lms = 0;
  write_imu_cali_para(ACC_cali_para_addr, lms_mb, MenuItemsSize(lms_mb));
  Serial.println("Store acc para done.");
  Serial << "para:";
  for (int i = 0; i < 3; i++) {
    Serial << " m" << i + 1 << ": ";
    Serial.print(lms_mb[i * 2], 5);
    Serial << " b" << i + 2 << ": ";
    Serial.print(lms_mb[i * 2 + 1], 5);
  }
  Serial << '\n';
  calibration_mag();
  float mag_bias_[3]={mpu.getMagBias(0), mpu.getMagBias(1), mpu.getMagBias(2)};
  write_imu_cali_para(Mag_cali_para_addr, mag_bias_, MenuItemsSize(mag_bias_));
  Serial.println("Stored all data into flash.");
}

void read_imu_cali_para(int address, float *data, int size) {
  for (int i = 0; i < size * sizeof(float); i += sizeof(float)) {
    data[i / sizeof(float) + address] = EEPROM.readFloat(i);
    Serial.println(EEPROM.readFloat(i), 6);
  }
}

void write_imu_cali_para(int address, float data[], int size) {
  for (int i = 0; i < (size * sizeof(float)); i += sizeof(float)) {
    EEPROM.writeFloat(i + address, data[i / sizeof(float)]);
    //Serial.println(lms_mb[i]);
    Serial.println(EEPROM.readFloat(i));
  }
  EEPROM.commit();
}
int print_;
void read_Serial_input() {
  if (Serial.available()) {
    const char c = Serial.read();
    delay(200);

    if (c == 'c') {
      Serial << "Run Calibration: ----------------------\n";
      //calibration[0] = 1;
      calibration_imu();
    } else if (c == 'r') {
      Serial << "Reset position\n";
      for (int i = 0; i < 4; i++) X_INS(i) = 0;
    } else if (c == 'd') {
#ifndef DEBUG_MODE
#define DEBUG_MODE 1
#endif
#ifdef DEBUG_MODE
#undef DEBUG_MODE
#endif
    } else if (c == 'a' | c == 'v' | c == 'p' | c == 'o' | c == 'A' | c == 'V' | c == 'P')print_ = c;

  }
  print_data(print_);
}
float fix[20] = {};
float error = 0.01;
void check_stable_imu() {
  fix[0] = abs(get_sqre(acc, 3) - 1);
  for (int i = 0; i < 20; i++) fix[i + 1] = fix[i];
  int stationary = 0;
  for (int i = 0; i < 15; i++) {
    if (fix[i] <= error) stationary += 1;
  }
  if (stationary >= 15) {
    for (int i = 0; i < 2; i++) X_INS(i);
    for (int i = 0; i < 20; i++) fix[i] = 1;
  }
}


void print_data(char print_) {
  if (print_ == 'a') {
    Serial << "Ax: " << acc(0)
           << " Ay: " << acc(1)
           << " Az_e: " << acc(2) << " ";
    Serial.println(get_sqre(acc, 3), 6);
  } else if (print_ == 'v') {
    Serial << "Vx: " << X_INS(0)
           << " Vy: " << X_INS(1)
           //<< " Vz: " << v[2]
           << '\n';
  } else if (print_ == 'p') {
    Serial << "Px: " << X_INS(2)
           << " Py: " << X_INS(3)
           //<< " Pz: " << p[2]
           << '\n';
  }
}
#endif Calibration_imu
