/***************************************************************************
  Example sketch for the MPU9250_WE library

  This sketch shows how to get acceleration, gyroscocope, magnetometer and
  temperature data from the MPU9250. It might be a bit confusing with all the
  settings. Therefore you will also find sketches for the individual data.

  For further information visit my blog:

  https://wolles-elektronikkiste.de/mpu9250-9-achsen-sensormodul-teil-1  (German)
  https://wolles-elektronikkiste.de/en/mpu9250-9-axis-sensor-module-part-1  (English)

***************************************************************************/

#include "src/MPU9250_WE.h"
#include <Wire.h>
#include "src/QuaternionFilter.h"

/* There are several ways to create your MPU9250 object:
   MPU9250_WE myMPU9250 = MPU9250_WE()              -> uses Wire / I2C Address = 0x68
   MPU9250_WE myMPU9250 = MPU9250_WE(MPU9250_ADDR)  -> uses Wire / MPU9250_ADDR
   MPU9250_WE myMPU9250 = MPU9250_WE(&wire2)        -> uses the TwoWire object wire2 / MPU9250_ADDR
   MPU9250_WE myMPU9250 = MPU9250_WE(&wire2, MPU9250_ADDR) -> all together
   Successfully tested with two I2C busses on an ESP32
*/

//MPU9250_WE myMPU9250 = MPU9250_WE(MPU9250_ADDR);
TwoWire wire1 = TwoWire(1);
#define I2C_FREQ 400000
#define SDA_PIN 21
#define SCL_PIN 22
#define MPU9250_ADDR 0x68

MPU9250_WE myMPU9250 = MPU9250_WE(&wire1, MPU9250_ADDR);
QuaternionFilter quat_filter;
size_t n_filter_iter {1};
int sample_period = 10; //in ms
char in123 = 'm';         // char for input debug
char incomingByte = 'm';

//Define global variable
xyzFloat gValue;
xyzFloat gyr;
xyzFloat magValue;
xyzFloat angles;
float temp;
float resultantG;
float pitch;
float roll;
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};  // vector to hold quaternion
float rpy[3] {0.f, 0.f, 0.f};
float acc[3] = {0, 0, 0};
float Acc[3] = {};
float Vel[3][2] = {{}, {}}; //Vel( 3 raws, 2 cols   )
float Pos[3][2] = {{}, {}};
int stationary = 0, count = 0;


void setup() {
  Serial.begin(115200);
  wire1.begin(SDA_PIN, SCL_PIN, I2C_FREQ); // SDA, SCL
  if (!myMPU9250.init()) {
    Serial.println("MPU9250 does not respond");
  }
  else {
    Serial.println("MPU9250 is connected");
  }
  if (!myMPU9250.initMagnetometer()) {
    Serial.println("Magnetometer does not respond");
  }
  else {
    Serial.println("Magnetometer is connected");
  }

  /* The slope of the curve of acceleration vs measured values fits quite well to the theoretical
     values, e.g. 16384 units/g in the +/- 2g range. But the starting point, if you position the
     MPU9250 flat, is not necessarily 0g/0g/1g for x/y/z. The autoOffset function measures offset
     values. It assumes your MPU9250 is positioned flat with its x,y-plane. The more you deviate
     from this, the less accurate will be your results.
     The function also measures the offset of the gyroscope data. The gyroscope offset does not
     depend on the positioning.
     This function needs to be called at the beginning since it can overwrite your settings!
  */
  Serial.println("Position you MPU9250 flat and don't move it - calibrating...");
  delay(2000);
  myMPU9250.autoOffsets();
  Serial.println("Done!");

  /*  This is a more accurate method for calibration. You have to determine the minimum and maximum
      raw acceleration values of the axes determined in the range +/- 2 g.
      You call the function as follows: setAccOffsets(xMin,xMax,yMin,yMax,zMin,zMax);
      Use either autoOffset or setAccOffsets, not both.
  */
  //myMPU9250.setAccOffsets(-14240.0, 18220.0, -17280.0, 15590.0, -20930.0, 12080.0);

  /*  The gyroscope data is not zero, even if you don't move the MPU9250.
      To start at zero, you can apply offset values. These are the gyroscope raw values you obtain
      using the +/- 250 degrees/s range.
      Use either autoOffset or setGyrOffsets, not both.
  */
  //myMPU9250.setGyrOffsets(45.0, 145.0, -105.0);

  /*  You can enable or disable the digital low pass filter (DLPF). If you disable the DLPF, you
      need to select the bandwdith, which can be either 8800 or 3600 Hz. 8800 Hz has a shorter delay,
      but higher noise level. If DLPF is disabled, the output rate is 32 kHz.
      MPU9250_BW_WO_DLPF_3600
      MPU9250_BW_WO_DLPF_8800
  */
  myMPU9250.enableGyrDLPF();
  //myMPU9250.disableGyrDLPF(MPU9250_BW_WO_DLPF_8800); // bandwdith without DLPF

  /*  Digital Low Pass Filter for the gyroscope must be enabled to choose the level.
      MPU9250_DPLF_0, MPU9250_DPLF_2, ...... MPU9250_DPLF_7

      DLPF    Bandwidth [Hz]   Delay [ms]   Output Rate [kHz]
        0         250            0.97             8
        1         184            2.9              1
        2          92            3.9              1
        3          41            5.9              1
        4          20            9.9              1
        5          10           17.85             1
        6           5           33.48             1
        7        3600            0.17             8

        You achieve lowest noise using level 6
  */
  myMPU9250.setGyrDLPF(MPU9250_DLPF_6);

  /*  Sample rate divider divides the output rate of the gyroscope and accelerometer.
      Sample rate = Internal sample rate / (1 + divider)
      It can only be applied if the corresponding DLPF is enabled and 0<DLPF<7!
      Divider is a number 0...255
  */
  myMPU9250.setSampleRateDivider(5);

  /*  MPU9250_GYRO_RANGE_250       250 degrees per second (default)
      MPU9250_GYRO_RANGE_500       500 degrees per second
      MPU9250_GYRO_RANGE_1000     1000 degrees per second
      MPU9250_GYRO_RANGE_2000     2000 degrees per second
  */
  myMPU9250.setGyrRange(MPU9250_GYRO_RANGE_250);

  /*  MPU9250_ACC_RANGE_2G      2 g   (default)
      MPU9250_ACC_RANGE_4G      4 g
      MPU9250_ACC_RANGE_8G      8 g
      MPU9250_ACC_RANGE_16G    16 g
  */
  myMPU9250.setAccRange(MPU9250_ACC_RANGE_2G);

  /*  Enable/disable the digital low pass filter for the accelerometer
      If disabled the bandwidth is 1.13 kHz, delay is 0.75 ms, output rate is 4 kHz
  */
  myMPU9250.enableAccDLPF(true);

  /*  Digital low pass filter (DLPF) for the accelerometer, if enabled
      MPU9250_DPLF_0, MPU9250_DPLF_2, ...... MPU9250_DPLF_7
       DLPF     Bandwidth [Hz]      Delay [ms]    Output rate [kHz]
         0           460               1.94           1
         1           184               5.80           1
         2            92               7.80           1
         3            41              11.80           1
         4            20              19.80           1
         5            10              35.70           1
         6             5              66.96           1
         7           460               1.94           1
  */
  myMPU9250.setAccDLPF(MPU9250_DLPF_6);

  /* You can enable or disable the axes for gyroscope and/or accelerometer measurements.
     By default all axes are enabled. Parameters are:
     MPU9250_ENABLE_XYZ  //all axes are enabled (default)
     MPU9250_ENABLE_XY0  // X, Y enabled, Z disabled
     MPU9250_ENABLE_X0Z
     MPU9250_ENABLE_X00
     MPU9250_ENABLE_0YZ
     MPU9250_ENABLE_0Y0
     MPU9250_ENABLE_00Z
     MPU9250_ENABLE_000  // all axes disabled
  */
  //myMPU9250.enableAccAxes(MPU9250_ENABLE_XYZ);
  //myMPU9250.enableGyrAxes(MPU9250_ENABLE_XYZ);

  /*
     AK8963_PWR_DOWN
     AK8963_CONT_MODE_8HZ         default
     AK8963_CONT_MODE_100HZ
     AK8963_FUSE_ROM_ACC_MODE
  */
  myMPU9250.setMagOpMode(AK8963_CONT_MODE_100HZ);
  delay(200);
}

void update_quaternion() {
  // Madgwick function needs to be fed North, East, and Down direction like
  // (AN, AE, AD, GN, GE, GD, MN, ME, MD)
  // Accel and Gyro direction is Right-Hand, X-Forward, Z-Up
  // Magneto direction is Right-Hand, Y-Forward, Z-Down
  // So to adopt to the general Aircraft coordinate system (Right-Hand, X-Forward, Z-Down),
  // we need to feed (ax, -ay, -az, gx, -gy, -gz, my, -mx, mz)
  // but we pass (-ax, ay, az, gx, -gy, -gz, my, -mx, mz)
  // because gravity is by convention positive down, we need to ivnert the accel data

  // get quaternion based on aircraft coordinate (Right-Hand, X-Forward, Z-Down)
  // acc[mg], gyro[deg/s], mag [mG]
  // gyro will be convert from [deg/s] to [rad/s] inside of this function
  // quat_filter.update(-a[0], a[1], a[2], g[0] * DEG_TO_RAD, -g[1] * DEG_TO_RAD, -g[2] * DEG_TO_RAD, m[1], -m[0], m[2], q);

  float an = -gValue.x;
  float ae = +gValue.y;
  float ad = +gValue.z;
  float gn = +gyr.x * DEG_TO_RAD;
  float ge = -gyr.y * DEG_TO_RAD;
  float gd = -gyr.z * DEG_TO_RAD;
  float mn = +magValue.y;
  float me = -magValue.x;
  float md = +magValue.z;
  for (size_t i = 0; i < n_filter_iter; ++i) {
    quat_filter.update(an, ae, ad, gn, ge, gd, mn, me, md, q);
  }
  update_rpy(q[0], q[1], q[2], q[3]);
}

void setFilterIterations(const size_t n) {
  if (n > 0) n_filter_iter = n;
}

void update_rpy(float qw, float qx, float qy, float qz) {
  // Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
  // In this coordinate system, the positive z-axis is down toward Earth.
  // Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
  // Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
  // Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
  // These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
  // Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
  // applied in the correct order which for this configuration is yaw, pitch, and then roll.
  // For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
  float a12, a22, a31, a32, a33;  // rotation matrix coefficients for Euler angles and gravity components
  a12 = 2.0f * (qx * qy + qw * qz);
  a22 = qw * qw + qx * qx - qy * qy - qz * qz;
  a31 = 2.0f * (qw * qx + qy * qz);
  a32 = 2.0f * (qx * qz - qw * qy);
  a33 = qw * qw - qx * qx - qy * qy + qz * qz;
  rpy[0] = atan2f(a31, a33);    //Roll
  rpy[1] = -asinf(a32);         //Pitch
  rpy[2] = atan2f(a12, a22);    //Yaw
  rpy[0] *= 180.0f / PI;
  rpy[1] *= 180.0f / PI;
  rpy[2] *= 180.0f / PI;
  //rpy[2] += magnetic_declination;
  if (rpy[2] >= +180.f)
    rpy[2] -= 360.f;
  else if (rpy[2] < -180.f)
    rpy[2] += 360.f;
}

//Process Acceleration data to earth frame
void Get_globle_acc() {
  update_quaternion();
  float acc_x___ = gValue.x; //get from mpu
  float acc_y___ = gValue.y;
  float acc_z___ = gValue.z;
  float acc_the = rpy[0] / 180.f * PI;
  float acc_fin = rpy[1] / 180.f * PI;
  float acc_psi = rpy[2] / 180.f * PI;

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
  Acc[0] = acc[0] * 9.806;
  Acc[1] = acc[1] * 9.806;
  Acc[2] = (acc[2] - 1) * 9.806;

  //Set station to reset speed to 0 m/s
  if (abs(resultantG - 1) < 0.01) stationary = 1;
  else stationary = 0;
}

//Get position prediction
void Position_calulation() {
  Get_Acc_e();
  //Push data
  for (int j = 0; j < 3; j++) {
    for (int i = 1; i < 2; i++) {
      Vel[j][i] = Vel[j][i - 1];
      Pos[j][i] = Pos[j][i - 1];
    }
  }

  //Get New speed
  for (int j = 0; j < 3; j++) {
    //if (stationary == 1) Acc[j] = 0;
    Vel[j][0] = Vel[j][1] + Acc[j] * sample_period / 1000;
    if (stationary == 1) Vel[j][0] = 0;
    Pos[j][0] = Pos[j][1] + Vel[j][0] * sample_period / 1000 * 100;
  }
  //drift_eliminator();
}

//To eliminate drift by velocity
void drift_eliminator() {
  if (stationary == 0) {
    count++;
  } else {
    float drift[3] = {0, 0, 0};
    //Calculate drift = draft rate * factorial of count
    for (int i = 1; i <= count; i++) {
      drift[i] = (Vel[0][1] / count)*i; 
    }

    for (int j = 0; j < 3; j++) {
      Vel[j][0] = Vel[j][0] - Vel[j][0];
      Pos[j][0] = Pos[j][0] - Vel[j][0] * (count* sample_period / 1000);
    }
    count = 0;
  }
}


//void loop() {}
void loop() {
  static uint32_t prev_ms = millis();
  static uint32_t prev_ms_print = millis();
  gValue = myMPU9250.getGValues();
  gyr = myMPU9250.getGyrValues();
  magValue = myMPU9250.getMagValues();
  angles = myMPU9250.getAngles();
  temp = myMPU9250.getTemperature();
  resultantG = myMPU9250.getResultantG(gValue);
  pitch = myMPU9250.getPitch();
  roll  = myMPU9250.getRoll();

  if (millis() > prev_ms + sample_period) {
    Position_calulation();
    prev_ms = millis();
  }

  //  Serial.print("Orientation of the module: ");
  //  Serial.println(myMPU9250.getOrientationAsString());
  //
  //  Serial.print("Temperature in °C: ");
  //  Serial.println(temp);
  //
  //  Serial.println("********************************************");

  if (millis() > prev_ms_print + 100) {
    //For debug display by serial port
    if (Serial.available() > 0) {
      incomingByte = Serial.read();
      //Serial.print("I reeceived: ");
      if (incomingByte == 'v' | incomingByte == 'a' | incomingByte == 'A' |
          incomingByte == 'p' | incomingByte == 'r' | incomingByte == 'm' |
          incomingByte == 'R' | incomingByte == 'g') {
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
      Serial.println(Vel[2][0]);
    } else if (in123 == 'A') {
      Serial.println("Acceleration e-frame (x,y,z):");
      Serial.print(Acc[0]);
      Serial.print("   ");
      Serial.print(Acc[1]);
      Serial.print("   ");
      Serial.println(Acc[2]);
    } else if (in123 == 'a') {
      Serial.println("Acceleration in g (x,y,z):");
      Serial.print(gValue.x);
      Serial.print("   ");
      Serial.print(gValue.y);
      Serial.print("   ");
      Serial.println(gValue.z);
      Serial.print("Resultant g: ");
      Serial.println(resultantG);
    } else if (in123 == 'g') {
      Serial.println("Gyroscope data in degrees/s: ");
      Serial.print(gyr.x);
      Serial.print("   ");
      Serial.print(gyr.y);
      Serial.print("   ");
      Serial.println(gyr.z);
    } else if (in123 == 'R') {
      Serial.println("Rotation angle in Degree(Pitch, Roll, Yew)");
      Serial.print(rpy[0]);
      Serial.print("   ");
      Serial.print(rpy[1]);
      Serial.print("   ");
      Serial.println(rpy[2]);
    } else if (in123 == 'r') {
      for (int j = 0; j < 3; j++) {
        for (int i = 0; i < 2; i++) {
          Pos[j][i] = 0;
          Vel[j][i] = 0;
        }
      }
      in123 = 'p';
    } else if (in123 == 'm') {
      Serial.println("Magnetometer Data in µTesla: ");
      Serial.print(magValue.x);
      Serial.print("   ");
      Serial.print(magValue.y);
      Serial.print("   ");
      Serial.println(magValue.z);
      //delay(100);
    }
    prev_ms_print = millis();
  }

}
