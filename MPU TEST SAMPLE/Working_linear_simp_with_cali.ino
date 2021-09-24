/* This ino code performs the MPU9250 IMU to predict the position
 * The system is runing at 40Hz, the front Y-axis is pointing to West (Yew = 0 degree)
 * MPU library: https://github.com/hideakitai/MPU9250
 *              Another library: Wire.h
 * Port connection: MPU9250 --- ESP32
 *                    VCC   --- 3V3
 *                    GND   --- GND
 *                    SCL   --- 22
 *                    SDA   --- 21
 * Debug use: by send different char by serial to esp32, it will output current states:
 *            "a" :Raw acceleration data,
 *            "A" :Processed acceleration data on Earth Frame
 *            "m" :Yew, Pitch, Roll data(Y axis point to West),
 *            "v" :Velocity data
 *            "P" :Position data
 *            "r" :Rest position data to origin(xyz=0,0,0).
*/
#include "MPU9250.h"

MPU9250 mpu;
int sample_period = 25; //in ms
int Freq_acc = 1000/sample_period; //40Hz
char in123= 'm';          // char for input debug
char incomingByte = 'm';
int count=0;              //Count for drift velocity elimination

//Initialize variables for position calculation
float acc_x=0.f, acc_y=0.f, acc_z=0.f;
float ACC_X[20]={}, ACC_Y[20]={}, ACC_Z[20]={}, ACC_mag[20]={}; 
float VEL_X[20]={}, VEL_Y[20]={}, VEL_Z[20]={};
float POS_X[20], POS_Y[20], POS_Z[20], stationary[20];


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
    delay(5000);
    mpu.calibrateMag();

    print_calibration();
    mpu.verbose(false);

    
}

void loop() {
    if (mpu.update()) {
        static uint32_t prev_ms = millis();
        if (millis() > prev_ms + sample_period) {
            Position_cal();
            prev_ms = millis();
        }
    }
}

void get_sec_data(){
  Get_globle_acc();
            for(int i=1; i<20; i++){  //get acc data for every 25ms
              ACC_X[i] = ACC_X[i-1];
              ACC_Y[i] = ACC_Y[i-1];
              ACC_Z[i] = ACC_Z[i-1];
              ACC_mag[i] = ACC_mag[i-1];
              stationary[i] = stationary[i-1];
            }
            //remove G and set unit as m/s^2
            ACC_X[0] = acc_x*9.81;
            ACC_Y[0] = acc_y*9.81;
            ACC_Z[0] = (acc_z-1.06)*9.81;  
            ACC_mag[0] = sqrt(acc_x*acc_x+acc_y*acc_y+(acc_z)*(acc_z));

            //Set stationary
            if(abs(ACC_mag[0]-1.05) <= 0.01){
                stationary[0] = 1;
                ACC_X[0] = 0;
                ACC_Y[0] = 0;
                ACC_Z[0] = 0;
            }else stationary[0] =0; 
}

//Position Calculation
void Position_cal(){
  get_sec_data();  //Get acceleration data on earth frame

  //PUSH DATA
  for(int i=1; i<20; i++){
    VEL_X[i] = VEL_X[i-1];
    VEL_Y[i] = VEL_Y[i-1];
    VEL_Z[i] = VEL_Z[i-1];
    POS_X[i] = POS_X[i-1];
    POS_Y[i] = POS_Y[i-1];
    POS_Z[i] = POS_Z[i-1];
  }
    VEL_X[0] = VEL_X[1] + ACC_X[0] * sample_period/1000; //Get speed m/s
    VEL_Y[0] = VEL_Y[1] + ACC_Y[0] * sample_period/1000;
    VEL_Z[0] = VEL_Z[1] + ACC_Z[0] * sample_period/1000;
    if(stationary[0]==1){
      VEL_X[0] = 0;
      VEL_Y[0] = 0;
      VEL_Z[0] = 0;   
    }
//    if(stationary[0]==0){
//      VEL_X[0] = VEL_X[0] - (VEL_X[0]-VEL_X[1])*sample_period/1000; 
//      VEL_Y[0] = VEL_Y[0] - (VEL_Y[0]-VEL_Y[1])*sample_period/1000;
//      VEL_Z[0] = VEL_Z[0] - (VEL_Z[0]-VEL_Z[1])*sample_period/1000;      
//    }
//    POS_X[0] = POS_X[1] + VEL_X[0] * sample_period/1000 *100; //Get speed cm
//    POS_Y[0] = POS_Y[1] + VEL_Y[0] * sample_period/1000 *100; //Get speed cm
//    POS_Z[0] = POS_Z[1] + VEL_Z[0] * sample_period/1000 *100; //Get speed cm
    drift_eli(); //Eliminate drift and get position

    //For debug display by serial port
    if(Serial.available()>0){
      incomingByte = Serial.read();
      //Serial.print("I reeceived: ");
      if(incomingByte=='v'|incomingByte=='a'|incomingByte=='A'|
          incomingByte=='p'|incomingByte=='r'|incomingByte=='m'){
            in123 = incomingByte;
          }
    }

    if(in123 == 'p'){
      Serial.print("station ");
      Serial.print(stationary[0]);
      Serial.print("xyz ");
      Serial.print(POS_X[0]);
      Serial.print(" ");
      Serial.print(POS_Y[0]);
      Serial.print(" ");
      Serial.println(POS_Z[0]); 
    }else if(in123 == 'v'){
      Serial.print(" vel ");
      Serial.print(VEL_X[0]);
      Serial.print(" ");
      Serial.print(VEL_Y[0]);
      Serial.print(" ");
      Serial.println(VEL_Z[0]);
    }else if(in123 == 'A'){
      Serial.print(" ACC ");
      Serial.print(ACC_X[0]);
      Serial.print(" ");
      Serial.print(ACC_Y[0]);
      Serial.print(" ");
      Serial.print(ACC_Z[0]); 
      Serial.print(" ");
      Serial.println(ACC_mag[0]); 
    }else if(in123 == 'r'){
      POS_X[0]=0; POS_Y[0]=0; POS_Z[0]=0;
      in123 = 'p';
    }else if(in123 == 'm'){
      Serial.print("Yaw, Pitch, Roll: ");
      Serial.print(mpu.getYaw(), 2);
      Serial.print(", ");
      Serial.print(mpu.getPitch(), 2);
      Serial.print(", ");
      Serial.println(mpu.getRoll(), 2);
    }else{
      Serial.print(" acc ");
      Serial.print(acc_x);
      Serial.print(" ");
      Serial.print(acc_y);
      Serial.print(" ");
      Serial.println(acc_z); 
    }
    //Serial.println(in123); 
}

//To eliminate drift by velocity
void drift_eli(){
  int time_drift=0;
  float drift_rate[3]={0, 0, 0}; //for x, y and z = 0;
  if(stationary[0]==0) count++;
  if((stationary[0]==1) & (stationary[1]==0)){      //Find end
//    for(int i=1; i<20; i++){                      //get data
//      if (stationary==1) break;
//      else{
//        time_drift=i;
//      }
//    }
    time_drift = count;   //Set drift during time unit: /period
    count =0;   //reset count
    drift_rate[0] = VEL_X[1]/time_drift;  //Get drift rate
    drift_rate[1] = VEL_Y[1]/time_drift;
    drift_rate[2] = VEL_Z[1]/time_drift;
    for(int i=1; i<=time_drift; i++){
      VEL_X[i]=VEL_X[i]-(time_drift-i+1)*drift_rate[0]; //Eliminate drift
      VEL_Y[i]=VEL_Y[i]-(time_drift-i+1)*drift_rate[1];
      VEL_Z[i]=VEL_Z[i]-(time_drift-i+1)*drift_rate[2];
    }
    for(int i=time_drift; i>0; i--){
      POS_X[0] = POS_X[0] + VEL_X[i] * sample_period/1000 *100; //upadte pos only
      POS_Y[0] = POS_Y[0] + VEL_Y[i] * sample_period/1000 *100; //for once of 20 runs
      POS_Z[0] = POS_Z[0] + VEL_Z[i] * sample_period/1000 *100;
    }
    for(int i=1; i<20; i++){
      POS_X[i] = POS_X[i-1]; //record position data
      POS_Y[i] = POS_Y[i-1];
      POS_Z[i] = POS_Z[i-1];
    }
   //Serial.print("get count < 20: time_drift: ");
   //Serial.println(time_drift);
  }else if(count >= 19){    //If whole period is unstationary
    count =0;
    time_drift=20;
    drift_rate[0] = VEL_X[0]/time_drift;
    drift_rate[1] = VEL_Y[0]/time_drift;
    drift_rate[2] = VEL_Z[0]/time_drift;
    for(int i=0; i<=time_drift; i++){
      VEL_X[i]=VEL_X[i]-(time_drift-i)*drift_rate[0];
      VEL_Y[i]=VEL_Y[i]-(time_drift-i)*drift_rate[1];
      VEL_Z[i]=VEL_Z[i]-(time_drift-i)*drift_rate[2];
    }
    for(int i=time_drift; i>=0; i--){
      POS_X[0] = POS_X[0] + VEL_X[i] * sample_period/1000 *100; //upadte pos only
      POS_Y[0] = POS_Y[0] + VEL_Y[i] * sample_period/1000 *100; //for once of 20 runs
      POS_Z[0] = POS_Z[0] + VEL_Z[i] * sample_period/1000 *100;
    }
    for(int i=1; i<20; i++){
      POS_X[i] = POS_X[i-1]; //record pos
      POS_Y[i] = POS_Y[i-1];
      POS_Z[i] = POS_Z[i-1];
    }
    //Serial.println("get count = 20");
  }
  
}

//Process Acceleration data to earth frame
void Get_globle_acc(){
    float acc_x___ = mpu.getAccX(); //get from mpu
    float acc_y___ = mpu.getAccY();
    float acc_z___ = mpu.getAccZ();
    float acc_the = mpu.getRoll()/180.f*PI;
    float acc_fin = mpu.getPitch()/180.f*PI;
    float acc_psi = mpu.getYaw()/180.f*PI;

    //rotate from x-axis
    float acc_x__=acc_x___;
    float acc_y__=acc_y___*cos(acc_the)-acc_z___*sin(acc_the);
    float acc_z__=acc_z___*cos(acc_the)+acc_y___*sin(acc_the);
    //rotate from y-axis
    float acc_x_=acc_x__*cos(acc_fin)-acc_z__*sin(acc_fin);
    float acc_y_=acc_y__;
    float acc_z_=acc_z__*cos(acc_fin)+acc_x__*sin(acc_fin);
    //rotate from z-axis
    acc_x=acc_x_*cos(acc_psi)+acc_y_*sin(acc_psi);
    acc_y=acc_y_*cos(acc_psi)-acc_x_*sin(acc_psi);
    acc_z=acc_z_;

//    Serial.print("xyz ");
//    Serial.print(acc_x);
//    Serial.print(" ");
//    Serial.print(acc_y);
//    Serial.print(" ");
//    Serial.println(acc_z);   
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
