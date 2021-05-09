#include<Wire.h>

long sampling_timer;
const int MPU_addr=0x68;  // I2C address of the MPU-6050

int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ; // Raw data of MPU6050
float GAcX, GAcY, GAcZ; // Convert accelerometer to gravity value
float Cal_GyX,Cal_GyY,Cal_GyZ; // Pitch, Roll & Yaw of Gyroscope applied time factor
float acc_pitch, acc_roll, acc_yaw; // Pitch, Roll & Yaw from Accelerometer
float angle_pitch, angle_roll, angle_yaw; // Angle of Pitch, Roll, & Yaw
float alpha;// = 0.96; // Complementary constant
float timeconstant = 1.0;

static double AcXOff, AcYOff, AcZOff, GyXOff, GyYOff, GyZOff, YawOff;

void setup(){
  Wire.begin();
 
  init_MPU6050();
  
  Serial.begin(115200);
  Wire.setClock(400000);//fast_mode:400000,standard_mode:100000 


  double *Off = calibration();
  AcXOff = Off[0];
  AcYOff = Off[1];
  AcZOff = Off[2];
  GyXOff = Off[3];
  GyYOff = Off[4];
  GyZOff = Off[5];
  Serial.println("----------------------------------------Calibration finished!----------------------------------------");

  delay(1000);

  YawOff = Yaw_compensating();
  Serial.println("----------------------------------------Compensating finished!----------------------------------------");

  
  Serial.print("AcXOff = "); Serial.print(AcXOff);
  Serial.print(" | AcYOff = "); Serial.print(AcYOff);
  Serial.print(" | AcZOff = "); Serial.print(AcZOff);
  Serial.print(" | GyXOff = "); Serial.print(GyXOff);
  Serial.print(" | GyYOff = "); Serial.print(GyYOff);
  Serial.print(" | GyZOff = "); Serial.println(GyZOff);  
  
  Serial.println(YawOff,10);
  delay(2000);
}

int throttle = 0;
int on_off = 0;
double Direction = 0.0;

double BalX;
double BalY;
double BalZ;

void loop(){
  
  //dt 구하기
  double dt;
  static unsigned long t_prev = 0;
  unsigned long t_now = micros();
  dt = (t_now - t_prev)/1000000.0;
  t_prev = t_now;
  //Serial.print("dt = ");Serial.println(dt,10); //check how dt is
//////////////////////////////////////////////////////

//throttle이 없으면 기울기가 있더라도 프로펠러가 회전할 수 없다.
  if(throttle!=0)
    on_off = 1;
    else
    on_off = 0;
//////////////////////////////////////////////////////

  int32_t *raw_data = wire_set();
  AcX=raw_data[0];
  AcY=raw_data[1];
  AcZ=raw_data[2];
  Tmp=raw_data[3];
  GyX=raw_data[4];
  GyY=raw_data[5];
  GyZ=raw_data[6];

  double AcXR = (float) AcX - AcXOff;
  double AcYR = (float) AcY - AcYOff;
  double AcZR = (float) AcZ - AcZOff;



  GAcX = (float) AcXR / 4096.0;
  GAcY = (float) AcYR / 4096.0;
  GAcZ = (float) AcZR / 4096.0;

  acc_pitch = atan (GAcY / sqrt(GAcX * GAcX + GAcZ * GAcZ)) * 57.29577951; // 180 / PI = 57.29577951
  acc_roll = - atan (GAcX / sqrt(GAcY * GAcY + GAcZ * GAcZ)) * 57.29577951; 

//(각속도)
  double GyXR = (float)(GyX - GyXOff)/16.384;//(2^15/2000)
  double GyYR = (float)(GyY - GyYOff)/16.384;
  double GyZR = (float)(GyZ - GyZOff)/16.384;
//////////////////////////////////////////////////////

//(각가속도)
  double dGyXR;
  static double GyXR_prev = 0;
  double GyXR_now = GyXR;
  dGyXR = (GyXR_now - GyXR_prev)/dt;
  GyXR_prev = GyXR_now;

  double dGyYR;
  static double GyYR_prev = 0;
  double GyYR_now = GyYR;
  dGyYR = (GyYR_now - GyYR_prev)/dt;
  GyYR_prev = GyYR_now;

  double dGyZR;
  static double GyZR_prev = 0;
  double GyZR_now = GyZR;
  dGyZR = (GyZR_now - GyZR_prev)/dt;
  GyZR_prev = GyZR_now;
//////////////////////////////////////////////////////

//(자이로 각도, 오차누적)
  Cal_GyX += (float)GyXR * dt; // 2^15 / 2000 = 16.384, 250Hz, 1 /(250Hz * 16.384LSB)
  Cal_GyY += (float)GyYR * dt; // 2^15 / 2000 = 16.384, 250Hz, 1 /(250Hz * 16.384LSB)
  Cal_GyZ += (float)GyZR * dt;
//////////////////////////////////////////////////////

  alpha = timeconstant / (timeconstant + dt);
  //Serial.print("alpha= "); Serial.println(alpha,10);
  
//(각도)
  angle_pitch = alpha * (((float)GyXR * dt) + angle_pitch) + (1 - alpha) * acc_pitch;
  angle_roll = alpha * (((float)GyYR * dt) + angle_roll) + (1 - alpha) * acc_roll;
  angle_yaw = Cal_GyZ; // Accelerometer doesn't have yaw value
//////////////////////////////////////////////////////

//PID Gain
  double Kpo = 1.0/10;
  double Kp = 3.0/10;
  double Kd = 0.8/10;
  double Ki_in = 0.7/10;
  double Ki_out = 0.7/10;
//////////////////////////////////////////////////////

//Target Angle
  static double targetY = 0.0;
  static double targetX = 0.0;
  static double targetZ = 0.0;
  if(Direction==1)
  {
    targetX = -10;
  }
  else if(Direction==2)
  {
    targetX = 10;
  }
  else if(Direction==3)
  {
    targetY = -10;
  }
  else if(Direction==4)
  {
    targetY = 10;
  }
  else if(Direction==5)
  {
    targetX = targetY = 0;
  }
//////////////////////////////////////////////////////

//Angle_Error
  double errorX = targetX - angle_pitch;
  double errorY = targetY - angle_roll;
  //double errorZ = targetZ - angle_yaw;
//////////////////////////////////////////////////////

//Target_Rate
 double target_RateX = Kpo*errorX;
 double target_RateY = Kpo*errorY; 
 //double target_RateZ = Kpo*errorZ;
//////////////////////////////////////////////////////

//Rate_Error
  double Rate_errorX = target_RateX - GyXR;
  double Rate_errorY = target_RateY - GyYR;
  //double Rate_errorZ = target_RateZ - GyZR;
  
  //Serial.print(" | Rate_errorX = "); Serial.print(Rate_errorX);
  //Serial.print(" | Rate_errorY = "); Serial.println(Rate_errorY);
  //Serial.print(" | Rate_errorZ = "); Serial.println(Rate_errorZ);
  
//////////////////////////////////////////////////////

//P_Control
  double ProX = Kp*Rate_errorX;
  double ProY = Kp*Rate_errorY;
  //double ProZ = Kp*Rate_errorZ;
  
  //Serial.print(" ProX = "); Serial.print(ProX);
  //Serial.print(" | ProY = "); Serial.println(ProY);
  //Serial.print(" | ProZ = "); Serial.println(ProZ);
//////////////////////////////////////////////////////

//D_Control
  //double DeX = Kd*(Kpo*(-GyXR) - dGyXR);
  //double DeY = Kd*(Kpo*(-GyYR) - dGyYR);
  double DeZ = (double)((int)(Kd*(-GyZR)));
//////////////////////////////////////////////////////

//I_Control(in)
  static double ResXin = 0.0;
  static double ResYin = 0.0;
  static double ResZin = 0.0;
  ResXin += Ki_in*Rate_errorX*dt;
  ResYin += Ki_in*Rate_errorY*dt;
  //ResZin += Ki_in*Rate_errorZ*dt;
  if(throttle == 0) ResXin = ResYin = ResZin = 0.0;
//////////////////////////////////////////////////////

//I_Control(out)
  static double ResXout = 0.0;
  static double ResYout = 0.0;
  static double ResZout = 0.0;
  ResXout += Ki_out*Rate_errorX*dt;
  ResYout += Ki_out*Rate_errorY*dt;
  //ResZout += Ki_out*Rate_errorZ*dt;
  if(throttle == 0) ResXout = ResYout = ResZout = 0.0;
//////////////////////////////////////////////////////

  BalX = ProX + ResXin + ResXout;
  BalY = ProY + ResYin + ResYout;
  BalZ = DeZ;
  //Serial.print(" BalX = "); Serial.print(BalX);
  //Serial.print(" | BalY = "); Serial.print(BalY);
  //Serial.print(" | BalZ = "); Serial.println(BalZ);

  if(Serial.available()>0)
  {
    throttle = throttle_set();
    Direction = direction_set();
  }

  double speedA = on_off*(throttle + BalY + BalX + BalZ);
  double speedB = on_off*(throttle - BalY + BalX - BalZ);
  double speedC = on_off*(throttle - BalY - BalX + BalZ);
  double speedD = on_off*(throttle + BalY - BalX - BalZ);
  
  //Serial.print(" | 6Speed = "); Serial.print(throttle + BalY + BalX + BalZ);
  //Serial.print(" | 10Speed = "); Serial.print(throttle - BalY + BalX - BalZ);
  //Serial.print(" | 9Speed = "); Serial.print(throttle - BalY - BalX + BalZ);
  //Serial.print(" | 5Speed = "); Serial.println(throttle + BalY - BalX - BalZ);
  
  int iSpeedA = constrain((int)speedA,0,250);
  int iSpeedB = constrain((int)speedB,0,250);
  int iSpeedC = constrain((int)speedC,0,250);
  int iSpeedD = constrain((int)speedD,0,250);

  //Serial.print(" | angle_pitch = "); Serial.print(angle_pitch);
  //Serial.print(" | angle_roll = "); Serial.print(angle_roll);
  //Serial.print(" | angle_yaw = "); Serial.println(angle_yaw);

  Serial.print("Cal_GyX= "); Serial.print(Cal_GyX);
  Serial.print(" |acc_pitch= "); Serial.print(acc_pitch);
  Serial.print(" |angle_pitch= "); Serial.println(angle_pitch);
  
  analogWrite(6,iSpeedA);
  analogWrite(10,iSpeedB);
  analogWrite(9,iSpeedC);
  analogWrite(5,iSpeedD);

  //Serial.print(" 6Speed= "); Serial.print(iSpeedA);
  //Serial.print(" 10Speed= "); Serial.print(iSpeedB);
  //Serial.print(" 9Speed= "); Serial.print(iSpeedC);
  //Serial.print(" 5Speed= "); Serial.println(iSpeedD);
  
}

int throttle_set()
{
    int Throttle;
    char userInput = Serial.read();
    Serial.println(userInput);
    
    if(userInput>='0'&&userInput<='9')
    {
      Throttle = (userInput-'0')*25;
    }
    return Throttle;
}

int direction_set()
{
    char userInput = Serial.read();
    Serial.println(userInput);
    
    if(userInput == 'w')
    {
      return 1;// -10.0
    }
    else if(userInput == 'x')
    {
      return 2;// 10.0
    }
    else if(userInput == 'a')
    {
      return 3;// -10.0
    }
    else if(userInput == 'd')
    {
      return 4;// 10.0
    }
    else if(userInput == 's')
    {
      return 5;// 0.0
    }
    else
    return 6;      
}

void init_MPU6050(){
  //MPU6050 Initializing & Reset
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  //MPU6050 Clock Type
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0x03);     // Selection Clock 'PLL with Z axis gyroscope reference'
  Wire.endTransmission(true);

  //MPU6050 Gyroscope Configuration Setting
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1B);  // Gyroscope Configuration register
  //Wire.write(0x00);     // FS_SEL=0, Full Scale Range = +/- 250 [degree/sec]
  //Wire.write(0x08);     // FS_SEL=1, Full Scale Range = +/- 500 [degree/sec]
  //Wire.write(0x10);     // FS_SEL=2, Full Scale Range = +/- 1000 [degree/sec]
  Wire.write(0x18);     // FS_SEL=3, Full Scale Range = +/- 2000 [degree/sec]
  Wire.endTransmission(true);

  //MPU6050 Accelerometer Configuration Setting
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1C);  // Accelerometer Configuration register
  //Wire.write(0x00);     // AFS_SEL=0, Full Scale Range = +/- 2 [g]
  //Wire.write(0x08);     // AFS_SEL=1, Full Scale Range = +/- 4 [g]
  Wire.write(0x10);     // AFS_SEL=2, Full Scale Range = +/- 8 [g]
  //Wire.write(0x18);     // AFS_SEL=3, Full Scale Range = +/- 10 [g]
  Wire.endTransmission(true);

  //MPU6050 DLPF(Digital Low Pass Filter)
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1A);  // DLPF_CFG register
  Wire.write(0x00);     // Accel BW 260Hz, Delay 0ms / Gyro BW 256Hz, Delay 0.98ms, Fs 8KHz 
  //Wire.write(0x01);     // Accel BW 184Hz, Delay 2ms / Gyro BW 188Hz, Delay 1.9ms, Fs 1KHz 
  //Wire.write(0x02);     // Accel BW 94Hz, Delay 3ms / Gyro BW 98Hz, Delay 2.8ms, Fs 1KHz 
  //Wire.write(0x03);     // Accel BW 44Hz, Delay 4.9ms / Gyro BW 42Hz, Delay 4.8ms, Fs 1KHz 
  //Wire.write(0x04);     // Accel BW 21Hz, Delay 8.5ms / Gyro BW 20Hz, Delay 8.3ms, Fs 1KHz 
  //Wire.write(0x05);     // Accel BW 10Hz, Delay 13.8ms / Gyro BW 10Hz, Delay 13.4ms, Fs 1KHz 
  //Wire.write(0x06);     // Accel BW 5Hz, Delay 19ms / Gyro BW 5Hz, Delay 18.6ms, Fs 1KHz 
  Wire.endTransmission(true);
}


int32_t* wire_set()
{
  static int32_t raw_data[7];
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  int16_t AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
  int16_t AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  int16_t AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  int16_t Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  int16_t GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  int16_t GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  int16_t GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  raw_data[0] = AcX;
  raw_data[1] = AcY;
  raw_data[2] = AcZ;
  raw_data[3] = Tmp;
  raw_data[4] = GyX;
  raw_data[5] = GyY;
  raw_data[6] = GyZ;
  return raw_data;
}

double* calibration()
{
  int32_t AcXsum = 0;
  int32_t AcYsum = 0;
  int32_t AcZsum = 0;
  int32_t GyXsum = 0;
  int32_t GyYsum = 0;
  int32_t GyZsum = 0;
  double AcXoff = 0.0;
  double AcYoff = 0.0;
  double AcZoff = 0.0;
  double GyXoff = 0.0;
  double GyYoff = 0.0;
  double GyZoff = 0.0;

  static double Off[6];
  for (double n = 1;n<=1000;n++)
  {
      int32_t *Gy = wire_set();
      int16_t AcX = Gy[0];
      int16_t AcY = Gy[1];
      int16_t AcZ = Gy[2];
      
      int16_t GyX = Gy[4];
      int16_t GyY = Gy[5];
      int16_t GyZ = Gy[6];

      Serial.print("Calibration: ");
      Serial.print(n/10,1);
      Serial.println("%");
     
      AcXsum += AcX;
      AcYsum += AcY;
      AcZsum += AcZ;
      
      GyXsum += GyX;
      GyYsum += GyY;
      GyZsum += GyZ;
  }
  AcXoff = AcXsum / 1000.0;
  AcYoff = AcYsum / 1000.0;
  AcZoff = AcZsum / 1000.0;
  GyXoff = GyXsum / 1000.0;
  GyYoff = GyYsum / 1000.0;
  GyZoff = GyZsum / 1000.0;

  Off[0] = AcXoff;
  Off[1] = AcYoff;
  Off[2] = AcZoff - 4096.0; //중력은 z방향이기 때문에 기본적으로 4096의 값을 가지고 있다.
  Off[3] = GyXoff;
  Off[4] = GyYoff;
  Off[5] = GyZoff;
  return Off;
}

double Yaw_compensating()
{
  int16_t dZSum = 0;
  double YawOff = 0.0;
  for (int n = 1;n<=1000;n++)
  {
      double dZ;
      static int16_t Z_prev = 0;
      int32_t *Z_pointer = wire_set();
      int32_t Z_now = Z_pointer[6];
      dZ = Z_now - Z_prev;
      Z_prev = Z_now;
      
      Serial.print("Compensating: ");
      Serial.print(n/10);
      Serial.println("%");
      dZSum += dZ;
  }
  YawOff = dZSum / 1000.0;
  return YawOff;
}
