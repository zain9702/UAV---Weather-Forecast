#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include <Encoder.h>
#include <math.h>

Encoder myEnc(2, 3);

///////////// Pitch Control ///////////////

float Pitch;
float Pitch_Error;
float Pitch_Setpoint;
double Pitch_PID;
double Previous_Pitch_Control;
float Pitch_Control; // Sum of PID output and decoupler
float Kp_Pitch = 50.1;
float Kd_Pitch = 2.22;
float Pitch_Speed;
float tmp1;
//////////////////////////////////////////////

//////////// Yaw Control ///////////////

float Yaw;
float Yaw_Error;
float Yaw_Setpoint;
double Yaw_PID;
double Previous_Yaw_Control;
double Yaw_Control; // Sum of PID output and decoupler
float KP_Yaw = 35.02;
float tmp2;

/////////////////////////////////////////

/////////// Decouplers ///////////////

float Yaw_Decoupler;   // This decouples the yaw disturbance and is added to the pitch output
float Pitch_Decoupler; // This decouples the pitch disturbance and is added to the Yaw output

/////////////////////////////////////

MPU6050 imu(0x68);


int SampleTime = 10; // ms sample time
float GyroAngle;        //Stores the Summed angle out og they gyroscope
double CurrentTime;     //stores the current time from mills()
double CurrentTime2;
double TimeDiff;        //stores the time that has elapsed
double TimeDiff2
float angle;           //stores the filterted angle value
float Pangle;          //stores the previous angle vlaue
float PreviousTime;    //stores the previous time
float PreviousTime2;    

uint8_t Accel, Gyro = 1;  //sets gyro range to 1 = +- 500 dps and +- 4g


void setup() {
  
Wire.begin();           //Start I2C wire

  imu.initialize(); // sets gyro and accel to most sensitive
  
  imu.setFullScaleGyroRange(Gyro);    //Set range of gyro 
  imu.setFullScaleAccelRange(Accel);  //Set range of accelerometer
  

  imu.setZGyroOffset(150 + Signal_Conditioning()); // aplly the offset onto the gyro signal

Pitch_Setpoint = 0; // Initial Setpoints
  Yaw_Setpoint = 30;
  
}

long oldPosition  = -999;

void loop() {
  
  
CurrentTime2 = millis();
TimeDiff2 = CurrentTime2 - PreviousTime2;

if (angle == Pitch_Setpoint) and (Yaw == Yaw_Setpoint) and (TimeDiff2 >= 10000) // if both setpoints held for 10 seconds
{
  Pitch_Setpoint = Pitch_Setpoint + 15; // Pitch Setpoint adds 15 degrees
  
  if (Pitch_Setpoint > 15) // If pitch setpoint is higher than 15
  {
    Pitch_Setpoint = 0; // Reset setpoint to 0
  }
  Yaw_Setpoint = Yaw_Setpoint - 30;  // Update Yaw setpoint
  
   if (Yaw_Setpoint > 15) // If yaw setpoint is lower than 0 
  {
    Yaw_Setpoint = 30; // Reset setpoint to 30
  }
}
 else // If setpoints not held for 10 seconds then continue with original setpoint
 { 
  CurrentTime = millis();                   //reads the current time in microseconds
  TimeDiff = CurrentTime - PreviousTime;     //Gets the time difference that has elapsed

  //*******************************************************sampling Time********************************************//
  if (TimeDiff >= SampleTime) //Sampling Time has elapsed
  {
    int16_t gyroz = imu.getRotationZ(); //gets the z gyro raw data every sample

    GetGAngle(gyroz); //update the global gyro angle

    Comp_condition(GyroAngle, GetAAngle(), SampleTime); // get angle from comp fillter
    // reterns to global vairable "angle"
    
    PreviousTime = CurrentTime; //resets the current time to the brevious time for the next sampling time
  }


 long newPosition = myEnc.read();
  if (newPosition != oldPosition) {
    oldPosition = newPosition;

// Constrain 0 to 180 and 0 to -180

  if (newPosition >= 400)
    {
      myEnc.write(newPosition-800);
    
    }
   else if (newPosition < -400)
    {
      myEnc.write(newPosition+800);
    }

     Yaw = sqrt(sq(newPosition/2.222));
     
  ////////////////////////////////////////////////////// Pitch Controller ///////////////////////////////////////////////////////////////////////

tmp1 = (yaw_PID - (-1.974 * Previous_Yaw_Control)); // For decoupler
Pitch_Error = Pitch_Setpoint - angle; 
Pitch_PID = (Kp_Pitch * Pitch_Error) + (Kd_Pitch * GyroRate); // Main pitch PID output
Yaw_Decoupler = - ((0.03375 * tmp1) + (-0.06599 * Previous_Yaw_Control) + (0.0344 * Yaw_Control)); // Decoupler that negates yaw disturbance of pitch
Pitch_Control = Pitch_PID + Yaw_Decoupler; // Final Pitch control signal to be sent to actuator

//////////////////////////////////////////////////// Yaw Controller /////////////////////////////////////////////////////////////////////////////

tmp2 = (Pitch_PID - (-0.005958 * Pitch_Control)); // For decoupler
 Yaw_Error = Yaw_Setpoint - Yaw;
  Yaw_PID = (Kp_Yaw * Yaw_Error); // Main Yaw PID output
  Pitch_Decoupler = -(0.035 * tmp2 + (0.139 * Pitch_Control)); // Decoupler that negates pitch disturbance of yaw
  Yaw_Control = Yaw_PID + Pitch_Decoupler; // Final Yaw control signal to be sent to actuator

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

PreviousTime = CurrentTime;
PreviousTime2 = CurrentTime2;
}
 }

float GetAAngle() // gets accelerometer angle
{
  int16_t x = imu.getAccelerationX (); // gets raw data from accel x
  int16_t y = imu.getAccelerationY (); // gets raw data from accel y
  float angle = atan2(y, x) * 180 / PI; //Calculate the angle (simple way)
  return (angle);
}

float GetGAngle(int16_t gz) // calculates the gyro angle 
{
  float GyroRate = (gz) / 65.5;   //Devide by LSB from data sheet for +- 500 dps
  GyroAngle += (GyroRate * SampleTime / 1000); //integrate the gyro angle and save as a global var
  return (GyroRate);
}

float Signal_Conditioning() //gets the offset of the Z gyro 
{
  float z_error_sum = 0; //stores sum of resting gyro data (should be 0)
  float z_error; // stores the error

  int16_t gyroz; // stores the gyro z data

  for ( int i = 1; i <= 10000; i++) // loop for 1000 timnes
  {
    gyroz = imu.getRotationZ(); // get gyro z raw data
    z_error_sum += gyroz;       // sum data
  }

   z_error = z_error_sum / 10000; // get average
   return z_error;
}

void Comp_condition(float Gyro_in, float Accan, float sampleTime) // runs a complementary fillter
{
  float alpha = 0.85; // the ratio of the gyro and ACC
  angle = (alpha) * (Pangle + Gyro_in * (sampleTime / 1000)) + ((1 - alpha) * Accan); //saves the filltered angle to a globle variable
  Pangle = angle;
}
