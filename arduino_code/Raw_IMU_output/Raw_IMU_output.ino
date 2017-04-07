#include "CurieIMU.h"
#include "math.h"
#define PRINT_DELAY 30 //milliseconds between print statements

int rate = 100;  // sample rate
int gyroRange = 500;
int accelRange = 4;
bool print_raw_output = true; // flag to set whether to print raw or scaled output
const float pi = 3.14159;
const float ninety_degrees = 90.0*180.0/pi;

float q1State, q2State, q3State, q4State; //current body angle (quaternions)
float vxState, vyState, vzState;
float pxState, pyState, pzState;

long time_reference;  // reference time to be used to 
long next_read;       // compute when the next 

double Q_update[4][4] = {
  {0, 0, 0, 0},
  {0, 0, 0, 0},
  {0, 0, 0, 0},
  {0, 0, 0, 0}
};

double Q_state[1][4] = {1,0,0,0};

void setup() {
  // put your setup code here, to run once:
  
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(250000);

  ////// Header Output //////////
  if(print_raw_output)
  {
    Serial.println("IMU Header.  Raw (unscaled) output.");
  }
  else
  {
    Serial.println("IMU Header.  Scaled output.");    
  }
  Serial.print("Gyro Range: "); 
  Serial.print(gyroRange);
  Serial.print(", Accel range: ");
  Serial.print(accelRange);
  Serial.print(", Sample rate (hz): ");
  Serial.println(rate);

  
  
  CurieIMU.begin();
   // Set the accelerometer range to 2G
  CurieIMU.setAccelerometerRange(accelRange);
  // Set the gyro range to 250 degrees/second
  CurieIMU.setGyroRange(gyroRange);


  CurieIMU.setGyroRate(rate);
  CurieIMU.setAccelerometerRate(rate);
  
  q1State = 1; 
  q2State = 0; q3State = 0; q4State = 0; //zero rotation
 
  
  vxState = 0;
  vyState = 0;
  vzState = 0;

  pxState = 0;
  pyState = 0;
  pzState = 0;


  time_reference = micros(); // 
  

}

void loop() {
  // read the IMU and spit it out on the serial port.  
   static int i=0;
   static int millis_print = 0; 
   static int prevTime = 0;

   static int frameNum = 0;           // current IMU frame (increments) 
   const int  frame_dt_micros = 1000000/rate; // number of microseconds per IMU frame (10ms : 100Hz)

   int currTime = micros();
   int deltaTime = currTime - prevTime; 
   prevTime = currTime;
   float dt = ((float)deltaTime/1000000.0);  //Frame time in seconds
   

   if(currTime >= time_reference + frameNum * frame_dt_micros)
   { // this should occur once per 
       frameNum++;
       float ax, ay, az;   //scaled accelerometer values
       float gx, gy, gz;   //scaled gyro values

       int gxr, gyr, gzr; //raw gyro values (int)
       int axr, ayr, azr; //raw accel values (int);

       if(print_raw_output)
       {        
         CurieIMU.readGyro(gxr, gyr, gzr);
         CurieIMU.readAccelerometer(axr, ayr, azr);  

         Serial.print(currTime); // current time in microseconds
         Serial.print(", ");
  
         // print these values on the serial port.  
         Serial.print(axr);
         Serial.print(", ");
         Serial.print(ayr);
         Serial.print(", ");
         Serial.print(azr);
         Serial.print(", ");
         Serial.print(gxr);
         Serial.print(", ");
         Serial.print(gyr);
         Serial.print(", ");
         Serial.println(gzr);
        
       }
       else
       {
         CurieIMU.readGyroScaled(gx, gy, gz);
         CurieIMU.readAccelerometerScaled(ax, ay, az);     

         Serial.print(currTime); // current time in microseconds
         Serial.print(", ");
         // print these values on the serial port.  
         Serial.print(ax,4);
         Serial.print(", ");
         Serial.print(ay,4);
         Serial.print(", ");
         Serial.print(az,4);
         Serial.print(", ");
         Serial.print(gx,4);
         Serial.print(", ");
         Serial.print(gy,4);
         Serial.print(", ");
         Serial.println(gz,4);
       }

   }

}


