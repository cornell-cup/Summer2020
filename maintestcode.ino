#include <MatrixMath.h>
#include <Event.h>
#include <Timer.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

//#include <arduino-timer.h>
#include <EEPROM.h>

#define N  (2)

//initializing motor 1 vars:
//volatile long A1counter = 0;
volatile long A1counternoreset = 0; 
volatile long A2counternoreset = 0;

//initializing motor 2 vars:
//volatile long A2counter = 0;

//long cm = 0.0;
long duration = 0.0;
float USspeed = 0.0;
float lastcm = 0.0;

//array variables:
float disArray[50];
float spArray[50];
int locCount = 0; 

//max distance bot travels
const int maxdistcm = 150;

//1D kalman filter testing variables:
/*
//constant for filtering (for belief estimation)-- try measuring dist variances again w noreset
const float del = 9.6007;
//if still getting junk data, try ignoring value from lowest average column to fix stdev 
const float phi2 = 9.3019089;
const float psi2 = 2.86155;

//kalman filter variables:
float ut = 0.0;
float utb = 0.0;
float ot2 = 0.8;
float ot2b = 0.0;

//transition vars:
float kt1 = 0.0;
//position measurement var-- need to use noresetcounters:
float et1 = 0.0;

//next timestep vars:
float ut1 = 0.0;
float o2t1 = 0.0;
*/

//2D kalman filter testing variables:
//2x2 constant matrices:
mtx_type A[2][2];
mtx_type C[2][2];
mtx_type psi[2][2];

//2x2 variable matrices:
mtx_type sig[2][2];
mtx_type sigb[2][2];
mtx_type sig1[2][2];
mtx_type K[2][2];

//2x1 matrices:
mtx_type u[2];
mtx_type ub[2];
mtx_type u1[2];
mtx_type et1[2];

mtx_type mid[2][2];
mtx_type mid2[2][2];

//loop variable account for a:
bool looper = false;

//Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);


// create a timer with default settings
Timer t;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  //Motor input pin
  pinMode(9, OUTPUT);
  //MOTOR 1 OUT B
  pinMode(3, INPUT);
  //MOTOR 2 OUT B
  pinMode(2, INPUT);

  //trying 3, 4 motor pin, also 6, 2 pinswap swap to test results (now reading motor 1 out a)
  //MOTOR 1 OUT A 
  //pinMode(4, INPUT);  
  //MOTOR 2 OUT A
  //pinMode(6, INPUT);

  //Ultrasonic sensor pins
  //Trigger pin 
  pinMode(11, OUTPUT);
  //Data pin
  pinMode(12, INPUT);

  //Pin 3: corresponds to A1counter
  attachInterrupt(1, Increment1, CHANGE);
  //Pin 2: corresponds to A2counter
  attachInterrupt(0, Increment2, CHANGE);

  //define components of each matrix here
  A[0][0] = 1;
  A[0][1] = .2;
  A[1][0] = 0;
  A[1][1] = 1;

  C[0][0] = 1;
  C[0][1] = 0;
  C[1][0] = 0;
  C[1][1] = 1;

  //2. psi[0][0] = 29.67973;
  //1. 125.4702284
  psi[0][0] = 28.49802553;
  psi[0][1] = 0;
  psi[1][0] = 0;
  psi[1][1] = 1501.203381;
  //1. psi[1][1] = 9090.135727;
  //2. psi[1][1] = 5375.875347;


  //tried 1 before
  sig[0][0] = 1;
  sig[0][1] = 0;
  sig[1][0] = 0;
  sig[1][1] = 2500;
  //try 30992
  //sig[1][1] = 2500;
  
  u[0] = 0;
  u[1] = 0;

  Matrix.Print((mtx_type*)A, 2, 2, "A");
  Matrix.Print((mtx_type*)C, 2, 2, "C");
  Matrix.Print((mtx_type*)psi, 2, 2, "psi");
  Matrix.Print((mtx_type*)sig, 2, 2, "sig");
  Matrix.Print((mtx_type*)sigb, 2, 2, "sigb");  
  Matrix.Print((mtx_type*)u, 2, 1, "u");
  
  digitalWrite(9, LOW);
  delay(16000);

  //setup accelerometer for data collection:
  //if(!bno.begin())
  //{
    /* There was a problem detecting the BNO055 ... check your connections */
   // Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
   // while(1);
  //}

  //find out precise time it takes for robot to *just* begin moving
  digitalWrite(9, HIGH);
  //insert the measured time in this delay:
  delay(150);

  t.every(200, usReadings);

  
}

void loop() {
  //previous wheel diameter constants used:
  //reference measurementnoise code for updated vals
  
  float countA1 = (A1counternoreset+A2counternoreset)/16.3862778735153;
  //float countA1 = (ut);
  //Serial.println(ut, 3);  
  if (countA1 < (maxdistcm)) {
    t.update();     
  }
  else {  
    digitalWrite(9, LOW);
    //appending final data:
    
    //implement accounting for deceleration here: 
    //looper = true; 
    //imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    //trying to alter measurement error matrix to see if velocities are taken more into account

    //psi[0][0] = 2.667;
    //psi[1][1] = 5;

    //float v0 = accelSave();
    delay(200);
    //remember to put finalcount float after usReadings call to get correct distance
    usReadings();
    delay(200);
    usReadings();
    
    //try omitting following usReadings, why is velocity variance so high-- can decrease?
    //adjust speed variance based on timestep?? end timesteps always have significantly lower var
    //(decrease measurement noise var.s for speed)
    //decrease speed variance just in else {} end loop?
    
    //try measuring stopping avg accel, add into step 1 formula 1/2(at^2)?
    //incorporate w decrease in velocity variance
    //can estimate from final velocity readings-- compare to see if consistent w IMU data

    //try going back to old /16.399 constant factor, 100ms refresh rate-- is raw encoder data more acc?

    looper = true;
    delay(1000);
    usReadings(); 
    //float v1 = accelSave();
    //float ac = (v1 - v0)/1.4;
    //u[0] = (u[0] + (.5*(ac)*(1.4*1.4)));
    //u[1] = (u[1] - (ac*1.4))
        
    float finalcount = (A1counternoreset+A2counternoreset)/16.3862778735153;
    
    Serial.println(" "); 
    Serial.println(" ");  
    
    //Serial.println(u[0], 3);
    //Serial.println(finalcount, 3);

    EEPROM.put(0, u[0]);
    delay(100);
    EEPROM.put(5, countA1);

    //for evaluating filter: filtered vs just encoder
    /*
    float finalcount1 = ut;    
    usReadings();
    Serial.println(ut, 3);
    Serial.println(finalcount, 3);
    EEPROM.put(0, ut);
    delay(20);
    
    EEPROM.put(5, finalcount);
    */
        
    //Serial.println(" "); 
    //Serial.println(" ");          

    delay(50000);
  }     
}

void usReadings()
{    
  float Mavg = (A1counternoreset+A2counternoreset)/16.3862778735153;
  
  //Serial.println(Mavg, 3);

  //basic kalman filtering implementation testing (1-D, assuming constant v):
  /*
  et1 = Mavg;
  utb = (ut + del);
  ot2b = (ot2 + phi2);
  
  kt1 = ot2b/(ot2b + psi2);
  ut1 = utb + kt1*(et1 - utb);
  o2t1 = (1 - kt1)*(ot2b);
  
  Serial.println(ut, 3);
  Serial.println(ut1, 3);
  Serial.println(et1, 3);
  Serial.println(utb, 3);
  Serial.println(ot2b, 3);
  Serial.println(kt1, 3);
  Serial.println(o2t1, 3);
  Serial.println();

  disArray[locCount] = ut1;
  locCount = locCount + 1;
  
  ut = ut1;
  ot2 = o2t1;
  */  
     
  //deprecated form of measuring distance per timestep: 
  //A1counter = 0; 
  //A2counter = 0;  

  //sending ultrasonic pulse, getting speed measurement at timestep
  digitalWrite(11, LOW);
  delayMicroseconds(5);
  digitalWrite(11, HIGH);
  delayMicroseconds(10);
  digitalWrite(11, LOW);

  duration = pulseIn(12, HIGH);
  float cm = (duration/2) / 29.1545;  
  //how to calculate factor * (cm-lastcm): take refresh rate (time-- 100 ms- change second 1000/refrate)
  USspeed = ((cm - lastcm)/100)*(1000/200);

  if (looper = true){
    USspeed = ((cm - lastcm)/100);
  }
  
  USspeed = (abs(USspeed) * 100);
 
  if((-500 < USspeed) && (USspeed < 500))
  {
    //2d kalman filter implementation:
  //1.
  Matrix.Multiply((mtx_type*)A, (mtx_type*)u, N, N, 1, (mtx_type*)ub);
  Matrix.Print((mtx_type*)ub, N, 1, "ub"); 
  //2.
  Matrix.Transpose((mtx_type*)A, N, N, (mtx_type*)mid);
  Matrix.Multiply((mtx_type*)A, (mtx_type*)sig, N, N, N, (mtx_type*)mid2);
  Matrix.Multiply((mtx_type*)mid2, (mtx_type*)mid, N, N, N, (mtx_type*)sigb);
  sigb[0][1] = 0;
  sigb[1][0] = 0;
  Matrix.Print((mtx_type*)sigb, N, N, "sigb");
  //3.
  Matrix.Add((mtx_type*)sigb, (mtx_type*)psi, N, N, (mtx_type*)mid2);
  Matrix.Print((mtx_type*)mid2, N, N, "denom");
  Matrix.Invert((mtx_type*)mid2, N);
  Matrix.Multiply((mtx_type*)sigb, (mtx_type*)mid2, N, N, N, (mtx_type*)K);
  Matrix.Print((mtx_type*)K, N, N, "K");
  //4.
  et1[0] = Mavg;
  et1[1] = USspeed;
  
  Matrix.Subtract((mtx_type*)et1, (mtx_type*)ub, N, 1, (mtx_type*)mid2);
  Matrix.Print((mtx_type*)et1, N, 1, "et1: ");
  Matrix.Multiply((mtx_type*)K, (mtx_type*)mid2, N, N, 1, (mtx_type*)mid);
  Matrix.Add((mtx_type*)ub, (mtx_type*)mid, N, 1, (mtx_type*)u1);
  Matrix.Print((mtx_type*)u1, N, 1, "u1: ");
    
  //5.
  Matrix.Subtract((mtx_type*)C, (mtx_type*)K, N, N, (mtx_type*)mid);
  Matrix.Multiply((mtx_type*)mid, (mtx_type*)sigb, N, N, N, (mtx_type*)sig1);
  Matrix.Print((mtx_type*)sig1, N, N, "sig1");
  
  //updating for next loop timestep:
  Matrix.Copy((mtx_type*)u1, N, 1, (mtx_type*)u);
  Matrix.Copy((mtx_type*)sig1, N, N, (mtx_type*)sig);    

  //acceleration print experimentation:
  /*
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  Serial.println(accel.x() * 100);
  */
    
  }
  else
  {
   
  }
  lastcm = cm; 

}
/*
float accelSave(){
  //sending ultrasonic pulse, getting speed measurement at timestep
  digitalWrite(11, LOW);
  delayMicroseconds(5);
  digitalWrite(11, HIGH);
  delayMicroseconds(10);
  digitalWrite(11, LOW);

  duration = pulseIn(12, HIGH);
  float cm = (duration/2) / 29.1545;  
  //how to calculate factor * (cm-lastcm): take refresh rate (time-- 100 ms- change second 1000/refrate)
  USspeed = ((cm - lastcm)/100)*(1000/200);
  USspeed = (abs(USspeed) * 100);
  return USspeed;  
}
*/

void Increment1(){
  //A1counter = A1counter + 1; 
  A1counternoreset = A1counternoreset + 1; 
}

void Increment2(){
  //A2counter = A2counter + 1;  
  A2counternoreset = A2counternoreset + 1;
}
