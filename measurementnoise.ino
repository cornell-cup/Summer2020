#include <MatrixMath.h>
#include <Event.h>
#include <Timer.h>

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
float disArray[25];
float spArray[25];
int locCount = 0; 

//max distance bot travels
const int maxdistcm = 150;

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

  //Flash LED to signal ready to read data
  pinMode(13, OUTPUT);

  //Pin 3: corresponds to A1counter
  attachInterrupt(1, Increment1, CHANGE);
  //Pin 2: corresponds to A2counter
  attachInterrupt(0, Increment2, CHANGE);
  
  digitalWrite(9, LOW);
  delay(16000);

  //find out precise time it takes for robot to *just* begin moving
  digitalWrite(9, HIGH);
  //insert the measured time in this delay:
  delay(150);

  t.every(200, usReadings);
  
}

void loop() {
  //previous wheel diameter constants used:
  //prev: 16.399173805597
  //trying: 16.3862778735153
  
  float countA1 = (A1counternoreset+A2counternoreset)/16.3862778735153;
  
  //float countA1 = (ut);
  //Serial.println(ut, 3);  
  if (countA1 < (maxdistcm)) {
    t.update();    
    //Serial.println(countA1);
  }
  else {  
    digitalWrite(9, LOW);
    //appending final data:
    
    delay(200);
    //remember to put finalcount float after usReadings call to get correct distance
    usReadings();
    delay(200);
    usReadings();
    delay(1000);
    usReadings();
    
    //Serial.println(" "); 
    //Serial.println(" ");          

    //store data from distance and speed arrays to EEPROM (diag testing):
    
    int j = 0;   
    for (int i = 0; i < sizeof(disArray); i++){
      if(disArray[i] != 0.000 && spArray[i] != 0.000) {  
              
        if(disArray[i] != 0.000){
          EEPROM.put((j+1)*5, disArray[i]);
          delay(10);
          EEPROM.put((j*5), spArray[i]);
          j = j + 2;
          delay(10);
        }
    }
    } 
        
      
      delay(50000);
  }     
}

void usReadings()
{    
  float Mavg = (A1counternoreset+A2counternoreset)/16.3862778735153;
  
  Serial.println(Mavg, 3);
   
  disArray[locCount] = Mavg;  
  
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
  USspeed = (abs(USspeed) * 100);
 
  if((-500 < USspeed) && (USspeed < 500))
  {
  //Serial.println(USspeed, 3);
  spArray[locCount] = USspeed;
  locCount = locCount + 1;
  Serial.println(" ");
  }
  else
  {
   
  }
  lastcm = cm;   
  
}

void Increment1(){
  //A1counter = A1counter + 1; 
  A1counternoreset = A1counternoreset + 1; 
}

void Increment2(){
  //A2counter = A2counter + 1;  
  A2counternoreset = A2counternoreset + 1;
}
