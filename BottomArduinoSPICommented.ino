#include <SPI.h>
#include <elapsedMillis.h>

elapsedMillis timeElapsed;

int val;
//Regulates two motors to spin at same speed for encoder of motor one
int encoder0PinA = A1; //J3 motor on board
//int encoder0PinB = 4;
int encoder0Pos = 0; //Motor's angular position read by the encoder
int encoder0PinALast = LOW;

//for encoder for motor two
int encoder1PinA = A2; //J4 motor on board
//int encoder1PinB = 8;
int encoder1Pos = 0;
int encoder1PinALast = LOW;

int encoderCountpRev = 360;
//setpoint is turn rate to compare to/reach
int setpoint = 120; //(degrees/sec) 
double Integral0 = 0; //accumulated error with motors from desired number of turns
double Integral1 = 0; //accumulated error with motors from desired number of turns
int n = LOW;
int m = LOW;

//for driver for IN1 and IN2 for motor one
int motor0pin1 = 2; // J3 on Board //right motors  *******************SWITCH MOTORS IN HARDWARE
int motor0pin2 = 3; //pwm pin
int pwm0 = 80; //123
int digital0 = 1; //0?

//for driver for IN1 and IN2 for motor two
int motor1pin1 = 8; // J4 on Board //left motors
int motor1pin2 = 5; //pwm (controls voltage signal) pin
int pwm1 = 80; //123
int digital1 = 1; //0?

//**line follow vars
int set = 0;
int left_Q = A6;
int right_Q = A3;
int left_calib;
int right_calib;
//***line follow vars
int left_threshold;
int right_threshold;


int left_read;
int right_read;

//CONSTANT TO GET THE REFLECTANCE OF A LINE
int left_line_refl=870;
int right_line_refl=835;

int left_line=false;
int right_line=false;



int encoder0PrevCount = 0;
int lastSpeed0 = 0;
int encoder1PrevCount = 0;
int lastSpeed1 = 0;

double timeSec = .5;
//PID constants
//P (proportional) is how much to adjust when turn rate is not equal to set rate. Matters most.
double kP = 0.25;//0.20 or .15
//I (integral) is how much to adjust based on accumulated error
double kI = 0.2;//0.01 or .05
//D (derivative) how quickly it deviates from set rate. Adjusts quicker for greater rates
double kD = 0.211;//0.01 or .01


int test;
//char buff [50]; Use multiple parameters
char updated;
volatile byte indx;
volatile boolean process;
int  interruptPin = 10;

int IRPin = 4; //S4 on J5
int in; 
int trigPin = 9; //J10 on board
int echoPin = A3; //this is the ADC pin
long duration,cm;


void setup() {  

  pinMode (encoder0PinA, INPUT);
  //  pinMode (encoder0PinB, INPUT);
  pinMode (motor0pin1, OUTPUT);
  pinMode (motor0pin2, OUTPUT);

  pinMode (encoder1PinA, INPUT);
  //pinMode (encoder1PinB, INPUT);
  pinMode (motor1pin1, OUTPUT);
  pinMode (motor1pin2, OUTPUT);

  Serial.begin(115200);
  pinMode(MISO,OUTPUT); //init spi
  //pinMode(3,OUTPUT);

  pinMode(20,OUTPUT);
  pinMode(22,OUTPUT);
  
//linefollow***************
  left_calib=analogRead(left_Q);
  right_calib=analogRead(right_Q);

  left_threshold = abs((left_calib-left_line_refl)/2);
  right_threshold = abs((right_calib-right_line_refl)/2);
//linefollow***************
  
  SPCR |= bit (SPE); // slave control register
  indx = 0; //buffer empty
  process = false;

  
  pinMode(interruptPin,INPUT);
  int val = digitalRead(interruptPin);

  delay(1000);
  SPI.attachInterrupt();
 // attachInterrupt(digitalPinToInterrupt(interruptPin), tester, LOW); //enable interrupt
  //val = 1;

  
  pinMode(IRPin, INPUT); 
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  test = 0;
  }

ISR (SPI_STC_vect) { //SPI Interrupt Service Routine
 // digitalWrite(3,1);
  Serial.println("entered ISR"); //debugging print line to indicate the beginning of the interrupt sequence
  if (SPDR != c)  //if the new value does not equal the value already contained in SPDR
    Serial.println("Value has been changed"); //debugging print line to show value has changed successfully
  byte c = SPDR; //read byte from SPI data register
  updated = c;// save data in the next index in the array buff
  Serial.print("ISR Value: ");
  Serial.println(updated); //debugging statement to check if value has been changed successfully
 // if (c == '\r') //check for the end of the word
  process = true;
  
}  

//Adjust PWM FOR PID CODE
void adjustPWM() {
  int speedNow0 = calculateSpeed0();
  int error0 = setpoint - speedNow0;
  double dError0 = ((double)speedNow0 - (double)lastSpeed0) / timeSec;
  Integral0 += (double) error0;

  int speedNow1 = calculateSpeed1();
  int error1 = setpoint - speedNow1;
  double dError1 = ((double)speedNow1 - (double)lastSpeed1) / timeSec;
  Integral1 += (double) error1;

  if (Integral0 > 255) Integral0 = 255;
  else if (Integral0 < 0) Integral0 = 0;
//  Serial.print("Integral0: ");
//  Serial.println( Integral0);

  if (Integral1 > 255) Integral1 = 255;
  else if (Integral1 < 0) Integral1 = 0;
//  Serial.print("Integral1: ");
//  Serial.println( Integral1);

  int adjust0 = (kP * (double)error0) + kI * Integral0 + kD * dError0;
  int adjust1 = (kP * (double)error1) + kI * Integral1 + kD * dError1;

  if (digital0 == 0) pwm0 += adjust0;
  else pwm0 -= adjust0;

  if (digital1 == 0) pwm1 += adjust1;    
  else pwm1 -= adjust1;

  if (pwm0 > 255) pwm0 = 255;
  else if (pwm0 < 0) pwm0 = 0;
  
  if (pwm1 > 255) pwm1 = 255;
  else if (pwm1 < 0) pwm1 = 0;

  lastSpeed0 = speedNow0;
  lastSpeed1 = speedNow1;
//
//  Serial.print("adjustment0: ");
//  Serial.println( adjust0);
//  Serial.print("PWM0: ");
//  Serial.println( pwm0 );
//
//  Serial.print("adjustment1: ");
//  Serial.println( adjust1);
//  Serial.print("PWM1: ");
//  Serial.println( pwm1 );
}


int calculateSpeed0() {
  int speedDetect = (encoder0Pos - encoder0PrevCount) / timeSec;
//  Serial.print("Encoder0pos: ");
//  Serial.print( encoder0Pos );
//  Serial.print("  ");
//  Serial.println( encoder0PrevCount);
  encoder0PrevCount = encoder0Pos;
//  Serial.print( "Speed0: ");
//  Serial.println( speedDetect);
  return speedDetect;
}

int calculateSpeed1() {
  int speedDetect = (encoder1Pos - encoder1PrevCount) / timeSec;
//  Serial.print("Encoder0pos: ");
//  Serial.print( encoder1Pos);
//  Serial.print("  ");
//  Serial.println( encoder1PrevCount);
  encoder1PrevCount = encoder1Pos;
//  Serial.print( "Speed1: ");
//  Serial.println( speedDetect);
  return speedDetect;
}

void PID() {
   if (digital0 == 1)
    digitalWrite( motor0pin1, HIGH);
  else digitalWrite( motor0pin1, LOW);
  analogWrite( motor0pin2, pwm0);

  if (digital1 == 1)
    digitalWrite( motor1pin1, HIGH);
  else digitalWrite( motor1pin1, LOW);
  analogWrite( motor1pin2, pwm1);


  timeElapsed = 0;
  //calculateSpeed();
  while ( timeElapsed < 500 ) {

    n = digitalRead(encoder0PinA);
    if ((encoder0PinALast == LOW) && (n == HIGH)) {
      encoder0Pos++;
    }
    //Serial.print (encoder0Pos);
    //Serial.print ("/");

    encoder0PinALast = n;

    m = digitalRead(encoder1PinA);
    if ((encoder1PinALast == LOW) && (m == HIGH)) {
      encoder1Pos++;
    }
    // Serial.print (encoder1Pos);
    //Serial.print ("/");

    encoder1PinALast = m;

  }

  //unsigned long CurrentTime = millis();
  //unsigned long ElapsedTime = CurrentTime - StartTime;
  timeSec = 1.0 ;//(double)( ElapsedTime * .001);
//  Serial.print( "Time: ");
//  Serial.println(timeSec); // time needs to be fixed
  adjustPWM();
//  Serial.println(" ");

}

void moveForward() {  //forward code used in loop
//  Serial.println("forward code");
    //low is nearby, high is far
  in = digitalRead(IRPin);
//  Serial.println(in);
  digitalWrite(trigPin,LOW);
  delayMicroseconds(30);
  digitalWrite(trigPin,HIGH);
  delayMicroseconds(30);
  pinMode(echoPin,INPUT);
  duration = pulseIn(echoPin,HIGH);
  //convert the time into distance: 29ms per cm
  cm = (duration/2)/29.1;
  Serial.println(cm);
  if (in == 1 || cm < 30){
    //stop
    digitalWrite(motor0pin2, HIGH);//1 high 2 low is clockwise
    digitalWrite(motor0pin1, LOW);
    digitalWrite(motor1pin2, HIGH);
    digitalWrite(motor1pin1, LOW);
    delay(400);
    digitalWrite(motor0pin2, LOW);
    digitalWrite(motor1pin2, LOW);
    delay(1000);
//    Serial.println("stop");
  }
  else {
    //move
    digital0 = 1;
    digital1 = 1;
    PID();
//    digitalWrite(motor0pin2, LOW);//1 high 2 low is clockwise
//    digitalWrite(motor0pin1, HIGH);
//    digitalWrite(motor1pin2, LOW);
//    digitalWrite(motor1pin1, HIGH);
    //delay(400);
//    Serial.println("move");
  }
  //check again
   if (in == 1 || cm < 10){
   if( in == 1)
//    Serial.println("IR detects");
    //stop
    digitalWrite(motor0pin2, HIGH);//1 high 2 low is clockwise
    digitalWrite(motor0pin1, LOW);
    digitalWrite(motor1pin2, HIGH);
    digitalWrite(motor1pin1, LOW);
    delay(400);
    digitalWrite(motor0pin2, LOW);
    digitalWrite(motor1pin2, LOW);
    delay(1000);
//    Serial.println("stop");
  } 
}
//***Line follow functions***
void stop(){
  digitalWrite(motor0pin1,LOW);
  digitalWrite(motor1pin1,LOW);
  analogWrite(motor0pin2,0);
  analogWrite(motor1pin2, 0);
}
void veer_right(){
  digitalWrite(motor0pin1,HIGH);
  digitalWrite(motor1pin1,HIGH);
  analogWrite(motor0pin2,140);
  analogWrite(motor1pin2, 50);
}

void veer_left(){
  digitalWrite(motor0pin1,HIGH);
  digitalWrite(motor1pin1,HIGH);
  analogWrite(motor0pin2,50);
  analogWrite(motor1pin2, 140);
}
void readSensors(){
  left_line= (((analogRead(left_Q)-left_line_refl)<left_threshold) ||(left_line_refl-analogRead(left_Q))>left_threshold);
  right_line=((analogRead(right_Q)-right_line_refl<right_threshold) || (left_line_refl-analogRead(right_Q))>right_threshold);
}
void drive_forward() {
  digitalWrite(motor0pin1,HIGH); 
  digitalWrite(motor1pin1,HIGH);
  analogWrite(motor0pin2,75);
  analogWrite(motor1pin2, 75);
}

void LineFollow() {
  if (set == 0) {
    left_calib=analogRead(left_Q);
    right_calib=analogRead(right_Q);

    left_threshold = abs((left_calib-left_line_refl)/2);
    right_threshold = abs((right_calib-right_line_refl)/2);
//    Serial.println(left_calib);
//    Serial.println(right_calib);
//    Serial.println(left_threshold);
//    Serial.println(right_threshold);
  }
  set++;  
    readSensors();
//    Serial.print("LEFT SENSOR: ");
//    Serial.println(analogRead(left_Q));
//    Serial.print("RIGHT SENSOR: ");
//    Serial.println(analogRead(right_Q));
    delay(20);
 
   if(!left_line && !right_line){
      drive_forward();
//      Serial.println("forward no line");
    }
   else if(!left_line && right_line){
      veer_left();
//      Serial.println("veer left");
    }
    else if(left_line && !right_line){
      veer_right();
//      Serial.println("veer right");
    }
    else {
      drive_forward();
//      Serial.println("forward else");
    }
}

//**Line follow functions***


void loop() {
//  moveForward();
//  Serial.println(" test");
  if (process) {  //process is true if interrupt is received this loop used to reset interrupt
//    buff[indx] = 0; Use later for multiple parameters
    process = false; //reset flag
//    digitalWrite(3,0);
 //   Serial.println("work");
//    Serial.print(buff); //print to serial monitor
//    int i = 0;
   
//    if (i < sizeof(buff)) {  //Multiple parameteres
    Serial.println(updated); //debugging statement prints the value of updated for inspection
    switch(updated) { //function changes the letter value of updated to a command
      case 'F' : //fwd
//          Serial.println("moving forward");
        
        moveForward(); //runs move forward function from above
        //digitalWrite(motor0pin2, LOW);
        //digitalWrite(motor0pin1, HIGH);
        //digitalWrite(motor1pin2, LOW);
        //digitalWrite(motor1pin1, HIGH);
        set = 0;
        //delay(6000);
        break; //breaks out of the switch loop and continues the original search
      case 'B' : //Backwards (back())
//          Serial.println("back");
        digital0 = 0;
        digital1 = 0;
        set = 0;
        PID();
       // delay(6000);
        break; //breaks out of the switch loop and continues the original search
      case 'L' : //left
//          Serial.println("Left");
        digitalWrite(motor0pin2, LOW);
        digitalWrite(motor0pin1, HIGH);
        digitalWrite(motor1pin2, HIGH);
        digitalWrite(motor1pin1, LOW);
        set = 0;
        //delay(6000);
        break; //breaks out of the switch loop and continues the original search
      case 'R' : //right
        digitalWrite(motor0pin2, HIGH);
        digitalWrite(motor0pin1, LOW);
        digitalWrite(motor1pin2, LOW);
        digitalWrite(motor1pin1, HIGH);
        set = 0;
        //delay(6000);
        break; //breaks out of the switch loop and continues the original search
      case 'S' : //stop, makes all pins low
        digitalWrite(motor0pin2, LOW);
        digitalWrite(motor0pin1, LOW);
        digitalWrite(motor1pin2, LOW);
        digitalWrite(motor1pin1, LOW);
        //delay(6000);
        set = 0;
        break; //breaks out of the switch loop and continues the original search
      case 'T' : //Line Follow mode
        LineFollow(); //starts line follow 
        break; //breaks out of the switch loop and continues the original search
//       case 'M' : //move servo
      default: //code run when none of the cases are met
        set = 0;
        //i++;
        break; //breaks out of the switch loop and continues the original search
    }
   // }
  }
  //TODO Case where switching functions 
    //SPDR = data  //sends value to master via SPDR
    //indx = 0; //reset button to zero
}
