#include <EEPROM.h>

void setup() {
  // put your setup code here, to run once:

  float f = 0.00f;   //Variable to store data read from EEPROM.
  float f2 = 0.00f;
  float f3 = 0.00f;
  
  int eeAddress = 10; //EEPROM address to start reading from

  Serial.begin(9600);

  
  //use for testing kalman filter implementations (retrieves ut and encval):

  EEPROM.get(0, f);
  Serial.println(f, 3);
  delay(20);
  EEPROM.get(5, f2);
  Serial.println(f2, 3);
  
  

  //for loop that reads all of the EEPROM stored position and velocity values:
  //adjust i < 35 based on number needed to read (if several distances are around 12-13 35 works properly, increase if slower (lesser distances)
  /*
  for (int i = 0; i < 45; i+=2){
  EEPROM.get((i+1)*5, f);
  Serial.println(f, 3);
  
  EEPROM.get((i*5), f2);
  Serial.println(f2, 3);
  
  Serial.println(" ");
  
  }
  */
  
 
  //EEPROM.get(900, f3);
  //Serial.println(f3, 3);

}

void loop() {
  // put your main code here, to run repeatedly:

}
