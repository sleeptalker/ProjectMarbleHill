
#include <LiquidCrystal.h>
#include <NewPing.h>
#include <Wire.h>
#include <FreeSixIMU.h>
#include <FIMU_ADXL345.h>
#include <FIMU_ITG3200.h>
 
#define TRIGGER_PIN  7 // Pin 12 to 'trigger'
#define ECHO_PIN     6  // Pin 11 to 'echo'
#define MAX_DISTANCE 500 // Maximum distance
 
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); 
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

float angles[3]; 
FreeSixIMU sixDOF = FreeSixIMU();


// initialise LCD library and pins
void setup() {
  
  lcd.begin(16, 2);
  
  delay(5);
  sixDOF.init(); 
  delay(5);

}

  
void loop()
{ 
 lcd.setCursor(0,0);
  delay(50); // Wait 50ms between pings (about 20 pings/sec).
  unsigned int distance = sonar.ping_cm();
  lcd.print(distance);
  lcd.print("   cm   ");
 
 lcd.setCursor(0,1); 

  sixDOF.getYawPitchRoll(angles);
  
          int zaxis = (angles[0]);
          int xaxis = (angles[1]);
          int yaxis = (angles[2]);
  
  lcd.print("X=");
  lcd.print(xaxis);
  lcd.print(" Y=");
  lcd.print(yaxis);
  lcd.print(" Z=");
  lcd.print(zaxis);
  
  delay(40);
}
