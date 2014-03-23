/*
  Distance-related blink
   
  Uses the NewPing library for HC-SR04 Ultrasonic 
  distance measuring, and the onboard
  LED to flash at a rate proportional to the distance
  measured.
 */
  
#include <NewPing.h>

 
#define TRIGGER_PIN  12  // Pin 12 to 'trigger'
#define ECHO_PIN     11  // Pin 11 to 'echo'
#define MAX_DISTANCE 2000 // Maximum distance
#define LED 13           // Onboard LED - pin 13
 
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); 
 
 
void setup() {                
  pinMode(LED, OUTPUT); // Initialize the LED pin as an output.
  Serial.begin(115200); // Serial comms set at 115200 baud. 
}
 
 
void loop() {
  delay(50); // Wait 50ms between pings (about 20 pings/sec).
  unsigned int distance = sonar.ping() / US_ROUNDTRIP_CM;
  Serial.println(distance);
   
  digitalWrite(LED, HIGH);   // Turn the LED on
  delay(distance << 1);      // Wait proportional to distance
  digitalWrite(LED, LOW);    // Turn the LED off
  delay(distance << 1);      // Wait again
}
