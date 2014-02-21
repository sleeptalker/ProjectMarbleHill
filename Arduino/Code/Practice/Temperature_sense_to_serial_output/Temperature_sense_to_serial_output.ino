int sensorPin = 0; //the analog pin the TMP35's Vout (sense) pin is connected to
                      
void setup()
{
  Serial.begin(9600);  //Start the serial connection with the computer
}
 
void loop()                     // run over and over again
{
 //getting the voltage reading from the temperature sensor
 int reading = analogRead(sensorPin);  
 
 // converting that reading to voltage, for 3.3v arduino use 3.3
 float voltage = reading * 5.0;
 voltage /= 1024.0; 
 
 // print out the voltage
 Serial.print(voltage); Serial.println(" volts");
 Serial.print(reading); Serial.println(" mvolt reading");
 

 // now print out the temperature
 float temperatureC = (5.0 * analogRead(sensorPin) * 100.0) / 1024 ;  //converting from 10 mv per degree to C
 Serial.print(temperatureC); Serial.println(" degrees C");
 
 // now convert to Fahrenheight
 float temperatureF = (temperatureC * 9.0 / 5.0) + 32.0;
 Serial.print(temperatureF); Serial.println(" degrees F");
 
 delay(1000);                                     //waiting a second
}

