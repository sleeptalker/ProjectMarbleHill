 
  // Setup Radio Control Receiver channel variables
    int rc_channel_1;
 //   int rc_channel_2;
  
    // Setup RC receiver channel input range limits
  //  int rc_channel_lower_limit = 1030;
    //int rc_channel_upper_limit = 1850;
  
    // Setup RC receiver joystick neutral values
    int rc_channel_1_neutral = 1430;
 //   int rc_channel_2_neutral = 1440;


void setup() {

  pinMode(7, INPUT); // Set our input pins as such
 // pinMode(6, INPUT);

  Serial.begin(9600); 

}

void loop() {

    // Read input on RC channels
    rc_channel_1 = pulseIn(7, HIGH, 25000); // Read the pulse width of 
   // rc_channel_2 = pulseIn(6, HIGH, 25000); // each RC channel

    // Constrain reading to 'known' valid signal range
   // rc_channel_1 = constrain(rc_channel_1,rc_channel_lower_limit,rc_channel_upper_limit);
   // rc_channel_2 = constrain(rc_channel_2,rc_channel_lower_limit,rc_channel_upper_limit);

  Serial.println(rc_channel_1);
    
    // Remap RC channel input to 0-100 range - representing % thrust
    //rc_channel_1 = map(rc_channel_1,rc_channel_lower_limit,rc_channel_upper_limit,-100,100);
    //rc_channel_2 = map(rc_channel_2,rc_channel_lower_limit,rc_channel_upper_limit,-100,100);


  Serial.println(rc_channel_1);

  delay(100); 
}
