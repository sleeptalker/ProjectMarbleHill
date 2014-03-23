// Initialize variables, pin modes, include libraries, etc. ------------------ 

  // include I2C library (mainly for 6DOF, barometer and magnetometer sensors)
      #include <Wire.h>

  // Include accelerometer (6DOF) sense/output libraries
      #include <FreeSixIMU.h>
        #include <FIMU_ADXL345.h>
        #include <FIMU_ITG3200.h>
  
  // Include updated Ultrasonic library
      #include <NewPing.h>
  
  // Include thrust motor and servo control library
      #include <Servo.h> 

//Set objects  --------------------------------------------------------------

  // Setup yaw, pitch, roll angle array
    // 0 = pitch = up/down (thrustLR/thrustRF and thrustLA/thrustRA controlled)
    // 1 = yaw = left/right rotate (presently uncontrolled - maybe use torque?)
    // 2 = roll = left/right roll (thrustLF/thrustLA and thrustRF/thrustRA controlled)
    float angles[3]; 

  // Define the FreeSixIMU object
    FreeSixIMU sixDOF = FreeSixIMU();

  // Define main thrust motor objects
    // a maximum of eight motor/servo objects can be created 
    // System is a 4-thrust motor 'X' configuration
    Servo thrustLF; //left-forward   
    Servo thrustLA; //left-aft   
    Servo thrustRF; //right-forward   
    Servo thrustRA; //right-aft   
  
  // Define variables to store the thrust motor adjusted thrust-bias (if needed)
    int thrustLF_bias = 0;   
    int thrustLA_bias = 0;   
    int thrustRF_bias = 0;   
    int thrustRA_bias = 0;   
    
  // Define variables to store the thrust motor output (0 as initial thrust % value - goes to 100)   
    int thrustLF_output = 0;
    int thrustLA_output = 0;
    int thrustRF_output = 0;
    int thrustRA_output = 0;

  // Define Ultrasonic ground height 'sonar' setup
    #define TRIGGER_PIN  7
    #define ECHO_PIN     6
    #define MAX_DISTANCE 500 // Maximum sensing distance in CM
    
    // Define ground height sensor
     NewPing sonar_ground_height(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); 

      // Define minimum 'safe' altitude in CM
      int safe_altitude = 100;
    
      // Define last recorded height in CM to compare against current state - to check for uncontrolled ascent/descent
      int last_recorded_altitude = 0;

      // Define cruise height in CM - used for when auto pilot takes over from manual control
      int cruise_altitude = 0;
    
   // Define general incremental % value to adjust thrust by when correcting attitude and horizon
     int thrust_adjust = 5;
    
  // Setup RC receiver channel variables - presently only 2-channel
    int rc_channel_1;
    int rc_channel_2;
  
      // Setup RC receiver channel input range limits
      int rc_channel_lower_limit = 1030;
      int rc_channel_upper_limit = 1850;
  
      // Setup RC receiver joystick neutral values
      int rc_channel_1_neutral = 1430;
      int rc_channel_2_neutral = 1440;
    
    
// Setup code runs once  ---------------------------------------------------------
void setup() {
  
 // Configure all pin names and outputs
  
  //Begin serial output - for monitoring output on PC
    Serial.begin(9600);
    
  //Begin I2C comms
    Wire.begin();
  
  //Start-up the 6DOF IMU
    delay(5);
    sixDOF.init(); 
    delay(5);

  // Attaches the thrust servos PWM out to specific Arduino pins
    thrustLF.attach(5);  
    thrustLA.attach(11);  
    thrustRF.attach(9);  
    thrustRA.attach(10);
  
 // Setup PWM RC receiver INPUT pins (one pin per channel)
    pinMode(3, INPUT);
    pinMode(6, INPUT);
    
}

// Main program code --------------------------------------------------------------

void loop() {
  
  // Self check
  
  // Spin up delay
  
  // Spin up
  
  // Read for RC interrupt and convert to meaningful signal  
    // Read input on RC channels
    rc_channel_1 = pulseIn(3, HIGH, 25000); // Read the pulse width of 
    rc_channel_2 = pulseIn(6, HIGH, 25000); // each RC channel
    // Constrain reading to 'known' valid signal range
    rc_channel_1 = constrain(rc_channel_1,rc_channel_lower_limit,rc_channel_upper_limit);
    rc_channel_2 = constrain(rc_channel_2,rc_channel_lower_limit,rc_channel_upper_limit);
    // Remap RC channel input to -100 to 100 range - representing % thrust and 'brake'
    rc_channel_1 = map(rc_channel_1,rc_channel_lower_limit,rc_channel_upper_limit,-100,100);
    rc_channel_2 = map(rc_channel_2,rc_channel_lower_limit,rc_channel_upper_limit,-100,100);
    
  // Read current cruise altitude - ULTRASONIC
  delay(50); // Wait 50ms between pings (about 20 pings/sec).
  unsigned int cruise_altitude = sonar_ground_height.ping_cm(); 
          
  // Auto-Pilot Routine
  // If RC inputs for sticks represent 'neutral' position then check and maintain stable horizon and altitude - at 'safe' altitude  
    if (rc_channel_1 == rc_channel_1_neutral && rc_channel_2 == rc_channel_2_neutral)
    {
         
          // Horizon Correction - NEEDS WORK
        
          // Read orientation yaw, pitch, roll angles
            sixDOF.getYawPitchRoll(angles);
            int yaw = (angles[0]); // unsure of use - perhaps replace with magnetometer?
            int pitch = (angles[1]); // 0 - level, 90 vertical climb, -90 dive
            int roll = (angles[2]);  // 0 - level, -90 left roll, 90 right roll
  
          // Adjust thrust output % or 'level' on the four independant motors depending on y/p/r input
          // Only ever DECREASE thrust on corrected motors to ensure system doesn't cause sudden increased altitude
          
            // if pitched upward/climbing angle of attack - decrease forward thrust motor output by thrust_adjust pre-defined %
            if (pitch > 0) {thrustLF_output = thrustLA_output - thrust_adjust; thrustRF_output = thrustRA_output - thrust_adjust;}
         
            // if pitched downward - decrease aft thrust motor output by %
            if (pitch < 0) {thrustLA_output = thrustLF_output - thrust_adjust; thrustRA_output = thrustRF_output - thrust_adjust;}
          
            // if rolled right - decrease left thrust motor output by %
            if (roll > 0) {thrustLF_output = thrustRF_output - thrust_adjust; thrustLA_output = thrustRA_output - thrust_adjust;}
            
            // if rolled left - decrease right thrust motor output by %
            if (roll < 0) {thrustRA_output = thrustLA_output - thrust_adjust; thrustRF_output = thrustLF_output - thrust_adjust;}
 
 
          // Altitude Maintain - NEEDS WORK
          
          // Read current altitude
            delay(50); // Wait 50ms between pings (about 20 pings/sec).
            unsigned int current_altitude = sonar_ground_height.ping_cm();
        
          // Check altitude is above minimum 'safe' altitude - if not then increase power on all motors, equally
          if (current_altitude < safe_altitude) {
            
                thrustRA_output = thrustRA_output + thrust_adjust; 
                thrustRF_output = thrustRF_output + thrust_adjust;
                thrustLF_output = thrustLF_output + thrust_adjust; 
                thrustLA_output = thrustLA_output + thrust_adjust;         
            
          } else {
            
              // if below cruise_altitude (that set when auto-pilot engaged) then increase all thrust motor output
              if (current_altitude < cruise_altitude) {
                thrustRA_output = thrustRA_output + thrust_adjust; 
                thrustRF_output = thrustRF_output + thrust_adjust;
                thrustLF_output = thrustLF_output + thrust_adjust; 
                thrustLA_output = thrustLA_output + thrust_adjust;
               }
          
              // if above cruise_altitude then decrease all thrust motor output
              if (current_altitude > cruise_altitude) {
                thrustRA_output = thrustRA_output - thrust_adjust; 
                thrustRF_output = thrustRF_output - thrust_adjust;
                thrustLF_output = thrustLF_output - thrust_adjust; 
                thrustLA_output = thrustLA_output - thrust_adjust;
              }
             
              // Record last loop's recorded altitude to compare against current altitude
              last_recorded_altitude = current_altitude;
          }
          
    } else {
     
      
    }
  
          // Convert thrust output to PWM signal change
          int thrustLF_PWM = (thrustLF_output+thrustLF_bias);
          int thrustLA_PWM = (thrustLA_output+thrustLA_bias);
          int thrustRF_PWM = (thrustRF_output+thrustRF_bias);
          int thrustRA_PWM = (thrustRA_output+thrustRA_bias);
          
          // Instruct thrust motors to alter output
          thrustLF.write(thrustLF_PWM);
          thrustLA.write(thrustLA_PWM);
          thrustRF.write(thrustRF_PWM);
          thrustRA.write(thrustRA_PWM); 
          
            // Allow spin-change delay to take effect
            delay(100);          
  
  // Maintain height for 5 minutes
  
  
  // Gradual descent - accelerometer feedback to prevent free-fall state


  // Land


  // Spin down


  // Power off  
  
}
