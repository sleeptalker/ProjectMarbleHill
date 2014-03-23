// ********************************************************************************
// Initialize variables, pin modes, include libraries, etc. -----------------------
// ********************************************************************************

  // include I2C library (mainly for 6DOF, barometer and magnetometer sensors)
      #include <Wire.h>

  // Include accelerometer (6DOF) sense/output libraries
      #include <FreeSixIMU.h>
        #include <FIMU_ADXL345.h>
        #include <FIMU_ITG3200.h>
   
  // Include thrust motor and servo control library
      #include <Servo.h> 
  
  // Include PID library
      #include <PID_v1.h>

//Set objects  --------------------------------------------------------------

  // Setup yaw, pitch, roll angle array
    // 0 = pitch = up/down (thrustLR/thrustRF and thrustLA/thrustRA controlled)
    // 1 = yaw = left/right rotate (presently uncontrolled - use torque to control?)
    // 2 = roll = left/right roll (thrustLF/thrustLA and thrustRF/thrustRA controlled)
    float angles[3]; 
    // Define the FreeSixIMU object and variables
    FreeSixIMU sixDOF = FreeSixIMU();

  // Define main thrust motor objects
    // a maximum of eight motor/servo objects can be created 
    // System is a 4-thrust motor 'X' configuration
    Servo thrustLF; //left-forward   
    Servo thrustLA; //left-aft   
    Servo thrustRF; //right-forward   
    Servo thrustRA; //right-aft       

  // Define variables to store the thrust motor output (0 as initial thrust % value - goes to 100)   
    int thrustLF_PWM = 0;
    int thrustLA_PWM = 0;
    int thrustRF_PWM = 0;
    int thrustRA_PWM = 0;

  // Define minimum 'safe' altitude in CM
    int altitude_safe = 100;
  // Define variable for current altitude
    int altitude_current = 0;
    
  // Setup RC receiver channel variables - presently only 2-channel
      int rc_channel_1;
      int rc_channel_2;
      
      // Setup RC receiver channel input range limits
      int rc_channel_lower_limit = 1030;
      int rc_channel_upper_limit = 1850;
      
      // Setup RC receiver joystick neutral values
      int rc_channel_1_neutral = 1430;
      int rc_channel_2_neutral = 1440;

  // PID Adaptive Tuning Setup
    //Define aggressive and conservative Tuning Parameters
    double aggKp=4, aggKi=0.2, aggKd=1;
    double consKp=1, consKi=0.05, consKd=0.25;

    //Define Variables we'll be connecting to for 6DOF X (pitch) axis
    double Setpoint_xaxis, InputX, PID_Output_Xaxis;
    //Specify the links and initial tuning parameters
    PID X_axis_PID(&InputX, &PID_Output_Xaxis, &Setpoint_xaxis, consKp, consKi, consKd, DIRECT);

    //Define Variables we'll be connecting to for 6DOF Y (roll) axis
    double Setpoint_yaxis, InputY, PID_Output_Yaxis;
    //Specify the links and initial tuning parameters
    PID Y_axis_PID(&InputY, &PID_Output_Yaxis, &Setpoint_yaxis, consKp, consKi, consKd, DIRECT);
    
    //Define Variables we'll be connecting to for barometer-measured altitude
    double Setpoint_cruise_altitude, InputAltitude, PID_Output_Altitude;
    //Specify the links and initial tuning parameters
    PID Altitude_PID(&InputAltitude, &PID_Output_Altitude, &Setpoint_cruise_altitude, consKp, consKi, consKd, DIRECT);

// ********************************************************************************
// Setup code runs once  ----------------------------------------------------------
// ********************************************************************************

void setup() {
// Configure all pin names and outputs  
  
// ##############################################
// TEMP VARIABLE - Simulates Thrust at 50% for both axial controls
// PID_Output_Xaxis = 50.00;
// PID_Output_Yaxis = 50.00;
// ##############################################  

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
  
 // PID enable and configure ranges and modes
    X_axis_PID.SetMode(AUTOMATIC);
    Y_axis_PID.SetMode(AUTOMATIC);
    Altitude_PID.SetMode(AUTOMATIC);
  
    // PID Output permitted range
    // -100 is used for 'negative' thrust, 100 used for 'postive' thrust
    X_axis_PID.SetOutputLimits(-100,100);
    Y_axis_PID.SetOutputLimits(-100,100);
    Altitude_PID.SetOutputLimits(-100,100);
  
    // PID initialize the setpoint ('target') variables for each axis
    // 0 represent 'stable' axial position
    Setpoint_xaxis = 0.00;
    Setpoint_yaxis = 0.00;
    Setpoint_cruise_altitude = 0.00;  

}    
    
// ********************************************************************************
// Main program code --------------------------------------------------------------
// ********************************************************************************

void loop() {
  
  // Self check
  
  // Spin up delay
  
  // Spin up
  
  // Read for RC input (ideally an interrupt) and convert to meaningful signal  

    // Read input on RC channels
    rc_channel_1 = pulseIn(3, HIGH, 25000); // Read the pulse width of 
    rc_channel_2 = pulseIn(6, HIGH, 25000); // each RC channel

    // Constrain reading to 'known' valid signal range
    rc_channel_1 = constrain(rc_channel_1,rc_channel_lower_limit,rc_channel_upper_limit);
    rc_channel_2 = constrain(rc_channel_2,rc_channel_lower_limit,rc_channel_upper_limit);

    // Remap RC channel input to -100 to 100 range - representing % thrust
    rc_channel_1 = map(rc_channel_1,rc_channel_lower_limit,rc_channel_upper_limit,-100,100);
    rc_channel_2 = map(rc_channel_2,rc_channel_lower_limit,rc_channel_upper_limit,-100,100);
    
  // Read current cruise altitude
//            altitude_current = BAROMETER INPUT in CM or M;
          
  // If RC inputs for sticks represent 'neutral' position then check and maintain stable horizon and altitude - at 'safe' altitude  
//    if (rc_channel_1 == rc_channel_1_neutral && rc_channel_2 == rc_channel_2_neutral)
//    {
         // ************************************************
         // Auto-Pilot Routine
         // ************************************************
          
         // Enable AUTOPILOT
         // PID - Change mode to 'AUTOMATIC' to allow autopilot control
         X_axis_PID.SetMode(AUTOMATIC);
         Y_axis_PID.SetMode(AUTOMATIC);
         Altitude_PID.SetMode(AUTOMATIC);
          
          // Read orientation yaw, pitch, roll angles
            sixDOF.getYawPitchRoll(angles);
                int yaw = (angles[0]); // unsure of use - perhaps replace with magnetometer?
                int pitch = (angles[1]); // Reports = 0 - level, 90 vertical climb, -90 dive
                int roll = (angles[2]);  // Reports = 0 - level, -90 left roll, 90 right roll
            
                // X (pitch) axis PID compute
                double gap = abs(Setpoint_xaxis-pitch); //distance away from setpoint
                if(gap<10)
                  {  
                     //we're close to setpoint, use conservative tuning parameters
                     X_axis_PID.SetTunings(consKp, consKi, consKd);
                  }
                  else
                  {
                     //we're far from setpoint, use aggressive tuning parameters
                     X_axis_PID.SetTunings(aggKp, aggKi, aggKd);
                  }
      
                  X_axis_PID.Compute();
                  
                     
                // Y (roll) axis PID compute
                gap = abs(Setpoint_yaxis-roll); //distance away from setpoint
                if(gap<10)
                  {  
                     //we're close to setpoint, use conservative tuning parameters
                     Y_axis_PID.SetTunings(consKp, consKi, consKd);
                  }
                  else
                  {
                     //we're far from setpoint, use aggressive tuning parameters
                     Y_axis_PID.SetTunings(aggKp, aggKi, aggKd);
                  }
      
                  Y_axis_PID.Compute();           

        
          // Read current altitude
//            unsigned int altitude_current = BAROMETER INPUT in CM or M;

                // Altitude PID compute
                gap = abs(Setpoint_cruise_altitude-altitude_current); //distance away from setpoint
                if(gap<10)
                  {  //we're close to setpoint, use conservative tuning parameters
                     Altitude_PID.SetTunings(consKp, consKi, consKd);
                  }
                  else
                  {
                     //we're far from setpoint, use aggressive tuning parameters
                     Altitude_PID.SetTunings(aggKp, aggKi, aggKd);
                  }
      
                  Altitude_PID.Compute();

                // Assign axial feedback to PID input function
                InputX = pitch;
                InputY = roll;
                InputAltitude = altitude_current;

                // Map -100 to +100 PID output to appropriate motor control and change
                
                // If in nose-dive
                if (InputX > 0) {
                
                // If in climb  
                }else{
                  
                }
                
                // If in left roll
                if (InputY > 0) {
                
                // If in right roll  
                }else{
                  
                }
                
                
                if (InputAltitude > 0) {
                  
                }else{
                  
                }
                


// ######################################
// Output to serial - DEBUGGING
//Serial.print(PID_Output_Xaxis);
//Serial.print("/");
//Serial.print(pitch);
//Serial.print("  ");
//Serial.print(PID_Output_Yaxis);
//Serial.print("/");
//Serial.print(roll);
//Serial.print("  ");
//Serial.print(PID_Output_Altitude);
//Serial.print("/");
//Serial.print(altitude_current);
//Serial.println(",");
// ######################################
       
//    } else {     
 
         // ************************************************
         // RC Routine
         // ************************************************
            
         // Disable AUTOPILOT
         // PID - Change mode to 'MANUAL' to allow direct RC override
         X_axis_PID.SetMode(MANUAL);
         Y_axis_PID.SetMode(MANUAL);
         Altitude_PID.SetMode(MANUAL);

         // Read RC inputs for manual flight  

//    }
 
 
 // ************************************************
 // Common to BOTH Auto Pilot and RC routines
 // Convert thrust output to PWM signal change
 // ************************************************     
          
 // Instruct thrust motors to alter output to new PWM values
 thrustLF.write(thrustLF_PWM);
 thrustLA.write(thrustLA_PWM);
 thrustRF.write(thrustRF_PWM);
 thrustRA.write(thrustRA_PWM);         
  
 // Maintain height for 5 minutes
  
  
   // Gradual descent - accelerometer feedback to prevent free-fall state


   // Land


   // Spin down


   // Power off  
  
}
