/*
Project Marble Hill
2013
Anders Reeves

// #############################################
NOTES:

Connect the Arduino's pin labeled "GND" to the ESCs BLACK cable. 
Connect the Arduino's PWM pin output to ESCs WHITE cable.
DO NOT connect the Arduino pin labelled 5V to the ESC at all.


// #############################################
Current Testing Tweaks:



// #############################################
To Do:

Need to add YAW (z-axis) sense to 6DOF output by including Magnetometer reading
  UNcomment out HMC58x3 part to 6DOF lib files
  http://www.dfrobot.com/wiki/index.php/10_DOF_Sensor_(SKU:SEN0140)

Need to write alt routine that sets "Setpoint_cruise_altitude" to where manual RC input stopped and autopilot took over, or if no manual then set to "altitude_safe" at very least

Need an RC signal 'lost' routine

Can we use accelerometer to show up/down motion - or at least do a free-fall interrupt sense?

Need to add emergency cut-off procedure on "throttle cut" button press on RC transmitter (but do have engine_arm switch already)

Battery sense to force landing on low Ve sense

// #############################################
Futures:

Record all vars to onboard SD-CARD

GPS

Add video camera

*/


// ********************************************************************************
// Initialize variables, pin modes, include libraries, etc. -----------------------
// ********************************************************************************

  // include I2C library (mainly for 6DOF, barometer and (potentially) magnetometer sensors)
      #include <Wire.h>

  // Include accelerometer (6DOF) sense/output libraries
      #include <FreeSixIMU.h>
        #include <FIMU_ADXL345.h>
        #include <FIMU_ITG3200.h>
   
  // Include thrust motor and servo control library
      #include <Servo.h> 
  
  // Include PID library
      #include <PID_v1.h>

  // Include Barometric BMP085 library
      #include <Adafruit_BMP085.h>

//Set objects  --------------------------------------------------------------

  // ########## 6DOF - ORIENTATION Setup -------------------------
    // Setup yaw, pitch, roll angle array
    // 0 = pitch = up/down 
    // 1 = yaw = left/right rotate
    // 2 = roll = left/right roll
    float angles[3]; 
    // Define the FreeSixIMU object and variables
    FreeSixIMU sixDOF = FreeSixIMU();

  // ########## Altitude/Barometric Setup ------------------------
    Adafruit_BMP085 BMP085_Alt;
    // Safe altitude in CMs    
    int altitude_safe = 200;
    int ground_altitude;
    // Adjust this variable to present location's ambient pressure in pascals (millibars * 1000)
    long current_pascals = 101900;
        #define numHistory 20 // how many samples of moving-average to use - more gives slower response
        int history[numHistory];
        int historyPtr = -1; // 0 to numHistory-1. -1 = not yet used.
        long historyTotal;

  // ########## PWM and MOTOR Setup ------------------------------
  // Define main thrust motor objects
    // a maximum of eight motor/servo objects can be created 
    // System is a 4-thrust motor 'X' configuration
    Servo thrustLF; //left-forward   
    Servo thrustLA; //left-aft   
    Servo thrustRF; //right-forward   
    Servo thrustRA; //right-aft       

  // Define variables to store the thrust range and motor output supported by servo library and ESCs - ESCs should be tested to see min/max response thresholds
  // All subsequent thrust commands are expressed as % thrust from 0-100 for each motor using a mapping based on the range of these two limits at the end of the loop
    int thrust_minimum_PWM = 60;
    int thrust_maximum_PWM = 160;
    
    // Setup each motor's PWM/thrust input variable
    int thrustLF_PWM;
    int thrustLA_PWM;
    int thrustRF_PWM;
    int thrustRA_PWM;

  // ########## Radio Control Setup  ---------------------------
  // Setup RC receiver channel variables - presently 6-channel
      // rc_channel_1 = altitude/throttle
      // rc_channel_2 = roll
      // rc_channel_3 = pitch
      // rc_channel_4 = yaw
      // rc_channel_5 = engines_armed on/off
      // rc_channel_6 = Autopilot on/off
      
          // Setup Channel variable
          int rc_channel_1;
          int rc_channel_2;
          int rc_channel_3;
          int rc_channel_4; 
          int rc_channel_5; 
          int rc_channel_6; 
          
          // Setup Channe PIN input
          int rc_channel_1_pin = 4;
          int rc_channel_2_pin = 5;
          int rc_channel_3_pin = 6;
          int rc_channel_4_pin = 7;
          int rc_channel_5_pin = 8;
          int rc_channel_6_pin = 12;
      
      // Setup RC receiver channel input range limits - RC transmitter channels should be tested to see min/max response thresholds
      int rc_channel_lower_limit = 1090;
      int rc_channel_upper_limit = 1910;
      
      // Setup RC receiver joystick neutral values for all 4 axes plus 2 aux inputs - RC transmitter channels should be tested to see min/max response thresholds
      int rc_channel_1_neutral = 1077;
      int rc_channel_2_neutral = 1497;
      int rc_channel_3_neutral = 1497;
      int rc_channel_4_neutral = 1497;
      int rc_channel_5_neutral = 1077; // 'neutral' is 'off' position of this switch
      int rc_channel_6_neutral = 1497; // 'neutral' is 'off' position of this switch


  // ########## PID Input/Output Tuning setup  ----------------
    //Define aggressive and conservative Tuning Parameters - these can be adjusted if PID corrections are too harsh/relaxed
    // Aggresive
    double aggKp=4, aggKi=0.2, aggKd=1;
    // Conservative
    double consKp=1, consKi=0.05, consKd=0.25;

    //Define Variables we'll be connecting to for 6DOF X (pitch) axis
    double Setpoint_xaxis, InputX, PID_Output_Xaxis;
    //Specify the links and initial tuning parameters
    PID X_axis_PID(&InputX, &PID_Output_Xaxis, &Setpoint_xaxis, consKp, consKi, consKd, DIRECT);

    //Define Variables we'll be connecting to for 6DOF Y (roll) axis
    double Setpoint_yaxis, InputY, PID_Output_Yaxis;
    //Specify the links and initial tuning parameters
    PID Y_axis_PID(&InputY, &PID_Output_Yaxis, &Setpoint_yaxis, consKp, consKi, consKd, DIRECT);

    //Define Variables we'll be connecting to for 6DOF Z (yaw) axis
    double Setpoint_zaxis, InputZ, PID_Output_Zaxis;
    //Specify the links and initial tuning parameters
    PID Z_axis_PID(&InputZ, &PID_Output_Zaxis, &Setpoint_zaxis, consKp, consKi, consKd, DIRECT);
    
    //Define Variables we'll be connecting to for barometer-measured altitude
    double Setpoint_cruise_altitude, InputAltitude, PID_Output_Altitude;
    //Specify the links and initial tuning parameters
    PID Altitude_PID(&InputAltitude, &PID_Output_Altitude, &Setpoint_cruise_altitude, consKp, consKi, consKd, DIRECT);
    
  //  ########## Setup Autopilot status variable - OFF by default
  int Autopilot_status = 0;

  //  ########## Setup 'engines armed' variable - OFF by default
  int engines_armed = 0;

// ********************************************************************************
// Setup code runs once  ----------------------------------------------------------
// ********************************************************************************

void setup() {
// Configure all pin names and outputs  

  //Begin serial output - for monitoring output on PC
    Serial.begin(9600);
    
  //Begin I2C comms for 6DOF and Barometer
    Wire.begin();
  
  //Start-up the 6DOF IMU circuitry
    delay(5);
      sixDOF.init(); 
    delay(5);

  //Start-up the Barometer circuitry
    // 0-3 levels set sensitivity of sensor; more sensitive (3) is slower to respond
    BMP085_Alt.begin(3);    
  
 // ########## Radio Control PIN setup   
 // Setup RC receiver INPUT pins (one pin input per channel - until I work out how to PPM instead of PWM)
    pinMode(rc_channel_1_pin, INPUT);
    pinMode(rc_channel_2_pin, INPUT);
    pinMode(rc_channel_3_pin, INPUT);
    pinMode(rc_channel_4_pin, INPUT); 
    pinMode(rc_channel_5_pin, INPUT); 
    pinMode(rc_channel_6_pin, INPUT);    
    
 // ########## PID axis setup 
 // PID enable and configure ranges and modes
    // Setup PID to ENABLE PID Autopilot as startup mode
    X_axis_PID.SetMode(AUTOMATIC);
    Y_axis_PID.SetMode(AUTOMATIC);
    Z_axis_PID.SetMode(AUTOMATIC);
    Altitude_PID.SetMode(AUTOMATIC);
  
    // PID Output permitted range
    // Concept is that PID output is a % change to current thrust of any motor
    // Altitude affects all motors equally while yaw, pitch and roll are treated as 'incremental' deltas - all treated as % values for thrust
    X_axis_PID.SetOutputLimits(-10,10);
    Y_axis_PID.SetOutputLimits(-10,10);
    Z_axis_PID.SetOutputLimits(-10,10);
    Altitude_PID.SetOutputLimits(40,90);
  
    // PID initialize the setpoint ('target') variables for each axis
    // 0.00 represent 'stable'/'flat'/head-on axial position as reported by 6DOF and Magnetometer
    Setpoint_xaxis = 0.00;
    Setpoint_yaxis = 0.00;
    Setpoint_zaxis = 0.00;

  // ########## Motor and ESC setup
  // Attaches the thrust servos PWM outpuy to specific Arduino pins
    thrustLF.attach(3);  
    thrustLA.attach(11);  
    thrustRF.attach(9);  
    thrustRA.attach(10);
    
    //Send all ESCs min value - helps prime ESCs when powered on
    thrustLF_PWM = thrust_minimum_PWM;
    thrustLA_PWM = thrust_minimum_PWM;
    thrustRF_PWM = thrust_minimum_PWM;
    thrustRA_PWM = thrust_minimum_PWM;
       thrustLF.write(thrustLF_PWM);
       thrustLA.write(thrustLA_PWM);
       thrustRF.write(thrustRF_PWM);
       thrustRA.write(thrustRA_PWM); 

    // Wait 5 seconds for ESCs to register min input value and boot up
    delay(5000);
}    
    
// ********************************************************************************
// Main program code --------------------------------------------------------------
// ********************************************************************************

void loop() {
  
// ########## Radio Control Input Update
// Read for RC input (ideally as an interrupt but not at the moment) and convert to meaningful signal  

// Read the pulse width of each RC channel
rc_channel_1 = pulseIn(rc_channel_1_pin, HIGH, 25000);
rc_channel_2 = pulseIn(rc_channel_2_pin, HIGH, 25000);
rc_channel_3 = pulseIn(rc_channel_3_pin, HIGH, 25000);
rc_channel_4 = pulseIn(rc_channel_4_pin, HIGH, 25000);
rc_channel_5 = pulseIn(rc_channel_5_pin, HIGH, 25000);
rc_channel_6 = pulseIn(rc_channel_6_pin, HIGH, 25000);

// Constrain reading to pre-calibrated/valid signal range
rc_channel_1 = constrain(rc_channel_1,rc_channel_lower_limit,rc_channel_upper_limit);
rc_channel_2 = constrain(rc_channel_2,rc_channel_lower_limit,rc_channel_upper_limit);
rc_channel_3 = constrain(rc_channel_3,rc_channel_lower_limit,rc_channel_upper_limit);
rc_channel_4 = constrain(rc_channel_4,rc_channel_lower_limit,rc_channel_upper_limit);
rc_channel_5 = constrain(rc_channel_5,rc_channel_lower_limit,rc_channel_upper_limit);
rc_channel_6 = constrain(rc_channel_6,rc_channel_lower_limit,rc_channel_upper_limit);

// Remap RC channel input to 0 to 100 (or -50 to 50 if non-altitude/throttle) range - representing % thrust
rc_channel_1 = map(rc_channel_1,rc_channel_lower_limit,rc_channel_upper_limit,0,100);
rc_channel_2 = map(rc_channel_2,rc_channel_lower_limit,rc_channel_upper_limit,-50,50);    
rc_channel_3 = map(rc_channel_3,rc_channel_lower_limit,rc_channel_upper_limit,-50,50);    
rc_channel_4 = map(rc_channel_4,rc_channel_lower_limit,rc_channel_upper_limit,-50,50);    
rc_channel_5 = map(rc_channel_5,rc_channel_lower_limit,rc_channel_upper_limit,0,100); // 0 is off 100 is on    
rc_channel_6 = map(rc_channel_6,rc_channel_lower_limit,rc_channel_upper_limit,0,100); // 0 is off 100 is on    

// To compensate for RC input noise adjust yaw,pitch,roll values to 0 if +/- 1-5%
if ((rc_channel_2 <= 2) & (rc_channel_2 >= -8)){rc_channel_2=0;}
if ((rc_channel_3 <= 2) & (rc_channel_3 >= -8)){rc_channel_3=0;}
if ((rc_channel_4 <= 2) & (rc_channel_4 >= -8)){rc_channel_4=0;}

 
// ########## Set Auto or Manual Flight Mode required based on RC input

// If Autopilot RC input ON - then check and maintain stable horizon and altitude - at minimum 'safe' altitude  
// To prevent RC noise it's value must be at least 80% ON
if (rc_channel_6 >= 80){     
  
     // Enable AUTOPILOT
       // PID - Change all axial PID control to 'AUTOMATIC' to allow autopilot control
       X_axis_PID.SetMode(AUTOMATIC);
       Y_axis_PID.SetMode(AUTOMATIC);
       Z_axis_PID.SetMode(AUTOMATIC);
       Altitude_PID.SetMode(AUTOMATIC);
       Autopilot_status = 1;
  
} else {     
 
     // Disable AUTOPILOT
       // PID - Change main Altitude thrust axial control to 'MANUAL' to allow direct RC override
       X_axis_PID.SetMode(AUTOMATIC);
       Y_axis_PID.SetMode(AUTOMATIC);
       Z_axis_PID.SetMode(AUTOMATIC);
       Altitude_PID.SetMode(MANUAL);
       Autopilot_status = 0;
}

// ########## Altitude Input Update
// Moving-average for the altitude
if(historyPtr == -1)
{
      // First-time, fill in all array-held samples with our initial value
      for(int i = 0; i < numHistory; i++)
      {
        history[i] = (BMP085_Alt.readAltitude(current_pascals))*100;
      }
      
      historyTotal = numHistory * ((BMP085_Alt.readAltitude(current_pascals))*100);
      historyPtr = 0;
        
        // Read ground height and convert metres to cm
        ground_altitude = (BMP085_Alt.readAltitude(current_pascals))*100;
        
        // Set cruise altitude as minimum safe altitude until we can take over from a manual-set alt
        Setpoint_cruise_altitude = altitude_safe;
}
else
{
      // subtract the oldest reading, add the newest reading
      historyTotal -= history[historyPtr];
      history[historyPtr] = (BMP085_Alt.readAltitude(current_pascals)*100);
      historyTotal += history[historyPtr];
      historyPtr++;
      
      if(historyPtr >= numHistory)
      {
        historyPtr = 0;
      }
}  

      // Calculate averaged altitude
      int altitude_current = (historyTotal / numHistory)-ground_altitude;
       
      // Restrict altitude to +ve range of numbers
      altitude_current = constrain(altitude_current, 0, 1000);

                
// ########## Read 6DOF orientation yaw, pitch, roll angles
sixDOF.getYawPitchRoll(angles);
  int yaw = (angles[0]); // ** Yaw needs work and hook into magnetometer sensor **
  int pitch = (angles[1]); // Reports = 0 - level, 90 vertical climb, -90 nosedive
  int roll = (angles[2]);  // Reports = 0 - level, -90 left roll, 90 right roll
  
  
// ########## PID Calculations
// Assign axial 6DOF and Barometer input variables feedback to PID input function
InputX = pitch;
InputY = roll;
InputZ = yaw;
InputAltitude = altitude_current; 

      // ########## Altitude PID compute - goes to aggresive if smaller gap than other feedback PID loops
      double gap = abs(Setpoint_cruise_altitude-altitude_current); //distance away from setpoint
      if(gap<20)
        {  //we're close to setpoint, use conservative tuning parameters
           Altitude_PID.SetTunings(consKp, consKi, consKd);
        }
        else
        {
           //we're far from setpoint, use aggressive tuning parameters
           Altitude_PID.SetTunings(aggKp, aggKi, aggKd);
        }       
        Altitude_PID.Compute();                  
  
      // ########## X (pitch) axis PID compute
      gap = abs(Setpoint_xaxis-pitch); //distance away from setpoint
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
                   
      // ########## Y (roll) axis PID compute
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

            
      // ########## Z (yaw) axis PID compute
      gap = abs(Setpoint_zaxis-roll); //distance away from setpoint
      if(gap<10)
        {  
           //we're close to setpoint, use conservative tuning parameters
           Z_axis_PID.SetTunings(consKp, consKi, consKd);
        }
        else
        {
           //we're far from setpoint, use aggressive tuning parameters
           Z_axis_PID.SetTunings(aggKp, aggKi, aggKd);
        }       
        Z_axis_PID.Compute();           
    
// #################################################################################################################################################  
// ########## Auto Pilot/PID Motor Adjustment Mapping
// Which motors to correct thrust on depends on orientation angle:
// Adjustment is based on using thrust needed to sustain altitude and +/- % thrust from this on each motor to maintain axial X,Y,Z/Yaw,Pitch,Roll balance

// Check if Autopilot ENABLED mode before adjusting
if (Autopilot_status == 1){
              
    // ########## PITCH correction    
       thrustLF_PWM=PID_Output_Altitude+PID_Output_Xaxis+PID_Output_Yaxis+PID_Output_Zaxis;
       thrustRF_PWM=PID_Output_Altitude+PID_Output_Xaxis-PID_Output_Yaxis-PID_Output_Zaxis;
       thrustLA_PWM=PID_Output_Altitude-PID_Output_Xaxis+PID_Output_Yaxis-PID_Output_Zaxis;
       thrustRA_PWM=PID_Output_Altitude-PID_Output_Xaxis-PID_Output_Yaxis+PID_Output_Zaxis;  
                          
}else{
 
// #################################################################################################################################################    
// ########## Manual Radio Control (defined by RC Autopilot On/Off switch) 
// Inc. auto-stablise if no RC roll, pitch or yaw input received
// Which motors to correct thrust on depends on RC input
// Roll, Pitch, Yaw adjustment is based on adjusting thrust RC input by +/- % 
    
  // rc_channel_1 = altitude/throttle
  // rc_channel_2 = roll
  // rc_channel_3 = pitch
  // rc_channel_4 = yaw
 
   // Set all motors to thrust level dictated by rc_channel_1 input (altitude thrust)
   thrustLF_PWM=rc_channel_1;
   thrustLA_PWM=rc_channel_1;        
   thrustRF_PWM=rc_channel_1;
   thrustRA_PWM=rc_channel_1;   
   
   // ##########  Do NOT Auto-stablise individual axes if RC roll, pitch or yaw input received
   // Force corresponding PID correction to 0;

   if (rc_channel_2 != 0){
     PID_Output_Yaxis == 0;
   }

   if (rc_channel_3 != 0){     
     PID_Output_Xaxis == 0;
   }

   if (rc_channel_4 != 0){
     PID_Output_Zaxis == 0;
   }

  // Auto Stablise correction 
  thrustLF_PWM=rc_channel_1+PID_Output_Xaxis+PID_Output_Yaxis+PID_Output_Zaxis;
  thrustRF_PWM=rc_channel_1+PID_Output_Xaxis-PID_Output_Yaxis-PID_Output_Zaxis;
  thrustLA_PWM=rc_channel_1-PID_Output_Xaxis+PID_Output_Yaxis-PID_Output_Zaxis;
  thrustRA_PWM=rc_channel_1-PID_Output_Xaxis-PID_Output_Yaxis+PID_Output_Zaxis;
                    
  // Pitch, Roll, Yaw additions
  thrustLF_PWM=thrustLF_PWM-rc_channel_2-rc_channel_3+rc_channel_4;
  thrustRF_PWM=thrustRF_PWM+rc_channel_2-rc_channel_3-rc_channel_4;
  thrustLA_PWM=thrustLA_PWM-rc_channel_2+rc_channel_3-rc_channel_4;
  thrustRA_PWM=thrustRA_PWM+rc_channel_2+rc_channel_3+rc_channel_4;
                           
} 
 
// ################################################################################################################################################# 
// Common PWM thrust motor control routine
   
//Constrain PWM values as RC can exceed 100% in some cases once all PID corrections have been summed
thrustLF_PWM = constrain(thrustLF_PWM,0,100);   
thrustLA_PWM = constrain(thrustLA_PWM,0,100);
thrustRF_PWM = constrain(thrustRF_PWM,0,100);
thrustRA_PWM = constrain(thrustRA_PWM,0,100);
 
// Instruct thrust motors to alter output to new PWM values based on converting % output to 'real' PWM min/max range 
thrustLF_PWM = map(thrustLF_PWM,0,100,thrust_minimum_PWM,thrust_maximum_PWM);
thrustLA_PWM = map(thrustLA_PWM,0,100,thrust_minimum_PWM,thrust_maximum_PWM);
thrustRF_PWM = map(thrustRF_PWM,0,100,thrust_minimum_PWM,thrust_maximum_PWM);
thrustRA_PWM = map(thrustRA_PWM,0,100,thrust_minimum_PWM,thrust_maximum_PWM);

// ***************** SAFETY CHECK *******************
// If engines_armed RC input ON - then update motors with 'live' thrust %
// To prevent RC noise it's value must be at least 80% ON
if (rc_channel_5 >= 80){     
    
    // Set engines_armed to ON
    engines_armed = 1;
    
    // Update ESCs with new thrust values
    thrustLF.write(thrustLF_PWM);
    thrustLA.write(thrustLA_PWM);
    thrustRF.write(thrustRF_PWM);
    thrustRA.write(thrustRA_PWM);
    
}else{

    // Set engines_armed to OFF
    engines_armed = 0;
  
    // Kill all thrust to motors  
    thrustLF_PWM = thrust_minimum_PWM;
    thrustLA_PWM = thrust_minimum_PWM;
    thrustRF_PWM = thrust_minimum_PWM;
    thrustRA_PWM = thrust_minimum_PWM;
       thrustLF.write(thrustLF_PWM);
       thrustLA.write(thrustLA_PWM);
       thrustRF.write(thrustRF_PWM);
       thrustRA.write(thrustRA_PWM);   
}

// *****************************************************************************
// ****************************************************************************** 
 // DEBUG AREA 
                  // Output to serial   
                  
         Serial.print("rc_channel_1 = ");
                  Serial.println(rc_channel_1);
                  Serial.print("rc_channel_2 = ");
                  Serial.println(rc_channel_2);
                  Serial.print("rc_channel_3 = ");
                  Serial.println(rc_channel_3);
                  Serial.print("rc_channel_4 = ");
                  Serial.println(rc_channel_4);
                  
                  Serial.print("rc_channel_5 = ");
                  Serial.println(rc_channel_5);
                  Serial.print("rc_channel_6 = ");
                  Serial.println(rc_channel_6);
                
/*                                   
                  Serial.println("************ DEBUG LOOP *************");
                  Serial.print("altitude_safe = ");
                  Serial.println(altitude_safe);
                  Serial.print("ground_altitude = ");
                  Serial.println(ground_altitude);
                  Serial.print("altitude_current = ");
                  Serial.println(altitude_current);
                  Serial.println(" ");               
                  Serial.print("thrust_minimum_PWM = ");
                  Serial.println(thrust_minimum_PWM);
                  Serial.print("thrust_maximum_PWM = ");
                  Serial.println(thrust_maximum_PWM);
                  Serial.println(" ");
                  Serial.print("thrustLF_PWM = ");
                  Serial.println(thrustLF_PWM);
                  Serial.print("thrustLA_PWM = ");
                  Serial.println(thrustLA_PWM);
                  Serial.print("thrustRF_PWM = ");
                  Serial.println(thrustRF_PWM);
                  Serial.print("thrustRA_PWM = ");
                  Serial.println(thrustRA_PWM);
                  Serial.println(" ");
                  Serial.print("rc_channel_1 = ");
                  Serial.println(rc_channel_1);
                  Serial.print("rc_channel_2 = ");
                  Serial.println(rc_channel_2);
                  Serial.print("rc_channel_3 = ");
                  Serial.println(rc_channel_3);
                  Serial.print("rc_channel_4 = ");
                  Serial.println(rc_channel_4);
                  Serial.print("rc_channel_5 = ");
                  Serial.println(rc_channel_5);
                  Serial.print("rc_channel_6 = ");
                  Serial.println(rc_channel_6);
                  Serial.println(" ");
                  Serial.print("Setpoint_xaxis = ");
                  Serial.println(Setpoint_xaxis);
                  Serial.print("InputX = ");
                  Serial.println(InputX);
                  Serial.print("PID_Output_Xaxis = ");
                  Serial.println(PID_Output_Xaxis);
                  Serial.println(" ");
                  Serial.print("Setpoint_yaxis = ");
                  Serial.println(Setpoint_yaxis);
                  Serial.print("InputY = ");
                  Serial.println(InputY);
                  Serial.print("PID_Output_Yaxis = ");
                  Serial.println(PID_Output_Yaxis);
                  Serial.println(" ");
                  Serial.print("Setpoint_zaxis = ");
                  Serial.println(Setpoint_zaxis);
                  Serial.print("InputZ = ");
                  Serial.println(InputZ);
                  Serial.print("PID_Output_Zaxis = ");
                  Serial.println(PID_Output_Zaxis);
                  Serial.println(" ");
                  Serial.print("Setpoint_cruise_altitude = ");
                  Serial.println(Setpoint_cruise_altitude);
                  Serial.print("InputAltitude = ");
                  Serial.println(InputAltitude);
                  Serial.print("PID_Output_Altitude = ");
                  Serial.println(PID_Output_Altitude);
                  Serial.println(" ");
                  Serial.print("yaw = ");
                  Serial.println(yaw);
                  Serial.print("pitch = ");
                  Serial.println(pitch);
                  Serial.print("roll = ");
                  Serial.println(roll);
                  Serial.println(" ");
                  Serial.print("Autopilot_status = ");
                  Serial.println(Autopilot_status);
                  Serial.print("engines_armed = ");
                  Serial.println(engines_armed);
                  Serial.println("*********************************");                 
                  Serial.println(" ");
*/
                  
// ******************************************************************************
// ******************************************************************************

 // Maintain height for 5 minutes
  
  
   // Gradual descent - accelerometer feedback to prevent free-fall state


   // Land


   // Spin down


   // Power off  
  
}
