/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package edu.mphs.first.interfaces;

/**
 * You are to use this interface to define all public static final variables
 * that the Robot will use.  The primary use of this interface is the define
 * all variables that represent device wiring configuration back to the cRIO
 * and Digital Side Car.  These are perfect candidates to take advantage of
 * the Interface pattern.  Use this for other variable declarations as well but
 * if you are unsure, define it to your local class.method() and we can discuss
 * it later.
 *
 * @author marnold
 */
public interface RobotInterface {
    
//------------------------Misc Runtime Config Parameters-------------------------------
    //Constants
        //0 = Off, 1 = On.  In the future there may be other levels...
        public static final int M_DEBUG_LEVEL = 0;
        public static final int M_LCD_LEVEL = 0;
        public static final double Servo_Low = 0.0;           // Drive Shifter Servo in one direction
        public static final double Servo_High = 1.0;          // Drive Shifter Servo in one direction
        public static final double Shift_Rate = 0.25;         // Constant multiplier to slow motor to allow shift on the fly.
        public static final double AngleUpSetSpeed = .65;      // Must Be Positive between 0 and 1
        public static final double AngleDownSetSpeed = -.5;   // Must be Negative between 0 and -1
        public static final double scrollUp = .005;           // Constant multiplier to scroll shot speed up.
        public static final double scrollDown = -.005;          // Constant multiplier to scroll shot speed down.
        public static final double lowRate = .5;              // Slowest shot motor rate
        public static final double midRate = .75;             // Medium shot motor rate
        public static final double highRate = 1.0;            // Fastest shot motor rate  
        public static final double lowangle = 480;            // Lowest shot angle
        public static final double midangle = 366;            // Middle shot angle
        public static final double highangle = 251;           // Highest shot angle
        //PIDControlller Constants...
        public static final double Kp = .08;
        public static final double Ki = 0.0;
        public static final double Kd = .5;
        public static final double Kperiod = .005;
        
        //Shot PIDController Constants...
        public static final double SPC_In_Top = 242;     //794
        public static final double SPC_In_Bot = 489;     //565
        public static final double SPC_Out_Min = -.6;
        public static final double SPC_Out_Max = .6;
        public static final double SPC_Tollerance = ((1/90)*100);
        public static final double SPC_Angle = 366;
        
        //Bridge PIDController Constants...
        public static final double BPC_In_Min = -20;
        public static final double BPC_In_Max = 20;
        public static final double BPC_Out_Min = -.70;
        public static final double BPC_Out_Max = .70;
        public static final double BPC_Tollerance = ((1/90)*100);
        public static final double BPC_Angle = 1;
        public static final double BPC_Reverse = -.2;

        // Declare Drive Mode selection
        static final int UNINITIALIZED_DRIVE = 0;

        //Declare Sick Controls...
        public static final int ARCADE_MODE = 5;  //Right JoyStick
	public static final int TANK_MODE = 4;    //Right JoyStick
        public static final int Balance_On = 2;   //Right JoyStick
        public static final int Balance_Off = 3;  //Right JoyStick

        public static final int Shift_High = 2;   //Left JoyStick
        public static final int Shift_Low = 3;    //Left JoyStick
        public static final int ARMS_OUT = 5;     //Left Joystick
        public static final int ARMS_IN = 4;      //Left Joystick
        public static final int Gyro_Reset = 1;   //Left Joystick
        
        public static final int BALL_ON = 1;      //Game Controller
        public static final int BALL_OFF = 3;     //Game Controller
        public static final int SHOOT_UP = 5;     //Game Controller
        public static final int SHOOT_DOWN = 7;   //Game Controller
        public static final int SHOOT_MORE = 6;   //Game Controller
        public static final int SHOOT_LESS = 8;   //Game Controller
        public static final int SHOOT = 2;        //Game Controller
        public static final int sScroll = 9;      //Game Controller
        public static final int aScroll = 10;     //Game Controller
        public static final int DisplayInfo = 4;  //Game Controller
        
        // Declare Joystick variables
        public static final int LEFT_JOYSTICK = 2;
        public static final int RIGHT_Z_JOYSTICK = 1;
        public static final int CONTROLLER = 3;


        // Joystick Button Mappings:
        public static final int NUM_STICK_BUTTONS = 16;
        public static final int NUM_CONTROLLER_BUTTONS = 10;
        //Right Stick (1)

        
        //Controller (2)
        //Axis

        
        //Buttons (10)

        
        // Declare Kinect Variables...
            //Constants which define the valid arm positions
                static final int ARM_MAX_ANGLE = 105;
                static final int ARM_MIN_ANGLE = -90;
                static final double Z_PLANE_TOLERANCE = 0.3;    /* In meters */

            //Constants which define the "trigger" angles for the various buttons
                static final int LEG_FORWARD = -110;
                static final int LEG_BACKWARD = -80;
                static final int LEG_OUT = -75;
                static final int HEAD_LEFT = 98;
                static final int HEAD_RIGHT = 82;


        // Declare variables for each of the eight solenoid outputs
	public static final int NUM_SOLENOIDS = 8;

        // Autonomous Timer Constant Units Seconds
        public static final double M_AUTO_TIMELIMIT = 15;

        // Autonomous Controls...
        public static final double Speed2PT = .575;
        public static final double Speed3PT = .675;
        public static final double SPC_3PT_Angle = 310;
        public static final double SPC_2PT_Angle = 370;
        

        //CPU cycles limits and counts

//--------------------------------ROBOT PHYSICAL DEVICE MAPPING AREA-----------------------------------
    //8 Slot CRIO Module Mapping
        public static final int Analog_Slot = 1;
        public static final int Digital_Slot = 1;
        public static final int PNEU_Slot = 1;
        public static final int Analog_Slot_2nd = 2;
        public static final int Digital_Slot_2nd = 2;
        public static final int PNEU_Slot_2nd = 2;
    
    //Analog Bumper (Slot 1)
    // Declare variables for Potentiometer
        public static final int POT1_CHANNEL = 1;
        public static final int POT1_SLOT = 1;
        public static final int GYRO_CHANNEL = 2;
        public static final int GYRO_SLOT = 1;


    //Digital Side Car (Slot 2)
        //PWM OUT
        // Declare variables for the Jaguar Drives
        public static final int LEFT_JAGUAR = 3;
        public static final int RIGHT_JAGUAR = 4;
        public static final int Drive_Shift = 4;
        public static final int BALL_JAGUAR = 5;
        public static final int SHOOTER = 6;
        public static final int ANGLE = 7;


        //Digital IO (Switches)
        public static final int lSwitchChannel = 5;
        public static final int Auto_3PT = 1;
        public static final int Auto_2PT = 2;
        // Declare a variable to use to access the Compressor object
        public static final int COMP_P_SWITCH_CHANNEL = 8;
        
        // Declare a variable to use to access the Encoder object

        //Relays
        public static final int COMP_C_RELAY_CHANNEL = 8;


        //Pneumatic Bumper (Slot 3)
        public static final int Arm_Out = 1;
        public static final int Arm_Back = 2;
        public static final int FIRE_PIN = 3;
        public static final int gateOpen = 5;
        public static final int gateClosed = 4;
        public static final int Shifter = 6;
       

}
