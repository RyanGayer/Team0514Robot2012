/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.mphs.first.wpilibj;
// imports for MPHS Classes
import edu.mphs.first.interfaces.RobotInterface;
import edu.mphs.first.utils.DriveUtil;
import edu.mphs.first.utils.DashBoardUtil;
import edu.mphs.first.utils.AutonomousUtil;
import edu.mphs.first.utils.BridgeUtil;
import edu.mphs.first.utils.BallUtil;
import edu.mphs.first.utils.ShotUtil;
import edu.mphs.first.utils.KinectUtil;
import edu.mphs.first.utils.BalanceUtil;

// imports for WPI Classes
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStationLCD;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
//import edu.wpi.first.wpilibj.camera.AxisCamera;
import edu.wpi.first.wpilibj.AnalogChannel;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Kinect;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.SensorBase;
import edu.wpi.first.wpilibj.communication.ModulePresence;
import edu.wpi.first.wpilibj.communication.ModulePresence.ModuleType;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.Victor;
/**
 * Robot Functionality is described here...
 * 
 */
public class Team0514Robot extends IterativeRobot {
	// Declare variable for the robot drive system
	DriveUtil m_driveUtil;                  // We will use a basic 2 drive configuration, PWM 1 and 2
        RobotDrive m_robotDrive;		// robot will use PWM 1-4 for drive motors
        SpeedController m_rightJag;
        SpeedController m_leftJag;
//        Servo m_driveShift;
        Solenoid m_pneuShifter;

        //Declare Balance Util Class
        BalanceUtil m_balanceUtil;
        
        //Declare variables for PIDController
        PIDController m_shotPC;
        PIDSource m_shotPS;
        PIDController m_bridgePC;
        PIDSource m_bridgePS;
        Gyro m_gyro;
        boolean track_pid, tipping, balance;
        int curr_angle, prev_angle;
        
        // Declare variables for Kinects...
        Kinect m_kinect;
        KinectUtil m_kinectUtil;
        boolean hybrid, auto, hybrid2;
        
        // Declare variables for the Axis Camera
  //      AxisCamera m_cam;
        
        // Declare variables for the ShotUtil
        Solenoid m_firePin;
        DoubleSolenoid m_ballGate;
        DigitalInput m_limitSwitch;
        ShotUtil m_shotUtil;
        SpeedController m_shooter;
        AnalogChannel m_pot;
        SpeedController m_angle;
        double m_shotSpeed, m_prevSpeed, m_prevAngle;
        
        // Declare variables for BallUtil
        BallUtil m_ballUtil;
        SpeedController m_ballMotor;
        
	// Declare variables for Dashboard
        int m_dsPacketsReceivedInCurrentSecond;	// keep track of the ds packets received in the current second
        DashBoardUtil m_db;
        
        // Declare variables for DriveStation LCD
        DriverStationLCD m_dsLCD;
        String msg;
        boolean send;
        
        // Declare Compressor variables
        Compressor m_comp;
        
        // Declare variables for Autonomous
        AutonomousUtil m_autoUtil;
        DigitalInput m_auto3PT;
        DigitalInput m_auto2PT;
        double speed;
        
	// Declare variables for the two joysticks being used
	Joystick m_rightStick;			// joystick 1 (arcade stick or right tank stick)
	Joystick m_leftStick;			// joystick 2 (tank left stick)
        Joystick m_controller;
	boolean[] m_rightStickButtonState = new boolean[(RobotInterface.NUM_STICK_BUTTONS+1)];
	boolean[] m_leftStickButtonState = new boolean[(RobotInterface.NUM_STICK_BUTTONS+1)];
        
        //Declare variable for bridge control
        BridgeUtil m_bridge;
        DoubleSolenoid m_Arm;


	// Declare variables for each of the eight solenoid outputs

	// drive mode selection
	int m_driveMode;
        double gear;
        boolean squaredInputs;
        
        //Ball Motor Activated...
        boolean disp_ball, disp_angle, disp_speed;
        String msg1, msg2;
        int potValue;

	// Local variables to count the number of periodic loops performed
	int m_autoPeriodicLoops;
	int m_disabledPeriodicLoops;
	int m_telePeriodicLoops;

    /**
     * Constructor for this "Team0514Robot" Class.
     *
     * The constructor creates all of the objects used for the different inputs and outputs of
     * the robot.  Essentially, the constructor defines the input/output mapping for the robot,
     * providing named objects for each of the robot interfaces.
     */
    public Team0514Robot() {
        System.out.println("Team0514Robot Constructor Started\n");
        //Instantiate all objects used by the Robot.  The word NEW should never occur in a Util Class!!!
        
		// Create a robot using standard right/left robot drive on PWMS 1 and 2
                m_leftJag = new Victor(RobotInterface.LEFT_JAGUAR);  //Digital Slot
                m_rightJag = new Victor(RobotInterface.RIGHT_JAGUAR);  //Digital Slot
                m_robotDrive = new RobotDrive(m_leftJag, m_rightJag);
                m_driveUtil = new DriveUtil();
//                m_driveShift = new Servo(RobotInterface.Drive_Shift);  //Digital Slot
                m_pneuShifter = new Solenoid(RobotInterface.Shifter);
                
                //Create Balance Util Objects
                m_balanceUtil = new BalanceUtil();
                
                //Create Kinect Objects
                m_kinectUtil = new KinectUtil();
                
                //Create shooter objects
                m_shotUtil = new ShotUtil();
                m_ballGate = new DoubleSolenoid(RobotInterface.gateClosed, RobotInterface.gateOpen);
                m_limitSwitch = new DigitalInput(RobotInterface.lSwitchChannel);
                m_firePin = new Solenoid(RobotInterface.FIRE_PIN);  //PNEU Slot
                m_shooter = new Jaguar(RobotInterface.SHOOTER);  //Digital Slot
                m_pot = new AnalogChannel(RobotInterface.POT1_CHANNEL);  //Analog Slot
                m_angle = new Jaguar(RobotInterface.ANGLE);
                //Create ball motor object
                m_ballUtil = new BallUtil();
                m_ballMotor = new Jaguar(RobotInterface.BALL_JAGUAR);  //Digital Slot
                
                //Create bridge control object
                m_bridge = new BridgeUtil();
                m_Arm = new DoubleSolenoid(RobotInterface.Arm_Back, RobotInterface.Arm_Out);  //PNEU Slot
                
		// Create Compressor Object
                m_comp = new Compressor(RobotInterface.COMP_P_SWITCH_CHANNEL, 
                                        RobotInterface.COMP_C_RELAY_CHANNEL);  //Digital Slot

                // Create Dashboard objects
                m_db = new DashBoardUtil();
                m_dsPacketsReceivedInCurrentSecond = 0;

                // Create autonomous control objects
                m_autoUtil = new AutonomousUtil();
                m_auto3PT = new DigitalInput(RobotInterface.Auto_3PT);
                m_auto2PT = new DigitalInput(RobotInterface.Auto_2PT);

		// Define joysticks being used at USB port #1 and USB port #2 on the Drivers Station
		m_rightStick = new Joystick(RobotInterface.RIGHT_Z_JOYSTICK);
		m_leftStick = new Joystick(RobotInterface.LEFT_JOYSTICK);
                m_controller = new Joystick(RobotInterface.CONTROLLER);

		// Iterate over all the buttons on each joystick, setting state to false for each
		int buttonNum = 1;						// start counting buttons at button 1
		for (buttonNum = 1; buttonNum <= RobotInterface.NUM_STICK_BUTTONS; buttonNum++) {
			m_rightStickButtonState[buttonNum] = false;
			m_leftStickButtonState[buttonNum] = false;
		}

		// Initialize counters to record the number of loops completed in autonomous and teleop modes
		m_autoPeriodicLoops = 0;
		m_disabledPeriodicLoops = 0;
		m_telePeriodicLoops = 0;
                
                //Instantiate Shot PIDController and PID Source
                m_shotPS = new PIDSource() {

            public double pidGet() {
                return m_pot.pidGet();
            }
        };
                m_shotPC = new PIDController(RobotInterface.Kp, 
                                             RobotInterface.Ki, 
                                             RobotInterface.Kd, 
                                             m_shotPS, new PIDOutput() {

            public void pidWrite(double output) {
                m_angle.set(-output);
            }
                }, RobotInterface.Kperiod);
                    
                //Instantiate Bridge PIDController and PID Source
                m_gyro = new Gyro(RobotInterface.GYRO_CHANNEL);
                m_bridgePS = new PIDSource(){

            public double pidGet() {
                curr_angle = (int) m_gyro.pidGet();
                return m_gyro.pidGet();
            }
                    
                };
                  
                m_bridgePC = new PIDController(RobotInterface.Kp, 
                                               RobotInterface.Ki, 
                                               RobotInterface.Kd, 
                                               m_bridgePS, new PIDOutput() {

            public void pidWrite(double output) {
                m_robotDrive.tankDrive(output, output);
            }
        }, RobotInterface.Kperiod);

		System.out.println("Team0514Robot Constructor Completed\n");
	}


	/********************************** Init Routines *************************************/

	public void robotInit() {
		
            //    printSensorBaseDetails();

            // Actions which would be performed once (and only once) upon initialization of the

                m_comp.start();
                
                
                

                // robot would be put here.
    //            m_cam = AxisCamera.getInstance();
                m_robotDrive.setInvertedMotor(RobotDrive.MotorType.kRearRight, true);
                m_robotDrive.setInvertedMotor(RobotDrive.MotorType.kRearLeft, false);
                m_angle.set(0);
                m_shotSpeed = 0;
                m_prevSpeed = 0;
                m_prevAngle = 0;
                m_dsLCD = DriverStationLCD.getInstance();
                squaredInputs = false;
                m_driveMode = 0;   // 0 = Tank, 1 = Arcade, default to Tank.
                gear = 1;          // 0 = Low Gear, 1 = High Gear, default to High
                msg = "HIGH GEAR";
                buildMsg(2, msg);
                msg1 = "BALL MOTOR OFF";
                buildMsg(3, msg1);
                msg2 = " ";
                disp_ball = false;
                disp_angle = false;
                disp_speed = false;
                potValue = 0;
                send = true;
                m_dsLCD.updateLCD();
                
                //Start Competition in High Gear.
                //May need to put this in Low Gear for Hybrid Mode...
//                m_driveShift.set(RobotInterface.Servo_High);
                m_pneuShifter.set(false);
                
                //Start Competition with Arms Back...
                m_Arm.set(DoubleSolenoid.Value.kForward);
                m_ballGate.set(DoubleSolenoid.Value.kForward);
                //Get Instance of Kinect Object...
                m_kinect = Kinect.getInstance();
                hybrid = false;
                hybrid2 = false;
                auto = false;

		System.out.println("RobotInit() completed.\n");
	}

	public void disabledInit() {
		m_disabledPeriodicLoops = 0;			// Reset the loop counter for disabled mode
	}
        
	public void autonomousInit() {
		m_autoPeriodicLoops = 0;    // Reset the loop counter for autonomous mode

                m_autoUtil.resetCounter();
                
                // Set Shooter PIDController state...
                m_shotPC.setInputRange(RobotInterface.SPC_In_Top, RobotInterface.SPC_In_Bot);
                m_shotPC.setOutputRange(RobotInterface.SPC_Out_Min, RobotInterface.SPC_Out_Max);
                m_shotPC.setTolerance(RobotInterface.SPC_Tollerance);
                m_shotPC.disable();                
                
                if(m_auto2PT.get() && m_auto3PT.get()){  //switch in middle position...
                    hybrid = true;
                }else if(m_auto3PT.get()){               //switch in the forward position...
                    //set up 3PT Logic here...
                    hybrid = false;
                    speed = RobotInterface.Speed3PT;
                    m_shotPC.setSetpoint(RobotInterface.SPC_3PT_Angle); 
                    m_shotPC.enable();
                }else{                                  //switch in the backward position...
                    //set up 2PT Logic here...
                    hybrid = false;
                    speed = RobotInterface.Speed2PT;
                    m_shotPC.setSetpoint(RobotInterface.SPC_2PT_Angle);
                    m_shotPC.enable();
                }                

	}

	public void teleopInit() {
		m_telePeriodicLoops = 0;				// Reset the loop counter for teleop mode
		m_dsPacketsReceivedInCurrentSecond = 0;                 // Reset the number of dsPackets in current second

                // Set Bridge PIDController state...
                m_bridgePC.setInputRange(RobotInterface.BPC_In_Min, RobotInterface.BPC_In_Max);
                m_bridgePC.setOutputRange(RobotInterface.BPC_Out_Min, RobotInterface.BPC_Out_Max);
                m_bridgePC.setTolerance(RobotInterface.BPC_Tollerance);
                m_bridgePC.disable();
                balance = false;
                
                
                // Clear Autonomous PIDController from Memory.
                if(m_shotPC != null){
                    m_shotPC.free();
                }
               m_shooter.set(0); 
        }

	/********************************** Periodic Routines *************************************/

	public void disabledPeriodic()  {
		// feed the user watchdog at every period when disabled
		Watchdog.getInstance().feed();

		// increment the number of disabled periodic loops completed
		m_disabledPeriodicLoops++;

	}

	public void autonomousPeriodic() {
		// feed the user watchdog at every period when in autonomous
		Watchdog.getInstance().feed();

		m_autoPeriodicLoops++;
                
                if(hybrid){
                    m_kinectUtil.manageKinect(m_kinect, m_shooter, m_firePin,
                                              m_ballGate, m_limitSwitch, 
                                              m_angle, m_robotDrive, m_Arm);
                }else{
                    if(m_pot.getAverageValue() <= (m_shotPC.getSetpoint() + 6) &&
                       m_pot.getAverageValue() >= (m_shotPC.getSetpoint() - 6)){
                        if(!auto){
                            auto = true;
                            m_shotPC.reset();                            
                        }
                    }
                }
                if(auto){
                        hybrid2 = m_autoUtil.manageAutonomous(m_shooter, m_firePin, 
                                                              m_ballGate, speed);                                        
                }
                if(hybrid2){
                    m_kinectUtil.manageKinect(m_kinect, m_shooter, m_firePin,
                                              m_ballGate, m_limitSwitch, 
                                              m_angle, m_robotDrive, m_Arm);                    
                }
	}

	   public void teleopPeriodic() {
        // feed the user watchdog at every period when in autonomous
        Watchdog.getInstance().feed();

        // increment the number of teleop periodic loops completed
        m_telePeriodicLoops++;

        /*
         * Code placed in here will be called only when a new packet of information
         * has been received by the Driver Station.  Any code which needs new information
         * from the DS should go in here
         */

        m_dsPacketsReceivedInCurrentSecond++;					// increment DS packets received
        
      //  m_cam.freshImage();

        // update the Dashboard
        m_db.manageDashBoard();
        
        // put Driver Station-dependent code here
        gear = m_driveUtil.manageDrive(m_robotDrive, m_leftStick, m_rightStick, squaredInputs,
                                m_pneuShifter);
      
        //The code to handle collecting balls for shooter goes here.
        disp_ball = m_ballUtil.manageBall(m_controller, m_ballMotor);
        
        //The code to handle the shooter goes here.
        m_shotSpeed = m_shotUtil.manageShot(m_controller, m_shooter, m_firePin,
                                            m_pot, m_angle ,m_ballGate, m_limitSwitch);
        if(m_shotSpeed != m_prevSpeed){
                m_prevSpeed = m_shotSpeed;
                disp_speed = true;
        }else{
            disp_speed = false;
        }

        potValue = m_pot.getAverageValue();
        if(potValue != m_prevAngle){
            m_prevAngle = potValue;
            disp_angle = true;    
        }else{
            disp_angle = false;
        }
  
       //The code to execute to go up the ramp goes here.
        m_bridge.manageBridge(m_leftStick, m_Arm);
        
        //The code to run the robot in auto balance mode goes here.
        m_balanceUtil.manageBalance(m_leftStick, m_rightStick, m_bridgePC, m_gyro, m_robotDrive);
        
        updateDriverStation();
 
  }
                
        public void updateDriverStation(){
            //Only update the string when the gear was changed!  Saves Memory!
            if(gear == 0 && msg.equalsIgnoreCase("HIGH GEAR")){
                msg = "LOW GEAR ";
                buildMsg(2, msg);
                send = true;
            }
            if(gear == 1 && msg.equalsIgnoreCase("LOW GEAR ")){
                msg = "HIGH GEAR";
                buildMsg(2, msg);
                send = true;
            }
            if(disp_ball  && msg1.equalsIgnoreCase("BALL MOTOR OFF")){
                msg1 = "BALL MOTOR ON";
                buildMsg(3, msg1);
                send = true;
            }
            if(!disp_ball && msg1.equalsIgnoreCase("BALL MOTOR ON")){
                msg1 = "BALL MOTOR OFF";
                buildMsg(3, msg1);
                send = true;
            }
            if(disp_speed || disp_angle){
                msg2 =  ("A/S = " + potValue +
                         "  / " + Double.toString(m_shotSpeed));
                buildMsg(4, msg2);
                send = true;
            }
          // Send Messages to Driver Station            
            if(send){
                m_dsLCD.updateLCD();
                send = false;
            }
        }
        
        public void buildMsg(int line, String msg){
            if(line == 2){
                m_dsLCD.println(DriverStationLCD.Line.kUser2, 6, msg);
            }
            if(line == 3){
                m_dsLCD.println(DriverStationLCD.Line.kUser3, 6, msg);                
            }
            if(line == 4){
                m_dsLCD.println(DriverStationLCD.Line.kUser4, 2, msg);                
            }
        }
        private void printSensorBaseDetails(){
                        
                System.out.println("Solenoid Default Module = " + SensorBase.getDefaultSolenoidModule());
            
            int numSolenoid = 0;
            for (numSolenoid = 0; numSolenoid < SensorBase.kSolenoidChannels; numSolenoid++){
                if(numSolenoid == RobotInterface.Arm_Out){
                    System.out.println("kSolenoid[ARM_OUT] = " + 
                                       ModulePresence.getModulePresence(ModuleType.kSolenoid, numSolenoid));
                }
                if(numSolenoid == RobotInterface.Arm_Back){
                        System.out.println("kSolenoid[ARM_BACK] = " + 
                                           ModulePresence.getModulePresence(ModuleType.kSolenoid, numSolenoid));
                }
                System.out.println("kSolenoid[" + numSolenoid + "] = " + 
                                   ModulePresence.getModulePresence(ModuleType.kSolenoid, numSolenoid));                
                }

                System.out.println("Digital Default Module = " + SensorBase.getDefaultDigitalModule());

            int numDigital = 1;
            for (numDigital = 1; numDigital <= SensorBase.kDigitalChannels; numDigital++){
                if(numDigital == RobotInterface.LEFT_JAGUAR){
                    System.out.println("kDigital[LEFT_JAG} = " + 
                                        ModulePresence.getModulePresence(ModuleType.kDigital, numDigital));
                }
                if(numDigital == RobotInterface.RIGHT_JAGUAR){
                    System.out.println("kDigital[RIGHT_JAG} = " + 
                                        ModulePresence.getModulePresence(ModuleType.kDigital, numDigital));
                }
                if(numDigital == RobotInterface.Drive_Shift){
                    System.out.println("kDigital[DRIVE_SHIFT} = " + 
                                        ModulePresence.getModulePresence(ModuleType.kDigital, numDigital));
                }
                if(numDigital == RobotInterface.COMP_C_RELAY_CHANNEL){
                    System.out.println("kDigital[COMP_RELAY} = " + 
                                        ModulePresence.getModulePresence(ModuleType.kDigital, numDigital));
                }
                if(numDigital == RobotInterface.COMP_P_SWITCH_CHANNEL){
                    System.out.println("kDigital[COMP_SWITCH} = " + 
                                        ModulePresence.getModulePresence(ModuleType.kDigital, numDigital));
                }
                System.out.println("kDigital[" + numDigital + "] = " + 
                                   ModulePresence.getModulePresence(ModuleType.kDigital, numDigital));                
            }

                System.out.println("Analog Default Module = " + SensorBase.getDefaultAnalogModule());
            
            int numAnalog = 1;
            for (numAnalog = 1; numAnalog <= SensorBase.kAnalogChannels; numAnalog++){
                if(numAnalog == 8){
                    System.out.println("kAnalog[POT] = " + 
                                    ModulePresence.getModulePresence(ModuleType.kAnalog, numAnalog));
                }
                System.out.println("kAnalog[" + numAnalog + "] = " +
                                   ModulePresence.getModulePresence(ModuleType.kAnalog, numAnalog));
            }

        }
        
}
