/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package edu.mphs.first.utils;

import edu.mphs.first.interfaces.RobotInterface;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
//import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Solenoid;


/**
 *
 * @author marnold
 */
public class DriveUtil {
int driveMode;
double gear;

    public DriveUtil()
    {
        driveMode = 0;  // Default Tank Mode
        gear = 1.0;       // Default High Gear
    }

    private void arcadeDrive(Joystick rightStick, RobotDrive roboDrive, boolean squaredInput)
    {
        roboDrive.arcadeDrive(rightStick.getY(), rightStick.getX(), squaredInput);
    }

    private void tankDrive(Joystick leftStick, Joystick rightStick, RobotDrive roboDrive)
    {
        roboDrive.tankDrive(leftStick.getY(), rightStick.getY());
    }


    private int getDriveMode(Joystick rightStick, int currentDrive)
    {
        if(rightStick.getRawButton(RobotInterface.TANK_MODE))
        {
            return 0;
        }
        if(rightStick.getRawButton(RobotInterface.ARCADE_MODE))
        {
            return 1;
        }
        return currentDrive;
    }
    
    private double shiftGears(RobotDrive roboDrive, 
                            int driveMode, double currentGear,
                            Joystick leftStick, Joystick rightStick,
                            Solenoid pneuShifter){
        if(driveMode == 0){
            roboDrive.tankDrive(leftStick.getY()*RobotInterface.Shift_Rate,
                                rightStick.getY()*RobotInterface.Shift_Rate);
        }else{
            roboDrive.arcadeDrive(rightStick.getY()*RobotInterface.Shift_Rate,
                                  rightStick.getX()*RobotInterface.Shift_Rate,
                                  false);
        }
        if(leftStick.getRawButton(RobotInterface.Shift_High)){ // Low Gear shift to high
            pneuShifter.set(false);
            //Shifter.set(RobotInterface.Servo_High);
            return RobotInterface.Servo_High;
        }
        if(leftStick.getRawButton(RobotInterface.Shift_Low)){  // High Gear shift to low
            pneuShifter.set(true);
            //Shifter.set(RobotInterface.Servo_Low);
            return RobotInterface.Servo_Low;
        }
        return currentGear;
    }

    public double manageDrive(RobotDrive robotDrive, Joystick leftStick, 
                            Joystick rightStick, boolean squaredInput,
                            Solenoid pneuShifter){
        
        driveMode = getDriveMode(rightStick, driveMode);

        if(driveMode == 0){
            tankDrive(leftStick, rightStick, robotDrive);
        }else{
            if(driveMode == 1){
                arcadeDrive(rightStick, robotDrive, squaredInput);
            }
        }
        if(leftStick.getRawButton(RobotInterface.Shift_High) ||
           leftStick.getRawButton(RobotInterface.Shift_Low)){
           gear = shiftGears(robotDrive, driveMode, gear,
                             leftStick, rightStick, pneuShifter);
           if(driveMode == 0){
               tankDrive(leftStick, rightStick, robotDrive);
           }else{
               if(driveMode == 1){
                   arcadeDrive(rightStick, robotDrive, squaredInput);
               }
           }
        }
        return gear;
    }
}