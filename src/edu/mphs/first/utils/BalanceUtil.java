/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package edu.mphs.first.utils;

import edu.mphs.first.interfaces.RobotInterface;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.RobotDrive;


/**
 *
 * @author marnold
 */
public class BalanceUtil {
    // Class variables go here...
        boolean track_pid, tipping, balance;
        int curr_angle, prev_angle, loopCount;

    public BalanceUtil(){
        // Class Constructor stuff goes here...
        track_pid = false;
        tipping = false;
        balance = false;
        curr_angle = 0;
        prev_angle = -1;
        loopCount = 0;
        
    }
    public void manageBalance(Joystick m_leftStick, Joystick m_rightStick, 
                              PIDController m_bridgePC, Gyro m_gyro, 
                              RobotDrive m_robotDrive){
        
        if(m_rightStick.getRawButton(RobotInterface.Arm_Out) 
                   || m_leftStick.getRawButton(RobotInterface.Gyro_Reset)){
                        m_gyro.reset();
                        prev_angle = 0;
                        track_pid = false;
                        tipping = false;            
        }
        if(m_rightStick.getRawButton(RobotInterface.Balance_On)){
            balance = true;
            m_bridgePC.setSetpoint(0);
            m_bridgePC.enable();
        }
        if(m_rightStick.getRawButton(RobotInterface.Balance_Off)){
            balance = false;
            m_bridgePC.disable();
        }
        if(balance){
            senseRobotState(m_bridgePC, m_robotDrive);
            if(!m_bridgePC.isEnable()){
                if(tipping){
                    loopCount++;
                    centerRobot(m_robotDrive);
                }
            }
        }

    }
            private void senseRobotState(PIDController m_bridgePC,
                                        RobotDrive m_robotDrive){

            //If onTarget, shut robot down...
            if(m_bridgePC.onTarget()){
                m_bridgePC.disable();
                m_robotDrive.tankDrive(0, 0);
                balance = false;
                tipping = false;
            }
            //Check the current and previous integer values... if net 0 then we are running up the bridge
            //Angle of assent is constant...
            if(!track_pid){
                if(Math.abs(curr_angle - prev_angle) == 0){
                    track_pid = true;
                }else{
                    prev_angle = curr_angle;
                    track_pid = false;
                }
            }
            //Once climb started, look for the angle to change (not 0) to signify we need to back up.
            if(track_pid){
                if(Math.abs(curr_angle - prev_angle) >= RobotInterface.BPC_Angle){
                    tipping = true;
                }else{
                    tipping = false;
                }
            }
            //Need to stop and back up quickly... Modify the case counts to determine quickness of reverse.
            if(tipping){
                m_bridgePC.disable();
                m_robotDrive.tankDrive(0, 0);
                // Don't execute centering logic...  Just terminate here...
                //balance = false;
                //tipping = false;
            }

        }
        
        private void centerRobot(RobotDrive m_robotDrive){
                    switch(loopCount){
                        case 5:{
                            //Reverse the robot slightly
                            m_robotDrive.tankDrive(RobotInterface.BPC_Reverse,
                                                   RobotInterface.BPC_Reverse);
                            break;
                        }
                        case 10:{
                            //Stop the Robot.
                            m_robotDrive.tankDrive(0, 0);
                            tipping = false;
                            balance = false;
                            break;
                        }    
                    }            
        }

}
