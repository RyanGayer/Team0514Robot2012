/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package edu.mphs.first.utils;

import edu.mphs.first.interfaces.RobotInterface;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedController;  //Could be a Victor, whichever is stronger.
/**
 *
 * @author marnold
 */
public class BallUtil {
    //Class variables go here...
    boolean activate;
    
    public BallUtil(){
        //Class Constructor
        activate = false;
    }
    
    public boolean manageBall(Joystick controller, SpeedController ballMotor){
        if(controller.getRawButton(RobotInterface.BALL_ON)){
            activate = true;
        }
        if(controller.getRawButton(RobotInterface.BALL_OFF)){
            activate = false;
        }
        runMotor(ballMotor);
        return activate;
    }
    
    private void runMotor(SpeedController ballMotor){
        if(activate){
            ballMotor.set(1);
        }else{
            ballMotor.set(0);
        }
    }
}
