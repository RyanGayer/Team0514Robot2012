/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package edu.mphs.first.utils;
import edu.mphs.first.interfaces.RobotInterface;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DoubleSolenoid;


/**
 *
 * @author marnold
 */
public class BridgeUtil {
    //Class variables here...
    
    public BridgeUtil(){
        //Class Constructor
    }
    
    public void manageBridge(Joystick leftStick, DoubleSolenoid Arm)
    {
        if(leftStick.getRawButton(RobotInterface.ARMS_OUT))
        {
            Arm.set(DoubleSolenoid.Value.kForward);
        }
        if(leftStick.getRawButton(RobotInterface.ARMS_IN)){
            Arm.set(DoubleSolenoid.Value.kReverse);
        }
    }
}
