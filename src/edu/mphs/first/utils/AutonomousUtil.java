/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package edu.mphs.first.utils;

import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid;



/**
 *
 * @author marnold
 */
public class AutonomousUtil {
    int autoLoopCounter;
    
    public AutonomousUtil(){
        autoLoopCounter = 0;
    }
    public boolean manageAutonomous(SpeedController shooter, 
                                 Solenoid firepin, DoubleSolenoid ballGate,
                                 double speed){
        Watchdog.getInstance().feed();
            autoLoopCounter++;
            switch(autoLoopCounter){
                case 1:
                {   
                    //Set Speed
                    shooter.set(speed);
                    break;
                }
                case 95:
                {
                    //Shoot First Ball
                    firepin.set(true);
                    ballGate.set(DoubleSolenoid.Value.kReverse);
                    break;
                }
                case 120:
                {
                    //Return firepin
                    firepin.set(false);
                    break;
                }
                case 170:
                {
                    ballGate.set(DoubleSolenoid.Value.kForward);
                    break;
                }
                case 270:
                {
                    //Shoot Second Ball
                    firepin.set(true);
                    break;
                }
                case 295:
                {
                    //Return firepin
                    firepin.set(false);
                    break;
                }
            }
            if(autoLoopCounter > 295){
                return true;
            }else{
                return false;
            }
        }
    public void resetCounter(){
        autoLoopCounter = 0;
    }
}
