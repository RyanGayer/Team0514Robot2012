/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package edu.mphs.first.utils;

import edu.mphs.first.interfaces.RobotInterface;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedController;  // Could be a Victor... Whichever is stronger.
import edu.wpi.first.wpilibj.AnalogChannel;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;

/**
 *
 * @author marnold
 */
public class ShotUtil {
    //Class variables go here...

    double speed;
    int shotSpeed;
    double scroll;
    int setpoint, previoussetpoint;
    double target;

    public ShotUtil() {
        //Class Constructor   
        speed = 0;
        shotSpeed = 0;
        scroll = 0;
        setpoint = 1;
        previoussetpoint = 1;
        target = RobotInterface.lowangle;
    }

    public double manageShot(Joystick controller, SpeedController shooter,
            Solenoid firepin, AnalogChannel pot, SpeedController angle, DoubleSolenoid ballgate,
            DigitalInput lSwitch) {
        if (lSwitch.get()) {
            ballgate.set(DoubleSolenoid.Value.kForward);
        } else {
            ballgate.set(DoubleSolenoid.Value.kReverse);
        }

        if (controller.getRawButton(RobotInterface.aScroll)) {
            changeAngleScroll(controller, angle, pot);
        }else{
            changeAnglePreset(controller, angle, pot);
        }

        if (controller.getRawButton(RobotInterface.SHOOT)) {
            firepin.set(true);
        } else {
            firepin.set(false);
        }
        
        if(controller.getRawButton(RobotInterface.sScroll)){
            changeSpeedScroll(controller, shooter);
        }else{
            if (controller.getRawButton(RobotInterface.SHOOT_LESS)) {
                changeSpeed(controller, shooter, 0);
            }
            if (controller.getRawButton(RobotInterface.SHOOT_MORE)) {
                changeSpeed(controller, shooter, 1);
            }
        }
        return speed;
    }

    private void changeAngleScroll(Joystick controller, SpeedController angle, AnalogChannel pot) {

        while (controller.getRawButton(RobotInterface.SHOOT_UP)) {
            if (pot.getAverageValue() > RobotInterface.SPC_In_Top) {
                angle.set(RobotInterface.AngleUpSetSpeed);
            } else {
                angle.set(0);
            }
        }

        while (controller.getRawButton(RobotInterface.SHOOT_DOWN)) {
            if (pot.getAverageValue() < RobotInterface.SPC_In_Bot) {
                angle.set(RobotInterface.AngleDownSetSpeed);
            } else {
                angle.set(0);
            }
        }

        previoussetpoint =1;
        setpoint =1;
        angle.set(0);
    }

    private void changeAnglePreset(Joystick controller, SpeedController angle, AnalogChannel pot) {

        if (controller.getRawButton(RobotInterface.SHOOT_UP)) {
            if (setpoint < 2) {
                setpoint++;
            }else{
                setpoint = 2;
            }

        }
        if (controller.getRawButton(RobotInterface.SHOOT_DOWN)) {
            if (setpoint > 0) {
                setpoint--;
            }else{
                setpoint = 0;
            }
        }


        switch (setpoint) {
            case 0: {
                target = RobotInterface.lowangle;
                setangle(target, angle, pot);
                break;
            }
            case 1: {
                target = RobotInterface.midangle;
                setangle(target, angle, pot);
                break;
            }
            case 2: {
                target = RobotInterface.highangle;
                setangle(target, angle, pot);
                break;
            }
        }
    }

    private void setangle(double angle, SpeedController anglemotor, AnalogChannel pot) {
        if(previoussetpoint != setpoint) {
            if (pot.getAverageValue() <= angle + 10 && pot.getAverageValue() >= angle - 10) {
                anglemotor.set(0);
                previoussetpoint = setpoint;
                return;
            }

            if (pot.getAverageValue() > angle) {
                anglemotor.set(RobotInterface.AngleUpSetSpeed);
            }
            if (pot.getAverageValue() < angle) {
                anglemotor.set(RobotInterface.AngleDownSetSpeed);
            }
        }else{
            anglemotor.set(0);
        }
        if(pot.getAverageValue() <= RobotInterface.SPC_In_Top){
            anglemotor.set(0);
        }
        if(pot.getAverageValue() >= RobotInterface.SPC_In_Bot){
            anglemotor.set(0);
        }

    }

    private void changeSpeed(Joystick controller, SpeedController shooter, int direction) {
        // Based on Direction selected, determine speed preset and set scroll constant.
        switch (direction) {
            case 1: {
                shotSpeed += shotSpeed == 3 ? 0 : 1;
                if (shotSpeed > 3) {
                    shotSpeed = 3;
                }
                break;
            }
            case 0: {
                shotSpeed -= shotSpeed == 0 ? 0 : 1;
                if (shotSpeed < 0) {
                    shotSpeed = 0;
                }
                break;
            }
        }

        //Based on speed preset, set jaguar rate of speed
        switch (shotSpeed) {
            case 0: {
                speed = 0;
                break;
            }

            case 1: {
                speed = RobotInterface.lowRate;
                break;
            }
            case 2: {
                speed = RobotInterface.midRate;
                break;
            }
            case 3: {
                speed = RobotInterface.highRate;
                break;
            }
        }
        //Ensure speed rate is within tollerance range.  Saftey Check!!!
        if (speed > 1) {
            speed = 1;
        }
        if (speed < 0) {
            speed = 0;
        }

        RunMotor(shooter, speed);


    }
    private void changeSpeedScroll(Joystick controller, SpeedController shooter){
        // If scroll control is activated apply scroll ratio to speed
        if(controller.getRawButton(RobotInterface.SHOOT_MORE)){
            scroll = RobotInterface.scrollUp;
            speed = (speed + scroll);
        }
        if(controller.getRawButton(RobotInterface.SHOOT_LESS)){
            scroll = RobotInterface.scrollDown;
            speed = (speed + scroll);
        }
        

        //Ensure speed rate is within tollerance range.  Saftey Check!!!
        if (speed > 1) {
            speed = 1;
        }
        if (speed < 0) {
            speed = 0;
        }

        RunMotor(shooter, speed);
    }

    private void RunMotor(SpeedController shooter, double speed) {
        shooter.set(speed);
    }
}
