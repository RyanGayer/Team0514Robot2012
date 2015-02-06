/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package edu.mphs.first.utils;

import edu.mphs.first.interfaces.RobotInterface;

import com.sun.squawk.util.MathUtils;
import edu.wpi.first.wpilibj.Kinect;
import edu.wpi.first.wpilibj.Skeleton;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotDrive;


/**
 *
 * @author marnold
 */
public class KinectUtil {
    //Class variables go here...
    boolean kicked;
    boolean armout;
    public KinectUtil(){
        //Class Constructor
        kicked = true;
        armout = false;
    }
    
    public void manageKinect(Kinect kinect, SpeedController shooter, Solenoid firepin, DoubleSolenoid ballGate,
                             DigitalInput lSwitch, SpeedController angle,RobotDrive drive, DoubleSolenoid bridgepush){
        double leftAxis = 0;
        double rightAxis = 0;
        double leftAngle, rightAngle, headAngle, rightLegAngle, leftLegAngle, rightLegYZ, leftLegYZ;
        boolean dataWithinExpectedRange;
        boolean[] buttons = new boolean[8];


        /* Only process data if skeleton is tracked */
        if (kinect.getSkeleton().GetTrackState() == Skeleton.tTrackState.kTracked) {
                /* Determine angle of each arm and map to range -1,1 */
                leftAngle = AngleXY(kinect.getSkeleton().GetShoulderLeft(), kinect.getSkeleton().GetWristLeft(), true);
                rightAngle = AngleXY(kinect.getSkeleton().GetShoulderRight(), kinect.getSkeleton().GetWristRight(), false);
                leftAxis = CoerceToRange(leftAngle, -70, 70, -1, 1);
                rightAxis = CoerceToRange(rightAngle, -70, 70, -1, 1);

                /* Check if arms are within valid range and at approximately the same z-value */
                dataWithinExpectedRange = leftAngle < RobotInterface.ARM_MAX_ANGLE && 
                                          leftAngle > RobotInterface.ARM_MIN_ANGLE &&
                                          rightAngle < RobotInterface.ARM_MAX_ANGLE && 
                                          rightAngle > RobotInterface.ARM_MIN_ANGLE;
                dataWithinExpectedRange = dataWithinExpectedRange &&
                                          InSameZPlane(kinect.getSkeleton().GetShoulderLeft(),
                                                   kinect.getSkeleton().GetWristLeft(),
                                                   RobotInterface.Z_PLANE_TOLERANCE) &&
                                          InSameZPlane(kinect.getSkeleton().GetShoulderRight(),
                                                   kinect.getSkeleton().GetWristRight(),
                                                   RobotInterface.Z_PLANE_TOLERANCE);

                /* Determine the head angle and use it to set the Head buttons */
                headAngle = AngleXY(kinect.getSkeleton().GetShoulderCenter(), kinect.getSkeleton().GetHead(), false);
                buttons[0] = headAngle > RobotInterface.HEAD_LEFT;
                buttons[1] = headAngle < RobotInterface.HEAD_RIGHT;

                /* Calculate the leg angles in the XY plane and use them to set the Leg Out buttons */
                leftLegAngle = AngleXY(kinect.getSkeleton().GetHipLeft(), kinect.getSkeleton().GetAnkleLeft(), true);
                rightLegAngle = AngleXY(kinect.getSkeleton().GetHipRight(), kinect.getSkeleton().GetAnkleRight(), false);
                buttons[2] = leftLegAngle > RobotInterface.LEG_OUT;
                buttons[3] = rightLegAngle > RobotInterface.LEG_OUT;

                /* Calculate the leg angle in the YZ plane and use them to set the Leg Forward and Leg Back buttons */
                leftLegYZ = AngleYZ(kinect.getSkeleton().GetHipLeft(), kinect.getSkeleton().GetAnkleLeft(), false);
                rightLegYZ = AngleYZ(kinect.getSkeleton().GetHipRight(), kinect.getSkeleton().GetAnkleRight(), false);
                buttons[4] = rightLegYZ < RobotInterface.LEG_FORWARD;
                buttons[5] = rightLegYZ > RobotInterface.LEG_BACKWARD;
                buttons[6] = leftLegYZ < RobotInterface.LEG_FORWARD;
                buttons[7] = leftLegYZ > RobotInterface.LEG_BACKWARD;

                
                
                
                if (dataWithinExpectedRange){
                    
                    if(buttons[6] == true){
                        if(kicked){
                            kicked = false;
                        }else{
                            kicked = true;
                        }
                    }
                    
                    if(kicked){
                        
                        drive.tankDrive(leftAxis, rightAxis);
                        
                        if(armout){   
                            bridgepush.set(DoubleSolenoid.Value.kReverse);
                            if(buttons[4] == true){
                                Timer.delay(1);
                                armout = false;
                            }
                        }else{
                            bridgepush.set(DoubleSolenoid.Value.kForward);
                            if(buttons[4] == true){
                                Timer.delay(1);
                                armout = true;
                            }
                        }    
                    }else{                        
                    /**
                     * Drives using the Kinect axes scaled to 1/3 power
                     * Axes are inverted so arms up == joystick pushed away from you
                     */
                    //drive.tankDrive(-leftAxis*.3, -rightAxis*.3);
                    
                    //Left Arm Up Down drives Shot Motor...
                    shooter.set(leftAxis);
                    
                    //Right Arm Up Down drives Shot Angle...
                    angle.set(rightAxis);
                    
                    if(lSwitch.get()){
                        ballGate.set(DoubleSolenoid.Value.kForward);
                    }else{
                        ballGate.set(DoubleSolenoid.Value.kReverse);
                    }
                    
                    //Right Leg Forward Kick Fires Shot!
                    if(buttons[4] == true){
                        firepin.set(true);
                    }else{
                        firepin.set(false);
                    }
                    
                    }
                    /**
                     * Do something with boolean "buttons" here
                     */

                    /* Optional SmartDashboard display of Kinect values */
                    //SmartDashboard.putDouble("Left Arm", -leftAxis);
                    //SmartDashboard.putDouble("Right Arm", -rightAxis);
                    //SmartDashboard.putBoolean("Head Left", buttons[0]);
                    //SmartDashboard.putBoolean("Head Right", buttons[1]);
                    //...etc...
                
                    }else{
                    /* Arms are outside valid range */

                    drive.tankDrive(0, 0);
                    shooter.set(0);
                    angle.set(0);
                    firepin.set(false);
                    ballGate.set(DoubleSolenoid.Value.kForward);
                    bridgepush.set(DoubleSolenoid.Value.kForward);

                    /**
                     * Do default behavior with boolean "buttons" here
                     */

                    /* Optional SmartDashboard display of Kinect values */
                    //SmartDashboard.putDouble("Left Arm", 0);
                    //SmartDashboard.putDouble("Right Arm", 0);
                    //SmartDashboard.putBoolean("Head Left", false);
                    //SmartDashboard.putBoolean("Head Right", false);
                    //...etc...
                    }
                }else{
                /* Skeleton not tracked */

                drive.tankDrive(0, 0);
                shooter.set(0);
                angle.set(0);
                firepin.set(false);
                ballGate.set(DoubleSolenoid.Value.kForward);
                bridgepush.set(DoubleSolenoid.Value.kForward);

                
                /**
                * Do default behavior with boolean "buttons" here
                */

                /* Optional SmartDashboard display of Kinect values */
                //SmartDashboard.putDouble("Left Arm", 0);
                //SmartDashboard.putDouble("Right Arm", 0);
                //SmartDashboard.putBoolean("Head Left", false);
                //SmartDashboard.putBoolean("Head Right", false);
                //...etc...
            }
            Timer.delay(.01);   /* Delay 10ms to reduce proceessing load*/
        }

    
    /**
     * This method returns the angle (in degrees) of the vector pointing from Origin to Measured
     * projected to the XY plane. If the mirrored parameter is true the vector is flipped about the Y-axis.
     * Mirroring is used to avoid the region where the atan2 function is discontinuous
     * @param origin The Skeleton Joint to use as the origin point
     * @param measured The Skeleton Joint to use as the endpoint of the vector
     * @param mirrored Whether to mirror the X coordinate of the joint about the Y-axis
     * @return The angle in degrees
     */
    private double AngleXY(Skeleton.Joint origin, Skeleton.Joint measured, boolean mirrored){
        return Math.toDegrees(MathUtils.atan2(measured.getY()- origin.getY(),
                (mirrored) ? (origin.getX() - measured.getX()) : (measured.getX() - origin.getX())));
    }

     /**
     * This method returns the angle (in degrees) of the vector pointing from Origin to Measured
     * projected to the YZ plane. If the mirrored parameter is true the vector is flipped about the Y-axis.
     * Mirroring is used to avoid the region where the atan2 function is discontinuous
     * @param origin The Skeleton Joint to use as the origin point
     * @param measured The Skeleton Joint to use as the endpoint of the vector
     * @param mirrored Whether to mirror the Z coordinate of the joint about the Y-axis
     * @return The angle in degrees
     */
    private double AngleYZ(Skeleton.Joint origin, Skeleton.Joint measured, boolean mirrored){
        return Math.toDegrees(MathUtils.atan2(measured.getY()- origin.getY(),
                (mirrored) ? (origin.getZ() - measured.getZ()) : (measured.getZ() - origin.getZ())));
    }

    /**
     * This method checks if two Joints have z-coordinates within a given tolerance
     * @param origin
     * @param measured
     * @param tolerance
     * @return True if the z-coordinates are within tolerance
     */
    private boolean InSameZPlane(Skeleton.Joint origin, Skeleton.Joint measured, double tolerance)
        {
            return Math.abs(measured.getZ() - origin.getZ()) < tolerance;
        }

    /**
     * This method takes an input, an input range, and an output range,
     * and uses them to scale and constrain the input to the output range
     * @param input The input value to be manipulated
     * @param inputMin The minimum value of the input range
     * @param inputMax The maximum value of the input range
     * @param outputMin The minimum value of the output range
     * @param outputMax The maximum value of the output range
     * @return The output value scaled and constrained to the output range
     */
    private double CoerceToRange(double input, double inputMin, double inputMax, double outputMin, double outputMax)
        {
            /* Determine the center of the input range and output range */
            double inputCenter = Math.abs(inputMax - inputMin) / 2 + inputMin;
            double outputCenter = Math.abs(outputMax - outputMin) / 2 + outputMin;

            /* Scale the input range to the output range */
            double scale = (outputMax - outputMin) / (inputMax - inputMin);

            /* Apply the transformation */
            double result = (input + -inputCenter) * scale + outputCenter;

            /* Constrain to the output range */
            return Math.max(Math.min(result, outputMax), outputMin);
        }

}
