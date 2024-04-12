/*
 * This is the subsystem in charge of controlling the two wheels we use to shoot game pieces.
 */
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PieceConstants;

public class ShootSubsystem extends SubsystemBase {

    /*
     * This uses two SparMAX Motor Controllers and two encoders. 
     * PID Controllers are still present in the code, but we did not use them (I didn't want to deal with PID tuning).
     */
    private final CANSparkMax leftShootMotor = new CANSparkMax(PieceConstants.leftShootId, MotorType.kBrushless);
    private final CANSparkMax rightShootMotor = new CANSparkMax(PieceConstants.rightShootId, MotorType.kBrushless);


    private final SparkPIDController leftShootPID = leftShootMotor.getPIDController();
    private final SparkPIDController rightShootPID = rightShootMotor.getPIDController();
    private final RelativeEncoder leftShootEncoder = leftShootMotor.getEncoder();
    private final RelativeEncoder rightShootEncoder = rightShootMotor.getEncoder();

    /*
     * This makes sure brake mode and the smart current limit is set.
     */
    public ShootSubsystem() {
        leftShootMotor.restoreFactoryDefaults();
        rightShootMotor.restoreFactoryDefaults();

        leftShootMotor.setSmartCurrentLimit(PieceConstants.shootCurrent);
        leftShootMotor.setIdleMode(IdleMode.kBrake);

        rightShootMotor.setSmartCurrentLimit(PieceConstants.shootCurrent);
        rightShootMotor.setIdleMode(IdleMode.kBrake);

        leftShootMotor.burnFlash();
        rightShootMotor.burnFlash();
    }
    
    /*
     * This sets the two motors to two given speeds.
     * 
     * The left speed has a negative since it needs to be inverted.
     * Another option would be to have leftShootMotor.setInverted(true);
     */
    public void shoot (double leftSpeed, double rightSpeed) {
        leftShootMotor.set(-leftSpeed);
        rightShootMotor.set(rightSpeed);
    }

    /*
     * This gets the velocity value of the left encoder.
     */
    public double getLeftShootEncoder() {
        return leftShootEncoder.getVelocity() * -1;
    }

    /*
     * This gets the velocity value of the right encoder.
     */
    public double getRightShootEncoder() {
        return rightShootEncoder.getVelocity();
    }
    
    /*
     * This was an attempt at using PID to control our shooting system.
     * It was not used because I was lazy and had better things to do during my time with the robot. 
     * It seemed like PID values had to be at least somewhat tuned properly for it to work. 
     * 
     * ControlType.kVelocity tells the PID controller to go to a set velocity (not position). 
     */
    public void shootAuto (double rpm, double P, double I, double D) {
        leftShootPID.setP(P);
        leftShootPID.setI(I);
        leftShootPID.setD(D);

        rightShootPID.setP(P);
        rightShootPID.setI(I);
        rightShootPID.setD(D);

        leftShootPID.setReference(rpm, ControlType.kVelocity);
        rightShootPID.setReference(rpm, ControlType.kVelocity);
    }

    /*
     * This method returns a boolean that returns true if both motors are running above a desired speed. 
     * It is used to ensure the shooting motors have reached the needed speed before trying to shoot. 
     */
    public boolean getShootReady(double motorSpeeds) {
        if (getLeftShootEncoder() >= motorSpeeds && getRightShootEncoder() >= motorSpeeds) {
            return true;
        } else {
            return false;
        }
    }

    /*
     * This method returns a boolean that returns true if the right motor is running above a desired speed.
     */
    public boolean getRightShootReady(double rightSpeed) {
        if (getRightShootEncoder() >= rightSpeed) {
            return true;
        } else {
            return false;
        }
    }

    /*
     * This method returns a boolean that returns true if the left motor is running above a desired speed. 
     */
    public boolean getLeftShootReady(double leftSpeed) {
        if (getLeftShootEncoder() >= leftSpeed) {
            return true;
        } else {
            return false;
        }
    }

    /*
     * This method and getIntakeError() from IntakeSubsystem.java were created in a last minute effort. 
     * If our shooting system broke and the motors were not spinning properly, the drivers were slow to transition to defense. 
     * This return a boolean that can be displayed.
     * 
     * If either the left or right motor is spinning while the other is very slow, return true. 
     * If there is a large difference between the motor speeds, return true. 
     * 
     * The values are not tested and were arbitrarily set. 
     */
    public boolean getShootIssue() {

        if (Math.abs(getLeftShootEncoder()) >= 1000 && Math.abs(getRightShootEncoder()) <= 500) {
            return true;

        } else if (Math.abs(getRightShootEncoder()) >= 1000 && Math.abs(getLeftShootEncoder()) <= 500) {
            return true;

        } else if (Math.abs(getLeftShootEncoder() - getRightShootEncoder()) >= 750) {
            return true;

        } else {
            return false;
        }
    }

    /*
     * This displays information.
     */
    @Override
    public void periodic() {
        SmartDashboard.putNumber("leftShoot", getLeftShootEncoder());
        SmartDashboard.putNumber("rightShoot", getRightShootEncoder());
        SmartDashboard.putBoolean("shootReady", getShootReady(PieceConstants.speakerMotorSpeed));
        SmartDashboard.putBoolean("shootISSUE", getShootIssue());
    }
}
