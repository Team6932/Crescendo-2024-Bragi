/*
 * This is the subsystem in charge of moving the two "rollers"/belts on our intake system. 
 * This is used to grab pieces from the gorund and to feed pieces into the shooting system. 
 */
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PieceConstants;

public class IntakeSubsystem extends SubsystemBase{

    /*
     * This uses two SparkMAX Motor Controllers, two encoders, and a limit switch (digital input). 
     */
    private final CANSparkMax leftIntakeMotor = new CANSparkMax(PieceConstants.intakeLeftUpId, MotorType.kBrushless);
    private final CANSparkMax rightIntakeMotor = new CANSparkMax(PieceConstants.intakeRightDownId, MotorType.kBrushless);

    private final RelativeEncoder leftIntakeEncoder = leftIntakeMotor.getEncoder();
    private final RelativeEncoder rightIntakeEncoder = rightIntakeMotor.getEncoder();

    DigitalInput intakeSwitch = new DigitalInput(PieceConstants.intakeSwitch);

    /*
     * This makes sure brake mode and the smart current limit is set. 
     */
    public IntakeSubsystem() {
        leftIntakeMotor.restoreFactoryDefaults();
        rightIntakeMotor.restoreFactoryDefaults();

        leftIntakeMotor.setSmartCurrentLimit(PieceConstants.intakeCurrent);
        leftIntakeMotor.setIdleMode(IdleMode.kBrake);

        rightIntakeMotor.setSmartCurrentLimit(PieceConstants.intakeCurrent);
        rightIntakeMotor.setIdleMode(IdleMode.kBrake);
        
        leftIntakeMotor.burnFlash();
        rightIntakeMotor.burnFlash();
    }

    /*
     * This gets if the limit switch is pressed or not. 
     * The "!" makes the method return the opposite of what intakeSwitch.get() says (it outputs true when switch is not pressed)
     */
    public boolean getIntakeSwitch() {
        return !intakeSwitch.get();
    } 

    /*
     * This sets the two motors to two given speeds.
     */
    public void intake (double leftSpeed, double rightSpeed) {
        leftIntakeMotor.set(leftSpeed);
        rightIntakeMotor.set(rightSpeed);
    }

    /*
     * This gets the velocity value of the left encoder.
     */
    public double getLeftIntakeEncoder () {
        return leftIntakeEncoder.getVelocity();
    }

    /*
     * This gets the velocity value of the right encoder. 
     */
    public double getRightIntakeEncoder () {
        return rightIntakeEncoder.getVelocity();
    }

    /*
     * This method and getShootIssue() from ShootSubsystem.java were created in a last minute effort. 
     * If our intake system broke and the motors were not spinning properly, the drivers were slow to transition to defense. 
     * This return a boolean that can be displayed.
     * 
     * If either the left or right motor is spinning while the other is very slow, return true. 
     * If there is a large difference between the motor speeds, return true. 
     * 
     * The values are not tested and were arbitrarily set. 
     */
    public boolean getIntakeError() {

        if (Math.abs(getLeftIntakeEncoder()) >= 500 && Math.abs(getRightIntakeEncoder()) <= 200) {
            return true;

        } else if (Math.abs(getRightIntakeEncoder()) >= 500 && Math.abs(getLeftIntakeEncoder()) <= 200) {
            return true;

        } else if (Math.abs(getLeftIntakeEncoder() - getRightIntakeEncoder()) >= 1000) {
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
        SmartDashboard.putBoolean("Piece?", getIntakeSwitch());
        SmartDashboard.putNumber("leftIntake", getLeftIntakeEncoder());
        SmartDashboard.putNumber("rightIntake", getRightIntakeEncoder());
        SmartDashboard.putBoolean("intakeISSUE", getIntakeError());
    } 
}
