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
    private final CANSparkMax leftShootMotor = new CANSparkMax(PieceConstants.leftShootId, MotorType.kBrushless);
    private final CANSparkMax rightShootMotor = new CANSparkMax(PieceConstants.rightShootId, MotorType.kBrushless);


    private final SparkPIDController leftShootPID = leftShootMotor.getPIDController();
    private final SparkPIDController rightShootPID = rightShootMotor.getPIDController();
    private final RelativeEncoder leftShootEncoder = leftShootMotor.getEncoder();
    private final RelativeEncoder rightShootEncoder = rightShootMotor.getEncoder();

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
    
    public void shoot (double leftSpeed, double rightSpeed) {
        leftShootMotor.set(-leftSpeed);
        rightShootMotor.set(rightSpeed);
    }

    public double getLeftShootEncoder() {
        return leftShootEncoder.getVelocity() * -1;
    }

    public double getRightShootEncoder() {
        return rightShootEncoder.getVelocity();
    }
    
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

    public boolean getShootReady(double motorSpeeds) {
        if (getLeftShootEncoder() >= motorSpeeds && getRightShootEncoder() >= motorSpeeds) {
            return true;
        } else {
            return false;
        }
    }

    public boolean getRightShootReady(double rightSpeed) {
        if (getRightShootEncoder() >= rightSpeed) {
            return true;
        } else {
            return false;
        }
    }

    public boolean getLeftShootReady(double leftSpeed) {
        if (getLeftShootEncoder() >= leftSpeed) {
            return true;
        } else {
            return false;
        }
    }

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

    @Override
    public void periodic() {
        SmartDashboard.putNumber("leftShoot", getLeftShootEncoder());
        SmartDashboard.putNumber("rightShoot", getRightShootEncoder());
        SmartDashboard.putBoolean("shootReady", getShootReady(PieceConstants.speakerMotorSpeed));
        SmartDashboard.putBoolean("shootISSUE", getShootIssue());
    }
}
