package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
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

    public ShootSubsystem() {}
    
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

    @Override
    public void periodic() {
        SmartDashboard.putNumber("leftShoot", getLeftShootEncoder());
        SmartDashboard.putNumber("rightShoot", getRightShootEncoder());
        SmartDashboard.putBoolean("shootReady", getShootReady(PieceConstants.speakerMotorSpeed));
    }
}
