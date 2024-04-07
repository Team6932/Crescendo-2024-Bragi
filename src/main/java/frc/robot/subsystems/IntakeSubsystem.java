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
    private final CANSparkMax leftIntakeMotor = new CANSparkMax(PieceConstants.intakeLeftUpId, MotorType.kBrushless);
    private final CANSparkMax rightIntakeMotor = new CANSparkMax(PieceConstants.intakeRightDownId, MotorType.kBrushless);

    private final RelativeEncoder leftIntakeEncoder = leftIntakeMotor.getEncoder();
    private final RelativeEncoder rightIntakeEncoder = rightIntakeMotor.getEncoder();

    DigitalInput intakeSwitch = new DigitalInput(PieceConstants.intakeSwitch);

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

    public boolean getIntakeSwitch() {
        return !intakeSwitch.get();
    } 

    public void intake (double leftSpeed, double rightSpeed) {
        leftIntakeMotor.set(leftSpeed);
        rightIntakeMotor.set(rightSpeed);
    }

    public double getLeftIntakeEncoder () {
        return leftIntakeEncoder.getVelocity();
    }

    public double getRightIntakeEncoder () {
        return rightIntakeEncoder.getVelocity();
    }

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

   @Override
    public void periodic() {
        SmartDashboard.putBoolean("Piece?", getIntakeSwitch());
        SmartDashboard.putNumber("leftIntake", getLeftIntakeEncoder());
        SmartDashboard.putNumber("rightIntake", getRightIntakeEncoder());
        SmartDashboard.putBoolean("intakeISSUE", getIntakeError());
    } 
}
