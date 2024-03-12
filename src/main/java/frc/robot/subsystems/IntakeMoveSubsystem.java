package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.SparkLimitSwitch;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PieceConstants;

public class IntakeMoveSubsystem extends SubsystemBase { 
    private final CANSparkMax intakeMoveMotor = new CANSparkMax(PieceConstants.intakeMoveId, MotorType.kBrushless);
    private final SparkPIDController intakeMovePID = intakeMoveMotor.getPIDController();
    private final SparkLimitSwitch outLimitSwitch = intakeMoveMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);
    private final SparkLimitSwitch inLimitSwitch = intakeMoveMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);

    public IntakeMoveSubsystem() {}
    
    public boolean outLimitSwitch() {
        return outLimitSwitch.isPressed();
    }

    public boolean inLimitSwitch() {
        return inLimitSwitch.isPressed();
    }

    public void intakeMove (double degrees, double P) {
        intakeMovePID.setP(P);
        intakeMovePID.setReference(degrees, ControlType.kPosition);
    } 

    public void simpleIntakeMove (double speed) {
        intakeMoveMotor.set(speed);
    }
}
