package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.SparkLimitSwitch;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PieceConstants;

public class IntakeMoveSubsystem extends SubsystemBase { 
    private final CANSparkMax intakeMoveMotor = new CANSparkMax(PieceConstants.intakeMoveId, MotorType.kBrushless);
    private final SparkPIDController intakeMovePID = intakeMoveMotor.getPIDController();
    private final RelativeEncoder intakeMoveEncoder = intakeMoveMotor.getEncoder();
    
    // these 2 limit switches currently unused
    //private final SparkLimitSwitch outLimitSwitch = intakeMoveMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);
    //private final SparkLimitSwitch inLimitSwitch = intakeMoveMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);

    public IntakeMoveSubsystem() {}
    /*
    // unused 
    public boolean outLimitSwitch() {
        return outLimitSwitch.isPressed();
    }

    // unused
    public boolean inLimitSwitch() {
        return inLimitSwitch.isPressed();
    } */


    public void intakeMove (double degrees, double P, double I, double D) {
        intakeMovePID.setP(P);
        intakeMovePID.setI(I);
        intakeMovePID.setD(D);
        intakeMovePID.setReference(degrees, ControlType.kPosition);
    } 

    public void simpleIntakeMove (double speed) {
        intakeMoveMotor.set(speed);
        intakeMoveMotor.setSecondaryCurrentLimit(4);
    }

    public double getIntakeEncoder() {
        return intakeMoveEncoder.getPosition();
    }

    @Override
    public void periodic () {
        SmartDashboard.putNumber("intakeMoveValue", getIntakeEncoder());
    }
}
