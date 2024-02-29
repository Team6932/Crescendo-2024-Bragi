package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PieceConstants;

public class IntakeMoveSubsystem extends SubsystemBase {
    private final CANSparkMax intakeMoveMotor = new CANSparkMax(PieceConstants.intakeMove, MotorType.kBrushless);
    private final SparkPIDController intakeMovePID = intakeMoveMotor.getPIDController();

    public IntakeMoveSubsystem() {}
    
    public void intakeMove (double degrees) {
        intakeMovePID.setP(PieceConstants.defaultP);
        intakeMovePID.setReference(degrees, ControlType.kPosition);
    }
}

