package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PieceConstants;

public class SimpleIntakeMoveSubsystem extends SubsystemBase { 
    private final CANSparkMax intakeMoveMotor = new CANSparkMax(PieceConstants.intakeMove, MotorType.kBrushless);

    public SimpleIntakeMoveSubsystem() {}
    
    public void simpleIntakeMove (double speed) {
        intakeMoveMotor.set(speed);
    } 
}
