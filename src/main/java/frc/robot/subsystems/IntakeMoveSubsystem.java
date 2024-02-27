package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PieceConstants;

public class IntakeMoveSubsystem extends SubsystemBase{
    private CANSparkMax intakeMoveMotor = new CANSparkMax(PieceConstants.intakeMove, MotorType.kBrushless);

    public IntakeMoveSubsystem() {}

    public void intakeMove (double speed) {
        intakeMoveMotor.set(speed);
    }
}
