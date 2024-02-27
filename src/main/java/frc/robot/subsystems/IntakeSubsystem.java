package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PieceConstants;

public class IntakeSubsystem extends SubsystemBase{
    private final CANSparkMax leftIntake = new CANSparkMax(PieceConstants.intakeLeft, MotorType.kBrushless);
    private final CANSparkMax rightIntake = new CANSparkMax(PieceConstants.intakeRight, MotorType.kBrushless);

    public IntakeSubsystem() {}

    public void intake (double speed) {
        leftIntake.set(speed);
        rightIntake.set(speed);
    }
}
