package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PieceConstants;

public class ShootSubsystem extends SubsystemBase {
    private final CANSparkMax leftShootMotor = new CANSparkMax(PieceConstants.leftShootId, MotorType.kBrushless);
    private final CANSparkMax rightShootMotor = new CANSparkMax(PieceConstants.rightShootId, MotorType.kBrushless);

    public ShootSubsystem() {}
    
    public void shoot (double leftSpeed, double rightSpeed) {
        leftShootMotor.set(-leftSpeed);
        rightShootMotor.set(rightSpeed);
    }
}
