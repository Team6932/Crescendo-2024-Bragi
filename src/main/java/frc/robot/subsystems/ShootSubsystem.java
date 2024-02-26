package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PieceConstants;

public class ShootSubsystem extends SubsystemBase {
    private final CANSparkMax leftShoot = new CANSparkMax(PieceConstants.leftShootId, MotorType.kBrushless);
    private final CANSparkMax rightShoot = new CANSparkMax(PieceConstants.rightShootId, MotorType.kBrushless);

    public ShootSubsystem() {}
    
    public void shoot (double leftSpeed, double rightSpeed) {
        leftShoot.set(-leftSpeed);
        rightShoot.set(rightSpeed);
    }
}
