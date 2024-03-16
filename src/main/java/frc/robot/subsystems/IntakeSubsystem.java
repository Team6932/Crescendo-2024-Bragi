package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PieceConstants;

public class IntakeSubsystem extends SubsystemBase{
    private final CANSparkMax leftIntakeMotor = new CANSparkMax(PieceConstants.intakeLeftUpId, MotorType.kBrushless);
    private final CANSparkMax rightIntakeMotor = new CANSparkMax(PieceConstants.intakeRightDownId, MotorType.kBrushless);
    //private final SparkLimitSwitch intakeLimit = leftIntakeMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);

    //DigitalInput limitSwitch = new DigitalInput(0);

    public IntakeSubsystem() {}

    /*public boolean intakeLimit() {
        return intakeLimit.isPressed();
    } */

    /*public boolean getLimitSwitch() {
        return limitSwitch.get();
    } */

    public void intake (double leftSpeed, double rightSpeed) {
        leftIntakeMotor.set(leftSpeed);
        rightIntakeMotor.set(rightSpeed);
    }

   /* @Override
    public void periodic() {
        SmartDashboard.putBoolean("limit switch", getLimitSwitch());
    } */
}
