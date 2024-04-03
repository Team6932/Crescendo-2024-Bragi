package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PieceConstants;

public class IntakeMoveSubsystem extends SubsystemBase { 
    private final CANSparkMax intakeMoveMotor = new CANSparkMax(PieceConstants.intakeMoveId, MotorType.kBrushless);
    private final SparkPIDController intakeMovePID = intakeMoveMotor.getPIDController();
    private final RelativeEncoder intakeMoveEncoder = intakeMoveMotor.getEncoder();
    
    //DigitalInput intakeOutSwitch = new DigitalInput(PieceConstants.intakeOutSwitch);
    //DigitalInput intakeInSwitch = new DigitalInput(PieceConstants.intakeInSwitch);

    public IntakeMoveSubsystem() {
        intakeMoveMotor.restoreFactoryDefaults();

        intakeMoveMotor.setSmartCurrentLimit(PieceConstants.intakeMoveCurrent);
        intakeMoveMotor.setIdleMode(IdleMode.kBrake);

        intakeMoveMotor.burnFlash();
    }

    /*public boolean getIntakeOutSwitch() {
        return intakeOutSwitch.get();
    }

    public boolean getIntakeInSwitch() {
        return intakeInSwitch.get();
    } */

    public void intakeMove (double degrees, double P, double I, double D) {
        intakeMovePID.setP(P);
        intakeMovePID.setI(I);
        intakeMovePID.setD(D);
        intakeMovePID.setOutputRange(-PieceConstants.maxIntakeMovePID, PieceConstants.maxIntakeMovePID);
        intakeMovePID.setReference(degrees, ControlType.kPosition);
    } 

    public void simpleIntakeMove (double speed) {
        intakeMoveMotor.set(speed);
        intakeMoveMotor.setSecondaryCurrentLimit(4);
    }

    public double getIntakeEncoder() {
        return intakeMoveEncoder.getPosition();
    }

    public void resetEncoder() {
        intakeMoveEncoder.setPosition(0);
    }

    public boolean getNeedReset() {
        if (getIntakeEncoder() > 0.8) { // arbitrary encoder value I believe will cause issues
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void periodic () {
        SmartDashboard.putNumber("intakeMoveValue", getIntakeEncoder());
        /*SmartDashboard.putBoolean("intakeOut", getIntakeOutSwitch());
        SmartDashboard.putBoolean("intakeIn", getIntakeInSwitch());*/
        SmartDashboard.putBoolean("pieceRESET", getNeedReset());
    }
}
