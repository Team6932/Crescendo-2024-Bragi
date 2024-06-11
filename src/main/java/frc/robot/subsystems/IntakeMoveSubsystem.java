/*
 * This is the subsystem in charge of moving our intake system in/out.
 * We have an over-the-bumper intake system. 
 */
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

    /*
     * This uses a SparkMAX Motor Controller, a PID Controller, and an encoder. 
     * Everything relating to limit switches is commented out because we never attached any. 
     */
    private final CANSparkMax intakeMoveMotor = new CANSparkMax(PieceConstants.intakeMoveId, MotorType.kBrushless);
    private final SparkPIDController intakeMovePID = intakeMoveMotor.getPIDController();
    private final RelativeEncoder intakeMoveEncoder = intakeMoveMotor.getEncoder();
    
    //DigitalInput intakeOutSwitch = new DigitalInput(PieceConstants.intakeOutSwitch);
    //DigitalInput intakeInSwitch = new DigitalInput(PieceConstants.intakeInSwitch);

    /*
     * This makes sure brake mode and the smart current limit is set. 
     */
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

    /*
     * This moves the intake system using PID controls. 
     * 
     * setP, setI, and setD establish the PID constants. There is also FF (feed forward) that I didn't try because it already worked.
     * They are parameters in this method because we may want different overshoot/undershoot tendencies depending on our goal. 
     * 
     * setOutputRange controls the maximum power level the PID Controller can tell the motor to run at (-1 to 1).
     * 
     * setReference is what makes the motor actually move. 
     * ControlType.kPosition tells the PID controller to go to a set position (not velocity). 
     */
    public void intakeMove (double degrees, double P, double I, double D) {
        intakeMovePID.setP(P);
        intakeMovePID.setI(I);
        intakeMovePID.setD(D);
        intakeMovePID.setOutputRange(-PieceConstants.maxIntakeMovePID, PieceConstants.maxIntakeMovePID);
        intakeMovePID.setReference(degrees, ControlType.kPosition);
    } 


    /*
     * This sets the motor to a given speed. 
     */
    public void simpleIntakeMove (double speed) {
        intakeMoveMotor.set(speed);
    }

    /*
     * This gets the position value of the encoder. 
     */
    public double getIntakeEncoder() {
        return intakeMoveEncoder.getPosition();
    }

    /*
     * This resets the encoder value to 0.
     */
    public void resetEncoder() {
        intakeMoveEncoder.setPosition(0);
    }

    /*
     * This returns a boolean that says if the encoder value is getting off. 
     */
    public boolean getNeedReset() {
        if (getIntakeEncoder() > 0.8 || getIntakeEncoder() < -35.0) { // arbitrary encoder value I believe will cause issues
            return true;
        } else {
            return false;
        }
    }

    /*
     * This displays information. 
     */
    @Override
    public void periodic () {
        SmartDashboard.putNumber("intakeMoveValue", getIntakeEncoder());
        /*SmartDashboard.putBoolean("intakeOut", getIntakeOutSwitch());
        SmartDashboard.putBoolean("intakeIn", getIntakeInSwitch());*/
        SmartDashboard.putBoolean("pieceRESET", getNeedReset());
    }
}
