package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.ShuffleboardInfo

public class Limelight extends SubsystemBase{  
    private final NetworkTable limelightTable;
    private double tv, tx, ty, ta;
    private ArrayList<Double> targetList;
    private final int MAX_ENTRIES = 50;
    private final NetworkTableEntry isTargetValid, ledEntry;

    public Limelight() {
        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
        targetList = new ArrayList<Double>(MAX_ENTRIES);
        isTargetValid = limelightTable.getEntry("tv");
        ledEntry = limelightTable.getEntry("ledMode");
    }

    @Override
    public void periodic() {
        tv = limelightTable.getEntry("tv").getDouble(0);
        tx = limelightTable.getEntry("tx").getDouble(0);
        ty = limelightTable.getEntry("ty").getDouble(0);
        ta = limelightTable.getEntry("ta").getDouble(0);
    }

    public double getTX() {
        return tx;
    }

    public double getTY() {
        return ty;
    }

    
} 
