package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;
import frc.robot.utils.LimelightHelpers;

public class LimelightSubsystem extends SubsystemBase{  
    private final NetworkTable limelightTable;
    private double tv, tx, ty, ta, aprilId;
    private ArrayList<Double> targetList;
    private final int MAX_ENTRIES = 50;
    private final NetworkTableEntry isTargetValid, ledEntry;

    public LimelightSubsystem() {
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
        aprilId = LimelightHelpers.getFiducialID("limelight");

        SmartDashboard.putNumber("valid target", tv);
        SmartDashboard.putNumber("horizontal offset", tx);
        SmartDashboard.putNumber("vertical offset", ty); 
        SmartDashboard.putNumber("test", 1.5);   
        SmartDashboard.putNumber("april tag", aprilId);
    }

    /**
     * Get horizontal offset from crosshair to target in radians
     */
    public double getTX() {
        return Math.toRadians(tx);
    }

    /**
     * Get vertical offset from crosshair to target in radians
     */
    public double getTY() {
        return Math.toRadians(ty);
    }

    /**
     * Get if there are valid targets (int 0 for false and int 1 for true)
     */
    public double getTV() {
        return tv;
    } 

    public double getDist() {
        return (-LimelightConstants.limelightHeight) / Math.tan(getTY());
    }

    public Pose2d getLimelightPosition() {
        return new Pose2d(new Translation2d(
            getDist() * Math.cos(getTX()), 
            getDist() * Math.sin(getTX())), 
            new Rotation2d(getTX()));
    }
} 
