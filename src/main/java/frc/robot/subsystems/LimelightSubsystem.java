package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
    private double tv, tx, ty, ta, pipeline, camMode;
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
        pipeline = limelightTable.getEntry("pipeline").getDouble(9);
        camMode = limelightTable.getEntry("camMode").getDouble(0);

        SmartDashboard.putBoolean("valid target", getTV());
        SmartDashboard.putNumber("horizontal deg", tx);
        SmartDashboard.putNumber("vertical deg", ty); 
        SmartDashboard.putNumber("horizontal m", getXMeters());
        SmartDashboard.putNumber("vertical m", getYMeters());
        SmartDashboard.putNumber("pipeline (9 = null)", pipeline);
        SmartDashboard.putNumber("camera (1 = drive camera)", camMode);

        SmartDashboard.putBoolean("shootAligned", getShootAligned());
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
    public boolean getTV() {
        if (tv == 1.0) {
            return true;
        } else {
            return false;
        }
    } 

    public void setLimelightAlliance() { // true for blue, false for red 
        if (DriverStation.getAlliance().get() == Alliance.Red) {
            limelightTable.getEntry("priorityid").setNumber(LimelightConstants.redSpeakerId);
        } else {
            limelightTable.getEntry("priorityid").setNumber(LimelightConstants.blueSpeakerId);
        }
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

    public Pose2d getLimelightPose2d() {

        double xMeters = getXMeters();
        double yMeters = getYMeters();
        double rotDeg = getRotDeg();

        return new Pose2d(new Translation2d(xMeters, yMeters), Rotation2d.fromDegrees(rotDeg));
    }

    public double getXMeters () {
        double[] xMetersArray = limelightTable.getEntry("targetpose_cameraspace").getDoubleArray(new double [1]);
        return xMetersArray[0] - LimelightConstants.speakerBumper;
    }

    public double getYMeters () {
        double[] yMetersArray = limelightTable.getEntry("targetpose_cameraspace").getDoubleArray(new double [2]);
        return yMetersArray[0];
    }

    public double getRotDeg () {
        double[] rotDegArray = limelightTable.getEntry("targetpose_cameraspace").getDoubleArray(new double [5]);
        return rotDegArray[0];
    }

    public boolean getShootAligned () {
        if (getXMeters() < LimelightConstants.xDistError && 
            getYMeters() < LimelightConstants.yDistError && 
            getRotDeg() < LimelightConstants.rotDegError) {
            return true;
        } else {
            return false;
        }
    }

    public void setAprilTag() {
        limelightTable.getEntry("pipeline").setNumber(1);
    }

    public void setCamera() {
        limelightTable.getEntry("pipeline").setNumber(0);
    }

    public void setNeuralNetwork() {
        limelightTable.getEntry("pipeline").setNumber(2);
    }

    public void setVisionMode (int camModeIndex) {
        limelightTable.getEntry("camMode").setNumber(camModeIndex);
    }
} 
