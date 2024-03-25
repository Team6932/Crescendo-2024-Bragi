package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;
import frc.robot.utils.LimelightHelpers;

public class LimelightSubsystem extends SubsystemBase{

    public double getPipeline() { // get what pipeline the Limelight is on
        return LimelightHelpers.getCurrentPipelineIndex("limelight");
    }

    public boolean getAlliance() { // get what alliance color and set the priority April Tag depending on color
        if (DriverStation.getAlliance().get() == Alliance.Red) {
            LimelightHelpers.setPriorityTagID("limelight", LimelightConstants.redSpeakerId);
            return false;
        } else {
            LimelightHelpers.setPriorityTagID("limelight", LimelightConstants.blueSpeakerId);
            return true;
        }
    }

    public boolean getHasTarget() { // get if the Limelight has a target
        return LimelightHelpers.getTV("limelight");
    }

    public double getXAngle() { // get horizontal angle of target from camera in deg
        return LimelightHelpers.getTX("limelight");
    }
    
    public double getYAngle() { // get vertical angle of target from camera in deg
        return LimelightHelpers.getTY("limelight");
    }

    public double[] getTargetPoseOfCamera() { // get position of April Tag using the camera as the reference
        return LimelightHelpers.getTargetPose_CameraSpace("limelight"); // returns a doubleArray
    }

    public double getYaw() { // get yaw component of position in deg
        return getTargetPoseOfCamera()[5];
    }

    public double getXMeters() { // get horizontal component of position in m
        return getTargetPoseOfCamera()[1];
    }

    public double getYMeters() { // get vertical component of position in m
        return getTargetPoseOfCamera()[2];
    }

    public double getAprilTagId() { // get April Tag Id
        return LimelightHelpers.getFiducialID("limelight");
    }
    
    public void cameraOn() { // turn the LEDs of the camera on
        LimelightHelpers.setLEDMode_ForceOn("limelight");
    }

    public void cameraOff() { // turn the LEDs of the camera off
        LimelightHelpers.setLEDMode_ForceOff("limelight");
    }

    public boolean getInPosition() {
        if (getXMeters() < LimelightConstants.xDistError && 
            getYMeters() < LimelightConstants.yDistError && 
            getYaw() < LimelightConstants.rotDegError) {
            return true;
        } else {
            return false;
        }
    }

    // COPY PASTED FROM LYNK
    public double getRot() { // angle error is deg from April Tag
        double angleErrorDeg = getYaw();
        double magnitude = Math.abs(angleErrorDeg);
        double rotationVal = 0.0;

        // controls how much the robot rotates
        if (magnitude > 10.0) {
            rotationVal = 0.10 * Math.signum(angleErrorDeg);
        } else if (magnitude > 7.0) {
            rotationVal = 0.06 * Math.signum(angleErrorDeg);
        } else if (magnitude > 3.0) {
            rotationVal = 0.04 * Math.signum(angleErrorDeg);
        } else if (magnitude > 0.3) {
            rotationVal = 0.03 * Math.signum(angleErrorDeg);
        }
        return rotationVal;
    }
    
    @Override
    public void periodic() {
        SmartDashboard.putBoolean("inShootPosition", getInPosition());
    }
}
