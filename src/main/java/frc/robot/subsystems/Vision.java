// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Vision {
    // Values for the Shed in late January
    private static final double CUSTOM_FIELD_LENGTH = 8.780;    // meters
    private static final double CUSTOM_FIELD_WIDTH = 6.0;       // meters
    private static final AprilTagFieldLayout SHED_TAG_FIELD_LAYOUT = 
            new AprilTagFieldLayout(new ArrayList<AprilTag>() {
                {
                    add(constructTag(26, 0, 1.636, 0.865, 0));
                    add(constructTag(25, 0, 3.24, 0.895, 0));
                    add(constructTag(24, 1.915, 0, 0.857, 90));
                    add(constructTag(23, 4.958, 0, 0.845, 90));
                    add(constructTag(22, 7.763, 0, 0.896, 90));
                    add(constructTag(21, 8.780, 1.373, 0.895, 180));
                    add(constructTag(20, 8.780, 2.392, 0.946, 180));
                }
            }, CUSTOM_FIELD_LENGTH, CUSTOM_FIELD_WIDTH);

    private final PhotonCamera m_aprilTagCamera = new PhotonCamera("ApriltagCamera");
    private final AprilTagFieldLayout m_aprilTagFieldLayout;
    
    // Forward B&W camera for Apriltags
    // relative position of the camera on the robot ot the robot center
    private final Transform3d m_robotToAprilTagCam = new Transform3d(
        new Translation3d(Units.inchesToMeters(31.25 / 2.0), 0.0, Units.inchesToMeters(21.0)),
        new Rotation3d(0.0, 0.0, 0.0)); 

    private final PhotonPoseEstimator m_photonPoseEstimator;

    private double m_lastImageTimeStamp = -1.0;

    public Vision() {
        // try{
        // 	m_aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
        // } catch(IOException e){
        // 	System.out.println("Unable to load AprilTag layout" + e.getMessage());
        // 	m_aprilTagFieldLayout = null;
        // }
        m_aprilTagFieldLayout = SHED_TAG_FIELD_LAYOUT;
        System.out.println("Vision is currently using: SHED_TAG_FIELD_LAYOUT");

        m_photonPoseEstimator = new PhotonPoseEstimator(m_aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
                m_aprilTagCamera, m_robotToAprilTagCam);

        // set the driver mode to false
        m_aprilTagCamera.setDriverMode(false);
    }

    public void updateOdometry(SwerveDrivePoseEstimator odometry) {
        if (!m_aprilTagCamera.isConnected())
            return;

        var targetResult = m_aprilTagCamera.getLatestResult();
        double curImageTimeStamp = targetResult.getTimestampSeconds();

        if (curImageTimeStamp <= m_lastImageTimeStamp) 
            return;

        m_lastImageTimeStamp = curImageTimeStamp;

        SmartDashboard.putBoolean("vision/hasTargets", targetResult.hasTargets());
        if (!targetResult.hasTargets()) 
            return;

        // Get the current best target.
        PhotonTrackedTarget target = targetResult.getBestTarget();
        SmartDashboard.putNumber("vision/targetID", target.getFiducialId());
        Transform3d cameraToTarget = target.getBestCameraToTarget();
        SmartDashboard.putNumber("vision/tagOffsetX", cameraToTarget.getX());
        SmartDashboard.putNumber("vision/tagOffsetY", cameraToTarget.getY());
        SmartDashboard.putNumber("vision/tagOffsetYaw", Math.toDegrees(cameraToTarget.getRotation().getZ()));

        if (m_aprilTagFieldLayout == null)
            return;
        
        // Estimate the robot pose.
        // If successful, update the odometry using the timestamp of the measurement
        Optional<EstimatedRobotPose> result = getEstimatedGlobalPose(odometry.getEstimatedPosition());
        if (result.isPresent()) {
            EstimatedRobotPose camPose = result.get();
            var estimatedPose = camPose.estimatedPose;
            odometry.addVisionMeasurement(estimatedPose.toPose2d(), curImageTimeStamp);
            SmartDashboard.putNumber("vision/estimatedPoseX", estimatedPose.getX());
            SmartDashboard.putNumber("vision/estimatedPoseY", estimatedPose.getY());
            SmartDashboard.putNumber("vision/estimatedPoseZ", estimatedPose.getRotation().getAngle());
        // } else {
        //     // move it way off the screen to make it disappear
        //     SmartDashboard.putNumber("vision/estimatedPoseX", 0);
        //     SmartDashboard.putNumber("vision/estimatedPoseY", 0);
        //     SmartDashboard.putNumber("vision/estimatedPoseZ", 0);	
        }
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        m_photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return m_photonPoseEstimator.update();
    }

	public Pose2d getCentralTagPose() { //gets target closets to center of camera
		var targetResult = m_aprilTagCamera.getLatestResult();
		int targetID = -1;
		if (targetResult.hasTargets()) {
			// Get lists of targets
            List<PhotonTrackedTarget> targets = targetResult.getTargets();
            //make a temp holder var for least Y translation
            Translation2d translationHolder = new Translation2d(0, 0);
            for(PhotonTrackedTarget tag: targets){ //for every target in camera
                //get transformation to target
                Transform3d tagTransform = tag.getBestCameraToTarget(); 
                //get translation to target from transformation
                Translation2d tagTranslation = tagTransform.getTranslation().toTranslation2d();

                //looking for smallest absolute relative to camera Y
                //if abs Y translation of new tag is less then holder tag, it becomes holder tag
                if(Math.abs(tagTranslation.getY()) < Math.abs(translationHolder.getY())){
                    translationHolder = tagTranslation;
                    targetID = tag.getFiducialId(); //set targetID
                }
            }
		}
		return m_aprilTagFieldLayout.getTagPose(targetID).get().toPose2d();

	}

    public Optional<Pose2d> getTagPose(int tagID){
        return Optional.of(m_aprilTagFieldLayout.getTagPose(tagID).get().toPose2d());
    }

    private static AprilTag constructTag(int id, double x, double y, double z, double angle){
        return new AprilTag(id, new Pose3d(x, y, z, new Rotation3d(0, 0, Math.toRadians(angle))));
    }
}
