// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
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
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Vision {
	private final double CUSTOM_FIELD_LENGTH = 10;
	private final double CUSTOM_FIELD_WIDTH = 10;

	final AprilTag tag18 = new AprilTag(
			18,
			new Pose3d(
					new Pose2d(
							CUSTOM_FIELD_LENGTH,
							CUSTOM_FIELD_WIDTH / 2.0,
							Rotation2d.fromDegrees(180))));

	final AprilTag tag01 = new AprilTag(
			01,
			new Pose3d(new Pose2d(0.0, CUSTOM_FIELD_WIDTH / 2.0, Rotation2d.fromDegrees(0.0)))); 
	
	private AprilTagFieldLayout m_customFieldLayout;

	private final PhotonCamera m_camera = new PhotonCamera("ApriltagCamera");
	private AprilTagFieldLayout m_aprilTagFieldLayout;
	
	//Forward Camera
	//relative position of the camera on the robot ot the robot center
	private final Transform3d m_robotToCam = new Transform3d(new Translation3d(Constants.CAMERA_X_OFFSET, Constants.CAMERA_Y_OFFSET, Constants.CAMERA_Z_OFFSET), new Rotation3d(Constants.CAMERA_ROLL_OFFSET,Constants.CAMERA_PITCH_OFFSET,Constants.CAMERA_YAW_OFFSET)); //Cam mounted facing forward, half a meter forward of center, half a meter up from center.

	PhotonPoseEstimator m_photonPoseEstimator;

	public Vision() {
		try{
			m_aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
		} catch(IOException e){
			System.out.println("Unable to load AprilTag layout" + e.getMessage());
			m_aprilTagFieldLayout = null;
		}

		m_photonPoseEstimator = new PhotonPoseEstimator(m_aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, m_camera, m_robotToCam);
		// set the driver mode to false
		m_camera.setDriverMode(false);


		m_customFieldLayout = new AprilTagFieldLayout(new ArrayList<AprilTag>() {
			{
				add(tag01);
				add(tag18);
			}
		}, CUSTOM_FIELD_LENGTH, CUSTOM_FIELD_WIDTH);
	}

	public void updateOdometry(SwerveDrivePoseEstimator odometry) {

		var targetResult = m_camera.getLatestResult();

		SmartDashboard.putBoolean("hasTargets?", targetResult.hasTargets());
		int targetID = -1;
		if (targetResult.hasTargets()) {
			// Get the current best target.
			PhotonTrackedTarget target = targetResult.getBestTarget();
			targetID = target.getFiducialId();
			SmartDashboard.putNumber("vision/tagOffsetX", target.getBestCameraToTarget().getX());
			SmartDashboard.putNumber("vision/tagOffsetY", target.getBestCameraToTarget().getY());
			SmartDashboard.putNumber("vision/tagOffsetAngle", target.getBestCameraToTarget().getRotation().getAngle());

		}
		SmartDashboard.putNumber("targetID", targetID);

        // Also apply vision measurements. We use 0.3 seconds in the past as an example
        // -- on
        // a real robot, this must be calculated based either on latency or timestamps.
        Optional<EstimatedRobotPose> result =
                getEstimatedGlobalPose(odometry.getEstimatedPosition());
        if (result.isPresent()) {
            EstimatedRobotPose camPose = result.get();
			var estimatedPose = camPose.estimatedPose;
            odometry.addVisionMeasurement(estimatedPose.toPose2d(), camPose.timestampSeconds);
			SmartDashboard.putNumber("vision/estimatedPoseX", estimatedPose.getX());
			SmartDashboard.putNumber("vision/estimatedPoseY", estimatedPose.getY());
			SmartDashboard.putNumber("vision/estimatedPoseZ", estimatedPose.getRotation().getAngle());

            // m_fieldSim.getObject("Cam Est Pos").setPose(camPose.estimatedPose.toPose2d());

        } else {
            // move it way off the screen to make it disappear
            // m_fieldSim.getObject("Cam Est Pos").setPose(new Pose2d(-100, -100, new Rotation2d()));
			SmartDashboard.putNumber("vision/estimatedPoseX", 0);
			SmartDashboard.putNumber("vision/estimatedPoseY", 0);
			SmartDashboard.putNumber("vision/estimatedPoseZ", 0);	
        }

        // m_fieldSim.getObject("Actual Pos").setPose(getPose());
        // m_fieldSim.setRobotPose(m_odometry.getEstimatedPosition());
    }

	public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        m_photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return m_photonPoseEstimator.update();
    }
}
