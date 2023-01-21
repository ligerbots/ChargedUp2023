// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.util.Optional;

import javax.imageio.IIOException;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Vision {

	private final PhotonCamera m_camera = new PhotonCamera("Cam");
	private AprilTagFieldLayout m_aprilTagFieldLayout;
	
	//Forward Camera
	//relative position of the camera on the robot ot the robot center
	private final Transform3d m_robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0)); //Cam mounted facing forward, half a meter forward of center, half a meter up from center.

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
	}

	public void updateOdometry(SwerveDrivePoseEstimator odometry) {

		var targetResult = m_camera.getLatestResult();

		SmartDashboard.putBoolean("hasTargets?", targetResult.hasTargets());
		int targetID = -1;
		if (targetResult.hasTargets()) {
			// Get the current best target.
			PhotonTrackedTarget target = targetResult.getBestTarget();
			targetID = target.getFiducialId();
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
        }

        // m_fieldSim.getObject("Actual Pos").setPose(getPose());
        // m_fieldSim.setRobotPose(m_odometry.getEstimatedPosition());
    }

	public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        m_photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return m_photonPoseEstimator.update();
    }
}
