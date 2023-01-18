// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Vision extends SubsystemBase {

	private final PhotonCamera m_camera = new PhotonCamera("Cam");
	private final DriveTrain m_driveTrain;

	/** Creates a new Vision. */
	public Vision(DriveTrain driveTrain) {
		this.m_driveTrain = driveTrain;
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run

		// System.out.println(m_camera.getDriverMode());

		var result = m_camera.getLatestResult();

		SmartDashboard.putBoolean("hasTargets?", result.hasTargets());
		SmartDashboard.putBoolean("drivemode", m_camera.getDriverMode());
		if (result.hasTargets()) {
			// Get the current best target.
			PhotonTrackedTarget target = result.getBestTarget();
			int targetID = target.getFiducialId();
			SmartDashboard.putNumber("targetID", targetID);

		} else {
			SmartDashboard.putNumber("targetID", -1);
			SmartDashboard.putNumber("PPL", m_camera.getPipelineIndex());
		}
	}

	// adjusts robot automatically so it is in position to pick up cone
	public void AdjustRobotCone() {

	}

	// adjusts robot automatically so it is in position to pick up cube
	public void AdjustRobotCube() {
		double forwardSpeed; // how much speed robot needs to go forward to target
		double rotationSpeed; // how much speed robot needs to rotate to target

		var result = m_camera.getLatestResult(); // camera's latest result

		if (result.hasTargets()) {
			// First calculate range
			double range = PhotonUtils.calculateDistanceToTargetMeters(
					Constants.CAMERA_HEIGHT_METERS,
					Constants.TARGET_HEIGHT_METERS_CUBE,
					Constants.CAMERA_PITCH_RADIANS,
					Units.degreesToRadians(result.getBestTarget().getPitch()));

			// Use this range as the measurement we give to the PID controller.
			// -1.0 required to ensure positive PID controller effort _increases_ range
			forwardSpeed = -m_driveTrain.getXController().calculate(range, Constants.GOAL_RANGE_METERS);
			// use x controller to move forward?

			// Also calculate angular power
			// -1.0 required to ensure positive PID controller effort _increases_ yaw
			rotationSpeed = -m_driveTrain.getThetaController().calculate(result.getBestTarget().getYaw(), 0);
			// use theta to rotate
		} else {
			// If we have no targets, stay still.
			forwardSpeed = 0;
			rotationSpeed = 0;
		}
		m_driveTrain.drive(new ChassisSpeeds(forwardSpeed, 0.0, rotationSpeed)); // make it move forward and rotate
		// is y affected by adjusting?
	}
}
