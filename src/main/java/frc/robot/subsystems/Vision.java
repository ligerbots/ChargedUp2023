// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Vision extends SubsystemBase {

	private final PhotonCamera m_camera = new PhotonCamera("Cam");


	
	/** Creates a new Vision. */
	public Vision() {}

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

	public PhotonCamera getCamera(){
		return m_camera;
	}
}
