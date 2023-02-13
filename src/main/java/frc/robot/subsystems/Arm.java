// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
	private Shoulder m_shoulder;
	private Reacher m_reacher;

	public Arm() {
		// Construct the shoulder and reacher trapezoid subsystems
		m_shoulder = new Shoulder();
		m_reacher = new Reacher();
	}

	public void periodic() {
	}

	public void setReacherExtent(TrapezoidProfile.State extent) {
		m_reacher.setSetPoint(extent);
	}

	// rotates the arms to a certain angle
	public void setShoulderAngle(double degree) {
		m_shoulder.setGoal(degree);
	}

	// returns the currrent height of the Reacher
	public double getReacherExtent() {
		return m_reacher.getExtent();
	}

	// returns the current angle of the arm
	public double getArmAngle() {
		return m_shoulder.getEncoder().getIntegratedSensorAbsolutePosition();
	}

	public double getSimArmAngle() {
		return m_shoulder.getEncoder().getIntegratedSensorAbsolutePosition();
	}
}
