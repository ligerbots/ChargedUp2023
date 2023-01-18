// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.PhotonUtils;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Vision;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

//this command uses Apriltags to automatically adjust the robot so it can score a cube
// a lot of code taken from https://docs.photonvision.org/en/latest/docs/examples/aimandrange.html
public class AdjustRobotCube extends InstantCommand {
    private final DriveTrain m_driveTrain;
    private final Vision m_vision;

    public AdjustRobotCube(DriveTrain driveTrain, Vision vision) {
    // Use addRequirements() here to declare subsystem dependencies.
        m_driveTrain = driveTrain;
        m_vision = vision;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        double forwardSpeed; //how much speed robot needs to go forward to target
        double rotationSpeed; //how much speed robot needs to rotate to target

        var result = m_vision.getCamera().getLatestResult(); //camera's latest result

		if (result.hasTargets()) {
			// First calculate range
			double range =
					PhotonUtils.calculateDistanceToTargetMeters(
							Constants.CAMERA_HEIGHT_METERS,
							Constants.TARGET_HEIGHT_METERS_CUBE,
							Constants.CAMERA_PITCH_RADIANS,
							Units.degreesToRadians(result.getBestTarget().getPitch()));

			// Use this range as the measurement we give to the PID controller.
			// -1.0 required to ensure positive PID controller effort _increases_ range
			forwardSpeed = -m_driveTrain.getXController().calculate(range, Constants.GOAL_RANGE_METERS);
            //use x controller to move forward?

			// Also calculate angular power
			// -1.0 required to ensure positive PID controller effort _increases_ yaw
			rotationSpeed = -m_driveTrain.getThetaController().calculate(result.getBestTarget().getYaw(), 0);
            //use theta to rotate
        } else {
			// If we have no targets, stay still.
			forwardSpeed = 0;
			rotationSpeed = 0;
		}
		m_driveTrain.drive(new ChassisSpeeds(forwardSpeed, 0.0, rotationSpeed)); //make it move forward and rotate
        //is y affected by adjusting?
  }

  
}