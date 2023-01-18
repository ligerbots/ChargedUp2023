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

//this command uses reflective tape to automatically adjust the robot so it can score a cone on highest level
public class AdjustRobotCone extends InstantCommand {
    private final DriveTrain m_driveTrain;
    private final Vision m_vision;

    public AdjustRobotCone(DriveTrain driveTrain, Vision vision) {
    // Use addRequirements() here to declare subsystem dependencies.
        m_driveTrain = driveTrain;
        m_vision = vision;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        var result = m_vision.getCamera().getLatestResult(); //camera's latest result

		if (result.hasTargets()) {
			// First calculate range
			double range =
					PhotonUtils.calculateDistanceToTargetMeters(
							Constants.CAMERA_HEIGHT_METERS,
							Constants.TARGET_HEIGHT_METERS_CUBE,
							Constants.CAMERA_PITCH_RADIANS,
							Units.degreesToRadians(result.getBestTarget().getPitch()));

			
		}
		m_driveTrain.drive(new ChassisSpeeds(0.0, 0.0, 0.0)); //make it move forward and rotate
        //is y affected by adjusting?
  }

  
}