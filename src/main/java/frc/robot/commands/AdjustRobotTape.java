// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Vision;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

//this command adjusts robot to the best reflective tape node target
public class AdjustRobotTape extends InstantCommand {
    private final Vision m_vision;
    private final DriveTrain m_driveTrain;

    public AdjustRobotTape(Vision vision, DriveTrain driveTrain) {
        // Use addRequirements() here to declare subsystem dependencies.
        m_vision = vision;
        m_driveTrain = driveTrain;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() { //should I still check if hasTargets?
        double forwardSpeed; // how much speed robot needs to go forward to target
		double rotationSpeed; // how much speed robot needs to rotate to target

		double range = m_vision.getDistanceFromTarget(true); //range for cone/tape

        forwardSpeed = -m_driveTrain.getXController().calculate(range, Constants.GOAL_RANGE_METERS);
        // use x controller to move forward?

        // Also calculate angular power
        // -1.0 required to ensure positive PID controller effort _increases_ yaw
        rotationSpeed = -m_driveTrain.getThetaController().calculate(m_vision.getYawFromTarget(), 0); //how much yaw do we want?
        // use theta to rotate

        m_driveTrain.drive(new ChassisSpeeds(forwardSpeed, 0.0, rotationSpeed)); // make it move forward and rotate
		// is y affected by adjusting?

    }

}
