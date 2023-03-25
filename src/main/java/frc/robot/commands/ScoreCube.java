// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.Position;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Vision;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreCube extends ParallelCommandGroup {
	static final double SAFE_MINIMUM_ANGLE = Math.toRadians(-15.0);

	/** Creates a new ScoreCube. */
	public ScoreCube(Arm arm, DriveTrain driveTrain, Vision vision, Position targetPosition, JoystickButton overrideButton) {
		// Add your commands in the addCommands() call, e.g.
		// addCommands(new FooCommand(), new BarCommand());
		addCommands(
			new ScoreArm(arm, driveTrain, targetPosition, overrideButton).withTimeout(5),
			// drive to the correct position
			new WaitForArmAngle(arm, SAFE_MINIMUM_ANGLE).withTimeout(5.0).andThen(new TagPositionDrive(driveTrain, vision, targetPosition))
		);
	}
}
