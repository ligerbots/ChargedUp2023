// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.Position;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.DriveTrain;

public class MoveArmAndDrive extends SequentialCommandGroup {

    /** Creates a new StowArm. */
    public MoveArmAndDrive(Arm arm, DriveTrain driveTrain, Vision vision, Position targetPosition) {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(
                // set arm start position
                new ScoreArm(arm, targetPosition).withTimeout(5),
                // drive to the correct position
                new TagPositionDrive(driveTrain, vision, targetPosition)
        );
    }
}
