// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.Constants.Position;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.DriveTrain;

public class DriveAndMoveArm extends SequentialCommandGroup {

    /** Creates a new StowArm. */
    public DriveAndMoveArm(Arm arm, DriveTrain driveTrain, Vision vision, Position targetPosition) {

        addCommands(
                // set arm start position
                new TagPositionDrive(driveTrain, vision, targetPosition),
                new ScoreArm(arm, targetPosition).withTimeout(5)
        );
    }
}
