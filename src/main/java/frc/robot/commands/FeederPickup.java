// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.swing.GroupLayout.ParallelGroup;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.Position;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.DriveTrain;

public class FeederPickup extends ParallelCommandGroup {

    /** Creates a new StowArm. */
    public FeederPickup(Arm arm, DriveTrain driveTrain, Vision vision, Claw claw, Position targetPosition, JoystickButton overrideButton) {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(
                // set arm start position
                new InstantCommand(claw::startIntake),
                new ScoreArm(arm, driveTrain, targetPosition, overrideButton).withTimeout(5),
                new DriveToFeeder(driveTrain, vision, targetPosition)
        );
    }
}
