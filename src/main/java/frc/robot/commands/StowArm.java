// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class StowArm extends SequentialCommandGroup {
  /** Creates a new StowArm. */
  public StowArm(Arm arm) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      //set arm star position
      new SetArmAngle(arm, Constants.STARTING_ARM_ANGLE),
      new SetArmLength(arm, Constants.STARTING_ARM_LENGTH),
      // arm comands for low grid
      new SetArmAngle(arm, Constants.LOW_GIRD_ARM_ANGLE),
      new SetArmLength(arm, Constants.LOW_GIRD_ARM_LENGTH ),
      // arm comand for middle gird
      new SetArmAngle(arm, Constants.MIDDLE_GRID_ARM_ANGLE),
      new SetArmLength(arm, Constants.MIDDLE_GRID_ARM_LENGTH),
      // arm comand for high grid
      new SetArmAngle(arm, Constants.HIGH_GRID_ARM_ANGLE),
      new SetArmLength(arm, Constants.HIGH_GRID_ARM_LENGTH),
    )
  }
}
