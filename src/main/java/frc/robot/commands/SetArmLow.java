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
public class SetArmLow extends SequentialCommandGroup {
  /** Creates a new SetArmLow. */
  public SetArmLow(Arm arm) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // arm comands for low grid
      new SetArmAngle(arm, Constants.LOW_GIRD_ARM_ANGLE),
      new SetArmLength(arm, Constants.LOW_GIRD_ARM_LENGTH ),
    );
    
  }
}
