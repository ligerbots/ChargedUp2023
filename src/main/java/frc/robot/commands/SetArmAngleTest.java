// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Arm;

public class SetArmAngleTest extends InstantCommand {
    Command m_command;

    Arm m_arm;

    public SetArmAngleTest(Arm arm) {
        m_arm = arm;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_command = new SetArmAngle(m_arm,
                Units.degreesToRadians(SmartDashboard.getNumber("Testing/SetArmAngleTest", 0.0)));

        CommandScheduler.getInstance().schedule(m_command);
    }
}
