// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

// Do nothing but wait for the arm to be above an angle. Used in Command Groups.
// Make sure that the threshold is enough below the final target so that it actually finishes.
public class WaitForArmAngle extends CommandBase {
    private double m_angle;
    private Arm m_arm;
    /** Creates a new WaitForArmAngle. */
    public WaitForArmAngle(Arm arm, double angle) {
        // Use addRequirements() here to declare subsystem dependencies.
        m_angle = angle;
        m_arm = arm;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_arm.getArmAngle() > m_angle;
    }
}
