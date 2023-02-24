// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Reacher;

public class SetArmLength extends CommandBase {

    private static final double REACHER_OFFSET_TOLERANCE_METERS = Units.inchesToMeters(0.5);

    /** Creates a new SetArmLength. */
    Arm m_arm;
    double m_length;

    public SetArmLength(Arm arm, double length) {
        // Use addRequirements() here to declare subsystem dependencies.
        m_arm = arm;
        m_length = Reacher.limitReacherLength(length);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_arm.setArmLength(m_length);
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
        double curLength = m_arm.getArmLength();
        return Math.abs(curLength - m_length) < REACHER_OFFSET_TOLERANCE_METERS;
    }
}
