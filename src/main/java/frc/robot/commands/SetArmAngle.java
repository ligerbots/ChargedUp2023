// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Shoulder;

public class SetArmAngle extends CommandBase {


    private static final double SHOULDER_ANGLE_TOLERANCE_RADIAN = Math.toRadians(1.0);

    /** Creates a new SetArmAngle. */
    Arm m_arm;
    double m_angle;

    public SetArmAngle(Arm arm, double angle) {
        // Use addRequirements() here to declare subsystem dependencies.
        m_arm = arm;
        m_angle = angle;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_arm.setArmAngle(m_angle);
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
        double curAngle = m_arm.getArmAngle();
        if(curAngle < Shoulder.SHOULDER_MIN_ANGLE || curAngle > Shoulder.SHOULDER_MAX_ANGLE){
            m_arm.resetShoulderPos();
            return true;
        }
        return Math.abs(curAngle - m_angle) < SHOULDER_ANGLE_TOLERANCE_RADIAN;
    }
}
