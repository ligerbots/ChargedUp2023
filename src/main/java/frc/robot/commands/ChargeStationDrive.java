// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.FieldConstants;
import frc.robot.subsystems.DriveTrain;

public class ChargeStationDrive extends CommandBase {
    Command m_command;
    DriveTrain m_driveTrain;

    /** Creates a new ChargeStationDrive. */
    public ChargeStationDrive(DriveTrain driveTrain) {
        m_driveTrain = driveTrain;

        addRequirements(m_driveTrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_command = new AutoXPositionDrive(m_driveTrain, FieldConstants.CHARGE_STATION_MIDDLE_X_BLUE, DriveTrain.CHARGE_STATION_DRIVE_MPS);
        m_command.initialize();
    }

    
    @Override
    public void execute() {
        if (m_command != null)
            m_command.execute();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // if interrupted, stop the follow trajectory
        // System.out.println("TagPositionDrive end interrupted = " + interrupted);
        if (m_command != null)
            m_command.end(interrupted);
        m_command = null;
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // if the FollowTrajectory commad is null or not scheduled, end
        return m_command == null || m_command.isFinished();
    }
}
