// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.FieldConstants;
import frc.robot.subsystems.DriveTrain;

public class ChargeStationDrive extends CommandBase {
    private static final double CHARGE_STATION_DRIVE_MPS = 3.0;
    /** Creates a new ChargeStationDrive. */
    Command m_command;
    DriveTrain m_driveTrain;
    public ChargeStationDrive(DriveTrain driveTrain) {
        // Use addRequirements() here to declare subsystem dependencies.
        m_driveTrain = driveTrain;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        double goalX;
        if(DriverStation.getAlliance() == Alliance.Red)
            goalX = FieldConstants.CHARGE_STATION_MIDDLE_X_RED;
        else 
            goalX = FieldConstants.CHARGE_STATION_MIDDLE_X_BLUE;
        m_command = new AutoXPositionDrive(m_driveTrain, goalX, CHARGE_STATION_DRIVE_MPS);
        CommandScheduler.getInstance().schedule(m_command);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            m_command.cancel();
        }
        m_command = null;
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_command == null || !m_command.isScheduled();
    }
}
