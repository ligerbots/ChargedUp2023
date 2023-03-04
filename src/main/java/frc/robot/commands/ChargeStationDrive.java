// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class ChargeStationDrive extends CommandBase {
    private static final double DRIVE_MPS = 0.75;
    private static final double CHARGE_STATION_MIDDLE_BLUE = 3.9;
    private static final double CHARGE_STATION_MIDDLE_RED = 12.6;

    private static final double CHARGE_STATION_TOLERANCE = 0.25;

    private DriveTrain m_driveTrain;

    /** Creates a new ChargeStationDrive. */
    public ChargeStationDrive(DriveTrain driveTrain) {
        m_driveTrain = driveTrain;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(driveTrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double driveSpeed = DRIVE_MPS;
        if(DriverStation.getAlliance() == Alliance.Red)
            driveSpeed = -driveSpeed;
        
        //robot drives at set speed in mps
        m_driveTrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(driveSpeed, 0.0, 0.0, m_driveTrain.getHeading()));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_driveTrain.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {      
        double curX = m_driveTrain.getPose().getX();

        double goalX;
        if(DriverStation.getAlliance() == Alliance.Red)
            goalX = CHARGE_STATION_MIDDLE_RED;
        else
            goalX = CHARGE_STATION_MIDDLE_BLUE;

        //stops when robot is on ramp of charge station 
        return Math.abs(curX - goalX) < CHARGE_STATION_TOLERANCE;
    }
}
