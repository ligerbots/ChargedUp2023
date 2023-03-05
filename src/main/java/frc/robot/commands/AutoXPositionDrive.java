// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.DriveTrain;

public class AutoXPositionDrive extends CommandBase {

    private static final double GOAL_TOLERANCE = 0.15;

    private DriveTrain m_driveTrain;
    private double m_driveMPS;
    private double m_goalX;
    private double m_direction;

    /** Creates a new ChargeStationDrive. */
    public AutoXPositionDrive(DriveTrain driveTrain, double goalX, double driveMPS) {
        m_driveTrain = driveTrain;
        m_goalX = goalX;
        m_driveMPS = driveMPS;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(driveTrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // driving towards the goal
        if(m_driveTrain.getPose().getX() < m_goalX)
            m_direction = 1.0;
        else
            m_direction = -1.0;
    }   

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double driveSpeed = m_direction * m_driveMPS;
        
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

        //stops when robot is on ramp of charge station 
        return Math.abs(curX - m_goalX) < GOAL_TOLERANCE;
    }
}
