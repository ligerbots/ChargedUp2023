// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import frc.robot.FieldConstants;
import frc.robot.subsystems.DriveTrain;

public class AutoXPositionDrive extends CommandBase {

    private static final double GOAL_TOLERANCE = 0.15;

    private DriveTrain m_driveTrain;
    private double m_driveMPSX;
    private double m_goalXBlue;
    private double m_directionX;
    private double m_goalX;

    private final double Y_MAX_SPEED = 0.25; // we want 0.5 m/s
    private final double Y_PID_CONTROLLER_P = 1.0;

    // Work in degrees
    private double m_targetHeadingDegrees;
    private final double ROT_MAX_SPEED = 0.2; // we want 10.0 deg/s
    private final double ROT_PID_CONTROLLER_P = 1.0;

    /** Creates a new ChargeStationDrive. */
    public AutoXPositionDrive(DriveTrain driveTrain, double goalXBlue, double driveMPS) {
        m_driveTrain = driveTrain;
        m_goalXBlue = goalXBlue;
        m_driveMPSX = driveMPS;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(driveTrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_goalX = FieldConstants.flipX(m_goalXBlue);

        // driving towards the goal
        if (m_driveTrain.getPose().getX() < m_goalX)
            m_directionX = 1.0;
        else
            m_directionX = -1.0;

        if (DriverStation.getAlliance() == Alliance.Red)
            m_targetHeadingDegrees = 0;
        else
            m_targetHeadingDegrees = 180;
    }   

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double driveSpeedX = m_directionX * m_driveMPSX;
        
        Pose2d curPose = m_driveTrain.getPose();

        // this includes the Y direction for the robot to move 
        double yError = FieldConstants.CHARGE_STATION_CENTER_Y - curPose.getY();
        double driveSpeedY = MathUtil.clamp(yError * Y_PID_CONTROLLER_P, -Y_MAX_SPEED, Y_MAX_SPEED);

        // The angle error is the difference between the target and actual heading
        // However, the result should be between -90 -> 90 degrees to choose the smallest possible turn.
        double angError = MathUtil.inputModulus(m_targetHeadingDegrees - curPose.getRotation().getDegrees(), -90.0, 90.0);
        double driveSpeedRot = MathUtil.clamp(angError * ROT_PID_CONTROLLER_P, -ROT_MAX_SPEED, ROT_MAX_SPEED);

        //robot drives at set speed in mps
        m_driveTrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(driveSpeedX, driveSpeedY, driveSpeedRot, m_driveTrain.getHeading()));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // System.out.println("** End AutoXPostionDrive INTERRUPTED = " + interrupted);
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
