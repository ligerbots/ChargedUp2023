// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.FieldConstants;
import frc.robot.subsystems.DriveTrain;

public class AutoXPositionDrive extends CommandBase {

    private static final double GOAL_TOLERANCE = 0.15;

    private DriveTrain m_driveTrain;
    private double m_driveMPSX;
    private double m_goalX;
    private double m_directionX;

    private final double Y_MAX_SPEED = 0.2; // we want 0.5 m/s
    private final double Y_PID_CONTROLLER_P = 1.0;

    private Rotation2d m_rotationHeading;
    private final double ROT_MAX_SPEED = 0.2; // we want 10.0 deg/s
    private final double ROT_PID_CONTROLLER_P = 1.0;

    /** Creates a new ChargeStationDrive. */
    public AutoXPositionDrive(DriveTrain driveTrain, double goalX, double driveMPS) {
        m_driveTrain = driveTrain;
        m_goalX = goalX;
        m_driveMPSX = driveMPS;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(driveTrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // driving towards the goal
        if(m_driveTrain.getPose().getX() < m_goalX)
            m_directionX = 1.0;
        else
            m_directionX = -1.0;

        if(DriverStation.getAlliance() == Alliance.Red)
            m_rotationHeading = Rotation2d.fromDegrees(0);
        else
            m_rotationHeading = Rotation2d.fromDegrees(180);
    }   

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double driveSpeedX = m_directionX * m_driveMPSX;
        
        Pose2d curPose = m_driveTrain.getPose();

        // this includes the Y direction for the robot to move 
        double driveSpeedY = (FieldConstants.FIELD_HORIZONTAL_CENTER_LINE_Y - curPose.getY()) * Y_PID_CONTROLLER_P;
        driveSpeedY = MathUtil.clamp(driveSpeedX, -Y_MAX_SPEED, Y_MAX_SPEED);

        // keep the heading (-180, 180]
        double curHeading = MathUtil.inputModulus(curPose.getRotation().getDegrees(), -180, 180);
        // similarly, the direction of turning is handled
        double driveSpeedRot = (m_rotationHeading.getDegrees() - curHeading) * ROT_PID_CONTROLLER_P;
        driveSpeedRot = MathUtil.clamp(driveSpeedRot, -ROT_MAX_SPEED, ROT_MAX_SPEED);

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
