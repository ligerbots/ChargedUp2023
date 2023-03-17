// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
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
    private double m_driveMPS;
    private double m_goalX;
    private double m_directionX;

    private final double Y_PID_CONTROLLER_P = 1.0;
    private PIDController m_yController = new PIDController(Y_PID_CONTROLLER_P, 0, 0);

    private Rotation2d m_rotationHeading;
    private final double ROT_PID_CONTROLLER_P = 1.0;
    private PIDController m_rotController = new PIDController(ROT_PID_CONTROLLER_P, 0, 0);

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
        double driveSpeedX = m_directionX * m_driveMPS;
        
        Pose2d curPose = m_driveTrain.getPose();
        
        double directionY = curPose.getY() < FieldConstants.FIELD_HORIZONTAL_CENTER_LINE_Y ? 1.0 : -1.0;
        double driveSpeedY = directionY * m_yController.calculate(curPose.getY(), FieldConstants.FIELD_HORIZONTAL_CENTER_LINE_Y);

        double directionRot = curPose.getRotation().getDegrees() < m_rotationHeading.getDegrees() ? 1.0 : -1.0;
        double driveSpeedRot = directionRot * m_rotController.calculate(curPose.getRotation().getDegrees(), m_rotationHeading.getDegrees());

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
