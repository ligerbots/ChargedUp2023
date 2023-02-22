// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// drives until robot is near certain roll angle 
public class AngleDrive extends CommandBase {
    private static final double MAX_MPS = 5;
    
    private DriveTrain m_driveTrain;
    private double m_driveMPS;
    private Rotation2d m_currentAngle;
    private Rotation2d m_angleGoal;

    /** Creates a new ChargeStationDrive. */
    public AngleDrive(DriveTrain driveTrain, double driveMPS, Rotation2d angleGoal) {
        m_driveTrain = driveTrain;
        m_angleGoal = angleGoal;
        m_driveMPS = driveMPS;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_driveTrain);
        SmartDashboard.putNumber("angleDrive/angle", 0.0);
        SmartDashboard.putNumber("angleDrive/driveMPS", 0.0);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_currentAngle = m_driveTrain.getRoll();    

        // test statements
        SmartDashboard.putNumber("angleDrive/angle", m_currentAngle.getDegrees());
        SmartDashboard.putNumber("angleDrive/driveMPS", m_driveMPS);

        // cap max speed
        if (Math.abs(m_driveMPS) >= MAX_MPS){
            m_driveMPS = Math.copySign(MAX_MPS, m_driveMPS);
        }

        // robot drives at set speed in mps
        m_driveTrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(m_driveMPS, 0.0, 0.0, m_driveTrain.getHeading()));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_driveTrain.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (m_angleGoal.getDegrees() == 0.0){
            // stops when robot is close to 0 degrees 
            return (Math.abs(m_currentAngle.getDegrees()) <= 0.1);
        } else if (m_angleGoal.getDegrees() < 0){
            // stops when robot is on community side of charge station
            return (m_currentAngle.getDegrees() <= m_angleGoal.getDegrees());
        } else {
            // stops when robot is on side of charge station opposite of community area
            return (m_currentAngle.getDegrees() >= m_angleGoal.getDegrees());
        }
    }
}
