// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

//command to drive a set distance along the x axis
public class DriveDistance extends CommandBase {
    private static final double MAX_MPS = 5;

    private Encoder encoder = new Encoder(0, 1); // TODO find actual values to configure encoder
    private DriveTrain m_driveTrain;
    private double m_driveMPS;
    private double m_distanceGoal;

    /** Creates a new DriveDistance. */
    public DriveDistance(DriveTrain driveTrain, double driveMPS, double distanceGoal) {
        m_driveTrain = driveTrain;
        m_driveMPS = driveMPS;
        m_distanceGoal = distanceGoal;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_driveTrain);

        SmartDashboard.putNumber("driveDistance/encoderDistance", 0);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        //encoder.setDistancePerPulse(); // TODO find actual values to configure encoder
        encoder.reset();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        SmartDashboard.putNumber("driveDistance/encoderDistance", encoder.getDistance());

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
        encoder.reset();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return encoder.getDistance() >= Math.abs(m_distanceGoal);
    }
}