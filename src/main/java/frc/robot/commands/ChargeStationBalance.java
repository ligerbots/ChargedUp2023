// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;

public class ChargeStationBalance extends CommandBase {

    private static final double BALANCED_ERROR_DEGREES = 2.5; // max error for what counts as balanced
    private static final double BALANCED_DEGREES = 0;
    private static final double BALANCE_KP = 0.02; // change to control how fast robot drives during balancing
    private static final double MAX_MPS = 1.0;
    private static final double BALANCE_SECONDS = 1; // how many seconds the robot has to be balanced before stopping

    private DriveTrain m_driveTrain;
    private final Timer m_timer = new Timer();

    /** Creates a new ChargeStationBalance. */
    public ChargeStationBalance(DriveTrain driveTrain) {
        m_driveTrain = driveTrain;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_driveTrain);

        SmartDashboard.putNumber("balanceCommand/driveMPS", 0.0);
        SmartDashboard.putNumber("balanceCommand/error", 0.0);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_timer.reset();
        m_timer.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // uses angle of robot to set its speed
        double errorDegrees = m_driveTrain.getTiltDegrees() - BALANCED_DEGREES;
        double driveMPS = errorDegrees * BALANCE_KP;
    
        // cap max speed
        if (Math.abs(driveMPS) > MAX_MPS) {
            driveMPS = Math.copySign(MAX_MPS, driveMPS);
        }
        SmartDashboard.putNumber("balanceCommand/driveMPS", driveMPS);
        SmartDashboard.putNumber("balanceCommand/error", errorDegrees);

        Rotation2d driveAngle = m_driveTrain.getTiltDirection().plus(Rotation2d.fromDegrees(180));
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
            driveMPS * driveAngle.getCos(),
            driveMPS * driveAngle.getSin(),
            0
        );

        m_driveTrain.drive(chassisSpeeds);
        
        // if not balanced, resets timer
        if (Math.abs(errorDegrees) >= BALANCED_ERROR_DEGREES){
            m_timer.reset();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_driveTrain.lockWheels();
        m_timer.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        //if robot is balanced and it has been for at least one second, robot ends
        return m_timer.hasElapsed(BALANCE_SECONDS);
    }
}

