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

//command to balance when on charge station
public class ChargeStationDistance extends CommandBase {

    private static final Rotation2d BALANCED_ERROR = Rotation2d.fromDegrees(2.5); //error for what counts as balanced
    private static final double BALANCED_DEGREES = 0;
    private static final double BALANCE_KP = 0.02; //change to control how fast robot drives during balancing
    private static final double MAX_MPS = 1.0;
    private static final double BALANCE_SECONDS = 1; //how many seconds the robot has to be balanced before stopping
    private static final double DRIVE_DISTANCE = 1; //place holder


    private DriveTrain m_driveTrain;
    private final Timer m_timer = new Timer();

    /** Creates a new ChargeStationBalance. */
    public ChargeStationDistance(DriveTrain driveTrain) {
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
        //uses angle of robot to set its speed
        // *current robot has Roll
        Rotation2d currentAngle = m_driveTrain.getRoll();
        Rotation2d error = Rotation2d.fromDegrees(BALANCED_DEGREES - currentAngle.getDegrees());
        double driveMPS = -error.getDegrees() * BALANCE_KP;

        // cap max speed
        if (Math.abs(driveMPS) > MAX_MPS) {
            driveMPS = Math.copySign(MAX_MPS, driveMPS);
        }
        SmartDashboard.putNumber("balanceCommand/driveMPS", driveMPS);
        SmartDashboard.putNumber("balanceCommand/error", error.getDegrees());

        m_driveTrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(driveMPS, 0.0, 0.0, m_driveTrain.getHeading()));
        
        //if not balanced, resets timer
        if (Math.abs(error.getDegrees()) >= BALANCED_ERROR.getDegrees()){
            m_timer.reset();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_driveTrain.stop();
        m_timer.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        //if robot is balanced and it has been for at least one second, robot ends
        // return m_timer.hasElapsed(BALANCE_SECONDS);
        return m_driveTrain.getPose().getTranslation().getX() > DRIVE_DISTANCE;
    }
}

