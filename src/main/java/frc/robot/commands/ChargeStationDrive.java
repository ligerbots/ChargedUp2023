// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class ChargeStationDrive extends CommandBase {
  private static final Rotation2d RAMP_ANGLE = Rotation2d.fromDegrees(-10); 
  private static final double DRIVE_MPS = 0.75;

  private DriveTrain m_driveTrain;
  private Rotation2d currentAngle;

  /** Creates a new ChargeStationDrive. */
  public ChargeStationDrive() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentAngle = m_driveTrain.getPitch();

    //robot drives at set speed in mps
    m_driveTrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(DRIVE_MPS, 0.0, 0.0, m_driveTrain.getHeading()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //stops when robot is on ramp of charge station 
    return (currentAngle.getDegrees() <= RAMP_ANGLE.getDegrees());
  }
}
