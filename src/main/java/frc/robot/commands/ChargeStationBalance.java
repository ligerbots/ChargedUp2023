// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.math.geometry.Rotation2d;

public class ChargeStationBalance extends CommandBase {

  private final Rotation2d BALANCED_ERROR = Rotation2d.fromDegrees(2.5); //error for what counts as balanced
  private final double BALANCED_DEGREES = 0;
  private final double BALANCE_KP = 0.02; //change to control how fast robot drives during balancing
  private final double MAX_MPS = 0.75;
  private final double BALANCE_SECONDS = 1; //how many seconds the robot has to be balanced before stopping

  private DriveTrain m_driveTrain;
  private Boolean m_balanced;
  private final Timer m_timer = new Timer();
  private double m_curTime;

  /** Creates a new ChargeStationBalance. */
  public ChargeStationBalance() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();

    m_balanced = false;
    m_curTime = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //uses angle of robot to set its speed
    Rotation2d currentAngle = m_driveTrain.getPitch();
    Rotation2d error = Rotation2d.fromDegrees(BALANCED_DEGREES - currentAngle.getDegrees());
    double driveMPS = error.getDegrees() * BALANCE_KP;

    // cap max speed at 0.75 meters per second
    if (Math.abs(driveMPS) > MAX_MPS) {
      driveMPS = Math.copySign(MAX_MPS, driveMPS);
    }
    
    m_driveTrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(driveMPS, 0.0, 0.0, m_driveTrain.getHeading()));
    
    //if balanced, records time when state of being balanced started, if not balanced, sets curtime to 0
    if (error.getDegrees() <= BALANCED_ERROR.getDegrees()){
      m_balanced = true;
      if (m_curTime == 0){
      m_curTime = m_timer.get();
      }
    } else {
      m_balanced = false;
      m_curTime = 0;
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
    return (m_balanced && m_timer.get() >= m_curTime + BALANCE_SECONDS);
  }
}

