// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class ChargeStationBalance extends CommandBase {

  private DriveTrain m_driveTrain;
  private enum m_stages {
    GROUND,
    CHARGESTATION
  }
  private m_stages m_stage = m_stages.GROUND;
  private Boolean m_balanced = false;
  private final Timer m_timer = new Timer();
  private double m_curTime = 0;

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
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double error = 0;
    double driveMPS = 0;
    double currentAngle = m_driveTrain.getPitch().getDegrees();

    // uses difference in current angle and 0 to get speed
    // while robot is on the ground, drives until gets onto ramp
    // while robot is on ramp, drives until balanced
    switch (m_stage) {
    case GROUND: 
      error = Constants.CHARGESTATION_RAMP_ANGLE_DEGREES.getDegrees() - currentAngle;
      driveMPS = error * Constants.CHARGESTATION_BALANCED_KP;
      
      //set stage to CHARGESTATION once robot is on ramp
      if (error < Constants.CHARGESTATION_BALANCE_ERROR.getDegrees()){
        m_stage = m_stages.CHARGESTATION;
      }

    case CHARGESTATION: 
      error = Constants.CHARGESTATION_BALANCED_GOAL_DEGREES - currentAngle;
      driveMPS = -(error * Constants.CHARGESTATION_BALANCED_KP);
    }

    // cap max speed at 0.75 meters per second
    if (Math.abs(driveMPS) > Constants.CHARGESTATION_MAX_METERSPERSECOND) {
      driveMPS = Math.copySign(Constants.CHARGESTATION_MAX_METERSPERSECOND, driveMPS);
    }

    m_driveTrain.drive(new ChassisSpeeds(driveMPS, 0.0, 0.0));
    
    //if balanced, records time when state of being balanced started, if not balanced, sets time to 0
    if (error <= Constants.CHARGESTATION_BALANCE_ERROR.getDegrees() && m_stage == m_stages.CHARGESTATION){
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
    //if robot is balanced and it has been for one second, robot ends
    if (m_balanced = true && m_timer.get() == m_curTime + Constants.CHARGESTATION_BALANCE_SECONDS){
      return true;
    } else {
      return false;
    }
  }
}

