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

  //private final Rotation2d m_rampAngle = Rotation2d.fromDegrees(-10); //useful if we are going to automate going on the ramp
  private final Rotation2d m_balancedError = Rotation2d.fromDegrees(2.5); //error for what counts as balanced
  private final double m_balancedDegrees = 0;
  private final double m_balanceKP = 0.02; //change to control how fast robot drives during balancing
  private final double m_maxMPS = 0.75;
  private final double m_balanceSeconds = 1; //how many seconds the robot has to be balanced before stopping

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

    // Automated way to drive robot onto charge station from ground as well as balance on charge station
    // switch (m_stage) {
    // case GROUND: 
    //   error = m_rampAngle.getDegrees() - currentAngle;
    //   driveMPS = error * CHARGESTATION_BALANCED_KP;
    //
    // set stage to CHARGESTATION once robot is on ramp
    // if (error < CHARGESTATION_BALANCE_ERROR.getDegrees()){
    //   m_stage = m_stages.CHARGESTATION;
    // }
    // case CHARGESTATION: 

    // uses difference in current angle and 0 to get speed, balancing while on charge station 
    error = m_balancedDegrees - currentAngle;
    driveMPS = error * m_balanceKP;

    // cap max speed at 0.75 meters per second
    if (Math.abs(driveMPS) > m_maxMPS) {
      driveMPS = Math.copySign(m_maxMPS, driveMPS);
    }
    
    m_driveTrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(driveMPS, 0.0, 0.0, m_driveTrain.getHeading()));
    
    //if balanced, records time when state of being balanced started, if not balanced, sets time to 0
    if (error <= m_balancedError.getDegrees() && m_stage == m_stages.CHARGESTATION){
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
    if (m_balanced = true && m_timer.get() == m_curTime + m_balanceSeconds){
      return true;
    } else {
      return false;
    }
  }
}

