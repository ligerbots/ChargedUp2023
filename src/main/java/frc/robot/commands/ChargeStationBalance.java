// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import java.util.ArrayList;

public class ChargeStationBalance extends CommandBase {

  private DriveTrain m_driveTrain;

  private double error;
  private ArrayList<Boolean> balanceCheck = new ArrayList<Boolean>();
  private double driveMPS;
  private double currentAngle;

  /** Creates a new ChargeStationBalance. */
  public ChargeStationBalance() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.currentAngle = m_driveTrain.getPitch();

    // uses difference in current angle and 0 to get speed
    error = Constants.CHARGESTATION_BALANCED_GOAL_DEGREES - currentAngle;
    driveMPS = -(error * Constants.CHARGESTATION_BALANCED_KP);

    // cap max speed at 0.75 meters per second
     if (Math.abs(driveMPS) > Constants.CHARGESTATION_MAX_METERSPERSECOND) {
      driveMPS = Math.copySign(Constants.CHARGESTATION_MAX_METERSPERSECOND, driveMPS);
    }

    m_driveTrain.drive(new ChassisSpeeds(driveMPS, 0.0, 0.0));
    
    if (error < Constants.CHARGESTATION_BALANCE_ERROR){
      balanceCheck.add(true);
    } else {
      balanceCheck.add(false);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    int a = 0;
    for (int i = balanceCheck.size() - 50; i < balanceCheck.size(); i++) {
      if (balanceCheck.get(i-1) == true) {
        a++;
      }
    }

    if (a>=50){
      return true;
    } else {
      return false;
    }

  }
}

