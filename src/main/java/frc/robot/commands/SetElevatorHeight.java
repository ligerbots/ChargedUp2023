// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetElevatorHeight extends CommandBase {
  Arm m_arm;
  double m_height;

  double m_maxVel;
  double m_maxAcc;

  Command m_command;

  public SetElevatorHeight(Arm arm, double height) {
    this(arm, height, Constants.ELEVATOR_MAX_VEL_METER_PER_SEC_DESCEND,
        Constants.ELEVATOR_MAX_ACC_METER_PER_SEC_SQ_DESCEND);
  }

  public SetElevatorHeight(Arm arm, double height, final double MAX_VEL_METER_PER_SEC,
      final double MAX_ACC_METER_PER_SEC) {
    m_maxAcc = MAX_ACC_METER_PER_SEC;
    m_maxVel = MAX_VEL_METER_PER_SEC;
    m_arm = arm;
    m_height = height;
  }

  @Override
  public void initialize() {
    m_command = new TrapezoidProfileCommand(new TrapezoidProfile(
        // Limit the max acceleration and velocity
        new TrapezoidProfile.Constraints(
            m_maxVel,
            m_maxAcc),
        // End at desired position in meters; implicitly starts at 0
        new TrapezoidProfile.State(m_height, 0),
        // initial position state
        new TrapezoidProfile.State(m_arm.getElevatorHeight()[0], 0)),
        // Pipe the profile state to the drive
        setpointState -> m_arm.setElevatorHeight(setpointState));

    CommandScheduler.getInstance().schedule(m_command);
  }

  @Override
  public boolean isFinished() {
    return m_command != null && m_command.isFinished();
  }
}
