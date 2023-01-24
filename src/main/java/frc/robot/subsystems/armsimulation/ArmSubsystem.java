// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.armsimulation;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants;

/** A robot arm subsystem that moves with a motion profile. */
public class ArmSubsystem extends ProfiledPIDSubsystem {
  private final PWMSparkMax m_motor = new PWMSparkMax(Constants.kMotorPort);
  private final Encoder m_encoder =
      new Encoder(Constants.kEncoderPorts[0], Constants.kEncoderPorts[1]);
  private final ArmFeedforward m_feedforward =
      new ArmFeedforward(
          Constants.kSVolts, Constants.kGVolts,
          Constants.kVVoltSecondPerRad, Constants.kAVoltSecondSquaredPerRad);

  /** Create a new ArmSubsystem. */
  public ArmSubsystem() {
    super(
        new ProfiledPIDController(
            Constants.kP,
            0,
            0,
            new TrapezoidProfile.Constraints(
                Constants.kMaxVelocityRadPerSecond,
                Constants.kMaxAccelerationRadPerSecSquared)),
        0);
    m_encoder.setDistancePerPulse(Constants.kEncoderDistancePerPulse);
    // Start arm at rest in neutral position
    setGoal(Constants.kArmOffsetRads);
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    // Calculate the feedforward from the sepoint
    double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
    // Add the feedforward to the PID output to get the motor output
    m_motor.setVoltage(output + feedforward);
  }

  @Override
  public double getMeasurement() {
    return m_encoder.getDistance() + Constants.kArmOffsetRads;
  }
}
