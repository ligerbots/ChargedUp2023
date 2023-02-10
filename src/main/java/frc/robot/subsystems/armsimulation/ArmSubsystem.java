// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.armsimulation;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** This is a sample program to demonstrate the use of arm simulation with existing code. */
public class ArmSubsystem extends SubsystemBase {
  public static final int kMotorPort = 0;
  public static final int kEncoderAChannel = 0;
  public static final int kEncoderBChannel = 1;
  public static final int kJoystickPort = 0;

  public static final String kArmPositionKey = "ArmPosition";
  public static final String kArmPKey = "ArmP";

  // The P gain for the PID controller that drives this arm.
  public static double kArmKp = 50.0;

  public static double armPositionDeg = 75.0;
  
  // distance per pulse = (angle per revolution) / (pulses per revolution)
  //  = (2 * PI rads) / (4096 pulses)
  public static final double kArmEncoderDistPerPulse = 2.0 * Math.PI / 4096;

  // The arm gearbox represents a gearbox containing two Vex 775pro motors.
  public final DCMotor m_armGearbox = DCMotor.getVex775Pro(2);

  // Standard classes for controlling our arm
  public final PIDController m_controller = new PIDController(kArmKp, 0, 0);
  public final Encoder m_encoder = new Encoder(kEncoderAChannel, kEncoderBChannel);
  public final PWMSparkMax m_motor = new PWMSparkMax(kMotorPort);
  public final Joystick m_joystick = new Joystick(kJoystickPort);

  // Simulation classes help us simulate what's going on, including gravity.
  public static final double m_armReduction = 200;
  public static final double m_armMass = 14.0; // Kilograms
  public static final double m_armLength = Units.inchesToMeters(30);
  // This arm sim represents an arm that can travel from -75 degrees (rotated down front)
  // to 255 degrees (rotated down in the back).
  public final SingleJointedArmSim m_armSim =
      new SingleJointedArmSim(
          m_armGearbox,
          m_armReduction,
          SingleJointedArmSim.estimateMOI(m_armLength, m_armMass),
          m_armLength,
          Units.degreesToRadians(-75),
          Units.degreesToRadians(255),
          m_armMass, 
          false,
          VecBuilder.fill(kArmEncoderDistPerPulse) // Add noise with a std-dev of 1 tick
          );
  public final EncoderSim m_encoderSim = new EncoderSim(m_encoder);

  // Create a Mechanism2d display of an Arm with a fixed ArmTower and moving Arm.
  public final Mechanism2d m_mech2d = new Mechanism2d(60, 60);
  public final MechanismRoot2d m_armPivot = m_mech2d.getRoot("ArmPivot", 30, 30);
  public final MechanismLigament2d m_armTower =
      m_armPivot.append(new MechanismLigament2d("ArmTower", 30, -90));
  public final MechanismLigament2d m_arm =
      m_armPivot.append(
          new MechanismLigament2d(
              "Arm",
              30,
              Units.radiansToDegrees(m_armSim.getAngleRads()),
              6,
              new Color8Bit(Color.kYellow)));


  public ArmSubsystem() {
    SmartDashboard.putData("Arm Sim", m_mech2d);
  }
              @Override
  public void simulationPeriodic() {
    // In this method, we update our simulation of what our arm is doing
    // First, we set our "inputs" (voltages)
    m_armSim.setInput(m_motor.get() * RobotController.getBatteryVoltage());

    // Next, we update it. The standard loop time is 20ms.
    m_armSim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery voltage
    m_encoderSim.setDistance(m_armSim.getAngleRads());
    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_armSim.getCurrentDrawAmps()));

    // Update the Mechanism Arm angle based on the simulated arm angle
    m_arm.setAngle(Units.radiansToDegrees(m_armSim.getAngleRads()));
  }
}