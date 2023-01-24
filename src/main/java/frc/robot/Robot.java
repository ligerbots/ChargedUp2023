// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
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

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private SendableChooser<Command> m_chosenTrajectory = new SendableChooser<>();
  private RobotContainer m_robotContainer;
  private static final int kMotorPort = 0;
  private static final int kEncoderAChannel = 0;
  private static final int kEncoderBChannel = 1;
  private static final int kJoystickPort = 0;

  public static final String kArmPositionKey = "ArmPosition";
  public static final String kArmPKey = "ArmP";

  // The P gain for the PID controller that drives this arm.
  private static double kArmKp = 50.0;

  private static double armPositionDeg = 75.0;

  // distance per pulse = (angle per revolution) / (pulses per revolution)
  // = (2 * PI rads) / (4096 pulses)
  private static final double kArmEncoderDistPerPulse = 2.0 * Math.PI / 4096;

  // The arm gearbox represents a gearbox containing two neo motors.
  private final DCMotor m_armGearbox = DCMotor.getVex775Pro(2);

  // Standard classes for controlling our arm
  private final PIDController m_controller = new PIDController(kArmKp, 0, 0);
  private final Encoder m_encoder = new Encoder(kEncoderAChannel, kEncoderBChannel);
  private final PWMSparkMax m_motor = new PWMSparkMax(kMotorPort);
  private final Joystick m_joystick = new Joystick(kJoystickPort);

  // Simulation classes help us simulate what's going on, including gravity.
  private static final double m_armReduction = 5;
  private static final double m_armMass = 5; // Kilograms
  private static final double m_armLength = Units.inchesToMeters(28.0);
  // This arm sim represents an arm that can travel from -75 degrees (rotated down
  // front)
  // to 255 degrees (rotated down in the back).
  private final SingleJointedArmSim m_armSim = new SingleJointedArmSim(
      m_armGearbox,
      m_armReduction,
      14,
      m_armLength,
      Units.degreesToRadians(-75),
      Units.degreesToRadians(255),
      m_armMass,
      true,
      VecBuilder.fill(kArmEncoderDistPerPulse) // Add noise with a std-dev of 1 tick
  );
  private final EncoderSim m_encoderSim = new EncoderSim(m_encoder);

  // Create a Mechanism2d display of an Arm with a fixed ArmTower and moving Arm.
  private final Mechanism2d m_mech2d = new Mechanism2d(60, 60);
  private final MechanismRoot2d m_armPivot = m_mech2d.getRoot("ArmPivot", 30, 30);
  private final MechanismLigament2d m_armTower = m_armPivot.append(new MechanismLigament2d("ArmTower", 30, -90));
  private final MechanismLigament2d m_arm = m_armPivot.append(
      new MechanismLigament2d(
          "Arm",
          30,
          Units.radiansToDegrees(m_armSim.getAngleRads()),
          6,
          new Color8Bit(Color.kYellow)));

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    m_chosenTrajectory.setDefaultOption("drive_1m",
        m_robotContainer.getDriveTrain().getTrajectoryFollowingCommand("drive_1m"));
    m_chosenTrajectory.addOption("drive_and_slide",
        m_robotContainer.getDriveTrain().getTrajectoryFollowingCommand("drive_and_slide"));
    m_chosenTrajectory.addOption("drive_and_turn",
        m_robotContainer.getDriveTrain().getTrajectoryFollowingCommand("drive_and_turn"));
    m_chosenTrajectory.addOption("Test", m_robotContainer.getDriveTrain().getTrajectoryFollowingCommand("Test"));
    SmartDashboard.putData("Chosen Trajectory", m_chosenTrajectory);
    m_encoder.setDistancePerPulse(kArmEncoderDistPerPulse);

    // Put Mechanism 2d to SmartDashboard
    SmartDashboard.putData("Arm Sim", m_mech2d);
    m_armTower.setColor(new Color8Bit(Color.kBlue));

    // Set the Arm position setpoint and P constant to Preferences if the keys don't
    // already exist
    if (!Preferences.containsKey(kArmPositionKey)) {
      Preferences.setDouble(kArmPositionKey, armPositionDeg);
    }
    if (!Preferences.containsKey(kArmPKey)) {
      Preferences.setDouble(kArmPKey, kArmKp);
    }

  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and
   * test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */

  @Override
  public void simulationPeriodic() {
    // In this method, we update our simulation of what our arm is doing
    // First, we set our "inputs" (voltages)
    m_armSim.setInput(m_motor.get() * RobotController.getBatteryVoltage());

    // Next, we update it. The standard loop time is 20ms.
    m_armSim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery
    // voltage
    m_encoderSim.setDistance(m_armSim.getAngleRads());
    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_armSim.getCurrentDrawAmps()));

    // Update the Mechanism Arm angle based on the simulated arm angle
    m_arm.setAngle(Units.radiansToDegrees(m_armSim.getAngleRads()));

  }

  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    m_motor.set(0.0);
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_chosenTrajectory.getSelected();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.getDriveCommand().schedule();
       // Read Preferences for Arm setpoint and kP on entering Teleop
       armPositionDeg = Preferences.getDouble(kArmPositionKey, armPositionDeg);
       if (kArmKp != Preferences.getDouble(kArmPKey, kArmKp)) {
         kArmKp = Preferences.getDouble(kArmPKey, kArmKp);
         m_controller.setP(kArmKp);
       }
   
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    if (m_joystick.getTrigger()) {
      // Here, we run PID control like normal, with a constant setpoint of 75 degrees.
      var pidOutput =
          m_controller.calculate(m_encoder.getDistance(), Units.degreesToRadians(armPositionDeg));
      m_motor.setVoltage(pidOutput);
    } else {
      // Otherwise, we disable the motor.
      m_motor.set(0.0);
    }

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }
}
