// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

<<<<<<< HEAD
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.networktables.NetworkTableInstance;
=======
>>>>>>> main
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

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
<<<<<<< HEAD
  private Command m_autonomousCommand;
  private SendableChooser<Command> m_chosenTrajectory = new SendableChooser<>();
  private RobotContainer m_robotContainer;
  double m_goal = Math.toRadians(90.0);
  WPI_TalonFX m_motorLeader = new WPI_TalonFX(2, "rio");  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    // if(this.isSimulation()){
    //   //At-Home Network Debug Only - host the NT server on photonvision and connect to it.
    //   NetworkTableInstance.getDefault().stopServer();
    //   // NetworkTableInstance.getDefault().setServer("photonvision.local");
    //   NetworkTableInstance.getDefault().setServer("127.0.0.1");
    //   NetworkTableInstance.getDefault().startClient4("MainRobotProgram");
    // }
    m_motorLeader.configFactoryDefault();
		
		/* Config the sensor used for Primary PID and sensor direction */
        m_motorLeader.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 
                                            0,
				                            0);

		/* Ensure sensor is positive when output is positive */
		m_motorLeader.setSensorPhase(true);
    m_motorLeader.setInverted(false);
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    m_chosenTrajectory.setDefaultOption("drive_1m", m_robotContainer.getDriveTrain().getTrajectoryFollowingCommand("drive_1m"));
    m_chosenTrajectory.addOption("drive_and_slide", m_robotContainer.getDriveTrain().getTrajectoryFollowingCommand("drive_and_slide"));
    m_chosenTrajectory.addOption("drive_and_turn", m_robotContainer.getDriveTrain().getTrajectoryFollowingCommand("drive_and_turn"));
    m_chosenTrajectory.addOption("Test", m_robotContainer.getDriveTrain().getTrajectoryFollowingCommand("Test"));
    SmartDashboard.putData("Chosen Trajectory", m_chosenTrajectory);
    SmartDashboard.putNumber("Arm Goal", Math.toDegrees(m_goal));
    
  
  }
  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_chosenTrajectory.getSelected();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
=======
    private Command m_autonomousCommand;
    private SendableChooser<Command> m_chosenTrajectory = new SendableChooser<>();
    private RobotContainer m_robotContainer;

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {

        // If you are trying to work with a running PV and simulation, enable this code.
        // Otherwise it is not needed, even when running a simulation
        // if (isSimulation()) {
        //   // At-Home Network Debug Only - host the NT server on photonvision and connect to it.
        //   var ntinst = edu.wpi.first.networktables.NetworkTableInstance.getDefault();
        //   ntinst.stopServer();
        //   // ntinst.setServer("photonvision.local");
        //   ntinst.setServer("127.0.0.1");
        //   ntinst.startClient4("MainRobotProgram");
        // }
        
        // Instantiate our RobotContainer.  This will perform all our button bindings.
        m_robotContainer = new RobotContainer();

        // Initialize the list of available Autonomous routines
        m_chosenTrajectory.setDefaultOption("drive_1m", m_robotContainer.getDriveTrain().getTrajectoryFollowingCommand("drive_1m"));
        m_chosenTrajectory.addOption("drive_and_slide", m_robotContainer.getDriveTrain().getTrajectoryFollowingCommand("drive_and_slide"));
        m_chosenTrajectory.addOption("drive_and_turn", m_robotContainer.getDriveTrain().getTrajectoryFollowingCommand("drive_and_turn"));
        SmartDashboard.putData("Chosen Trajectory", m_chosenTrajectory);
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
        m_robotContainer.getDriveTrain().syncSwerveAngleEncoders();
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
>>>>>>> main
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

    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
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
