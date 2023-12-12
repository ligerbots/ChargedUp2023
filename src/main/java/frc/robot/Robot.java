// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.Position;
import frc.robot.commands.AutoBarrierTwoCones;
// import frc.robot.commands.AutoChargeStationOneCone;
import frc.robot.commands.AutoChargeStationOneConeOtherSide;
import frc.robot.commands.AutoCommandInterface;
import frc.robot.commands.AutoFollowTrajectory;
import frc.robot.commands.AutoWallTwoCones;
// import frc.robot.commands.AutoChargeStationOneCubeOtherSide;
import frc.robot.commands.TrajectoryPlotter;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LedLight.Color;
import frc.robot.subsystems.Vision;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
// public class Robot extends TimedRobot {
public class Robot extends LoggedRobot {
    private Command m_autonomousCommand;
    private SendableChooser<AutoCommandInterface> m_chosenTrajectory = new SendableChooser<>();
    private RobotContainer m_robotContainer;
    private TrajectoryPlotter m_plotter;
    private AutoCommandInterface m_prevAutoCommand = null;

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {



        // AdvantageKit sample init

        Logger logger = Logger.getInstance();

        logger.recordMetadata("ProjectName", "MyProject"); // Set a metadata value

        if (isReal()) {
            logger.addDataReceiver(new WPILOGWriter("/U")); // Log to a USB stick
            logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
            new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
        } else {
            setUseTiming(false); // Run as fast as possible
            String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the
                                                          // user)
            logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
            logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a
                                                                                                  // new log
        }

        // Logger.disableDeterministicTimestamps() // See "Deterministic Timestamps" in the "Understanding Data Flow"
        // page
        logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.

        /////////////////////////////
        /////////////////////////////
        
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
        
        // Disable the LiveWindow telemetry to lower the network load
        LiveWindow.disableAllTelemetry();

        // Enable local logging.
        // ** CAREFUL: this probably should be disabled during competition.
        DataLogManager.start();
        
        // Instantiate our RobotContainer.  This will perform all our button bindings.
        m_robotContainer = new RobotContainer();

        DriveTrain driveTrain = m_robotContainer.getDriveTrain();
        Arm arm = m_robotContainer.getArm();
        Vision vision = m_robotContainer.getVision();
        Claw claw = m_robotContainer.getClaw();
        JoystickButton overrideButton = m_robotContainer.getOverRideButton();

        // Initialize the list of available Autonomous routines
        m_chosenTrajectory.setDefaultOption("drive_1m", new AutoFollowTrajectory(driveTrain, "drive_1m"));
        m_chosenTrajectory.addOption("Barrier Cone Cube", new AutoBarrierTwoCones(driveTrain, arm, vision, claw, Position.CENTER_TOP, overrideButton));
        m_chosenTrajectory.addOption("Wall Cone Cube", new AutoWallTwoCones(driveTrain, arm, vision, claw, Position.CENTER_TOP, overrideButton));
        // m_chosenTrajectory.addOption("Charge Station Cube", new AutoChargeStationOneCube(driveTrain, arm, vision, claw));
        // m_chosenTrajectory.addOption("Charge Station Cube", new AutoChargeStationOneCubeOtherSide(driveTrain, arm, vision, claw));
        m_chosenTrajectory.addOption("Charge Station WALL Cone", new AutoChargeStationOneConeOtherSide(driveTrain, arm, vision, claw, true, overrideButton));
        m_chosenTrajectory.addOption("Charge Station BARRIER Cone", new AutoChargeStationOneConeOtherSide(driveTrain, arm, vision, claw, false, overrideButton));

        // For BattleCry, no need to go over and back
        // m_chosenTrajectory.addOption("Charge Station WALL Cone", new AutoChargeStationOneCone(driveTrain, arm, vision, claw, true, overrideButton));
        // m_chosenTrajectory.addOption("Charge Station BARRIER Cone", new AutoChargeStationOneCone(driveTrain, arm, vision, claw, false, overrideButton));
        // m_chosenTrajectory.addOption("drive_and_slide", new AutoFollowTrajectory(driveTrain, "drive_and_slide"));
        // m_chosenTrajectory.addOption("drive_and_turn", new AutoFollowTrajectory(driveTrain, "drive_and_turn"));
        // m_chosenTrajectory.addOption("c_forward_balance", new AutoFollowTrajectory(driveTrain, "c_forward_balance"));
        // m_chosenTrajectory.addOption("top_grid_s1", new AutoFollowTrajectory(driveTrain, "top_grid_s1"));

        SmartDashboard.putData("Chosen Trajectory", m_chosenTrajectory);

        m_plotter = new TrajectoryPlotter(m_robotContainer.getDriveTrain().getField2d());

        m_robotContainer.getLED().setColor(Color.OFF);
        m_robotContainer.getClaw().enableCompressor();

        // // update Claw sensor every 2ms
        // addPeriodic(claw.updateIRSensorPeriodic(), 0.002);
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

        // Make sure the Arm Shoulder and Reacher won't move again when we re-enable.
        m_robotContainer.getArm().resetGoal();

        // auto trajectory plotter
        AutoCommandInterface autoCommandInterface = m_chosenTrajectory.getSelected();
        if (autoCommandInterface != null && autoCommandInterface != m_prevAutoCommand) {
            m_robotContainer.getDriveTrain().setPose(autoCommandInterface.getInitialPose());
            m_prevAutoCommand = autoCommandInterface;

            if (Robot.isSimulation()) {
                m_plotter.clear();
                autoCommandInterface.plotTrajectory(m_plotter);
            }
        }
    }

    /**
     * This autonomous runs the autonomous command selected by your
     * {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        //Resets the goal of the arm So that it does not turn back to its origional position 
        // m_robotContainer.getArm().resetGoal();
        m_autonomousCommand = m_chosenTrajectory.getSelected();
    
        // schedule the autonomous command
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }

        // m_robotContainer.getClaw().enableCompressor();
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
            m_plotter.clear();
        }

        // for safety, some autos need to raise the arm 
        //  because it can fall while disabled
        m_robotContainer.getArm().raiseArmAfterAuto();

        m_robotContainer.getDriveCommand().schedule();
        m_robotContainer.getDriveTrain().resetDrivingModes();
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
