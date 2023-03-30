// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.Position;

import frc.robot.commands.AutoWallTwoCones;
import frc.robot.commands.AutoCommandInterface;
import frc.robot.commands.AutoFollowTrajectory;
import frc.robot.commands.AutoBarrierTwoCones;
import frc.robot.commands.AutoChargeStationOneConeOtherSide;
import frc.robot.commands.AutoChargeStationOneCubeOtherSide;
import frc.robot.commands.TrajectoryPlotter;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.LedLight.Color;

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
        // DataLogManager.start();
        
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
        // m_chosenTrajectory.addOption("drive_and_slide", new AutoFollowTrajectory(driveTrain, "drive_and_slide"));
        // m_chosenTrajectory.addOption("drive_and_turn", new AutoFollowTrajectory(driveTrain, "drive_and_turn"));
        // m_chosenTrajectory.addOption("c_forward_balance", new AutoFollowTrajectory(driveTrain, "c_forward_balance"));
        // m_chosenTrajectory.addOption("top_grid_s1", new AutoFollowTrajectory(driveTrain, "top_grid_s1"));

        SmartDashboard.putData("Chosen Trajectory", m_chosenTrajectory);

        m_plotter = new TrajectoryPlotter(m_robotContainer.getDriveTrain().getField2d());

        m_robotContainer.getLED().setColor(Color.OFF);
        m_robotContainer.getClaw().enableCompressor();

        // TODO: tune the period
        addPeriodic(claw.updateIRSensorPeriodic(), 0.01);
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

        // Make sure the Arm Sooulder and Reacher won't move again when we re-enable.
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
        }
        m_robotContainer.getDriveCommand().schedule();

        // m_robotContainer.getClaw().enableCompressor();
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
