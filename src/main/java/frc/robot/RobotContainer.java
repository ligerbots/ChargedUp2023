// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.commands.Drive;
import frc.robot.commands.DriveAndMoveArm;
import frc.robot.commands.FeederPickup;
import frc.robot.commands.MoveArmAndDrive;
import frc.robot.commands.ScoreArm;
import frc.robot.commands.DriveAndMoveArm;
import frc.robot.commands.SetArmAngleTest;
import frc.robot.commands.SetArmLengthTest;
import frc.robot.commands.TagPositionDrive;
import frc.robot.commands.ChargeStationBalance;

import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LedLight;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Xbox button mapping
    private static final int XBOX_A = 1;
    private static final int XBOX_B = 2;
    private static final int XBOX_X = 3;
    private static final int XBOX_Y = 4;

    // left and right bumpers
    private static final int XBOX_LB = 5;
    private static final int XBOX_RB = 6;
    
    // private static final int XBOX_BACK = 7;
    private static final int XBOX_START = 8;

    // joy stick button
    // private static final int XBOX_JL = 9;
    // private static final int XBOX_JR = 10;

    private final XboxController m_controller = new XboxController(0);
    private final Joystick m_farm = new Joystick(1);

    // The robot's subsystems and commands are defined here...
    private final Vision m_vision = new Vision();
    private final DriveTrain m_driveTrain = new DriveTrain(m_vision);
    private final Arm m_arm = new Arm();
    private final Claw m_claw = new Claw();
    private final LedLight m_ledLight = new LedLight();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        configureButtonBindings();
        m_driveTrain.setDefaultCommand(getDriveCommand());
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // when button A is pressed, toggle field-centric drive mode
        JoystickButton xboxAButton = new JoystickButton(m_controller, XBOX_A);
        xboxAButton.onTrue(new InstantCommand(m_driveTrain::toggleFieldCentric));

        // when button B is pressed, lock wheels
        JoystickButton xboxBButton = new JoystickButton(m_controller, XBOX_B);
        xboxBButton.onTrue(new InstantCommand(m_driveTrain::lockWheels, m_driveTrain));

        // when button X is pressed, toggle precision (slow) drive mode
        // JoystickButton xboxXButton = new JoystickButton(m_controller, XBOX_X);
        // xboxXButton.onTrue(new InstantCommand(m_driveTrain::togglePrecisionMode));

        // when button START is pressed, reset the robot heading
        // whichever way the robot is facing becomes the forward direction
        JoystickButton xboxStartButton = new JoystickButton(m_controller, XBOX_START);
        xboxStartButton.onTrue(new InstantCommand(m_driveTrain::resetHeading));

        JoystickButton leftBumper = new JoystickButton(m_controller, XBOX_LB);
        leftBumper.onTrue(new InstantCommand(m_claw::open));

        JoystickButton rightBumper = new JoystickButton(m_controller, XBOX_RB);
        rightBumper.onTrue(new InstantCommand(m_claw::close));
                
        //Turns analog triggers into buttons that actuate when it is half pressed 
        Trigger rightTriggerButton = new Trigger(() -> m_controller.getRightTriggerAxis() >= 0.5);
        rightTriggerButton.onTrue(new ScoreArm(m_arm, Constants.Position.PICK_UP).andThen(new InstantCommand(m_claw::open)));
        
        Trigger leftTriggerButton = new Trigger(() -> m_controller.getLeftTriggerAxis() >= 0.5);
        leftTriggerButton.onTrue(new InstantCommand(m_claw::close).andThen(new ScoreArm(m_arm, Constants.Position.STOW_ARM).andThen()));

        // ---- TESTING  ----
        JoystickButton xboxYButton = new JoystickButton(m_controller, XBOX_Y);
        JoystickButton xboxXButton = new JoystickButton(m_controller, XBOX_X);
        // xboxYButton.onTrue(new InstantCommand(m_claw::close));
        xboxYButton.onTrue(new SetArmAngleTest(m_arm));
        xboxXButton.onTrue(new SetArmLengthTest(m_arm));
        // testing if the command works by passing in a position, we need more buttons for all 11
        // xboxYButton.onTrue(new TagPositionDrive(m_driveTrain, m_vision, Constants.Position.LEFT_TOP));
        
        // // when button Y is pressed, attempt to balance on the Charging Station
        // // assumes that the robot is already mostly up on the Station
        // xboxYButton.onTrue(new ChargeStationBalance(m_driveTrain));

        // when button Y is pressed, attempt to drive up onto the Charging Station
        // JoystickButton xboxYButton = new JoystickButton(m_controller, XBOX_Y);
        // xboxYButton.onTrue(new ChargeStationDrive());

        /* //Commented out for now
        // button B
        JoystickButton xboxBButton = new JoystickButton(m_controller, XBOX_B);
        // inline command to create trajectory from robot pose to middle of the best apriltag
        xboxBButton.onTrue(new ProxyCommand(() -> m_driveTrain.trajectoryToPose(m_driveTrain.getTagRobotPose(false, false))));
        //need a proxy so command is not created before button pressed
        
        // button Y
        JoystickButton xboxYButton = new JoystickButton(m_controller, XBOX_Y);
        // inline command to create trajectory from robot pose to right of the best apriltag
        xboxYButton.onTrue(new ProxyCommand(() -> m_driveTrain.trajectoryToPose(m_driveTrain.getTagRobotPose(false, true))));
        */

        JoystickButton farm1 = new JoystickButton(m_farm, 1);
        farm1.onTrue(new DriveAndMoveArm(m_arm, m_driveTrain, m_vision, Constants.Position.LEFT_BOTTOM));
        
        JoystickButton farm2 = new JoystickButton(m_farm, 2);
        farm2.onTrue(new DriveAndMoveArm(m_arm, m_driveTrain, m_vision, Constants.Position.CENTER_BOTTOM));

        JoystickButton farm3 = new JoystickButton(m_farm, 3);
        farm3.onTrue(new DriveAndMoveArm(m_arm, m_driveTrain, m_vision, Constants.Position.RIGHT_BOTTOM));

        JoystickButton farm6 = new JoystickButton(m_farm, 6);
        farm6.onTrue(new DriveAndMoveArm(m_arm, m_driveTrain, m_vision, Constants.Position.LEFT_MIDDLE));

        JoystickButton farm7 = new JoystickButton(m_farm, 7);
        farm7.onTrue(new MoveArmAndDrive(m_arm, m_driveTrain, m_vision, Constants.Position.CENTER_MIDDLE));

        JoystickButton farm8 = new JoystickButton(m_farm, 8);
        farm8.onTrue(new DriveAndMoveArm(m_arm, m_driveTrain, m_vision, Constants.Position.RIGHT_MIDDLE));

        JoystickButton farm11 = new JoystickButton(m_farm, 11);
        farm11.onTrue(new DriveAndMoveArm(m_arm, m_driveTrain, m_vision, Constants.Position.LEFT_TOP));

        JoystickButton farm13 = new JoystickButton(m_farm, 13);
        farm13.onTrue(new MoveArmAndDrive(m_arm, m_driveTrain, m_vision, Constants.Position.CENTER_TOP));

        JoystickButton farm15 = new JoystickButton(m_farm, 15);
        farm15.onTrue(new DriveAndMoveArm(m_arm, m_driveTrain, m_vision, Constants.Position.RIGHT_TOP));

        //Feeder Stations 
        JoystickButton farm4 = new JoystickButton(m_farm, 4);
        farm4.onTrue(new FeederPickup(m_arm, m_driveTrain, m_vision, m_claw, Constants.Position.LEFT_SUBSTATION));

        JoystickButton farm5 = new JoystickButton(m_farm, 5);
        farm5.onTrue(new FeederPickup(m_arm, m_driveTrain, m_vision, m_claw, Constants.Position.RIGHT_SUBSTATION));

        // LED Lights
        JoystickButton farm9 = new JoystickButton(m_farm, 9);
        farm9.onTrue(new InstantCommand(()->m_ledLight.setColor(LedLight.Color.ORANGE)));

        JoystickButton farm10 = new JoystickButton(m_farm, 10);
        farm10.onTrue(new InstantCommand(()->m_ledLight.setColor(LedLight.Color.PURPLE)));
    }

    public Command getDriveCommand() {
        // The controls are for field-oriented driving:
        // Left stick Y axis -> forward and backwards movement
        // Left stick X axis -> left and right movement
        // Right stick X axis -> rotation
        return new Drive(
                m_driveTrain,
                () -> -modifyAxis(m_controller.getLeftY()),
                () -> -modifyAxis(m_controller.getLeftX()),
                () -> -modifyAxis(m_controller.getRightX()));
    }

    private static double deadband(double value, double deadband) {
        if (Math.abs(value) > deadband) {
            if (value > 0.0) {
                return (value - deadband) / (1.0 - deadband);
            } else {
                return (value + deadband) / (1.0 - deadband);
            }
        } else {
            return 0.0;
        }
    }

    private static double modifyAxis(double value) {
        // Deadband
        value = deadband(value, 0.05);

        // Square the axis
        value = Math.copySign(value * value, value);

        return value;
    }

    public DriveTrain getDriveTrain() {
        return m_driveTrain;
    }
    public Arm getArm(){
        return m_arm;
    }
    public Claw getClaw(){
        return m_claw;
    }
}
