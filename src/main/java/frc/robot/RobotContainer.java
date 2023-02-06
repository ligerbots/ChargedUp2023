// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import frc.robot.commands.Drive;
import frc.robot.commands.ChargeStationBalance;

import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Vision;

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
    // The robot's subsystems and commands are defined here...

    private final Vision m_vision = new Vision();

    private final DriveTrain m_driveTrain = new DriveTrain(m_vision);

    private final XboxController m_controller = new XboxController(0);

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
		JoystickButton xboxAButton = new JoystickButton(m_controller, Constants.XBOX_A);
		xboxAButton.onTrue(new InstantCommand(m_driveTrain::toggleFieldCentric));

		// trajectory to AprilTag vision buttons (3 of them, left, middle, right)

		// button X
		JoystickButton xboxXButton = new JoystickButton(m_controller, Constants.XBOX_X);
		// inline command to create trajectory from robot pose to left of the best apriltag
		xboxXButton.onTrue(new ProxyCommand(() -> m_driveTrain.trajectoryToPose(m_driveTrain.getTagRobotPose(true, false))));
		
		// button B
		JoystickButton xboxBButton = new JoystickButton(m_controller, Constants.XBOX_B);
		// inline command to create trajectory from robot pose to middle of the best apriltag
		xboxBButton.onTrue(new ProxyCommand(() -> m_driveTrain.trajectoryToPose(m_driveTrain.getTagRobotPose(false, false))));
		//need a proxy so command is not created before button pressed
		
		// button Y
		JoystickButton xboxYButton = new JoystickButton(m_controller, Constants.XBOX_Y);
		// inline command to create trajectory from robot pose to right of the best apriltag
		xboxYButton.onTrue(new ProxyCommand(() -> m_driveTrain.trajectoryToPose(m_driveTrain.getTagRobotPose(false, true))));
		
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
}
