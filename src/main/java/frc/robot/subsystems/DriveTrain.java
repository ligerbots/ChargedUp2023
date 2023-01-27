// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.commands.FollowTrajectory;
import frc.robot.subsystems.DriveTrain;

import frc.robot.swerve.*;
import static frc.robot.Constants.*;

public class DriveTrain extends SubsystemBase {

	// the max velocity for drivetrain
	// adjusted when in precision driving mode
	private double m_maxVelocity = MAX_VELOCITY_METERS_PER_SECOND;

	private double m_maxAngularVelocity = MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;

	// if true, then robot is in field centric mode
	private boolean m_fieldCentric = true;

	// if true, then robot is in precision mode
	private boolean m_precisionMode = false;

	// FIXME Measure the drivetrain's maximum velocity or calculate the theoretical.
	// The formula for calculating the theoretical maximum velocity is:
	// <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> *
	// pi
	// By default this value is setup for a Mk3 standard module using Falcon500s to
	// drive.
	// An example of this constant for a Mk4 L2 module with NEOs to drive is:
	// 5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() *
	// SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
	/**
	 * The maximum velocity of the robot in meters per second.
	 * <p>
	 * This is a measure of how fast the robot should be able to drive in a straight
	 * line.
	 */
	private static final double MAX_VELOCITY_METERS_PER_SECOND = 5880.0 / 60.0 *
			NeoDriveController.DRIVE_REDUCTION * NeoDriveController.WHEEL_DIAMETER * Math.PI;

	// TODO: tune and check this
	private static final double MAX_VELOCITY_PRECISION_MODE = MAX_VELOCITY_METERS_PER_SECOND / 6.0;

	/**
	 * The maximum angular velocity of the robot in radians per second.
	 * <p>
	 * This is a measure of how fast the robot can rotate in place.
	 */
	// Here we calculate the theoretical maximum angular velocity. You can also
	// replace this with a measured amount.
	private static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
			Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);


	private static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND_PRECISION_MODE = 
			MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND / 6.0;

	private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
			// Front left
			new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
			// Front right
			new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
			// Back left
			new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
			// Back right
			new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0));

	// NavX connected over MXP
	private final AHRS m_navx = new AHRS(Port.kMXP, (byte) 200);

	// These are our modules. We initialize them in the constructor.
	private final SwerveModule[] m_swerveModules = new SwerveModule[4];

	// the odometry class to keep track of where the robot is on the field
	private final SwerveDrivePoseEstimator m_odometry;

	private final Vision m_vision;

	// private final Field2d m_fieldSim;

	// PID controller for swerve
	private final PIDController m_xController = new PIDController(X_PID_CONTROLLER_P, 0, 0);
	private final PIDController m_yController = new PIDController(Y_PID_CONTROLLER_P, 0, 0);
	private final ProfiledPIDController m_thetaController = new ProfiledPIDController(THETA_PID_CONTROLLER_P,
			0, 0,
			new TrapezoidProfile.Constraints(4 * Math.PI, 4 * Math.PI));

	public DriveTrain(Vision vision) {

		m_swerveModules[0] = new SwerveModule(
				new frc.robot.swerve.NeoDriveController(FRONT_LEFT_MODULE_DRIVE_MOTOR),
				new frc.robot.swerve.NeoSteerController(FRONT_LEFT_MODULE_STEER_MOTOR, FRONT_LEFT_MODULE_STEER_ENCODER,
						FRONT_LEFT_MODULE_STEER_OFFSET));

		m_swerveModules[1] = new frc.robot.swerve.SwerveModule(
				new frc.robot.swerve.NeoDriveController(FRONT_RIGHT_MODULE_DRIVE_MOTOR),
				new frc.robot.swerve.NeoSteerController(FRONT_RIGHT_MODULE_STEER_MOTOR,
						FRONT_RIGHT_MODULE_STEER_ENCODER, FRONT_RIGHT_MODULE_STEER_OFFSET));

		m_swerveModules[2] = new frc.robot.swerve.SwerveModule(
				new frc.robot.swerve.NeoDriveController(BACK_LEFT_MODULE_DRIVE_MOTOR),
				new frc.robot.swerve.NeoSteerController(BACK_LEFT_MODULE_STEER_MOTOR, BACK_LEFT_MODULE_STEER_ENCODER,
						BACK_LEFT_MODULE_STEER_OFFSET));

		m_swerveModules[3] = new frc.robot.swerve.SwerveModule(
				new frc.robot.swerve.NeoDriveController(BACK_RIGHT_MODULE_DRIVE_MOTOR),
				new frc.robot.swerve.NeoSteerController(BACK_RIGHT_MODULE_STEER_MOTOR, BACK_RIGHT_MODULE_STEER_ENCODER,
						BACK_RIGHT_MODULE_STEER_OFFSET));

		// initialize the odometry class
		// needs to be done after the Modules are created and initialized
		// TODO add in the uncertainty matrices for encoders vs vision measurements
		m_odometry = new SwerveDrivePoseEstimator(m_kinematics, getGyroscopeRotation(), getModulePositions(),
				new Pose2d());

		m_vision = vision;

		// as late as possible, re-sync the swerve angle encoders
		for (SwerveModule module : m_swerveModules) {
			module.syncAngleEncoders(true);
		}
	}

	// sets the heading to zero with the existing pose
	public void resetHeading() {
		Pose2d pose = getPose();

		Pose2d newPose = new Pose2d(pose.getX(), pose.getY(), new Rotation2d(0));

		setPose(newPose);
	}

	public Pose2d getPose() {
		return m_odometry.getEstimatedPosition();
	}

	// sets the odometry to the specified pose
	public void setPose(Pose2d pose) {
		m_odometry.resetPosition(getGyroscopeRotation(), getModulePositions(), pose);
	}

	public Rotation2d getHeading() {
		return m_odometry.getEstimatedPosition().getRotation();
	}

	// the gyro reading should be private.
	// Everyone else who wants the robot angle should call getHeading()
	private Rotation2d getGyroscopeRotation() {
		if (m_navx.isMagnetometerCalibrated()) {
			// We will only get valid fused headings if the magnetometer is calibrated
			return Rotation2d.fromDegrees(m_navx.getFusedHeading());
		}

		// We have to invert the angle of the NavX so that rotating the robot
		// counter-clockwise makes the angle increase.
		return Rotation2d.fromDegrees(360.0 - m_navx.getYaw());
	}

	public void joystickDrive(double inputX, double inputY, double inputRotation) {
		ChassisSpeeds chassisSpeeds;
		// when in field-relative mode
		if (m_fieldCentric) {
			chassisSpeeds = 
					ChassisSpeeds.fromFieldRelativeSpeeds(
							inputX * m_maxVelocity,
							inputY * m_maxVelocity,
							inputRotation * m_maxAngularVelocity,
							getHeading());
		}
		// when in robot-centric mode
		else {
			chassisSpeeds = new ChassisSpeeds(inputX * m_maxVelocity,
					inputY * m_maxVelocity,
					inputRotation * m_maxAngularVelocity);
		}
		drive(chassisSpeeds);		
	}

	public void drive(ChassisSpeeds chassisSpeeds) {
		SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(chassisSpeeds);
		SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);
		for (int i = 0; i < 4; i++) {
			m_swerveModules[i].set(states[i].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
					states[i].angle.getRadians());
		}
	}

	// future changes: maybe leave the modules so the angles remain the same instead
	// of pointing at 0
	public void stop() {
		drive(new ChassisSpeeds(0, 0, 0));
	}

	// for the beginning of auto rountines
	public void resetDrivingModes(){
		m_fieldCentric = true;
		m_precisionMode = false;
	}

	// toggle whether driving is field-centric
	public void toggleFieldCentric() {
		m_fieldCentric = !m_fieldCentric;
	}

	// toggle precision mode for driving
	public void togglePrecisionMode() {
		m_precisionMode = !m_precisionMode;
		m_maxVelocity = m_precisionMode ? MAX_VELOCITY_PRECISION_MODE : MAX_VELOCITY_METERS_PER_SECOND;
		m_maxAngularVelocity = m_precisionMode ? MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND_PRECISION_MODE : MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
	}

	public PIDController getXController(){ //gets the controller for x position of robot
		return m_xController;
	}

	public PIDController getYController(){ //gets controller for y position of bot
		return m_yController;
	}

	public ProfiledPIDController getThetaController(){ //gets controller for angle
		return m_thetaController;
	}
	// get the swerveModuleState manually
	public SwerveModulePosition[] getModulePositions() {
		SwerveModulePosition[] state = new SwerveModulePosition[4];
		for (int i = 0; i < 4; i++) {
			state[i] = m_swerveModules[i].getSwerveModulePosition();
		}
		return state;
	}
	    
	public void syncSwerveAngleEncoders() {
		// check if we can sync the swerve angle encoders
		// this will only trigger if the chassis is idle for 10 seconds
		for (SwerveModule module : m_swerveModules) {
			module.syncAngleEncoders(false);
		}
	}

	@Override
	public void periodic() {
		Pose2d pose = m_odometry.update(getGyroscopeRotation(), getModulePositions());

		// Have the vision system update based on the Apriltags, if seen
		// Comment out for now so we don't get exceptions
		// m_vision.updateOdometry(m_odometry);

		SmartDashboard.putNumber("drivetrain/xPosition", pose.getX());
		SmartDashboard.putNumber("drivetrain/yPosition", pose.getY());
		SmartDashboard.putNumber("drivetrain/heading", pose.getRotation().getDegrees());

		SmartDashboard.putBoolean("drivetrain/fieldCentric", m_fieldCentric);

		m_swerveModules[0].updateSmartDashboard("drivetrain/frontLeft");
		m_swerveModules[1].updateSmartDashboard("drivetrain/frontRight");
		m_swerveModules[2].updateSmartDashboard("drivetrain/backLeft");
		m_swerveModules[3].updateSmartDashboard("drivetrain/backRight");
	}

	// get the trajectory following autonomous command in PathPlanner using the name
	public Command getTrajectoryFollowingCommand(String trajectoryName) {

		PathPlannerTrajectory traj = PathPlanner.loadPath(trajectoryName, 2.0, 1.0);

		Command command = new FollowTrajectory(
				this,
				traj,
				() -> this.getPose(),
				m_kinematics,
				m_xController,
				m_yController,
				m_thetaController,
				(states) -> {
					this.drive(m_kinematics.toChassisSpeeds(states));
				},
				this).andThen(() -> stop());

		return command;
	}
}
