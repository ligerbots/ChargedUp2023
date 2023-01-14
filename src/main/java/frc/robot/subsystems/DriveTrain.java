// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.wpilibj.SPI.Port;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.commands.FollowTrajectory;
import frc.robot.subsystems.DriveTrain;
import static frc.robot.Constants.*;

public class DriveTrain extends SubsystemBase {

	private double m_maxVoltage = Constants.MAX_VOLTAGE; //var that acts as max voltage for drivetrain

	// if var = true, then robot is in field relative mode
	private boolean m_fieldRelative = true; // var that controls if the robot is in field relative or robot centric mode

	//if var = true, then robot is in precision mode
	private boolean m_precisionMode = false; //var that determines precision or non precision mode

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
	// add arbitrary 3/4 reduction - PaulR
	public static final double MAX_VELOCITY_METERS_PER_SECOND = 0.75 * 5880.0 / 60.0 *
			SdsModuleConfigurations.MK4I_L2.getDriveReduction() *
			SdsModuleConfigurations.MK4I_L2.getWheelDiameter() * Math.PI;
	/**
	 * The maximum angular velocity of the robot in radians per second.
	 * <p>
	 * This is a measure of how fast the robot can rotate in place.
	 */
	// Here we calculate the theoretical maximum angular velocity. You can also
	// replace this with a measured amount.
	public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
			Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

	private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
			// Front left
			new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
			// Front right
			new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
			// Back left
			new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
			// Back right
			new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0));

	// By default we use a Pigeon for our gyroscope. But if you use another
	// gyroscope, like a NavX, you can change this.
	// The important thing about how you configure your gyroscope is that rotating
	// the robot counter-clockwise should
	// cause the angle reading to increase until it wraps back over to zero.
	private final AHRS m_navx = new AHRS(Port.kMXP, (byte) 200); // NavX connected over MXP

	// These are our modules. We initialize them in the constructor.
	private final SwerveModule[] m_swerveModules = new SwerveModule[4];

	// private final SwerveDrivePoseEstimator m_odometry = new SwerveDrivePoseEstimator(
	// 		getGyroscopeRotation(),
	// 		new Pose2d(),
	// 		m_kinematics,
	// 		VecBuilder.fill(0.1, 0.1, 0.1),
	// 		VecBuilder.fill(0.05),
	// 		VecBuilder.fill(0.1, 0.1, 0.1));

	// PID controller for swerve
	private final PIDController m_xController = new PIDController(Constants.X_PID_CONTROLLER_P, 0, 0);
	private final PIDController m_yController = new PIDController(Constants.Y_PID_CONTROLLER_P, 0, 0);
	private final ProfiledPIDController m_thetaController = new ProfiledPIDController(Constants.THETA_PID_CONTROLLER_P, 0, 0,
			new TrapezoidProfile.Constraints(4 * Math.PI, 4 * Math.PI));

	public DriveTrain() {
		ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

		m_swerveModules[0] = Mk4iSwerveModuleHelper.createNeo(
				// This parameter is optional, but will allow you to see the current state of
				// the module on the dashboard.
				tab.getLayout("Front Left Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(0, 0),
				// This can either be STANDARD or FAST depending on your gear configuration
				Mk4iSwerveModuleHelper.GearRatio.L2,
				// This is the ID of the drive motor
				FRONT_LEFT_MODULE_DRIVE_MOTOR,
				// This is the ID of the steer motor
				FRONT_LEFT_MODULE_STEER_MOTOR,
				// This is the ID of the steer encoder
				FRONT_LEFT_MODULE_STEER_ENCODER,
				// This is how much the steer encoder is offset from true zero (In our case,
				// zero is facing straight forward)
				FRONT_LEFT_MODULE_STEER_OFFSET);

		// We will do the same for the other modules
		m_swerveModules[1] = Mk4iSwerveModuleHelper.createNeo(
				tab.getLayout("Front Right Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(2, 0),
				Mk4iSwerveModuleHelper.GearRatio.L2,
				FRONT_RIGHT_MODULE_DRIVE_MOTOR,
				FRONT_RIGHT_MODULE_STEER_MOTOR,
				FRONT_RIGHT_MODULE_STEER_ENCODER,
				FRONT_RIGHT_MODULE_STEER_OFFSET);

		m_swerveModules[2] = Mk4iSwerveModuleHelper.createNeo(
				tab.getLayout("Back Left Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(4, 0),
				Mk4iSwerveModuleHelper.GearRatio.L2,
				BACK_LEFT_MODULE_DRIVE_MOTOR,
				BACK_LEFT_MODULE_STEER_MOTOR,
				BACK_LEFT_MODULE_STEER_ENCODER,
				BACK_LEFT_MODULE_STEER_OFFSET);

		m_swerveModules[3] = Mk4iSwerveModuleHelper.createNeo(
				tab.getLayout("Back Right Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(6, 0),
				Mk4iSwerveModuleHelper.GearRatio.L2,
				BACK_RIGHT_MODULE_DRIVE_MOTOR,
				BACK_RIGHT_MODULE_STEER_MOTOR,
				BACK_RIGHT_MODULE_STEER_ENCODER,
				BACK_RIGHT_MODULE_STEER_OFFSET);
	}

	/**
	 * Sets the gyroscope angle to zero. This can be used to set the direction the
	 * robot is currently facing to the
	 * 'forwards' direction.
	 */
	// This is wrong. You need to reset the odometry if you really need to do this
	// public void zeroGyroscope() {
	// m_navx.zeroYaw();
	// }

	public Pose2d getPose() {
		return new Pose2d();
		// return m_odometry.getEstimatedPosition();
	}

	public Rotation2d getGyroscopeRotation() {
		if (m_navx.isMagnetometerCalibrated()) {
			// We will only get valid fused headings if the magnetometer is calibrated
			return Rotation2d.fromDegrees(m_navx.getFusedHeading());
		}

		// We have to invert the angle of the NavX so that rotating the robot
		// counter-clockwise makes the angle increase.
		return Rotation2d.fromDegrees(360.0 - m_navx.getYaw());
	}

	public void drive(ChassisSpeeds chassisSpeeds) {
		SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(chassisSpeeds);
		SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);
		for (int i = 0; i < 4; i++) {
			m_swerveModules[i].set(states[i].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * m_maxVoltage,
					states[i].angle.getRadians());
		}
	}

	// get the trajectory following autonomous command in PathPlanner using the name
	public Command getTrajectoryFollowingCommand(String trajectoryName){
		
		var traj = PathPlanner.loadPath(trajectoryName, 2.0, 1.0);

		var autonomousCommand = new FollowTrajectory(
			this,
			traj,
			() -> this.getPose(),
			this.getKinematics(),
			m_xController,
			m_yController,
			m_thetaController,
			(states) -> {
				this.drive(this.getKinematics().toChassisSpeeds(states));
			},
			this
		).andThen(() -> stop());

		return autonomousCommand;
	}

	/**
	 * Resets the odometry to the specified pose.
	 *
	 * @param pose The pose to which to set the odometry.
	 */
	public void setPose(Pose2d pose) {
		// zeroGyroscope(); resetPosition says not to reset gyro
		// m_odometry.resetPosition(pose, getGyroscopeRotation());
	}

	public Rotation2d getHeading() {
		return new Rotation2d();
		// return m_odometry.getEstimatedPosition().getRotation();
	}

	public SwerveDriveKinematics getKinematics() {
		return m_kinematics;
	}

	// future changes: maybe leave the modules in the angles remain the same instead
	// of pointint at 0
	public void stop() {
		drive(new ChassisSpeeds(0, 0, 0));
	}

	public boolean getFieldRelative() { // gets if in field relative mode
		return m_fieldRelative;
	}

	public void toggleFieldRelative() { // flips mode of robot
		m_fieldRelative = !m_fieldRelative;
	}
	  
	public void togglePrecisionMode() { //toggles precision mode of robot
		m_precisionMode = !m_precisionMode;
		m_maxVoltage = m_precisionMode ? Constants.PRECISION_MAX_VOLTAGE : Constants.MAX_VOLTAGE;
	}

	// get the swerveModuleState manually
	public SwerveModuleState[] getModuleState() {
		SwerveModuleState[] state = new SwerveModuleState[4];
		for (int i = 0; i < 4; i++) {
			state[i] = new SwerveModuleState(m_swerveModules[i].getDriveVelocity(),
					Rotation2d.fromDegrees(m_swerveModules[i].getSteerAngle()));
		}
		return state;
	}

	@Override
	public void periodic() {
		// Pose2d pose = m_odometry.update(getGyroscopeRotation(), getModuleState());

		// SmartDashboard.putNumber("drivetrain/xposition", pose.getX());
		// SmartDashboard.putNumber("drivetrain/yposition", pose.getY());
		// SmartDashboard.putNumber("drivetrain/heading", pose.getRotation().getDegrees());

		SmartDashboard.putString("drivetrain/driveMode", m_fieldRelative ? "field-centric" : "robot-centric");
	}
}
