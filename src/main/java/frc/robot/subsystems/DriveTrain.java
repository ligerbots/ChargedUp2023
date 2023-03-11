// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.filter.SlewRateLimiter;

import frc.robot.Constants;
import frc.robot.commands.AutoCommandInterface;
import frc.robot.commands.FollowTrajectory;
import frc.robot.subsystems.DriveTrain;

import frc.robot.swerve.*;

public class DriveTrain extends SubsystemBase {
    /**
     * The left-to-right distance between the drivetrain wheels
     *
     * Should be measured from center to center.
     */
    private static final double TRACKWIDTH_METERS = Units.inchesToMeters(19.5625);

    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    private static final double WHEELBASE_METERS = Units.inchesToMeters(24.625);

    // P constants for controllin during trajectory following
    private static final double X_PID_CONTROLLER_P = 2.0;
    private static final double Y_PID_CONTROLLER_P = 2.0;
    private static final double THETA_PID_CONTROLLER_P = 4.0;

    // speed used to drive onto/over the ChargeStation
    public static final double CHARGE_STATION_DRIVE_MPS = 2.0;

    // the max velocity for drivetrain
    // adjusted when in precision driving mode
    private double m_maxVelocity = MAX_VELOCITY_METERS_PER_SECOND;

    private double m_maxAngularVelocity = MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;

    // if true, then robot is in field centric mode
    private boolean m_fieldCentric = true;

    // if true, then robot is in precision mode
    private boolean m_precisionMode = false;

    // limit the acceleration from 0 to full power to take 1/3 second.
    private SlewRateLimiter m_xLimiter = new SlewRateLimiter(3);
    private SlewRateLimiter m_yLimiter = new SlewRateLimiter(3);
    private SlewRateLimiter m_rotationLimiter = new SlewRateLimiter(3);

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
     * This is a measure of how fast the robot should be able to drive in a straight line.
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
            Math.hypot(TRACKWIDTH_METERS / 2.0, WHEELBASE_METERS / 2.0);

    private static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND_PRECISION_MODE = 
            MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND / 6.0;

    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
            // Front left
            new Translation2d(TRACKWIDTH_METERS / 2.0, WHEELBASE_METERS / 2.0),
            // Front right
            new Translation2d(TRACKWIDTH_METERS / 2.0, -WHEELBASE_METERS / 2.0),
            // Back left
            new Translation2d(-TRACKWIDTH_METERS / 2.0, WHEELBASE_METERS / 2.0),
            // Back right
            new Translation2d(-TRACKWIDTH_METERS / 2.0, -WHEELBASE_METERS / 2.0));

    // NavX connected over MXP
    private final AHRS m_navx = new AHRS(Port.kMXP, (byte) 200);

    // These are our modules. We initialize them in the constructor.
    private final SwerveModule[] m_swerveModules = new SwerveModule[4];

    // the odometry class to keep track of where the robot is on the field
    private final SwerveDrivePoseEstimator m_odometry;

    private final Vision m_vision;

    private final Field2d m_field = new Field2d();

    // PID controller for swerve
    private final PIDController m_xController = new PIDController(X_PID_CONTROLLER_P, 0, 0);
    private final PIDController m_yController = new PIDController(Y_PID_CONTROLLER_P, 0, 0);
    private final ProfiledPIDController m_thetaController = new ProfiledPIDController(THETA_PID_CONTROLLER_P,
            0, 0,
            new TrapezoidProfile.Constraints(4 * Math.PI, 4 * Math.PI));

    public DriveTrain(Vision vision) {

        m_swerveModules[0] = new SwerveModule("frontLeft",
                new frc.robot.swerve.NeoDriveController(Constants.FRONT_LEFT_MODULE_DRIVE_MOTOR),
                new frc.robot.swerve.NeoSteerController(Constants.FRONT_LEFT_MODULE_STEER_MOTOR,
                        Constants.FRONT_LEFT_MODULE_STEER_ENCODER, Constants.FRONT_LEFT_MODULE_STEER_OFFSET));

        m_swerveModules[1] = new frc.robot.swerve.SwerveModule("frontRight",
                new frc.robot.swerve.NeoDriveController(Constants.FRONT_RIGHT_MODULE_DRIVE_MOTOR),
                new frc.robot.swerve.NeoSteerController(Constants.FRONT_RIGHT_MODULE_STEER_MOTOR,
                        Constants.FRONT_RIGHT_MODULE_STEER_ENCODER, Constants.FRONT_RIGHT_MODULE_STEER_OFFSET));

        m_swerveModules[2] = new frc.robot.swerve.SwerveModule("backLeft",
                new frc.robot.swerve.NeoDriveController(Constants.BACK_LEFT_MODULE_DRIVE_MOTOR),
                new frc.robot.swerve.NeoSteerController(Constants.BACK_LEFT_MODULE_STEER_MOTOR,
                        Constants.BACK_LEFT_MODULE_STEER_ENCODER, Constants.BACK_LEFT_MODULE_STEER_OFFSET));

        m_swerveModules[3] = new frc.robot.swerve.SwerveModule("backRight",
                new frc.robot.swerve.NeoDriveController(Constants.BACK_RIGHT_MODULE_DRIVE_MOTOR),
                new frc.robot.swerve.NeoSteerController(Constants.BACK_RIGHT_MODULE_STEER_MOTOR,
                        Constants.BACK_RIGHT_MODULE_STEER_ENCODER, Constants.BACK_RIGHT_MODULE_STEER_OFFSET));

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

        SmartDashboard.putData("Field", m_field);
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
            return Rotation2d.fromDegrees(360.0 - m_navx.getFusedHeading());
        }

		// We have to invert the angle of the NavX so that rotating the robot
		// counter-clockwise makes the angle increase.
		return Rotation2d.fromDegrees(360.0 - m_navx.getYaw());
	}
	// know the robot heading and get pitch and roll
    private Translation3d getNormalVector3d() {
		Rotation3d tilt = new Rotation3d(Units.degreesToRadians(m_navx.getRoll()), Units.degreesToRadians(m_navx.getPitch()), 0);
        return new Translation3d(0, 0, 1).rotateBy(tilt);
		
    }

    // know how much it tilted, so we know if it's balance on the ramp
    public double getTiltDegrees() {
        return Math.toDegrees(Math.acos(getNormalVector3d().getZ()));
    }

    public Rotation2d getTiltDirection() {
        return new Rotation2d(getNormalVector3d().getX(), getNormalVector3d().getY());
    }

    public void joystickDrive(double inputX, double inputY, double inputRotation) {
        // apply SlewLimiters to the joystick values to control acceleration
        double newInputX = m_xLimiter.calculate(inputX);
        double newInputY = m_yLimiter.calculate(inputY);
        double newInputRotation = m_rotationLimiter.calculate(inputRotation);

        // prevents a drive call with parameters of 0 0 0
        if (Math.abs(newInputX) < 0.01 && Math.abs(newInputY) < 0.01 && Math.abs(newInputRotation) < 0.01){
            stop();
            return;
        } 

        ChassisSpeeds chassisSpeeds;
        // when in field-relative mode
        if (m_fieldCentric) {
            // if we are Red, field-cenric points the other way in absolute coordinates
            // this is equivalent to flipping the X and Y joysticks
            double redFlip = (DriverStation.getAlliance() == Alliance.Red) ? -1.0 : 1.0;

            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    redFlip * newInputX * m_maxVelocity,
                    redFlip * newInputY * m_maxVelocity,
                    newInputRotation * m_maxAngularVelocity,
                    getHeading());
        }
        // when in robot-centric mode
        else {
            chassisSpeeds = new ChassisSpeeds(
                    newInputX * m_maxVelocity,
                    newInputY * m_maxVelocity,
                    newInputRotation * m_maxAngularVelocity);
        }

        drive(chassisSpeeds);
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        // // for debugging
        // SmartDashboard.putNumber("drivetrain/chassisX", chassisSpeeds.vxMetersPerSecond);
        // SmartDashboard.putNumber("drivetrain/chassisY", chassisSpeeds.vyMetersPerSecond);
        // SmartDashboard.putNumber("drivetrain/chassisAngle", Units.radiansToDegrees(chassisSpeeds.omegaRadiansPerSecond));

        SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);
        for (int i = 0; i < 4; i++) {
            m_swerveModules[i].set(
                    states[i].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * Constants.MAX_VOLTAGE,
                    states[i].angle.getRadians());
        }
    }

    public void stop() {
        for (int i = 0; i < 4; i++) {
            m_swerveModules[i].stopWheel();
        }
    }

    // for the beginning of auto rountines
    public void resetDrivingModes() {
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
        m_maxAngularVelocity = m_precisionMode ? MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND_PRECISION_MODE
                : MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
    }

    // lock wheels in x position to resist pushing
    public void lockWheels() {
        double lockRadians = Math.toRadians(45);
        m_swerveModules[0].set(0.0, lockRadians);
        m_swerveModules[1].set(0.0, -lockRadians);
        m_swerveModules[2].set(0.0, -lockRadians);
        m_swerveModules[3].set(0.0, lockRadians);
    }

    public Rotation2d getPitch() {
        // gets pitch of robot
        return Rotation2d.fromDegrees(m_navx.getPitch());
    }

    public Rotation2d getYaw() {
        // gets yaw of robot
        return Rotation2d.fromDegrees(m_navx.getYaw());
    }

    public Rotation2d getRoll() {
        // gets roll of robot
        return Rotation2d.fromDegrees(m_navx.getRoll());
    }

    public Field2d getField2d(){
        return m_field;
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
        m_odometry.update(getGyroscopeRotation(), getModulePositions());

        // Have the vision system update based on the Apriltags, if seen
        m_vision.updateOdometry(m_odometry);

        // Pose2d pose = m_odometry.getEstimatedPosition();
        // SmartDashboard.putNumber("drivetrain/xPosition", pose.getX());
        // SmartDashboard.putNumber("drivetrain/yPosition", pose.getY());
        // SmartDashboard.putNumber("drivetrain/heading", pose.getRotation().getDegrees());
        // SmartDashboard.putNumber("drivetrain/gyro", getGyroscopeRotation().getDegrees());

        // // SmartDashboard.putNumber("drivetrain/pitch", getPitch().getDegrees());
        // // SmartDashboard.putNumber(""drivetrain/roll", getRoll().getDegrees());
        // // SmartDashboard.putNumber("drivetrain/yaw", getYaw().getDegrees());

        // SmartDashboard.putBoolean("drivetrain/fieldCentric", m_fieldCentric);

        // for (SwerveModule mod : m_swerveModules) {
        //     mod.updateSmartDashboard();
        // }
    }

    // Make a command to follow a given trajectory
    // Note this does NOT include stopping at the end
    public Command makeFollowTrajectoryCommand(PathPlannerTrajectory traj) {
        return new FollowTrajectory(
                this,
                traj,
                () -> this.getPose(),
                m_kinematics,
                m_xController,
                m_yController,
                m_thetaController,
                (states) -> {
                    this.drive(m_kinematics.toChassisSpeeds(states));
                });
    }

    public AutoCommandInterface getTrajectoryFollowingCommand(String string) {
        return null;
    }
}
