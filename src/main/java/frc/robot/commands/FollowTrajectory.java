// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Consumer;
import java.util.function.Supplier;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.DriveTrain;

public class FollowTrajectory extends CommandBase {

    private final Timer m_timer = new Timer();
    private final PathPlannerTrajectory m_trajectory;
    private final Supplier<Pose2d> m_pose;
    private final SwerveDriveKinematics m_kinematics;
    private final HolonomicDriveController m_controller;
    private final Consumer<SwerveModuleState[]> m_outputModuleStates;

    DriveTrain m_driveTrain;

    /**
     * Constructs a new PPSwerveControllerCommand that when executed will follow the
     * provided
     * trajectory. This command will not return output voltages but rather raw
     * module states from the
     * position controllers which need to be put into a velocity PID.
     *
     * <p>
     * Note: The controllers will *not* set the outputVolts to zero upon completion
     * of the path-
     * this is left to the user, since it is not appropriate for paths with
     * nonstationary endstates.
     *
     * @param trajectory         The trajectory to follow.
     * @param pose               A function that supplies the robot pose - use one
     *                           of the odometry classes to
     *                           provide this.
     * @param kinematics         The kinematics for the robot drivetrain.
     * @param xController        The Trajectory Tracker PID controller for the
     *                           robot's x position.
     * @param yController        The Trajectory Tracker PID controller for the
     *                           robot's y position.
     * @param thetaController    The Trajectory Tracker PID controller for angle for
     *                           the robot.
     * @param outputModuleStates The raw output module states from the position
     *                           controllers.
     * @param requirements       The subsystems to require.
     */

    // TODO: this is way too many parameters. All of this should be part of the Traj command or DriveTrain
    // Probably should just be robotDrive and trajectory.
    public FollowTrajectory(DriveTrain driveTrain,
            PathPlannerTrajectory trajectory,
            Supplier<Pose2d> pose,
            SwerveDriveKinematics kinematics,
            PIDController xController,
            PIDController yController,
            ProfiledPIDController thetaController,
            Consumer<SwerveModuleState[]> outputModuleStates) {
        m_driveTrain = driveTrain;
        m_trajectory = trajectory;
        m_pose = pose;
        m_kinematics = kinematics;

        m_controller = new HolonomicDriveController(
                xController,
                yController,
                thetaController);

        m_outputModuleStates = outputModuleStates;

        addRequirements(m_driveTrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_timer.reset();
        m_timer.start();

        // clear all robot mode and drive in field-centric at normal speeds
        m_driveTrain.resetDrivingModes();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double curTime = m_timer.get();
        var desiredState = (PathPlannerState) m_trajectory.sample(curTime);

        var targetChassisSpeeds = m_controller.calculate(m_pose.get(), desiredState, desiredState.holonomicRotation);
        var targetModuleStates = m_kinematics.toSwerveModuleStates(targetChassisSpeeds);

        m_outputModuleStates.accept(targetModuleStates);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_timer.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_timer.hasElapsed(m_trajectory.getTotalTimeSeconds());
    }
}
