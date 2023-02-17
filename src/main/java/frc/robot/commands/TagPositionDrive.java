// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Constants.Position;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Vision;

public class TagPositionDrive extends CommandBase {
    private DriveTrain m_driveTrain;
    private Command m_followTrajectory;
    private Vision m_vision;
    private Position m_targetPosition;

    // converted to Transform because cannot add two Poses together
    private static final Map<Position, Pose2d> ROBOT_POSITIONS = new HashMap<Position, Pose2d>() {
        {
            // scoring transformations, change later
            // constant rotation offset of 180 so robot faces opposite direction as Apriltag (they both face each other)
            put(Position.LEFT_TOP, new Pose2d(new Translation2d(2, -1), new Rotation2d(180))); // position1
            put(Position.CENTER_TOP, new Pose2d(new Translation2d(2, 0), new Rotation2d(180)));
            put(Position.RIGHT_TOP, new Pose2d(new Translation2d(2, 1), new Rotation2d(180)));
            put(Position.LEFT_MIDDLE, new Pose2d(new Translation2d(2, -1), new Rotation2d(180)));
            put(Position.CENTER_MIDDLE, new Pose2d(new Translation2d(2, 0), new Rotation2d(180)));
            put(Position.RIGHT_MIDDLE, new Pose2d(new Translation2d(2, 1), new Rotation2d(180)));
            put(Position.LEFT_BOTTOM, new Pose2d(new Translation2d(2, -1), new Rotation2d(180)));
            put(Position.CENTER_BOTTOM, new Pose2d(new Translation2d(2, 0), new Rotation2d(180)));
            put(Position.RIGHT_BOTTOM, new Pose2d(new Translation2d(2, 1), new Rotation2d(180)));
            // substation positions, change later
            put(Position.LEFT_SUBSTATION, new Pose2d(new Translation2d(100, 0), new Rotation2d(180)));
            put(Position.RIGHT_SUBSTATION, new Pose2d(new Translation2d(100, 0), new Rotation2d(180)));
        }
    };

    public TagPositionDrive(DriveTrain driveTrain, Vision vision, Position targetPosition) {
        this.m_driveTrain = driveTrain;
        this.m_vision = vision;
        // pass in a Position ex. pass in Position.LEFT_TOP
        this.m_targetPosition = targetPosition;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // for safety, set command to null
        this.m_followTrajectory = null;

        Optional<Pose2d> centralTagPose = m_vision.getCentralTagPose();
        if (centralTagPose.isEmpty()) {
            return; // return a null, stop command

        }
        // get from the optional if its not null, check if central tag exists
        Pose2d tagPose = centralTagPose.get(); // get AprilTag pose of target ID tag
        System.out.println("Target Tag Pose " + tagPose.toString());

        // this is the transformation we want to translate from the target AprilTag by
        // can do this instead of if checks
        Pose2d robotTransformation = ROBOT_POSITIONS.get(m_targetPosition);

        // to rotate universal coordinates so translation is correct direction
        Translation2d poseOffset = robotTransformation.getTranslation().rotateBy(tagPose.getRotation());
        System.out.println("Pose Offset Translation " + poseOffset.toString());

        // add to get target rotation and translation for robot
        Translation2d robotTargetTranslation = tagPose.getTranslation().plus(poseOffset);
        System.out.println("Robot Target Pose Translation " + robotTargetTranslation.toString());
        Rotation2d robotTargetRotation = tagPose.getRotation().plus(robotTransformation.getRotation());
        System.out.println("Robot Target Pose Rotation " + robotTargetRotation.toString());

        // get robot current pose
        Pose2d currentPose = m_driveTrain.getPose();
        System.out.println("Current Robot Pose " + currentPose.toString());

        // we need a heading for the trajectory
        // use the angle of the straight line between the 2 points
        Rotation2d heading = robotTargetTranslation.minus(currentPose.getTranslation()).getAngle();
        System.out.println("Heading angle " + heading.getDegrees());

        PathPlannerTrajectory traj = PathPlanner.generatePath(new PathConstraints(2.0, 1.0), // velocity, acceleration
                new PathPoint(currentPose.getTranslation(), heading, currentPose.getRotation()), // starting pose
                new PathPoint(robotTargetTranslation, heading, robotTargetRotation) // position, heading
        );
        m_followTrajectory = m_driveTrain.makeFollowTrajectoryCommand(traj);
        m_followTrajectory.schedule();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // if interrupted, stop the follow trajectory
        if (interrupted) {
            m_followTrajectory.cancel();
        }
        m_followTrajectory = null;
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // if the FollowTrajectory commad is null or not scheduled, end
        return m_followTrajectory == null || !m_followTrajectory.isScheduled();
    }
}
