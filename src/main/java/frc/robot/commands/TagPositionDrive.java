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

    //converted to Transform because cannot add two Poses together
    private static final Map<Position, Transform2d> ROBOT_POSITIONS = new HashMap<Position, Transform2d>() {
        {
            // scoring transformations, change later
            put(Position.LEFT_TOP, new Transform2d(new Translation2d(2, -1), new Rotation2d(180))); // position1
            put(Position.CENTER_TOP, new Transform2d(new Translation2d(2, 0), new Rotation2d(180)));
            put(Position.RIGHT_TOP, new Transform2d(new Translation2d(2, 1), new Rotation2d(180)));
            put(Position.LEFT_MIDDLE, new Transform2d(new Translation2d(2, -1), new Rotation2d(180)));
            put(Position.CENTER_MIDDLE, new Transform2d(new Translation2d(2, 0), new Rotation2d(180)));
            put(Position.RIGHT_MIDDLE, new Transform2d(new Translation2d(2, 1), new Rotation2d(180)));
            put(Position.LEFT_BOTTOM, new Transform2d(new Translation2d(2, -1), new Rotation2d(180)));
            put(Position.CENTER_BOTTOM, new Transform2d(new Translation2d(2, 0), new Rotation2d(180)));
            put(Position.RIGHT_BOTTOM, new Transform2d(new Translation2d(2, 1), new Rotation2d(180)));
            // substation positions, change later
            put(Position.LEFT_SUBSTATION, new Transform2d(new Translation2d(100, 0), new Rotation2d(180)));
            put(Position.RIGHT_SUBSTATION, new Transform2d(new Translation2d(100, 0), new Rotation2d(180)));

        }
    };

    public TagPositionDrive(DriveTrain driveTrain, Vision vision, Position targetPosition) {
        this.m_driveTrain = driveTrain;
        this.m_vision = vision;
        //pass in a Position ex. pass in Position.LEFT_TOP
        this.m_targetPosition = targetPosition; 
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        //for safety, set command to null
        this.m_followTrajectory = null;

        Optional<Pose2d> centralTagPose = m_vision.getCentralTagPose();
        if (centralTagPose.isEmpty()) {
            return; // return a null, stop command

        }
        //get from the optional if its not null, check if central tag exists
        Pose2d tagPose = centralTagPose.get(); // get AprilTag pose of target ID tag
        
        //this is the transformation we want to translate from the target AprilTag by
        //can do this instead of if checks
        Transform2d robotTransformation = ROBOT_POSITIONS.get(m_targetPosition);
        
        //because tag is rotated
        Translation2d poseOffset =  robotTransformation.getTranslation().rotateBy(tagPose.getRotation());
        //add to get target rotation and translation for robot     
        Translation2d robotTargetTranslation = tagPose.getTranslation().plus(poseOffset);
        Rotation2d robotTargetRotation = tagPose.getRotation().plus(robotTransformation.getRotation());
        
        // get robot current pose
        Pose2d currentPose = m_driveTrain.getPose(); 
        //robot target position
        Pose2d targetPose = new Pose2d(robotTargetTranslation, robotTargetRotation); 
        PathPlannerTrajectory traj = PathPlanner.generatePath(new PathConstraints(2.0, 1.0), // velocity, acceleration
                new PathPoint(currentPose.getTranslation(), currentPose.getRotation()), // starting pose
                new PathPoint(targetPose.getTranslation(), targetPose.getRotation()) // position, heading
        // always look at same direction
        );
        m_followTrajectory = m_driveTrain.makeFollowTrajectoryCommand(traj);
        m_followTrajectory.schedule();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        //if interrupted, stop the follow trajectory
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
