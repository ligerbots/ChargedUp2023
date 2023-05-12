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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
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
    private double m_trajectoryTime;

    // Dictionary of robot positions for a desired operation
    // This is *relative* to the AprilTag pose, but left/right from the Driver's perspective

    // distance in front of tag. Roughly 36cm + 1/2 of robot length
    private static final double SCORE_OFFSET_X_METERS = 0.90;
    // distance from center to cone pipe
    private static final double SCORE_OFFSET_Y_METERS = Units.inchesToMeters(22.0);

    // left/right offset for pickup at the Substation
    private static final double SUBSTATION_OFFSET_X_METERS = 0.54;
    private static final double SUBSTATION_OFFSET_Y_METERS = 0.7;

    private static final double SUBSTATION_DRIVE_MAX_VELOCITY = 2.0;
    private static final double SUBSTATION_DRIVE_MAX_ACCEL = 2.0;
    
    private static final double SCORE_DRIVE_MAX_VELOCITY = 2.0;
    private static final double SCORE_DRIVE_MAX_ACCEL = 1.0;


    private static final Map<Position, Pose2d> ROBOT_POSITIONS = new HashMap<Position, Pose2d>() {
        {
            // scoring transformations
            // constant rotation offset of 180 so robot faces opposite direction as Apriltag (they both face each other)
            put(Position.LEFT_TOP, new Pose2d(SCORE_OFFSET_X_METERS, SCORE_OFFSET_Y_METERS, Rotation2d.fromDegrees(180)));
            put(Position.CENTER_TOP, new Pose2d(SCORE_OFFSET_X_METERS, 0, Rotation2d.fromDegrees(180)));
            put(Position.RIGHT_TOP, new Pose2d(SCORE_OFFSET_X_METERS, -SCORE_OFFSET_Y_METERS, Rotation2d.fromDegrees(180)));
            put(Position.LEFT_MIDDLE, new Pose2d(SCORE_OFFSET_X_METERS, SCORE_OFFSET_Y_METERS, Rotation2d.fromDegrees(180)));
            put(Position.CENTER_MIDDLE, new Pose2d(SCORE_OFFSET_X_METERS, 0, Rotation2d.fromDegrees(180)));
            put(Position.RIGHT_MIDDLE, new Pose2d(SCORE_OFFSET_X_METERS, -SCORE_OFFSET_Y_METERS, Rotation2d.fromDegrees(180)));
            put(Position.LEFT_BOTTOM, new Pose2d(SCORE_OFFSET_X_METERS, SCORE_OFFSET_Y_METERS, Rotation2d.fromDegrees(180)));
            put(Position.CENTER_BOTTOM, new Pose2d(SCORE_OFFSET_X_METERS, 0, Rotation2d.fromDegrees(180)));
            put(Position.RIGHT_BOTTOM, new Pose2d(SCORE_OFFSET_X_METERS, -SCORE_OFFSET_Y_METERS, Rotation2d.fromDegrees(180)));

            // substation positions
            // NOTE substation left/right is flipped because we are going with the Driver's perspective
            put(Position.LEFT_SUBSTATION, new Pose2d(SUBSTATION_OFFSET_X_METERS, -SUBSTATION_OFFSET_Y_METERS, Rotation2d.fromDegrees(180)));
            put(Position.RIGHT_SUBSTATION, new Pose2d(SUBSTATION_OFFSET_X_METERS, SUBSTATION_OFFSET_Y_METERS, Rotation2d.fromDegrees(180)));
        }
    };

    public TagPositionDrive(DriveTrain driveTrain, Vision vision, Position targetPosition) {
        m_driveTrain = driveTrain;
        m_vision = vision;
        m_targetPosition = targetPosition;

        // The scheduler does not know about the sub-command, so include the requirements
        addRequirements(m_driveTrain);
    }
    public double getTrajectoryTime(){
       return m_trajectoryTime;   
    }
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // for safety, set command to null
        m_followTrajectory = null;

        // keep track if target is substation or grid
        boolean isSubTarget = isSubstationTarget(m_targetPosition);
        
        // getting central tag
        int centralTagId = m_vision.getCentralTagId(isSubTarget);

        Optional<Pose2d> centralTagPose = m_vision.getTagPose(centralTagId);
        if (centralTagPose.isEmpty()) {
            return; // return a null, stop command
        }

        // get from the optional if its not null, check if central tag exists
        Pose2d tagPose = centralTagPose.get(); // get AprilTag pose of target ID tag
        // System.out.println("Target Tag Pose " + tagPose.toString());

        // this is the transformation we want to translate from the target AprilTag by
        // can do this instead of if checks
        Pose2d robotTransformation = ROBOT_POSITIONS.get(m_targetPosition);

        // to rotate universal coordinates so translation is correct direction
        Translation2d poseOffset = robotTransformation.getTranslation().rotateBy(tagPose.getRotation());
        // System.out.println("Pose Offset Translation " + poseOffset.toString());

        // add to get target rotation and translation for robot
        Translation2d robotTargetTranslation = tagPose.getTranslation().plus(poseOffset);
        // System.out.println("Robot Target Pose Translation " + robotTargetTranslation.toString());
        Rotation2d robotTargetRotation = tagPose.getRotation().plus(robotTransformation.getRotation());
        // System.out.println("Robot Target Pose Rotation " + robotTargetRotation.toString());
        // SmartDashboard.putNumber("tagPosDrive/targetX", robotTargetTranslation.getX());
        // SmartDashboard.putNumber("tagPosDrive/targetY", robotTargetTranslation.getY());
        // SmartDashboard.putNumber("tagPosDrive/targetRot",  robotTargetRotation.getDegrees());

        // get robot current pose
        Pose2d currentPose = m_driveTrain.getPose();
        // System.out.println("Current Robot Pose " + currentPose.toString());

        // we need a heading for the trajectory
        // use the angle of the straight line between the 2 points
        Rotation2d heading = robotTargetTranslation.minus(currentPose.getTranslation()).getAngle();
        // System.out.println("Heading angle " + heading.getDegrees());

        double maxVel = SCORE_DRIVE_MAX_VELOCITY;
        double maxAcc = SCORE_DRIVE_MAX_ACCEL;
        if (isSubTarget) {
            maxVel = SUBSTATION_DRIVE_MAX_VELOCITY;
            maxAcc = SUBSTATION_DRIVE_MAX_ACCEL;    
        }

        PathPlannerTrajectory traj = PathPlanner.generatePath(
                new PathConstraints(maxVel, maxAcc), // velocity, acceleration
                new PathPoint(currentPose.getTranslation(), heading, currentPose.getRotation()), // starting pose
                new PathPoint(robotTargetTranslation, heading, robotTargetRotation) // position, heading
        );
        
        m_trajectoryTime = traj.getTotalTimeSeconds();

        m_followTrajectory = m_driveTrain.makeFollowTrajectoryCommand(traj);
        m_followTrajectory.initialize();
        System.out.println("*******tajectory time********* = " + traj.getTotalTimeSeconds());

        // if we are trying to score, switch automatically into precisionMode
        if (! isSubTarget) {
            m_driveTrain.setPrecisionMode(true);
        }
    }

    @Override
    public void execute() {
        if (m_followTrajectory != null)
            m_followTrajectory.execute();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // if interrupted, stop the follow trajectory
        System.out.println("TagPositionDrive end interrupted = " + interrupted);

        if (m_followTrajectory != null)
            m_followTrajectory.end(interrupted);
        m_followTrajectory = null;
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // if the FollowTrajectory commad is null or not scheduled, end
        return m_followTrajectory == null || m_followTrajectory.isFinished();
    }

    private boolean isSubstationTarget(Position pos) {
        return pos == Position.LEFT_SUBSTATION || pos == Position.RIGHT_SUBSTATION;
    }
}