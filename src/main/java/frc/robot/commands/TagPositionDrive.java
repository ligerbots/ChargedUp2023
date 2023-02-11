// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.Position;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Vision;

public class TagPositionDrive extends CommandBase {
    private DriveTrain m_driveTrain;
    private FollowTrajectory m_followTrajectory;
    private Vision m_vision;
    private Position m_targetPosition;


    private static final Map<Position, Pose2d> ROBOT_POSITIONS = new HashMap<Position, Pose2d>() {
        {
            // scoring positions, change later
            put(Position.LEFT_TOP, new Pose2d(new Translation2d(3, -4.5), new Rotation2d(0))); // position1
            put(Position.CENTER_TOP, new Pose2d(new Translation2d(3, 0), new Rotation2d(0)));
            put(Position.RIGHT_TOP, new Pose2d(new Translation2d(3, 4.5), new Rotation2d(0)));
            put(Position.LEFT_MIDDLE, new Pose2d(new Translation2d(3, -4.5), new Rotation2d(0)));
            put(Position.CENTER_MIDDLE, new Pose2d(new Translation2d(3, 0), new Rotation2d(0)));
            put(Position.RIGHT_MIDDLE, new Pose2d(new Translation2d(3, 4.5), new Rotation2d(0)));
            put(Position.LEFT_BOTTOM, new Pose2d(new Translation2d(3, -4.5), new Rotation2d(0)));
            put(Position.CENTER_BOTTOM, new Pose2d(new Translation2d(3, 0), new Rotation2d(0)));
            put(Position.RIGHT_BOTTOM, new Pose2d(new Translation2d(3, 4.5), new Rotation2d(0)));
            // substation positions, change later
            put(Position.LEFT_SUBSTATION, new Pose2d(new Translation2d(100, 0), new Rotation2d(0)));
            put(Position.RIGHT_SUBSTATION, new Pose2d(new Translation2d(100, 0), new Rotation2d(0)));

        }
    };

    /** Creates a new ChargeStationDrive. */
    public TagPositionDrive(DriveTrain driveTrain, Vision vision, Position targetPosition) {
        this.m_driveTrain = driveTrain;
        this.m_vision = vision;
        this.m_targetPosition = targetPosition; 
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_driveTrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Optional<Pose2d> centralTagPose = m_vision.getCentralTagPose();
        if (centralTagPose.isEmpty()) {
            return; // return a null, stop this

        }
        Pose2d tagPose = centralTagPose.get(); // get AprilTag pose of target ID tag
        
        //this is the pose we want to translate from the target AprilTag by
        //a relative pose, not coordinates
        //can do this instead of if checks
        Pose2d relativePose = ROBOT_POSITIONS.get(m_targetPosition);
        
        //should I only add translation, or should I also add rotation?
        Pose2d trajectoryPose = tagPose.plus(relativePose);
        
        /* uneccessary if I pass the target Position into the command

        Translation2d translated = new Translation2d(0, 0); // temp holder
        if (shiftLeft) { // if aiming for left
            // make new translation to go to left of tag
            translated = tagPose.getTranslation().plus(Constants.CONE_LEFT_TRANSLATION);
        } else if (shiftRight) { // if aiming for right
            // make new translation to go right of tag
            translated = tagPose.getTranslation().plus(Constants.CONE_RIGHT_TRANSLATION);

        } else { // if aiming for center
                 // make new translation to go to middle of tag
            translated = tagPose.getTranslation().plus(Constants.CUBE_TRANSLATION);
        }
        return Optional.of(new Pose2d(translated, tagPose.getRotation()));
        */
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_driveTrain.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // stops when robot is on ramp of charge station
    }
}
