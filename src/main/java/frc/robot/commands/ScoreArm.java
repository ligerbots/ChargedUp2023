// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.Position;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreArm extends SequentialCommandGroup {
  public 
  
  private static final Map<Position, Map.Entry<Double, Double>> SCORE_POSITIONS = new HashMap<Position, Map.Entry<Double, Double>>() {
    {
        // scoring transformations
        put(Position.LEFT_TOP, new);
        put(Position.CENTER_TOP, new Pose2d(SCORE_OFFSET_X_METERS, 0, Rotation2d.fromDegrees(180)));
        put(Position.RIGHT_TOP, new Pose2d(SCORE_OFFSET_X_METERS, -SCORE_OFFSET_Y_METERS, Rotation2d.fromDegrees(180)));
        put(Position.LEFT_MIDDLE, new Pose2d(SCORE_OFFSET_X_METERS, SCORE_OFFSET_Y_METERS, Rotation2d.fromDegrees(180)));
        put(Position.CENTER_MIDDLE, new Pose2d(SCORE_OFFSET_X_METERS, 0, Rotation2d.fromDegrees(180)));
        put(Position.RIGHT_MIDDLE, new Pose2d(SCORE_OFFSET_X_METERS, -SCORE_OFFSET_Y_METERS, Rotation2d.fromDegrees(180)));
        put(Position.LEFT_BOTTOM, new Pose2d(SCORE_OFFSET_X_METERS, SCORE_OFFSET_Y_METERS, Rotation2d.fromDegrees(180)));
        put(Position.CENTER_BOTTOM, 
        new Pose2d(SCORE_OFFSET_X_METERS, 0, Rotation2d.fromDegrees(180)));
        put(Position.RIGHT_BOTTOM, new Pose2d(SCORE_OFFSET_X_METERS, -SCORE_OFFSET_Y_METERS, Rotation2d.fromDegrees(180)));
        // substation positions, change later
        // NOTE substation left/right is flipped because we are going with the Driver's perspective
        put(Position.LEFT_SUBSTATION, new Pose2d(1, -1, Rotation2d.fromDegrees(180)));
        put(Position.RIGHT_SUBSTATION, new Pose2d(1, 1, Rotation2d.fromDegrees(180)));
    }
  /** Creates a new ScoreArm. */
  public ScoreArm() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands();
  }
}
