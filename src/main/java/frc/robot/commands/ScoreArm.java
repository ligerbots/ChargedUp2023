// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.Position;
import frc.robot.subsystems.Arm;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreArm extends SequentialCommandGroup {

    private static final Map<Position, Pair<Double, Double>> SCORE_POSITIONS = new HashMap<Position, Pair<Double, Double>>(){
        {
            // scoring arm length and angle (angle, length)
            put(Position.LEFT_TOP, new Pair<>(Constants.HIGH_GRID_CONE_ARM_ANGLE, Constants.HIGH_GRID_CONE_ARM_LENGTH));
            put(Position.CENTER_TOP, new Pair<>(Constants.HIGH_GRID_CUBE_ARM_ANGLE, Constants.HIGH_GRID_CUBE_ARM_LENGTH));
            put(Position.RIGHT_TOP, new Pair<>(Constants.HIGH_GRID_CONE_ARM_ANGLE, Constants.HIGH_GRID_CONE_ARM_LENGTH));

            put(Position.LEFT_MIDDLE, new Pair<>(Constants.MIDDLE_GRID_CONE_ARM_ANGLE, Constants.MIDDLE_GRID_CONE_ARM_LENGTH));
            put(Position.CENTER_MIDDLE, new Pair<>(Constants.MIDDLE_GRID_CUBE_ARM_ANGLE, Constants.MIDDLE_GRID_CUBE_ARM_LENGTH));
            put(Position.RIGHT_MIDDLE, new Pair<>(Constants.MIDDLE_GRID_CONE_ARM_ANGLE, Constants.MIDDLE_GRID_CONE_ARM_LENGTH));

            put(Position.LEFT_BOTTOM, new Pair<>(Constants.LOW_GIRD_ARM_ANGLE, Constants.LOW_GIRD_ARM_LENGTH));
            put(Position.CENTER_BOTTOM, new Pair<>(Constants.LOW_GIRD_ARM_ANGLE, Constants.LOW_GIRD_ARM_LENGTH));
            put(Position.RIGHT_BOTTOM, new Pair<>(Constants.LOW_GIRD_ARM_ANGLE, Constants.LOW_GIRD_ARM_LENGTH));
            // substation positions, change later
            // NOTE substation left/right is flipped because we are going with the Driver's perspective
            
            //TODO: get constants for the substations
            put(Position.LEFT_SUBSTATION, new Pair<>(Constants.HIGH_GRID_CONE_ARM_ANGLE, Constants.HIGH_GRID_CONE_ARM_LENGTH));
            put(Position.RIGHT_SUBSTATION, new Pair<>(Constants.HIGH_GRID_CONE_ARM_ANGLE, Constants.HIGH_GRID_CONE_ARM_LENGTH));
        }
    };

    /** Creates a new ScoreArm. */
    public ScoreArm(Arm arm, Constants.Position position) {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        var desiredArmPos = SCORE_POSITIONS.get(position);
        addCommands(
            new SetArmAngle(arm, desiredArmPos.getFirst()),
            new SetArmLength(arm, desiredArmPos.getSecond())
        );
    }
}
