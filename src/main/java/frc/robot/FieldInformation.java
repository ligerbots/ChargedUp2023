package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class FieldInformation {
    public static final double LEFT_GRID_FRONT = Units.inchesToMeters(56.25);

    // TODO: need to consider the robot dimension, including the bumbpers
    public static final Translation2d LEFT_GRID_START = new Translation2d(1.071626 - Units.inchesToMeters(23.25), LEFT_GRID_FRONT);
    public static final Translation2d MID_GRID_START = new Translation2d(2.748026 - Units.inchesToMeters(23.25), LEFT_GRID_FRONT);
    public static final Translation2d RIGHT_GRID_START = new Translation2d(4.424426 + Units.inchesToMeters(23.25), LEFT_GRID_FRONT);

    public static final Translation2d GAME_PIECE_4 = new Translation2d(LEFT_GRID_FRONT + Units.inchesToMeters(224), Units.inchesToMeters(36.25));
    public static final Translation2d GAME_PIECE_3 = GAME_PIECE_4.plus(new Translation2d(0, Units.inchesToMeters(48)));
    public static final Translation2d GAME_PIECE_2 = GAME_PIECE_3.plus(new Translation2d(0, Units.inchesToMeters(48)));
    public static final Translation2d GAME_PIECE_1 = GAME_PIECE_2.plus(new Translation2d(0, Units.inchesToMeters(48)));

}