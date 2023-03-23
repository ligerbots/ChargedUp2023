// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes) wherever the constants are needed, to
 * reduce verbosity.
 */
public final class FieldConstants {
    // Field size, copied from AprilTag layout
    public static final double FIELD_LENGTH = 16.54175;
    public static final double FIELD_WIDTH = 8.0137;    
    
    // safeZone x limit on blue side, cannot stow in this zone
    public static final double BAD_ZONE_X_BLUE = 1.8;
    // safeZone x limit on red side, cannot stow in this zone
    public static final double BAD_ZONE_X_RED = FIELD_LENGTH - BAD_ZONE_X_BLUE;

    // Center (x) of the Charging Station (in meters)
    public static final double CHARGE_STATION_MIDDLE_X_BLUE = 3.87;
    //public static final double CHARGE_STATION_MIDDLE_X_RED = FIELD_LENGTH - CHARGE_STATION_MIDDLE_X_BLUE;
    public static final double CHARGE_STATION_CENTER_Y = 2.75;

    // x position outside Community for auto
    public static final double CENTER_AUTO_OUTSIDE_COMMUNITY_X_BLUE = 5.8;

    public static final Pose2d GAME_PIECE_MIDDLE_1_BLUE = new Pose2d(7.07, 3.36, Rotation2d.fromDegrees(0.0));
    public static final Pose2d GAME_PIECE_MIDDLE_2_BLUE = new Pose2d(7.07, 2.14, Rotation2d.fromDegrees(0.0));
    
    // Flip position
    public static Pose2d flipPose(Pose2d pose) {
        if (DriverStation.getAlliance() == Alliance.Blue)
            return pose;

        Rotation2d rot = pose.getRotation();
        // reflect the pose over center line, flip both the X and the rotation
        return new Pose2d(FIELD_LENGTH - pose.getX(), pose.getY(), new Rotation2d(-rot.getCos(), rot.getSin()));
    }

    public static double flipX(double x){
        if (DriverStation.getAlliance() == Alliance.Blue)
            return x;
        return FIELD_LENGTH - x;
    }
}
