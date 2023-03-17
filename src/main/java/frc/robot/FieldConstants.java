// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

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
    // safeZone x limit on blue side, cannot stow in this zone
    public static final double BAD_ZONE_X_BLUE = 2.0;
    // safeZone x limit on red side, cannot stow in this zone
    public static final double BAD_ZONE_X_RED = 14.6;

    // Center (x) of the Charging Station (in meters)
    public static final double CHARGE_STATION_MIDDLE_X_BLUE = 3.9;
    public static final double CHARGE_STATION_MIDDLE_X_RED = 12.6;

    public static final double FIELD_LENGTH = 16.6;

    // Flip position
    public static Pose2d flipPose(Pose2d pose) {
        Rotation2d rot = pose.getRotation();
        // reflect the pose over center line, flip both the X and the rotation
        Pose2d flippedPose = new Pose2d(FIELD_LENGTH - pose.getX(), pose.getY(), new Rotation2d(-rot.getCos(), rot.getSin()));
        return flippedPose;
    }

}