// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrain;

// NOTE:    Consider using this command inline, rather than writing a subclass.    For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

// routine for auto that drives the robot over the charge station and into the middle area, picks up a cone,
// then drives back onto the charge station
// might be used in other routines
public class DriveOverChargeStation extends SequentialCommandGroup {
    // goal angle for driving onto chargestation from community area
    private static final Rotation2d FRONT_ANGLE_GOAL = Rotation2d.fromDegrees(-10); 
    // goal angle for driving onto chargestation from middle area
    private static final Rotation2d BACK_ANGLE_GOAL = Rotation2d.fromDegrees(10); 
    private static final double ANGLE_DRIVE_MPS = 1.5; 
    private static final double CHARGESTATION_CONE_DISTANCE = 85.13/12; // in feet
    private static final double ROBOT_LENGTH = 3; // in feet // TODO: find actual length of robot

    private DriveTrain m_driveTrain;

    /** Creates a new driveOverChargeStation. */
    public DriveOverChargeStation(DriveTrain driveTrain) {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        m_driveTrain = driveTrain;
        addCommands(
            new AngleDrive(m_driveTrain, ANGLE_DRIVE_MPS, FRONT_ANGLE_GOAL), // drive onto charge station
            new AngleDrive(m_driveTrain, ANGLE_DRIVE_MPS, Rotation2d.fromDegrees(0.0)), // drive over chargestation onto floor
            new DriveDistance(m_driveTrain, ANGLE_DRIVE_MPS, CHARGESTATION_CONE_DISTANCE-ROBOT_LENGTH),
            // TODO command to pick up cone when it is made
            new AngleDrive(m_driveTrain, -ANGLE_DRIVE_MPS, BACK_ANGLE_GOAL), // drive backwards onto charge station
            new ChargeStationBalance(driveTrain)); // balance onto charge station
    }
}
