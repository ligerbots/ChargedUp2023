// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.FieldConstants;
import frc.robot.Constants.Position;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Vision;

public class AutoChargeStationOneCubeOtherSide extends SequentialCommandGroup implements AutoCommandInterface {

    private final double BACKING_MPS = 0.5;

    private final Pose2d INITIAL_POSE = new Pose2d(2.20, 2.74, Rotation2d.fromDegrees(180.0));

    /** Creates a new AutoChargeStationOneCube */
    public AutoChargeStationOneCubeOtherSide(DriveTrain driveTrain, Arm arm, Vision vision, Claw claw, JoystickButton overrideButton) {
        
        addCommands(
            new ScoreArm(arm, driveTrain, Position.CENTER_TOP, overrideButton).withTimeout(5),
            // drive to the correct position
            new TagPositionDrive(driveTrain, vision, Position.CENTER_TOP),

            // new PrintCommand("Step 2 of AutoCS"),
            new InstantCommand(claw::open),
            new WaitCommand(0.25),

            // back up to stow the arm
            new AutoXPositionDrive(driveTrain, getInitialPose().getX(), BACKING_MPS),
            
            new ScoreArm(arm, driveTrain, Position.STOW_ARM, overrideButton).withTimeout(2).alongWith(new InstantCommand(claw::close)),
            
            new AutoXPositionDrive(driveTrain, FieldConstants.flipX(FieldConstants.CHARGE_STATION_STOP_X), DriveTrain.CHARGE_STATION_DRIVE_MPS),

            new ChargeStationDrive(driveTrain),
            new ChargeStationBalance(driveTrain));
                    
        // Do NOT require any Subsystems. That is handled by the subcommands.
    }

    @Override
    public Pose2d getInitialPose() {
        return FieldConstants.flipPose(INITIAL_POSE);
    }
}
