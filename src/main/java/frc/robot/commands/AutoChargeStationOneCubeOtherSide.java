// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
    final Pose2d m_initialPoseBlue;

    /** Creates a new AutoChargeStationOneCubeOtherSide */
    public AutoChargeStationOneCubeOtherSide(DriveTrain driveTrain, Arm arm, Vision vision, Claw claw, JoystickButton overrideButton) {
        m_initialPoseBlue = new Pose2d(2.0, FieldConstants.CHARGE_STATION_CENTER_Y, Rotation2d.fromDegrees(180));

        addCommands(
            new InstantCommand(arm::retractArm),
            new WaitCommand(0.1),

            new ScoreArm(arm, driveTrain, Position.CENTER_TOP, overrideButton).withTimeout(5),

            new InstantCommand(claw::open),
            new WaitCommand(0.25),

            // We start outside the danger zone, so we are fine. 
            new ScoreArm(arm, driveTrain, Position.STOW_ARM, overrideButton).withTimeout(2).alongWith(new InstantCommand(claw::close)),
            
            // Drive over the CS to out of the Community 
            new AutoXPositionDrive(driveTrain, FieldConstants.CENTER_AUTO_OUTSIDE_COMMUNITY_X_BLUE, DriveTrain.CHARGE_STATION_DRIVE_MPS),

            new ChargeStationDrive(driveTrain),
            new ChargeStationBalance(driveTrain));
                    
        // Do NOT require any Subsystems. That is handled by the subcommands.
    }

    @Override
    public Pose2d getInitialPose() {
        return FieldConstants.flipPose(m_initialPoseBlue);
    }
}
