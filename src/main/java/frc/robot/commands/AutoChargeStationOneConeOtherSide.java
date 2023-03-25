// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
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

public class AutoChargeStationOneConeOtherSide extends SequentialCommandGroup implements AutoCommandInterface {
    final Pose2d m_initialPoseBlue;

    /** Creates a new AutoChargeStationOneCube */
    public AutoChargeStationOneConeOtherSide(DriveTrain driveTrain, Arm arm, Vision vision, Claw claw, boolean wallPosition, JoystickButton overrideButton) {
        // barrier cone position
        double initY = FieldConstants.CHARGE_STATION_CENTER_Y + Units.inchesToMeters(22.0);
        if (wallPosition) {
            // wall cone position
            initY = FieldConstants.CHARGE_STATION_CENTER_Y - Units.inchesToMeters(22.0);
        }

        m_initialPoseBlue = new Pose2d(2.0, initY, Rotation2d.fromDegrees(180));

        addCommands(
            new InstantCommand(arm::retractArm),
            new WaitCommand(0.1),

            new ScoreArm(arm, driveTrain, Position.LEFT_TOP, overrideButton).withTimeout(5),
            new InstantCommand(claw::open),
            new WaitCommand(0.25),
            
            // The robot is already outside the danger zone for stowing arm if we try to score a cone 
            new ScoreArm(arm, driveTrain, Position.STOW_ARM, overrideButton).withTimeout(2).alongWith(new InstantCommand(claw::close)),
        
            // TODO: AutoXPositionDrive seems to care about robot heading based on the setup of alliance color instead of current rotation (-90, 90)
            // Drive over the CS to out of the Community 
            new AutoXPositionDrive(driveTrain, FieldConstants.CENTER_AUTO_OUTSIDE_COMMUNITY_X_BLUE, DriveTrain.CHARGE_STATION_DRIVE_MPS),

            // driveTrain.driveToPoseTrajectoryCommand(FieldConstants.GAME_PIECE_MIDDLE_1_BLUE),
            // TODO: to add start intaking and stuff

            // Drive back to the center of the CS
            new ChargeStationDrive(driveTrain),
            new ChargeStationBalance(driveTrain));
                    
        // Do NOT require any Subsystems. That is handled by the subcommands.
    }

    @Override
    public Pose2d getInitialPose() {
        return FieldConstants.flipPose(m_initialPoseBlue);
    }
}
