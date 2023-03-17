// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.Position;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Vision;

public class AutoChargeStationOneConeOtherSide extends SequentialCommandGroup implements AutoCommandInterface {

    // private final double BACKING_MPS = 0.5;

    /** Creates a new AutoChargeStationOneCube */
    public AutoChargeStationOneConeOtherSide(DriveTrain driveTrain, Arm arm, Vision vision, Claw claw, JoystickButton overrideButton) {
        
        addCommands(
            new ScoreArm(arm, driveTrain, Position.LEFT_TOP, overrideButton).withTimeout(5),
            new InstantCommand(claw::open),
            new WaitCommand(0.25),
            
            new ScoreArm(arm, driveTrain, Position.STOW_ARM, overrideButton).withTimeout(2).alongWith(new InstantCommand(claw::close)),
            
            // new AutoXPositionDrive(driveTrain, m_traj.getEndPose().getX(), DriveTrain.CHARGE_STATION_DRIVE_MPS),

            new ChargeStationDrive(driveTrain),
            new ChargeStationBalance(driveTrain));
                    
        // Do NOT require any Subsystems. That is handled by the subcommands.
    }
}
