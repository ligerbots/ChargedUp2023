// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.Constants.Position;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Vision;

public class AutoChargeStationOneCube extends SequentialCommandGroup implements AutoCommandInterface {

    AutoFollowTrajectory m_traj;
    /** Creates a new AutoWallTwoCones */
    public AutoChargeStationOneCube(DriveTrain driveTrain, Arm arm, Vision vision, Claw claw) {
        m_traj = new AutoFollowTrajectory(driveTrain, "c_forward_balance");
        addCommands(
            // TODO: use DriveAndMove
            new ScoreArm(arm, driveTrain, Position.CENTER_TOP).withTimeout(5),
            new InstantCommand(claw::open),
            // TODO: wont stow because in the bad region
            new ScoreArm(arm, driveTrain, Position.STOW_ARM).withTimeout(5).alongWith(new InstantCommand(claw::close)),
            new ChargeStationDrive(driveTrain),
            new ChargeStationBalance(driveTrain));
                    
        // Do NOT require any Subsystems. That is handled by the subcommands.
    }

    @Override
    public Pose2d getInitialPose() {
        // TODO Auto-generated method stub
        return m_traj.getInitialPose();
    }

}
