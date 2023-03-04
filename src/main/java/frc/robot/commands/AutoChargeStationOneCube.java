// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

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
            new ScoreArm(arm, Position.CENTER_TOP).withTimeout(5),
            new InstantCommand(claw::open),
            m_traj.alongWith(new ScoreArm(arm, Position.STOW_ARM).withTimeout(5).alongWith(new InstantCommand(claw::close))),
            new ChargeStationDrive(driveTrain),
            new ChargeStationBalance(driveTrain));
                    
        // Do NOT require any Subsystems. That is handled by the subcommands.
    }

    @Override
    public Pose2d getInitialPose() {
        return m_traj.getInitialPose();
    }

    @Override
    public void plotTrajectory(TrajectoryPlotter plotter) {
        m_traj.plotTrajectory(plotter);
    }
}
