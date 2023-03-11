// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.Position;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Vision;

public class AutoBarrierTwoCones extends SequentialCommandGroup implements AutoCommandInterface {
    
    // save the trajectories so we can plot them in the simulation
    AutoFollowTrajectory[] m_traj;

    /** Creates a new AutoBarrierTwoCones */
    public AutoBarrierTwoCones(DriveTrain driveTrain, Arm arm, Vision vision, Claw claw, Position secondConePos, JoystickButton overrideButton) {
        m_traj = new AutoFollowTrajectory[] { new AutoFollowTrajectory(driveTrain, "top_grid_s1"),
                new AutoFollowTrajectory(driveTrain, "top_grid_s2"),
                new AutoFollowTrajectory(driveTrain, "top_grid_s1") };

        addCommands(
            new ScoreArm(arm, driveTrain, Position.LEFT_TOP, overrideButton).withTimeout(5),
            new InstantCommand(claw::open),
            m_traj[0].alongWith(new ScoreArm(arm, driveTrain, Position.PICK_UP, overrideButton).withTimeout(5).andThen(new InstantCommand(claw::startIntake))),
            new InstantCommand(claw::close),
            new ScoreArm(arm, driveTrain, Position.STOW_ARM, overrideButton).withTimeout(5)
            // m_traj[1],
            // new DriveAndMoveArm(arm, driveTrain, vision, secondConePos),
            // new InstantCommand(claw::open),
            // m_traj[2].alongWith(new ScoreArm(arm, driveTrain, Position.STOW_ARM).withTimeout(5))
            );

        // Do NOT require any Subsystems. That is handled by the subcommands.
    }

    @Override
    public Pose2d getInitialPose() {
        return m_traj[0].getInitialPose();
    }

    @Override
    public void plotTrajectory(TrajectoryPlotter plotter) {
        for (int i=0; i<m_traj.length; i++) {
            m_traj[i].plotTrajectory(plotter, i);
        }
    }
}
