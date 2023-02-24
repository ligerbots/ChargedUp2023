// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoBottomGrid extends SequentialCommandGroup implements AutoCommandInterface {
    /** Creates a new AutoBottomGrid. */
    AutoFollowTrajectory[] m_traj;
    public AutoBottomGrid(DriveTrain driveTrain) {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addRequirements(driveTrain);
        AutoFollowTrajectory[] traj = { new AutoFollowTrajectory(driveTrain, "bot_grid_s1"),
                new AutoFollowTrajectory(driveTrain, "bot_grid_s2"),
                new AutoFollowTrajectory(driveTrain, "bot_grid_s3") };
        m_traj = traj;
        addCommands(traj[0], new WaitCommand(1.0), traj[1], new WaitCommand(1.0), traj[2]);
    }

    @Override
    public Pose2d getInitialPose() {
        // TODO Auto-generated method stub
        return m_traj[0].getInitialPose();
    }

    @Override
    public void plotTrajectory(TrajectoryPlotter plotter) {
        for(var traj : m_traj){
            traj.plotTrajectory(plotter);
        }
    }

}
