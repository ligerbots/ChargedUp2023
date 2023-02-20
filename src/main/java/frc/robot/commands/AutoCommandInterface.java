package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public interface AutoCommandInterface extends Command {
    public Pose2d getInitialPose();

    public default void plotTrajectory(TrajectoryPlotter plotter) {
        plotter.clear();
    }
}