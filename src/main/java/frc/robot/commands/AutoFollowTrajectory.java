// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class AutoFollowTrajectory extends CommandBase implements AutoCommandInterface {

    private final DriveTrain m_driveTrain;
    private final PathPlannerTrajectory m_blueTrajectory, m_redTrajectory;
    private Command m_trajFollowCommand;

    /** Creates a new AutoFollowTrajectory. */
    public AutoFollowTrajectory(DriveTrain driveTrain, String trajectoryName) {
        m_driveTrain = driveTrain;

        m_blueTrajectory = PathPlanner.loadPath(trajectoryName + "_blue", Constants.TRAJ_MAX_VEL,
                Constants.TRAJ_MAX_ACC);
        m_redTrajectory = PathPlanner.loadPath(trajectoryName + "_red", Constants.TRAJ_MAX_VEL,
                Constants.TRAJ_MAX_ACC);

        // Do NOT require the Drivetrain. That is handled by the m_trajFollowCommand subcommand.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        PathPlannerTrajectory curTraj;
        if (DriverStation.getAlliance() == Alliance.Red)
            curTraj = m_redTrajectory;
        else
            curTraj = m_blueTrajectory;
        m_trajFollowCommand = m_driveTrain.makeFollowTrajectoryCommand(curTraj);
        CommandScheduler.getInstance().schedule(m_trajFollowCommand);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            m_trajFollowCommand.cancel();
        }
        m_trajFollowCommand = null;
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_trajFollowCommand == null || !m_trajFollowCommand.isScheduled();
    }

    @Override
    public Pose2d getInitialPose() {
        // Use the Holonmic pose to get the correct robot heading
        if (DriverStation.getAlliance() == Alliance.Red)
            return m_redTrajectory.getInitialHolonomicPose();
        else
            return m_blueTrajectory.getInitialHolonomicPose();
    }

    // get the end pose with holomonic rotation
    public Pose2d getEndPose() {
        // Use the Holonmic pose to get the correct robot heading
        if (DriverStation.getAlliance() == Alliance.Red){
            PathPlannerState state = m_redTrajectory.getEndState();
            return new Pose2d(state.poseMeters.getTranslation(), state.holonomicRotation);
        }else{
            PathPlannerState state = m_blueTrajectory.getEndState();
            return new Pose2d(state.poseMeters.getTranslation(), state.holonomicRotation);
        }
    }

    @Override
    public void plotTrajectory(TrajectoryPlotter plotter) {
        plotTrajectory(plotter, 0);
    }

    public void plotTrajectory(TrajectoryPlotter plotter, int index) {
        if (DriverStation.getAlliance() == Alliance.Red)
            plotter.plotTrajectory(index, m_redTrajectory);
        else
            plotter.plotTrajectory(index, m_blueTrajectory);
    }
}
