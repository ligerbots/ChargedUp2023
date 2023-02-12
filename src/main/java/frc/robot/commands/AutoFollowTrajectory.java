// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
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
		// Use addRequirements() here to declare subsystem dependencies.
		m_driveTrain = driveTrain;
		m_blueTrajectory = PathPlanner.loadPath(trajectoryName, Constants.TRAJ_MAX_VEL, Constants.TRAJ_MAX_ACC);
		m_redTrajectory = reflectTrajOverCenterLine(m_blueTrajectory);
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
	public void execute() {}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		if(interrupted){
			m_trajFollowCommand.cancel();
		}
		m_trajFollowCommand = null;
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return m_trajFollowCommand == null || !m_trajFollowCommand.isScheduled();
	}

	private PathPlannerTrajectory reflectTrajOverCenterLine(PathPlannerTrajectory traj) {
		List<PathPoint> transformedPathPoint = new ArrayList<>();

        for (Trajectory.State s : traj.getStates()) {
			PathPlannerState state = (PathPlannerState) s;

            // Create a new state so that we don't overwrite the original
			// reflect the position over the center line

			Translation2d transformedTranslation = new Translation2d(
					Constants.CUSTOM_FIELD_LENGTH - state.poseMeters.getX(), state.poseMeters.getY());

			Rotation2d transformedRotation = state.poseMeters.getRotation().times(-1);

			Rotation2d transformedHolonomicRotation = state.holonomicRotation.times(-1);

			transformedPathPoint
					.add(new PathPoint(transformedTranslation, transformedRotation, transformedHolonomicRotation));
		}

		// has to generate the trajectory using pathpoints because the PathPlannerTrajectory constructor is private
		return PathPlanner.generatePath(new PathConstraints(Constants.TRAJ_MAX_VEL, Constants.TRAJ_MAX_ACC),
				transformedPathPoint);
	}

    @Override
    public Pose2d getInitialPose() {
        // TODO Auto-generated method stub
        if(DriverStation.getAlliance() == Alliance.Red)
            return m_redTrajectory.getInitialPose();
        else 
            return m_blueTrajectory.getInitialPose();
    }

    @Override
    public void plotTrajectory(TrajectoryPlotter plotter) {
        if(DriverStation.getAlliance() == Alliance.Red)
            plotter.plotTrajectory(m_redTrajectory);
        else 
            plotter.plotTrajectory(m_blueTrajectory);
    }
}
