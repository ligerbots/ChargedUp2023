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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class AutoFollowTrajectory extends CommandBase {

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
		PathPlannerTrajectory m_curTraj;
		if(DriverStation.getAlliance() == Alliance.Red)
			m_curTraj = m_redTrajectory;
		else
			m_curTraj = m_blueTrajectory;
		m_trajFollowCommand = m_driveTrain.makeFollowTrajectoryCommand(m_curTraj);
		CommandScheduler.getInstance().schedule(m_trajFollowCommand);
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return m_trajFollowCommand.isFinished();
	}

	private PathPlannerTrajectory reflectTrajOverCenterLine(PathPlannerTrajectory traj) {
		List<PathPoint> transformedPathPoint = new ArrayList<>();

		for (Trajectory.State s : traj.getStates()) {
			PathPlannerState state = (PathPlannerState) s;

			// Create a new state so that we don't overwrite the original
			// reflect the position over the center line
			// have to use the tranformStateForAlliance because curveRadius and deltaPos are private variable
			PathPlannerState transformedState = PathPlannerTrajectory.transformStateForAlliance(state, Alliance.Red);

			Translation2d transformedTranslation = new Translation2d(
					Constants.CUSTOM_FIELD_LENGTH - state.poseMeters.getX(), state.poseMeters.getY());

			transformedState.poseMeters = new Pose2d(transformedTranslation, transformedState.poseMeters.getRotation());

			transformedPathPoint.add(new PathPoint(transformedState.poseMeters.getTranslation(),
					transformedState.poseMeters.getRotation(), transformedState.holonomicRotation));
		}

		// has to generate the trajectory using pathpoints because the PathPlannerTrajectory constructor is private
		return PathPlanner.generatePath(new PathConstraints(Constants.TRAJ_MAX_VEL, Constants.TRAJ_MAX_ACC), transformedPathPoint);
	}	
}