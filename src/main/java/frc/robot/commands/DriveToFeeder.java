package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.DriveTrain;
import frc.robot.Constants.Position;
import frc.robot.subsystems.Vision;

public class DriveToFeeder extends CommandBase {
    Timer m_timer = new Timer();
    TagPositionDrive m_driveCommand;
    double m_waitTime;
    static final double MAX_ARM_TIME = 1.2;

    public DriveToFeeder(DriveTrain driveTrain, Vision vision, Position targetPosition) {
        m_driveCommand = new TagPositionDrive(driveTrain, vision, targetPosition);
        addRequirements(driveTrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_driveCommand.initialize();
        double trajTotalTime = m_driveCommand.getTrajectoryTime();
        m_waitTime = Math.max(0.0, MAX_ARM_TIME - trajTotalTime);
        m_timer.reset();
        m_timer.start();
    }

    @Override
    public void execute() {
        if (m_timer.hasElapsed(m_waitTime)){
            m_driveCommand.execute();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // if interrupted, stop the follow trajectory
        m_driveCommand.end(interrupted);
        m_timer.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // if the FollowTrajectory commad is null or not scheduled, end
        return m_timer.hasElapsed(m_waitTime) && m_driveCommand.isFinished();
    }
}
