package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveTrain;

public class ResetHeading extends InstantCommand {
    private final DriveTrain m_driveTrain;

    public ResetHeading(DriveTrain driveTrain) {
        m_driveTrain = driveTrain;
    }

    @Override
    public void initialize() {
        m_driveTrain.resetHeading();
    }
}
