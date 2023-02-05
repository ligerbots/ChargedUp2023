// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Vision;

public class ShiftToShoot extends CommandBase {
    private final DriveTrain m_driveTrain;
    private final Vision m_vision;

    public ShiftToShoot(DriveTrain driveTrain, Vision vision) {
        this.m_driveTrain = driveTrain;
        this.m_vision = vision;
        addRequirements(m_driveTrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

    }

}
