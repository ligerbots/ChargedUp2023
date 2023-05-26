// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.CubeShooterSpeed;
import frc.robot.subsystems.CubeShooter;

public class ShootCube extends CommandBase {
    private CubeShooter m_cubeShooter;
    private CubeShooterSpeed m_cubeShooterSpeed;

    // speeds of shooter, change later
    private static final Map<CubeShooterSpeed, Double> SHOOTER_SPEEDS = new HashMap<CubeShooterSpeed, Double>() {
        {
            // speeds are 0->1  
            //TODO Tune these
            put(CubeShooterSpeed.LOW, 0.2);
            put(CubeShooterSpeed.MIDDLE, 0.3);
            put(CubeShooterSpeed.HIGH, 0.5);
        }
    };
    
    public ShootCube(CubeShooter cubeShooter, CubeShooterSpeed cubeShooterSpeed) {
        m_cubeShooter = cubeShooter;
        m_cubeShooterSpeed = cubeShooterSpeed;

        addRequirements(m_cubeShooter);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // make the motor run at the corresponding speed of the requested enum speed
        m_cubeShooter.setSpeed(SHOOTER_SPEEDS.get(m_cubeShooterSpeed));
    }

    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_cubeShooter.setSpeed(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // run until there is no cube
        return m_cubeShooter.isBeamBreak();
    }
}