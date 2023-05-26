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
            put(CubeShooterSpeed.LOW, 1.0);
            put(CubeShooterSpeed.MIDDLE, 2.0);
            put(CubeShooterSpeed.HIGH, 3.0);
        }
    };
    
    public ShootCube(CubeShooter cubeShooter, CubeShooterSpeed cubeShooterSpeed) {
        m_cubeShooter = cubeShooter;
        m_cubeShooterSpeed = cubeShooterSpeed;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        // make the motor run at the corresponding speed of the requested enum speed
        m_cubeShooter.shootCube(SHOOTER_SPEEDS.get(m_cubeShooterSpeed));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // run until there is no cube
        return m_cubeShooter.isBeamBreak();
    }

}