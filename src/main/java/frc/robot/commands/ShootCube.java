// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.CubeShooterSpeed;
import frc.robot.subsystems.CubeShooter;

public class ShootCube extends CommandBase {
    private CubeShooter m_cubeShooter;

    private CubeShooterSpeed m_cubeShooterSpeed;
    //take from port 

    public ShootCube(CubeShooterSpeed cubeShooterSpeed) {
        m_cubeShooter = new CubeShooter();
        m_cubeShooterSpeed = cubeShooterSpeed;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

    }


}