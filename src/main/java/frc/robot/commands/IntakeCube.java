// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CubeShooter;

public class IntakeCube extends CommandBase {
    private CubeShooter m_cubeShooter; 


    public IntakeCube(CubeShooter cubeShooter) {
        m_cubeShooter = cubeShooter;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_cubeShooter.startIntake();

    }

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
        // stop intaking if there is a cube present
        return m_cubeShooter.isBeamBreak() == false;
    }


}