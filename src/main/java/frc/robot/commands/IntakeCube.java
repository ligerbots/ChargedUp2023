// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.CubeShooterSpeed;

public class IntakeCube extends CommandBase {

    //check if it first DigitalInput
    DigitalInput m_beamBreak = new DigitalInput(0);

    public IntakeCube() {

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        SmartDashboard.putBoolean("On?", m_beamBreak.get());
        
        //run motor

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // finish if there is a cube present
        return m_beamBreak.get() == false;
        
    }

}