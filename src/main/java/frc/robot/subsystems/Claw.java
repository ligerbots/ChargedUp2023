// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Claw extends SubsystemBase {
    // open the claw, but don't start the motor
    public abstract void open();
    
    // open the claw and start the motor
    public abstract void startIntake();

    // close the claw. If the motor is running, stop it after a bit
    public abstract void close();

    // enable/disable the compressor
    public abstract void enableCompressor();
    public abstract void disableCompressor();
}
