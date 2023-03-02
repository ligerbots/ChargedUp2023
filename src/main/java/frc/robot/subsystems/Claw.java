// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Claw extends SubsystemBase {
    /** Creates a new Claw. */
    public Claw() {}

    // This method will be called once per scheduler run
    @Override
    public void periodic() {}

    public abstract void open();
    
    public abstract void startIntake();

    public abstract void close();

    public abstract void enableCompressor();

    public abstract void disableCompressor();
}
