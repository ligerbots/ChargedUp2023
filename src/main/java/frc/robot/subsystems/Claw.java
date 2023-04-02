// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Claw extends SubsystemBase {
        
    private AnalogInput m_infraredSensor = new AnalogInput(1);

    // threshold to detect the game pieces in the claw 
    private static final double INTAKE_DISTANCE_VOLTAGE_THRESHOLD = 1.5;

    // median filter to filter the IR sensor reading
    private final MedianFilter m_medianFilter = new MedianFilter(10);

    protected double m_curIRSensorReading = 0.0;

    // open the claw, but don't start the motor
    public abstract void open();
    
    // open the claw and start the motor
    public abstract void startIntake();

    // close the claw. If the motor is running, stop it after a bit
    public abstract void close();

    // enable/disable the compressor
    public abstract void enableCompressor();
    public abstract void disableCompressor();

    public Runnable updateIRSensorPeriodic(){
        return () -> {
            m_curIRSensorReading = m_medianFilter.calculate(m_infraredSensor.getVoltage());
        };
    }

    public boolean hasGamePiece(){
        return m_curIRSensorReading > INTAKE_DISTANCE_VOLTAGE_THRESHOLD;
    }
}
