// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {

    private Shoulder m_shoulder;
    private Reacher m_reacher;

    public Arm() {
        // Construct the shoulder and reacher trapezoid subsystems
        m_shoulder = new Shoulder();
        m_reacher = new Reacher();
    }

    public void periodic() {
    }

    // set arm length in inches
    public void setArmLength(double length) {
        m_reacher.setLength(length);
    }

    // rotates the arm to a certain angle in degrees
    public void setArmAngle(double degree) {
        m_shoulder.setAngle(degree);
    }

    // returns the currrent length of the arm in inches
    public double getArmLength() {
        return m_reacher.getLength();
    }

    // returns the current angle of the arm in degrees
    public double getArmAngle() {
        return m_shoulder.getAngle();
    }
}
