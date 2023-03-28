// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {

    private Shoulder m_shoulder;
    private Reacher m_reacher;
    private DutyCycleEncoder m_Duty_Encoder = new DutyCycleEncoder(0);

    public Arm() {
        // Construct the shoulder and reacher trapezoid subsystems
        m_shoulder = new Shoulder(m_Duty_Encoder);
        m_reacher = new Reacher();
    
        // SD variables for the manual setting commands
        SmartDashboard.putNumber("Testing/SetArmLengthTest", 0.0);
        SmartDashboard.putNumber("Testing/SetArmAngleTest", 0.0);
    }

    public void periodic() {
    }

    public void retractArm(){
        m_reacher.setLength(0.0);
    }

    // set arm length in meters
    public void setArmLength(double length) {
        m_reacher.setLength(length);
    }

    // rotates the arm to a certain angle in radians
    public void setArmAngle(double radian) {
        m_shoulder.setAngle(radian);
    }

    // returns the currrent length of the arm in meters
    public double getArmLength() {
        return m_reacher.getLength();
    }

    // returns the current angle of the arm in radians
    public double getArmAngle() {
        return m_shoulder.getAngle();
    }
    
    public void resetGoal(){
        m_shoulder.resetGoal();
        m_reacher.resetGoal();
    }

    public void resetShoulderPos(){
        m_shoulder.resetShoulderPos();
    }

    public void resetReacherPos(){
        m_reacher.resetReacherPos();
    }
}
