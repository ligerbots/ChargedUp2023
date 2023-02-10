// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {

  public Shoulder m_shoulder;

  public Elevator m_elevator;

  public CANSparkMax m_elevatorMotor;
  
  public SparkMaxLimitSwitch m_limitSwitch;


  public Arm() {
    // Construct the arm trapezoid subsystems
    m_shoulder = new Shoulder();
   
    m_elevatorMotor = new CANSparkMax(Constants.ELEVATOR_CAN_ID, MotorType.kBrushless);

    m_elevator = new Elevator(m_elevatorMotor);
   
  }

  public void periodic() {
    
  }

  public void setElevatorExtent(TrapezoidProfile.State extent){
    m_elevator.setSetPoint(extent);
  }

  // rotates the arms to a certain angle
  public void setShoulderAngle(double degree) {
      m_shoulder.setGoal(degree);
  }

  // returns the currrent height of the elevator
  public double getElevatorExtent() {
    return m_elevator.getExtent();
  }

  // returns the current angle of the arm
  public double[] getArmAngle() {
    return new double[] {m_shoulder.getEncoder().getPosition()};
  }

  // Set idle mode of all motors
  public void setBrakeMode(boolean brake) {
    m_shoulder.setBrakeMode(brake);
    m_elevator.setBrakeMode(brake);
  }
}