// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch.Type;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {

  public Shoulder m_arm = new Shoulder();

  public Elevator m_elevator = new Elevator(null);

  public CANSparkMax[] m_elevatorMotor;
  
  public SparkMaxLimitSwitch[] m_limitSwitch;


  public Arm() {
    // Construct the arm trapezoid subsystems
    m_arm = new Shoulder();
   
    m_elevatorMotor = new CANSparkMax[] {new CANSparkMax(Constants.ELEVATOR_CAN_IDS[0], MotorType.kBrushless)};

    m_elevator = new Elevator(m_elevatorMotor[0]);
   

    m_limitSwitch = new SparkMaxLimitSwitch[2];

    m_limitSwitch[0] = m_elevatorMotor[0].getReverseLimitSwitch(Type.kNormallyClosed); // set it to normally open to make sure it will never be activated
    m_limitSwitch[0].enableLimitSwitch(true);

  }

  public void periodic() {
    SmartDashboard.putBoolean("elevator0/limitSwitchPressed", m_limitSwitch[0].isPressed());
  }

  public void setElevatorHeight(TrapezoidProfile.State height){
    m_elevator.setSetPoint(height);
  }

  public void setOneElevatorHeight(int index, TrapezoidProfile.State height){
    m_elevator.setSetPoint(height);
  }

  // rotates the arms to a certain angle
  public void setArmAngle(double degree) {
      m_arm.setGoal(degree);
  }

  // returns the currrent height of the elevator
  public double[] getElevatorHeight() {
    return new double[] {m_elevator.getEncoder().getPosition()};
  }

  // returns the current angle of the arm
  public double[] getArmAngle() {
    return new double[] {m_arm.getEncoder().getPosition()};
  }

  // Set idle mode of all motors
  public void setBrakeMode(boolean brake) {
    m_arm.getMotor().setIdleMode(brake ? CANSparkMax.IdleMode.kBrake : CANSparkMax.IdleMode.kCoast);
    m_elevator.getMotor().setIdleMode(brake ? CANSparkMax.IdleMode.kBrake : CANSparkMax.IdleMode.kCoast);
  }

  public void setArmCoastMode(){
    m_arm.idleMotor();
  }

  public void unsetArmCoastMode(){
    m_arm.unIdleMotor();
  }
}