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

public class Shoulder extends SubsystemBase {

  public ShoulderArm m_arm = new ShoulderArm();

  public Elevator[] m_elevator = new Elevator(0, false, null);

  public CANSparkMax[] m_elevatorMotor;
  
  public SparkMaxLimitSwitch[] m_limitSwitch;


  public Shoulder() {
    // Construct the arm trapezoid subsystems
    m_arm = new ShoulderArm();
   
    m_elevatorMotor = new CANSparkMax[] {new CANSparkMax(Constants.ELEVATOR_CAN_IDS[0], MotorType.kBrushless), new CANSparkMax(Constants.ELEVATOR_CAN_IDS[1], MotorType.kBrushless)};

    m_elevator = new Elevator(0, false);
   

    m_limitSwitch = new SparkMaxLimitSwitch[2];

    m_limitSwitch[0] = m_elevatorMotor[0].getReverseLimitSwitch(Type.kNormallyClosed); // set it to normally open to make sure it will never be activated
    m_limitSwitch[0].enableLimitSwitch(true);
    m_limitSwitch[1] = m_elevatorMotor[1].getReverseLimitSwitch(Type.kNormallyClosed);
    m_limitSwitch[1].enableLimitSwitch(true);
  }

  public void periodic() {
    SmartDashboard.putBoolean("elevator0/limitSwitchPressed", m_limitSwitch[0].isPressed());
    SmartDashboard.putBoolean("elevator1/limitSwitchPressed", m_limitSwitch[1].isPressed());
  }

  public void setElevatorHeight(TrapezoidProfile.State height){
    m_elevator[0].setSetPoint(height);
    m_elevator[1].setSetPoint(height);
  }

  public void setOneElevatorHeight(int index, TrapezoidProfile.State height){
    m_elevator[index].setSetPoint(height);
  }

  // rotates the arms to a certain angle
  public void setArmAngle(double degree) {
      m_arm[0].setGoal(degree);
      m_arm[1].setGoal(degree);
  }

  // returns the currrent height of the elevator
  public double[] getElevatorHeight() {
    return new double[] {m_elevator[0].getEncoder().getPosition(), m_elevator[1].getEncoder().getPosition()};
  }

  // returns the current angle of the arm
  public double[] getArmAngle() {
    return new double[] {m_arm[0].getEncoder().getPosition(), m_arm[1].getEncoder().getPosition()};
  }

  // Set idle mode of all motors
  public void setBrakeMode(boolean brake) {
    m_arm[0].getMotor().setIdleMode(brake ? CANSparkMax.IdleMode.kBrake : CANSparkMax.IdleMode.kCoast);
    m_arm[1].getMotor().setIdleMode(brake ? CANSparkMax.IdleMode.kBrake : CANSparkMax.IdleMode.kCoast);
    m_elevator[0].getMotor().setIdleMode(brake ? CANSparkMax.IdleMode.kBrake : CANSparkMax.IdleMode.kCoast);
    m_elevator[1].getMotor().setIdleMode(brake ? CANSparkMax.IdleMode.kBrake : CANSparkMax.IdleMode.kCoast);
  }

  public void setArmCoastMode(){
    m_arm[0].idleMotor();
  }

  public void unsetArmCoastMode(){
    m_arm[0].unIdleMotor();
  }
}