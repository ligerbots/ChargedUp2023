// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import frc.robot.Constants;

public class Shoulder extends TrapezoidProfileSubsystem {

  // Define the motor and encoders
  private final CANSparkMax m_motorLeader;
  private final CANSparkMax m_motorFollower;
  private final RelativeEncoder m_encoder;
  private final SparkMaxPIDController m_PIDController;

  private final ArmFeedforward m_Feedforward = 
    new ArmFeedforward(Constants.ARM_KS, Constants.ARM_KG, Constants.ARM_KV, Constants.ARM_KA);

  private double m_kPArm = Constants.ARM_K_P;
  private int m_index;

  private boolean m_coastMode = false;

  private boolean m_resetArmPos = false;
  
  /** Creates a new ShoulderArm
   *. */
  public Shoulder() {

    super(
      // The constraints for the generated profiles
      new TrapezoidProfile.Constraints(Constants.ARM_MAX_VEL_RAD_PER_SEC, Constants.ARM_MAX_ACC_RAD_PER_SEC_SQ),
      // The initial position of the mechanism
      Constants.ARM_OFFSET_RAD);

    
    // Create the motor, PID Controller and encoder.
    m_motorLeader = new CANSparkMax(Constants.SHOULDER_CAN_IDS[0], MotorType.kBrushless);
    m_motorFollower = new CANSparkMax(Constants.SHOULDER_CAN_IDS[0], MotorType.kBrushless);
    m_motorLeader.restoreFactoryDefaults();
    // Set follower and invert
    m_motorFollower.follow(m_motorLeader, true);
    
    m_PIDController = m_motorLeader.getPIDController();
    m_PIDController.setP(m_kPArm);
    m_PIDController.setI(Constants.ARM_K_I);
    m_PIDController.setD(Constants.ARM_K_D);
    m_PIDController.setFF(Constants.ARM_K_FF);

    m_encoder = m_motorLeader.getEncoder();
    // Set the position conversion factor. Note that the Trapezoidal control
    // expects angles in radians.
    // TODO: Set this based on shoulder gearbox gear ratio
    m_encoder.setPositionConversionFactor((1.0 / (25.0 * 60.0 / 16.0)) * 2.0 * Math.PI);
    m_encoder.setPosition(Constants.ARM_OFFSET_RAD);
    SmartDashboard.putNumber("arm" + m_index + "/P Gain", m_kPArm);
  }

  @Override
  public void periodic() {
    double encoderValue = m_encoder.getPosition();
    
    // Display current values on the SmartDashboard
    SmartDashboard.putNumber("arm" + m_index + "/Output" + m_index, m_motorLeader.getAppliedOutput());
    SmartDashboard.putNumber("arm" + m_index + "/Encoder" + m_index, Units.radiansToDegrees(encoderValue));
    SmartDashboard.putBoolean("arm" + m_index + "/CoastMode" + m_index, m_coastMode);

    // if in coast mode, stop the periodic() here to prevent the PID from setReference()
    if (m_coastMode)  return;
      
    // // First check if we've gone too far. If we have, reset the setPoint to the limit.
    // m_tooFarForward = encoderValue > Constants.ARM_MAX_ANGLE;
    // SmartDashboard.putBoolean("arm" + m_index + "/too Forward", m_tooFarForward);
    // if (m_tooFarForward) {
    //   // TODO: convert MAX to encoder position
    //   m_PIDController.setReference(Constants.ARM_MAX_ANGLE, ControlType.kPosition, 0, 0.0);
    //   return; // Do we really not want to run super.periodic()?
    // }

    // m_tooFarBack = encoderValue < Constants.ARM_MIN_ANGLE;
    // SmartDashboard.putBoolean("arm" + m_index + "/too Backward", m_tooFarBack);
    // if (m_tooFarBack) {
    //   // TODO: convert MIN to encoder position
    //   m_PIDController.setReference(Constants.ARM_MIN_ANGLE, ControlType.kPosition, 0, 0.0);
    //   return;
    // }
    
    // Execute the super class periodic method
    super.periodic();

    // Here we can check the SmartDashboard for any updates to the PIC constants.
    // Note that since this is Trapezoidal control, we only need to set P.
    // Each increment will only change the set point position a little bit.

    checkPIDVal();
  }

  @Override
  protected void useState(TrapezoidProfile.State setPoint) {
    // Calculate the feedforward fromteh setPoint
    double feedforward = m_Feedforward.calculate(setPoint.position, setPoint.velocity);

    // Add the feedforward to the PID output to get the motor output
    // The ArmFeedForward computes in radians. We need to convert back to degrees.
    // Remember that the encoder was already set to account for the gear ratios.
    
    // TODO: if the "12.0" is volts, should use RobotController.getBatteryVoltage()
    if(m_resetArmPos){
      setPoint.position = m_encoder.getPosition();
      m_resetArmPos = false;
    }
    m_PIDController.setReference(setPoint.position, ControlType.kPosition, 0, feedforward / 12.0);
    SmartDashboard.putNumber("arm" + m_index + "/feedforward" + m_index, feedforward);
    SmartDashboard.putNumber("arm" + m_index + "/setPoint" + m_index, Units.metersToInches(setPoint.position));
    SmartDashboard.putNumber("arm" + m_index + "/velocity" + m_index, Units.metersToInches(setPoint.velocity));
   
}

  private void checkPIDVal() {
    double p = SmartDashboard.getNumber("arm" + m_index + "/P Gain", 0);
    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if ((p != m_kPArm)) {
      m_PIDController.setP(p);
      m_kPArm = p;
    }
  }

  public CANSparkMax getMotor() {
    return m_motorLeader;
  }

  public RelativeEncoder getEncoder() {
    return m_encoder;
  }

  public void setBrakeMode (boolean brake) {
    m_motorLeader.setIdleMode(brake ? CANSparkMax.IdleMode.kBrake : CANSparkMax.IdleMode.kCoast);
    m_coastMode = (brake ? false : true);
  }

  public void resetArmPos(){
    super.setGoal(m_encoder.getPosition());
    m_resetArmPos = true;
  }
}
