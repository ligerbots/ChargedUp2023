// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
// import edu.wpi.first.math.controller.ArmFeedforward;
// import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;

import frc.robot.Constants;
import frc.robot.Robot;

public class Shoulder extends TrapezoidProfileSubsystem {
    private static final double SHOULDER_MAX_ANGLE = Math.toRadians(30.0);  
    private static final double SHOULDER_MIN_ANGLE = Math.toRadians(-65.0);

    public static final double SHOULDER_ANGLE_TOLERANCE_RADIAN = Math.toRadians(3.0);

    private static final double LEADER_CURRENT_LIMIT = 40.0;
    private static final double FOLLOW_CURRENT_LIMIT = 40.0;

    // All units are MKS with angles in Radians

    // // Feedforward constants for the shoulder
    // private static final double SHOULDER_KS = 0.182; // TODO: This may need to be tuned
    // // The following constants are computed from https://www.reca.lc/arm
    // private static final double SHOULDER_KG = 0.09; // V
    // private static final double SHOULDER_KV = 6.60; // V*sec/rad
    // private static final double SHOULDER_KA = 0.01; // V*sec^2/rad
    

    // Constants to limit the shoulder rotation speed
    private static final double SHOULDER_MAX_VEL_RADIAN_PER_SEC = Units.degreesToRadians(300.0); // 120 deg/sec
    private static final double SHOULDER_MAX_ACC_RADIAN_PER_SEC_SQ = Units.degreesToRadians(450.0); // 120 deg/sec^2

    private static final double SHOULDER_POSITION_OFFSET = 62.0/360.0;
    private static final double SHOULDER_OFFSET_RADIAN = SHOULDER_POSITION_OFFSET * 2 * Math.PI;

    // The Shoulder gear ratio is 288, but let's get it exactly.
    // private static final double SHOULDER_GEAR_RATIO = (84.0 /12.0) * (84.0 / 18.0) * (84.0 / 26.0) * (60.0 / 22.0);
    public static final double SHOULDER_GEAR_RATIO = (84.0 /12.0) * (84.0 / 18.0) * (70.0 / 40.0) * (60.0 / 22.0);

    // PID Constants for the shoulder PID controller
    // Since we're using Trapeziodal control, all values will be 0 except for P
    private static final double SHOULDER_K_P = 0.15;
    private static final double SHOULDER_K_I = 0.0;
    private static final double SHOULDER_K_D = 0.0;
    private static final double SHOULDER_K_FF = 0.0;
    private static final int kPIDLoopIdx = 0;
    private static final int kTimeoutMs = 0;

    // The TalonFX, the integrated motor controller for the Falcon, uses ticks as it's noative unit.
    // There are 2048 ticks per revolution. Need to account for the gear ratio.
    private static final double SHOULDER_RADIAN_PER_UNIT = 2 * Math.PI / (2048.0 * SHOULDER_GEAR_RATIO);

    // private final ArmFeedforward m_feedForward = new ArmFeedforward(SHOULDER_KS, SHOULDER_KG, SHOULDER_KV, SHOULDER_KA);

    // Define the motor and encoders
    private final WPI_TalonFX m_motorLeader;
    private final WPI_TalonFX m_motorFollower;
    private final TalonFXSensorCollection m_encoder;
    DutyCycleEncoder m_dutyEncoder;
    
    // The P gain for the PID controller that drives this shoulder.
    private double m_kPShoulder = SHOULDER_K_P;
    private double m_kFeedForward = SHOULDER_K_FF;

    private boolean m_resetShoulderPos = false;
    private boolean m_coastMode = false;

    // current goal in radians
    private double m_goal;

    // Use null to indicate if we should use the simulation
    private ShoulderSim m_armSim = null;
    private EncoderSim m_encoderSim = null;

    // Construct a new Shoulder subsystem
    public Shoulder(DutyCycleEncoder dutyCycleEncoder) {
        super(new TrapezoidProfile.Constraints(SHOULDER_MAX_VEL_RADIAN_PER_SEC / SHOULDER_RADIAN_PER_UNIT,
                SHOULDER_MAX_ACC_RADIAN_PER_SEC_SQ / SHOULDER_RADIAN_PER_UNIT),
                (SHOULDER_OFFSET_RADIAN-dutyCycleEncoder.getDistance() * 2 * Math.PI) / SHOULDER_RADIAN_PER_UNIT);

        m_motorLeader = new WPI_TalonFX(Constants.SHOULDER_CAN_ID_LEADER);
        m_motorFollower = new WPI_TalonFX(Constants.SHOULDER_CAN_ID_FOLLOWER);

        m_motorLeader.configFactoryDefault();
        m_motorFollower.configFactoryDefault();

        m_encoder = m_motorLeader.getSensorCollection();

        m_motorLeader.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, kPIDLoopIdx, kTimeoutMs);
        // Set follower
        m_motorFollower.follow(m_motorLeader, FollowerType.PercentOutput);

        m_motorLeader.config_kF(kPIDLoopIdx, SHOULDER_K_FF, kTimeoutMs);
		m_motorLeader.config_kP(kPIDLoopIdx, SHOULDER_K_P, kTimeoutMs);
		m_motorLeader.config_kI(kPIDLoopIdx, SHOULDER_K_I, kTimeoutMs);
		m_motorLeader.config_kD(kPIDLoopIdx, SHOULDER_K_D, kTimeoutMs);

        // limits for motor leader and folower
        // always limit current to the values. Trigger limit = 0 so that it is always enforced.
        m_motorLeader.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, LEADER_CURRENT_LIMIT, 0, 0));
        m_motorFollower.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, FOLLOW_CURRENT_LIMIT, 0, 0));

        m_dutyEncoder = dutyCycleEncoder;

        // Encoder distance is in radians
        m_dutyEncoder.setDistancePerRotation(2 * Math.PI);
        m_dutyEncoder.setPositionOffset(SHOULDER_POSITION_OFFSET);

        double initialAngle = -m_dutyEncoder.getDistance();
        // SmartDashboard.putNumber("shoulder/initAngle", Units.radiansToDegrees(initialAngle));

        // Set the motor encoder and Position setpoint to the initialAngle from the absolute encoder
        m_encoder.setIntegratedSensorPosition(initialAngle / SHOULDER_RADIAN_PER_UNIT, 0);

        SmartDashboard.putNumber("shoulder/leaderCurrentLimit", LEADER_CURRENT_LIMIT);
        SmartDashboard.putNumber("shoulder/followCurrentLimit", FOLLOW_CURRENT_LIMIT);

        setCoastMode(false);
        SmartDashboard.putBoolean("shoulder/coastMode", m_coastMode);

        if (Robot.isSimulation()) {
            // not sure how to sim and Talon encoder
            m_encoderSim = EncoderSim.createForIndex(10);
            m_encoderSim.setDistancePerPulse(SHOULDER_RADIAN_PER_UNIT);
            m_armSim = new ShoulderSim(m_motorLeader, m_encoderSim);
        }
    }

    @Override
    public void periodic() {
        // Display current values on the SmartDashboard
        // Add some extra numbers to diagnose the load on the motors
        SmartDashboard.putNumber("shoulder/leaderOutput", m_motorLeader.get());
        SmartDashboard.putNumber("shoulder/getAngle", Math.toDegrees(getAngle()));
        SmartDashboard.putNumber("shoulder/encoderCount", getEncoderCount());
        SmartDashboard.putNumber("shoulder/encoderSpeed", Math.toDegrees(getSpeed()));
        SmartDashboard.putNumber("shoulder/goal", Math.toDegrees(m_goal));
        SmartDashboard.putNumber("shoulder/absoluteEncoder", Math.toDegrees(-m_dutyEncoder.getDistance()));
        // SmartDashboard.putBoolean("shoulder/m_resetShoulderPos", m_resetShoulderPos);
        // SmartDashboard.putNumber("shoulder/leaderStatorI", m_motorLeader.getStatorCurrent());
        // SmartDashboard.putNumber("shoulder/followerStatorI", m_motorFollower.getStatorCurrent());
        // SmartDashboard.putNumber("shoulder/leaderSupplyI", m_motorLeader.getSupplyCurrent());
        // SmartDashboard.putNumber("shoulder/followerSupplyI", m_motorFollower.getSupplyCurrent());

        m_coastMode = SmartDashboard.getBoolean("shoulder/coastMode", m_coastMode);
        if (m_coastMode)
            setCoastMode(m_coastMode);
        
        // Jack: I dont think we need this check because we are only going to set to coast mode during disabled and the motor won't be moved by super.periodic() or useState() anyway
        // And we want the setPoint to follow the current encoder reading in disabled mode
        // if (m_coastMode)
        //     return;

        // Execute the super class periodic method
        super.periodic();

        // Here we can check the SmartDashboard for any updates to the PIC constants.
        // Note that since this is Trapezoidal control, we only need to set P.
        // Each increment will only change the set point position a little bit.
        // checkPIDVal();
    }

    @Override
    protected void useState(TrapezoidProfile.State setPoint) {
        // Calculate the feedforward from the setPoint
        // double feedforward = m_feedForward.calculate(setPoint.position, setPoint.velocity);

        // Add the feedforward to the PID output to get the motor output
        // The ArmFeedForward computes in radians. We need to convert back to degrees.
        // Remember that the encoder was already set to account for the gear ratios.

        // TODO: if the "12.0" is volts, should use RobotController.getBatteryVoltage()
        if (m_resetShoulderPos) {
            setPoint.position = getEncoderCount();
            m_resetShoulderPos = false;
        }

        m_kFeedForward = SmartDashboard.getNumber("shoulder/kFeadForward", m_kFeedForward);

        m_motorLeader.set(ControlMode.Position, setPoint.position, DemandType.ArbitraryFeedForward, m_kFeedForward); //, feedforward / 12.0);
        // SmartDashboard.putNumber("shoulder/feedforward", feedforward);
        SmartDashboard.putNumber("shoulder/setPoint", Math.toDegrees(setPoint.position * SHOULDER_RADIAN_PER_UNIT));
        // SmartDashboard.putNumber("shoulder/velocity", Units.metersToInches(setPoint.velocity));
    }

    // private void checkPIDVal() {
    //     double p = SmartDashboard.getNumber("shoulder/P Gain", 0);
    //     // if PID coefficients on SmartDashboard have changed, write new values to controller
    //     if ((p != m_kPShoulder)) {
    //         m_motorLeader.config_kP(kPIDLoopIdx, p, kTimeoutMs);
    //         m_kPShoulder = p;
    //     }
    // }

    // return current shoulder angle in radians
    public double getAngle() {
        if (m_armSim != null) {
            // simulation
            return m_encoderSim.getDistance();
        }
        else
            return m_encoder.getIntegratedSensorPosition() * SHOULDER_RADIAN_PER_UNIT;
    }

    double getEncoderCount() {
        if (m_encoderSim != null) {
            // simulation
            return m_encoderSim.getCount();
        } else
            return m_encoder.getIntegratedSensorPosition();
    }
    
    // return current shoulder angular speed in radians/sec
    public double getSpeed() {
        return m_encoder.getIntegratedSensorVelocity() * SHOULDER_RADIAN_PER_UNIT;
    }
    
    public void resetShoulderPos() {
        setAngle(getAngle());
        m_resetShoulderPos = true;
    }

    // needs to be public so that commands can get the restricted angle
    public static double limitShoulderAngle(double angle){
        return Math.min(SHOULDER_MAX_ANGLE, Math.max(SHOULDER_MIN_ANGLE, angle));
    }

    // set shoulder angle in radians
    public void setAngle(double angle) {
        m_goal = limitShoulderAngle(angle);
        super.setGoal(m_goal / SHOULDER_RADIAN_PER_UNIT);
    }
    public void resetGoal(){
        // m_encoder.setIntegratedSensorPosition(getAngle()/SHOULDER_RADIAN_PER_UNIT, kTimeoutMs);
        setAngle(getAngle());
        // super.disable();
        // m_superEnabled = false;
    }

    public void setCoastMode(boolean coastMode){
        if (coastMode) {
            m_motorLeader.setNeutralMode(NeutralMode.Coast);
            m_motorLeader.stopMotor();
        } else
            m_motorLeader.setNeutralMode(NeutralMode.Brake);
    }

    @Override
    public void simulationPeriodic() {
        if (m_armSim != null)
            m_armSim.simulationPeriodic();
    }
}
