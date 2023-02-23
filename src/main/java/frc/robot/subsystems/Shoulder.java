// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import frc.robot.Constants;

public class Shoulder extends TrapezoidProfileSubsystem {

    // TODO: The following constants came from the 2022 robot.
    // These need to be set for this robot.

    // All units are MKS with angles in Radians

    // Feedforward constants for the shoulder
    private static final double SHOULDER_KS = 0.182; // TODO: This may need to be tuned
    // The following constants are computed from https://www.reca.lc/arm
    private static final double SHOULDER_KG = 0.09; // V
    private static final double SHOULDER_KV = 6.60; // V*sec/rad
    private static final double SHOULDER_KA = 0.01; // V*sec^2/rad
    
  
    // Constants to limit the shoulder rotation speed
    // For initial testing, these should be very slow.
    // We can update them as we get more confidence.
    private static final double SHOULDER_MAX_VEL_RADIAN_PER_SEC = Units.degreesToRadians(20.0); // 5 deg/sec
    // Let's give it 2 seconds to get to max velocity.
    // Once tuned, I expect we will want this to be equal to SHOULDER_MAX_VEL_RADIAN_PER_SEC
    // so it will get to max velocity in one second.
    private static final double SHOULDER_MAX_ACC_RADIAN_PER_SEC_SQ = Units.degreesToRadians(20.0); // 2.5 deg/sec^2
    private static final double SHOULDER_POSITION_OFFSET = 242.0/360.0;
    private static final double SHOULDER_OFFSET_RADIAN = SHOULDER_POSITION_OFFSET * 2 * Math.PI;

    // The Shoulder gear ratio is ~395.77, but let's get it exactly.
    private static final double SHOULDER_GEAR_RATIO = (84.0 / 12.0) * (84.0 / 18.0) * (84.0 / 26.0) * (60.0 / 12.0);

    // PID Constants for the shoulder PID controller
    // Since we're using Trapeziodal control, all values will be 0 except for P
    private static final double SHOULDER_K_P = 0.1;
    private static final double SHOULDER_K_I = 0.0;
    private static final double SHOULDER_K_D = 0.0;
    private static final double SHOULDER_K_FF = 0.0;
    private static final int kPIDLoopIdx = 0;
    private static final int kTimeoutMs = 0;

    // The TalonFX, the integrated motor controller for the Falcon, uses ticks as it's noative unit.
    // There are 2048 ticks per revolution. Need to account for the gear ratio.
    private static final double SHOULDER_RADIAN_PER_UNIT = 2 * Math.PI / (2048.0 * SHOULDER_GEAR_RATIO);

    private final ArmFeedforward m_feedForward = new ArmFeedforward(SHOULDER_KS, SHOULDER_KG, SHOULDER_KV, SHOULDER_KA);

    // Define the motor and encoders
    private final WPI_TalonFX m_motorLeader;
    private final WPI_TalonFX m_motorFollower;
    private final TalonFXSensorCollection m_encoder;
    // private final EncoderSim m_encoderSim;
    DutyCycleEncoder m_Duty_Encoder;
    // The P gain for the PID controller that drives this shoulder.
    private static double m_kPShoulder = SHOULDER_K_P;

    private boolean m_resetShoulderPos = false;
    private boolean m_coastMode = false;

    // current goal in degrees
    private double m_goal;

    // *********************Simulation Stuff************************

    // The arm gearbox represents a gearbox containing two Falcon motors.
    private final DCMotor m_shoulderGearbox = DCMotor.getFalcon500(2);

    // Simulation classes help us simulate what's going on, including gravity.
    private static final double SHOULDER_MASS = 10; // Kilograms
    private static final double SHOULDER_LENGTH = Units.inchesToMeters(30);

    // private TalonFXSimCollection m_motorSim;

    private final SingleJointedArmSim m_shoulderSim = new SingleJointedArmSim(m_shoulderGearbox, SHOULDER_GEAR_RATIO,
            SingleJointedArmSim.estimateMOI(SHOULDER_LENGTH, SHOULDER_MASS), SHOULDER_LENGTH,
            Units.degreesToRadians(-75), Units.degreesToRadians(120), true);

    // Create a Mechanism2d display of an Arm with a fixed ArmTower and moving Arm.
    // private final EncoderSim m_encoderSim = new EncoderSim(m_encoder);
    private final Mechanism2d m_mech2d = new Mechanism2d(60, 60);
    private final MechanismRoot2d m_shoulderPivot = m_mech2d.getRoot("ArmPivot", 30, 30);
    private final MechanismLigament2d m_shoulderTower = m_shoulderPivot.append(new MechanismLigament2d("ArmTower", 30, -90));
    private final MechanismLigament2d m_shoulder = m_shoulderPivot.append(new MechanismLigament2d("Arm", 30,
            Units.radiansToDegrees(m_shoulderSim.getAngleRads()), 6, new Color8Bit(Color.kYellow)));

    // Construct a new Shoulder subsystem
    public Shoulder(DutyCycleEncoder dutyCycleEncoder) {
        super(new TrapezoidProfile.Constraints(SHOULDER_MAX_VEL_RADIAN_PER_SEC / SHOULDER_RADIAN_PER_UNIT,
                SHOULDER_MAX_ACC_RADIAN_PER_SEC_SQ / SHOULDER_RADIAN_PER_UNIT),
                (SHOULDER_OFFSET_RADIAN-dutyCycleEncoder.getDistance() * 2 * Math.PI) / SHOULDER_RADIAN_PER_UNIT);

        m_motorLeader = new WPI_TalonFX(Constants.SHOULDER_CAN_ID_LEADER);
        m_motorFollower = new WPI_TalonFX(Constants.SHOULDER_CAN_ID_FOLLOWER);
        m_encoder = m_motorLeader.getSensorCollection();

        m_motorLeader.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, kPIDLoopIdx, kTimeoutMs);
        // Set follower
        m_motorFollower.follow(m_motorLeader, FollowerType.PercentOutput);

        m_motorLeader.config_kF(kPIDLoopIdx, SHOULDER_K_FF, kTimeoutMs);
		m_motorLeader.config_kP(kPIDLoopIdx, SHOULDER_K_P, kTimeoutMs);
		m_motorLeader.config_kI(kPIDLoopIdx, SHOULDER_K_I, kTimeoutMs);
		m_motorLeader.config_kD(kPIDLoopIdx, SHOULDER_K_D, kTimeoutMs);

        m_Duty_Encoder = dutyCycleEncoder;

        // Encoder distance is in radians
        m_Duty_Encoder.setDistancePerRotation(2 * Math.PI);
        m_Duty_Encoder.setPositionOffset(SHOULDER_POSITION_OFFSET);

        double initialAngle = -m_Duty_Encoder.getDistance();
        SmartDashboard.putNumber("shoulder/initAngle", Units.radiansToDegrees(initialAngle));

        // Set the motor encoder and Position setpoint to the initialAngle from the absolute encoder
        m_encoder.setIntegratedSensorPosition(initialAngle / SHOULDER_RADIAN_PER_UNIT, 0);
        SmartDashboard.putNumber("shoulder/initialAngle_SHOULDER_RADIAN_PER_UNIT", initialAngle / SHOULDER_RADIAN_PER_UNIT);
        SmartDashboard.putNumber("shoulder/encoderIntegSensPos", m_encoder.getIntegratedSensorPosition());
        // m_motorLeader.setSelectedSensorPosition(-m_Duty_Encoder.getDistance());
        // m_motorLeader.setSelectedSensorPosition(m_encoder.getIntegratedSensorPosition());
        // m_motorLeader.setSelectedSensorPosition(initialAngle / SHOULDER_RADIAN_PER_UNIT);
        // SmartDashboard.putNumber("shoulder/motorLeaderIntegSensPos", m_motorLeader.getSelectedSensorPosition());

        // m_Duty_Encoder.setPositionOffset(SHOULDER_OFFSET_RADIAN);
        // m_motorSim = new TalonFXSimCollection(m_motorLeader);
        // m_encoderSim = new TalonFXSimCollection(m_encoder);
        SmartDashboard.putNumber("shoulder/absolute Encoder", Math.toDegrees(-m_Duty_Encoder.getDistance()));
        SmartDashboard.putNumber("shoulder/P Gain", m_kPShoulder);
        SmartDashboard.putData("shoulder Sim", m_mech2d);
        // m_motorLeader.set(ControlMode.Position, )
        // m_motorLeader.set(ControlMode.Position, m_encoder.getIntegratedSensorPosition(), DemandType.ArbitraryFeedForward, 0.0);//feedforward/12.0);
        // setAngle(m_encoder.getIntegratedSensorPosition() * SHOULDER_RADIAN_PER_UNIT);
    }

    public void simulationPeriodic() {
        // First, we set our "inputs" (voltages)
        m_shoulderSim.setInput(m_motorLeader.get() * RobotController.getBatteryVoltage());
        // Next, we update it. The standard loop time is 20ms.
        m_shoulderSim.update(0.020);

        // Finally, we set our simulated encoder's readings and simulated battery
        // voltage
        // m_motorSim.setIntegratedSensorRawPosition(m_armSim.getAngleRads());
        // SimBattery estimates loaded battery voltages
        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(m_shoulderSim.getCurrentDrawAmps()));

        // Update the Mechanism Arm angle based on the simulated arm angle
        m_shoulder.setAngle(Units.radiansToDegrees(m_shoulderSim.getAngleRads()));
        SmartDashboard.putNumber("Simulated Shoulder Angle", Units.radiansToDegrees(m_shoulderSim.getAngleRads()));
        SmartDashboard.putNumber("Sim Battery voltage", RobotController.getBatteryVoltage());
    }

    @Override
    public void periodic() {
        // Display current values on the SmartDashboard
        SmartDashboard.putNumber("shoulder/Output", m_motorLeader.get());
        SmartDashboard.putNumber("shoulder/Encoder", Units.radiansToDegrees(getAngle()));
        // SmartDashboard.putNumber("shoulder/Encoder", getAngle());
        SmartDashboard.putNumber("shoulder/Goal", Units.radiansToDegrees(m_goal));
        SmartDashboard.putBoolean("shoulder/CoastMode", m_coastMode);
        // SmartDashboard.putNumber("shoulder/absolute Encoder", m_Duty_Encoder.getDistance() / 1024.0 * 360.0);        
        SmartDashboard.putNumber("shoulder/absolute Encoder", Math.toDegrees(-m_Duty_Encoder.getDistance()));
        SmartDashboard.putBoolean("shoulder/m_resetShoulderPos", m_resetShoulderPos);
        if (m_coastMode)
            return;

        // Execute the super class periodic method
        super.periodic();

        // Here we can check the SmartDashboard for any updates to the PIC constants.
        // Note that since this is Trapezoidal control, we only need to set P.
        // Each increment will only change the set point position a little bit.
        checkPIDVal();
    }

    @Override
    protected void useState(TrapezoidProfile.State setPoint) {
        // Calculate the feedforward from the setPoint
        double feedforward = m_feedForward.calculate(setPoint.position, setPoint.velocity);

        // Add the feedforward to the PID output to get the motor output
        // The ArmFeedForward computes in radians. We need to convert back to degrees.
        // Remember that the encoder was already set to account for the gear ratios.

        // TODO: if the "12.0" is volts, should use RobotController.getBatteryVoltage()
        if (m_resetShoulderPos) {
            setPoint.position = m_encoder.getIntegratedSensorPosition();
            m_resetShoulderPos = false;
        }

        m_motorLeader.set(ControlMode.Position, setPoint.position, DemandType.ArbitraryFeedForward, 0.0);//feedforward/12.0);
        SmartDashboard.putNumber("shoulder/feedforward", feedforward);
        SmartDashboard.putNumber("shoulder/setPoint", Units.radiansToDegrees(setPoint.position * SHOULDER_RADIAN_PER_UNIT));
        // SmartDashboard.putNumber("shoulder/velocity", Units.metersToInches(setPoint.velocity));
    }

    private void checkPIDVal() {
        double p = SmartDashboard.getNumber("shoulder/P Gain", 0);
        // if PID coefficients on SmartDashboard have changed, write new values to controller
        if ((p != m_kPShoulder)) {
            m_motorLeader.config_kP(kPIDLoopIdx, p, kTimeoutMs);
            m_kPShoulder = p;
        }
    }

    // return current shoulder angle in radians
    public double getAngle() {
        // return m_encoder.getIntegratedSensorAbsolutePosition() * SHOULDER_RADIAN_PER_UNIT;
        return m_encoder.getIntegratedSensorPosition() * SHOULDER_RADIAN_PER_UNIT;
    }

    public void resetShoulderPos() {
        setAngle(getAngle());
        m_resetShoulderPos = true;
    }

    // set shoulder angle in radians
    public void setAngle(double angle) {
        m_goal = angle;
        super.setGoal(angle / SHOULDER_RADIAN_PER_UNIT);
    }
    public void resetGoal(){
        // m_encoder.setIntegratedSensorPosition(getAngle()/SHOULDER_RADIAN_PER_UNIT, kTimeoutMs);
        setAngle(getAngle());
        // super.disable();
        // m_superEnabled = false;
    }
}
