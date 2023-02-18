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

public class Shoulder extends TrapezoidProfileSubsystem {

    // TODO: The following constants came from the 2022 robot.
    // These need to be set for this robot.

    // CAND IDs for the Shoulder motors
    private static final int SHOULDER_CAN_ID_LEADER = 14;
    private static final int SHOULDER_CAN_ID_FOLLWER = 15;

    // Feedforward constants for the shoulder
    private static final double SHOULDER_KS = 0.182; // TODO: This may need to be tuned
    // The following constants are computed from https://www.reca.lc/arm
    private static final double SHOULDER_KG = 2.07;
    private static final double SHOULDER_KV = 1.83;
    private static final double SHOULDER_KA = 0.08;

    // Constants to limit the shoulder rotation speed
    private static final double SHOULDER_MAX_VEL_RAD_PER_SEC = Math.toRadians(200.0);
    private static final double SHOULDER_MAX_ACC_RAD_PER_SEC_SQ = Math.toRadians(200.0);
    private static final double SHOULDER_OFFSET_ANGLE = 0.0;

    // PID Constants for the shoulder PID controller
    // Since we're using Trapeziodal control, all values will be 0 except for P
    private static final double SHOULDER_K_P = 1.0;
    private static final double SHOULDER_K_I = 0.0;
    private static final double SHOULDER_K_D = 0.0;
    private static final double SHOULDER_K_FF = 0.0;
    private static final int kPIDLoopIdx = 0;
    private static final int kTimeoutMs = 0;

    private static final double SHOULDER_ANGLE_PER_UNIT = 360.0 / 2048.0;

    private final ArmFeedforward m_feedForward = new ArmFeedforward(SHOULDER_KS, SHOULDER_KG,
    SHOULDER_KV, SHOULDER_KA);

    // Define the motor and encoders
    private final WPI_TalonFX m_motorLeader;
    private final WPI_TalonFX m_motorFollower;
    
    private final TalonFXSensorCollection m_encoder;
    // private final EncoderSim m_encoderSim;

    // The P gain for the PID controller that drives this shoulder.
    private static double m_kPShoulder = 1.0;

    private boolean m_resetShoulderPos = false;
    private boolean m_coastMode = false;

    // current goal in degrees
    private double m_goal;

    // *********************Simulation Stuff************************

    // distance per pulse = (angle per revolution) / (pulses per revolution)
    // = (2 * PI rads) / (4096 pulses)
    private static final double kShoulderEncoderDistPerPulse = 2.0 * Math.PI / 4096;
    // The arm gearbox represents a gearbox containing two Vex 775pro motors.
    private final DCMotor m_shoulderGearbox = DCMotor.getFalcon500(2);

    // Simulation classes help us simulate what's going on, including gravity.
    private static final double m_shoulderReduction = 395.77;
    private static final double m_shoulderMass = 10; // Kilograms
    private static final double m_shoulderLength = Units.inchesToMeters(30);

    private TalonFXSimCollection m_motorSim;

    private final SingleJointedArmSim m_shoulderSim = new SingleJointedArmSim(m_shoulderGearbox, m_shoulderReduction,
            SingleJointedArmSim.estimateMOI(m_shoulderLength, m_shoulderMass), m_shoulderLength,
            Units.degreesToRadians(-75), Units.degreesToRadians(120), true);

    // Create a Mechanism2d display of an Arm with a fixed ArmTower and moving Arm.
    // private final EncoderSim m_encoderSim = new EncoderSim(m_encoder);
    private final Mechanism2d m_mech2d = new Mechanism2d(60, 60);
    private final MechanismRoot2d m_shoulderPivot = m_mech2d.getRoot("ArmPivot", 30, 30);
    private final MechanismLigament2d m_shoulderTower = m_shoulderPivot.append(new MechanismLigament2d("ArmTower", 30, -90));
    private final MechanismLigament2d m_shoulder = m_shoulderPivot.append(new MechanismLigament2d("Arm", 30,
            Units.radiansToDegrees(m_shoulderSim.getAngleRads()), 6, new Color8Bit(Color.kYellow)));

    // Construct a new Shoulder subsystem
    public Shoulder() {
        super(new TrapezoidProfile.Constraints(SHOULDER_MAX_VEL_RAD_PER_SEC, SHOULDER_MAX_ACC_RAD_PER_SEC_SQ));

        m_motorLeader = new WPI_TalonFX(SHOULDER_CAN_ID_LEADER);
        m_motorFollower = new WPI_TalonFX(SHOULDER_CAN_ID_FOLLWER);
        m_encoder = m_motorLeader.getSensorCollection();
        // m_motorSim = new TalonFXSimCollection(m_motorLeader);
        // m_encoderSim = new TalonFXSimCollection(m_encoder);
        m_motorLeader.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, kPIDLoopIdx, kTimeoutMs);
        // Set follower and invert
        m_motorFollower.follow(m_motorLeader, FollowerType.PercentOutput);
        m_motorLeader.config_kF(kPIDLoopIdx, SHOULDER_K_FF, kTimeoutMs);
		m_motorLeader.config_kP(kPIDLoopIdx, SHOULDER_K_P, kTimeoutMs);
		m_motorLeader.config_kI(kPIDLoopIdx, SHOULDER_K_I, kTimeoutMs);
		m_motorLeader.config_kD(kPIDLoopIdx, SHOULDER_K_D, kTimeoutMs);

        m_encoder.setIntegratedSensorPosition(SHOULDER_OFFSET_ANGLE / SHOULDER_ANGLE_PER_UNIT, 0);
        // m_motorLeader.setSelectedSensorPosition(0.0);

        SmartDashboard.putNumber("shoulder/P Gain", m_kPShoulder);
        SmartDashboard.putData("shoulder Sim", m_mech2d);
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
        SmartDashboard.putNumber("Simulated Shoulder Angle", m_shoulderSim.getAngleRads());
        SmartDashboard.putNumber("Sim Battery voltage", RobotController.getBatteryVoltage());
    }

    @Override
    public void periodic() {
        // Display current values on the SmartDashboard
        SmartDashboard.putNumber("shoulder/Output", m_motorLeader.get());
        SmartDashboard.putNumber("shoulder/Encoder", getAngle());
        SmartDashboard.putNumber("shoulder/Goal", m_goal);
        SmartDashboard.putBoolean("shoulder/CoastMode", m_coastMode);

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
            setPoint.position = m_encoder.getIntegratedSensorAbsolutePosition();
            m_resetShoulderPos = false;
        }

        m_motorLeader.set(ControlMode.Position, setPoint.position, DemandType.ArbitraryFeedForward, 0.0);//feedforward/12.0);
        SmartDashboard.putNumber("shoulder/feedforward", feedforward);
        SmartDashboard.putNumber("shoulder/setPoint", setPoint.position * SHOULDER_ANGLE_PER_UNIT);
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

    // return current shoulder angle in degrees
    public double getAngle() {
        return m_encoder.getIntegratedSensorAbsolutePosition() * SHOULDER_ANGLE_PER_UNIT;
    }

    public void resetShoulderPos() {
        setAngle(getAngle());
        m_resetShoulderPos = true;
    }

    // set shoulder angle in degrees
    public void setAngle(double angle) {
        m_goal = angle;
        super.setGoal(angle / SHOULDER_ANGLE_PER_UNIT);
    }
}
