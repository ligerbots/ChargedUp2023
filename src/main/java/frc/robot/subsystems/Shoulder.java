// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
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

    // Following CAN IDs are for the Arm subsystem
    public static final int SHOULDER_CAN_ID[] = {14, 15}; // TODO: Set CANIDs

    // Feedforward constants for the shoulder
    public static final double SHOULDER_KS = 0.182; // TODO: This may need to be tuned
    // The following constants are computed from https://www.reca.lc/arm
    public static final double SHOULDER_KG = 2.07;
    public static final double SHOULDER_KV = 1.83;
    public static final double SHOULDER_KA = 0.08;

    // Constants to limit the shoulder rotation speed
    public static final double SHOULDER_MAX_VEL_RAD_PER_SEC = Math.toRadians(200.0);
    public static final double SHOULDER_MAX_ACC_RAD_PER_SEC_SQ = Math.toRadians(200.0);
    public static final double SHOULDER_OFFSET_RAD = Math.toRadians(110.0);

    // PID Constants for the shoulder PID controller
    // Since we're using Trapeziodal control, all values will be 0 except for P
    public static final double SHOULDER_K_P = 10.0;
    public static final double SHOULDER_K_I = 0.0;
    public static final double SHOULDER_K_D = 0.0;
    public static final double SHOULDER_K_FF = 0.0;
    public static final int kPIDLoopIdx = 0;
    public static final int kTimeoutMs = 0;

    // TODO: calculate this
    public static final double SHOULDER_ANGLE_PER_UNIT = 1.0;

    // Define the motor and encoders
    private final WPI_TalonFX[] m_motor = new WPI_TalonFX[2];
    private final TalonFXSensorCollection[] m_encoder = new TalonFXSensorCollection[2];
    // private final EncoderSim m_encoderSim;

    // The P gain for the PID controller that drives this shoulder.
    private static double m_kPShoulder = 1.0;

    private boolean m_resetShoulderPos;
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
    private final ArmFeedforward m_feedForward = new ArmFeedforward(SHOULDER_KS, SHOULDER_KG,
            SHOULDER_KV, SHOULDER_KA);

    private TalonFXSimCollection m_motorSim;

    final SingleJointedArmSim m_shoulderSim = new SingleJointedArmSim(m_shoulderGearbox, m_shoulderReduction,
            SingleJointedArmSim.estimateMOI(m_shoulderLength, m_shoulderMass), m_shoulderLength,
            Units.degreesToRadians(-75), Units.degreesToRadians(120), true);

    // Create a Mechanism2d display of an Arm with a fixed ArmTower and moving Arm.
    // public final EncoderSim m_encoderSim = new EncoderSim(m_encoder);
    final Mechanism2d m_mech2d = new Mechanism2d(60, 60);
    final MechanismRoot2d m_shoulderPivot = m_mech2d.getRoot("ArmPivot", 30, 30);
    final MechanismLigament2d m_shoulderTower = m_shoulderPivot.append(new MechanismLigament2d("ArmTower", 30, -90));
    final MechanismLigament2d m_shoulder = m_shoulderPivot.append(new MechanismLigament2d("Arm", 30,
            Units.radiansToDegrees(m_shoulderSim.getAngleRads()), 6, new Color8Bit(Color.kYellow)));

    /**
     * TalonFXPIDSetConfiguration Creates a new ShoulderArm .
     * 
     * @return
     */
    public Shoulder() {
        super(new TrapezoidProfile.Constraints(SHOULDER_MAX_VEL_RAD_PER_SEC, SHOULDER_MAX_ACC_RAD_PER_SEC_SQ));

        for (int i = 0; i <= 1; i++) {
            // Create the motor, PID Controller and encoder.
            m_motor[i] = new WPI_TalonFX(SHOULDER_CAN_ID[i]);
            m_encoder[i] = m_motor[i].getSensorCollection();

            // m_motorSim = new TalonFXSimCollection(m_motorLeader);
            // m_encoderSim = new TalonFXSimCollection(m_encoder);
            m_motor[i].configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, kPIDLoopIdx, kTimeoutMs);
            // Set follower and invert
            m_motor[i].config_kF(kPIDLoopIdx, SHOULDER_K_FF, kTimeoutMs);
            m_motor[i].config_kP(kPIDLoopIdx, SHOULDER_K_P, kTimeoutMs);
            m_motor[i].config_kI(kPIDLoopIdx, SHOULDER_K_I, kTimeoutMs);
            m_motor[i].config_kD(kPIDLoopIdx, SHOULDER_K_D, kTimeoutMs);

            // Set the position conversion factor. Note that the Trapezoidal control
            // expects angles in radians.
            // TODO: Set this based on shoulder gearbox gear ratio

        }

        SmartDashboard.putNumber("shoulder/P Gain", m_kPShoulder);
        SmartDashboard.putData("shoulder Sim", m_mech2d);
    }

    public void simulationPeriodic() {
        // First, we set our "inputs" (voltages)
        m_shoulderSim.setInput(m_motor[0].get() * RobotController.getBatteryVoltage());
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
        SmartDashboard.putNumber("shoulder/Output", m_motor[0].get());
        SmartDashboard.putNumber("shoulder/Encoder", Math.toDegrees(getAngle()));
        SmartDashboard.putNumber("shoulder/Goal", m_goal);

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
            setPoint.position = getAngle();
            m_resetShoulderPos = false;
        }

        for(int i = 0; i <= 1; i++)
            m_motor[i].set(ControlMode.Position, feedforward);
            
        SmartDashboard.putNumber("shoulder/feedforward", feedforward);
        SmartDashboard.putNumber("shoulder/setPoint", Units.metersToInches(setPoint.position));
        SmartDashboard.putNumber("shoulder/velocity", Units.metersToInches(setPoint.velocity));
    }

    private void checkPIDVal() {
        double p = SmartDashboard.getNumber("shoulder/P Gain", 0);
        // if PID coefficients on SmartDashboard have changed, write new values to
        // controller
        if ((p != m_kPShoulder)) {
            for(int i = 0; i <= 1; i++)
                m_motor[i].config_kP(kPIDLoopIdx, SHOULDER_K_P, kTimeoutMs);
            m_kPShoulder = p;
        }
    }

    public double getAngle() {
        return (m_encoder[0].getIntegratedSensorAbsolutePosition() + m_encoder[1].getIntegratedSensorAbsolutePosition())/2.0;
    }

    public void resetShoulderPos() {
        super.setGoal(getAngle());
        m_resetShoulderPos = true;
    }

    public void setGoal(double goal) {
        m_goal = goal;
        super.setGoal(goal);
    }

    public double getEncoderDistance() {
        return (m_motor[0].getSelectedSensorPosition() * SHOULDER_ANGLE_PER_UNIT + m_motor[1].getSelectedSensorPosition() * SHOULDER_ANGLE_PER_UNIT)/2.0;
    }
}
