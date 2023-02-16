// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
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
import frc.robot.Constants;

public class Shoulder extends TrapezoidProfileSubsystem {

    // Define the motor and encoders
    private final WPI_TalonFX m_motor;
    private final TalonFXSensorCollection m_encoder;
    // private final EncoderSim m_encoderSim;

    // The P gain for the PID controller that drives this shoulder.
    private static double m_kPShoulder = 1.0;

    // *********************Simulation Stuff************************

    // distance per pulse = (angle per revolution) / (pulses per revolution)
    // = (2 * PI rads) / (4096 pulses)
    private static final double kShoulderEncoderDistPerPulse = 2.0 * Math.PI / 4096;
    // The arm gearbox represents a gearbox containing two Vex 775pro motors.
    private final DCMotor m_shoulderGearbox = DCMotor.getFalcon500(2);

    // Standard classes for controlling our arm
    private final PIDController m_controller = new PIDController(m_kPShoulder, 0, 0);

    // Simulation classes help us simulate what's going on, including gravity.
    private static final double m_shoulderReduction = 395.77;
    private static final double m_shoulderMass = 10; // Kilograms
    private static final double m_shoulderLength = Units.inchesToMeters(30);
    private final ArmFeedforward m_feedForward = new ArmFeedforward(Constants.SHOULDER_KS, Constants.SHOULDER_KG,
            Constants.SHOULDER_KV, Constants.SHOULDER_KA);

    private TalonFXSimCollection m_motorSim;

    final SingleJointedArmSim m_shoulderSim = new SingleJointedArmSim(m_shoulderGearbox, m_shoulderReduction,
            SingleJointedArmSim.estimateMOI(m_shoulderLength, m_shoulderMass), m_shoulderLength,
            Units.degreesToRadians(-75), Units.degreesToRadians(120), m_shoulderMass, true,
            VecBuilder.fill(kShoulderEncoderDistPerPulse) // Add noise with a
    // std-dev of 1 tick
    );
    // Create a Mechanism2d display of an Arm with a fixed ArmTower and moving Arm.
    // public final EncoderSim m_encoderSim = new EncoderSim(m_encoder);
    final Mechanism2d m_mech2d = new Mechanism2d(60, 60);
    final MechanismRoot2d m_shoulderPivot = m_mech2d.getRoot("ArmPivot", 30, 30);
    final MechanismLigament2d m_shoulderTower = m_shoulderPivot.append(new MechanismLigament2d("ArmTower", 30, -90));
    final MechanismLigament2d m_shoulder = m_shoulderPivot.append(new MechanismLigament2d("Arm", 30,
            Units.radiansToDegrees(m_shoulderSim.getAngleRads()), 6, new Color8Bit(Color.kYellow)));

    private boolean m_coastMode = false;
    private boolean m_resetShoulderPos = false;
    private double m_goal;

    /**
     * TalonFXPIDSetConfiguration Creates a new ShoulderArm .
     * 
     * @return
     */
    public Shoulder() {
        super(new TrapezoidProfile.Constraints(Constants.SHOULDER_MAX_VEL_RAD_PER_SEC,
                Constants.SHOULDER_MAX_ACC_RAD_PER_SEC_SQ));

        TrapezoidProfile.State previousProfiledReference = new TrapezoidProfile.State(Constants.SHOULDER_OFFSET_RAD,
                0.0);

        // Create the motor, PID Controller and encoder.
        m_motor = new WPI_TalonFX(Constants.SHOULDER_CAN_ID);
        m_encoder = m_motor.getSensorCollection();

        // m_motorSim = new TalonFXSimCollection(m_motorLeader);
        // m_encoderSim = new TalonFXSimCollection(m_encoder);
        m_motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.kPIDLoopIdx,
                Constants.kTimeoutMs);
        // Set follower and invert
        m_motor.config_kF(Constants.kPIDLoopIdx, Constants.SHOULDER_K_FF, Constants.kTimeoutMs);
        m_motor.config_kP(Constants.kPIDLoopIdx, Constants.SHOULDER_K_P, Constants.kTimeoutMs);
        m_motor.config_kI(Constants.kPIDLoopIdx, Constants.SHOULDER_K_I, Constants.kTimeoutMs);
        m_motor.config_kD(Constants.kPIDLoopIdx, Constants.SHOULDER_K_D, Constants.kTimeoutMs);

        // Set the position conversion factor. Note that the Trapezoidal control
        // expects angles in radians.
        // TODO: Set this based on shoulder gearbox gear ratio

        SmartDashboard.putNumber("shoulder/P Gain", m_kPShoulder);
        SmartDashboard.putData("shoulder Sim", m_mech2d);
    }

    public void simulationPeriodic() {
        // First, we set our "inputs" (voltages)
        m_shoulderSim.setInput(m_motor.get() * RobotController.getBatteryVoltage());
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
        double encoderValue = m_encoder.getIntegratedSensorAbsolutePosition();

        // Display current values on the SmartDashboard
        SmartDashboard.putNumber("shoulder/Output", m_motor.get());
        SmartDashboard.putNumber("shoulder/Encoder", Math.toDegrees(encoderValue));
        SmartDashboard.putBoolean("shoulder/CoastMode", m_coastMode);
        SmartDashboard.putNumber("shoulder/Goal", m_goal);

        // if in coast mode, stop the periodic() here to prevent the PID from
        // setReference()
        if (m_coastMode)
            return;

        // // First check if we've gone too far. If we have, reset the setPoint to the
        // limit.
        // m_tooFarForward = encoderValue > Constants.ARM_MAX_ANGLE;
        // SmartDashboard.putBoolean("arm" + m_index + "/too Forward", m_tooFarForward);
        // if (m_tooFarForward) {
        // // TODO: convert MAX to encoder position
        // m_PIDController.setReference(Constants.ARM_MAX_ANGLE, ControlType.kPosition,
        // 0, 0.0);
        // return; // Do we really not want to run super.periodic()?
        // }

        // m_tooFarBack = encoderValue < Constants.ARM_MIN_ANGLE;
        // SmartDashboard.putBoolean("arm" + m_index + "/too Backward", m_tooFarBack);
        // if (m_tooFarBack) {
        // // TODO: convert MIN to encoder position
        // m_PIDController.setReference(Constants.ARM_MIN_ANGLE, ControlType.kPosition,
        // 0, 0.0);
        // return;
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
        double feedforward = m_feedForward.calculate(setPoint.position, setPoint.velocity);

        // Add the feedforward to the PID output to get the motor output
        // The ArmFeedForward computes in radians. We need to convert back to degrees.
        // Remember that the encoder was already set to account for the gear ratios.

        // TODO: if the "12.0" is volts, should use RobotController.getBatteryVoltage()
        if (m_resetShoulderPos) {
            setPoint.position = m_encoder.getIntegratedSensorAbsolutePosition();
            m_resetShoulderPos = false;
        }

        m_motor.set(ControlMode.Position, feedforward);
        SmartDashboard.putNumber("shoulder/feedforward", feedforward);
        SmartDashboard.putNumber("shoulder/setPoint", Units.metersToInches(setPoint.position));
        SmartDashboard.putNumber("shoulder/velocity", Units.metersToInches(setPoint.velocity));
    }

    private void checkPIDVal() {
        double p = SmartDashboard.getNumber("shoulder/P Gain", 0);
        // if PID coefficients on SmartDashboard have changed, write new values to
        // controller
        if ((p != m_kPShoulder)) {
            m_motor.config_kP(Constants.kPIDLoopIdx, Constants.SHOULDER_K_P, Constants.kTimeoutMs);
            m_kPShoulder = p;
        }
    }

    public double getAngle() {
        return m_encoder.getIntegratedSensorPosition();
    }

    public void resetShoulderPos() {
        super.setGoal(m_encoder.getIntegratedSensorAbsolutePosition());
        m_resetShoulderPos = true;
    }

    public void setGoal(double goal) {
        m_goal = goal;
        super.setGoal(goal);
    }

}
