// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Reacher extends SubsystemBase {

    public static final int REACHER_CAN_ID = 13; // TODO: Set CanID

    // Feedforward constants for the reacher
    public static final double REACHER_KS = 0.182; // TODO: This may need to be tuned
    // The following constants are computed from https://www.reca.lc/arm
    public static final double REACHER_KG = 1.19;
    public static final double REACHER_KV = 7.67;
    public static final double REACHER_KA = 0.19;

    // PID Constants for the reacher PID controller
    // Since we're using Trapeziodal control, all values will be 0 except for P
    public static final double REACHER_K_P0 = 100;
    public static final double REACHER_K_P1 = 100;
    public static final double REACHER_K_I = 0.0;
    public static final double REACHER_K_D = 0.0;
    public static final double REACHER_K_FF = 0.0;
    public static final double REACHER_OFFSET_METER = Units.inchesToMeters(1.5);


	/** Creates a new Reacher. */
	// Define the motor and encoders
	private final CANSparkMax m_motor;
	private final RelativeEncoder m_encoder;
	private final SparkMaxPIDController m_PIDController;

	private final ElevatorFeedforward m_Feedforward = new ElevatorFeedforward(REACHER_KS,
			REACHER_KG, REACHER_KV, REACHER_KA);

	private double m_kPReacher;
	private boolean m_resetReacherPos = false;

	/** Creates a new Reacher. */
	public Reacher() {
		m_kPReacher = REACHER_K_P0;

		// Create the motor, PID Controller and encoder.
		m_motor = new CANSparkMax(REACHER_CAN_ID, MotorType.kBrushless);
		m_motor.restoreFactoryDefaults();

		m_PIDController = m_motor.getPIDController();
		m_PIDController.setP(m_kPReacher);
		m_PIDController.setI(REACHER_K_I);
		m_PIDController.setD(REACHER_K_D);
		m_PIDController.setFF(REACHER_K_FF);

		m_encoder = m_motor.getEncoder();

		// Set the position conversion factor.
		m_encoder.setPositionConversionFactor((12.0 / 72.0) * Units.inchesToMeters((7.0 / 8.0) * Math.PI)); // was 5/8

		m_encoder.setPosition(REACHER_OFFSET_METER);

		SmartDashboard.putNumber("Reacher/P Gain", m_kPReacher);
	}

	@Override
	public void periodic() {
		double encoderValue = m_encoder.getPosition();
		SmartDashboard.putNumber("Reacher/Encoder", Units.metersToInches(encoderValue));

		// update the PID val
		checkPIDVal();
	}

	protected void setSetPoint(TrapezoidProfile.State setPoint) {
		// Calculate the feedforward from the setPoint
		double feedforward = m_Feedforward.calculate(setPoint.position, setPoint.velocity);

		// Add the feedforward to the PID output to get the motor output
		// Remember that the encoder was already set to account for the gear ratios.

		if (m_resetReacherPos) {
			setPoint.position = m_encoder.getPosition();
			m_resetReacherPos = false;
		}
		m_PIDController.setReference(setPoint.position, ControlType.kPosition, 0, feedforward / 12.0);
		SmartDashboard.putNumber("Reacher/setPoint", Units.metersToInches(setPoint.position));
	}

	private void checkPIDVal() {
		double p = SmartDashboard.getNumber("Reacher/P Gain", 0);
		// if PID coefficients on SmartDashboard have changed, write new values to controller
		if ((p != m_kPReacher)) {
			m_PIDController.setP(p);
			m_kPReacher = p;
		}
	}

	public double getLength() {
		return m_encoder.getPosition();
	}

	public void resetLength() {
		setSetPoint(new TrapezoidProfile.State(m_encoder.getPosition(), 0.0));
		m_resetReacherPos = true;
	}

	public void setBrakeMode(boolean brake) {
		m_motor.setIdleMode(brake ? CANSparkMax.IdleMode.kBrake : CANSparkMax.IdleMode.kCoast);
	}
}
