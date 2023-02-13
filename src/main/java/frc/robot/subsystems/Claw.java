// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Claw extends SubsystemBase {
	/** Creates a new Claw. */

	// TODO: check what is the module number
	private Compressor m_phCompressor = new Compressor(Constants.PNEUMATIC_HUB_PORT, PneumaticsModuleType.REVPH);
	private DoubleSolenoid m_clawSolenoid = new DoubleSolenoid(Constants.DOUBLE_SOLENOID_MODULE_NUMBER, PneumaticsModuleType.REVPH, Constants.DOUBLE_SOLENOID_FORWARD_CHANNEL, Constants.DOUBLE_SOLENOID_REVERSE_CHANNEL);

	private CANSparkMax m_motor;
	private RelativeEncoder m_encoder;

	public Claw() {
		// TODO: which one??
		// m_phCompressor.enableAnalog(0, 0);
		m_phCompressor.enableDigital();
		// TODO: check the port
		m_motor = new CANSparkMax(0, MotorType.kBrushless);
		m_encoder = m_motor.getEncoder();

		// TODO: set this conversion factor for RPM
		m_encoder.setVelocityConversionFactor(0);
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		SmartDashboard.putBoolean("claw/not full?", m_phCompressor.getPressureSwitchValue());
		SmartDashboard.putBoolean("claw/isFwdSolenoidDisabled", m_clawSolenoid.isFwdSolenoidDisabled());
		SmartDashboard.putBoolean("claw/isRevSolenoidDisabled", m_clawSolenoid.isRevSolenoidDisabled());
	}

	public void toggleClaw(){
		m_clawSolenoid.toggle();
	}

	public void open(){
		m_clawSolenoid.set(Value.kForward);
		m_motor.getEncoder().getVelocity();

	}

	public void close(){
		m_clawSolenoid.set(Value.kReverse);
	}
}
