// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Claw extends SubsystemBase {
	/** Creates a new Claw. */

	private Compressor m_phCompressor = new Compressor(Constants.PNEUMATIC_HUB_PORT, PneumaticsModuleType.REVPH);
	private DoubleSolenoid m_doubleSolenoid = new DoubleSolenoid(Constants.DOUBLE_SOLENOID_MODULE_NUMBER, PneumaticsModuleType.REVPH, Constants.DOUBLE_SOLENOID_FORWARD_CHANNEL, Constants.DOUBLE_SOLENOID_REVERSE_CHANNEL);

	public Claw() {
		m_doubleSolenoid.set(Value.kReverse);
		// TODO: which one??
		m_phCompressor.enableAnalog(0, 0);
		m_phCompressor.enableDigital();
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}

	public void toggleDoubleSolenoid(){
		m_doubleSolenoid.toggle();
	}
}
