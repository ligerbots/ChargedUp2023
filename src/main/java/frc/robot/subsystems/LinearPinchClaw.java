// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// import com.revrobotics.CANSparkMax;
// import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants;

public class LinearPinchClaw extends Claw {
    PneumaticHub m_pH = new PneumaticHub(Constants.PNEUMATIC_HUB_PORT);
    DoubleSolenoid m_clawSolenoid = m_pH.makeDoubleSolenoid(Constants.DOUBLE_SOLENOID_FORWARD_CHANNEL, Constants.DOUBLE_SOLENOID_REVERSE_CHANNEL);

    /** Creates a new Claw. */
    public LinearPinchClaw() {
        SmartDashboard.putBoolean("claw/isCompressorEnabled", true);
    }

    // This method will be called once per scheduler run
    @Override
    public void periodic() {

        // SmartDashboard.putBoolean("claw/not full?", m_phCompressor.getPressureSwitchValue());
        // SmartDashboard.putBoolean("claw/isFwdSolenoidDisabled", m_clawSolenoid.isFwdSolenoidDisabled());
        // SmartDashboard.putBoolean("claw/isRevSolenoidDisabled", m_clawSolenoid.isRevSolenoidDisabled());
        
        if (SmartDashboard.getBoolean("claw/isCompressorEnabled", false))
            enableCompressor();
        else
            disableCompressor();
    }

    @Override
    public void open(){
        m_clawSolenoid.set(Value.kForward);
    }

    @Override
    public void close(){
        m_clawSolenoid.set(Value.kReverse);
    }

    @Override
    public void startIntake() {
        open();        
    }

    @Override
    public void enableCompressor(){
        m_pH.enableCompressorDigital();
    }

    @Override
    public void disableCompressor(){
        m_pH.disableCompressor();
    }
}
