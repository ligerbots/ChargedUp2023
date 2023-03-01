// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class RollerClaw extends SubsystemBase {
    PneumaticHub m_pH = new PneumaticHub(Constants.PNEUMATIC_HUB_PORT);
    DoubleSolenoid m_clawSolenoid = m_pH.makeDoubleSolenoid(Constants.DOUBLE_SOLENOID_FORWARD_CHANNEL, Constants.DOUBLE_SOLENOID_REVERSE_CHANNEL);
    private CANSparkMax m_motor;
    private RelativeEncoder m_encoder;
    private double m_speed = 0;

    /** Creates a new Claw. */
    public RollerClaw() {
        SmartDashboard.putBoolean("claw/isCompressorEnabled", m_pH.getCompressor());
    }

    // This method will be called once per scheduler run
    @Override
    public void periodic() {

        // TODO: this does not make sense. Testing the compressor does not test if something is in the claw
        // SmartDashboard.putBoolean("claw/not full?", m_phCompressor.getPressureSwitchValue());
        SmartDashboard.putBoolean("claw/isFwdSolenoidDisabled", m_clawSolenoid.isFwdSolenoidDisabled());
        SmartDashboard.putBoolean("claw/isRevSolenoidDisabled", m_clawSolenoid.isRevSolenoidDisabled());
        if(SmartDashboard.getBoolean("claw/isCompressorEnabled", false))
            m_pH.enableCompressorDigital();
        else
            m_pH.disableCompressor();
    }

    public void toggleClaw(){
        m_clawSolenoid.toggle();
    }

    public void turnMotorOn(){
        m_motor.set(.75);
    }

    public void open(){
        System.out.println("Setting Claw to OPEN");
        m_clawSolenoid.set(Value.kForward);
    }

    public void close(){
        System.out.println("Setting Claw to CLOSED");
        m_clawSolenoid.set(Value.kReverse);
    }

    public void enableCompressor(){
        m_pH.enableCompressorDigital();
    }
    public void turnMotorOff(){
        m_motor.set(0);
    }
}
