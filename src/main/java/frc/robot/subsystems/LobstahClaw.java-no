// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;

public class LobstahClaw extends Claw {
    PneumaticHub m_pH = new PneumaticHub(Constants.PNEUMATIC_HUB_PORT);
    DoubleSolenoid m_clawSolenoid = m_pH.makeDoubleSolenoid(Constants.DOUBLE_SOLENOID_FORWARD_CHANNEL, Constants.DOUBLE_SOLENOID_REVERSE_CHANNEL);
    private CANSparkMax m_motor;
    private double m_speed;

    /** Creates a new Claw. */
    public LobstahClaw() {
        // TODO: set the CAN ID
        m_motor = new CANSparkMax(0, MotorType.kBrushless);
        SmartDashboard.putBoolean("claw/isCompressorEnabled", true);
        m_speed = 0.0;
    }

    // This method will be called once per scheduler run
    @Override
    public void periodic() {
        m_motor.set(m_speed);
        SmartDashboard.putNumber("claw/m_speed", m_speed);

        // TODO: tune this current limit, borrowed from 2021 game
        if(m_motor.getOutputCurrent() > 10.35){
            stopMotor();
        }

        SmartDashboard.putBoolean("claw/isFwdSolenoidDisabled", m_clawSolenoid.isFwdSolenoidDisabled());
        SmartDashboard.putBoolean("claw/isRevSolenoidDisabled", m_clawSolenoid.isRevSolenoidDisabled());
        
        if (SmartDashboard.getBoolean("claw/isCompressorEnabled", false))
            enableCompressor();
        else
            disableCompressor();
    }

    @Override
    public void open(){
        System.out.println("Setting Claw to OPEN");
        m_clawSolenoid.set(Value.kForward);
        stopMotor();
    }

    @Override
    public void close(){
        System.out.println("Setting Claw to CLOSED");
        m_clawSolenoid.set(Value.kReverse);
        if(m_speed != 0.0)
            new WaitCommand(0.5).andThen(new InstantCommand(this::stopMotor)).schedule();
    }

    @Override
    public void enableCompressor(){
        m_pH.enableCompressorDigital();
    }

    @Override
    public void disableCompressor(){
        m_pH.disableCompressor();
    }

    public void stopMotor(){
        m_speed = 0.0;
    }

    public void runMotor(){
        m_speed = 1.0;
    }

    @Override
    public void startIntake() {
        open();
        runMotor();
    }

}
