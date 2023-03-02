// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class RollerClaw extends Claw {
    // TODO: tune this current limit, borrowed from 2021 game
    private static final double MOTOR_CURRENT_LIMIT = 10.35;

    // speed to run the motor for intake
    private static final double INTAKE_MOTOR_SPEED = 0.2;
    
    PneumaticHub m_pH = new PneumaticHub(Constants.PNEUMATIC_HUB_PORT);
    DoubleSolenoid m_clawSolenoid = m_pH.makeDoubleSolenoid(Constants.DOUBLE_SOLENOID_FORWARD_CHANNEL, Constants.DOUBLE_SOLENOID_REVERSE_CHANNEL);
    private CANSparkMax m_motor = new CANSparkMax(Constants.CLAW_MOTOR_CAN_ID, MotorType.kBrushless);;
    private ColorSensorV3 colorSensor = new ColorSensorV3(I2C.Port.kOnboard);

    private double m_speed = 0;

    /** Creates a new RollerClaw. */
    public RollerClaw() {
        // limit the current to 15A
        m_motor.setSmartCurrentLimit(15);
        
        SmartDashboard.putBoolean("claw/isCompressorEnabled", true);
    }

    // This method will be called once per scheduler run
    @Override
    public void periodic() {
        m_motor.set(m_speed);
        SmartDashboard.putNumber("claw/speed", m_speed);

        if (m_motor.getOutputCurrent() > MOTOR_CURRENT_LIMIT){
            stopMotor();
        }

        SmartDashboard.putBoolean("claw/isFwdSolenoidDisabled", m_clawSolenoid.isFwdSolenoidDisabled());
        SmartDashboard.putBoolean("claw/isRevSolenoidDisabled", m_clawSolenoid.isRevSolenoidDisabled());
        if(SmartDashboard.getBoolean("claw/isCompressorEnabled", true))
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

    @Override
    public void open(){
        m_clawSolenoid.set(Value.kForward);
        stopMotor();
    }

    @Override
    public void close(){
        m_clawSolenoid.set(Value.kReverse);
        // if(m_speed != 0.0)
        //     new WaitCommand(0.5).andThen(new InstantCommand(this::stopMotor)).schedule();
    }

    @Override
    public void startIntake() {
        open();
        runMotor();
    }

    public void stopMotor(){
        m_motor.set(0);
    }

    public void runMotor(){
        m_speed = INTAKE_MOTOR_SPEED;
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
