// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.commands.Rumble;

public class RollerClaw extends Claw {
    // TODO: tune this current limit, borrowed from 2021 game
    // private static final double MOTOR_CURRENT_LIMIT = 10.35;

    // speed to run the motor for intake
    private static final double INTAKE_MOTOR_SPEED = 0.75;

    private static final double INTAKE_STOP_SPEED = 0.05;
    
    // delay time for shutting off the motor
    private static final double STOP_MOTOR_DELAY = 2.0;   // seconds
    private double m_speed;

    private Timer m_timer;
    private boolean m_needStop = false;

    private boolean m_waitForPiece = false;

    PneumaticHub m_pH = new PneumaticHub(Constants.PNEUMATIC_HUB_PORT);
    DoubleSolenoid m_clawSolenoid = m_pH.makeDoubleSolenoid(Constants.DOUBLE_SOLENOID_FORWARD_CHANNEL, Constants.DOUBLE_SOLENOID_REVERSE_CHANNEL);
    private CANSparkMax m_motor = new CANSparkMax(Constants.CLAW_MOTOR_CAN_ID, MotorType.kBrushless);;

    /** Creates a new RollerClaw. */
    public RollerClaw(Rumble rumbleCommand) {
        m_rumbleCommand = rumbleCommand;
        m_motor.setInverted(true);
        // limit the current to 15A
        m_motor.setSmartCurrentLimit(15);
        m_motor.setIdleMode(IdleMode.kBrake);
        
        m_timer = new Timer();
        SmartDashboard.putBoolean("claw/isCompressorEnabled", true);
        // SmartDashboard.putNumber("claw/speed", 0.0);
        m_speed = 0.0;
    }

    // This method will be called once per scheduler run
    @Override
    public void periodic() {
        // if (m_motor.getOutputCurrent() > MOTOR_CURRENT_LIMIT) {
        //     setMotor(0);
        // }

        if(m_waitForPiece && detectedGamePiece()){
            // rumble when there is a game piece inside the claw
            // only rumble in teleop mode
            if(DriverStation.isTeleopEnabled())
                m_rumbleCommand.schedule();
            close();
            m_hasGamePiece = true;
        }

        // Timer is turned on only in close() method
        if (m_needStop && m_timer.hasElapsed(STOP_MOTOR_DELAY)) {
            m_timer.stop();
            setMotor(INTAKE_STOP_SPEED);
            m_needStop = false;
        }

        // SmartDashboard.putNumber("claw/motorCurrent", m_motor.getOutputCurrent());

        // SmartDashboard.putBoolean("claw/isFwdSolenoidDisabled", m_clawSolenoid.isFwdSolenoidDisabled());
        // SmartDashboard.putBoolean("claw/isRevSolenoidDisabled", m_clawSolenoid.isRevSolenoidDisabled());
        if (SmartDashboard.getBoolean("claw/isCompressorEnabled", true))
            enableCompressor();
        else
            disableCompressor();

        SmartDashboard.putNumber("claw/curIRSensorReading", super.m_curIRSensorReading);
    }

    @Override
    public void open(){
        // System.out.println("RollerClaw open called");
        m_clawSolenoid.set(Value.kForward);
        setMotor(0.0);
        m_hasGamePiece = false;
    }

    @Override
    public void close() {
        m_waitForPiece = false;
        m_clawSolenoid.set(Value.kReverse);

        if (Math.abs(m_speed) >= (INTAKE_STOP_SPEED + 0.05)) {
            // Timer is turned on only in close() method
            m_timer.reset();
            m_timer.start();
            m_needStop = true;
        }
    }

    @Override
    public void startIntake() {
        // don't call open, since it does extra stuff
        m_waitForPiece = true;
        m_hasGamePiece = false;

        m_clawSolenoid.set(Value.kForward);
        setMotor(INTAKE_MOTOR_SPEED);
    }

    private void setMotor(double speed) {
        // System.out.println("Setting claw motor to " + speed);
        m_motor.set(speed);
        m_speed = speed;
    }

    @Override
    public void enableCompressor() {
        m_pH.enableCompressorDigital();
    }

    @Override
    public void disableCompressor() {
        m_pH.disableCompressor();
    }
}