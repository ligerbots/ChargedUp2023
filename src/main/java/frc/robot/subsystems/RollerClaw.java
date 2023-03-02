// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ColorSensorV3.RawColor;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants;

public class RollerClaw extends Claw {
    // TODO: tune this current limit, borrowed from 2021 game
    private static final double MOTOR_CURRENT_LIMIT = 10.35;

    // speed to run the motor for intake
    private static final double INTAKE_MOTOR_SPEED = 0.2;
    
    // delay time for shutting off the motor
    private static final double STOP_MOTOR_DELAY = 0.25;   // seconds

    private double m_speed;

    private Timer m_timer;

    PneumaticHub m_pH = new PneumaticHub(Constants.PNEUMATIC_HUB_PORT);
    DoubleSolenoid m_clawSolenoid = m_pH.makeDoubleSolenoid(Constants.DOUBLE_SOLENOID_FORWARD_CHANNEL, Constants.DOUBLE_SOLENOID_REVERSE_CHANNEL);
    private CANSparkMax m_motor = new CANSparkMax(Constants.CLAW_MOTOR_CAN_ID, MotorType.kBrushless);;
    private ColorSensorV3 m_colorSensor = new ColorSensorV3(I2C.Port.kOnboard);

    /** Creates a new RollerClaw. */
    public RollerClaw() {
        // limit the current to 15A
        m_motor.setSmartCurrentLimit(15);
        m_timer = new Timer();
        SmartDashboard.putBoolean("claw/isCompressorEnabled", true);
        m_speed = 0.0;
    }

    // This method will be called once per scheduler run
    @Override
    public void periodic() {
        SmartDashboard.putNumber("claw/speed", m_speed);

        if (m_motor.getOutputCurrent() > MOTOR_CURRENT_LIMIT) {
            setMotor(0);
        }

        // Timer is turned on only in close() method
        if(m_timer.hasElapsed(STOP_MOTOR_DELAY)){
            m_timer.stop();
            setMotor(0.0);
        }

        SmartDashboard.putNumber("claw/motorCurrent", m_motor.getOutputCurrent());
        SmartDashboard.putNumber("claw/Color Sensor distance", getColorSensorProximity());
        SmartDashboard.putNumberArray("claw/colorRGB", getColor());

        SmartDashboard.putBoolean("claw/isFwdSolenoidDisabled", m_clawSolenoid.isFwdSolenoidDisabled());
        SmartDashboard.putBoolean("claw/isRevSolenoidDisabled", m_clawSolenoid.isRevSolenoidDisabled());
        if(SmartDashboard.getBoolean("claw/isCompressorEnabled", true))
            m_pH.enableCompressorDigital();
        else
            m_pH.disableCompressor();
    }

    @Override
    public void open(){
        m_clawSolenoid.set(Value.kForward);
        setMotor(0);
    }

    @Override
    public void close() {
        m_clawSolenoid.set(Value.kReverse);
        // setMotor(0);
        // TODO delay stopping the motor to let it completely grab the cone.
        // This can't work. This is a subsystem not a command
        // Need to use a Timer, and check in periodic()
        // if(m_speed != 0.0)
        // new WaitCommand(0.5).andThen(new InstantCommand(this::stopMotor)).schedule();

        if(Math.abs(m_speed) >= 0.001){
            // Timer is turned on only in close() method
            m_timer.reset();
            m_timer.start();
        }
        
    }

    @Override
    public void startIntake() {
        // don't call open, since it does extra stuff
        m_clawSolenoid.set(Value.kForward);
        setMotor(INTAKE_MOTOR_SPEED);
    }

    private void setMotor(double speed) {
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

    public boolean isBallInFront () {
        return m_colorSensor.getProximity() > 110;
    }

    public int getColorSensorProximity() {
        return m_colorSensor.getProximity();
    }

    public double[] getColor(){
        RawColor color = m_colorSensor.getRawColor();
        return new double[]{color.red, color.green, color.blue};
    }
}
